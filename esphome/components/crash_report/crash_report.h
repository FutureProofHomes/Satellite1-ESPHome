#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

namespace esphome {
namespace crash_report {

/// How much RTC slow memory the log ring spends. The S3 has 8KB of it and this firmware uses none
/// elsewhere (no deep sleep), so 4KB of ring plus the ~30-byte time mark leaves half the segment
/// free. It costs nothing from the internal heap, which is the number this product defends.
static constexpr size_t CR_LOG_RING = 4096;

/// The most one log line contributes to the ring, prefix included. Also the bound on the
/// critical section the writer holds - a memcpy this size is a few microseconds.
static constexpr size_t CR_LINE_MAX = 224;

/// Crashes remembered across power loss. Eight matches the app's other rings, and a device that
/// has crashed nine distinct times needs a fixed firmware, not a longer list.
static constexpr uint8_t CR_RECORDS = 8;

/// How often loop() refreshes the RTC time mark. The mark is what stamps a crash, so this is the
/// stamp's resolution; the write is ~30 bytes into RTC memory, so the cost of tightening it would
/// not be the writes but the pointlessness - "when" at crash-hunting granularity is minutes.
static constexpr uint32_t CR_MARK_PERIOD_MS = 10000;

/// Record flags.
static constexpr uint8_t CR_F_SUMMARY = 1 << 0;     ///< The core dump summary below is filled in.
static constexpr uint8_t CR_F_BT_CORRUPT = 1 << 1;  ///< IDF marked the backtrace corrupted.
static constexpr uint8_t CR_F_DUMP_OK = 1 << 2;     ///< A full dump image passed its CRC at harvest.
static constexpr uint8_t CR_F_EPOCH = 1 << 3;       ///< `epoch` is real; 0 in the field means unknown anyway.

/// One remembered crash, fixed-size and naturally aligned so the whole ring round-trips NVS as one
/// blob. 172 bytes; eight of them plus the header is a ~1.4KB preference, written once per crash
/// and once per boot (the boot counter).
struct CrashRecord {
  uint32_t epoch;      ///< Best-effort wall time of the crash (unix seconds), 0 unknown.
  uint32_t uptime_s;   ///< Uptime at the last RTC mark before the crash - "how long it ran".
  uint32_t boot;       ///< Session number that crashed, against boot_count() for "N restarts ago".
  uint32_t exc_cause;  ///< Xtensa EXCCAUSE, when CR_F_SUMMARY.
  uint32_t exc_vaddr;  ///< Faulting address, when CR_F_SUMMARY.
  uint32_t pc;         ///< Program counter at the exception, when CR_F_SUMMARY.
  uint32_t bt[16];     ///< Backtrace PCs, `bt_depth` of them, when CR_F_SUMMARY.
  char task[16];       ///< Crashing task's name, when CR_F_SUMMARY. NUL-terminated.
  char text[64];       ///< IDF's panic reason ("LoadProhibited", "assert failed: ..."), or "".
  uint8_t bt_depth;
  uint8_t reason;  ///< esp_reset_reason_t value.
  uint8_t flags;   ///< CR_F_*.
  uint8_t reserved;
};

/// The NVS blob: a boot counter plus the ring, newest record last.
struct CrashStore {
  uint32_t boot_count;
  uint8_t count;
  uint8_t reserved[3];
  CrashRecord records[CR_RECORDS];
};

/**
 * The crash flight recorder and its harvest.
 *
 * While running: a log-listener callback copies every line at or above the capture floor into a
 * ring in RTC slow memory (any task; a spinlock bounds the copy), and loop() refreshes a time mark
 * beside it - last-known epoch and uptime - every CR_MARK_PERIOD_MS.
 *
 * At the boot after a crash: setup() reads the reset reason, validates what survived in RTC,
 * pulls the core dump summary from flash (tolerating an absent or corrupt image - an interrupt
 * watchdog that fired with the flash cache disabled can fail the dump write), appends one compact
 * record to the NVS ring, and copies the log tail into PSRAM for serving. The full dump image
 * stays in flash until a browser erases it.
 *
 * Thread contract for the web handler (httpd task): everything below record_count() is safe there.
 * The tail is immutable after setup; records are guarded by a mutex against erase().
 */
class CrashReport : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  /// DATA (600): after global preferences exist, before anything that could log the lines this
  /// exists to capture - and long before the web listener at 249 can serve a request.
  float get_setup_priority() const override { return setup_priority::DATA; }

#ifdef USE_TIME
  void set_time(time::RealTimeClock *clock) { this->time_ = clock; }
#endif
  void set_capture_level(uint8_t level) { this->capture_level_ = level; }

  /* ---- Read surface for satellite1_web_ui, httpd task safe ---- */

  /// Current session number. Records carry the session that crashed, so the UI's "N restarts ago"
  /// is boot_count() minus record.boot.
  uint32_t boot_count() const { return this->store_ == nullptr ? 0 : this->store_->boot_count; }

  uint8_t record_count();
  /// Copies record `at` out under the lock, index 0 oldest. False past the end.
  bool get_record(uint8_t at, CrashRecord &out);

  /// The pre-crash log tail harvested at boot, in PSRAM, NUL-free plain text. Immutable for the
  /// life of the session, so the caller may stream it without holding anything. Empty when the
  /// last reboot was clean or RTC memory did not survive (true power loss).
  const char *log_tail(size_t &len) const {
    len = this->tail_len_;
    return this->tail_;
  }

  /// Whether the coredump partition exists in this build's flashed table - false on a device that
  /// took this firmware over OTA, which cannot add a partition. The app words the difference.
  bool partition_present() const { return this->partition_present_; }

  /// Size in bytes of the dump image that passed its integrity check at harvest, 0 when none.
  size_t dump_size() const { return this->dump_size_; }

  /// Erases the flash dump image and the NVS record ring. Called from the httpd task - both the
  /// partition erase and the NVS write are thread-safe IDF calls, and the record lock covers the
  /// in-RAM state.
  void erase_all();

 protected:
  /// The logger hook, any task. Filters by level, strips ANSI colour, prefixes uptime, and copies
  /// one bounded line into the RTC ring under a spinlock.
  static void log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len);

  /// Reads what the previous session left in RTC + flash and appends a record if it crashed.
  void harvest_(uint8_t reason);
  /// Rewrites the RTC time mark from the current clock; called from loop() on the mark period.
  void mark_();

#ifdef USE_TIME
  time::RealTimeClock *time_{nullptr};
#endif
  uint8_t capture_level_{3};  // ESPHOME_LOG_LEVEL_INFO

  /// The store lives in PSRAM: ~1.4KB that a request only ever reads a copy of, with no business
  /// on the internal heap. Loaded once at setup, written on crash-harvest and the boot counter.
  CrashStore *store_{nullptr};
  ESPPreferenceObject pref_;
  Mutex store_lock_;

  /// The harvested log tail, PSRAM, sized to what actually survived rather than the ring's max.
  char *tail_{nullptr};
  size_t tail_len_{0};

  bool partition_present_{false};
  size_t dump_size_{0};

  /// Set by erase_all() on the httpd task, consumed by loop(): the NVS write itself must run on
  /// the main loop, because ESP32Preferences batches saves in a list the main loop owns.
  std::atomic<bool> save_pending_{false};

  uint32_t last_mark_ms_{0};
};

}  // namespace crash_report
}  // namespace esphome
