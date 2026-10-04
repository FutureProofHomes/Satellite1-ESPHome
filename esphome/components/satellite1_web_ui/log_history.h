#pragma once

#include <cstddef>
#include <cstdint>

#include "esphome/core/defines.h"

#ifdef USE_SAT1_LOG_HISTORY

namespace esphome {
namespace satellite1_web_ui {

/// The most one log line becomes in a ring: the logger's 512-byte line, the uptime prefix and the
/// record's newline. Also the size of the writer's stack copy, on the main loop task.
static constexpr size_t LH_RECORD_MAX = 528;

/// The largest copy a reader takes under the lock. Bounds how long the httpd task can hold the
/// writer off - about ten microseconds - and must hold a whole record, since reads end on one.
static constexpr size_t LH_READ_WINDOW = 1024;

/// One ring of whole records in PSRAM. A record is "<millis> <line>\n", with the line's own
/// newlines stored as \x1f, so '\n' only ever ends a record and a reader never sees half of one.
/// Positions are absolute byte counts since boot: a reader can tell how far the writer has run
/// past it, and a browser can ask for everything after a position it was given.
struct LogRing {
  char *buf{nullptr};
  size_t size{0};
  uint64_t head{0};  ///< Absolute position of the next record.
  uint64_t tail{0};  ///< Absolute position of the oldest whole record.
  size_t head_at{0};  ///< head % size, kept so the writer never divides 64 bits per byte.
  size_t tail_at{0};
};

/**
 * The device's recent log, kept so the app can show what happened before anyone opened it.
 *
 * Two rings, both PSRAM. The main ring takes every line at or above the capture level, DEBUG by
 * default. The alert ring takes warnings and errors a second time: a chatty DEBUG session turns
 * the main ring over in minutes, and the warning someone comes looking for an hour later is still
 * in the alert ring when the main one has long forgotten it. Neither survives a reboot; the crash
 * report's RTC ring covers the seconds before a crash.
 *
 * The writer is the logger listener, which ESPHome only ever calls on the main loop task - lines
 * from other tasks are queued and replayed there. The reader is the httpd task, through read().
 * One spinlock covers both, held for one record's copy or one read window.
 */
class LogHistory {
 public:
  enum class Ring : uint8_t { MAIN, ALERT };

  /// Where both rings stand, taken together under the lock.
  struct Bounds {
    uint64_t main_tail;
    uint64_t main_head;
    uint64_t alert_tail;
    uint64_t alert_head;
    /// The millis stamp of the main ring's oldest record. Alert records from before it are the
    /// ones the main ring has dropped. Meaningless while main_tail == main_head.
    uint32_t main_oldest_ms;
  };

  /// Allocates both rings in PSRAM and subscribes to the logger. False, with nothing allocated or
  /// subscribed, when PSRAM cannot hold them; the endpoint then answers 404 and the app shows the
  /// live stream alone.
  bool begin(size_t size, size_t alert_size, uint8_t level);
  bool active() const { return this->main_.buf != nullptr; }

  Bounds bounds() const;

  /// Copies whole records from absolute position `pos` of `ring` into `out`, at most `cap` bytes
  /// (at least LH_RECORD_MAX) and never past `end`, and advances `pos` past them. A `pos` the
  /// writer has since overwritten skips forward to the oldest record that survives. Returns the
  /// bytes copied, 0 once `pos` reaches `end`.
  size_t read(Ring ring, uint64_t &pos, uint64_t end, char *out, size_t cap) const;

  /// Random per boot, so a browser holding positions from before a reboot is told to start over.
  uint32_t boot_id() const { return this->boot_id_; }
  size_t size() const { return this->main_.size; }
  size_t alert_size() const { return this->alert_.size; }
  uint8_t level() const { return this->level_; }

 protected:
  static void log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len);
  static void append_(LogRing &ring, const char *rec, size_t len);

  LogRing main_;
  LogRing alert_;
  uint8_t level_{5};  // ESPHOME_LOG_LEVEL_DEBUG
  uint32_t boot_id_{0};
};

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_LOG_HISTORY
