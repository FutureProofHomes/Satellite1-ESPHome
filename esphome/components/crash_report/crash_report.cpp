#include "crash_report.h"

#include <cstdio>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esphome/components/logger/logger.h"

#include <sdkconfig.h>

#include <esp_attr.h>
#include <esp_partition.h>
#include <esp_rom_crc.h>
#include <esp_system.h>

#include <freertos/FreeRTOS.h>

// Always present (the image_get/image_erase half is compiled unconditionally); the summary half
// of the API only exists under the two CONFIG guards used below.
#include <esp_core_dump.h>

namespace esphome {
namespace crash_report {

static const char *const TAG = "crash_report";

/* ---- What survives the reboot -------------------------------------------------------------- */

/// Distinct magics so a mark that survived beside a ring that did not (or the reverse) is usable
/// on its own. Bump either to invalidate old layouts across a firmware change to these structs.
static constexpr uint32_t CR_MARK_MAGIC = 0x43526D31;  // "CRm1"
static constexpr uint32_t CR_RING_MAGIC = 0x43527231;  // "CRr1"

/// The time mark: enough to reconstruct "when" after the crash. epoch_sync/uptime_sync pin the
/// last moment wall time was known; uptime_s is the newest mark. The crash's epoch is then
/// epoch_sync + (uptime_s - uptime_sync), which stays honest however long ago the last Home
/// Assistant sync was, because both sides of the delta come from the same millis clock.
struct RtcMark {
  uint32_t magic;
  uint32_t epoch_sync;   ///< Unix seconds at the last valid clock read, 0 never.
  uint32_t uptime_sync;  ///< Uptime seconds at that read.
  uint32_t uptime_s;     ///< Uptime seconds at the newest mark - "how long it ran".
  uint32_t crc;          ///< Over the four fields above; garbage after true power loss.
};

/// The log ring. No CRC, deliberately: it is appended to on every log line, and the failure mode
/// it would catch - a line torn by the crash itself - is exactly the last thing written before
/// the evidence, worth serving as-is rather than discarding the whole ring for.
struct RtcRing {
  uint32_t magic;
  uint16_t head;  ///< Next write position.
  uint16_t used;  ///< Bytes valid; == CR_LOG_RING once wrapped.
  char data[CR_LOG_RING];
};

/// RTC slow memory: cleared by nothing short of losing power, which is precisely the property the
/// flight recorder needs - a panic reboot preserves it. Costs zero internal DRAM.
RTC_NOINIT_ATTR static RtcMark s_mark;
RTC_NOINIT_ATTR static RtcRing s_ring;

/// Guards the ring. A spinlock rather than a mutex because the writer is the logger callback,
/// which runs on whatever task logged - and a bounded ~200-byte copy is a few microseconds,
/// cheaper than parking a task.
static portMUX_TYPE s_ring_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t mark_crc_(const RtcMark &m) {
  return esp_rom_crc32_le(0, reinterpret_cast<const uint8_t *>(&m), offsetof(RtcMark, crc));
}

/// Appends `len` bytes, wrapping. Only ever called with len <= CR_LINE_MAX < CR_LOG_RING.
static void ring_append_(const char *bytes, size_t len) {
  portENTER_CRITICAL(&s_ring_mux);
  size_t head = s_ring.head;
  for (size_t i = 0; i < len; i++) {
    s_ring.data[head] = bytes[i];
    head = (head + 1) % CR_LOG_RING;
  }
  s_ring.head = static_cast<uint16_t>(head);
  size_t used = s_ring.used + len;
  s_ring.used = static_cast<uint16_t>(used > CR_LOG_RING ? CR_LOG_RING : used);
  portEXIT_CRITICAL(&s_ring_mux);
}

/* ---- The always-on half: capture ------------------------------------------------------------ */

void CrashReport::log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len) {
  auto *me = static_cast<CrashReport *>(self);
  // Levels count up as severity counts down (ERROR=1 ... VERY_VERBOSE=7), so the floor is a <=.
  if (level == 0 || level > me->capture_level_)
    return;

  // The line is assembled on this stack first and committed in one append, so two tasks logging at
  // once interleave as whole lines rather than as shredded characters.
  //
  // `message` is the logger's whole formatted buffer - ANSI colour runs, then "[W][tag:line]:",
  // then the text (the same bytes the console gets; hardware-found by the wake word tuner, whose
  // parser assumed offset zero and read colour codes). The colour runs are stripped here rather
  // than stored: they are ~12 bytes a line that mean nothing in the app's <pre>.
  char line[CR_LINE_MAX];
  const uint32_t now_ms = millis();
  int at = snprintf(line, sizeof(line), "[%7lu.%03lu] ", static_cast<unsigned long>(now_ms / 1000),
                    static_cast<unsigned long>(now_ms % 1000));
  if (at < 0)
    return;

  for (size_t i = 0; i < len && at < static_cast<int>(sizeof(line)) - 1; i++) {
    const char c = message[i];
    if (c == '\x1b') {
      // Skip the escape sequence through its final byte ('m' for the SGR colour codes the logger
      // emits). A truncated sequence at the end of the buffer just ends the line.
      while (i < len && message[i] != 'm')
        i++;
      continue;
    }
    if (c == '\r')
      continue;
    // A multi-line message (dump_config tables) stays one ring entry; the ring's own separator is
    // the newline appended below.
    line[at++] = (c == '\n') ? ' ' : c;
  }
  line[at++] = '\n';

  ring_append_(line, static_cast<size_t>(at));
  (void) tag;
}

void CrashReport::mark_() {
  RtcMark next = s_mark;
#ifdef USE_TIME
  if (this->time_ != nullptr) {
    const auto now = this->time_->now();
    if (now.is_valid()) {
      next.epoch_sync = static_cast<uint32_t>(now.timestamp);
      next.uptime_sync = static_cast<uint32_t>(millis_64() / 1000);
    }
  }
#endif
  next.uptime_s = static_cast<uint32_t>(millis_64() / 1000);
  next.magic = CR_MARK_MAGIC;
  next.crc = mark_crc_(next);
  // Struct assignment rather than field writes, so a crash mid-mark leaves at worst a stale CRC
  // that fails closed (times omitted) instead of a plausible wrong time.
  s_mark = next;
}

/* ---- The boot-after half: harvest ----------------------------------------------------------- */

void CrashReport::harvest_(uint8_t reason) {
  CrashRecord rec{};
  rec.reason = reason;
  // The session being recorded is the one that just died - the counter is bumped after this.
  rec.boot = this->store_->boot_count;

  // "When", from whatever the mark preserved. A CRC failure here is the power-loss case: RTC
  // memory came up as noise, and no time is more honest than a confident wrong one.
  if (s_mark.magic == CR_MARK_MAGIC && mark_crc_(s_mark) == s_mark.crc) {
    rec.uptime_s = s_mark.uptime_s;
    if (s_mark.epoch_sync != 0) {
      rec.epoch = s_mark.epoch_sync + (s_mark.uptime_s - s_mark.uptime_sync);
      rec.flags |= CR_F_EPOCH;
    }
  }

  // "Where", from the dump the panic handler wrote on its way down. Guarded twice: the reset
  // reasons that run the panic handler (a brownout or an unknown reset writes no dump, and
  // attributing a stale image to it would invent a backtrace), and the image's own checksum
  // (an interrupt watchdog that fired with the flash cache disabled can fail the write partway).
#if defined(CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH) && defined(CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF)
  const bool dump_writing_reason = reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
                                   reason == ESP_RST_WDT || reason == ESP_RST_CPU_LOCKUP;
  if (dump_writing_reason && this->partition_present_ && esp_core_dump_image_check() == ESP_OK) {
    rec.flags |= CR_F_DUMP_OK;

    // ~250 bytes; setup runs on the main task, whose stack is comfortably past this.
    esp_core_dump_summary_t summary{};
    if (esp_core_dump_get_summary(&summary) == ESP_OK) {
      rec.flags |= CR_F_SUMMARY;
      rec.pc = summary.exc_pc;
      rec.exc_cause = summary.ex_info.exc_cause;
      rec.exc_vaddr = summary.ex_info.exc_vaddr;
      memcpy(rec.task, summary.exc_task, sizeof(rec.task) - 1);
      rec.task[sizeof(rec.task) - 1] = '\0';
      rec.bt_depth = static_cast<uint8_t>(summary.exc_bt_info.depth > 16 ? 16 : summary.exc_bt_info.depth);
      memcpy(rec.bt, summary.exc_bt_info.bt, rec.bt_depth * sizeof(uint32_t));
      if (summary.exc_bt_info.corrupted)
        rec.flags |= CR_F_BT_CORRUPT;
    }
    // Human-readable and worth its 64 bytes: "LoadProhibited", "assert failed: ...", "Interrupt
    // wdt timeout on CPU1" - the string a support thread actually needs. Best-effort; older dumps
    // or a summary failure leave it empty.
    if (esp_core_dump_get_panic_reason(rec.text, sizeof(rec.text)) != ESP_OK)
      rec.text[0] = '\0';
  }
#endif

  // "What led up to it": the ring, unrolled oldest-first into PSRAM for serving. Sanitised to
  // printable-or-newline so it can be streamed as text/plain and measured with strlen-family
  // logic without surprises - RTC noise after a partial survival becomes dots, not a broken page.
  if (s_ring.magic == CR_RING_MAGIC && s_ring.head < CR_LOG_RING && s_ring.used <= CR_LOG_RING && s_ring.used > 0) {
    RAMAllocator<char> alloc(RAMAllocator<char>::ALLOC_EXTERNAL);
    this->tail_ = alloc.allocate(s_ring.used);
    if (this->tail_ != nullptr) {
      const size_t used = s_ring.used;
      size_t from = (used == CR_LOG_RING) ? s_ring.head : 0;
      for (size_t i = 0; i < used; i++) {
        char c = s_ring.data[(from + i) % CR_LOG_RING];
        if (c != '\n' && (c < 0x20 || c == 0x7f))
          c = '.';
        this->tail_[i] = c;
      }
      this->tail_len_ = used;
    }
  }

  // Into the ring of records, oldest shifted out. No lock: setup runs before the web listener
  // accepts anything.
  CrashStore *st = this->store_;
  if (st->count >= CR_RECORDS) {
    memmove(&st->records[0], &st->records[1], sizeof(CrashRecord) * (CR_RECORDS - 1));
    st->count = CR_RECORDS - 1;
  }
  st->records[st->count++] = rec;

  ESP_LOGW(TAG,
           "Previous session (boot %lu) ended in a crash: reason %u, task '%s', PC 0x%08lx%s%s - "
           "%u log bytes recovered",
           static_cast<unsigned long>(rec.boot), rec.reason, rec.task[0] ? rec.task : "?",
           static_cast<unsigned long>(rec.pc), rec.text[0] ? ", " : "", rec.text,
           static_cast<unsigned>(this->tail_len_));
}

void CrashReport::setup() {
  this->partition_present_ =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr) != nullptr;

  // The store lives in PSRAM - 1.4KB the internal heap never needs to hold. Zeroed before load so
  // a missing or short preference behaves as an empty store rather than as noise.
  RAMAllocator<CrashStore> alloc(RAMAllocator<CrashStore>::ALLOC_EXTERNAL);
  this->store_ = alloc.allocate(1);
  if (this->store_ == nullptr) {
    // No PSRAM this early would be a broken board; refuse the feature rather than the boot.
    ESP_LOGE(TAG, "No memory for the crash store; crash reporting disabled");
    this->mark_failed();
    return;
  }
  memset(this->store_, 0, sizeof(CrashStore));
  this->pref_ = global_preferences->make_preference<CrashStore>(fnv1_hash("crash_report_store"));
  if (this->pref_.load(this->store_)) {
    if (this->store_->count > CR_RECORDS) {
      ESP_LOGW(TAG, "Stored crash ring claims %u records; starting fresh", this->store_->count);
      memset(this->store_, 0, sizeof(CrashStore));
    }
  } else {
    memset(this->store_, 0, sizeof(CrashStore));
  }

  const auto reason = esp_reset_reason();
  const bool crashed = reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
                       reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT || reason == ESP_RST_CPU_LOCKUP ||
                       reason == ESP_RST_UNKNOWN;
  if (crashed)
    this->harvest_(static_cast<uint8_t>(reason));

  // The dump's current size, for the download button - independent of `crashed`, because the image
  // sits in flash until someone erases it and stays downloadable across clean reboots.
  size_t addr = 0, size = 0;
  if (this->partition_present_ && esp_core_dump_image_check() == ESP_OK &&
      esp_core_dump_image_get(&addr, &size) == ESP_OK) {
    this->dump_size_ = size;
  }

  // One NVS write per boot: the counter that makes "N restarts ago" computable, plus whatever the
  // harvest appended. Main task, before anything else contends for the preference machinery.
  this->store_->boot_count++;
  this->pref_.save(this->store_);
  global_preferences->sync();

  // Re-arm the flight recorder for this session. The ring restarts empty: its previous contents
  // are either harvested above or from a session that ended cleanly and told us nothing.
  portENTER_CRITICAL(&s_ring_mux);
  s_ring.magic = CR_RING_MAGIC;
  s_ring.head = 0;
  s_ring.used = 0;
  portEXIT_CRITICAL(&s_ring_mux);
  s_mark = RtcMark{};
  this->mark_();

#ifdef USE_LOG_LISTENERS
  // Registered after the harvest so this session's lines can never leak into the previous
  // session's tail. The callback runs on whatever task logs.
  if (logger::global_logger != nullptr)
    logger::global_logger->add_log_callback(this, CrashReport::log_callback_);
#endif
}

void CrashReport::loop() {
  const uint32_t now = millis();
  if (now - this->last_mark_ms_ >= CR_MARK_PERIOD_MS) {
    this->last_mark_ms_ = now;
    this->mark_();
  }

  // The erase endpoint cleared the RAM state from the httpd task and left the NVS write here:
  // ESP32Preferences batches saves in a pending list the main loop owns, and touching it from
  // another task would race every other component's save.
  if (this->save_pending_.exchange(false)) {
    LockGuard guard{this->store_lock_};
    this->pref_.save(this->store_);
    global_preferences->sync();
  }
}

/* ---- The read surface (httpd task) ---------------------------------------------------------- */

uint8_t CrashReport::record_count() {
  LockGuard guard{this->store_lock_};
  return this->store_ == nullptr ? 0 : this->store_->count;
}

bool CrashReport::get_record(uint8_t at, CrashRecord &out) {
  LockGuard guard{this->store_lock_};
  if (this->store_ == nullptr || at >= this->store_->count)
    return false;
  out = this->store_->records[at];
  return true;
}

void CrashReport::erase_all() {
  // The flash erase is a thread-safe IDF call and fine from here; the NVS write is not - see
  // loop(). image_erase on a build without the partition answers an error, which is the no-op
  // this wants.
  esp_core_dump_image_erase();
  {
    LockGuard guard{this->store_lock_};
    if (this->store_ != nullptr)
      this->store_->count = 0;
  }
  this->dump_size_ = 0;
  this->save_pending_.store(true);
  ESP_LOGI(TAG, "Crash history and dump erased");
}

void CrashReport::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Crash Report:\n"
                "  Coredump partition: %s\n"
                "  Boot count: %lu, stored crashes: %u\n"
                "  Dump image: %u bytes\n"
                "  Log capture floor: level %u into %u bytes of RTC ring",
                this->partition_present_ ? "present" : "MISSING (USB flash required to add it)",
                static_cast<unsigned long>(this->boot_count()), this->record_count(),
                static_cast<unsigned>(this->dump_size_), this->capture_level_, static_cast<unsigned>(CR_LOG_RING));
}

}  // namespace crash_report
}  // namespace esphome
