#include "log_history.h"

#ifdef USE_SAT1_LOG_HISTORY

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <esp_random.h>
#include <freertos/FreeRTOS.h>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_LOGGER
#include "esphome/components/logger/logger.h"
#endif

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG = "log_history";

/// File scope, not a member: the component object lives in PSRAM (extram_bss), and a spinlock
/// must be in internal RAM.
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;

static bool allocate_(LogRing &ring, size_t size) {
  RAMAllocator<char> alloc(RAMAllocator<char>::ALLOC_EXTERNAL);
  ring.buf = alloc.allocate(size);
  ring.size = ring.buf == nullptr ? 0 : size;
  return ring.buf != nullptr;
}

bool LogHistory::begin(size_t size, size_t alert_size, uint8_t level) {
#ifdef USE_LOGGER
  if (logger::global_logger == nullptr)
    return false;
  if (!allocate_(this->main_, size))
    return false;
  if (!allocate_(this->alert_, alert_size)) {
    RAMAllocator<char>(RAMAllocator<char>::ALLOC_EXTERNAL).deallocate(this->main_.buf, size);
    this->main_ = LogRing{};
    return false;
  }
  this->level_ = level;
  do {
    this->boot_id_ = esp_random();
  } while (this->boot_id_ == 0);
  logger::global_logger->add_log_callback(this, LogHistory::log_callback_);
  return true;
#else
  return false;
#endif
}

void LogHistory::log_callback_(void *self, uint8_t level, const char *tag, const char *message, size_t len) {
  (void) tag;
  auto *me = static_cast<LogHistory *>(self);
  if (level == 0 || level > me->level_)
    return;

  // Assembled here first and committed in one copy, so the lock covers a memcpy rather than the
  // colour stripping. `message` is the console's bytes: ANSI colour runs, then "[W][tag:line]:",
  // then the text - the same bytes crash_report's ring strips the same way.
  char rec[LH_RECORD_MAX];
  int at = snprintf(rec, sizeof(rec), "%" PRIu32 " ", millis());
  if (at < 0)
    return;
  const int start = at;
  for (size_t i = 0; i < len && at < static_cast<int>(sizeof(rec)) - 1; i++) {
    const char c = message[i];
    if (c == '\x1b') {
      while (i < len && message[i] != 'm')
        i++;
      continue;
    }
    if (c == '\r')
      continue;
    rec[at++] = c == '\n' ? '\x1f' : c;
  }
  while (at > start && rec[at - 1] == '\x1f')
    at--;
  rec[at++] = '\n';

  portENTER_CRITICAL(&s_log_mux);
  append_(me->main_, rec, static_cast<size_t>(at));
  if (level <= ESPHOME_LOG_LEVEL_WARN)
    append_(me->alert_, rec, static_cast<size_t>(at));
  portEXIT_CRITICAL(&s_log_mux);
}

void LogHistory::append_(LogRing &r, const char *rec, size_t len) {
  // Whole records off the old end until this one fits. The region between tail and head is only
  // ever whole newline-ended records, so the scan always finds one.
  while (r.size - static_cast<size_t>(r.head - r.tail) < len) {
    size_t n = 0;
    while (r.buf[r.tail_at] != '\n') {
      r.tail_at = r.tail_at + 1 == r.size ? 0 : r.tail_at + 1;
      n++;
    }
    r.tail_at = r.tail_at + 1 == r.size ? 0 : r.tail_at + 1;
    r.tail += n + 1;
  }
  const size_t first = std::min(len, r.size - r.head_at);
  memcpy(r.buf + r.head_at, rec, first);
  memcpy(r.buf, rec + first, len - first);
  r.head_at = (r.head_at + len) % r.size;
  r.head += len;
}

LogHistory::Bounds LogHistory::bounds() const {
  Bounds b{};
  portENTER_CRITICAL(&s_log_mux);
  b.main_tail = this->main_.tail;
  b.main_head = this->main_.head;
  b.alert_tail = this->alert_.tail;
  b.alert_head = this->alert_.head;
  uint32_t ms = 0;
  if (b.main_head != b.main_tail) {
    for (size_t i = 0, at = this->main_.tail_at; i < 10; i++, at = at + 1 == this->main_.size ? 0 : at + 1) {
      const char c = this->main_.buf[at];
      if (c < '0' || c > '9')
        break;
      ms = ms * 10 + static_cast<uint32_t>(c - '0');
    }
  }
  b.main_oldest_ms = ms;
  portEXIT_CRITICAL(&s_log_mux);
  return b;
}

size_t LogHistory::read(Ring which, uint64_t &pos, uint64_t end, char *out, size_t cap) const {
  const LogRing &r = which == Ring::MAIN ? this->main_ : this->alert_;
  cap = std::min(cap, LH_READ_WINDOW);
  size_t n = 0;
  portENTER_CRITICAL(&s_log_mux);
  if (pos < r.tail)
    pos = r.tail;
  end = std::min(end, r.head);
  if (pos < end) {
    n = static_cast<size_t>(std::min<uint64_t>(cap, end - pos));
    const size_t at = static_cast<size_t>(pos % r.size);
    const size_t first = std::min(n, r.size - at);
    memcpy(out, r.buf + at, first);
    memcpy(out + first, r.buf, n - first);
  }
  portEXIT_CRITICAL(&s_log_mux);

  // A window that stopped mid-record gives the partial record back for the next call.
  while (n > 0 && out[n - 1] != '\n')
    n--;
  pos += n;
  return n;
}

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_LOG_HISTORY
