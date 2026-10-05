#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "esphome/core/defines.h"

#ifdef USE_SAT1_SYSMON

#include "esphome/core/helpers.h"

namespace esphome {
namespace satellite1_web_ui {

static constexpr uint32_t SM_INTERVAL_MS = 10 * 1000;
/// Twelve hours of samples at the interval above: about 100 KB of PSRAM.
static constexpr uint32_t SM_SLOTS = 12 * 360;

/// Flag bits, as the browser reads them.
static constexpr uint8_t SM_FLAG_ASSISTANT = 0x01;
static constexpr uint8_t SM_FLAG_XMOS_NOT_READY = 0x02;
static constexpr uint8_t SM_FLAG_MIC_STREAM = 0x04;

struct SysSample {
  uint32_t t;  ///< Uptime, seconds.
  uint32_t int_free;
  uint32_t int_block;  ///< Largest free internal block.
  uint32_t int_min;    ///< Internal low-water mark since boot.
  uint32_t psram_free;
  uint8_t flags;
};

/**
 * Developer builds only: the memory history behind Settings > Developer's system monitor.
 *
 * Samples internal and PSRAM heap every ten seconds into a PSRAM ring that holds twelve hours, so a
 * leak or a fragmentation trend is visible before anyone opened the page. The writer is the main
 * loop; the reader is the httpd task, through read(), under one lock held for a small batch.
 * Positions are absolute sample counts since boot, so a browser asks for what is newer than the
 * last position it was given.
 */
class SysMon {
 public:
  /// Allocates the ring. False when PSRAM cannot hold it; the endpoint then answers 404.
  bool begin();
  bool active() const { return this->ring_ != nullptr; }
  void set_flags_fn(std::function<uint8_t()> fn) { this->flags_fn_ = std::move(fn); }
  void loop();

  /// Copies up to `cap` samples starting at absolute position `pos` (moved forward to the oldest
  /// surviving sample when the ring has passed it) and advances `pos`. Returns the count copied;
  /// `end` is set to the position of the next sample to be written.
  size_t read(uint32_t &pos, SysSample *out, size_t cap, uint32_t &end) const;
  uint32_t boot_id() const { return this->boot_id_; }

 protected:
  SysSample *ring_{nullptr};
  uint32_t count_{0};
  uint32_t next_ms_{0};
  uint32_t boot_id_{0};
  std::function<uint8_t()> flags_fn_{};
  mutable Mutex lock_;
};

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_SYSMON
