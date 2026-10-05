#include "sysmon.h"

#ifdef USE_SAT1_SYSMON

#include <algorithm>
#include <cstring>

#include <esp_heap_caps.h>
#include <esp_random.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_SM = "web_ui.sysmon";

bool SysMon::begin() {
  RAMAllocator<SysSample> alloc(RAMAllocator<SysSample>::ALLOC_EXTERNAL);
  this->ring_ = alloc.allocate(SM_SLOTS);
  if (this->ring_ == nullptr) {
    ESP_LOGW(TAG_SM, "No PSRAM for the memory history; the system monitor is off");
    return false;
  }
  memset(this->ring_, 0, SM_SLOTS * sizeof(SysSample));
  this->boot_id_ = esp_random();
  return true;
}

void SysMon::loop() {
  if (this->ring_ == nullptr)
    return;
  const uint32_t now = millis();
  if (this->count_ != 0 && static_cast<int32_t>(now - this->next_ms_) < 0)
    return;
  this->next_ms_ = now + SM_INTERVAL_MS;

  SysSample s{};
  s.t = static_cast<uint32_t>(millis_64() / 1000);
  s.int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s.int_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s.int_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s.psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  s.flags = this->flags_fn_ ? this->flags_fn_() : 0;

  LockGuard guard{this->lock_};
  this->ring_[this->count_ % SM_SLOTS] = s;
  this->count_++;
}

size_t SysMon::read(uint32_t &pos, SysSample *out, size_t cap, uint32_t &end) const {
  LockGuard guard{this->lock_};
  end = this->count_;
  if (this->ring_ == nullptr)
    return 0;
  const uint32_t oldest = this->count_ > SM_SLOTS ? this->count_ - SM_SLOTS : 0;
  if (pos < oldest || pos > this->count_)
    pos = oldest;
  const size_t n = std::min<size_t>(cap, this->count_ - pos);
  for (size_t i = 0; i < n; i++)
    out[i] = this->ring_[(pos + i) % SM_SLOTS];
  pos += n;
  return n;
}

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_SYSMON
