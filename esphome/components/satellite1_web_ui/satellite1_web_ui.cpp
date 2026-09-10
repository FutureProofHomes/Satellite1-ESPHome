#include "satellite1_web_ui.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG = "satellite1_web_ui";

void Satellite1WebUI::setup() {
  this->handler_.set_loop_time_source(&this->max_loop_ms_);

  // add_handler, not add_handler_without_auth: the app reaches every entity this device has,
  // Factory Reset and XMOS Erase Chip included.
  web_server_base::global_web_server_base->add_handler(&this->handler_);

  this->last_loop_ms_ = millis();
}

void Satellite1WebUI::loop() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->last_loop_ms_;
  this->last_loop_ms_ = now;

  // Monotonic max, so a slow loop is never lost to a concurrent read that happened to land first.
  uint32_t seen = this->max_loop_ms_.load(std::memory_order_relaxed);
  while (elapsed > seen && !this->max_loop_ms_.compare_exchange_weak(seen, elapsed, std::memory_order_relaxed)) {
  }

  // Collapses any number of refresh requests since the last iteration into one sync, which is what we
  // want: several tabs opening at once should ask Home Assistant a single time.
  if (this->handler_.take_ha_refresh_request())
    this->ha_refresh_trigger_.trigger();
}

void Satellite1WebUI::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Web UI:\n"
                     "  Serving / and /ui/ from PROGMEM");
}

}  // namespace satellite1_web_ui
}  // namespace esphome
