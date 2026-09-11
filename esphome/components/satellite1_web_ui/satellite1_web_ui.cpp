#include "satellite1_web_ui.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG = "satellite1_web_ui";

void Satellite1WebUI::setup() {
  this->handler_.set_loop_time_source(&this->max_loop_ms_);

  // Before the handler is registered, so a request arriving immediately cannot read an empty
  // selection and report that nothing is configured.
  this->selection_.setup();
  this->handler_.set_selection(&this->selection_);

  // Only a flag is set here. The callback runs on the httpd task, and what it ultimately starts are
  // Home Assistant actions, so the trigger has to be fired from the main loop instead.
  this->selection_.add_on_change_callback([this]() { this->selection_changed_.store(true); });

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

  // Same reasoning, and the same collapsing: dragging through a tree writes several times in a
  // second, and each one would otherwise re-run every routing check.
  if (this->selection_changed_.exchange(false))
    this->selection_change_trigger_.trigger();
}

void Satellite1WebUI::set_ha_payload(const std::string &json, int rung) {
  this->handler_.set_ha_payload(json, rung);

  // A find rather than a JSON parse: ArduinoJson is only linked in when something in the config uses
  // capture_response, and the area id is one flat string in a payload we generate ourselves. An empty
  // or missing `aid` is left alone rather than written, so a sync that happens while the device is
  // between areas cannot clear an area the switches are currently projecting.
  static const char *const KEY = "\"aid\":\"";
  const size_t at = json.find(KEY);
  if (at == std::string::npos)
    return;
  const size_t start = at + strlen(KEY);
  const size_t end = json.find('"', start);
  if (end == std::string::npos || end == start)
    return;
  this->selection_.set_own_area(json.substr(start, end - start));
}

void Satellite1WebUI::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Web UI:\n"
                     "  Serving / and /ui/ from PROGMEM");
}

}  // namespace satellite1_web_ui
}  // namespace esphome
