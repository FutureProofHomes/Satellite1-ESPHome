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

  // One per iteration rather than draining the queue, so the automation cannot be re-entered. Choosing
  // an assistant for a wake word can be up to four writes, which then land over four iterations.
  SelectWrite pending;
  if (this->handler_.take_select_write(pending))
    this->ha_select_trigger_.trigger(pending.entity, pending.option);

#ifdef USE_MICRO_WAKE_WORD
  // Not a trigger, because nothing in YAML has to happen - this is the whole of the work. It runs here
  // rather than in the endpoint because enabling a model writes NVS and races the inference task.
  this->handler_.apply_wake_word_requests();
#endif
}

char *Satellite1WebUI::stage_ha_payload(size_t capacity) { return this->handler_.stage_ha_payload(capacity); }

void Satellite1WebUI::commit_ha_payload(size_t len, int rung) {
  this->adopt_ha_area_(this->handler_.commit_ha_payload(len, rung));
}

void Satellite1WebUI::commit_ha_pages(int rung) { this->adopt_ha_area_(this->handler_.commit_ha_pages(rung)); }

void Satellite1WebUI::adopt_ha_area_(const char *json) {
  if (json == nullptr)
    return;

  // A find over the buffer rather than a JSON parse, and rather than a std::string built from it:
  // ArduinoJson is only linked in when something in the config uses capture_response, the area id is one
  // flat string in a payload we generate ourselves, and a copy of the payload to search it would put
  // several kilobytes on the internal heap - which is the whole thing the staging pair avoids. An empty
  // or missing `aid` is left alone rather than written, so a sync that happens while the device is
  // between areas cannot clear an area the switches are currently projecting.
  static const char *const KEY = "\"aid\":\"";
  const char *at = strstr(json, KEY);
  if (at == nullptr)
    return;
  const char *start = at + strlen(KEY);
  const char *end = strchr(start, '"');
  if (end == nullptr || end == start)
    return;
  this->selection_.set_own_area(std::string(start, static_cast<size_t>(end - start)));
}

void Satellite1WebUI::set_ha_payload(const std::string &json, int rung) {
  char *staged = this->stage_ha_payload(json.size() + 1);
  if (staged == nullptr)
    return;
  memcpy(staged, json.c_str(), json.size());
  this->commit_ha_payload(json.size(), rung);
}

void Satellite1WebUI::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Web UI:\n"
                     "  Serving / and /ui/ from PROGMEM");
}

}  // namespace satellite1_web_ui
}  // namespace esphome
