#pragma once

#include <atomic>

#include "esphome/core/component.h"

#include "web_ui_handler.h"

namespace esphome {
namespace satellite1_web_ui {

/**
 * Serves the on-device web app from PROGMEM as a handler on ESPHome's shared web server.
 *
 * The whole component exists at setup_priority::WIFI (250) for one reason: registration order is
 * what decides who answers "/". WebServerBase::add_handler appends to a vector that
 * AsyncWebServer::request_handler_ walks in order, first canHandle wins, and WebServer claims "/"
 * unconditionally at web_server.cpp:2339. WebServer::get_setup_priority() is WIFI - 1.0f = 249, so
 * anything above that registers first and takes the root.
 *
 * Registering from setup() rather than from codegen is load bearing, not stylistic.
 * WebServerBase::add_handler wraps a handler in AuthMiddlewareHandler only if credentials_.is_set()
 * at the moment it is called, and web_server's to_code emits set_auth_username() as a statement in
 * main.cpp. Every codegen statement runs before App.setup(), so a setup() registration is always
 * after the username is installed. A codegen registration placed ahead of web_server's own
 * statement would put the whole app on port 80 unauthenticated, silently.
 */
class Satellite1WebUI : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  void set_index(const uint8_t *gz, size_t gz_len) { this->handler_.set_index(gz, gz_len); }
  void set_etag(const char *etag) { this->handler_.set_etag(etag); }
  void add_entity(const char *key, const char *domain, EntityBase *entity) {
    this->handler_.add_entity(key, domain, entity);
  }

#ifdef USE_VOICE_ASSISTANT
  void set_voice_assistant(voice_assistant::VoiceAssistant *va) { this->handler_.set_voice_assistant(va); }
  void set_voice_phase_fn(std::function<int()> fn) { this->handler_.set_voice_phase_fn(std::move(fn)); }

  /// Called from the on_stt_end and on_tts_start lambdas in common/voice_assistant.yaml.
  void push_utterance(const std::string &text, bool heard) { this->handler_.push_utterance(text, heard); }
#endif

#ifdef USE_MICRO_WAKE_WORD
  void set_micro_wake_word(micro_wake_word::MicroWakeWord *mww) { this->handler_.set_micro_wake_word(mww); }
#endif

  /// All called from the rung lambdas in common/web_ui_ha.yaml.
  ///
  /// The payload arrives as a buffer rather than a string, in two steps: stage hands out PSRAM to write
  /// into, commit publishes what was written. What that buys is that the several kilobytes Home Assistant
  /// renders never exist on the internal heap - a std::string of them did, and the allocation aborted the
  /// device on syncs where the heap could not find one contiguous block that size.
  ///
  /// Commit also lifts `aid` out of the payload into the selection store, which is the only moment the
  /// device can learn its own Home Assistant area. Done here rather than in the YAML lambda so the one
  /// place that receives the payload is the one place that unpacks it.
  char *stage_ha_payload(size_t capacity);
  void commit_ha_payload(size_t len, int rung);

  /// The same thing for a caller that already holds the payload as a string. Not what the sync uses: by
  /// the time such a string exists, the internal-heap copy the pair above avoids has been paid for.
  void set_ha_payload(const std::string &json, int rung);
  void set_ha_failed() { this->handler_.set_ha_failed(); }

  /// Fired from loop() when a browser has posted to /api/sat1/ha/refresh. A trigger rather than a
  /// direct call, because the work is a Home Assistant action and the script that owns it lives in
  /// YAML - which also keeps the ladder's only caller in one file.
  Trigger<> *get_ha_refresh_trigger() { return &this->ha_refresh_trigger_; }

  /// Fired from loop() with an entity id and an option, for the Home Assistant selects that decide
  /// which assistant answers which wake word. The action call itself is in YAML for the same reason the
  /// refresh ladder is: homeassistant.action belongs next to the rest of the data layer.
  ///
  /// At most one per iteration, so the automation behind this is never re-entered while running. A
  /// `delay:` in it would still be a mistake - it would stall the queue rather than corrupt it.
  Trigger<std::string, std::string> *get_ha_select_trigger() { return &this->ha_select_trigger_; }

  /// Fired from loop() after the app has written a new selection. tts_routing.yaml hangs its re-check
  /// scripts here, in place of the `on_value` the deleted Remote TTS Targets text entity carried.
  ///
  /// Deferred to loop() rather than fired from the change callback, because that callback runs on the
  /// httpd task and the scripts it starts perform Home Assistant actions.
  Trigger<> *get_selection_change_trigger() { return &this->selection_change_trigger_; }

  /// What the app has chosen, and the only copy of it. Read from YAML lambdas in tts_routing.yaml and
  /// area_ducking.yaml, and by the template switches that project it.
  Selection &selection() { return this->selection_; }

 protected:
  WebUIHandler handler_;
  Selection selection_;
  Trigger<> ha_refresh_trigger_;
  Trigger<std::string, std::string> ha_select_trigger_;
  Trigger<> selection_change_trigger_;
  std::atomic<bool> selection_changed_{false};

  /// Our loop() runs once per main-loop iteration, so the gap between two calls is the main loop
  /// period. That makes the loop-time readout free, where the debug: component would cost a sensor
  /// entity riding /events to every open tab for a number only Diagnostics ever shows.
  uint32_t last_loop_ms_{0};
  std::atomic<uint32_t> max_loop_ms_{0};
};

}  // namespace satellite1_web_ui
}  // namespace esphome
