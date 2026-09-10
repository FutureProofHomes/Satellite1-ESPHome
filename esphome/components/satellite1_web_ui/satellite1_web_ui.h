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

 protected:
  WebUIHandler handler_;

  /// Our loop() runs once per main-loop iteration, so the gap between two calls is the main loop
  /// period. That makes the loop-time readout free, where the debug: component would cost a sensor
  /// entity riding /events to every open tab for a number only Diagnostics ever shows.
  uint32_t last_loop_ms_{0};
  std::atomic<uint32_t> max_loop_ms_{0};
};

}  // namespace satellite1_web_ui
}  // namespace esphome
