#pragma once

#include <atomic>
#include <functional>
#include <string>

#include "esphome/components/web_server_base/web_server_base.h"

#include "ld2410_handler.h"
#include "ld2450_handler.h"

namespace esphome {
namespace satellite1_radar {

static const int RT_NUM_TARGETS = 3;
static const int RT_NUM_GATES = 9;

/// Path the tuner page is served from. Deliberately not "/": web_server owns the root, and in
/// Phase 2 the SPA takes it over. The tuner's API paths are absolute from the origin
/// (`const BASE=window.location.origin` in both tuner_ui pages), so moving the page off the root
/// needs no change to the HTML.
static const char *const RT_URL_ROOT = "/radar_tuner";

/**
 * The radar tuner's HTTP surface, as a handler on ESPHome's shared web server rather than a
 * second esp_http_server of its own.
 *
 * Two consequences of sharing that shaped this file:
 *
 * The httpd task stack is no longer ours to size. The old server asked for 8192 bytes; ESPHome's
 * `AsyncWebServer::begin()` hardcodes `HTTPD_DEFAULT_CONFIG().stack_size + 256`, which is 4352,
 * with no YAML knob. The LD2450 config response used to be built in a `char buf[3072]` on that
 * stack. Every response here is streamed into an `AsyncResponseStream` instead, which accumulates
 * on the heap.
 *
 * `web_server_idf` registers exactly three wildcard URI handlers - GET, POST and OPTIONS - so
 * PATCH cannot reach us at all. Both config endpoints are POST, and a POST whose Content-Type is
 * not form-urlencoded arrives through `handleBody` in 1460-byte chunks followed by one
 * `handleRequest`, which is why the body is accumulated rather than read inline.
 */
class RadarTunerHandler : public AsyncWebHandler {
 public:
  void set_html_content(const uint8_t *data, unsigned int len) {
    html_gz_ = data;
    html_gz_len_ = len;
  }

  void set_ld2410_handler(LD2410Handler *handler) { ld2410_ = handler; }
  void set_ld2450_handler(LD2450Handler *handler) { ld2450_ = handler; }
  void set_ld2410_apply_callback(std::function<void()> cb) { on_ld2410_apply_ = std::move(cb); }

  /// Called from the main loop via LD2450Handler::on_target_update, read from the httpd task.
  /// Three plain floats, torn at worst, exactly as the previous server had it: a display coordinate
  /// one poll out of date is not worth a mutex on the parse path.
  void update_target(int index, float x, float y);

  void clear_registrations();

  /// millis() at the last GET /api/v1/ld2410/live, or 0 if never. Read by Satellite1Radar::loop()
  /// to decide whether the LD2410 needs engineering mode right now.
  ///
  /// The old server turned engineering mode on when the tuner switch went on and off when it went
  /// off. With the tuner always mounted there is no such switch, and leaving engineering mode on
  /// forever would mean parsing per-gate energy out of every frame for a page nobody is looking
  /// at. So the gate becomes "is someone polling", and the decision is made on the main loop
  /// rather than here: enable_engineering_mode() pushes onto the handler's command queue, which is
  /// drained by process_queue_() from loop() and is not guarded for another task's writes.
  uint32_t ld2410_live_poll_ms() const { return this->ld2410_live_poll_ms_.load(std::memory_order_relaxed); }

  // NOLINTNEXTLINE(readability-identifier-naming)
  bool canHandle(AsyncWebServerRequest *request) const override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleRequest(AsyncWebServerRequest *request) override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  bool isRequestHandlerTrivial() const override { return false; }

 protected:
  /// One route table for both canHandle and handleRequest, so they can never disagree about which
  /// paths exist.
  enum class Route : uint8_t {
    NONE = 0,
    ROOT,
    LD2410_CONFIG_GET,
    LD2410_CONFIG_SET,
    LD2410_APPLY,
    LD2410_LIVE,
    LD2450_CONFIG_GET,
    LD2450_CONFIG_SET,
    LD2450_LIVE,
    SAVE,
    REBOOT,
  };

  static Route match_route_(AsyncWebServerRequest *request);

  void handle_root_(AsyncWebServerRequest *request);
  void handle_ld2410_get_config_(AsyncWebServerRequest *request);
  void handle_ld2410_set_config_(AsyncWebServerRequest *request);
  void handle_ld2410_apply_(AsyncWebServerRequest *request);
  void handle_ld2410_live_(AsyncWebServerRequest *request);
  void handle_ld2450_get_config_(AsyncWebServerRequest *request);
  void handle_ld2450_set_config_(AsyncWebServerRequest *request);
  void handle_ld2450_live_(AsyncWebServerRequest *request);
  void handle_save_(AsyncWebServerRequest *request);
  void handle_reboot_(AsyncWebServerRequest *request);

  LD2410Handler *ld2410_{nullptr};
  LD2450Handler *ld2450_{nullptr};

  const uint8_t *html_gz_{nullptr};
  unsigned int html_gz_len_{0};
  std::function<void()> on_ld2410_apply_;

  /// Accumulated request body. Reset on the chunk at index 0 rather than after use, so a request
  /// that dies mid-body cannot leave a fragment to be prepended to the next one.
  std::string body_;

  std::atomic<uint32_t> ld2410_live_poll_ms_{0};

  struct TargetData {
    float x{0};
    float y{0};
  };
  TargetData targets_[RT_NUM_TARGETS]{};
};

}  // namespace satellite1_radar
}  // namespace esphome
