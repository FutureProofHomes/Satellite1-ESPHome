#include "web_ui_handler.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <esp_heap_caps.h>
#include <esp_psram.h>
#include <esp_system.h>

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_WU = "web_ui";

/// Revalidate rather than refetch. A long max-age on a URL whose content changes with every
/// firmware update is a bug; "no-cache" plus an ETag means one conditional request per load and a
/// 304 with no body for the 99% of loads where the firmware has not moved.
static const char *const CACHE_REVALIDATE = "no-cache";

static const char *reset_reason_str_() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
      return "Power on";
    case ESP_RST_EXT:
      return "External pin";
    case ESP_RST_SW:
      return "Software reset";
    case ESP_RST_PANIC:
      return "Panic or exception";
    case ESP_RST_INT_WDT:
      return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:
      return "Task watchdog";
    case ESP_RST_WDT:
      return "Other watchdog";
    case ESP_RST_DEEPSLEEP:
      return "Deep sleep wake";
    case ESP_RST_BROWNOUT:
      return "Brownout";
    case ESP_RST_SDIO:
      return "SDIO";
    // The five below are why this switch is exhaustive rather than convenient. A device that has
    // just been flashed over USB reports ESP_RST_USB, so leaving these to the default made the
    // single most common reset during development read as "Unknown" - which is exactly the value
    // a support conversation cannot use.
    case ESP_RST_USB:
      return "USB peripheral";
    case ESP_RST_JTAG:
      return "JTAG";
    case ESP_RST_EFUSE:
      return "eFuse error";
    case ESP_RST_PWR_GLITCH:
      return "Power glitch";
    case ESP_RST_CPU_LOCKUP:
      return "CPU lockup (double exception)";
    default:
      return "Unknown";
  }
}

WebUIHandler::Route WebUIHandler::match_route_(AsyncWebServerRequest *request) {
  if (request->method() != HTTP_GET)
    return Route::NONE;

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const StringRef url = request->url_to(url_buf);

  if (url == WU_URL_ROOT || url == WU_URL_ALIAS || url == WU_URL_ALIAS_SLASH)
    return Route::INDEX;
  if (url == "/api/sat1/state")
    return Route::STATE;

  return Route::NONE;
}

bool WebUIHandler::canHandle(AsyncWebServerRequest *request) const { return match_route_(request) != Route::NONE; }

void WebUIHandler::handleRequest(AsyncWebServerRequest *request) {
  switch (match_route_(request)) {
    case Route::INDEX:
      this->handle_index_(request);
      break;
    case Route::STATE:
      this->handle_state_(request);
      break;
    case Route::NONE:
      break;
  }
}

void WebUIHandler::handle_index_(AsyncWebServerRequest *request) {
  if (this->index_gz_ == nullptr || this->index_gz_len_ == 0) {
    request->send(500, "text/plain", "no bundle");
    return;
  }

  // A conditional request that already has this build. Answered through the raw httpd API because
  // ESPHome's init_response_ has no 304 case and would turn it into a 500 - see the class comment.
  if (this->etag_ != nullptr) {
    auto inm = request->get_header("If-None-Match");
    if (inm.has_value() && inm.value().find(this->etag_) != std::string::npos) {
      httpd_resp_set_status(*request, "304 Not Modified");
      httpd_resp_set_hdr(*request, "ETag", this->etag_);
      httpd_resp_set_hdr(*request, "Cache-Control", CACHE_REVALIDATE);
      httpd_resp_send(*request, nullptr, 0);
      return;
    }
  }

  auto *response = request->beginResponse(200, "text/html", this->index_gz_, this->index_gz_len_);
  response->addHeader("Content-Encoding", "gzip");
  response->addHeader("Cache-Control", CACHE_REVALIDATE);
  if (this->etag_ != nullptr)
    response->addHeader("ETag", this->etag_);
  request->send(response);
}

void WebUIHandler::handle_state_(AsyncWebServerRequest *request) {
  auto *stream = request->beginResponseStream("application/json");

  char mac_buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  get_mac_address_pretty_into_buffer(mac_buf);

  char ip_buf[network::IP_ADDRESS_BUFFER_SIZE] = "";
  for (auto &addr : network::get_ip_addresses()) {
    if (addr.is_set()) {
      addr.str_to(ip_buf);
      break;
    }
  }

  char build_buf[App.BUILD_TIME_STR_SIZE];
  App.get_build_time_string(build_buf);

  // Free-of-total rather than free alone. "142 KB" means nothing; "142 KB of 340 KB" means
  // something, which is the one idea worth keeping from the debug_dashboard prototype's updMem().
  multi_heap_info_t internal{};
  heap_caps_get_info(&internal, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  multi_heap_info_t psram{};
  heap_caps_get_info(&psram, MALLOC_CAP_SPIRAM);

  stream->printf(R"({"name":"%s","friendly_name":"%s","mac":"%s","ip":"%s",)", App.get_name().c_str(),
                 App.get_friendly_name().c_str(), mac_buf, ip_buf);

#ifdef USE_ETHERNET
  stream->print(R"("net":"ethernet",)");
#elif defined(USE_WIFI)
  stream->printf(R"("net":"wifi","rssi":%d,)",
                 wifi::global_wifi_component == nullptr ? 0 : (int) wifi::global_wifi_component->wifi_rssi());
#else
  stream->print(R"("net":"none",)");
#endif

  stream->printf(R"("esphome":"%s","built":"%s",)", ESPHOME_VERSION, build_buf);
#ifdef ESPHOME_PROJECT_NAME
  stream->printf(R"("project":"%s","fw":"%s",)", ESPHOME_PROJECT_NAME, ESPHOME_PROJECT_VERSION);
#endif

  // %u with an explicit cast rather than PRIu32: these are raw string literals, so a PRIu32 in the
  // middle of one is not a macro at all - it is the eleven characters `" PRIu32 "`.
  stream->printf(R"("reset":"%s","uptime":%u,)", reset_reason_str_(), static_cast<unsigned int>(millis_64() / 1000));

  // Whether Home Assistant is actually attached, so the UI can say so rather than leaving someone to
  // work out why their media controls are missing. api_connection_count_ != 0, which is the same
  // condition the LED ring already uses to decide whether the assistant can run at all.
#ifdef USE_API
  stream->printf(R"("ha":%s,)",
                 api::global_api_server != nullptr && api::global_api_server->is_connected() ? "true" : "false");
#else
  stream->print(R"("ha":false,)");
#endif

  stream->printf(R"("heap":{"free":%zu,"total":%zu,"block":%zu},)", internal.total_free_bytes,
                 internal.total_free_bytes + internal.total_allocated_bytes, internal.largest_free_block);
  // Three numbers rather than two, because free-of-total is ambiguous for PSRAM and the difference
  // is large enough to look like a fault. heap_caps only knows the region it was handed - measured
  // 5.4MB on an 8MB part, the rest going to the SPIRAM cache and to allocations made outside the
  // heap - so "4.4 MB free of 5.4 MB" is the true allocator picture while "installed" is the number
  // on the datasheet that a customer would otherwise think we had lost 2.6MB of.
  stream->printf(R"("psram":{"free":%zu,"total":%zu,"installed":%zu},)", psram.total_free_bytes,
                 psram.total_free_bytes + psram.total_allocated_bytes, esp_psram_get_size());

  // Reset on read: the value is "worst loop since you last asked", which is what a diagnostics
  // page refreshing every couple of seconds wants. A never-reset maximum only ever tells you
  // about boot.
  const uint32_t loop_ms = this->max_loop_ms_ == nullptr ? 0 : this->max_loop_ms_->exchange(0);
  stream->printf(R"("loop_ms":%u,)", static_cast<unsigned int>(loop_ms));

  // The key -> "<domain>/<name>" table. Names are read here rather than cached at setup because a
  // few entities are named from runtime state, and because nothing about this is hot: Diagnostics
  // polls a couple of times a second and the app reads the table once per load.
  stream->print(R"("e":{)");
  bool first = true;
  for (const auto &ref : this->entities_) {
    if (ref.entity == nullptr)
      continue;
    stream->printf(R"(%s"%s":"%s/%s")", first ? "" : ",", ref.key, ref.domain, ref.entity->get_name().c_str());
    first = false;
  }
  stream->print("}}");

  request->send(stream);
}

}  // namespace satellite1_web_ui
}  // namespace esphome
