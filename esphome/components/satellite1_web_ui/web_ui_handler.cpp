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
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const StringRef url = request->url_to(url_buf);

  // POST, not PATCH: web_server_idf registers wildcard handlers for GET, POST and OPTIONS only, so
  // any other verb never reaches a handler at all.
  if (request->method() == HTTP_POST) {
    if (url == "/api/sat1/ha/refresh")
      return Route::HA_REFRESH;
    if (url == "/api/sat1/sel")
      return Route::SEL_SET;
    return Route::NONE;
  }

  if (request->method() != HTTP_GET)
    return Route::NONE;

  if (url == WU_URL_ROOT || url == WU_URL_ALIAS || url == WU_URL_ALIAS_SLASH)
    return Route::INDEX;
  if (url == "/api/sat1/state")
    return Route::STATE;
  if (url == "/api/sat1/ha")
    return Route::HA;
  if (url == "/api/sat1/sel")
    return Route::SEL;
#ifdef USE_VOICE_ASSISTANT
  if (url == "/api/sat1/voice")
    return Route::VOICE;
#endif

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
    case Route::VOICE:
      this->handle_voice_(request);
      break;
    case Route::HA:
      this->handle_ha_(request);
      break;
    case Route::HA_REFRESH:
      this->handle_ha_refresh_(request);
      break;
    case Route::SEL:
      this->handle_sel_(request);
      break;
    case Route::SEL_SET:
      this->handle_sel_set_(request);
      break;
    case Route::NONE:
      break;
  }
}

void WebUIHandler::handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  if (match_route_(request) != Route::SEL_SET)
    return;

  // Cleared on the first chunk rather than after the last, so a connection dropped mid-body cannot
  // leave a fragment to be parsed as the front of the next request.
  if (index == 0) {
    this->body_.clear();
    this->body_.reserve(total);
  }
  if (this->body_.size() + len > SEL_BODY_MAX) {
    // Marked poisoned rather than truncated: half a selection is a selection the customer never made.
    this->body_ = "!";
    return;
  }
  if (this->body_ != "!")
    this->body_.append(reinterpret_cast<const char *>(data), len);
}

void WebUIHandler::handle_sel_(AsyncWebServerRequest *request) {
  auto *stream = request->beginResponseStream("application/json");
  if (this->selection_ == nullptr) {
    stream->print("{}");
  } else {
    std::string json;
    this->selection_->to_json(json);
    stream->print(json);
  }
  stream->addHeader("Cache-Control", CACHE_REVALIDATE);
  request->send(stream);
}

/// Reads one JSON string array into a comma-separated list, without a JSON parser.
///
/// ArduinoJson is only linked in when something in the config uses capture_response, and this endpoint
/// has to work regardless - so this walks the text instead. The shape is ours on both ends and narrow:
/// arrays of quoted ids containing no escapes, because area ids and entity ids are slugs.
static bool json_array_to_csv(const std::string &body, size_t from, size_t to, const char *key, std::string &out) {
  out.clear();
  const std::string needle = std::string("\"") + key + "\":[";
  const size_t at = body.find(needle, from);
  if (at == std::string::npos || at >= to)
    return false;

  size_t i = at + needle.size();
  while (i < to && body[i] != ']') {
    if (body[i] != '"') {
      i++;
      continue;
    }
    const size_t start = ++i;
    while (i < to && body[i] != '"')
      i++;
    if (i >= to)
      return false;
    if (!out.empty())
      out += ',';
    out.append(body, start, i - start);
    i++;
  }
  return i < to;
}

/// Locates `"key":{ ... }` and returns the span between the braces, so the array reader above cannot
/// pick up the routing arrays when it was asked for the ducking ones.
static bool json_object_span(const std::string &body, const char *key, size_t &from, size_t &to) {
  const std::string needle = std::string("\"") + key + "\":{";
  const size_t at = body.find(needle);
  if (at == std::string::npos)
    return false;
  from = at + needle.size();
  to = body.find('}', from);
  return to != std::string::npos;
}

void WebUIHandler::handle_sel_set_(AsyncWebServerRequest *request) {
  if (this->selection_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  if (this->body_ == "!" || this->body_.empty()) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }

  SelectionSet routing;
  SelectionSet duck;
  size_t from = 0;
  size_t to = 0;

  bool ok = json_object_span(this->body_, "routing", from, to);
  ok = ok && json_array_to_csv(this->body_, from, to, "areas", routing.areas);
  ok = ok && json_array_to_csv(this->body_, from, to, "extra", routing.extra);
  ok = ok && json_array_to_csv(this->body_, from, to, "excluded", routing.excluded);
  ok = ok && json_object_span(this->body_, "duck", from, to);
  ok = ok && json_array_to_csv(this->body_, from, to, "areas", duck.areas);
  ok = ok && json_array_to_csv(this->body_, from, to, "extra", duck.extra);
  ok = ok && json_array_to_csv(this->body_, from, to, "excluded", duck.excluded);

  if (!ok) {
    ESP_LOGW(TAG_WU, "Selection POST did not parse; keeping the previous one");
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }

  // Absent means unchanged rather than false, so a caller that only wants to move a checkbox in the
  // tree does not have to know about the local speaker to avoid silencing this device.
  bool local = this->selection_->local_speaker();
  if (this->body_.find("\"local\":1") != std::string::npos) {
    local = true;
  } else if (this->body_.find("\"local\":0") != std::string::npos) {
    local = false;
  }
  this->selection_->replace(routing, duck, local);
  this->body_.clear();

  // 200 rather than the 204 this deserves: init_response_ knows 200, 204, 400, 401, 404, 409 and 422,
  // and 204 is in that set - but a body is more useful to the caller than a bare status, and a 204
  // carrying one is malformed.
  request->send(200, "application/json", "{\"ok\":1}");
}

void WebUIHandler::set_ha_payload(const std::string &json, int rung) {
  LockGuard guard{this->ha_lock_};

  if (json.size() + 1 > this->ha_cap_) {
    char *grown = this->ha_buf_ == nullptr ? this->ha_alloc_.allocate(json.size() + 1)
                                           : this->ha_alloc_.reallocate(this->ha_buf_, json.size() + 1);
    if (grown == nullptr) {
      // The previous payload is still intact and still served, with its real age. Losing the update
      // is better than dropping what we have for an installation that has not changed much.
      ESP_LOGW(TAG_WU, "No room for a %u byte Home Assistant payload; keeping the previous one",
               static_cast<unsigned>(json.size()));
      return;
    }
    this->ha_buf_ = grown;
    this->ha_cap_ = json.size() + 1;
  }

  memcpy(this->ha_buf_, json.c_str(), json.size() + 1);
  this->ha_len_ = json.size();
  this->ha_at_ = static_cast<uint32_t>(millis_64() / 1000);
  this->ha_rung_ = rung;
  ESP_LOGD(TAG_WU, "Home Assistant payload: %u bytes via rung %d", static_cast<unsigned>(json.size()), rung);
}

void WebUIHandler::handle_ha_(AsyncWebServerRequest *request) {
  auto *stream = request->beginResponseStream("application/json");

  LockGuard guard{this->ha_lock_};

  // `d` is whatever Home Assistant rendered, passed through byte for byte rather than reparsed. The
  // age is what lets the app decide to ask again; -1 says nothing has ever arrived, which is a
  // different thing from a payload that is merely old.
  //
  // AsyncResponseStream::print has overloads for const char * and float but not for an integer, so
  // every number here goes through printf - a float would render an age as "12.00". The payload goes
  // out through the const char * overload because there is no block write; set_ha_payload copies the
  // terminator with it for exactly this reason.
  if (this->ha_at_ == 0) {
    stream->printf(R"({"rung":%d,"age":-1,"d":null})", this->ha_rung_);
  } else {
    stream->printf(R"({"rung":%d,"age":%u,"d":)", this->ha_rung_,
                   static_cast<unsigned int>(static_cast<uint32_t>(millis_64() / 1000) - this->ha_at_));
    stream->print(this->ha_buf_);
    stream->print("}");
  }

  stream->addHeader("Cache-Control", CACHE_REVALIDATE);
  request->send(stream);
}

void WebUIHandler::handle_ha_refresh_(AsyncWebServerRequest *request) {
  // Only a request. The sync itself is a Home Assistant action call, which has to be started from the
  // main loop, so loop() picks this up and fires the trigger that runs the script.
  this->ha_refresh_requested_.store(true);

  // 200 and not the 202 this deserves: init_response_ knows 200, 204, 400, 401, 404, 409 and 422, and
  // maps everything else to 500 - so a 202 was answered as a 500 while still sending this body, which
  // is a failure the browser would have believed. The body carries the meaning instead.
  request->send(200, "application/json", "{\"queued\":1}");
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

#ifdef USE_VOICE_ASSISTANT

void WebUIHandler::push_utterance(const std::string &text, bool heard) {
  if (text.empty())
    return;
  LockGuard guard{this->transcript_lock_};
  if (this->transcript_.size() >= WU_TRANSCRIPT_RING)
    this->transcript_.erase(this->transcript_.begin());
  this->transcript_.push_back({text, static_cast<uint32_t>(millis_64() / 1000), heard});
}

/// Escapes the two characters that can break a JSON string, plus control characters.
///
/// Every other string this file emits is a device name or a version, but these come from speech
/// recognition by way of Home Assistant, so they are the one place a quotation mark or a stray
/// newline is plausible - and unescaped, one would truncate the whole response into invalid JSON.
static void write_json_string(AsyncResponseStream *stream, const std::string &text) {
  stream->print("\"");
  for (const char c : text) {
    switch (c) {
      case '"':
        stream->print("\\\"");
        break;
      case '\\':
        stream->print("\\\\");
        break;
      case '\n':
        stream->print("\\n");
        break;
      case '\r':
        stream->print("\\r");
        break;
      case '\t':
        stream->print("\\t");
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          stream->printf("\\u%04x", static_cast<unsigned int>(c));
        } else {
          stream->write(static_cast<uint8_t>(c));
        }
        break;
    }
  }
  stream->print("\"");
}

/// Timers and the assistant's phase, neither of which web_server can express.
///
/// Timers are held entirely on the device - VoiceAssistant::get_timers() returns its own vector, and
/// they keep counting and still ring with Home Assistant gone - but they are not entities, so there
/// is no REST path or /events id for them. The phase is the `voice_assistant_phase` global that
/// config/ already maintains.
void WebUIHandler::handle_voice_(AsyncWebServerRequest *request) {
  auto *stream = request->beginResponseStream("application/json");

  const int phase = this->voice_phase_fn_ ? this->voice_phase_fn_() : 0;
  stream->printf(R"({"phase":%d,"running":%s,)", phase,
                 this->va_ != nullptr && this->va_->is_running() ? "true" : "false");

  stream->print(R"("timers":[)");
  if (this->va_ != nullptr) {
    bool first = true;
    for (const auto &timer : this->va_->get_timers()) {
      stream->printf(R"(%s{"id":)", first ? "" : ",");
      write_json_string(stream, timer.id);
      stream->print(R"(,"name":)");
      write_json_string(stream, timer.name);
      stream->printf(R"(,"total":%u,"left":%u,"active":%s})", static_cast<unsigned int>(timer.total_seconds),
                     static_cast<unsigned int>(timer.seconds_left), timer.is_active ? "true" : "false");
      first = false;
    }
  }
  stream->print("],");

  stream->print(R"("transcript":[)");
  {
    LockGuard guard{this->transcript_lock_};
    bool first = true;
    for (const auto &line : this->transcript_) {
      stream->printf(R"(%s{"heard":%s,"at":%u,"text":)", first ? "" : ",", line.heard ? "true" : "false",
                     static_cast<unsigned int>(line.at_uptime));
      write_json_string(stream, line.text);
      stream->print("}");
      first = false;
    }
  }
  stream->print("]}");

  request->send(stream);
}

#endif  // USE_VOICE_ASSISTANT

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
