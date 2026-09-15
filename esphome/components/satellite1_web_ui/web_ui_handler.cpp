#include "web_ui_handler.h"

#include <cmath>
#include <cstdio>
#include <cstring>

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
    if (url == "/api/sat1/ha/select")
      return Route::HA_SELECT;
    if (url == "/api/sat1/sel")
      return Route::SEL_SET;
#ifdef USE_MICRO_WAKE_WORD
    if (url == "/api/sat1/wakewords")
      return Route::WAKE_WORDS_SET;
#endif
#ifdef USE_MEDIA_PLAYER
    // A prefix, not an equality: the command rides the path as /api/sat1/media/<cmd>, and the
    // handler parses the tail. url_buf is NUL-terminated by url_to, so strncmp is safe here.
    if (strncmp(url_buf, "/api/sat1/media/", 16) == 0)
      return Route::MEDIA_SET;
#endif
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
#ifdef USE_MICRO_WAKE_WORD
  if (url == "/api/sat1/wakewords")
    return Route::WAKE_WORDS;
#endif
#ifdef USE_VOICE_ASSISTANT
  if (url == "/api/sat1/voice")
    return Route::VOICE;
#endif
#ifdef USE_MEDIA_PLAYER
  if (url == "/api/sat1/media")
    return Route::MEDIA;
#endif

  return Route::NONE;
}

bool WebUIHandler::canHandle(AsyncWebServerRequest *request) const { return match_route_(request) != Route::NONE; }

bool WebUIHandler::route_streams_(Route route) {
  // An exhaustive switch rather than a set test, so a route added later has to say here whether it
  // builds its body on the internal heap instead of quietly skipping the guard in handleRequest.
  switch (route) {
    case Route::STATE:
#ifdef USE_VOICE_ASSISTANT
    case Route::VOICE:
#endif
#ifdef USE_MICRO_WAKE_WORD
    case Route::WAKE_WORDS:
#endif
    case Route::SEL:
      return true;
    case Route::NONE:
    case Route::INDEX:
    case Route::HA:
    case Route::HA_REFRESH:
    case Route::HA_SELECT:
    case Route::SEL_SET:
#ifdef USE_MICRO_WAKE_WORD
    case Route::WAKE_WORDS_SET:
#endif
#ifdef USE_MEDIA_PLAYER
    // MEDIA is here as well: its whole body fits a stack snprintf, so unlike the other GETs it
    // needs no AsyncResponseStream and works when the heap could not provide one.
    case Route::MEDIA:
    case Route::MEDIA_SET:
#endif
      // The bundle is sent from PROGMEM, the Home Assistant payload from PSRAM, and every write
      // answers with a string literal. None of them need heap to reply.
      return false;
  }
  return false;
}

void WebUIHandler::send_low_memory_(AsyncWebServerRequest *request) {
  // 503 through the raw httpd API, because init_response_ knows 200, 204, 400, 401, 404, 409 and 422
  // and maps everything else to 500 - and a 500 reads as a bug in this device rather than as "ask me
  // again in a moment". handle_index_ reaches past it the same way for its 304.
  //
  // The status and header strings are literals: httpd_resp_set_status and httpd_resp_set_hdr store the
  // pointer rather than copying, and must stay valid until the send below.
  httpd_resp_set_status(*request, "503 Service Unavailable");
  httpd_resp_set_type(*request, "application/json");
  httpd_resp_set_hdr(*request, "Retry-After", "2");
  httpd_resp_send(*request, R"({"ok":0,"low_memory":1})", HTTPD_RESP_USE_STRLEN);
}

void WebUIHandler::handleRequest(AsyncWebServerRequest *request) {
  const Route route = match_route_(request);

  // A response that cannot be allocated must not be attempted. AsyncResponseStream accumulates into a
  // std::string on the internal heap, and with exceptions off a failed operator new aborts the whole
  // device - so a Diagnostics poll arriving during a squeeze used to reboot it. Answering 503 instead
  // costs the app one stale reading: useDeviceState keeps its error state and useHaData keeps the
  // payload it is holding, which is what they already do for a device that has gone away.
  //
  // Largest block rather than total free, because a response needs one contiguous allocation and not a
  // sum. The threshold is a floor rather than a margin - see WU_STREAM_MIN_BLOCK - and the block that was
  // actually seen is logged, because a 503 nobody can explain is worse than the reboot it replaced.
  if (route_streams_(route)) {
    const size_t block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (block < WU_STREAM_MIN_BLOCK) {
      ESP_LOGW(TAG_WU, "Largest free internal block is %u bytes; answering 503 rather than allocating",
               static_cast<unsigned>(block));
      this->send_low_memory_(request);
      return;
    }
  }

  switch (route) {
    case Route::INDEX:
      this->handle_index_(request);
      break;
    case Route::STATE:
      this->handle_state_(request);
      break;
// Guarded to match the definitions rather than left unconditional. An unreachable case label costs
// nothing, but an unguarded *call* still needs its symbol at link time, and handle_voice_ is only
// defined when voice_assistant is present - so without this the header's claim that a build without
// voice_assistant still compiles was not true.
#ifdef USE_VOICE_ASSISTANT
    case Route::VOICE:
      this->handle_voice_(request);
      break;
#endif
#ifdef USE_MICRO_WAKE_WORD
    case Route::WAKE_WORDS:
      this->handle_wake_words_(request);
      break;
    case Route::WAKE_WORDS_SET:
      this->handle_wake_words_set_(request);
      break;
#endif
    case Route::HA:
      this->handle_ha_(request);
      break;
    case Route::HA_REFRESH:
      this->handle_ha_refresh_(request);
      break;
    case Route::HA_SELECT:
      this->handle_ha_select_(request);
      break;
    case Route::SEL:
      this->handle_sel_(request);
      break;
    case Route::SEL_SET:
      this->handle_sel_set_(request);
      break;
#ifdef USE_MEDIA_PLAYER
    case Route::MEDIA:
      this->handle_media_(request);
      break;
    case Route::MEDIA_SET:
      this->handle_media_set_(request);
      break;
#endif
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

#if defined(USE_VOICE_ASSISTANT) || defined(USE_MICRO_WAKE_WORD)
/// Escapes the two characters that can break a JSON string, plus control characters.
///
/// Most strings this file emits are device names or versions, which are ours. Its two callers are the
/// exceptions: a transcript line comes from speech recognition by way of Home Assistant, and a wake
/// word's name comes from the model manifest fetched at build time. Both are the kind of string where
/// a quotation mark is plausible, and one unescaped would truncate the response into invalid JSON.
///
/// Guarded on either caller, so a build with neither does not carry an unused static.
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
#endif

#ifdef USE_MICRO_WAKE_WORD
/// The wake words this device can listen for, and which are armed.
///
/// Indexed by position, because that is the only key both ends can agree on cheaply and the list is
/// fixed at build time. The id is sent as well, purely so a log or a bug report names something a
/// human recognises; nothing reads it back.
///
/// internal_only models are left out, which is the same view Home Assistant takes of them. Here that
/// means the `stop` model: timer.yaml arms it for the duration of a ringing timer and disarms it after,
/// so a switch for it would be a switch over something that is rewritten from under the reader.
void WebUIHandler::handle_wake_words_(AsyncWebServerRequest *request) {
  auto *stream = request->beginResponseStream("application/json");
  stream->print("[");

  if (this->mww_ != nullptr) {
    const uint32_t mask = this->ww_pending_mask_.load(std::memory_order_acquire);
    const uint32_t wanted = this->ww_pending_on_.load(std::memory_order_relaxed);

    size_t index = 0;
    bool first = true;
    for (auto *model : this->mww_->get_wake_words()) {
      const size_t at = index++;
      if (model == nullptr || model->get_internal_only())
        continue;

      // A press that the main loop has not picked up yet reads as though it already happened. The
      // alternative is answering with the state the caller just changed, which looks like the press
      // was ignored and invites a second one.
      bool on = model->is_enabled();
      if (at < 32 && (mask & (1UL << at)) != 0)
        on = (wanted & (1UL << at)) != 0;

      if (!first)
        stream->print(",");
      first = false;

      stream->printf(R"({"i":%u,"id":)", static_cast<unsigned>(at));
      write_json_string(stream, model->get_id());
      stream->print(R"(,"w":)");
      write_json_string(stream, model->get_wake_word());
      stream->printf(R"(,"on":%s})", on ? "true" : "false");
    }
  }

  stream->print("]");
  stream->addHeader("Cache-Control", CACHE_REVALIDATE);
  request->send(stream);
}

/// Arms or disarms one wake word. Query parameters rather than a JSON body, because two short values
/// fit in a URL and getParam already reads the query string; the selection endpoint next door only
/// posts a body because a selection can outgrow the 1024-byte form limit.
void WebUIHandler::handle_wake_words_set_(AsyncWebServerRequest *request) {
  if (this->mww_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  auto *index_param = request->getParam("i");
  auto *on_param = request->getParam("on");
  if (index_param == nullptr || on_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }

  // strtoul rather than atoi so that a non-numeric i is rejected rather than silently read as model 0,
  // which is a wake word the caller did not name.
  const std::string &index_text = index_param->value();
  char *end = nullptr;
  const unsigned long parsed = strtoul(index_text.c_str(), &end, 10);
  if (end == index_text.c_str() || *end != '\0' || parsed >= 32) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }

  const size_t at = static_cast<size_t>(parsed);
  const auto &models = this->mww_->get_wake_words();
  if (at >= models.size() || models[at] == nullptr || models[at]->get_internal_only()) {
    // 404 rather than 400: the request was well formed and named something this device does not offer.
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  const std::string &on_text = on_param->value();
  const bool on = on_text == "1" || on_text == "true";

  // State first, then the mask, so apply_wake_word_requests can never see a claimed bit whose wanted
  // value has not been written yet.
  const uint32_t bit = 1UL << at;
  if (on) {
    this->ww_pending_on_.fetch_or(bit, std::memory_order_relaxed);
  } else {
    this->ww_pending_on_.fetch_and(~bit, std::memory_order_relaxed);
  }
  this->ww_pending_mask_.fetch_or(bit, std::memory_order_release);

  // Queued, not done - see the comment on ww_pending_mask_. The caller is told which way it will go so
  // it has something truthful to show before the next loop iteration.
  request->send(200, "application/json", on ? "{\"ok\":1,\"on\":1}" : "{\"ok\":1,\"on\":0}");
}

void WebUIHandler::apply_wake_word_requests() {
  const uint32_t mask = this->ww_pending_mask_.exchange(0, std::memory_order_acquire);
  if (mask == 0 || this->mww_ == nullptr)
    return;

  const uint32_t wanted = this->ww_pending_on_.load(std::memory_order_relaxed);
  const auto &models = this->mww_->get_wake_words();

  for (size_t at = 0; at < models.size() && at < 32; at++) {
    if ((mask & (1UL << at)) == 0 || models[at] == nullptr)
      continue;

    const bool on = (wanted & (1UL << at)) != 0;
    if (models[at]->is_enabled() == on)
      continue;

    // enable() and disable() each save to flash, so this is deliberately skipped above when the state
    // already matches: two browsers agreeing about a wake word should not cost an NVS write each.
    if (on) {
      models[at]->enable();
    } else {
      models[at]->disable();
    }
    ESP_LOGD(TAG_WU, "Wake word %s %s", models[at]->get_id().c_str(), on ? "enabled" : "disabled");
  }
}
#endif

#ifdef USE_MEDIA_PLAYER
media_player::MediaPlayer *WebUIHandler::active_media_() const {
  using media_player::MediaPlayerState;
  const auto ss = this->media_sendspin_ != nullptr ? this->media_sendspin_->state : MediaPlayerState::MEDIA_PLAYER_STATE_NONE;
  const auto lc = this->media_local_ != nullptr ? this->media_local_->state : MediaPlayerState::MEDIA_PLAYER_STATE_NONE;

  // The ladder the header explains: a playing group stream wins, then a local player actually making
  // sound, then a paused group stream (so its play button resumes it), then whatever local is.
  if (ss == MediaPlayerState::MEDIA_PLAYER_STATE_PLAYING)
    return this->media_sendspin_;
  if (lc == MediaPlayerState::MEDIA_PLAYER_STATE_PLAYING || lc == MediaPlayerState::MEDIA_PLAYER_STATE_ANNOUNCING)
    return this->media_local_;
  if (ss == MediaPlayerState::MEDIA_PLAYER_STATE_PAUSED)
    return this->media_sendspin_;
  return this->media_local_ != nullptr ? this->media_local_ : this->media_sendspin_;
}

media_player::MediaPlayer *WebUIHandler::resolve_media_(uint8_t src) const {
  if (src == 1)
    return this->media_sendspin_;
  if (src == 2)
    return this->media_local_;
  return this->active_media_();
}

void WebUIHandler::handle_media_(AsyncWebServerRequest *request) {
  auto *active = this->active_media_();
  if (active == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  // Read from the httpd task without a lock, deliberately. `state` is an aligned byte-wide enum and
  // `volume` an aligned 32-bit float, neither of which the ESP32 can tear, and web_server's own
  // entity REST handlers read entity state from this task the same way. The worst case is a value
  // one main-loop iteration old, which the next poll corrects.
  //
  // A stack buffer rather than an AsyncResponseStream: the body is bounded at well under 144 bytes,
  // so unlike the streamed GETs this endpoint keeps answering when the internal heap could not
  // provide a stream - route_streams_ says false for it, and this is why.
  char body[144];
  const int state = static_cast<int>(active->state);
  const int volume = static_cast<int>(std::roundf(active->volume * 100.0f));
  int len = snprintf(body, sizeof(body), R"({"src":"%s","state":%d,"volume":%d,"muted":%d)",
                     active == this->media_sendspin_ ? "sendspin" : "local", state, volume,
                     active->is_muted() ? 1 : 0);

  // The group player rides along whenever it exists, under its own keys. The card needs it to keep a
  // paused group stream operable: the Sendspin protocol has no paused state, so a paused group is an
  // idle player here and `src` above says "local" - and the volume the paused-group card shows has to
  // come from somewhere other than the player the device thinks is active.
  if (len > 0 && static_cast<size_t>(len) < sizeof(body) && this->media_sendspin_ != nullptr) {
    len += snprintf(body + len, sizeof(body) - static_cast<size_t>(len), R"(,"ss_state":%d,"ss_volume":%d)",
                    static_cast<int>(this->media_sendspin_->state),
                    static_cast<int>(std::roundf(this->media_sendspin_->volume * 100.0f)));
  }
  if (len > 0 && static_cast<size_t>(len) + 1 < sizeof(body)) {
    body[len++] = '}';
    body[len] = '\0';
  } else {
    request->send(500, "application/json", "{\"ok\":0}");
    return;
  }
  request->send(200, "application/json", body);
}

void WebUIHandler::handle_media_set_(AsyncWebServerRequest *request) {
  if (this->media_local_ == nullptr && this->media_sendspin_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  request->url_to(url_buf);
  const char *cmd = url_buf + 16;  // Past "/api/sat1/media/", which match_route_ has already tested.

  // An optional explicit target. The card sends the source it is showing, which protects every
  // command from the race where the active player changes between the render and the tap - and is
  // the only way to reach a paused group stream at all, since that reports as an idle player and
  // the active resolution walks past it. No src keeps the resolve-at-apply behaviour.
  uint8_t src = 0;
  if (auto *sp = request->getParam("src"); sp != nullptr) {
    if (sp->value() == "sendspin") {
      src = 1;
    } else if (sp->value() == "local") {
      src = 2;
    } else {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
  }

  if (strcmp(cmd, "volume") == 0) {
    auto *v = request->getParam("v");
    if (v == nullptr) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    // strtoul for the same reason the wake word index uses it: a non-numeric value must be refused,
    // not silently read as zero - which here would be a volume nobody chose.
    const std::string &text = v->value();
    char *end = nullptr;
    const unsigned long parsed = strtoul(text.c_str(), &end, 10);
    if (end == text.c_str() || *end != '\0' || parsed > 100) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    // Target first, then the value: the value is the flag apply reads, so it must never be seen
    // with a stale target beside it - the same ordering the wake word mask uses.
    this->media_pending_vol_src_.store(src, std::memory_order_relaxed);
    this->media_pending_vol_.store(static_cast<int16_t>(parsed), std::memory_order_release);
    request->send(200, "application/json", "{\"ok\":1}");
    return;
  }

  MediaCmd wanted = MediaCmd::NONE;
  if (strcmp(cmd, "play") == 0) {
    wanted = MediaCmd::PLAY;
  } else if (strcmp(cmd, "pause") == 0) {
    wanted = MediaCmd::PAUSE;
  } else if (strcmp(cmd, "next") == 0) {
    wanted = MediaCmd::NEXT;
  } else if (strcmp(cmd, "prev") == 0) {
    wanted = MediaCmd::PREVIOUS;
  }
  if (wanted == MediaCmd::NONE) {
    // 404 rather than 400: the path itself named something this device does not offer.
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  // Target first for the ordering reason the volume pair gives.
  this->media_pending_cmd_src_.store(src, std::memory_order_relaxed);
  this->media_pending_cmd_.store(static_cast<uint8_t>(wanted), std::memory_order_release);
  // Queued, not done - the main loop applies it within an iteration, and the card's next poll is
  // what reports the state that resulted.
  request->send(200, "application/json", "{\"ok\":1}");
}

void WebUIHandler::apply_media_requests() {
  const auto cmd = static_cast<MediaCmd>(this->media_pending_cmd_.exchange(0, std::memory_order_acquire));
  const int16_t vol = this->media_pending_vol_.exchange(-1, std::memory_order_acquire);
  if (cmd == MediaCmd::NONE && vol < 0)
    return;

  // Volume first and as its own call, so a pause arriving in the same iteration cannot make the
  // volume ride a call whose command the player might refuse. Each write resolves its own target: a
  // named player is taken at its word (see resolve_media_), and one this build lacks drops the write.
  if (vol >= 0) {
    auto *vt = this->resolve_media_(this->media_pending_vol_src_.load(std::memory_order_relaxed));
    if (vt != nullptr)
      vt->make_call().set_volume(static_cast<float>(vol) / 100.0f).perform();
  }

  auto *target = this->resolve_media_(this->media_pending_cmd_src_.load(std::memory_order_relaxed));
  if (target == nullptr)
    return;

  switch (cmd) {
    case MediaCmd::PLAY:
      target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_PLAY).perform();
      break;
    case MediaCmd::PAUSE:
      target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_PAUSE).perform();
      break;
    // Track skipping is the group stream's alone: the local speaker player has no queue to skip
    // within, and would log an unsupported-command warning for a button the card never shows on it.
    // Dropped rather than redirected, because a skip aimed at a player that has since stopped being
    // active is a stale intention, not a command for whoever is active now.
    case MediaCmd::NEXT:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_NEXT).perform();
      break;
    case MediaCmd::PREVIOUS:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_PREVIOUS).perform();
      break;
    case MediaCmd::NONE:
      break;
  }
}
#endif

char *WebUIHandler::ha_stage_grow_(size_t capacity) {
  // No lock: the staging buffer is only ever touched from the main loop, where the API callback that
  // writes it runs. What a request can reach is ha_buf_, and the two only meet in commit_ha_payload.
  if (capacity > this->ha_stage_cap_) {
    // reallocate rather than allocate, because the paged path grows the buffer between pages and
    // must not drop the ones already in it.
    char *grown = this->ha_stage_ == nullptr ? this->ha_alloc_.allocate(capacity)
                                             : this->ha_alloc_.reallocate(this->ha_stage_, capacity);
    if (grown == nullptr) {
      // The previous payload is still intact and still served, with its real age. Losing the update
      // is better than dropping what we have for an installation that has not changed much.
      ESP_LOGW(TAG_WU, "No room to stage a %u byte Home Assistant payload; keeping the previous one",
               static_cast<unsigned>(capacity));
      return nullptr;
    }
    this->ha_stage_ = grown;
    this->ha_stage_cap_ = capacity;
  }
  return this->ha_stage_;
}

char *WebUIHandler::stage_ha_payload(size_t capacity) {
  this->ha_stage_len_ = 0;
  return this->ha_stage_grow_(capacity);
}

void WebUIHandler::begin_ha_pages() { this->ha_stage_len_ = 0; }

char *WebUIHandler::stage_ha_page(size_t len) {
  // +1 for the terminator commit_ha_payload writes, since any page can turn out to be the last.
  char *base = this->ha_stage_grow_(this->ha_stage_len_ + len + 1);
  if (base == nullptr)
    return nullptr;
  char *at = base + this->ha_stage_len_;
  this->ha_stage_len_ += len;
  return at;
}

const char *WebUIHandler::commit_ha_pages(int rung) {
  if (this->ha_stage_len_ == 0) {
    ESP_LOGW(TAG_WU, "Home Assistant sync produced no pages; keeping the previous payload");
    return nullptr;
  }
  return this->commit_ha_payload(this->ha_stage_len_, rung);
}

const char *WebUIHandler::commit_ha_payload(size_t len, int rung) {
  if (this->ha_stage_ == nullptr || len + 1 > this->ha_stage_cap_) {
    ESP_LOGW(TAG_WU, "Commit of a %u byte payload with no staging buffer to match; ignored",
             static_cast<unsigned>(len));
    return nullptr;
  }

  // The terminator the served buffer is documented to carry, written here rather than by the caller so
  // that ha_payload_names_'s strstr is legal whatever wrote the bytes.
  this->ha_stage_[len] = '\0';

  LockGuard guard{this->ha_lock_};

  // A swap, not a copy. The buffer that was being served becomes the staging buffer for the next sync,
  // so a resync costs no allocation and no memcpy at all - and the payload has still never been on the
  // internal heap. Both buffers only grow, since a resync of the same installation is nearly the same
  // size and churning PSRAM to save a few hundred bytes would fragment it for no gain.
  std::swap(this->ha_buf_, this->ha_stage_);
  std::swap(this->ha_cap_, this->ha_stage_cap_);
  this->ha_len_ = len;
  this->ha_at_ = static_cast<uint32_t>(millis_64() / 1000);
  this->ha_rung_ = rung;
  ESP_LOGD(TAG_WU, "Home Assistant payload: %u bytes via rung %d", static_cast<unsigned>(len), rung);
  return this->ha_buf_;
}

/// The cached Home Assistant payload, sent out of PSRAM without ever being copied.
///
/// The one endpoint here that does not use an AsyncResponseStream, and the reason is that the stream
/// accumulates into a std::string - which, with CONFIG_SPIRAM_USE_CAPS_ALLOC, is the internal heap. So
/// printing the payload duplicated up to 24KB of PSRAM into the scarce heap, needing a contiguous
/// block of about twice the payload size while the string grew. On a wifi build the WiFi driver's
/// buffers leave that heap around 36KB free, the allocation failed, and a failed operator new with
/// exceptions off calls abort() - the device rebooted every time the app asked for this.
///
/// Chunked, so the payload goes to the socket straight from the pointer. `httpd_resp_send_chunk` is
/// reached through the request's operator httpd_req_t*(), the same way handle_index_ answers a 304 that
/// ESPHome's response API cannot express. No Content-Length results, which is what chunked means and
/// what every fetch() implementation handles.
void WebUIHandler::handle_ha_(AsyncWebServerRequest *request) {
  // Held across the send, so the buffer cannot be reallocated under a response that is still being
  // written. commit_ha_payload is the only writer and runs on the main loop when Home Assistant connects
  // or a browser asks for a resync, so a client slow enough to matter here is also rare enough; if the
  // Diagnostics loop time ever shows it, the fix is a reader flag that makes that writer keep the
  // payload it already has, which is a path it has for a failed allocation.
  LockGuard guard{this->ha_lock_};

  // `d` is whatever Home Assistant rendered, passed through byte for byte rather than reparsed. The
  // age is what lets the app decide to ask again; -1 says nothing has ever arrived, which is a
  // different thing from a payload that is merely old.
  if (this->ha_at_ == 0) {
    char empty[48];
    snprintf(empty, sizeof(empty), R"({"rung":%d,"age":-1,"d":null})", this->ha_rung_);
    request->send(200, "application/json", empty);
    return;
  }

  httpd_resp_set_type(*request, "application/json");
  httpd_resp_set_hdr(*request, "Cache-Control", CACHE_REVALIDATE);

  char head[48];
  const int head_len =
      snprintf(head, sizeof(head), R"({"rung":%d,"age":%u,"d":)", this->ha_rung_,
               static_cast<unsigned int>(static_cast<uint32_t>(millis_64() / 1000) - this->ha_at_));
  httpd_resp_send_chunk(*request, head, head_len);
  httpd_resp_send_chunk(*request, this->ha_buf_, static_cast<ssize_t>(this->ha_len_));
  httpd_resp_send_chunk(*request, "}", 1);
  httpd_resp_send_chunk(*request, nullptr, 0);
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

bool WebUIHandler::ha_payload_names_(const std::string &entity) {
  // strstr over the buffer rather than a std::string built from it. The payload is several kilobytes in
  // PSRAM and copying it to search it would put that on the internal heap, which is the one that is
  // scarce. commit_ha_payload writes the terminator with the payload, which is what makes this legal.
  const std::string quoted = "\"" + entity + "\"";

  LockGuard guard{this->ha_lock_};
  if (this->ha_buf_ == nullptr || this->ha_len_ == 0)
    return false;
  return strstr(this->ha_buf_, quoted.c_str()) != nullptr;
}

/// Queues a change to one of the Home Assistant selects that decide which assistant answers which
/// wake word.
///
/// This is the app's only write that lands somewhere other than this device, and the reason it has to
/// exist is that the setting is not the device's to keep. Home Assistant's ESPHome integration creates
/// two wake-word-and-assistant slot pairs per satellite, resolves which slot matched at the moment a
/// wake word fires, and stores the whole mapping on its own side - so there is no local flag to write.
/// See the `asst` notes in common/web_ui_ha.yaml for the mechanism.
///
/// Deliberately not a general "call any action" endpoint, and two guards keep it that way. The domain
/// must be `select`, and the entity id must appear in the payload Home Assistant last rendered for this
/// device - which is a walk of this device's own entities, so it cannot name a light in another room.
/// Anyone reaching this endpoint has already passed the server's authentication and could drive every
/// entity on the device through web_server anyway, so this is not the security boundary; it is what
/// stops a bug in the app, or in something later built on it, from turning the device into a remote
/// control for the whole installation.
void WebUIHandler::handle_ha_select_(AsyncWebServerRequest *request) {
  auto *entity_param = request->getParam("e");
  auto *option_param = request->getParam("o");
  if (entity_param == nullptr || option_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }

  const std::string &entity = entity_param->value();
  const std::string &option = option_param->value();

  if (entity.rfind("select.", 0) != 0 || !this->ha_payload_names_(entity)) {
    // 404 rather than 400: well formed, and naming something this device has not been told about. A
    // browser holding a payload from before a rename lands here, and refetching is the right answer.
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  // An empty option is rejected rather than forwarded, because Home Assistant would refuse it and this
  // endpoint cannot see that refusal - nothing here captures a response. Control characters go too:
  // the value crosses the API as a protobuf string and is used as a template variable rather than
  // interpolated into one, so there is nothing to escape, but a newline in a pipeline name is a
  // mistake worth failing on rather than passing along.
  if (option.empty() || option.size() > WU_SELECT_OPTION_MAX) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  for (const char c : option) {
    if (static_cast<unsigned char>(c) < 0x20) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
  }

  {
    LockGuard guard{this->select_lock_};

    bool queued = false;
    for (auto &pending : this->select_queue_) {
      if (pending.entity == entity) {
        // Last choice wins. Picking twice from an open dropdown should leave Home Assistant with the
        // second answer and cost one action call, not two whose order decides the outcome.
        pending.option = option;
        queued = true;
        break;
      }
    }

    if (!queued) {
      if (this->select_queue_.size() >= WU_SELECT_QUEUE) {
        // 409 rather than a silent drop, and 409 rather than 429 because init_response_ has no 429 and
        // would answer 500. The app retries; the alternative is a control that looks like it worked.
        request->send(409, "application/json", "{\"ok\":0}");
        return;
      }
      this->select_queue_.push_back({entity, option});
    }

    this->select_pending_.store(true, std::memory_order_release);
  }

  // Queued, like the refresh above. The app learns what actually happened by resyncing and reading the
  // states back out of the next payload, which is the only honest confirmation available: this call
  // captures no response, so the device never finds out whether Home Assistant accepted the option.
  request->send(200, "application/json", "{\"queued\":1}");
}

bool WebUIHandler::take_select_write(SelectWrite &out) {
  if (!this->select_pending_.load(std::memory_order_acquire))
    return false;

  LockGuard guard{this->select_lock_};
  if (this->select_queue_.empty()) {
    this->select_pending_.store(false, std::memory_order_relaxed);
    return false;
  }

  out = this->select_queue_.front();
  this->select_queue_.erase(this->select_queue_.begin());
  if (this->select_queue_.empty())
    this->select_pending_.store(false, std::memory_order_relaxed);
  return true;
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
