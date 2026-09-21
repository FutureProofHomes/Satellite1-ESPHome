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

#ifdef USE_SAT1_CRASH_REPORT
#include <esp_partition.h>
#endif

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_WU = "web_ui";

/// The remote-control API contract version - see the comment where handle_state_ serves it.
static constexpr int WU_API_VERSION = 1;

/// Revalidate rather than refetch. A long max-age on a URL whose content changes with every
/// firmware update is a bug; "no-cache" plus an ETag means one conditional request per load and a
/// 304 with no body for the 99% of loads where the firmware has not moved.
static const char *const CACHE_REVALIDATE = "no-cache";

/// By value rather than reading esp_reset_reason() itself, because the crash endpoint names past
/// sessions' reasons from stored records with the same vocabulary.
static const char *reset_reason_name_(esp_reset_reason_t reason) {
  switch (reason) {
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

static const char *reset_reason_str_() { return reset_reason_name_(esp_reset_reason()); }

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
    if (url == "/api/sat1/ma/refresh")
      return Route::MA_REFRESH;
    // A prefix like the media commands: the verb rides the path as /api/sat1/ma/<cmd>. Tested after
    // the refresh above, which would otherwise match the prefix too.
    if (strncmp(url_buf, "/api/sat1/ma/", 13) == 0)
      return Route::MA_SET;
    if (url == "/api/sat1/sel")
      return Route::SEL_SET;
#ifdef USE_SAT1_MWW_LOADER
    // Before the bare path below would ever match: these are prefixes of nothing, but the bare
    // /api/sat1/wakewords is a prefix of these, so exact-match order matters to nobody - listed
    // first purely so the reader sees the live routes ahead of the legacy one.
    if (url == "/api/sat1/wakewords/slot")
      return Route::WAKE_SLOT_SET;
    if (url == "/api/sat1/wakewords/cutoff")
      return Route::WAKE_CUTOFF_SET;
    if (url == "/api/sat1/wakewords/tune")
      return Route::WAKE_TUNE_SET;
#endif
#if defined(USE_MICRO_WAKE_WORD) && !defined(USE_SAT1_MWW_LOADER)
    if (url == "/api/sat1/wakewords")
      return Route::WAKE_WORDS_SET;
#endif
#ifdef USE_MEDIA_PLAYER
    // A prefix, not an equality: the command rides the path as /api/sat1/media/<cmd>, and the
    // handler parses the tail. url_buf is NUL-terminated by url_to, so strncmp is safe here.
    if (strncmp(url_buf, "/api/sat1/media/", 16) == 0)
      return Route::MEDIA_SET;
#endif
#ifdef USE_SAT1_CRASH_REPORT
    if (url == "/api/sat1/crash/erase")
      return Route::CRASH_ERASE;
#endif
    return Route::NONE;
  }

  if (request->method() != HTTP_GET)
    return Route::NONE;

  if (url == WU_URL_ROOT || url == WU_URL_ALIAS || url == WU_URL_ALIAS_SLASH)
    return Route::INDEX;
  if (url == "/ui/no-sensor.webp")
    return Route::ASSET_NO_SENSOR;
  // The PWA surface. All exempt in the session gate, because a browser needs them before it has a
  // session - iOS fetches the touch icon at add-to-home-screen time with no cookies at all.
  if (url == "/manifest.webmanifest")
    return Route::MANIFEST;
  if (url == "/ui/icon-192.png")
    return Route::ICON_192;
  if (url == "/ui/icon-512.png")
    return Route::ICON_512;
  if (url == "/apple-touch-icon.png")
    return Route::ICON_180;
  if (url == "/api/sat1/state")
    return Route::STATE;
  if (url == "/api/sat1/ha")
    return Route::HA;
  if (url == "/api/sat1/ma")
    return Route::MA;
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
#ifdef USE_SAT1_WEB_UI_SOUNDS
  // A prefix like MEDIA_SET's: the sound's name rides the path. Exempt in the session gate for the
  // same reason the PWA assets are - the fetcher is a Sonos or a Cast retrieving a mirrored timer
  // ring or wake chime, and it has no session and never will.
  if (strncmp(url_buf, "/api/sat1/sounds/", 17) == 0)
    return Route::SOUND;
#endif
#ifdef USE_SAT1_CRASH_REPORT
  // All three stay behind the session gate (nothing here joins the gate's exempt list): the dump
  // is raw RAM contents - every task's stack, whatever secrets sat in them - and the log tail is
  // whatever the device said in its last seconds. Longest match first, so the bare /api/sat1/crash
  // never shadows its children.
  if (url == "/api/sat1/crash/log")
    return Route::CRASH_LOG;
  if (url == "/api/sat1/crash/dump.bin")
    return Route::CRASH_DUMP;
  if (url == "/api/sat1/crash")
    return Route::CRASH;
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
#ifdef USE_SAT1_CRASH_REPORT
    // The records JSON accumulates in an AsyncResponseStream like STATE's; a few kilobytes at
    // most, but internal heap all the same.
    case Route::CRASH:
#endif
    case Route::SEL:
      return true;
    case Route::NONE:
    case Route::INDEX:
    // The image is PROGMEM like the bundle, so it too answers without heap - and so are the
    // manifest and the icons.
    case Route::ASSET_NO_SENSOR:
    case Route::MANIFEST:
    case Route::ICON_192:
    case Route::ICON_512:
    case Route::ICON_180:
    case Route::HA:
    case Route::HA_REFRESH:
    case Route::HA_SELECT:
    // The MA payload is served from PSRAM in chunks exactly as HA's is, and the writes answer with
    // string literals - none of the three needs internal heap.
    case Route::MA:
    case Route::MA_REFRESH:
    case Route::MA_SET:
    case Route::SEL_SET:
#if defined(USE_MICRO_WAKE_WORD) && !defined(USE_SAT1_MWW_LOADER)
    case Route::WAKE_WORDS_SET:
#endif
#ifdef USE_SAT1_MWW_LOADER
    // All answer with string literals, like the other writes.
    case Route::WAKE_SLOT_SET:
    case Route::WAKE_CUTOFF_SET:
    case Route::WAKE_TUNE_SET:
#endif
#ifdef USE_MEDIA_PLAYER
    // MEDIA is here as well: its whole body fits a stack snprintf, so unlike the other GETs it
    // needs no AsyncResponseStream and works when the heap could not provide one.
    case Route::MEDIA:
    case Route::MEDIA_SET:
#endif
#ifdef USE_SAT1_WEB_UI_SOUNDS
    // Sounds are PROGMEM sends like the icons, range slices included - the slice is a pointer
    // offset into flash, not a copy.
    case Route::SOUND:
#endif
#ifdef USE_SAT1_CRASH_REPORT
    // The log tail is sent straight from its PSRAM buffer, the dump is chunked from flash through
    // a PSRAM scratch block, and the erase answers with a literal - none touch the internal heap.
    case Route::CRASH_LOG:
    case Route::CRASH_DUMP:
    case Route::CRASH_ERASE:
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
  // Every raw-httpd response a remote page reads must carry this by hand: reaching past
  // init_response_ also reaches past the Access-Control-Allow-Origin: * default web_server_base
  // installs, and without it a peer Satellite1's page remote-controlling this device sees a CORS
  // error - indistinguishable from the device being gone, on exactly the response that says
  // "ask me again in a moment".
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
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
    case Route::ASSET_NO_SENSOR:
      this->handle_no_sensor_(request);
      break;
    case Route::MANIFEST:
      this->handle_pwa_asset_(request, this->manifest_, this->manifest_len_, "application/manifest+json");
      break;
    case Route::ICON_192:
      this->handle_pwa_asset_(request, this->icons_[0], this->icon_lens_[0], "image/png");
      break;
    case Route::ICON_512:
      this->handle_pwa_asset_(request, this->icons_[1], this->icon_lens_[1], "image/png");
      break;
    case Route::ICON_180:
      this->handle_pwa_asset_(request, this->icons_[2], this->icon_lens_[2], "image/png");
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
#ifndef USE_SAT1_MWW_LOADER
    case Route::WAKE_WORDS_SET:
      this->handle_wake_words_set_(request);
      break;
#endif
#endif
#ifdef USE_SAT1_MWW_LOADER
    case Route::WAKE_SLOT_SET:
      this->handle_wake_slot_set_(request);
      break;
    case Route::WAKE_CUTOFF_SET:
      this->handle_wake_cutoff_set_(request);
      break;
    case Route::WAKE_TUNE_SET:
      this->handle_wake_tune_set_(request);
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
    case Route::MA:
      this->handle_ma_(request);
      break;
    case Route::MA_REFRESH:
      this->handle_ma_refresh_(request);
      break;
    case Route::MA_SET:
      this->handle_ma_set_(request);
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
#ifdef USE_SAT1_WEB_UI_SOUNDS
    case Route::SOUND:
      this->handle_sound_(request);
      break;
#endif
#ifdef USE_SAT1_CRASH_REPORT
    case Route::CRASH:
      this->handle_crash_(request);
      break;
    case Route::CRASH_LOG:
      this->handle_crash_log_(request);
      break;
    case Route::CRASH_DUMP:
      this->handle_crash_dump_(request);
      break;
    case Route::CRASH_ERASE:
      this->handle_crash_erase_(request);
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

#if defined(USE_VOICE_ASSISTANT) || defined(USE_MICRO_WAKE_WORD) || defined(USE_SAT1_CRASH_REPORT)
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
void WebUIHandler::push_wake_detection(const std::string &word) {
  LockGuard guard{this->detection_lock_};
  this->detection_seq_++;
  this->detections_.push_back(WakeDetection{word, millis()});
  if (this->detections_.size() > WU_DETECTION_RING)
    this->detections_.erase(this->detections_.begin());
}

#ifdef USE_SAT1_MWW_LOADER
/// The two wake word slots, the built-in models the picker's Included group lists, and the
/// detection ring - one payload for the Wake Words card, the swap-in-progress poll, the "say it
/// now" test moment and the Diagnostics history, because all four are facts about the same thing.
///
/// Shape:
///   {"slots":[{"i":0,"m":"<spec>","w":"<phrase>","st":0,"err":0,"cut":0,"rt":0,"ld":1,"dl":0,"tot":0}],
///    "builtin":[["hey_jarvis","hey jarvis"]],
///    "det":[<seq>,"<phrase>",<ms ago>],
///    "hist":[["<phrase>",<ms ago>]]}
///
/// `m` is the slot's spec (a built-in id or a manifest URL, "" when the slot is silent), `st` and
/// `err` are the loader's SlotState/SlotError numbers, `cut` the persisted sensitivity (0 = model
/// default, else a tuned threshold), `ld` whether a model is actually loaded and listening - the
/// fact the tuner row keys on, since a failed swap leaves `st` on ERROR while the previous word
/// listens on - and `dl`/`tot` download progress on whichever slot is
/// mid-swap. `det` is the
/// newest firing with a monotonically increasing sequence number, so the test moment can tell a
/// fresh detection from the one it already celebrated.
void WebUIHandler::handle_wake_words_(AsyncWebServerRequest *request) {
  if (this->mww_ == nullptr || this->wake_loader_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  mww_runtime_loader::SlotView slots[mww_runtime_loader::WL_SLOTS];
  uint32_t dl = 0, total = 0;
  int dl_slot = -1;
  this->wake_loader_->snapshot(slots, dl, total, dl_slot);

  auto *stream = request->beginResponseStream("application/json");
  stream->print(R"({"slots":[)");
  for (uint8_t i = 0; i < mww_runtime_loader::WL_SLOTS; i++) {
    const auto &s = slots[i];
    if (i != 0)
      stream->print(",");
    stream->printf(R"({"i":%u,"m":)", static_cast<unsigned>(i));
    write_json_string(stream, s.spec);
    stream->print(R"(,"w":)");
    write_json_string(stream, s.word);
    stream->printf(R"(,"st":%u,"err":%u,"cut":%u,"rt":%u,"ld":%u)", s.state, s.error, s.cutoff, s.runtime ? 1 : 0,
                   s.id.empty() ? 0 : 1);
    if (dl_slot == i)
      stream->printf(",\"dl\":%lu,\"tot\":%lu", static_cast<unsigned long>(dl), static_cast<unsigned long>(total));
    stream->print("}");
  }

  stream->print(R"(],"builtin":[)");
  bool first = true;
  for (const auto &b : this->wake_loader_->builtins()) {
    if (!first)
      stream->print(",");
    first = false;
    stream->print("[");
    write_json_string(stream, b.id);
    stream->print(",");
    write_json_string(stream, b.word);
    stream->print("]");
  }
  stream->print("]");

  // The tuner's capability and, while a session is live, its score ring - newest last, as ages, so
  // the panel can both replay what it missed and draw the freshest attempt.
  stream->printf(",\"tcap\":%d", mww_runtime_loader::MwwRuntimeLoader::tune_capable() ? 1 : 0);
  {
    mww_runtime_loader::TuneView tv;
    this->wake_loader_->tune_snapshot(tv);
    if (tv.slot >= 0) {
      const uint32_t now = millis();
      stream->printf(",\"tune\":{\"i\":%d,\"seq\":%lu,\"ev\":[", tv.slot, static_cast<unsigned long>(tv.seq));
      for (size_t k = 0; k < tv.events.size(); k++) {
        const auto &ev = tv.events[k];
        if (k != 0)
          stream->print(",");
        stream->printf("[%u,%u,%u,%lu]", ev.peak, ev.avg, ev.vad_blocked ? 1 : 0,
                       static_cast<unsigned long>(now - ev.at_ms));
      }
      stream->print("]}");
    }
  }

  {
    LockGuard guard{this->detection_lock_};
    const uint32_t now = millis();
    if (!this->detections_.empty()) {
      const auto &last = this->detections_.back();
      stream->printf(",\"det\":[%lu,", static_cast<unsigned long>(this->detection_seq_));
      write_json_string(stream, last.word);
      stream->printf(",%lu]", static_cast<unsigned long>(now - last.at_ms));
    }
    stream->print(R"(,"hist":[)");
    // Newest first, which is the order both readers show them in.
    for (size_t k = 0; k < this->detections_.size(); k++) {
      const auto &d = this->detections_[this->detections_.size() - 1 - k];
      if (k != 0)
        stream->print(",");
      stream->print("[");
      write_json_string(stream, d.word);
      stream->printf(",%lu]", static_cast<unsigned long>(now - d.at_ms));
    }
    stream->print("]");
  }

  stream->print("}");
  stream->addHeader("Cache-Control", CACHE_REVALIDATE);
  request->send(stream);
}

/// Points a slot at a wake word: a built-in id, a manifest URL to download, or "none". Queued for
/// the loader's main-loop pass, like every write in this file; the app polls the GET above while
/// the swap runs.
void WebUIHandler::handle_wake_slot_set_(AsyncWebServerRequest *request) {
  if (this->wake_loader_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  auto *index_param = request->getParam("i");
  auto *model_param = request->getParam("m");
  if (index_param == nullptr || model_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  const std::string &index_text = index_param->value();
  char *end = nullptr;
  const unsigned long parsed = strtoul(index_text.c_str(), &end, 10);
  if (end == index_text.c_str() || *end != '\0' || parsed >= mww_runtime_loader::WL_SLOTS) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  if (!this->wake_loader_->queue_slot(static_cast<uint8_t>(parsed), model_param->value())) {
    // One refusal the caller can act on: everything queue_slot rejects is either malformed (400
    // territory) or names the word the other slot already holds - and the app prevents the latter,
    // so a 400 covers what remains.
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  request->send(200, "application/json", "{\"ok\":1}");
}

/// A slot's sensitivity: 0 hands the model its own tuning back, anything else is a measured
/// threshold from the Wake Word Tuner (quantized, 100-254).
void WebUIHandler::handle_wake_cutoff_set_(AsyncWebServerRequest *request) {
  if (this->wake_loader_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  auto *index_param = request->getParam("i");
  auto *value_param = request->getParam("v");
  if (index_param == nullptr || value_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  char *end = nullptr;
  const unsigned long at = strtoul(index_param->value().c_str(), &end, 10);
  if (end == index_param->value().c_str() || *end != '\0' || at >= mww_runtime_loader::WL_SLOTS) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  end = nullptr;
  const unsigned long value = strtoul(value_param->value().c_str(), &end, 10);
  if (end == value_param->value().c_str() || *end != '\0' ||
      (value != 0 && (value < mww_runtime_loader::WL_TUNED_MIN || value > 254))) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  this->wake_loader_->queue_cutoff(static_cast<uint8_t>(at), static_cast<uint8_t>(value));
  request->send(200, "application/json", "{\"ok\":1}");
}

/// Opens, keeps alive (`on=1`) or closes (`on=0`) a Wake Word Tuner session on slot `i`. The
/// loader owns the probe floor and the score ring; this endpoint additionally holds the wake-test
/// suppression window open for the session's lifetime, so a detection during tuning is recorded
/// (the ring the app polls) but starts no assistant and plays no chime. The window clears when the
/// session closes; if the browser vanishes, the loader's own keepalive expiry restores the cutoff
/// and the window lapses ~10s later.
///
/// The reply carries `cap`: whether this build can score attempts at all (the log-listener channel
/// needs DEBUG compiled into the logger). The panel shows an honest "cannot score on this build"
/// instead of an empty meter when it is 0.
void WebUIHandler::handle_wake_tune_set_(AsyncWebServerRequest *request) {
  if (this->wake_loader_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  auto *index_param = request->getParam("i");
  auto *on_param = request->getParam("on");
  if (index_param == nullptr || on_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  char *end = nullptr;
  const unsigned long at = strtoul(index_param->value().c_str(), &end, 10);
  if (end == index_param->value().c_str() || *end != '\0' || at >= mww_runtime_loader::WL_SLOTS) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  const bool on = on_param->value() == "1" || on_param->value() == "true";
  this->wake_loader_->queue_tune(static_cast<uint8_t>(at), on);
  if (on) {
    this->wake_test_until_.store(millis_64() + mww_runtime_loader::WL_TUNE_TTL_MS + 10000, std::memory_order_relaxed);
  } else {
    this->wake_test_until_.store(0, std::memory_order_relaxed);
  }
  request->send(200, "application/json",
                mww_runtime_loader::MwwRuntimeLoader::tune_capable() ? "{\"ok\":1,\"cap\":1}" : "{\"ok\":1,\"cap\":0}");
}

#else   // USE_SAT1_MWW_LOADER

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
#endif  // !USE_SAT1_MWW_LOADER

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
  const auto ss =
      this->media_sendspin_ != nullptr ? this->media_sendspin_->state : MediaPlayerState::MEDIA_PLAYER_STATE_NONE;
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
                     active == this->media_sendspin_ ? "sendspin" : "local", state, volume, active->is_muted() ? 1 : 0);

  // The group player rides along whenever it exists, under its own keys. The card needs it to keep a
  // paused group stream operable: the Sendspin protocol has no paused state, so a paused group is an
  // idle player here and `src` above says "local" - and the volume the paused-group card shows has to
  // come from somewhere other than the player the device thinks is active.
  if (len > 0 && static_cast<size_t>(len) < sizeof(body) && this->media_sendspin_ != nullptr) {
    len += snprintf(body + len, sizeof(body) - static_cast<size_t>(len), R"(,"ss_state":%d,"ss_volume":%d)",
                    static_cast<int>(this->media_sendspin_->state),
                    static_cast<int>(std::roundf(this->media_sendspin_->volume * 100.0f)));
  }
  if (len <= 0 || static_cast<size_t>(len) + 1 >= sizeof(body)) {
    request->send(500, "application/json", "{\"ok\":0}");
    return;
  }

#ifdef USE_SAT1_WEB_UI_SENDSPIN
  // The Sendspin extras: track timing, controller state and the pre-escaped metadata fragment. The
  // numbers fit beside the core on the stack; the fragment does not - a title, artist, album and
  // artwork URL can run several hundred bytes - so the full body is assembled in a persistent PSRAM
  // buffer instead. Failing to grow that buffer degrades to the core body rather than to an error,
  // because transport and volume working without metadata beats neither working.
  char extras[128];
  int elen = 0;
  const uint32_t dur = this->media_dur_ms_.load(std::memory_order_relaxed);
  if (dur > 0) {
    elen += snprintf(extras + elen, sizeof(extras) - static_cast<size_t>(elen), R"(,"pos":%u,"dur":%u)",
                     static_cast<unsigned>(this->media_pos_ms_.load(std::memory_order_relaxed)),
                     static_cast<unsigned>(dur));
  }
  if (this->media_ctrl_ok_.load(std::memory_order_acquire) != 0 && elen >= 0 &&
      static_cast<size_t>(elen) < sizeof(extras)) {
    elen += snprintf(extras + elen, sizeof(extras) - static_cast<size_t>(elen),
                     R"(,"shuffle":%d,"repeat":%d,"sup":%u,"seek_max":%u)",
                     this->media_shuffle_.load(std::memory_order_relaxed) ? 1 : 0,
                     static_cast<int>(this->media_repeat_.load(std::memory_order_relaxed)),
                     static_cast<unsigned>(this->media_sup_.load(std::memory_order_relaxed)),
                     static_cast<unsigned>(this->media_seek_max_.load(std::memory_order_relaxed)));
  }
  if (elen < 0 || static_cast<size_t>(elen) >= sizeof(extras))
    elen = 0;

  // The fragment is copied into its final position under the lock, and the send happens outside it:
  // the writer is the main loop, and making a metadata update wait on a socket write would be a
  // priority inversion against exactly the loop the ground rules protect.
  size_t flen = 0;
  {
    LockGuard guard(this->media_meta_lock_);
    flen = this->media_meta_json_.size();
    if (elen > 0 || flen > 0) {
      const size_t need = static_cast<size_t>(len) + static_cast<size_t>(elen) + flen + 2;
      if (need > this->media_body_cap_) {
        char *grown = this->media_body_ == nullptr ? this->ha_alloc_.allocate(need)
                                                   : this->ha_alloc_.reallocate(this->media_body_, need);
        if (grown != nullptr) {
          this->media_body_ = grown;
          this->media_body_cap_ = need;
        }
      }
      if (this->media_body_ != nullptr && this->media_body_cap_ >= need) {
        memcpy(this->media_body_ + len + elen, this->media_meta_json_.data(), flen);
      } else {
        flen = 0;
        elen = 0;  // No buffer to carry the extras either; the stack core below still answers.
      }
    }
  }
  if (elen > 0 || flen > 0) {
    memcpy(this->media_body_, body, static_cast<size_t>(len));
    memcpy(this->media_body_ + len, extras, static_cast<size_t>(elen));
    char *at = this->media_body_ + len + elen + flen;
    *at++ = '}';
    *at = '\0';
    request->send(200, "application/json", this->media_body_);
    return;
  }
#endif

  body[len++] = '}';
  body[len] = '\0';
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
  } else if (strcmp(cmd, "shuffle") == 0) {
    // The value travels as v=0|1 rather than as two verbs, so the footer's toggle posts what it
    // wants to be true and never has to know what the state was a network round trip ago.
    auto *v = request->getParam("v");
    if (v == nullptr || (v->value() != "0" && v->value() != "1")) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    wanted = v->value() == "1" ? MediaCmd::SHUFFLE_ON : MediaCmd::SHUFFLE_OFF;
    src = 1;  // Queue semantics are the group stream's alone; see apply_media_requests.
  } else if (strcmp(cmd, "repeat") == 0) {
    auto *m = request->getParam("m");
    if (m == nullptr) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    if (m->value() == "off") {
      wanted = MediaCmd::REPEAT_OFF;
    } else if (m->value() == "one") {
      wanted = MediaCmd::REPEAT_ONE;
    } else if (m->value() == "all") {
      wanted = MediaCmd::REPEAT_ALL;
    } else {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    src = 1;
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
    // Shuffle and repeat are the group stream's alone, for the same reason the skips are: the local
    // player has no queue for either to describe. The endpoint already forces src=sendspin for
    // them, so this gate only fires if a build without the group player somehow queued one.
    case MediaCmd::SHUFFLE_ON:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_SHUFFLE).perform();
      break;
    case MediaCmd::SHUFFLE_OFF:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_UNSHUFFLE).perform();
      break;
    case MediaCmd::REPEAT_OFF:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_REPEAT_OFF).perform();
      break;
    case MediaCmd::REPEAT_ONE:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_REPEAT_ONE).perform();
      break;
    case MediaCmd::REPEAT_ALL:
      if (target == this->media_sendspin_)
        target->make_call().set_command(media_player::MEDIA_PLAYER_COMMAND_REPEAT_ALL).perform();
      break;
    case MediaCmd::NONE:
      break;
  }
}

#ifdef USE_SAT1_WEB_UI_SENDSPIN
/// Appends `value` to `out` as JSON string content, escaping the two characters that can break the
/// document (quote and backslash) and the control range, and capping the contribution at `max`
/// bytes. The cap backs off UTF-8 continuation bytes so a truncated title ends on a whole character
/// rather than half of one. Metadata is the server's text, so the cap is politeness, not security -
/// the escape is what carries the correctness.
static void append_json_escaped_(std::basic_string<char, std::char_traits<char>, RAMAllocator<char>> &out,
                                 const char *value, size_t max) {
  size_t len = strlen(value);
  if (len > max) {
    len = max;
    while (len > 0 && (static_cast<uint8_t>(value[len]) & 0xC0) == 0x80)
      len--;
  }
  for (size_t i = 0; i < len; i++) {
    const char c = value[i];
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(c);
    } else if (static_cast<uint8_t>(c) < 0x20) {
      // Control characters are illegal raw inside a JSON string. Newlines and tabs have no business
      // in a track title, so a space keeps the document valid without a six-byte \u escape.
      out.push_back(' ');
    } else {
      out.push_back(c);
    }
  }
}

void WebUIHandler::media_set_meta(const char *title, const char *artist, const char *album, const char *art,
                                  uint32_t dur_ms) {
  // Built into a fresh string and swapped under the lock, so a poll on the httpd task never reads a
  // fragment mid-assembly. The allocator matches the member's, which is what makes swap legal.
  std::basic_string<char, std::char_traits<char>, RAMAllocator<char>> next;

  struct Field {
    const char *key;
    const char *value;
    size_t max;
  };
  // The URL's cap is the generous one: MA's image proxy URLs run ~120 bytes, but a CDN URL for a
  // streaming provider can be longer, and a truncated URL is worthless where a truncated title is
  // merely short.
  const Field fields[] = {
      {"\"title\":\"", title, 160},
      {"\"artist\":\"", artist, 160},
      {"\"album\":\"", album, 160},
      {"\"art\":\"", art, 320},
  };
  for (const auto &f : fields) {
    if (f.value == nullptr || f.value[0] == '\0')
      continue;
    next.push_back(',');
    next.append(f.key);
    append_json_escaped_(next, f.value, f.max);
    next.push_back('"');
  }

  this->media_dur_ms_.store(dur_ms, std::memory_order_relaxed);
  LockGuard guard(this->media_meta_lock_);
  this->media_meta_json_.swap(next);
}

void WebUIHandler::media_set_ctrl(bool shuffle, uint8_t repeat, uint16_t supported, uint32_t seek_max_ms) {
  this->media_shuffle_.store(shuffle ? 1 : 0, std::memory_order_relaxed);
  this->media_repeat_.store(repeat, std::memory_order_relaxed);
  this->media_sup_.store(supported, std::memory_order_relaxed);
  this->media_seek_max_.store(seek_max_ms, std::memory_order_relaxed);
  // Last, so a poll that sees the flag sees the fields behind it.
  this->media_ctrl_ok_.store(1, std::memory_order_release);
}

void WebUIHandler::media_clear_ctrl() {
  this->media_ctrl_ok_.store(0, std::memory_order_relaxed);
  this->media_pos_ms_.store(0, std::memory_order_relaxed);
}
#endif  // USE_SAT1_WEB_UI_SENDSPIN
#endif  // USE_MEDIA_PLAYER

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
  // `actions` rides both shapes: the checkbox verdict matters most on exactly the loads where the
  // payload is null, because a blocked device has never fetched one.
  if (this->ha_at_ == 0) {
    char empty[64];
    snprintf(empty, sizeof(empty), R"({"rung":%d,"actions":%d,"age":-1,"d":null})", this->ha_rung_,
             this->ha_actions_.load());
    request->send(200, "application/json", empty);
    return;
  }

  httpd_resp_set_type(*request, "application/json");
  httpd_resp_set_hdr(*request, "Cache-Control", CACHE_REVALIDATE);
  // By hand on this raw-httpd path - see send_low_memory_. Found live: everything else on the
  // remote-control path answered and the app still sat on "Connecting", because this one payload
  // is what the splash's verdict waits for.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");

  char head[64];
  const int head_len =
      snprintf(head, sizeof(head), R"({"rung":%d,"actions":%d,"age":%u,"d":)", this->ha_rung_, this->ha_actions_.load(),
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

/* --- The Music Assistant relay ------------------------------------------------------------------
 *
 * The footer's tier-1 surface: what must round-trip through Home Assistant because neither the
 * device nor the browser can do it alone. The payload (GET /api/sat1/ma) is this device's MA
 * player's group members with their volumes plus shuffle/repeat as Home Assistant sees them; the
 * discovery facts that change rarely - which entity is this device's MA twin, its favorite button,
 * and the groupable candidates - ride the big Home Assistant payload instead, where the paging and
 * size-cap machinery already live. Commands (POST /api/sat1/ma/<cmd>) queue here and leave as
 * homeassistant.action calls from the main loop, exactly like the select writes.
 */

char *WebUIHandler::stage_ma_payload(size_t capacity) {
  // Main loop only, like the HA stage; what requests read is ma_buf_, and the two only meet in the
  // commit below.
  if (capacity > this->ma_stage_cap_) {
    char *grown = this->ma_stage_ == nullptr ? this->ha_alloc_.allocate(capacity)
                                             : this->ha_alloc_.reallocate(this->ma_stage_, capacity);
    if (grown == nullptr) {
      ESP_LOGW(TAG_WU, "No room to stage a %u byte Music Assistant payload; keeping the previous one",
               static_cast<unsigned>(capacity));
      return nullptr;
    }
    this->ma_stage_ = grown;
    this->ma_stage_cap_ = capacity;
  }
  return this->ma_stage_;
}

void WebUIHandler::commit_ma_payload(size_t len) {
  if (this->ma_stage_ == nullptr || len + 1 > this->ma_stage_cap_)
    return;
  this->ma_stage_[len] = '\0';

  LockGuard guard{this->ma_lock_};
  char *served = this->ma_buf_;
  const size_t served_cap = this->ma_cap_;
  this->ma_buf_ = this->ma_stage_;
  this->ma_len_ = len;
  this->ma_cap_ = this->ma_stage_cap_;
  this->ma_stage_ = served;
  this->ma_stage_cap_ = served_cap;
  this->ma_at_ = static_cast<uint32_t>(millis_64() / 1000);
}

void WebUIHandler::handle_ma_(AsyncWebServerRequest *request) {
  // Held across the send for the reason handle_ha_ gives; this payload is far smaller.
  LockGuard guard{this->ma_lock_};

  if (this->ma_at_ == 0) {
    request->send(200, "application/json", R"({"age":-1,"d":null})");
    return;
  }

  httpd_resp_set_type(*request, "application/json");
  httpd_resp_set_hdr(*request, "Cache-Control", CACHE_REVALIDATE);
  // By hand on this raw-httpd path - see send_low_memory_.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");

  char head[32];
  const int head_len = snprintf(head, sizeof(head), R"({"age":%u,"d":)",
                                static_cast<unsigned int>(static_cast<uint32_t>(millis_64() / 1000) - this->ma_at_));
  httpd_resp_send_chunk(*request, head, head_len);
  httpd_resp_send_chunk(*request, this->ma_buf_, static_cast<ssize_t>(this->ma_len_));
  httpd_resp_send_chunk(*request, "}", 1);
  httpd_resp_send_chunk(*request, nullptr, 0);
}

void WebUIHandler::handle_ma_refresh_(AsyncWebServerRequest *request) {
  // Only a request, like the HA refresh: the sync is an action call and belongs on the main loop.
  this->ma_refresh_requested_.store(true);
  request->send(200, "application/json", "{\"queued\":1}");
}

bool WebUIHandler::ma_payload_names_(const std::string &entity) {
  const std::string quoted = "\"" + entity + "\"";

  LockGuard guard{this->ma_lock_};
  if (this->ma_buf_ == nullptr || this->ma_len_ == 0)
    return false;
  return strstr(this->ma_buf_, quoted.c_str()) != nullptr;
}

/// True when `value` is shaped like an entity id in `domain` - "media_player.kitchen" - with nothing
/// in it that has any business in one. The character walk matters less as validation (Home Assistant
/// would refuse a malformed id anyway) than as the same bug-containment the select endpoint's checks
/// buy: what leaves here goes into an action call as a template variable.
static bool ma_entity_ok_(const std::string &value, const char *domain) {
  if (value.size() > 120 || value.rfind(domain, 0) != 0)
    return false;
  for (const char c : value) {
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '.'))
      return false;
  }
  return true;
}

/// Queues one relayed Music Assistant command.
///
/// The same two guards as the select endpoint, for the same reason: the domain each verb needs is
/// enforced, and the entity must appear in a payload this device itself rendered - the big Home
/// Assistant one (which carries the MA twin, the favorite button and every groupable candidate) or
/// the members one above. This is not the security boundary (the caller is already authenticated);
/// it is what keeps a bug in the app from aiming action calls at arbitrary entities.
void WebUIHandler::handle_ma_set_(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  request->url_to(url_buf);
  const char *cmd = url_buf + 13;  // Past "/api/sat1/ma/", which match_route_ has already tested.

  auto *entity_param = request->getParam("e");
  if (entity_param == nullptr) {
    request->send(400, "application/json", "{\"ok\":0}");
    return;
  }
  const std::string &entity = entity_param->value();

  MaCmd kind = MaCmd::NONE;
  std::string arg;

  if (strcmp(cmd, "like") == 0) {
    // The favorite is a button entity the MA integration creates beside the player - pressing it is
    // how "like this track" works, and the only way: there is no action that takes a media item.
    if (!ma_entity_ok_(entity, "button.")) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    kind = MaCmd::LIKE;
  } else if (strcmp(cmd, "join") == 0 || strcmp(cmd, "unjoin") == 0 || strcmp(cmd, "vol") == 0 ||
             strcmp(cmd, "seek") == 0) {
    if (!ma_entity_ok_(entity, "media_player.")) {
      request->send(400, "application/json", "{\"ok\":0}");
      return;
    }
    if (strcmp(cmd, "join") == 0) {
      // One member per request - the sheet adds speakers one tap at a time - so `m` is a single
      // entity id held to the same shape as `e`, not a list to parse.
      auto *member = request->getParam("m");
      if (member == nullptr || !ma_entity_ok_(member->value(), "media_player.") ||
          !this->ha_payload_names_(member->value())) {
        request->send(400, "application/json", "{\"ok\":0}");
        return;
      }
      kind = MaCmd::JOIN;
      arg = member->value();
    } else if (strcmp(cmd, "unjoin") == 0) {
      kind = MaCmd::UNJOIN;
    } else if (strcmp(cmd, "vol") == 0) {
      auto *v = request->getParam("v");
      if (v == nullptr) {
        request->send(400, "application/json", "{\"ok\":0}");
        return;
      }
      const std::string &text = v->value();
      char *end = nullptr;
      const unsigned long parsed = strtoul(text.c_str(), &end, 10);
      if (end == text.c_str() || *end != '\0' || parsed > 100) {
        request->send(400, "application/json", "{\"ok\":0}");
        return;
      }
      kind = MaCmd::VOL;
      arg = text;
    } else {
      // Seek, in whole seconds. The scrubber's step is a second, and sub-second precision over a
      // relay with this much latency would be theatre.
      auto *t = request->getParam("t");
      if (t == nullptr) {
        request->send(400, "application/json", "{\"ok\":0}");
        return;
      }
      const std::string &text = t->value();
      char *end = nullptr;
      const unsigned long parsed = strtoul(text.c_str(), &end, 10);
      if (end == text.c_str() || *end != '\0' || parsed > 24UL * 3600UL) {
        request->send(400, "application/json", "{\"ok\":0}");
        return;
      }
      kind = MaCmd::SEEK;
      arg = text;
    }
  }

  if (kind == MaCmd::NONE) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  if (!this->ha_payload_names_(entity) && !this->ma_payload_names_(entity)) {
    // 404 for the reason the select endpoint gives: well formed, but naming something this device
    // has not been told about, which a stale browser does after a rename. Refetching is the answer.
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  {
    LockGuard guard{this->ma_queue_lock_};

    bool queued = false;
    for (auto &pending : this->ma_queue_) {
      if (pending.kind == static_cast<uint8_t>(kind) && pending.entity == entity) {
        // Last write wins per kind and entity: a member-volume drag or a scrub ends as one call.
        pending.arg = arg;
        queued = true;
        break;
      }
    }

    if (!queued) {
      if (this->ma_queue_.size() >= WU_MA_QUEUE) {
        request->send(409, "application/json", "{\"ok\":0}");
        return;
      }
      this->ma_queue_.push_back({static_cast<uint8_t>(kind), entity, arg});
    }

    this->ma_pending_.store(true, std::memory_order_release);
  }

  // Queued, not done. The app resyncs after the round trip and believes what comes back - none of
  // these calls captures a response, so there is nothing else to believe.
  request->send(200, "application/json", "{\"queued\":1}");
}

bool WebUIHandler::take_ma_write(MaWrite &out) {
  if (!this->ma_pending_.load(std::memory_order_acquire))
    return false;

  LockGuard guard{this->ma_queue_lock_};
  if (this->ma_queue_.empty()) {
    this->ma_pending_.store(false, std::memory_order_relaxed);
    return false;
  }

  out = this->ma_queue_.front();
  this->ma_queue_.erase(this->ma_queue_.begin());
  if (this->ma_queue_.empty())
    this->ma_pending_.store(false, std::memory_order_relaxed);
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

/// The no-sensor card's product photo, straight from PROGMEM. No Content-Encoding header - WebP is
/// already compressed and ships uncompressed-by-gzip - and a plain max-age rather than the bundle's
/// revalidate dance: the image changes when the firmware does, at which point a day-old cache entry
/// is a cosmetic staleness on a card most devices never render, not a stale application.
void WebUIHandler::handle_no_sensor_(AsyncWebServerRequest *request) {
  if (this->no_sensor_webp_ == nullptr || this->no_sensor_webp_len_ == 0) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  auto *response = request->beginResponse(200, "image/webp", this->no_sensor_webp_, this->no_sensor_webp_len_);
  response->addHeader("Cache-Control", "max-age=86400");
  request->send(response);
}

/// The manifest and the icons, the photo's exact recipe: PROGMEM, a day of cache, and no gzip layer
/// (PNG is already compressed; the manifest is a few hundred bytes and not worth a second ETag
/// dance).
void WebUIHandler::handle_pwa_asset_(AsyncWebServerRequest *request, const uint8_t *data, size_t len,
                                     const char *type) {
  if (data == nullptr || len == 0) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  auto *response = request->beginResponse(200, type, data, len);
  response->addHeader("Cache-Control", "max-age=86400");
  request->send(response);
}

#ifdef USE_SAT1_WEB_UI_SOUNDS

/// The sounds the routing feature hands out URLs for - the mirrored timer ring and the
/// non-Satellite1 wake chime - served straight from the audio_file bytes already in flash, so the
/// device carries exactly one copy of each and remote playback needs no internet.
///
/// The Range handling is not optional polish. Cast refuses media whose origin cannot answer a Range
/// request, and Sonos may probe with `bytes=0-` before fetching - and ESPHome's response API cannot
/// express a 206, so this reaches past it to the raw httpd calls the same way handle_index_ does for
/// its 304. Single ranges only; a multipart range asks for more ceremony than any speaker sends.
/// The Content-Range buffer lives on this stack frame, which outlives the send - httpd_resp_set_hdr
/// stores the pointer rather than copying.
void WebUIHandler::handle_sound_(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  request->url_to(url_buf);
  const char *name = url_buf + 17;  // strlen("/api/sat1/sounds/"), guaranteed by match_route_

  const audio::AudioFile *file = nullptr;
  for (const auto &sound : this->sounds_) {
    if (strcmp(sound.name, name) == 0) {
      file = sound.file;
      break;
    }
  }
  if (file == nullptr || file->data == nullptr || file->length == 0) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  // From the embedded bytes rather than the URL's extension, so the header cannot lie about the
  // payload. The enum's members only exist for compiled-in codecs, hence the guards.
  const char *type = "application/octet-stream";
  switch (file->file_type) {
#ifdef USE_AUDIO_MP3_SUPPORT
    case audio::AudioFileType::MP3:
      type = "audio/mpeg";
      break;
#endif
#ifdef USE_AUDIO_WAV_SUPPORT
    case audio::AudioFileType::WAV:
      type = "audio/wav";
      break;
#endif
#ifdef USE_AUDIO_FLAC_SUPPORT
    case audio::AudioFileType::FLAC:
      type = "audio/flac";
      break;
#endif
    default:
      break;
  }

  const auto range = request->get_header("Range");
  if (range.has_value()) {
    const char *spec = strstr(range.value().c_str(), "bytes=");
    if (spec != nullptr) {
      spec += 6;
      size_t start = 0;
      size_t end = file->length - 1;
      bool valid = false;
      if (*spec == '-') {
        // Suffix form: the last N bytes.
        char *endp = nullptr;
        unsigned long n = strtoul(spec + 1, &endp, 10);
        if (endp != spec + 1 && n > 0) {
          start = n >= file->length ? 0 : file->length - n;
          valid = true;
        }
      } else {
        char *endp = nullptr;
        unsigned long s = strtoul(spec, &endp, 10);
        if (endp != spec && *endp == '-' && s < file->length) {
          start = s;
          if (*(endp + 1) != '\0') {
            unsigned long e = strtoul(endp + 1, nullptr, 10);
            if (e >= s)
              end = e < file->length ? e : file->length - 1;
          }
          valid = true;
        }
      }
      if (!valid) {
        // A range that names nothing inside the file. 416 with the length, per the RFC, so the
        // fetcher can retry whole.
        char content_range[48];
        snprintf(content_range, sizeof(content_range), "bytes */%u", static_cast<unsigned>(file->length));
        httpd_resp_set_status(*request, "416 Range Not Satisfiable");
        httpd_resp_set_hdr(*request, "Content-Range", content_range);
        httpd_resp_send(*request, nullptr, 0);
        return;
      }
      char content_range[64];
      snprintf(content_range, sizeof(content_range), "bytes %u-%u/%u", static_cast<unsigned>(start),
               static_cast<unsigned>(end), static_cast<unsigned>(file->length));
      httpd_resp_set_status(*request, "206 Partial Content");
      httpd_resp_set_type(*request, type);
      httpd_resp_set_hdr(*request, "Content-Range", content_range);
      httpd_resp_set_hdr(*request, "Accept-Ranges", "bytes");
      httpd_resp_send(*request, reinterpret_cast<const char *>(file->data) + start, end - start + 1);
      return;
    }
  }

  auto *response = request->beginResponse(200, type, file->data, file->length);
  // The bytes only change with a reflash, exactly like the icons.
  response->addHeader("Cache-Control", "max-age=86400");
  response->addHeader("Accept-Ranges", "bytes");
  request->send(response);
}

#endif  // USE_SAT1_WEB_UI_SOUNDS

#ifdef USE_SAT1_CRASH_REPORT

/// The crash history: boot counter, dump presence, and the stored records newest-first.
///
/// Shape:
///   {"boot":N,"part":1,"dump":45056,"log":4096,"records":[
///     {"boot":12,"epoch":1789000000,"up":3642,"reason":4,"rs":"Panic or exception",
///      "task":"loopTask","cause":29,"vaddr":"0x0000000c","pc":"0x42012abc","cor":0,
///      "txt":"StoreProhibited","bt":["0x42012abc","0x420071f2"]},...]}
///
/// `epoch` 0 means the flight recorder never knew wall time that session (or RTC memory did not
/// survive); the app words the moment from `up` and the boot distance instead. PCs travel as hex
/// strings because that is what espcoredump and addr2line take, and a copy-paste that needs no
/// reformatting is the whole point of showing them.
void WebUIHandler::handle_crash_(AsyncWebServerRequest *request) {
  if (this->crash_report_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  size_t tail_len = 0;
  this->crash_report_->log_tail(tail_len);

  auto *stream = request->beginResponseStream("application/json");
  stream->printf(R"({"boot":%lu,"part":%d,"dump":%u,"log":%u,"records":[)",
                 static_cast<unsigned long>(this->crash_report_->boot_count()),
                 this->crash_report_->partition_present() ? 1 : 0,
                 static_cast<unsigned>(this->crash_report_->dump_size()), static_cast<unsigned>(tail_len));

  // Newest first, the order the card shows them in. The count is re-read per index through the
  // record lock, so an erase racing this loop ends the list early rather than serving garbage.
  const uint8_t n = this->crash_report_->record_count();
  bool first = true;
  for (uint8_t k = 0; k < n; k++) {
    crash_report::CrashRecord rec;
    if (!this->crash_report_->get_record(n - 1 - k, rec))
      break;
    if (!first)
      stream->print(",");
    first = false;

    stream->printf(R"({"boot":%lu,"epoch":%lu,"up":%lu,"reason":%u,"rs":"%s")", static_cast<unsigned long>(rec.boot),
                   static_cast<unsigned long>(rec.epoch), static_cast<unsigned long>(rec.uptime_s), rec.reason,
                   reset_reason_name_(static_cast<esp_reset_reason_t>(rec.reason)));

    if (rec.flags & crash_report::CR_F_SUMMARY) {
      stream->print(R"(,"task":)");
      write_json_string(stream, rec.task);
      stream->printf(R"(,"cause":%lu,"vaddr":"0x%08lx","pc":"0x%08lx","cor":%d)",
                     static_cast<unsigned long>(rec.exc_cause), static_cast<unsigned long>(rec.exc_vaddr),
                     static_cast<unsigned long>(rec.pc), (rec.flags & crash_report::CR_F_BT_CORRUPT) ? 1 : 0);
      stream->print(R"(,"bt":[)");
      for (uint8_t b = 0; b < rec.bt_depth && b < 16; b++)
        stream->printf(R"(%s"0x%08lx")", b == 0 ? "" : ",", static_cast<unsigned long>(rec.bt[b]));
      stream->print("]");
    }
    if (rec.text[0] != '\0') {
      stream->print(R"(,"txt":)");
      write_json_string(stream, rec.text);
    }
    stream->print("}");
  }

  stream->print("]}");
  stream->addHeader("Cache-Control", CACHE_REVALIDATE);
  request->send(stream);
}

/// The pre-crash log tail as it survived, text/plain. Its own route rather than a field of the
/// JSON above because escaping 4KB of log into a JSON string would double it through an escape
/// walk on every poll, and the tail is immutable after setup - the raw send costs nothing.
void WebUIHandler::handle_crash_log_(AsyncWebServerRequest *request) {
  if (this->crash_report_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  size_t len = 0;
  const char *tail = this->crash_report_->log_tail(len);

  httpd_resp_set_type(*request, "text/plain");
  httpd_resp_set_hdr(*request, "Cache-Control", CACHE_REVALIDATE);
  // By hand on this raw-httpd path - see send_low_memory_.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(*request, len == 0 ? "" : tail, static_cast<ssize_t>(len));
}

/// The core dump image - the coredump partition's own framing (header + ELF + checksum), raw. The
/// decode is `espcoredump.py info_corefile -t raw -c dump.bin firmware.elf` against the exact ELF
/// of this build. Chunked from flash through one small PSRAM block; the httpd task's 4352-byte
/// stack never holds any of it.
///
/// Served as the bare .bin on purpose, after a zip-wrapper detour taught the real lesson: the
/// browser interstitial that motivated the zip ("this file may have been tampered with") keys on
/// the download arriving over plain HTTP, not on the extension, so the wrapper solved nothing.
/// What solved it is the card fetching the bytes and saving them as an in-page Blob - see
/// downloadDump in the frontend - which makes the extension a matter of taste, and the honest
/// name for these bytes is .bin.
void WebUIHandler::handle_crash_dump_(AsyncWebServerRequest *request) {
  const size_t size = this->crash_report_ == nullptr ? 0 : this->crash_report_->dump_size();
  const esp_partition_t *part =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
  if (size == 0 || part == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }

  RAMAllocator<char> alloc(RAMAllocator<char>::ALLOC_EXTERNAL);
  constexpr size_t CHUNK = 4096;
  char *buf = alloc.allocate(CHUNK);
  if (buf == nullptr) {
    this->send_low_memory_(request);
    return;
  }

  // The filename carries the device name so a support inbox with three dumps can tell them apart.
  // Assembled on this frame, which outlives the sends - httpd_resp_set_hdr stores the pointer.
  // The card's Blob path names its own save and ignores this; curl -OJ honours it.
  char disposition[96];
  snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s-coredump.bin\"", App.get_name().c_str());

  httpd_resp_set_type(*request, "application/octet-stream");
  httpd_resp_set_hdr(*request, "Content-Disposition", disposition);
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");

  bool ok = true;
  for (size_t at = 0; at < size && ok; at += CHUNK) {
    const size_t n = size - at < CHUNK ? size - at : CHUNK;
    ok = esp_partition_read(part, at, buf, n) == ESP_OK &&
         httpd_resp_send_chunk(*request, buf, static_cast<ssize_t>(n)) == ESP_OK;
  }
  if (ok)
    httpd_resp_send_chunk(*request, nullptr, 0);
  alloc.deallocate(buf, CHUNK);
}

void WebUIHandler::handle_crash_erase_(AsyncWebServerRequest *request) {
  if (this->crash_report_ == nullptr) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  this->crash_report_->erase_all();
  request->send(200, "application/json", "{\"ok\":1}");
}

#endif  // USE_SAT1_CRASH_REPORT

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

  // The remote-control contract version, read by a peer Satellite1's page before it retargets
  // itself at this device (single-origin device switching). Bump it only for a breaking change to
  // the /api/sat1/* shapes, the entity key names, or the auth story; additive changes ride the
  // same number. A peer whose app finds this missing or out of range falls back to plain
  // navigation, so an old device is never driven by an app that misunderstands it.
  stream->printf(R"({"apiv":%d,)", WU_API_VERSION);
  stream->printf(R"("name":"%s","friendly_name":"%s","mac":"%s","ip":"%s",)", App.get_name().c_str(),
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

#ifdef USE_SAT1_CRASH_REPORT
  // How many crash records /api/sat1/crash holds, so the card knows to fetch without a poll of its
  // own - this endpoint is already the page's heartbeat.
  stream->printf(R"("crash":%u,)",
                 this->crash_report_ == nullptr ? 0 : static_cast<unsigned>(this->crash_report_->record_count()));
#endif

  // Whether Home Assistant is actually attached, so the UI can say so rather than leaving someone to
  // work out why their media controls are missing. api_connection_count_ != 0, which is the same
  // condition the LED ring already uses to decide whether the assistant can run at all.
#ifdef USE_API
  stream->printf(R"("ha":%s,)",
                 api::global_api_server != nullptr && api::global_api_server->is_connected() ? "true" : "false");
#else
  stream->print(R"("ha":false,)");
#endif

  // The sign-in key, for the Diagnostics Launch section's link and QR code. Safe to carry here
  // because this endpoint sits behind the session gate: whoever can read it already holds a
  // session minted from the same value. Re-read per request so a regenerate shows up on the next
  // poll rather than the next reboot.
  if (this->session_key_fn_) {
    const char *key = this->session_key_fn_();
    if (key != nullptr && key[0] != '\0')
      stream->printf(R"("key":"%s",)", key);
  }

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
