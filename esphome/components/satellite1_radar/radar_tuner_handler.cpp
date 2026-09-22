#include "radar_tuner_handler.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <cJSON.h>
#include <esp_heap_caps.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG_RT = "radar_tuner";

namespace {

/// Longest single fragment any printf below produces, rounded up. The LD2450 config header is the
/// one that sets it, at about 130 bytes with every field at its maximum width.
constexpr size_t RT_CHUNK_HEADROOM = 160;

/// A fixed stack buffer that empties itself into the response as chunked-encoding pieces.
///
/// Replaces the AsyncResponseStream these four endpoints used to build their bodies in. That is a
/// std::string, so it doubles on the internal heap as it grows, and both /live endpoints repeat the
/// exercise four times a second for as long as the Presence tab is open. Nothing here allocates.
///
/// The same shape WebUIHandler::handle_ha_ uses, and legal for the same reason: the request's
/// operator httpd_req_t*() hands back the raw handle. ESPHome's response API is skipped because a
/// chunked body has no Content-Length and nothing in AsyncWebServerResponse can express one.
///
/// 512 bytes so the largest body leaves after two flushes and the rest after one. Kept small rather
/// than sized past the largest response because the httpd task stack is 4352 bytes and not tunable -
/// AsyncWebServer::begin() hardcodes HTTPD_DEFAULT_CONFIG().stack_size + 256.
class ChunkWriter {
 public:
  explicit ChunkWriter(httpd_req_t *req) : req_(req) {}

  void print(const char *text) { this->write(text, strlen(text)); }

  void printf(const char *fmt, ...) __attribute__((format(printf, 2, 3))) {
    // Flushed ahead of the format, not after: vsnprintf truncates to the space it is given, so a
    // fragment that did not fit would go out half-written and cut the JSON mid-number.
    if (sizeof(this->buf_) - this->len_ < RT_CHUNK_HEADROOM)
      this->flush_();

    va_list ap;
    va_start(ap, fmt);
    const int written = vsnprintf(this->buf_ + this->len_, sizeof(this->buf_) - this->len_, fmt, ap);
    va_end(ap);
    if (written > 0)
      this->len_ += std::min(static_cast<size_t>(written), sizeof(this->buf_) - this->len_ - 1);
  }

  void write(const char *data, size_t len) {
    while (len > 0) {
      if (this->len_ == sizeof(this->buf_) && !this->flush_())
        return;
      const size_t take = std::min(len, sizeof(this->buf_) - this->len_);
      memcpy(this->buf_ + this->len_, data, take);
      this->len_ += take;
      data += take;
      len -= take;
    }
  }

  /// Sends what is left and closes the chunked response.
  void finish() {
    if (this->flush_())
      httpd_resp_send_chunk(this->req_, nullptr, 0);
  }

 private:
  /// False once a send has failed. esp_http_server has already closed the connection by then, so
  /// there is nobody left to report to and the rest of the body is dropped rather than retried.
  bool flush_() {
    if (!this->ok_ || this->len_ == 0)
      return this->ok_;
    this->ok_ = httpd_resp_send_chunk(this->req_, this->buf_, static_cast<ssize_t>(this->len_)) == ESP_OK;
    this->len_ = 0;
    return this->ok_;
  }

  httpd_req_t *req_;
  size_t len_{0};
  bool ok_{true};
  char buf_[512];
};

}  // namespace

/// Ceiling on an accumulated request body. The largest real payload is an LD2450 zone write -
/// three zones plus an exclusion polygon at eight points each, roughly 700 bytes - so this is
/// generous. It exists because handleBody streams with no cap of its own, and an accumulating
/// string is a heap growth primitive handed to whoever can reach the endpoint.
static constexpr size_t MAX_BODY_BYTES = 4096;

/// Routes cJSON's allocations to PSRAM, installed once before the first parse.
///
/// A config POST's parse tree runs to a few kilobytes of nodes (zone polygons especially), and
/// cJSON's default hooks are plain malloc - the internal heap. The hooks are process-global, and
/// that is deliberate rather than accepted: this handler is the only cJSON user in the firmware
/// image (checked against the build), so nothing else's allocations can be redirected by it. free()
/// on ESP-IDF routes any pointer back to the heap that owns it, so the fallback branch needs no
/// bookkeeping.
static void *cjson_psram_malloc_(size_t size) {
  void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p != nullptr ? p : malloc(size);  // NOLINT(cppcoreguidelines-no-malloc)
}

static void cjson_use_psram_() {
  static bool installed = false;
  if (installed)
    return;
  installed = true;
  cJSON_Hooks hooks{cjson_psram_malloc_, free};
  cJSON_InitHooks(&hooks);
}

static void send_json_(AsyncWebServerRequest *request, const char *body) {
  request->send(200, "application/json", body);
}

static void send_error_(AsyncWebServerRequest *request, int code, const char *message) {
  ESP_LOGW(TAG_RT, "%d %s", code, message);
  request->send(code, "application/json", message);
}

static bool parse_bool_field_(cJSON *root, const char *name, bool &out, bool &present) {
  present = false;
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
  if (item == nullptr)
    return true;
  if (!cJSON_IsBool(item))
    return false;
  out = cJSON_IsTrue(item);
  present = true;
  return true;
}

static bool parse_uint_field_(cJSON *root, const char *name, uint32_t &out, bool &present) {
  present = false;
  cJSON *item = cJSON_GetObjectItemCaseSensitive(root, name);
  if (item == nullptr)
    return true;
  if (!cJSON_IsNumber(item) || item->valuedouble < 0)
    return false;
  out = static_cast<uint32_t>(item->valuedouble);
  present = true;
  return true;
}

void RadarTunerHandler::update_target(int index, float x, float y) {
  if (index >= 0 && index < RT_NUM_TARGETS) {
    targets_[index].x = x;
    targets_[index].y = y;
  }
}

void RadarTunerHandler::clear_registrations() {
  ld2410_ = nullptr;
  ld2450_ = nullptr;
  on_ld2410_apply_ = nullptr;
}

RadarTunerHandler::Route RadarTunerHandler::match_route_(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const StringRef url = request->url_to(url_buf);
  const http_method method = request->method();

  if (method == HTTP_GET) {
    if (url == "/api/v1/ld2410/config")
      return Route::LD2410_CONFIG_GET;
    if (url == "/api/v1/ld2410/live")
      return Route::LD2410_LIVE;
    if (url == "/api/v1/ld2450/config")
      return Route::LD2450_CONFIG_GET;
    if (url == "/api/v1/ld2450/live")
      return Route::LD2450_LIVE;
    return Route::NONE;
  }

  if (method == HTTP_POST) {
    // Was PATCH on the standalone server. web_server_idf registers only GET, POST and OPTIONS
    // wildcard handlers, so a PATCH never reaches any handler at all.
    if (url == "/api/v1/ld2410/config")
      return Route::LD2410_CONFIG_SET;
    if (url == "/api/v1/ld2450/config")
      return Route::LD2450_CONFIG_SET;
    if (url == "/api/v1/ld2410/apply")
      return Route::LD2410_APPLY;
    if (url == "/api/v1/save")
      return Route::SAVE;
    if (url == "/api/v1/reboot")
      return Route::REBOOT;
    return Route::NONE;
  }

  return Route::NONE;
}

bool RadarTunerHandler::canHandle(AsyncWebServerRequest *request) const { return match_route_(request) != Route::NONE; }

void RadarTunerHandler::handleBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index,
                                   size_t total) {
  if (index == 0) {
    body_.clear();
    if (total <= MAX_BODY_BYTES)
      body_.reserve(total);
  }
  // Keep consuming past the ceiling rather than returning early: the chunks arrive whether or not
  // we want them, and handleRequest is what reports the failure.
  if (body_.size() + len <= MAX_BODY_BYTES)
    body_.append(reinterpret_cast<const char *>(data), len);
}

void RadarTunerHandler::handleRequest(AsyncWebServerRequest *request) {
  switch (match_route_(request)) {
    case Route::LD2410_CONFIG_GET:
      this->handle_ld2410_get_config_(request);
      break;
    case Route::LD2410_CONFIG_SET:
      this->handle_ld2410_set_config_(request);
      break;
    case Route::LD2410_APPLY:
      this->handle_ld2410_apply_(request);
      break;
    case Route::LD2410_LIVE:
      this->handle_ld2410_live_(request);
      break;
    case Route::LD2450_CONFIG_GET:
      this->handle_ld2450_get_config_(request);
      break;
    case Route::LD2450_CONFIG_SET:
      this->handle_ld2450_set_config_(request);
      break;
    case Route::LD2450_LIVE:
      this->handle_ld2450_live_(request);
      break;
    case Route::SAVE:
      this->handle_save_(request);
      break;
    case Route::REBOOT:
      this->handle_reboot_(request);
      break;
    case Route::NONE:
      break;
  }
  // Dropped here rather than in handleBody so a parse failure can still report the size it saw.
  this->body_.clear();
  this->body_.shrink_to_fit();
}

void RadarTunerHandler::handle_ld2410_get_config_(AsyncWebServerRequest *request) {
  if (ld2410_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2410 handler unavailable\"}");
    return;
  }

  const auto &cfg = ld2410_->get_backend_config();
  httpd_resp_set_type(*request, "application/json");
  // By hand on every raw-httpd path a peer Satellite1's remote-control page reads: reaching past
  // ESPHome's response API also reaches past the Access-Control-Allow-Origin default it stamps,
  // and without the header the cross-origin probe fails and the page reports no radar module.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
  ChunkWriter out(*request);
  out.printf("{\"timeout\":%u,\"max_move_gate\":%u,\"max_still_gate\":%u,\"distance_resolution\":\"%s\","
             "\"bluetooth\":%s,\"gate_move_thresholds\":[",
             static_cast<unsigned int>(cfg.timeout_seconds), static_cast<unsigned int>(cfg.max_move_gate),
             static_cast<unsigned int>(cfg.max_still_gate), cfg.distance_resolution ? "0.2m" : "0.75m",
             cfg.bluetooth_enabled ? "true" : "false");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    out.printf("%s%u", g ? "," : "", static_cast<unsigned int>(cfg.gate_move_threshold[g]));
  }
  out.print("],\"gate_still_thresholds\":[");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    out.printf("%s%u", g ? "," : "", static_cast<unsigned int>(cfg.gate_still_threshold[g]));
  }
  out.print("]}");
  out.finish();
}

void RadarTunerHandler::handle_ld2410_set_config_(AsyncWebServerRequest *request) {
  if (ld2410_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2410 handler unavailable\"}");
    return;
  }
  if (body_.empty() || body_.size() > MAX_BODY_BYTES) {
    send_error_(request, 400, "{\"error\":\"invalid body\"}");
    return;
  }

  cjson_use_psram_();
  cJSON *root = cJSON_ParseWithLength(body_.c_str(), body_.size());
  if (root == nullptr || !cJSON_IsObject(root)) {
    if (root != nullptr)
      cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid JSON\"}");
    return;
  }

  auto cfg = ld2410_->get_backend_config();
  uint32_t uval = 0;
  bool present = false;

  if (!parse_uint_field_(root, "timeout", uval, present) || (present && uval > 65535)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid timeout\"}");
    return;
  }
  if (present)
    cfg.timeout_seconds = static_cast<uint16_t>(uval);

  if (!parse_uint_field_(root, "max_move_gate", uval, present) || (present && uval > 8)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid max_move_gate\"}");
    return;
  }
  if (present)
    cfg.max_move_gate = static_cast<uint8_t>(uval);

  if (!parse_uint_field_(root, "max_still_gate", uval, present) || (present && uval > 8)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid max_still_gate\"}");
    return;
  }
  if (present)
    cfg.max_still_gate = static_cast<uint8_t>(uval);

  bool bool_val = false;
  if (!parse_bool_field_(root, "bluetooth", bool_val, present)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid bluetooth\"}");
    return;
  }
  if (present)
    cfg.bluetooth_enabled = bool_val;

  cJSON *distance_resolution = cJSON_GetObjectItemCaseSensitive(root, "distance_resolution");
  if (distance_resolution != nullptr) {
    if (!cJSON_IsString(distance_resolution) || distance_resolution->valuestring == nullptr) {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid distance_resolution\"}");
      return;
    }
    if (strcmp(distance_resolution->valuestring, "0.2m") == 0) {
      cfg.distance_resolution = 1;
    } else if (strcmp(distance_resolution->valuestring, "0.75m") == 0) {
      cfg.distance_resolution = 0;
    } else {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid distance_resolution value\"}");
      return;
    }
  }

  cJSON *move_thresholds = cJSON_GetObjectItemCaseSensitive(root, "gate_move_thresholds");
  if (move_thresholds != nullptr) {
    if (!cJSON_IsArray(move_thresholds) ||
        cJSON_GetArraySize(move_thresholds) != static_cast<int>(LD2410Handler::NUM_GATES)) {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid gate_move_thresholds\"}");
      return;
    }
    for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
      cJSON *item = cJSON_GetArrayItem(move_thresholds, static_cast<int>(g));
      if (!cJSON_IsNumber(item) || item->valuedouble < 0 || item->valuedouble > 100) {
        cJSON_Delete(root);
        send_error_(request, 400, "{\"error\":\"invalid move threshold value\"}");
        return;
      }
      cfg.gate_move_threshold[g] = static_cast<uint8_t>(item->valuedouble);
    }
  }

  cJSON *still_thresholds = cJSON_GetObjectItemCaseSensitive(root, "gate_still_thresholds");
  if (still_thresholds != nullptr) {
    if (!cJSON_IsArray(still_thresholds) ||
        cJSON_GetArraySize(still_thresholds) != static_cast<int>(LD2410Handler::NUM_GATES)) {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid gate_still_thresholds\"}");
      return;
    }
    for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
      cJSON *item = cJSON_GetArrayItem(still_thresholds, static_cast<int>(g));
      if (!cJSON_IsNumber(item) || item->valuedouble < 0 || item->valuedouble > 100) {
        cJSON_Delete(root);
        send_error_(request, 400, "{\"error\":\"invalid still threshold value\"}");
        return;
      }
      cfg.gate_still_threshold[g] = static_cast<uint8_t>(item->valuedouble);
    }
  }

  cJSON_Delete(root);
  if (!ld2410_->set_backend_config(cfg)) {
    send_error_(request, 400, "{\"error\":\"config validation failed\"}");
    return;
  }
  send_json_(request, "{\"status\":\"ok\"}");
}

void RadarTunerHandler::handle_ld2410_apply_(AsyncWebServerRequest *request) {
  if (on_ld2410_apply_ != nullptr)
    on_ld2410_apply_();
  send_json_(request, "{\"status\":\"ok\"}");
}

void RadarTunerHandler::handle_ld2410_live_(AsyncWebServerRequest *request) {
  if (ld2410_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2410 handler unavailable\"}");
    return;
  }
  // Recorded before the response, so the very first poll arms engineering mode even though its
  // own reply will still be all zeroes. The page polls twice a second and redraws.
  this->ld2410_live_poll_ms_.store(millis(), std::memory_order_relaxed);
  httpd_resp_set_type(*request, "application/json");
  // By hand on every raw-httpd path a peer Satellite1's remote-control page reads: reaching past
  // ESPHome's response API also reaches past the Access-Control-Allow-Origin default it stamps,
  // and without the header the cross-origin probe fails and the page reports no radar module.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
  ChunkWriter out(*request);
  out.print("{\"gates\":{\"move\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = ld2410_->get_gate_move_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    out.printf("%s%.0f", g ? "," : "", val);
  }
  out.print("],\"still\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = ld2410_->get_gate_still_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    out.printf("%s%.0f", g ? "," : "", val);
  }
  out.print("]}}");
  out.finish();
}

void RadarTunerHandler::handle_ld2450_get_config_(AsyncWebServerRequest *request) {
  if (ld2450_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2450 handler unavailable\"}");
    return;
  }

  const auto &cfg = ld2450_->get_backend_config();
  httpd_resp_set_type(*request, "application/json");
  // By hand on every raw-httpd path a peer Satellite1's remote-control page reads: reaching past
  // ESPHome's response API also reaches past the Access-Control-Allow-Origin default it stamps,
  // and without the header the cross-origin probe fails and the page reports no radar module.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
  ChunkWriter out(*request);
  out.printf("{\"detection_range\":%u,\"stability\":%u,\"timeout\":%u,\"bluetooth\":%s,\"multi_target\":%s,"
             "\"reboot_required\":%s,\"zones\":[",
             static_cast<unsigned int>(cfg.detection_range_cm), static_cast<unsigned int>(cfg.stability),
             static_cast<unsigned int>(cfg.timeout_seconds), cfg.bluetooth_enabled ? "true" : "false",
             cfg.multi_target_enabled ? "true" : "false", ld2450_->is_reboot_required() ? "true" : "false");

  for (size_t z = 0; z < LD2450Handler::NUM_ZONES; z++) {
    out.printf("%s[", z ? "," : "");
    for (size_t p = 0; p < cfg.zones[z].points_count; p++) {
      out.printf("%s{\"x\":%d,\"y\":%d}", p ? "," : "", static_cast<int>(cfg.zones[z].points[p].x),
                 static_cast<int>(cfg.zones[z].points[p].y));
    }
    out.print("]");
  }
  out.print("],\"exclusion\":[");
  for (size_t p = 0; p < cfg.exclusion.points_count; p++) {
    out.printf("%s{\"x\":%d,\"y\":%d}", p ? "," : "", static_cast<int>(cfg.exclusion.points[p].x),
               static_cast<int>(cfg.exclusion.points[p].y));
  }
  out.print("]}");
  out.finish();
}

static bool parse_polygon_points_(cJSON *array, LD2450Handler::Polygon &polygon) {
  if (!cJSON_IsArray(array))
    return false;
  int count = cJSON_GetArraySize(array);
  if (count < 0 || count > static_cast<int>(LD2450Handler::MAX_ZONE_POINTS))
    return false;
  polygon.points_count = static_cast<uint8_t>(count);
  for (int i = 0; i < count; i++) {
    cJSON *point = cJSON_GetArrayItem(array, i);
    if (!cJSON_IsObject(point))
      return false;
    cJSON *x = cJSON_GetObjectItemCaseSensitive(point, "x");
    cJSON *y = cJSON_GetObjectItemCaseSensitive(point, "y");
    if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y))
      return false;
    polygon.points[i].x = static_cast<int16_t>(x->valueint);
    polygon.points[i].y = static_cast<int16_t>(y->valueint);
  }
  for (size_t i = static_cast<size_t>(count); i < LD2450Handler::MAX_ZONE_POINTS; i++) {
    polygon.points[i].x = 0;
    polygon.points[i].y = 0;
  }
  return true;
}

void RadarTunerHandler::handle_ld2450_set_config_(AsyncWebServerRequest *request) {
  if (ld2450_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2450 handler unavailable\"}");
    return;
  }
  if (body_.empty() || body_.size() > MAX_BODY_BYTES) {
    send_error_(request, 400, "{\"error\":\"invalid body\"}");
    return;
  }

  cjson_use_psram_();
  cJSON *root = cJSON_ParseWithLength(body_.c_str(), body_.size());
  if (root == nullptr || !cJSON_IsObject(root)) {
    if (root != nullptr)
      cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid JSON\"}");
    return;
  }

  auto cfg = ld2450_->get_backend_config();
  uint32_t uval = 0;
  bool present = false;

  if (!parse_uint_field_(root, "detection_range", uval, present) || (present && uval > 600)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid detection_range\"}");
    return;
  }
  if (present)
    cfg.detection_range_cm = static_cast<uint16_t>(uval);

  if (!parse_uint_field_(root, "stability", uval, present) || (present && uval > 10)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid stability\"}");
    return;
  }
  if (present)
    cfg.stability = static_cast<uint8_t>(uval);

  if (!parse_uint_field_(root, "timeout", uval, present) || (present && uval > 65535)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid timeout\"}");
    return;
  }
  if (present)
    cfg.timeout_seconds = static_cast<uint16_t>(uval);

  bool bool_val = false;
  if (!parse_bool_field_(root, "bluetooth", bool_val, present)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid bluetooth\"}");
    return;
  }
  if (present)
    cfg.bluetooth_enabled = bool_val;

  if (!parse_bool_field_(root, "multi_target", bool_val, present)) {
    cJSON_Delete(root);
    send_error_(request, 400, "{\"error\":\"invalid multi_target\"}");
    return;
  }
  if (present)
    cfg.multi_target_enabled = bool_val;

  cJSON *zones = cJSON_GetObjectItemCaseSensitive(root, "zones");
  if (zones != nullptr) {
    if (!cJSON_IsArray(zones) || cJSON_GetArraySize(zones) != static_cast<int>(LD2450Handler::NUM_ZONES)) {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid zones\"}");
      return;
    }
    for (size_t z = 0; z < LD2450Handler::NUM_ZONES; z++) {
      cJSON *zone_points = cJSON_GetArrayItem(zones, static_cast<int>(z));
      if (!parse_polygon_points_(zone_points, cfg.zones[z])) {
        cJSON_Delete(root);
        send_error_(request, 400, "{\"error\":\"invalid zone points\"}");
        return;
      }
    }
  }

  cJSON *exclusion = cJSON_GetObjectItemCaseSensitive(root, "exclusion");
  if (exclusion != nullptr) {
    if (!parse_polygon_points_(exclusion, cfg.exclusion)) {
      cJSON_Delete(root);
      send_error_(request, 400, "{\"error\":\"invalid exclusion points\"}");
      return;
    }
  }

  cJSON_Delete(root);
  if (!ld2450_->set_backend_config(cfg)) {
    send_error_(request, 400, "{\"error\":\"config validation failed\"}");
    return;
  }
  send_json_(request, ld2450_->is_reboot_required() ? "{\"status\":\"ok\",\"reboot_required\":true}"
                                                    : "{\"status\":\"ok\",\"reboot_required\":false}");
}

void RadarTunerHandler::handle_ld2450_live_(AsyncWebServerRequest *request) {
  // Always fits one fragment, so the writer buys nothing here beyond reading like the other three.
  httpd_resp_set_type(*request, "application/json");
  // By hand on every raw-httpd path a peer Satellite1's remote-control page reads: reaching past
  // ESPHome's response API also reaches past the Access-Control-Allow-Origin default it stamps,
  // and without the header the cross-origin probe fails and the page reports no radar module.
  httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", "*");
  ChunkWriter out(*request);
  out.printf("{\"targets\":[{\"x\":%.1f,\"y\":%.1f},{\"x\":%.1f,\"y\":%.1f},{\"x\":%.1f,\"y\":%.1f}]}",
             targets_[0].x, targets_[0].y, targets_[1].x, targets_[1].y, targets_[2].x, targets_[2].y);
  out.finish();
}

void RadarTunerHandler::handle_save_(AsyncWebServerRequest *request) {
  global_preferences->sync();
  ESP_LOGI(TAG_RT, "Preferences flushed to NVS");
  send_json_(request, "{\"status\":\"ok\"}");
}

void RadarTunerHandler::handle_reboot_(AsyncWebServerRequest *request) {
  global_preferences->sync();
  ESP_LOGI(TAG_RT, "Reboot requested by tuner UI");
  send_json_(request, "{\"status\":\"ok\"}");
  App.safe_reboot();
}

}  // namespace satellite1_radar
}  // namespace esphome
