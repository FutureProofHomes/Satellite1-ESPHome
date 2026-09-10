#include "radar_tuner_handler.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <cJSON.h>
#include <cmath>
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG_RT = "radar_tuner";

/// Ceiling on an accumulated request body. The largest real payload is an LD2450 zone write -
/// three zones plus an exclusion polygon at eight points each, roughly 700 bytes - so this is
/// generous. It exists because handleBody streams with no cap of its own, and an accumulating
/// std::string is a heap growth primitive handed to whoever can reach the endpoint.
static constexpr size_t MAX_BODY_BYTES = 4096;

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
    if (url == RT_URL_ROOT)
      return Route::ROOT;
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
    case Route::ROOT:
      this->handle_root_(request);
      break;
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

void RadarTunerHandler::handle_root_(AsyncWebServerRequest *request) {
  if (html_gz_ == nullptr || html_gz_len_ == 0) {
    send_error_(request, 500, "{\"error\":\"no HTML content\"}");
    return;
  }
  auto *response = request->beginResponse(200, "text/html", html_gz_, html_gz_len_);
  response->addHeader("Content-Encoding", "gzip");
  request->send(response);
}

void RadarTunerHandler::handle_ld2410_get_config_(AsyncWebServerRequest *request) {
  if (ld2410_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2410 handler unavailable\"}");
    return;
  }

  const auto &cfg = ld2410_->get_backend_config();
  auto *stream = request->beginResponseStream("application/json");
  stream->printf("{\"timeout\":%u,\"max_move_gate\":%u,\"max_still_gate\":%u,\"distance_resolution\":\"%s\","
                 "\"bluetooth\":%s,\"gate_move_thresholds\":[",
                 static_cast<unsigned int>(cfg.timeout_seconds), static_cast<unsigned int>(cfg.max_move_gate),
                 static_cast<unsigned int>(cfg.max_still_gate), cfg.distance_resolution ? "0.2m" : "0.75m",
                 cfg.bluetooth_enabled ? "true" : "false");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    stream->printf("%s%u", g ? "," : "", static_cast<unsigned int>(cfg.gate_move_threshold[g]));
  }
  stream->print("],\"gate_still_thresholds\":[");
  for (size_t g = 0; g < LD2410Handler::NUM_GATES; g++) {
    stream->printf("%s%u", g ? "," : "", static_cast<unsigned int>(cfg.gate_still_threshold[g]));
  }
  stream->print("]}");
  request->send(stream);
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
  auto *stream = request->beginResponseStream("application/json");
  stream->print("{\"gates\":{\"move\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = ld2410_->get_gate_move_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    stream->printf("%s%.0f", g ? "," : "", val);
  }
  stream->print("],\"still\":[");
  for (int g = 0; g < RT_NUM_GATES; g++) {
    float val = ld2410_->get_gate_still_energy(static_cast<size_t>(g));
    if (std::isnan(val))
      val = 0;
    stream->printf("%s%.0f", g ? "," : "", val);
  }
  stream->print("]}}");
  request->send(stream);
}

void RadarTunerHandler::handle_ld2450_get_config_(AsyncWebServerRequest *request) {
  if (ld2450_ == nullptr) {
    send_error_(request, 404, "{\"error\":\"LD2450 handler unavailable\"}");
    return;
  }

  const auto &cfg = ld2450_->get_backend_config();
  auto *stream = request->beginResponseStream("application/json");
  stream->printf("{\"detection_range\":%u,\"stability\":%u,\"timeout\":%u,\"bluetooth\":%s,\"multi_target\":%s,"
                 "\"reboot_required\":%s,\"zones\":[",
                 static_cast<unsigned int>(cfg.detection_range_cm), static_cast<unsigned int>(cfg.stability),
                 static_cast<unsigned int>(cfg.timeout_seconds), cfg.bluetooth_enabled ? "true" : "false",
                 cfg.multi_target_enabled ? "true" : "false", ld2450_->is_reboot_required() ? "true" : "false");

  for (size_t z = 0; z < LD2450Handler::NUM_ZONES; z++) {
    stream->printf("%s[", z ? "," : "");
    for (size_t p = 0; p < cfg.zones[z].points_count; p++) {
      stream->printf("%s{\"x\":%d,\"y\":%d}", p ? "," : "", static_cast<int>(cfg.zones[z].points[p].x),
                     static_cast<int>(cfg.zones[z].points[p].y));
    }
    stream->print("]");
  }
  stream->print("],\"exclusion\":[");
  for (size_t p = 0; p < cfg.exclusion.points_count; p++) {
    stream->printf("%s{\"x\":%d,\"y\":%d}", p ? "," : "", static_cast<int>(cfg.exclusion.points[p].x),
                   static_cast<int>(cfg.exclusion.points[p].y));
  }
  stream->print("]}");
  request->send(stream);
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
  auto *stream = request->beginResponseStream("application/json");
  stream->printf("{\"targets\":[{\"x\":%.1f,\"y\":%.1f},{\"x\":%.1f,\"y\":%.1f},{\"x\":%.1f,\"y\":%.1f}]}",
                 targets_[0].x, targets_[0].y, targets_[1].x, targets_[1].y, targets_[2].x, targets_[2].y);
  request->send(stream);
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
