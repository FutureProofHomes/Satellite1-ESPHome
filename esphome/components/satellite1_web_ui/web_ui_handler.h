#pragma once

// Before any USE_* test in this file. Without it the guards below are evaluated against an empty
// macro set - the conditional include gets skipped, and then a transitive defines.h from one of the
// headers underneath turns the same guards on for the member declarations, which then reference a
// type that was never included.
#include "esphome/core/defines.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#ifdef USE_VOICE_ASSISTANT
#include "esphome/components/voice_assistant/voice_assistant.h"
#endif

#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace satellite1_web_ui {

/// Where the app is served. "/" is the real home; "/ui/" is a canary. If our handler ever stops
/// registering ahead of web_server the root goes back to ESPHome's stock dashboard, and the alias
/// is what makes that a cosmetic regression instead of an unreachable app.
static const char *const WU_URL_ROOT = "/";
static const char *const WU_URL_ALIAS = "/ui";
static const char *const WU_URL_ALIAS_SLASH = "/ui/";

/// How many exchanges the Voice card keeps. Eight is about a screen's worth on a phone, and the
/// point is "what did it just mishear", not a conversation history - which would want storage and a
/// retention policy that a device with 165KB of internal heap free has no business offering.
static constexpr size_t WU_TRANSCRIPT_RING = 8;

/// One heard-or-said line. `heard` distinguishes the two so the UI can attribute them without
/// carrying a second string per entry.
struct Utterance {
  std::string text;
  uint32_t at_uptime;
  bool heard;
};

/// One row of the key -> "<domain>/<name>" table the frontend reads from GET /api/sat1/state.
/// `key` and `domain` are string literals from generated code; `entity` is resolved to its name
/// lazily, because get_name() is only meaningful once the entity has been constructed.
struct EntityRef {
  const char *key;
  const char *domain;
  EntityBase *entity;
};

/**
 * The web app's HTTP surface: the bundle itself, plus the endpoints web_server cannot cover.
 *
 * Three constraints from the shared server shaped this file, all of them inherited from Phase 1:
 *
 * The httpd task stack is 4352 bytes and there is no YAML knob for it - AsyncWebServer::begin()
 * hardcodes HTTPD_DEFAULT_CONFIG().stack_size + 256. So every JSON response is streamed into an
 * AsyncResponseStream, which accumulates on the heap, rather than built in a stack buffer.
 *
 * web_server_idf registers exactly three wildcard URI handlers - GET, POST and OPTIONS - so PATCH
 * and DELETE never reach any handler at all. Writes are POST.
 *
 * ESPHome's response API cannot express 304. AsyncWebServerRequest::init_response_ maps status
 * codes through a switch whose default is 500, and 304 is not one of its cases, so
 * beginResponse(304, ...) would send a 500 carrying our cache headers. handle_index_ therefore
 * reaches past it to httpd_resp_set_status directly, which the operator httpd_req_t*() on the
 * request makes legal.
 */
class WebUIHandler : public AsyncWebHandler {
 public:
  void set_index(const uint8_t *gz, size_t gz_len) {
    this->index_gz_ = gz;
    this->index_gz_len_ = gz_len;
  }

  /// Must outlive every request: ESP-IDF's httpd_resp_set_hdr stores the pointer rather than
  /// copying, and its own documentation says "make sure that the lifetime of the field value
  /// strings are valid till send function is called". Codegen passes a string literal.
  void set_etag(const char *etag) { this->etag_ = etag; }

  /// Longest gap between two consecutive main-loop iterations since this was last read, in
  /// milliseconds. Written from the main loop, read from the httpd task, and reset by the read -
  /// which is why it is an exchange rather than a load. Diagnostics is the only reader.
  void set_loop_time_source(std::atomic<uint32_t> *max_loop_ms) { this->max_loop_ms_ = max_loop_ms; }

  /// Called only from generated setup code in main.cpp, which runs to completion before the
  /// listener accepts anything, so the table is immutable by the time the httpd task can read it.
  void add_entity(const char *key, const char *domain, EntityBase *entity) {
    this->entities_.push_back({key, domain, entity});
  }

#ifdef USE_VOICE_ASSISTANT
  void set_voice_assistant(voice_assistant::VoiceAssistant *va) { this->va_ = va; }

  /// Reads the existing `voice_assistant_phase` global rather than mirroring it.
  ///
  /// The plan called for an internal text_sensor fed by the phase triggers, but config/ already
  /// keeps that global and every trigger already maintains it - so a second copy would be one more
  /// thing to keep in step, and would go stale in exactly the situation the panel exists for. A
  /// lambda from YAML is how a C++ component reads a `globals:` entry without depending on the
  /// generated type of the global.
  void set_voice_phase_fn(std::function<int()> fn) { this->voice_phase_fn_ = std::move(fn); }

  /// Called from the on_stt_end and on_tts_start triggers. Runs on the main loop, and the ring is
  /// read from the httpd task, so both sides take the lock.
  void push_utterance(const std::string &text, bool heard);
#endif

  // NOLINTNEXTLINE(readability-identifier-naming)
  bool canHandle(AsyncWebServerRequest *request) const override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleRequest(AsyncWebServerRequest *request) override;

 protected:
  /// One route table for canHandle and handleRequest, so the two can never disagree about which
  /// paths exist. Deliberately narrow: this handler is first in web_server_base's vector, so
  /// anything it claims by accident it steals from /events, the entity REST API or /radar_tuner.
  enum class Route : uint8_t {
    NONE = 0,
    INDEX,
    STATE,
    VOICE,
  };

  static Route match_route_(AsyncWebServerRequest *request);

  void handle_index_(AsyncWebServerRequest *request);
  void handle_state_(AsyncWebServerRequest *request);
  void handle_voice_(AsyncWebServerRequest *request);

  const uint8_t *index_gz_{nullptr};
  size_t index_gz_len_{0};
  const char *etag_{nullptr};
  std::atomic<uint32_t> *max_loop_ms_{nullptr};
  std::vector<EntityRef> entities_;

#ifdef USE_VOICE_ASSISTANT
  voice_assistant::VoiceAssistant *va_{nullptr};
  std::function<int()> voice_phase_fn_{};
  std::vector<Utterance> transcript_;
  Mutex transcript_lock_;
#endif
};

}  // namespace satellite1_web_ui
}  // namespace esphome
