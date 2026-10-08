#pragma once

// Every JSON message this component builds: the client events sent to the Realtime server, and the
// documents the web UI reads. Each is a fill function over an ArduinoJson object, so the device
// runs it inside esphome::json::build_json() and the host test
// (tests/openai_realtime/test_rt_messages.cpp) runs the very same function and parses the result.
//
// One exception: the microphone's input_audio_buffer.append (build_audio_append). It goes out every
// 60 ms with ~3.8 KB of base64, which is encoded straight into a reused buffer instead of being
// copied into a document and out again. It is tested the same way as the rest.
//
// Depends on ArduinoJson and rt_util only - no ESP-IDF or ESPHome headers.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <ArduinoJson.h>

#include "rt_util.h"

namespace esphome {
namespace openai_realtime {

/// The credentials and choices the web UI edits. Persisted in NVS; YAML only provides defaults.
struct Settings {
  bool enabled{false};
  std::string base_url;
  std::string model;
  std::string voice;
  std::string api_key;
};

/// Sizes of the persisted fields. The web UI validates against these before anything is stored.
static constexpr size_t MAX_BASE_URL = 159;
static constexpr size_t MAX_MODEL = 63;
static constexpr size_t MAX_VOICE = 63;  // OpenAI custom voice ids and local voice names run long
static constexpr size_t MAX_API_KEY = 255;

/// Result of the discovery the web UI runs whenever its base URL or key changes: GET <base>/models
/// (which also proves the address and key work) and GET <base>/audio/voices.
struct ModelsStatus {
  uint32_t gen{0};
  uint8_t state{0};  ///< 0 idle, 1 loading, 2 ok, 3 error (the server could not be used at all)
  int http_status{0};
  std::string base_url;
  std::string error;
  /// The server answered /models with a list. False with state ok means it was reachable but has
  /// no model list (404 or empty) - the page then lets the person type the model.
  bool has_model_list{false};
  std::vector<std::string> ids;
  /// Built-in voices first (api.openai.com only), then whatever /audio/voices listed.
  std::vector<VoiceEntry> voices;
  uint8_t voices_from_server{0};  ///< how many of `voices` came from /audio/voices
};

/// Result of the web UI's "Test connection" - a real Realtime handshake without audio.
struct TestStatus {
  uint32_t gen{0};
  uint8_t state{0};  ///< 0 idle, 1 running, 2 ok, 3 error
  std::string message;
};

// ------------------------------------------------------------------------------------------------
// Client events sent to the Realtime server
// ------------------------------------------------------------------------------------------------

/// What a session is configured with: the saved model and voice plus the YAML's behaviour.
struct SessionConfig {
  std::string model;
  std::string voice;
  std::string instructions;
  std::string td_type{"semantic_vad"};  ///< semantic_vad | server_vad | none
  std::string td_eagerness{"auto"};
  float td_threshold{0.5f};
  uint32_t td_prefix_ms{300};
  uint32_t td_silence_ms{500};
  std::string noise_reduction;      ///< "" or "none" = not sent
  std::string transcription_model;  ///< "" = not sent
  std::string tools_json;           ///< JSON array of function tools from codegen, or ""
  bool end_tool{true};              ///< add the built-in end_conversation tool
};

/// session.update in the GA format (Flavor::GA) or the beta format (anything else).
void fill_session_update(JsonObject root, const SessionConfig &c, Flavor flavor);
/// conversation.item.truncate for an interrupted assistant item.
void fill_truncate(JsonObject root, const std::string &item_id, uint32_t audio_end_ms);
/// conversation.item.create with a function_call_output. `output` is sent as a string, as the
/// protocol requires (it usually holds JSON itself).
void fill_function_output(JsonObject root, const std::string &call_id, const std::string &output);
/// conversation.item.create with a user text message.
void fill_user_text(JsonObject root, const std::string &message);
/// response.create.
void fill_response_create(JsonObject root);

/// input_audio_buffer.append for `n` bytes of PCM, written into `out` without a JSON document (the
/// 60 ms microphone frame). Its only variable part is base64, whose alphabet needs no escaping.
/// Returns the frame length, or 0 when it would not fit in `cap`.
size_t build_audio_append(const uint8_t *pcm, size_t n, char *out, size_t cap);
/// The frame length build_audio_append() needs for `n` bytes of PCM.
size_t audio_append_size(size_t n);

// ------------------------------------------------------------------------------------------------
// Web UI documents (GET/POST /api/sat1/openai*)
// ------------------------------------------------------------------------------------------------

/// Everything GET /api/sat1/openai reports. Never the key itself - only whether one is stored and
/// its hint.
struct StatusView {
  Settings settings;  ///< api_key is not serialised; only key_set and key_hint are
  bool configured{false};
  std::string realtime_url;
  const char *state{"idle"};
  const char *phase{"none"};
  const char *proto{""};
  std::string last_error;
  ModelsStatus models;
  TestStatus test;
};

void fill_status(JsonObject root, const StatusView &v);
/// {"ok":1,"gen":N} - the answer to POST /models and /test.
void fill_ok_gen(JsonObject root, uint32_t gen);
/// {"ok":1} or {"ok":0,"err":"<field>"} - the answer to POST /api/sat1/openai.
void fill_result(JsonObject root, bool ok, const char *err);

}  // namespace openai_realtime
}  // namespace esphome
