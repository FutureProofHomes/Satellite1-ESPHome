#pragma once

// Every JSON message this component sends to the Realtime server. Each is a fill function over an
// ArduinoJson object, so the device runs it inside esphome::json::build_json() and the host test
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

#include <ArduinoJson.h>

#include "rt_util.h"

namespace esphome {
namespace openai_realtime {

// ------------------------------------------------------------------------------------------------
// Client events sent to the Realtime server
// ------------------------------------------------------------------------------------------------

/// What a session is configured with (all from YAML).
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
  bool interrupt_response{true};    ///< false in turn-based mode: speech never cancels an answer
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

}  // namespace openai_realtime
}  // namespace esphome
