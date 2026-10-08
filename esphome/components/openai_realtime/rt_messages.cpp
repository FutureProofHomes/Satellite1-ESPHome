#include "rt_messages.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace openai_realtime {

// "type" is always the first member: servers (and our own scanners) classify events by it.

/// Every string from outside the firmware (settings, server replies, Home Assistant answers, YAML
/// text) passes through here. ArduinoJson 7 escapes only " \\ \\b \\f \\n \\r \\t and writes other
/// control characters raw, which is invalid JSON (a browser's JSON.parse refuses it); they are
/// replaced by spaces.
static std::string text(const std::string &s) {
  std::string out = s;
  for (char &c : out) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (u < 0x20 && c != '\n' && c != '\r' && c != '\t' && c != '\b' && c != '\f')
      c = ' ';
  }
  return out;
}

// The helpers write through the parent object: in ArduinoJson 7 a JsonVariant taken from a missing
// member (obj["key"] converted) is unbound, and writes to it are silently dropped.
static void fill_turn_detection(JsonObject parent, const char *key, const SessionConfig &c) {
  if (c.td_type == "none") {
    parent[key] = nullptr;  // sent as null: turn detection off
    return;
  }
  JsonObject o = parent[key].to<JsonObject>();
  if (c.td_type == "server_vad") {
    o["type"] = "server_vad";
    // Two decimals, so 0.6f is sent as 0.6 and not as 0.60000002384.
    o["threshold"] = std::round(static_cast<double>(c.td_threshold) * 100.0) / 100.0;
    o["prefix_padding_ms"] = c.td_prefix_ms;
    o["silence_duration_ms"] = c.td_silence_ms;
  } else {
    o["type"] = "semantic_vad";
    o["eagerness"] = text(c.td_eagerness);
  }
  o["create_response"] = true;
  o["interrupt_response"] = true;
}

static void fill_voice(JsonObject parent, const std::string &voice, bool object_form_allowed) {
  // OpenAI custom voices ("voice_..." ids) are an object in the GA format; everything else a string.
  if (object_form_allowed && voice.rfind("voice_", 0) == 0) {
    parent["voice"].to<JsonObject>()["id"] = text(voice);
  } else {
    parent["voice"] = text(voice);
  }
}

static void fill_tools(JsonObject session, const SessionConfig &c) {
  JsonDocument declared;
  bool have_declared = false;
  if (!c.tools_json.empty()) {
    // Generated at build time by json.dumps(); a parse failure would be a codegen bug, and then the
    // declared tools are left out rather than sending something malformed.
    have_declared = deserializeJson(declared, c.tools_json) == DeserializationError::Ok && declared.is<JsonArray>();
  }
  if (!c.end_tool && (!have_declared || declared.as<JsonArrayConst>().size() == 0))
    return;
  JsonArray tools = session["tools"].to<JsonArray>();
  if (have_declared) {
    for (JsonVariantConst t : declared.as<JsonArrayConst>())
      tools.add(t);
  }
  if (c.end_tool) {
    JsonObject t = tools.add<JsonObject>();
    t["type"] = "function";
    t["name"] = "end_conversation";
    t["description"] =
        "End the voice conversation. Call this when the user says goodbye, says they are done, or asks you to "
        "stop listening. Say a short goodbye first.";
    JsonObject p = t["parameters"].to<JsonObject>();
    p["type"] = "object";
    p["properties"].to<JsonObject>();
  }
  session["tool_choice"] = "auto";
}

void fill_session_update(JsonObject root, const SessionConfig &c, Flavor flavor) {
  const bool nr = !c.noise_reduction.empty() && c.noise_reduction != "none";
  root["type"] = "session.update";
  JsonObject s = root["session"].to<JsonObject>();
  if (flavor == Flavor::GA) {
    // GA (api.openai.com): typed session, nested audio.input / audio.output, model in the session.
    s["type"] = "realtime";
    s["model"] = text(c.model);
    s["output_modalities"].to<JsonArray>().add("audio");
    if (!c.instructions.empty())
      s["instructions"] = text(c.instructions);
    JsonObject audio = s["audio"].to<JsonObject>();
    JsonObject in = audio["input"].to<JsonObject>();
    JsonObject in_fmt = in["format"].to<JsonObject>();
    in_fmt["type"] = "audio/pcm";
    in_fmt["rate"] = 24000;
    fill_turn_detection(in, "turn_detection", c);
    if (nr)
      in["noise_reduction"].to<JsonObject>()["type"] = text(c.noise_reduction);
    if (!c.transcription_model.empty())
      in["transcription"].to<JsonObject>()["model"] = text(c.transcription_model);
    JsonObject out = audio["output"].to<JsonObject>();
    JsonObject out_fmt = out["format"].to<JsonObject>();
    out_fmt["type"] = "audio/pcm";
    out_fmt["rate"] = 24000;
    fill_voice(out, c.voice, true);
  } else {
    // Beta (most self-hosted and compatible servers): flat session, pcm16 (24 kHz mono) both ways,
    // model taken from the URL. Audio needs "text" alongside it in this format.
    JsonArray mod = s["modalities"].to<JsonArray>();
    mod.add("audio");
    mod.add("text");
    if (!c.instructions.empty())
      s["instructions"] = text(c.instructions);
    fill_voice(s, c.voice, false);
    s["input_audio_format"] = "pcm16";
    s["output_audio_format"] = "pcm16";
    fill_turn_detection(s, "turn_detection", c);
    if (nr)
      s["input_audio_noise_reduction"].to<JsonObject>()["type"] = text(c.noise_reduction);
    if (!c.transcription_model.empty())
      s["input_audio_transcription"].to<JsonObject>()["model"] = text(c.transcription_model);
  }
  fill_tools(s, c);
}

void fill_truncate(JsonObject root, const std::string &item_id, uint32_t audio_end_ms) {
  root["type"] = "conversation.item.truncate";
  root["item_id"] = text(item_id);
  root["content_index"] = 0;
  root["audio_end_ms"] = audio_end_ms;
}

void fill_function_output(JsonObject root, const std::string &call_id, const std::string &output) {
  root["type"] = "conversation.item.create";
  JsonObject item = root["item"].to<JsonObject>();
  item["type"] = "function_call_output";
  item["call_id"] = text(call_id);
  item["output"] = output.empty() ? std::string("{}") : text(output);
}

void fill_user_text(JsonObject root, const std::string &message) {
  root["type"] = "conversation.item.create";
  JsonObject item = root["item"].to<JsonObject>();
  item["type"] = "message";
  item["role"] = "user";
  JsonObject part = item["content"].to<JsonArray>().add<JsonObject>();
  part["type"] = "input_text";
  part["text"] = text(message);
}

void fill_response_create(JsonObject root) { root["type"] = "response.create"; }

static const char AUDIO_PREFIX[] = R"({"type":"input_audio_buffer.append","audio":")";
static const char AUDIO_SUFFIX[] = "\"}";

size_t audio_append_size(size_t n) { return sizeof(AUDIO_PREFIX) - 1 + base64_encoded_len(n) + sizeof(AUDIO_SUFFIX) - 1; }

size_t build_audio_append(const uint8_t *pcm, size_t n, char *out, size_t cap) {
  const size_t plen = sizeof(AUDIO_PREFIX) - 1;
  const size_t blen = base64_encoded_len(n);
  const size_t total = audio_append_size(n);
  if (out == nullptr || total > cap)
    return 0;
  memcpy(out, AUDIO_PREFIX, plen);
  base64_encode(pcm, n, out + plen);
  memcpy(out + plen + blen, AUDIO_SUFFIX, sizeof(AUDIO_SUFFIX) - 1);
  return total;
}

// ------------------------------------------------------------------------------------------------

void fill_status(JsonObject root, const StatusView &v) {
  static const char *const MODEL_STATES[] = {"idle", "loading", "ok", "error"};
  static const char *const TEST_STATES[] = {"idle", "running", "ok", "error"};
  root["enabled"] = v.settings.enabled;
  root["configured"] = v.configured;
  root["base_url"] = text(v.settings.base_url);
  root["model"] = text(v.settings.model);
  root["voice"] = text(v.settings.voice);
  root["key_set"] = !v.settings.api_key.empty();
  root["key_hint"] = text(key_hint(v.settings.api_key));
  root["realtime_url"] = text(v.realtime_url);
  root["state"] = v.state;
  root["phase"] = v.phase;
  root["proto"] = v.proto;
  root["err"] = text(v.last_error);

  JsonObject m = root["models"].to<JsonObject>();
  m["gen"] = v.models.gen;
  m["st"] = MODEL_STATES[v.models.state < 4 ? v.models.state : 0];
  m["base"] = text(v.models.base_url);
  m["err"] = text(v.models.error);
  m["list"] = v.models.has_model_list;
  JsonArray ids = m["ids"].to<JsonArray>();
  for (const auto &id : v.models.ids)
    ids.add(text(id));
  // Voices: [id, label, listed-by-server]. Built-ins (api.openai.com) come first and are not
  // marked; the rest are what the server's /audio/voices returned.
  JsonArray voices = m["voices"].to<JsonArray>();
  const size_t n = v.models.voices.size();
  const size_t builtin = n - std::min<size_t>(v.models.voices_from_server, n);
  for (size_t i = 0; i < n; i++) {
    JsonArray e = voices.add<JsonArray>();
    e.add(text(v.models.voices[i].id));
    e.add(text(v.models.voices[i].name));
    e.add(i >= builtin ? 1 : 0);
  }

  JsonObject t = root["test"].to<JsonObject>();
  t["gen"] = v.test.gen;
  t["st"] = TEST_STATES[v.test.state < 4 ? v.test.state : 0];
  t["msg"] = text(v.test.message);
}

void fill_ok_gen(JsonObject root, uint32_t gen) {
  root["ok"] = 1;
  root["gen"] = gen;
}

void fill_result(JsonObject root, bool ok, const char *err) {
  root["ok"] = ok ? 1 : 0;
  if (!ok)
    root["err"] = err != nullptr ? text(err) : std::string("error");
}

}  // namespace openai_realtime
}  // namespace esphome
