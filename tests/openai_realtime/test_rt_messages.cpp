// Host tests for esphome/components/openai_realtime/rt_messages.{h,cpp}: every JSON message the
// component sends to the Realtime server or serves to the web UI.
//
// Each message is built exactly as on the device (the fill function inside a JSON builder, here
// ArduinoJson directly - esphome::json::build_json is a thin wrapper around the same calls), then
// parsed back and checked field by field, with hostile strings in every free-text field.
//
//   g++ -std=c++17 -O2 -Wall -Wextra -I esphome/components/openai_realtime -I <ArduinoJson>/src
//       tests/openai_realtime/test_rt_messages.cpp esphome/components/openai_realtime/rt_messages.cpp
//       esphome/components/openai_realtime/rt_util.cpp -o /tmp/tm && /tmp/tm
//
// With --dump it also prints every built message, one per line, so CI can have an independent
// parser (Python's json module) validate them too.

#include "rt_messages.h"

#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

using namespace esphome::openai_realtime;

static int failures = 0;
static bool dump = false;
static std::vector<std::string> built;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
      failures++;                                                  \
    }                                                              \
  } while (0)
#define CHECK_EQ(a, b)                                                                                  \
  do {                                                                                                  \
    const std::string _a = (a), _b = (b);                                                               \
    if (_a != _b) {                                                                                     \
      std::printf("FAIL %s:%d: %s\n   got:  %s\n   want: %s\n", __FILE__, __LINE__, #a, _a.c_str(),     \
                  _b.c_str());                                                                          \
      failures++;                                                                                       \
    }                                                                                                   \
  } while (0)

/// What esphome::json::build_json does: a document, the fill function on its root, serialised.
static std::string build(const std::function<void(JsonObject)> &fill) {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  fill(root);
  CHECK(!doc.overflowed());
  std::string out;
  serializeJson(doc, out);
  // RFC 8259: no raw control characters anywhere (ArduinoJson writes 0x00-0x1F other than its six
  // named escapes raw, so the fill functions must never hand it one).
  for (unsigned char ch : out) {
    if (ch < 0x20) {
      std::printf("FAIL: raw control character 0x%02x in: %.80s\n", ch, out.c_str());
      failures++;
      break;
    }
  }
  built.push_back(out);
  return out;
}

/// Parses strictly (whole input, nothing trailing) and checks "type" leads when the message has one.
static JsonDocument parse(const std::string &s, bool typed = true) {
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, s);
  if (err) {
    std::printf("FAIL: not valid JSON (%s): %s\n", err.c_str(), s.c_str());
    failures++;
  }
  if (typed && s.rfind("{\"type\":", 0) != 0) {
    std::printf("FAIL: \"type\" is not the first member: %.80s\n", s.c_str());
    failures++;
  }
  return doc;
}

// Everything JSON must escape, plus non-ASCII, in one string.
static const std::string NASTY = std::string("quote\" backslash\\ slash/ nl\n cr\r tab\t bs\b ff\f ctl") + '\x01' +
                                 '\x1f' + '\0' + " del\x7f utf8 \xC3\xA9 \xE2\x80\xA8 emoji \xF0\x9F\x98\x80 {\"}]";

/// What a string reads as after a round trip: other control characters become spaces.
static std::string clean(const std::string &s) {
  std::string out = s;
  for (char &c : out) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (u < 0x20 && c != '\n' && c != '\r' && c != '\t' && c != '\b' && c != '\f')
      c = ' ';
  }
  return out;
}

static SessionConfig config() {
  SessionConfig c;
  c.model = "gpt-realtime-2.1";
  c.voice = "marin";
  c.instructions = "Be brief. " + NASTY;
  c.tools_json =
      R"([{"type":"function","name":"home_assistant","description":"Send a request.","parameters":{"type":"object",)"
      R"("properties":{"command":{"type":"string"}},"required":["command"]}}])";
  return c;
}

static void test_session_update_ga() {
  SessionConfig c = config();
  JsonDocument d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); }));
  CHECK_EQ(d["type"] | "", "session.update");
  JsonObjectConst s = d["session"];
  CHECK_EQ(s["type"] | "", "realtime");
  CHECK_EQ(s["model"] | "", c.model);
  CHECK_EQ(s["instructions"] | "", clean(c.instructions));  // the nasty string survives the round trip
  CHECK_EQ(s["output_modalities"][0] | "", "audio");
  CHECK_EQ(s["audio"]["input"]["format"]["type"] | "", "audio/pcm");
  CHECK((s["audio"]["input"]["format"]["rate"] | 0) == 24000);
  CHECK((s["audio"]["output"]["format"]["rate"] | 0) == 24000);
  CHECK_EQ(s["audio"]["output"]["voice"] | "", "marin");
  JsonObjectConst td = s["audio"]["input"]["turn_detection"];
  CHECK_EQ(td["type"] | "", "semantic_vad");
  CHECK_EQ(td["eagerness"] | "", "auto");
  CHECK(td["create_response"] == true && td["interrupt_response"] == true);
  CHECK(s["audio"]["input"]["noise_reduction"].isNull());
  CHECK(s["audio"]["input"]["transcription"].isNull());
  JsonArrayConst tools = s["tools"];
  CHECK(tools.size() == 2);
  CHECK_EQ(tools[0]["name"] | "", "home_assistant");
  CHECK_EQ(tools[0]["parameters"]["required"][0] | "", "command");
  CHECK_EQ(tools[1]["name"] | "", "end_conversation");
  CHECK_EQ(tools[1]["parameters"]["type"] | "", "object");
  CHECK(tools[1]["parameters"]["properties"].is<JsonObjectConst>());
  CHECK_EQ(s["tool_choice"] | "", "auto");
  CHECK(s["modalities"].isNull() && s["input_audio_format"].isNull());

  // Custom OpenAI voices are an object in GA; options appear when configured.
  c.voice = "voice_123abc";
  c.noise_reduction = "far_field";
  c.transcription_model = "gpt-4o-mini-transcribe";
  d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); }));
  CHECK_EQ(d["session"]["audio"]["output"]["voice"]["id"] | "", "voice_123abc");
  CHECK_EQ(d["session"]["audio"]["input"]["noise_reduction"]["type"] | "", "far_field");
  CHECK_EQ(d["session"]["audio"]["input"]["transcription"]["model"] | "", "gpt-4o-mini-transcribe");
}

static void test_session_update_beta() {
  SessionConfig c = config();
  c.voice = "voice_123abc";  // a plain string in beta, whatever its prefix
  c.noise_reduction = "near_field";
  c.transcription_model = "whisper-1";
  JsonDocument d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::BETA); }));
  JsonObjectConst s = d["session"];
  CHECK(s["type"].isNull() && s["model"].isNull() && s["audio"].isNull());
  CHECK_EQ(s["modalities"][0] | "", "audio");
  CHECK_EQ(s["modalities"][1] | "", "text");
  CHECK_EQ(s["voice"] | "", "voice_123abc");
  CHECK_EQ(s["input_audio_format"] | "", "pcm16");
  CHECK_EQ(s["output_audio_format"] | "", "pcm16");
  CHECK_EQ(s["turn_detection"]["type"] | "", "semantic_vad");
  CHECK_EQ(s["input_audio_noise_reduction"]["type"] | "", "near_field");
  CHECK_EQ(s["input_audio_transcription"]["model"] | "", "whisper-1");
  CHECK_EQ(s["instructions"] | "", clean(c.instructions));
  CHECK(s["tools"].size() == 2);
}

static void test_turn_detection_and_tools() {
  SessionConfig c = config();
  c.td_type = "server_vad";
  c.td_threshold = 0.6f;
  c.td_prefix_ms = 250;
  c.td_silence_ms = 700;
  const std::string s = build([&](JsonObject r) { fill_session_update(r, c, Flavor::BETA); });
  JsonDocument d = parse(s);
  JsonObjectConst td = d["session"]["turn_detection"];
  CHECK_EQ(td["type"] | "", "server_vad");
  CHECK(td["prefix_padding_ms"] == 250 && td["silence_duration_ms"] == 700);
  CHECK(s.find("\"threshold\":0.6,") != std::string::npos);  // not 0.60000002384

  c.td_type = "none";
  const std::string off = build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); });
  d = parse(off);
  CHECK(d["session"]["audio"]["input"]["turn_detection"].isNull());
  CHECK(off.find("\"turn_detection\":null") != std::string::npos);  // sent as null, not left out

  // No declared tools: only the built-in one. Neither: no tools at all.
  c.tools_json = "";
  d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); }));
  CHECK(d["session"]["tools"].size() == 1);
  c.end_tool = false;
  d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); }));
  CHECK(d["session"]["tools"].isNull() && d["session"]["tool_choice"].isNull());
  // Malformed tools JSON (a codegen bug) is left out, never sent half-formed.
  c.end_tool = true;
  c.tools_json = "[{\"type\":";
  d = parse(build([&](JsonObject r) { fill_session_update(r, c, Flavor::GA); }));
  CHECK(d["session"]["tools"].size() == 1);
}

static void test_client_events() {
  JsonDocument d = parse(build([](JsonObject r) { fill_truncate(r, "item_\"x\\", 1840); }));
  CHECK_EQ(d["type"] | "", "conversation.item.truncate");
  CHECK_EQ(d["item_id"] | "", "item_\"x\\");
  CHECK(d["content_index"] == 0 && d["audio_end_ms"] == 1840);

  // Home Assistant's whole answer goes back as a string holding JSON.
  const std::string ha = R"({"response":{"speech":{"plain":{"speech":"Turned off \"Kitchen\" lights"}}}})";
  d = parse(build([&](JsonObject r) { fill_function_output(r, "call_1", ha); }));
  CHECK_EQ(d["type"] | "", "conversation.item.create");
  CHECK_EQ(d["item"]["type"] | "", "function_call_output");
  CHECK_EQ(d["item"]["call_id"] | "", "call_1");
  CHECK_EQ(d["item"]["output"] | "", ha);
  d = parse(build([](JsonObject r) { fill_function_output(r, "call_2", ""); }));
  CHECK_EQ(d["item"]["output"] | "", "{}");

  d = parse(build([](JsonObject r) { fill_user_text(r, NASTY); }));
  CHECK_EQ(d["item"]["type"] | "", "message");
  CHECK_EQ(d["item"]["role"] | "", "user");
  CHECK_EQ(d["item"]["content"][0]["type"] | "", "input_text");
  CHECK_EQ(d["item"]["content"][0]["text"] | "", clean(NASTY));

  CHECK_EQ(build([](JsonObject r) { fill_response_create(r); }), R"({"type":"response.create"})");
}

static void test_audio_append() {
  std::vector<uint8_t> pcm(2880);
  for (size_t i = 0; i < pcm.size(); i++)
    pcm[i] = static_cast<uint8_t>(i * 31 + 7);
  std::vector<char> buf(8192);
  const size_t n = build_audio_append(pcm.data(), pcm.size(), buf.data(), buf.size());
  CHECK(n == audio_append_size(pcm.size()));
  CHECK(n == 3840 + 47);
  const std::string frame(buf.data(), n);
  built.push_back(frame);
  JsonDocument d = parse(frame);
  CHECK_EQ(d["type"] | "", "input_audio_buffer.append");
  const std::string b64 = d["audio"] | "";
  std::vector<uint8_t> back(b64.size());
  bool ok;
  const size_t m = base64_decode(b64.data(), b64.size(), back.data(), ok);
  CHECK(ok && m == pcm.size() && memcmp(back.data(), pcm.data(), m) == 0);
  // Identical to what ArduinoJson would produce for the same message.
  std::string encoded(base64_encoded_len(pcm.size()), '\0');
  base64_encode(pcm.data(), pcm.size(), &encoded[0]);
  JsonDocument ref;
  ref["type"] = "input_audio_buffer.append";
  ref["audio"] = encoded;
  std::string expect;
  serializeJson(ref, expect);
  CHECK_EQ(frame, expect);
  // Refuses to overrun, never writes a truncated frame.
  CHECK(build_audio_append(pcm.data(), pcm.size(), buf.data(), n - 1) == 0);
  CHECK(build_audio_append(pcm.data(), 2, buf.data(), buf.size()) == audio_append_size(2));
}

static void test_status() {
  StatusView v;
  v.settings.enabled = true;
  v.settings.base_url = "http://192.168.1.20:8000/v1";
  v.settings.model = "qwen\"2.5";
  v.settings.voice = "af_bella";
  v.settings.api_key = "sk-SECRET-never-serialised-7f3a";
  v.configured = true;
  v.realtime_url = "ws://192.168.1.20:8000/v1/realtime?model=qwen%222.5";
  v.state = "active";
  v.phase = "speaking";
  v.proto = "beta";
  v.last_error = NASTY;
  v.models.gen = 7;
  v.models.state = 2;
  v.models.base_url = v.settings.base_url;
  v.models.has_model_list = true;
  v.models.ids = {"a", "b\"c"};
  v.models.voices = {{"marin", ""}, {"voice_1", NASTY}};
  v.models.voices_from_server = 1;
  v.test.gen = 3;
  v.test.state = 3;
  v.test.message = "HTTP 401: \"bad key\"";
  const std::string s = build([&](JsonObject r) { fill_status(r, v); });
  JsonDocument d = parse(s, false);
  CHECK(s.find("SECRET") == std::string::npos);  // the key never leaves the device
  CHECK(d["key_set"] == true);
  CHECK_EQ(d["key_hint"] | "", "\xE2\x80\xA6" "7f3a");
  CHECK(d["enabled"] == true && d["configured"] == true);
  CHECK_EQ(d["model"] | "", "qwen\"2.5");
  CHECK_EQ(d["state"] | "", "active");
  CHECK_EQ(d["phase"] | "", "speaking");
  CHECK_EQ(d["proto"] | "", "beta");
  CHECK_EQ(d["err"] | "", clean(NASTY));
  CHECK_EQ(d["models"]["st"] | "", "ok");
  CHECK(d["models"]["gen"] == 7 && d["models"]["list"] == true);
  CHECK_EQ(d["models"]["ids"][1] | "", "b\"c");
  CHECK_EQ(d["models"]["voices"][0][0] | "", "marin");
  CHECK(d["models"]["voices"][0][2] == 0);
  CHECK_EQ(d["models"]["voices"][1][1] | "", clean(NASTY));
  CHECK(d["models"]["voices"][1][2] == 1);
  CHECK_EQ(d["test"]["st"] | "", "error");
  CHECK_EQ(d["test"]["msg"] | "", v.test.message);

  // Out-of-range states and a server count larger than the list stay well-formed.
  v.models.state = 9;
  v.models.voices_from_server = 50;
  d = parse(build([&](JsonObject r) { fill_status(r, v); }), false);
  CHECK_EQ(d["models"]["st"] | "", "idle");
  CHECK(d["models"]["voices"][0][2] == 1);

  StatusView empty;
  d = parse(build([&](JsonObject r) { fill_status(r, empty); }), false);
  CHECK(d["key_set"] == false);
  CHECK_EQ(d["key_hint"] | "x", "");
  CHECK(d["models"]["ids"].size() == 0 && d["models"]["voices"].size() == 0);

  CHECK_EQ(build([](JsonObject r) { fill_ok_gen(r, 12); }), R"({"ok":1,"gen":12})");
  CHECK_EQ(build([](JsonObject r) { fill_result(r, true, nullptr); }), R"({"ok":1})");
  CHECK_EQ(build([](JsonObject r) { fill_result(r, false, "base_url"); }), R"({"ok":0,"err":"base_url"})");
}

int main(int argc, char **argv) {
  dump = argc > 1 && strcmp(argv[1], "--dump") == 0;
  test_session_update_ga();
  test_session_update_beta();
  test_turn_detection_and_tools();
  test_client_events();
  test_audio_append();
  test_status();
  if (dump) {
    for (const auto &m : built)
      std::printf("%s\n", m.c_str());
    return failures ? 1 : 0;
  }
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all rt_messages tests passed (%zu messages built and parsed)\n", built.size());
  return 0;
}
