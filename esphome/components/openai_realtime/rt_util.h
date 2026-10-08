#pragma once

// Everything in here is free of ESP-IDF and ESPHome dependencies on purpose: it is the part of the
// component that is easy to get subtly wrong (URL derivation, base64, the resampler, JSON scanning)
// and it is unit-tested on the host by tests/openai_realtime/test_rt_util.cpp.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome {
namespace openai_realtime {

// ------------------------------------------------------------------------------------------------
// Endpoints
// ------------------------------------------------------------------------------------------------

/// The two URLs one configured "base URL" stands for, plus the origin the stored API key belongs to.
///
/// The base URL is the OpenAI SDK's `baseURL` convention - `https://api.openai.com/v1` - because that
/// is what every OpenAI-compatible relay documents. From it:
///   realtime_url = wss://api.openai.com/v1/realtime?model=<model>
///   models_url   = https://api.openai.com/v1/models
///   voices_url   = https://api.openai.com/v1/audio/voices
/// A base given as ws:// / wss://, or already ending in /realtime, is accepted and mapped back, so a
/// pasted Realtime URL works as well. Query parameters on the base (some relays and Azure use them)
/// are kept on the Realtime URL; `model=` is only added when the base does not already carry one.
struct Endpoints {
  std::string realtime_url;
  std::string models_url;
  std::string voices_url;  ///< <base>/audio/voices
  std::string origin;  ///< http(s)://host[:port], lowercase host - what an API key is scoped to.
  bool secure{true};
};

/// Trims whitespace and trailing slashes. Does not validate.
std::string normalize_base_url(const std::string &base_url);

/// Fills `out` and returns true, or returns false with a short reason in `error`
/// ("empty", "scheme", "host", "chars").
bool derive_endpoints(const std::string &base_url, const std::string &model, Endpoints &out, std::string &error);

/// The origin of any http(s)/ws(s) URL, as derive_endpoints reports it; empty when unparseable.
std::string url_origin(const std::string &url);

/// RFC 3986 percent-encoding of everything but unreserved characters.
std::string url_encode_component(const std::string &value);

// ------------------------------------------------------------------------------------------------
// JSON
// ------------------------------------------------------------------------------------------------

/// Finds the first `"key"<ws>:<ws>"` in buf[from, len) and reports the raw (still escaped) value span.
/// Only meant for flat, machine-written payloads such as Realtime audio deltas, where a full parse of
/// tens of kilobytes of base64 would be wasted work. Returns false when absent or unterminated.
bool find_json_string(const char *buf, size_t len, const char *key, size_t &value_start, size_t &value_len,
                      size_t from = 0);

/// The value of the first "type" key in a payload, or empty. For Realtime server events the
/// top-level type is serialised first; nested objects carry types like "message" or "audio/pcm",
/// never an event name, so comparing this against an event name is safe even if it is not first.
std::string first_type(const char *buf, size_t len);

// ------------------------------------------------------------------------------------------------
// Base64
// ------------------------------------------------------------------------------------------------

inline size_t base64_encoded_len(size_t n) { return 4 * ((n + 2) / 3); }

/// Standard alphabet with padding. Writes exactly base64_encoded_len(n) chars, no terminator.
size_t base64_encode(const uint8_t *in, size_t n, char *out);

/// Decodes standard-alphabet base64. Skips backslashes (a JSON encoder may write "\/"), whitespace
/// and stops at padding. Safe to call with out == (uint8_t *) in: output never overtakes input.
/// Returns the number of bytes written; `ok` is false when a character outside the alphabet is seen.
size_t base64_decode(const char *in, size_t n, uint8_t *out, bool &ok);

// ------------------------------------------------------------------------------------------------
// 16 kHz -> 24 kHz
// ------------------------------------------------------------------------------------------------

/// Rational 3/2 polyphase resampler for mono int16, from the Satellite1 microphone's 16 kHz to the
/// Realtime API's 24 kHz PCM. A 96-tap Kaiser-windowed sinc prototype at the 48 kHz intermediate
/// rate (cutoff 7.2 kHz), split into three 32-tap phases; per input pair it emits three outputs.
/// About 0.8 M multiply-adds per second of audio - nothing for the S3's FPU.
class Upsampler2to3 {
 public:
  static constexpr int PHASES = 3;
  static constexpr int TAPS = 32;

  Upsampler2to3();
  void reset();

  /// Upper bound on the samples produced for `n` inputs.
  static size_t max_output(size_t n) { return (n * 3) / 2 + 2; }

  /// Consumes `n` samples, writes the produced samples to `out`, returns how many.
  size_t process(const int16_t *in, size_t n, int16_t *out);

 protected:
  float coef_[PHASES][TAPS];
  float hist_[2 * TAPS];
  int head_{0};
  bool odd_{false};
};

// ------------------------------------------------------------------------------------------------
// Discovery: /models and /audio/voices
// ------------------------------------------------------------------------------------------------

/// A small structural walker over JSON text: enough to visit the elements of one array and read
/// string members of the objects in it, without a DOM. The discovery responses can be hundreds of
/// kilobytes (an OpenAI /models list), and only a few top-level fields matter.
///
/// Positions are byte offsets into `buf`. Every function returns false on malformed input rather
/// than reading past `len`.
bool json_skip_value(const char *buf, size_t len, size_t &i);
/// With `i` at a '{', finds the member `key` at that object's top level and leaves `i` at the
/// start of its value.
bool json_object_member(const char *buf, size_t len, size_t &i, const char *key);
/// With `i` at a '{', reads the string member `key` (unescaped). False when absent or not a string.
bool json_object_string(const char *buf, size_t len, size_t i, const char *key, std::string &out);
/// Unescapes the JSON string literal whose opening quote is at `i` into `out`, leaves `i` past the
/// closing quote. \uXXXX escapes are written as UTF-8 (surrogate pairs combined).
bool json_read_string(const char *buf, size_t len, size_t &i, std::string &out);

/// The model ids a `/models` response lists: `data[].id` (OpenAI and most compatible servers), or
/// `models[].id|name` (some local servers), or a bare top-level array. Only those top-level
/// entries - nested ids such as vLLM's `permission[].id` ("modelperm-...") are not models.
/// Deduplicated, document order.
void collect_model_ids(const char *buf, size_t len, std::vector<std::string> &ids);

/// Keeps the ids that name a Realtime model (contain "realtime" but are not transcription or
/// translation models), sorted; if none do (a local server with its own names), keeps them all,
/// sorted. Caps the list at `max_items`.
void filter_realtime_models(std::vector<std::string> &ids, size_t max_items = 64);

/// One selectable voice. `id` is what a session is configured with; `name` is a label for people
/// (the voice's own name when the server gives one, else empty).
struct VoiceEntry {
  std::string id;
  std::string name;
};

/// The voices a `/audio/voices` response lists. Accepts OpenAI's shape (`data[]` objects with
/// `id` and `name` - custom voices only) and the common local-server shapes: `voices[]` of strings
/// (Kokoro-FastAPI) or of objects with `id`, `voice_id` or `name`, or a bare array. Deduplicated.
void collect_voices(const char *buf, size_t len, std::vector<VoiceEntry> &out, size_t max_items = 96);

/// The Realtime API's built-in voices (developers.openai.com, Realtime conversations guide),
/// recommended ones first. A /audio/voices listing never includes these - it only lists custom
/// voices - so for api.openai.com they are always offered.
extern const char *const BUILTIN_VOICES[];
extern const size_t BUILTIN_VOICE_COUNT;

// ------------------------------------------------------------------------------------------------
// Protocol flavour
// ------------------------------------------------------------------------------------------------

/// The two Realtime wire formats in the wild. GA is api.openai.com today (session.type
/// "realtime", nested audio.input/output, response.output_audio.*). Beta is the earlier format
/// most self-hosted and compatible servers still speak (flat session with input_audio_format,
/// voice at the top level, response.audio.*).
enum class Flavor : uint8_t { UNKNOWN = 0, GA = 1, BETA = 2 };

/// Reads the flavour from a `session.created` payload: GA sessions carry "type":"realtime" or a
/// nested "audio" object; beta sessions carry input_audio_format / modalities. UNKNOWN otherwise.
Flavor detect_flavor(const char *buf, size_t len);

/// "…abcd" for a stored key - never more than the last four characters.
std::string key_hint(const std::string &key);

}  // namespace openai_realtime
}  // namespace esphome
