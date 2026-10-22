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

/// The Realtime URL one configured "base URL" stands for, plus its origin.
///
/// The base URL is the OpenAI SDK's `baseURL` convention - `https://api.openai.com/v1` - because that
/// is what every OpenAI-compatible relay documents. From it:
///   realtime_url = wss://api.openai.com/v1/realtime?model=<model>
/// A base given as ws:// / wss://, or already ending in /realtime, is accepted and mapped back, so a
/// pasted Realtime URL works as well. Query parameters on the base (some relays and Azure use them)
/// are kept on the Realtime URL; `model=` is only added when the base does not already carry one.
/// With `noise_filter` set, `nexus_noise_filter=1` is appended the same way (Nexus's server-side
/// capture denoise — a handshake parameter, outside the OpenAI Realtime protocol schemas) unless the
/// base already pins an explicit value; a false `noise_filter` adds nothing.
struct Endpoints {
  std::string realtime_url;
  std::string origin;  ///< http(s)://host[:port], lowercase host
  bool secure{true};
};

/// Trims whitespace and trailing slashes. Does not validate.
std::string normalize_base_url(const std::string &base_url);

/// Fills `out` and returns true, or returns false with a short reason in `error`
/// ("empty", "scheme", "host", "chars").
bool derive_endpoints(const std::string &base_url, const std::string &model, bool noise_filter,
                      Endpoints &out, std::string &error);

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
// JSON walker
// ------------------------------------------------------------------------------------------------

/// A small structural walker over JSON text: enough to read members of one object without a DOM.
/// Used on the WebSocket task for session.created, where a full parse is not worth it.
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

}  // namespace openai_realtime
}  // namespace esphome
