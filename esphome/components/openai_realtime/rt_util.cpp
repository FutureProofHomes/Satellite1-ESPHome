#include "rt_util.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <initializer_list>

namespace esphome {
namespace openai_realtime {

// ------------------------------------------------------------------------------------------------
// Endpoints
// ------------------------------------------------------------------------------------------------

static bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

std::string normalize_base_url(const std::string &base_url) {
  size_t a = 0, b = base_url.size();
  while (a < b && is_ws(base_url[a]))
    a++;
  while (b > a && is_ws(base_url[b - 1]))
    b--;
  std::string s = base_url.substr(a, b - a);
  // Trailing slashes only on the path part; a query string is left alone.
  const size_t q = s.find('?');
  std::string path = q == std::string::npos ? s : s.substr(0, q);
  const std::string query = q == std::string::npos ? "" : s.substr(q);
  while (!path.empty() && path.back() == '/')
    path.pop_back();
  return path + query;
}

static bool starts_with_ci(const std::string &s, const char *prefix) {
  const size_t n = strlen(prefix);
  if (s.size() < n)
    return false;
  for (size_t i = 0; i < n; i++) {
    if (std::tolower(static_cast<unsigned char>(s[i])) != prefix[i])
      return false;
  }
  return true;
}

static bool ends_with(const std::string &s, const char *suffix) {
  const size_t n = strlen(suffix);
  return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

std::string url_encode_component(const std::string &value) {
  static const char HEX[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(value.size());
  for (unsigned char c : value) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += HEX[c >> 4];
      out += HEX[c & 0xF];
    }
  }
  return out;
}

bool derive_endpoints(const std::string &base_url, const std::string &model, Endpoints &out, std::string &error) {
  const std::string s = normalize_base_url(base_url);
  if (s.empty()) {
    error = "empty";
    return false;
  }
  for (unsigned char c : s) {
    if (c <= 0x20 || c >= 0x7f || c == '"' || c == '\\' || c == '<' || c == '>') {
      error = "chars";
      return false;
    }
  }

  size_t skip;
  if (starts_with_ci(s, "https://")) {
    out.secure = true;
    skip = 8;
  } else if (starts_with_ci(s, "wss://")) {
    out.secure = true;
    skip = 6;
  } else if (starts_with_ci(s, "http://")) {
    out.secure = false;
    skip = 7;
  } else if (starts_with_ci(s, "ws://")) {
    out.secure = false;
    skip = 5;
  } else {
    error = "scheme";
    return false;
  }

  std::string rest = s.substr(skip);
  std::string query;
  const size_t q = rest.find('?');
  if (q != std::string::npos) {
    query = rest.substr(q + 1);
    rest = rest.substr(0, q);
  }
  // Fragments have no business in an API URL.
  const size_t hash = query.find('#');
  if (hash != std::string::npos)
    query = query.substr(0, hash);

  const size_t slash = rest.find('/');
  std::string host = slash == std::string::npos ? rest : rest.substr(0, slash);
  std::string path = slash == std::string::npos ? "" : rest.substr(slash);
  if (host.empty() || host.find('@') != std::string::npos) {
    // Userinfo is refused: credentials belong in the API key field, not in a URL that is echoed back.
    error = "host";
    return false;
  }
  if (host.back() == ':') {
    error = "host";
    return false;
  }
  for (auto &c : host)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  std::string base_path = path;
  std::string rt_path;
  if (ends_with(path, "/realtime")) {
    base_path = path.substr(0, path.size() - strlen("/realtime"));
    rt_path = path;
  } else {
    rt_path = path + "/realtime";
  }

  std::string rt_query = query;
  const bool has_model = ("&" + query).find("&model=") != std::string::npos;
  if (!has_model && !model.empty()) {
    if (!rt_query.empty())
      rt_query += '&';
    rt_query += "model=" + url_encode_component(model);
  }

  out.realtime_url = std::string(out.secure ? "wss://" : "ws://") + host + rt_path;
  if (!rt_query.empty())
    out.realtime_url += "?" + rt_query;
  out.models_url = std::string(out.secure ? "https://" : "http://") + host + base_path + "/models";
  out.voices_url = std::string(out.secure ? "https://" : "http://") + host + base_path + "/audio/voices";
  out.origin = std::string(out.secure ? "https://" : "http://") + host;
  return true;
}

std::string url_origin(const std::string &url) {
  Endpoints ep;
  std::string err;
  if (!derive_endpoints(url, "", ep, err))
    return "";
  return ep.origin;
}

// ------------------------------------------------------------------------------------------------
// JSON
// ------------------------------------------------------------------------------------------------

bool find_json_string(const char *buf, size_t len, const char *key, size_t &value_start, size_t &value_len,
                      size_t from) {
  const size_t klen = strlen(key);
  size_t i = from;
  while (i + klen + 2 < len) {
    const char *p = static_cast<const char *>(memchr(buf + i, '"', len - i));
    if (p == nullptr)
      return false;
    size_t k = static_cast<size_t>(p - buf);
    if (k + klen + 2 <= len && memcmp(buf + k + 1, key, klen) == 0 && buf[k + klen + 1] == '"') {
      size_t j = k + klen + 2;
      while (j < len && is_ws(buf[j]))
        j++;
      if (j < len && buf[j] == ':') {
        j++;
        while (j < len && is_ws(buf[j]))
          j++;
        if (j < len && buf[j] == '"') {
          const size_t start = j + 1;
          size_t e = start;
          while (e < len) {
            if (buf[e] == '\\') {
              e += 2;
              continue;
            }
            if (buf[e] == '"')
              break;
            e++;
          }
          if (e >= len)
            return false;
          value_start = start;
          value_len = e - start;
          return true;
        }
      }
    }
    i = k + 1;
  }
  return false;
}

std::string first_type(const char *buf, size_t len) {
  size_t s, n;
  // The top-level type sits in the first few hundred bytes; bounding the scan keeps a malformed
  // payload from costing a walk over a large delta.
  const size_t bound = len < 512 ? len : 512;
  if (!find_json_string(buf, bound, "type", s, n))
    return "";
  return std::string(buf + s, n);
}

// ------------------------------------------------------------------------------------------------
// Base64
// ------------------------------------------------------------------------------------------------

static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64_encode(const uint8_t *in, size_t n, char *out) {
  size_t o = 0;
  size_t i = 0;
  for (; i + 2 < n; i += 3) {
    const uint32_t v = (uint32_t(in[i]) << 16) | (uint32_t(in[i + 1]) << 8) | in[i + 2];
    out[o++] = B64[(v >> 18) & 63];
    out[o++] = B64[(v >> 12) & 63];
    out[o++] = B64[(v >> 6) & 63];
    out[o++] = B64[v & 63];
  }
  if (i < n) {
    uint32_t v = uint32_t(in[i]) << 16;
    if (i + 1 < n)
      v |= uint32_t(in[i + 1]) << 8;
    out[o++] = B64[(v >> 18) & 63];
    out[o++] = B64[(v >> 12) & 63];
    out[o++] = i + 1 < n ? B64[(v >> 6) & 63] : '=';
    out[o++] = '=';
  }
  return o;
}

static int8_t b64_value(unsigned char c) {
  if (c >= 'A' && c <= 'Z')
    return static_cast<int8_t>(c - 'A');
  if (c >= 'a' && c <= 'z')
    return static_cast<int8_t>(c - 'a' + 26);
  if (c >= '0' && c <= '9')
    return static_cast<int8_t>(c - '0' + 52);
  if (c == '+' || c == '-')
    return 62;
  if (c == '/' || c == '_')
    return 63;
  return -1;
}

size_t base64_decode(const char *in, size_t n, uint8_t *out, bool &ok) {
  ok = true;
  uint32_t acc = 0;
  int bits = 0;
  size_t o = 0;
  for (size_t i = 0; i < n; i++) {
    const unsigned char c = static_cast<unsigned char>(in[i]);
    if (c == '=')
      break;
    if (c == '\\' || is_ws(static_cast<char>(c)))
      continue;
    const int8_t v = b64_value(c);
    if (v < 0) {
      ok = false;
      continue;
    }
    acc = (acc << 6) | static_cast<uint32_t>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      // o <= 3 * i / 4 < i, so writing in place never overtakes the input still to be read.
      out[o++] = static_cast<uint8_t>((acc >> bits) & 0xFF);
    }
  }
  return o;
}

// ------------------------------------------------------------------------------------------------
// Upsampler
// ------------------------------------------------------------------------------------------------

static double bessel_i0(double x) {
  double sum = 1.0, term = 1.0;
  const double q = x * x / 4.0;
  for (int k = 1; k < 40; k++) {
    term *= q / (double(k) * double(k));
    sum += term;
    if (term < 1e-12 * sum)
      break;
  }
  return sum;
}

Upsampler2to3::Upsampler2to3() {
  constexpr int N = PHASES * TAPS;  // 96 taps at the 48 kHz intermediate rate
  constexpr double FS = 48000.0;
  constexpr double FC = 7200.0;
  constexpr double BETA = 7.0;
  const double center = (N - 1) / 2.0;
  const double wc = 2.0 * FC / FS;
  const double i0b = bessel_i0(BETA);
  double h[N];
  for (int i = 0; i < N; i++) {
    const double t = i - center;
    const double x = M_PI * wc * t;
    const double sinc = std::fabs(t) < 1e-9 ? 1.0 : std::sin(x) / x;
    const double r = t / center;
    const double w = bessel_i0(BETA * std::sqrt(std::max(0.0, 1.0 - r * r))) / i0b;
    h[i] = wc * sinc * w;
  }
  // Each phase is normalised to unity DC gain on its own, so a constant input stays constant on
  // every output sample rather than rippling at the phase rate.
  for (int p = 0; p < PHASES; p++) {
    double sum = 0.0;
    for (int k = 0; k < TAPS; k++)
      sum += h[p + PHASES * k];
    for (int k = 0; k < TAPS; k++)
      this->coef_[p][k] = static_cast<float>(h[p + PHASES * k] / sum);
  }
  this->reset();
}

void Upsampler2to3::reset() {
  memset(this->hist_, 0, sizeof(this->hist_));
  this->head_ = 0;
  this->odd_ = false;
}

static inline int16_t clamp16(float v) {
  if (v > 32767.0f)
    return 32767;
  if (v < -32768.0f)
    return -32768;
  return static_cast<int16_t>(std::lrintf(v));
}

size_t Upsampler2to3::process(const int16_t *in, size_t n, int16_t *out) {
  size_t o = 0;
  for (size_t i = 0; i < n; i++) {
    // Newest sample at head_, older ones following; the doubled buffer keeps the window contiguous.
    this->head_ = (this->head_ + TAPS - 1) % TAPS;
    this->hist_[this->head_] = this->hist_[this->head_ + TAPS] = static_cast<float>(in[i]);
    const float *x = &this->hist_[this->head_];

    // Upsampled index u runs 3 per input sample; outputs are the even u. For an even input sample n
    // that is u = 3n (phase 0) and u = 3n + 2 (phase 2); for an odd one, u = 3n + 1 (phase 1).
    const int first = this->odd_ ? 1 : 0;
    const int count = this->odd_ ? 1 : 2;
    for (int j = 0; j < count; j++) {
      const int p = first + 2 * j;
      const float *c = this->coef_[p];
      float acc = 0.0f;
      for (int k = 0; k < TAPS; k++)
        acc += c[k] * x[k];
      out[o++] = clamp16(acc);
    }
    this->odd_ = !this->odd_;
  }
  return o;
}

// ------------------------------------------------------------------------------------------------
// JSON walker
// ------------------------------------------------------------------------------------------------

static void skip_ws(const char *buf, size_t len, size_t &i) {
  while (i < len && is_ws(buf[i]))
    i++;
}

static void utf8_append(std::string &out, uint32_t cp) {
  if (cp < 0x80) {
    out += static_cast<char>(cp);
  } else if (cp < 0x800) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    out += static_cast<char>(0xE0 | (cp >> 12));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    out += static_cast<char>(0xF0 | (cp >> 18));
    out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

static bool hex4(const char *buf, size_t len, size_t i, uint32_t &v) {
  if (i + 4 > len)
    return false;
  v = 0;
  for (size_t k = 0; k < 4; k++) {
    const char c = buf[i + k];
    v <<= 4;
    if (c >= '0' && c <= '9')
      v |= static_cast<uint32_t>(c - '0');
    else if (c >= 'a' && c <= 'f')
      v |= static_cast<uint32_t>(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F')
      v |= static_cast<uint32_t>(c - 'A' + 10);
    else
      return false;
  }
  return true;
}

static bool read_string_into(const char *buf, size_t len, size_t &i, std::string &out);

bool json_read_string(const char *buf, size_t len, size_t &i, std::string &out) {
  // Into a temporary: a truncated or malformed literal leaves `out` (and `i`) untouched.
  std::string tmp;
  size_t j = i;
  if (!read_string_into(buf, len, j, tmp))
    return false;
  out.swap(tmp);
  i = j;
  return true;
}

static bool read_string_into(const char *buf, size_t len, size_t &i, std::string &out) {
  if (i >= len || buf[i] != '"')
    return false;
  i++;
  while (i < len) {
    const char c = buf[i];
    if (c == '"') {
      i++;
      return true;
    }
    if (c != '\\') {
      out += c;
      i++;
      continue;
    }
    if (i + 1 >= len)
      return false;
    const char e = buf[i + 1];
    i += 2;
    switch (e) {
      case '"':
      case '\\':
      case '/':
        out += e;
        break;
      case 'b':
        out += '\b';
        break;
      case 'f':
        out += '\f';
        break;
      case 'n':
        out += '\n';
        break;
      case 'r':
        out += '\r';
        break;
      case 't':
        out += '\t';
        break;
      case 'u': {
        uint32_t cp;
        if (!hex4(buf, len, i, cp))
          return false;
        i += 4;
        if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= len && buf[i] == '\\' && buf[i + 1] == 'u') {
          uint32_t lo;
          if (hex4(buf, len, i + 2, lo) && lo >= 0xDC00 && lo < 0xE000) {
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
            i += 6;
          }
        }
        utf8_append(out, cp);
        break;
      }
      default:
        return false;
    }
  }
  return false;
}

bool json_skip_value(const char *buf, size_t len, size_t &i) {
  skip_ws(buf, len, i);
  if (i >= len)
    return false;
  const char c = buf[i];
  if (c == '"') {
    i++;
    while (i < len) {
      if (buf[i] == '\\') {
        i += 2;
        continue;
      }
      if (buf[i] == '"') {
        i++;
        return true;
      }
      i++;
    }
    return false;
  }
  if (c == '{' || c == '[') {
    // Iterative, so a hostile server cannot exhaust the stack with nesting; strings are skipped
    // whole so brackets inside them do not count.
    int depth = 0;
    while (i < len) {
      const char d = buf[i];
      if (d == '"') {
        if (!json_skip_value(buf, len, i))
          return false;
        continue;
      }
      if (d == '{' || d == '[') {
        depth++;
      } else if (d == '}' || d == ']') {
        depth--;
        if (depth == 0) {
          i++;
          return true;
        }
      }
      i++;
    }
    return false;
  }
  // Number, true, false, null.
  const size_t start = i;
  while (i < len && buf[i] != ',' && buf[i] != '}' && buf[i] != ']' && !is_ws(buf[i]))
    i++;
  return i > start;
}

bool json_object_member(const char *buf, size_t len, size_t &i, const char *key) {
  skip_ws(buf, len, i);
  if (i >= len || buf[i] != '{')
    return false;
  i++;
  std::string k;
  while (true) {
    skip_ws(buf, len, i);
    if (i >= len)
      return false;
    if (buf[i] == '}')
      return false;
    if (!json_read_string(buf, len, i, k))
      return false;
    skip_ws(buf, len, i);
    if (i >= len || buf[i] != ':')
      return false;
    i++;
    skip_ws(buf, len, i);
    if (k == key)
      return true;
    if (!json_skip_value(buf, len, i))
      return false;
    skip_ws(buf, len, i);
    if (i < len && buf[i] == ',') {
      i++;
      continue;
    }
    return false;
  }
}

bool json_object_string(const char *buf, size_t len, size_t i, const char *key, std::string &out) {
  if (!json_object_member(buf, len, i, key))
    return false;
  return json_read_string(buf, len, i, out);
}

/// Calls `f(pos)` with `pos` at each element of the array that starts at `i`. Stops early when `f`
/// returns false.
template<typename F> static bool json_each(const char *buf, size_t len, size_t i, F f) {
  skip_ws(buf, len, i);
  if (i >= len || buf[i] != '[')
    return false;
  i++;
  while (true) {
    skip_ws(buf, len, i);
    if (i >= len)
      return false;
    if (buf[i] == ']')
      return true;
    const size_t at = i;
    if (!f(at))
      return true;
    if (!json_skip_value(buf, len, i))
      return false;
    skip_ws(buf, len, i);
    if (i < len && buf[i] == ',') {
      i++;
      continue;
    }
    return i < len && buf[i] == ']';
  }
}

/// The array a discovery response lists its entries in: the first of `keys` present at the top
/// level of the root object, or the root itself when it is an array. Its start, or npos.
static size_t list_start(const char *buf, size_t len, std::initializer_list<const char *> keys) {
  size_t i = 0;
  skip_ws(buf, len, i);
  if (i >= len)
    return std::string::npos;
  if (buf[i] == '[')
    return i;
  for (const char *key : keys) {
    size_t j = i;
    if (json_object_member(buf, len, j, key) && j < len && buf[j] == '[')
      return j;
  }
  return std::string::npos;
}

static bool plain_token(const std::string &s, size_t max) {
  if (s.empty() || s.size() > max)
    return false;
  for (unsigned char c : s) {
    if (c <= 0x20 || c == '"' || c == '\\' || c == 0x7f)
      return false;
  }
  return true;
}

// ------------------------------------------------------------------------------------------------
// Discovery
// ------------------------------------------------------------------------------------------------

void collect_model_ids(const char *buf, size_t len, std::vector<std::string> &ids) {
  const size_t at = list_start(buf, len, {"data", "models"});
  if (at == std::string::npos)
    return;
  json_each(buf, len, at, [&](size_t pos) {
    std::string id;
    if (buf[pos] == '"') {
      size_t p = pos;
      json_read_string(buf, len, p, id);
    } else if (buf[pos] == '{') {
      if (!json_object_string(buf, len, pos, "id", id))
        json_object_string(buf, len, pos, "name", id);
    }
    if (plain_token(id, 96) && std::find(ids.begin(), ids.end(), id) == ids.end())
      ids.push_back(id);
    return ids.size() < 512;
  });
}

void filter_realtime_models(std::vector<std::string> &ids, size_t max_items) {
  std::vector<std::string> rt;
  for (const auto &id : ids) {
    if (id.find("realtime") != std::string::npos && id.find("transcri") == std::string::npos &&
        id.find("translat") == std::string::npos)
      rt.push_back(id);
  }
  if (!rt.empty())
    ids.swap(rt);
  std::sort(ids.begin(), ids.end());
  if (ids.size() > max_items)
    ids.resize(max_items);
}

void collect_voices(const char *buf, size_t len, std::vector<VoiceEntry> &out, size_t max_items) {
  const size_t at = list_start(buf, len, {"data", "voices"});
  if (at == std::string::npos)
    return;
  json_each(buf, len, at, [&](size_t pos) {
    VoiceEntry v;
    if (buf[pos] == '"') {
      size_t p = pos;
      json_read_string(buf, len, p, v.id);
    } else if (buf[pos] == '{') {
      if (!json_object_string(buf, len, pos, "id", v.id) && !json_object_string(buf, len, pos, "voice_id", v.id))
        json_object_string(buf, len, pos, "name", v.id);
      json_object_string(buf, len, pos, "name", v.name);
      if (v.name == v.id || v.name.size() > 64)
        v.name.clear();
    }
    if (plain_token(v.id, 63)) {
      bool dup = false;
      for (const auto &e : out)
        dup = dup || e.id == v.id;
      if (!dup)
        out.push_back(std::move(v));
    }
    return out.size() < max_items;
  });
}

const char *const BUILTIN_VOICES[] = {"marin", "cedar", "alloy", "ash",  "ballad",
                                      "coral", "echo",  "sage",  "shimmer", "verse"};
const size_t BUILTIN_VOICE_COUNT = sizeof(BUILTIN_VOICES) / sizeof(BUILTIN_VOICES[0]);

Flavor detect_flavor(const char *buf, size_t len) {
  size_t i = 0;
  if (!json_object_member(buf, len, i, "session") || i >= len || buf[i] != '{')
    return Flavor::UNKNOWN;
  std::string type;
  if (json_object_string(buf, len, i, "type", type) && type == "realtime")
    return Flavor::GA;
  size_t j = i;
  if (json_object_member(buf, len, j, "audio"))
    return Flavor::GA;
  j = i;
  if (json_object_member(buf, len, j, "input_audio_format"))
    return Flavor::BETA;
  j = i;
  if (json_object_member(buf, len, j, "modalities"))
    return Flavor::BETA;
  return Flavor::UNKNOWN;
}

std::string key_hint(const std::string &key) {
  if (key.empty())
    return "";
  if (key.size() <= 8)
    return "\xE2\x80\xA6";  // too short to show any of it
  return std::string("\xE2\x80\xA6") + key.substr(key.size() - 4);
}

}  // namespace openai_realtime
}  // namespace esphome
