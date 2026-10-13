// Host unit tests for esphome/components/openai_realtime/rt_util.{h,cpp}.
//   g++ -std=c++17 -O2 -Wall -Wextra -I esphome/components/openai_realtime
//       tests/openai_realtime/test_rt_util.cpp esphome/components/openai_realtime/rt_util.cpp -o /tmp/t && /tmp/t
#include "rt_util.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace esphome::openai_realtime;

static int failures = 0;
#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++; \
    } \
  } while (0)
#define CHECK_EQ(a, b) \
  do { \
    const std::string _a = (a), _b = (b); \
    if (_a != _b) { \
      std::printf("FAIL %s:%d: %s\n   got:  %s\n   want: %s\n", __FILE__, __LINE__, #a, _a.c_str(), _b.c_str()); \
      failures++; \
    } \
  } while (0)

static void test_endpoints() {
  Endpoints ep;
  std::string err;
  CHECK(derive_endpoints("https://api.openai.com/v1", "gpt-realtime-2.1", ep, err));
  CHECK_EQ(ep.realtime_url, "wss://api.openai.com/v1/realtime?model=gpt-realtime-2.1");
  CHECK_EQ(ep.origin, "https://api.openai.com");

  CHECK(derive_endpoints("  https://API.openai.com/v1/// ", "m", ep, err));
  CHECK_EQ(ep.realtime_url, "wss://api.openai.com/v1/realtime?model=m");

  CHECK(derive_endpoints("wss://api.openai.com/v1/realtime", "gpt-realtime", ep, err));
  CHECK_EQ(ep.realtime_url, "wss://api.openai.com/v1/realtime?model=gpt-realtime");

  CHECK(derive_endpoints("http://relay.lan:8080/v1?api-version=2025-08-28", "m x", ep, err));
  CHECK_EQ(ep.realtime_url, "ws://relay.lan:8080/v1/realtime?api-version=2025-08-28&model=m%20x");
  CHECK_EQ(ep.origin, "http://relay.lan:8080");

  CHECK(derive_endpoints("wss://x.example/openai/v1/realtime?model=fixed", "other", ep, err));
  CHECK_EQ(ep.realtime_url, "wss://x.example/openai/v1/realtime?model=fixed");

  CHECK(derive_endpoints("https://host.example", "", ep, err));
  CHECK_EQ(ep.realtime_url, "wss://host.example/realtime");

  CHECK(!derive_endpoints("", "m", ep, err));
  CHECK(!derive_endpoints("ftp://x", "m", ep, err));
  CHECK_EQ(err, "scheme");
  CHECK(!derive_endpoints("https://", "m", ep, err));
  CHECK(!derive_endpoints("https://user:pw@host/v1", "m", ep, err));
  CHECK(!derive_endpoints("https://ho st/v1", "m", ep, err));
}

static void test_json() {
  const char *msg = R"({"type" : "response.output_audio.delta","item_id":"item_1","delta":"AAEC"})";
  size_t s, n;
  CHECK(find_json_string(msg, strlen(msg), "delta", s, n));
  CHECK_EQ(std::string(msg + s, n), "AAEC");
  CHECK(find_json_string(msg, strlen(msg), "item_id", s, n));
  CHECK_EQ(std::string(msg + s, n), "item_1");
  CHECK(!find_json_string(msg, strlen(msg), "missing", s, n));
  CHECK_EQ(first_type(msg, strlen(msg)), "response.output_audio.delta");

  const char *esc = R"({"a":"x\"y","b":"z"})";
  CHECK(find_json_string(esc, strlen(esc), "a", s, n));
  CHECK_EQ(std::string(esc + s, n), "x\\\"y");
  CHECK(find_json_string(esc, strlen(esc), "b", s, n));
  CHECK_EQ(std::string(esc + s, n), "z");
  // A key name appearing as a value must not match.
  const char *trick = R"({"x":"delta","delta":"Zg=="})";
  CHECK(find_json_string(trick, strlen(trick), "delta", s, n));
  CHECK_EQ(std::string(trick + s, n), "Zg==");
}

static void test_base64() {
  const char *cases[][2] = {{"", ""}, {"f", "Zg=="}, {"fo", "Zm8="}, {"foo", "Zm9v"}, {"foobar", "Zm9vYmFy"}};
  for (auto &c : cases) {
    char enc[64];
    const size_t n = base64_encode(reinterpret_cast<const uint8_t *>(c[0]), strlen(c[0]), enc);
    CHECK_EQ(std::string(enc, n), c[1]);
    uint8_t dec[64];
    bool ok;
    const size_t m = base64_decode(c[1], strlen(c[1]), dec, ok);
    CHECK(ok);
    CHECK_EQ(std::string(reinterpret_cast<char *>(dec), m), c[0]);
  }
  // Round trip of all byte values, decoded in place.
  std::vector<uint8_t> data(1000);
  for (size_t i = 0; i < data.size(); i++)
    data[i] = static_cast<uint8_t>(i * 7 + 3);
  std::string enc(base64_encoded_len(data.size()), '\0');
  base64_encode(data.data(), data.size(), &enc[0]);
  bool ok;
  const size_t m = base64_decode(enc.data(), enc.size(), reinterpret_cast<uint8_t *>(&enc[0]), ok);
  CHECK(ok);
  CHECK(m == data.size());
  CHECK(memcmp(enc.data(), data.data(), m) == 0);
  // JSON-escaped slash.
  uint8_t dec[8];
  const size_t k = base64_decode("\\/w==", 5, dec, ok);
  CHECK(ok && k == 1 && dec[0] == 0xFF);
}

static double tone_error(int freq) {
  Upsampler2to3 up;
  const int n_in = 16000;
  std::vector<int16_t> in(n_in);
  for (int i = 0; i < n_in; i++)
    in[i] = static_cast<int16_t>(std::lrint(16000.0 * std::sin(2 * M_PI * freq * i / 16000.0)));
  std::vector<int16_t> out(Upsampler2to3::max_output(n_in));
  // Feed in odd-sized chunks to exercise the streaming state.
  size_t o = 0;
  for (int i = 0; i < n_in;) {
    const int chunk = std::min(n_in - i, 37 + (i % 5));
    o += up.process(&in[i], chunk, &out[o]);
    i += chunk;
  }
  if (o != static_cast<size_t>(n_in * 3 / 2))
    return 1e9;
  // Best-fit delay against the ideal 24 kHz tone, measured away from the start-up transient.
  double best = 1e18;
  // The filter's group delay is (96 - 1) / 2 samples at 48 kHz = 23.75 samples at 24 kHz, a
  // fractional delay, so the fit steps in quarter samples.
  for (double d = 0; d < 40; d += 0.25) {
    double err = 0, sig = 0;
    for (size_t i = 2000; i < o - 200; i++) {
      const double ref = 16000.0 * std::sin(2 * M_PI * freq * (double(i) - d) / 24000.0);
      err += (out[i] - ref) * (out[i] - ref);
      sig += ref * ref;
    }
    best = std::min(best, err / sig);
  }
  return best;
}

static void test_upsampler() {
  for (int f : {200, 1000, 3000, 5000}) {
    const double e = tone_error(f);
    std::printf("  upsampler %5d Hz: relative error %.2e (%.1f dB)\n", f, e, 10 * std::log10(e));
    CHECK(e < 1e-6);  // better than -60 dB across the speech band
  }
  // DC passes at unity on every phase.
  Upsampler2to3 up;
  std::vector<int16_t> in(400, 1000), out(Upsampler2to3::max_output(400));
  const size_t o = up.process(in.data(), in.size(), out.data());
  CHECK(o == 600);
  for (size_t i = 100; i < o; i++)
    CHECK(std::abs(out[i] - 1000) <= 1);
}

static void test_walker() {
  const char *j = R"({"a":{"x":[1,"]}",{"y":"}"}]},"b" : "q\"é😀\/","c":true})";
  size_t i = 0;
  CHECK(json_object_member(j, strlen(j), i, "b"));
  std::string v;
  CHECK(json_read_string(j, strlen(j), i, v));
  CHECK_EQ(v, "q\"\xC3\xA9\xF0\x9F\x98\x80/");
  i = 0;
  CHECK(json_object_member(j, strlen(j), i, "c"));
  i = 0;
  CHECK(!json_object_member(j, strlen(j), i, "zz"));
  // Deep nesting is walked iteratively.
  std::string deep(20000, '[');
  deep += std::string(20000, ']');
  i = 0;
  CHECK(json_skip_value(deep.data(), deep.size(), i));
  CHECK(i == deep.size());
  std::string open_only(100, '[');
  i = 0;
  CHECK(!json_skip_value(open_only.data(), open_only.size(), i));
}

static void test_flavor() {
  const char *ga = R"({"type":"session.created","event_id":"e","session":{"type":"realtime","object":)"
                   R"("realtime.session","audio":{"input":{"format":{"type":"audio/pcm","rate":24000}}}}})";
  CHECK(detect_flavor(ga, strlen(ga)) == Flavor::GA);
  const char *beta = R"({"type":"session.created","session":{"id":"sess_1","object":"realtime.session",)"
                     R"("modalities":["audio","text"],"voice":"alloy","input_audio_format":"pcm16"}})";
  CHECK(detect_flavor(beta, strlen(beta)) == Flavor::BETA);
  const char *bare = R"({"type":"session.created","session":{"id":"x"}})";
  CHECK(detect_flavor(bare, strlen(bare)) == Flavor::UNKNOWN);
  CHECK(detect_flavor("{}", 2) == Flavor::UNKNOWN);
}

int main() {
  test_endpoints();
  test_json();
  test_base64();
  test_upsampler();
  test_walker();
  test_flavor();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all rt_util tests passed\n");
  return 0;
}
