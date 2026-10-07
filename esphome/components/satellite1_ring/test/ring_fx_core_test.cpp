// Host test for ring_fx_core.h, built with g++ in CI (lint.yaml, "LED ring renderer parity"):
//
//   g++ -std=c++17 -O1 -o /tmp/ring_fx_core_test esphome/components/satellite1_ring/test/ring_fx_core_test.cpp
//   /tmp/ring_fx_core_test esphome/components/satellite1_web_ui/frontend/test/ringfx-golden.txt
//
// Three checks. Parity: every frame lib/ringfx.js wrote to the golden file, re-rendered here, within
// two units per channel (float here, double in the browser). Classic: the Classic style against
// re-implementations of the effect lambdas it replaced in config/common/led_ring.yaml (Rotating
// Blob, Thinking, Error, Timer Ring, Timer Tick, Volume Display, Muted or Silent), exactly, frame
// by frame at their old update intervals. Clamp: the flags clamp_style owns, since it is what
// sanitizes the styles loaded back from flash.

#include "../ring_fx_core.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace esphome::satellite1_ring;

static int failures = 0;

static void fail(const std::string &what, const Frame got, const Frame want) {
  if (++failures > 12)
    return;
  std::printf("FAIL %s\n", what.c_str());
  for (int i = 0; i < N; i++) {
    if (got[i][0] != want[i][0] || got[i][1] != want[i][1] || got[i][2] != want[i][2])
      std::printf("  led %2d: got %3d %3d %3d want %3d %3d %3d\n", i, got[i][0], got[i][1], got[i][2], want[i][0],
                  want[i][1], want[i][2]);
  }
}

static bool close(const Frame a, const Frame b, int tol) {
  for (int i = 0; i < N; i++)
    for (int k = 0; k < 3; k++) {
      int d = int(a[i][k]) - int(b[i][k]);
      if (d > tol || d < -tol)
        return false;
    }
  return true;
}

static void check(const std::string &what, const Frame got, const Frame want, int tol) {
  if (!close(got, want, tol))
    fail(what, got, want);
}

static int index_of(const char *const *names, int n, const std::string &key) {
  for (int i = 0; i < n; i++)
    if (key == names[i])
      return i;
  return -1;
}

static const char *const MOMENT_NAMES[M_COUNT] = {"wake", "listen", "think", "reply", "timer",
                                                  "ring", "vol",    "mute",  "err"};
static const char *const PRESET_NAMES[P_COUNT] = {"classic", "calm", "aurora", "party", "minimal"};

static void parse_frame(const std::string &hex, Frame out) {
  for (int i = 0; i < N * 3; i++)
    out[i / 3][i % 3] = uint8_t(std::stoi(hex.substr(i * 2, 2), nullptr, 16));
}

static Inputs read_inputs(std::istringstream &in) {
  Inputs v;
  int r, g, b, on, bp, mic, spk;
  in >> r >> g >> b >> on >> bp >> mic >> spk;
  v.ring[0] = uint8_t(r);
  v.ring[1] = uint8_t(g);
  v.ring[2] = uint8_t(b);
  v.ring_on = on;
  v.ratio = float(bp) / 10000.0f;
  v.mic_muted = mic;
  v.spk_silent = spk;
  return v;
}

static int run_golden(const char *path) {
  std::ifstream f(path);
  if (!f) {
    std::printf("cannot open %s\n", path);
    return 1;
  }
  std::string line;
  int n = 0;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;
    std::istringstream in(line);
    std::string kind, hex;
    in >> kind;
    Frame got, want;
    if (kind == "P") {
      std::string preset, moment;
      uint32_t t;
      int head0;
      in >> preset >> moment >> t >> head0;
      Inputs v = read_inputs(in);
      in >> hex;
      int p = index_of(PRESET_NAMES, P_COUNT, preset), m = index_of(MOMENT_NAMES, M_COUNT, moment);
      if (p < 0 || m < 0) {
        std::printf("bad line: %s\n", line.c_str());
        return 1;
      }
      render_moment(m, PRESETS[p][m], t, head0, v, got);
    } else if (kind == "C") {
      std::string moment;
      int fx, cm, nn, fl, sp, br, pp, c[9];
      uint32_t t;
      int head0;
      in >> moment >> fx >> cm >> nn >> fl >> sp >> br >> pp;
      for (int &x : c)
        in >> x;
      in >> t >> head0;
      Inputs v = read_inputs(in);
      in >> hex;
      Style s{uint8_t(fx), uint8_t(cm), uint8_t(nn), uint8_t(fl), uint16_t(sp), uint8_t(br), uint8_t(pp), {}};
      for (int i = 0; i < 9; i++)
        s.c[i / 3][i % 3] = uint8_t(c[i]);
      render_moment(index_of(MOMENT_NAMES, M_COUNT, moment), s, t, head0, v, got);
    } else {
      std::printf("bad line: %s\n", line.c_str());
      return 1;
    }
    parse_frame(hex, want);
    check("golden: " + line.substr(0, line.size() - N * 6), got, want, 2);
    n++;
  }
  std::printf("golden: %d frames\n", n);
  return 0;
}

// ---- The lambdas Classic replaced, as they were ----

static void put(uint8_t out[3], const uint8_t c[3]) { memcpy(out, c, 3); }
static void put8(uint8_t out[3], const uint8_t c[3], uint8_t k) {
  for (int i = 0; i < 3; i++)
    out[i] = scale8(c[i], k);
}
static void legacy_mic_cap(Frame f) {
  for (int m : MIC_LEDS) {
    uint8_t mx = f[m][0] > f[m][2] ? f[m][0] : f[m][2];
    mx = mx > f[m][1] ? mx : f[m][1];
    if (mx > 128) {
      float scale = 128.f * 255.f / float(mx) + .5;
      uint8_t s8 = (scale > 255.f) ? 255 : uint8_t(scale);
      put8(f[m], f[m], s8);
    }
  }
}

static void classic_spin(int moment, float speed, const uint8_t ring[3]) {
  Inputs v;
  memcpy(v.ring, ring, 3);
  v.ring_on = true;
  for (int p0 : {0, 5, 17}) {
    float pos = float(p0);
    for (int k = 0; k < 600; k++) {
      Frame want, got;
      memset(want, 0, sizeof(want));
      auto add_blob = [&](float center) {
        int p = int(floorf(center)) % 24;
        if (p < 0)
          p += 24;
        put(want[p], ring);
        put8(want[(p + 23) % 24], ring, 192);
        put8(want[(p + 22) % 24], ring, 128);
      };
      add_blob(pos);
      add_blob(pos + 12.0f);
      legacy_mic_cap(want);
      render_moment(moment, PRESETS[P_CLASSIC][moment], 50 * k + (k * 7) % 50, p0, v, got);
      check("classic spin " + std::string(MOMENT_NAMES[moment]) + " frame " + std::to_string(k), got, want, 0);
      pos += speed;
      while (pos >= 24.0f)
        pos -= 24.0f;
      while (pos < 0.0f)
        pos += 24.0f;
    }
  }
}

// Thinking, Error and Timer Ring: one shared 10 ms triangle.
static void classic_triangle(int moment, const uint8_t ring[3], bool mic_muted) {
  Inputs v;
  memcpy(v.ring, ring, 3);
  v.ring_on = true;
  v.mic_muted = mic_muted;
  const uint8_t red[3] = {255, 0, 0};
  uint8_t step = 0;
  bool decreasing = true;
  for (int k = 0; k < 400; k++) {
    Frame want, got;
    memset(want, 0, sizeof(want));
    uint8_t b = 255 / 10 * (10 - step);
    if (moment == M_THINK) {
      put8(want[2], ring, b);
      put8(want[14], ring, b);
    } else {
      for (int i = 0; i < 24; i++)
        put8(want[i], moment == M_ERR ? red : ring, b);
      if (moment == M_RING && mic_muted) {
        put(want[3], red);
        put(want[9], red);
      }
    }
    render_moment(moment, PRESETS[P_CLASSIC][moment], 10 * k + k % 10, 0, v, got);
    check("classic " + std::string(MOMENT_NAMES[moment]) + " frame " + std::to_string(k), got, want, 0);
    step = decreasing ? step + 1 : step - 1;
    if (step == 0 || step == 10)
      decreasing = !decreasing;
  }
}

static void classic_timer(float ratio, const uint8_t ring[3], bool mic_muted) {
  Inputs v;
  memcpy(v.ring, ring, 3);
  v.ring_on = true;
  v.ratio = ratio;
  v.mic_muted = mic_muted;
  const uint8_t red[3] = {255, 0, 0}, black[3] = {0, 0, 0};
  int g = 0;
  for (int k = 0; k < 120; k++) {
    Frame want, got;
    float timer_ratio = 24.0f * ratio;
    uint8_t last_led_on = static_cast<uint8_t>(ceil(timer_ratio)) - 1;
    for (int i = 0; i < 24; i++) {
      float dip = (i == g % 24 && i != last_led_on) ? 0.9f : 1.0f;
      if (i <= timer_ratio) {
        float a = 255.0f * dip * (timer_ratio - i), b = 255.0f * dip;
        put8(want[i], ring, uint8_t(a < b ? a : b));
      } else {
        put(want[i], black);
      }
    }
    if (mic_muted) {
      for (int i : {2, 4, 8, 10})
        put(want[i], black);
      put(want[3], red);
      put(want[9], red);
    }
    g = (24 + g - 1) % 24;
    render_moment(M_TIMER, PRESETS[P_CLASSIC][M_TIMER], 100 * k + k % 100, 0, v, got);
    check("classic timer " + std::to_string(ratio) + " frame " + std::to_string(k), got, want, 0);
  }
}

static void classic_volume(float volume, const uint8_t ring[3]) {
  Inputs v;
  memcpy(v.ring, ring, 3);
  v.ring_on = true;
  v.ratio = volume;
  Frame want, got;
  float volume_ratio = 24.0f * volume;
  for (int i = 0; i < 24; i++) {
    if (i <= volume_ratio) {
      float a = 255.0f * (volume_ratio - i);
      put8(want[i], ring, uint8_t(a < 255.0f ? a : 255.0f));
    } else {
      memset(want[i], 0, 3);
    }
  }
  if (volume == 0.0f) {
    want[0][0] = 255;
    want[0][1] = want[0][2] = 0;
  }
  for (uint32_t t : {0u, 1234u, 99999u}) {
    render_moment(M_VOL, PRESETS[P_CLASSIC][M_VOL], t, 0, v, got);
    check("classic volume " + std::to_string(volume), got, want, 0);
  }
}

static void classic_mute(const uint8_t ring[3], bool on, bool mic, bool spk) {
  Inputs v;
  memcpy(v.ring, ring, 3);
  v.ring_on = on;
  v.mic_muted = mic;
  v.spk_silent = spk;
  const uint8_t mic_c[3] = {255, 0, 0}, spk_c[3] = {200, 0, 0}, black[3] = {0, 0, 0};
  Frame want, got;
  for (int i = 0; i < 24; i++)
    put(want[i], on ? ring : black);
  if (mic) {
    for (int c : {0, 6, 12, 18}) {
      put(want[(c + 23) % 24], black);
      put(want[c], mic_c);
      put(want[(c + 1) % 24], black);
    }
  }
  if (spk) {
    for (int s : {1, 7, 13, 19}) {
      put(want[s], black);
      put(want[(s + 1) % 24], spk_c);
      put(want[(s + 2) % 24], spk_c);
      put(want[(s + 3) % 24], spk_c);
      put(want[(s + 4) % 24], black);
    }
  }
  render_moment(M_MUTE, PRESETS[P_CLASSIC][M_MUTE], 4321, 0, v, got);
  check("classic mute", got, want, 0);
}

static void run_classic() {
  const uint8_t rings[3][3] = {{57, 194, 255}, {255, 217, 160}, {255, 64, 128}};
  for (const auto &ring : rings) {
    classic_spin(M_WAKE, 0.5f, ring);
    classic_spin(M_LISTEN, 1.0f, ring);
    classic_spin(M_REPLY, -1.0f, ring);
    classic_triangle(M_THINK, ring, false);
    classic_triangle(M_ERR, ring, false);
    classic_triangle(M_RING, ring, false);
    classic_triangle(M_RING, ring, true);
    for (float r : {0.0f, 0.3125f, 0.40625f, 0.625f, 1.0f}) {
      classic_timer(r, ring, false);
      classic_timer(r, ring, true);
      classic_volume(r, ring);
    }
    for (int bits = 0; bits < 8; bits++)
      classic_mute(ring, bits & 1, bits & 2, bits & 4);
  }
  std::printf("classic: checked against the replaced lambdas\n");
}

static void run_clamp() {
  for (int m = 0; m < M_COUNT; m++) {
    if (m == M_MUTE)
      continue;
    Style s = S(m == M_TIMER || m == M_VOL ? FX_ARC : FX_SOLID, CM_RING, 100, 100, F_NODIP | F_REV);
    clamp_style(m, s);
    const uint8_t want = m == M_VOL ? (F_NODIP | F_REV) : F_REV;
    if (s.flags != want && ++failures <= 12)
      std::printf("FAIL clamp %s: flags %u, want %u\n", MOMENT_NAMES[m], unsigned(s.flags), unsigned(want));
  }
  std::printf("clamp: checked\n");
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::printf("usage: %s ringfx-golden.txt\n", argv[0]);
    return 2;
  }
  if (run_golden(argv[1]))
    return 1;
  run_classic();
  run_clamp();
  if (failures) {
    std::printf("%d failures\n", failures);
    return 1;
  }
  std::printf("ok\n");
  return 0;
}
