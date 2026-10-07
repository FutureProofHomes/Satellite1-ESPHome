#pragma once

// The 24-LED ring renderer: a pure function from (style, elapsed ms, inputs) to 24 RGB values, with
// no ESPHome includes so the host parity test can compile it with g++. The web UI's lib/ringfx.js
// is a line-for-line port; test/ringfx-golden.txt holds frames both must reproduce, so a change
// here needs the same change there and a regenerated fixture (npm run golden in the frontend).
//
// Phases are derived from elapsed time with integer arithmetic wherever a frame boundary matters
// (spin steps, the pulse triangle, the timer dip), so a frame does not depend on how often it is
// drawn and the browser lands on the same step as the device. Classic reproduces the effect
// lambdas the firmware shipped before this component (Rotating Blob, Thinking, Error, Timer Ring,
// Timer Tick, Volume Display, Muted or Silent) frame for frame; the host test checks that against
// re-implementations of them.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>

namespace esphome::satellite1_ring {

static constexpr int N = 24;

enum Fx : uint8_t {
  FX_OFF,
  FX_SOLID,
  FX_BREATHE,
  FX_PULSE,
  FX_SPIN,
  FX_COMET,
  FX_ORBIT,
  FX_RIPPLE,
  FX_TWINKLE,
  FX_WAVE,
  FX_FLOW,
  FX_DOT,
  FX_ARC,
  FX_COUNT,
};

enum ColorMode : uint8_t { CM_RING, CM_BLEND, CM_RAINBOW, CM_OWN, CM_RED, CM_COUNT };

// F_REV: counter-clockwise (spin, comet, orbit, wave, flow). F_FIXED: orbit holds LEDs 2 and 14
// (Classic's thinking). F_NODIP: the arc without the timer's travelling dip (volume). F_IF_ON: the
// base draws only while the LED Ring light is on (muted); the marks on top draw either way.
enum Flag : uint8_t { F_REV = 1, F_FIXED = 2, F_NODIP = 4, F_IF_ON = 8 };

// The nine moments a style covers.
enum Moment : uint8_t { M_WAKE, M_LISTEN, M_THINK, M_REPLY, M_TIMER, M_RING, M_VOL, M_MUTE, M_ERR, M_COUNT };

enum Preset : uint8_t { P_CLASSIC, P_CALM, P_AURORA, P_PARTY, P_MINIMAL, P_COUNT };

struct Style {
  uint8_t fx;
  uint8_t cm;
  uint8_t n;  // own color stops, 1-3
  uint8_t flags;
  uint16_t sp;  // percent of the animation's base speed
  uint8_t br;   // percent
  uint8_t p;    // spin heads, comet length, ripple's start (0-3: N E S W) or the dot's LED; 0 is the default
  uint8_t c[3][3];
};

struct Inputs {
  uint8_t ring[3]{0, 0, 0};  // the LED Ring light's color, brightest channel at 255
  bool ring_on{false};
  float ratio{0.0f};  // timer time left over total, or volume, 0-1
  bool mic_muted{false};
  bool spk_silent{false};
};

typedef uint8_t Frame[N][3];

static constexpr float TAU = 6.28318530718f;
static constexpr int MIC_LEDS[4] = {0, 6, 12, 18};

inline constexpr Style S(uint8_t fx, uint8_t cm, uint16_t sp, uint8_t br = 100, uint8_t flags = 0, uint8_t p = 0) {
  return Style{fx, cm, 1, flags, sp, br, p, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
}

// Motion only: every preset but Party takes its color from the ring (Aurora blends around it), and
// errors stay red in all of them. Muted is the same in all of them, MUTED below.
static constexpr Style MUTED = S(FX_SOLID, CM_RING, 100, 100, F_IF_ON);
static constexpr Style PRESETS[P_COUNT][M_COUNT] = {
    // Classic
    {S(FX_SPIN, CM_RING, 50), S(FX_SPIN, CM_RING, 100), S(FX_ORBIT, CM_RING, 100, 100, F_FIXED),
     S(FX_SPIN, CM_RING, 100, 100, F_REV), S(FX_ARC, CM_RING, 100), S(FX_PULSE, CM_RING, 100),
     S(FX_ARC, CM_RING, 100, 100, F_NODIP), MUTED, S(FX_PULSE, CM_RED, 100)},
    // Calm
    {S(FX_BREATHE, CM_RING, 80), S(FX_BREATHE, CM_RING, 140), S(FX_WAVE, CM_RING, 40),
     S(FX_COMET, CM_RING, 50, 100, 0, 14), S(FX_ARC, CM_RING, 100), S(FX_BREATHE, CM_RING, 180),
     S(FX_ARC, CM_RING, 100, 100, F_NODIP), MUTED, S(FX_BREATHE, CM_RED, 120)},
    // Aurora
    {S(FX_COMET, CM_BLEND, 80, 100, 0, 12), S(FX_WAVE, CM_BLEND, 120), S(FX_TWINKLE, CM_BLEND, 100),
     S(FX_COMET, CM_BLEND, 110, 100, F_REV, 12), S(FX_ARC, CM_BLEND, 100), S(FX_RIPPLE, CM_BLEND, 100),
     S(FX_ARC, CM_BLEND, 100, 100, F_NODIP), MUTED, S(FX_PULSE, CM_RED, 100)},
    // Party
    {S(FX_RIPPLE, CM_RAINBOW, 100), S(FX_FLOW, CM_RAINBOW, 140, 100, F_REV), S(FX_TWINKLE, CM_RAINBOW, 130),
     S(FX_SPIN, CM_RAINBOW, 120, 100, F_REV, 3), S(FX_ARC, CM_RAINBOW, 100), S(FX_FLOW, CM_RAINBOW, 300, 100, F_REV),
     S(FX_ARC, CM_RAINBOW, 100, 100, F_NODIP), MUTED, S(FX_PULSE, CM_RED, 100)},
    // Minimal
    {S(FX_DOT, CM_RING, 100, 50), S(FX_DOT, CM_RING, 220, 50), S(FX_ORBIT, CM_RING, 50, 50),
     S(FX_COMET, CM_RING, 70, 50, 0, 1), S(FX_ARC, CM_RING, 100, 40), S(FX_DOT, CM_RING, 350, 60),
     S(FX_ARC, CM_RING, 100, 40, F_NODIP), MUTED, S(FX_DOT, CM_RED, 300)},
};

// Brings a style inside what its moment allows: timer and volume are always an arc (the arc length
// is the data), only they may be one, errors are always red, and muted is always the red marks over
// what the ring shows at idle - nothing, or the LED Ring light's color.
inline void clamp_style(uint8_t m, Style &s) {
  if (m == M_MUTE) {
    s = MUTED;
    return;
  }
  if (s.fx >= FX_COUNT)
    s.fx = FX_SOLID;
  if (m == M_TIMER || m == M_VOL) {
    s.fx = FX_ARC;
  } else if (s.fx == FX_ARC) {
    s.fx = FX_SOLID;
  }
  if (m == M_ERR) {
    s.cm = CM_RED;
  } else if (s.cm >= CM_COUNT || s.cm == CM_RED) {
    s.cm = CM_RING;
  }
  if (s.n < 1)
    s.n = 1;
  if (s.n > 3)
    s.n = 3;
  if (s.sp < 10)
    s.sp = 10;
  if (s.sp > 400)
    s.sp = 400;
  if (s.br < 5)
    s.br = 5;
  if (s.br > 100)
    s.br = 100;
  if (s.fx == FX_SPIN && s.p > 4)
    s.p = 4;
  if (s.fx == FX_COMET && s.p > 20)
    s.p = 20;
  if (s.fx == FX_RIPPLE && s.p > 3)
    s.p = 3;
  if (s.fx == FX_DOT && s.p > N - 1)
    s.p = N - 1;
  if (m == M_VOL) {
    s.flags |= F_NODIP;
  } else {
    s.flags &= ~F_NODIP;
  }
}

inline uint8_t scale8(uint8_t v, uint8_t k) { return uint8_t((uint16_t(v) * (1 + uint16_t(k))) >> 8); }
inline int wrap_index(int i) { return ((i % N) + N) % N; }
inline uint8_t to_u8(float v) { return v <= 0.0f ? 0 : v >= 255.0f ? 255 : uint8_t(v + 0.5f); }

// An integer hash, so twinkle's choices are identical in the browser (Math.imul) and on the device.
inline uint32_t hash32(uint32_t i, uint32_t k) {
  uint32_t x = i * 0x9E3779B1u + k * 0x85EBCA77u;
  x ^= x >> 15;
  x *= 0x2C1B3C6Du;
  x ^= x >> 12;
  x *= 0x297A2D39u;
  x ^= x >> 15;
  return x;
}

// Where in a loop of period_ms (at 100% speed) the animation is, 0-1.
inline float phase(uint32_t t, uint16_t sp, uint32_t period_ms) {
  uint64_t span = uint64_t(period_ms) * 100;
  return float((uint64_t(t) * sp) % span) / float(span);
}

// The triangle Classic's thinking, error and timer-done pulses step through: 25 * (10 - step),
// one step per 10 ms at 100%.
inline uint8_t tri8(uint32_t t, uint16_t sp) {
  uint32_t m = uint32_t((uint64_t(t) * sp / 1000) % 20);
  uint32_t step = m <= 10 ? m : 20 - m;
  return uint8_t(25 * (10 - step));
}

struct Palette {
  uint8_t c[3][3];
  uint8_t n;
  bool rainbow;
};

inline void hsv_to_rgb(float h, float s, float v, uint8_t out[3]) {
  h = h - floorf(h / 360.0f) * 360.0f;
  float c = v * s, x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f)), m = v - c;
  int i = int(h / 60.0f) % 6;
  float r = 0, g = 0, b = 0;
  switch (i) {
    case 0:
      r = c, g = x;
      break;
    case 1:
      r = x, g = c;
      break;
    case 2:
      g = c, b = x;
      break;
    case 3:
      g = x, b = c;
      break;
    case 4:
      r = x, b = c;
      break;
    default:
      r = c, b = x;
      break;
  }
  out[0] = to_u8((r + m) * 255.0f);
  out[1] = to_u8((g + m) * 255.0f);
  out[2] = to_u8((b + m) * 255.0f);
}

inline void rgb_to_hsv(const uint8_t in[3], float &h, float &s, float &v) {
  float r = in[0] / 255.0f, g = in[1] / 255.0f, b = in[2] / 255.0f;
  float mx = fmaxf(r, fmaxf(g, b)), mn = fminf(r, fminf(g, b)), d = mx - mn;
  h = 0;
  if (d > 0) {
    if (mx == r) {
      h = fmodf((g - b) / d, 6.0f);
    } else if (mx == g) {
      h = (b - r) / d + 2.0f;
    } else {
      h = (r - g) / d + 4.0f;
    }
    h *= 60.0f;
    if (h < 0)
      h += 360.0f;
  }
  s = mx > 0 ? d / mx : 0;
  v = mx;
}

// Full-saturation hue wheel, u in turns.
inline void rainbow_at(float u, uint8_t out[3]) {
  float h = (u - floorf(u)) * 6.0f;
  int i = int(h);
  float f = h - float(i), q = 1.0f - f;
  float r, g, b;
  switch (i % 6) {
    case 0:
      r = 1, g = f, b = 0;
      break;
    case 1:
      r = q, g = 1, b = 0;
      break;
    case 2:
      r = 0, g = 1, b = f;
      break;
    case 3:
      r = 0, g = q, b = 1;
      break;
    case 4:
      r = f, g = 0, b = 1;
      break;
    default:
      r = 1, g = 0, b = q;
      break;
  }
  out[0] = to_u8(r * 255.0f);
  out[1] = to_u8(g * 255.0f);
  out[2] = to_u8(b * 255.0f);
}

inline Palette resolve(const Style &s, const Inputs &in) {
  Palette p{};
  p.n = 1;
  switch (s.cm) {
    case CM_BLEND: {
      // A sixth of the wheel either side of the ring color, pushed halfway to full saturation: the
      // ring colors are pale, and closer neighbours of a pale color read as that one color.
      float h, sat, v;
      rgb_to_hsv(in.ring, h, sat, v);
      float s2 = sat + (1.0f - sat) * 0.5f;
      hsv_to_rgb(h - 60.0f, s2, v, p.c[0]);
      memcpy(p.c[1], in.ring, 3);
      hsv_to_rgb(h + 60.0f, s2, v, p.c[2]);
      p.n = 3;
      break;
    }
    case CM_RAINBOW:
      p.rainbow = true;
      break;
    case CM_OWN:
      p.n = s.n < 1 ? 1 : s.n > 3 ? 3 : s.n;
      memcpy(p.c, s.c, sizeof(p.c));
      break;
    case CM_RED:
      p.c[0][0] = 255;
      break;
    default:
      memcpy(p.c[0], in.ring, 3);
      break;
  }
  return p;
}

// A color along the palette: around the ring (wrap) or from the first stop to the last.
inline void pal_at(const Palette &p, float u, bool wrap, uint8_t out[3]) {
  if (p.rainbow) {
    rainbow_at(u, out);
    return;
  }
  if (p.n == 1) {
    memcpy(out, p.c[0], 3);
    return;
  }
  int i, j;
  float f;
  if (wrap) {
    u -= floorf(u);
    float x = u * p.n;
    i = int(x) % p.n;
    j = (i + 1) % p.n;
    f = x - floorf(x);
  } else {
    u = u < 0 ? 0 : u > 1 ? 1 : u;
    float x = u * (p.n - 1);
    i = int(x);
    if (i > p.n - 2)
      i = p.n - 2;
    j = i + 1;
    f = x - float(i);
  }
  for (int k = 0; k < 3; k++)
    out[k] = to_u8(p.c[i][k] + (float(p.c[j][k]) - float(p.c[i][k])) * f);
}

inline void put_k(uint8_t out[3], const uint8_t c[3], float k) {
  for (int i = 0; i < 3; i++)
    out[i] = to_u8(c[i] * k);
}

inline void put_k8(uint8_t out[3], const uint8_t c[3], uint8_t k8) {
  for (int i = 0; i < 3; i++)
    out[i] = scale8(c[i], k8);
}

inline void add_k(uint8_t out[3], const uint8_t c[3], float k) {
  for (int i = 0; i < 3; i++) {
    int v = out[i] + to_u8(c[i] * k);
    out[i] = v > 255 ? 255 : uint8_t(v);
  }
}

// Draws the animation alone, into a cleared frame. head0 is where a spin or comet starts; *head
// receives the head it drew, so the next moment can carry on from it.
inline void draw_fx(const Style &s, uint32_t t, int head0, const Inputs &in, Frame out, int *head) {
  Palette p = resolve(s, in);
  uint8_t c[3], c2[3];
  int dir = (s.flags & F_REV) ? -1 : 1;
  switch (s.fx) {
    case FX_SOLID:
      for (int i = 0; i < N; i++)
        pal_at(p, float(i) / N, true, out[i]);
      break;
    case FX_BREATHE: {
      float k = 0.12f + 0.88f * (0.5f - 0.5f * cosf(TAU * phase(t, s.sp, 2856)));
      float d = float(t % 33333) / 33333.0f;
      for (int i = 0; i < N; i++) {
        pal_at(p, float(i) / N + d, true, c);
        put_k(out[i], c, k);
      }
      break;
    }
    case FX_PULSE: {
      uint8_t k8 = tri8(t, s.sp);
      for (int i = 0; i < N; i++) {
        pal_at(p, float(i) / N, true, c);
        put_k8(out[i], c, k8);
      }
      break;
    }
    case FX_SPIN: {
      int heads = s.p ? s.p : 2;
      uint32_t steps = uint32_t((uint64_t(t / 50) * s.sp) / 100);
      int h = wrap_index(head0 + dir * int(steps % N));
      if (head)
        *head = h;
      for (int q = 0; q < heads; q++) {
        int b = wrap_index(h + q * N / heads);
        for (int j = 0; j < 3; j++) {
          int x = wrap_index(b - j);
          pal_at(p, float(x) / N, true, c);
          if (j == 0) {
            memcpy(out[x], c, 3);
          } else {
            put_k8(out[x], c, j == 1 ? 192 : 128);
          }
        }
      }
      break;
    }
    case FX_COMET: {
      int len = s.p ? s.p : 10;
      uint32_t steps = uint32_t(uint64_t(t) * s.sp * 14 / 100000);
      int h = wrap_index(head0 + dir * int(steps % N));
      if (head)
        *head = h;
      for (int j = 0; j < len; j++) {
        float f = 1.0f - float(j) / float(len);
        pal_at(p, len > 1 ? float(j) / float(len - 1) : 0.0f, false, c);
        add_k(out[wrap_index(h - j * dir)], c, f * f);
      }
      break;
    }
    case FX_ORBIT: {
      int a = 2;
      uint8_t k8;
      if (s.flags & F_FIXED) {
        k8 = tri8(t, s.sp);
      } else {
        a += dir * int((uint64_t(t) * s.sp * 3 / 100000) % N);
        k8 = to_u8(255.0f * (0.35f + 0.65f * (0.5f + 0.5f * cosf(TAU * phase(t, s.sp, 1111)))));
      }
      pal_at(p, 0.0f, false, c);
      pal_at(p, 1.0f, false, c2);
      put_k8(out[wrap_index(a)], c, k8);
      put_k8(out[wrap_index(a + 12)], c2, k8);
      break;
    }
    case FX_RIPPLE: {
      float ph = phase(t, s.sp, 1429), d = ph * 12.0f;
      int o = s.p * (N / 4);
      for (int j = 0; j < 4; j++) {
        float dd = d - float(j) * 0.8f;
        if (dd < 0)
          continue;
        float f = float(1 << (4 - j)) / 16.0f * (1.0f - ph * 0.6f);
        pal_at(p, dd / 12.0f, false, c);
        int r = int(floorf(dd + 0.5f));
        int a = wrap_index(o + r), b = wrap_index(o - r);
        add_k(out[a], c, f);
        if (b != a)
          add_k(out[b], c, f);
      }
      break;
    }
    case FX_TWINKLE: {
      uint32_t per = 140000u / (s.sp ? s.sp : 1);
      for (int i = 0; i < N; i++) {
        uint32_t tt = t + hash32(i, 1) % per;
        uint32_t cyc = tt / per;
        float ph = float(tt % per) / float(per);
        if (hash32(i, cyc + 7) % 100 < 40)
          continue;
        float sn = sinf(3.14159265359f * ph);
        pal_at(p, float(hash32(i, cyc + 3) % 1000) / 1000.0f, true, c);
        put_k(out[i], c, sn * sn);
      }
      break;
    }
    case FX_WAVE: {
      float ph = phase(t, s.sp, 2513), d = phase(t, s.sp, 8333);
      for (int i = 0; i < N; i++) {
        float w = 0.5f + 0.5f * sinf(TAU * 2.0f * float(i) / N - float(dir) * TAU * ph);
        pal_at(p, float(i) / N + float(dir) * d, true, c);
        put_k(out[i], c, 0.25f + 0.75f * powf(w, 1.5f));
      }
      break;
    }
    case FX_FLOW: {
      float ph = phase(t, s.sp, 5556);
      for (int i = 0; i < N; i++)
        pal_at(p, float(i) / N - float(dir) * ph, true, out[i]);
      break;
    }
    case FX_DOT: {
      float k = 0.25f + 0.75f * (0.5f - 0.5f * cosf(TAU * phase(t, s.sp, 2417)));
      pal_at(p, 0.0f, false, c);
      put_k(out[wrap_index(s.p)], c, k);
      break;
    }
    case FX_ARC: {
      float x = 24.0f * in.ratio;
      int last = int(ceilf(x)) - 1;
      int dip = (s.flags & F_NODIP) ? -1 : wrap_index(-int((uint64_t(t) * s.sp / 10000) % N));
      for (int i = 0; i < N; i++) {
        if (float(i) > x)
          continue;
        float dk = (i == dip && i != last) ? 0.9f : 1.0f;
        float v = 255.0f * dk * (x - float(i)), cap = 255.0f * dk;
        pal_at(p, float(i) / N, false, c);
        put_k8(out[i], c, uint8_t(v < cap ? v : cap));
      }
      break;
    }
    default:
      break;
  }
}

// LEDs 0, 6, 12 and 18 sit next to the microphones and are held to half brightness through a
// conversation, from the wake word to the reply.
inline void mic_cap(Frame out) {
  for (int m : MIC_LEDS) {
    uint8_t *c = out[m];
    uint8_t mx = c[0] > c[1] ? c[0] : c[1];
    mx = mx > c[2] ? mx : c[2];
    if (mx > 128) {
      float sc = 128.0f * 255.0f / float(mx) + 0.5f;
      uint8_t s8 = sc > 255.0f ? 255 : uint8_t(sc);
      put_k8(c, c, s8);
    }
  }
}

inline void set_rgb(uint8_t out[3], uint8_t r, uint8_t g, uint8_t b) {
  out[0] = r;
  out[1] = g;
  out[2] = b;
}

// One moment's frame: the style's animation, its brightness, the mic cap, then the fixed marks
// that always mean the same thing (red for a muted mic, a silent speaker, volume zero).
inline void render_moment(uint8_t m, const Style &s, uint32_t t, int head0, const Inputs &in, Frame out,
                          int *head = nullptr) {
  memset(out, 0, sizeof(Frame));
  if (!((s.flags & F_IF_ON) && !in.ring_on))
    draw_fx(s, t, head0, in, out, head);
  if (s.br < 100) {
    for (int i = 0; i < N; i++)
      for (int k = 0; k < 3; k++)
        out[i][k] = uint8_t((out[i][k] * s.br + 50) / 100);
  }
  if (m <= M_REPLY)
    mic_cap(out);
  if (m == M_TIMER && in.mic_muted) {
    for (int i : {2, 4, 8, 10})
      set_rgb(out[i], 0, 0, 0);
    set_rgb(out[3], 255, 0, 0);
    set_rgb(out[9], 255, 0, 0);
  } else if (m == M_RING && in.mic_muted) {
    set_rgb(out[3], 255, 0, 0);
    set_rgb(out[9], 255, 0, 0);
  } else if (m == M_MUTE) {
    if (in.mic_muted) {
      for (int c : MIC_LEDS) {
        set_rgb(out[wrap_index(c - 1)], 0, 0, 0);
        set_rgb(out[c], 255, 0, 0);
        set_rgb(out[wrap_index(c + 1)], 0, 0, 0);
      }
    }
    if (in.spk_silent) {
      for (int st : {1, 7, 13, 19}) {
        set_rgb(out[st], 0, 0, 0);
        for (int j = 1; j <= 3; j++)
          set_rgb(out[wrap_index(st + j)], 200, 0, 0);
        set_rgb(out[wrap_index(st + 4)], 0, 0, 0);
      }
    }
  } else if (m == M_VOL && in.ratio <= 0.0f) {
    set_rgb(out[0], 255, 0, 0);
  }
}

}  // namespace esphome::satellite1_ring
