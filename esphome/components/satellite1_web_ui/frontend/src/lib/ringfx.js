/**
 * The LED ring renderer, a line-for-line port of esphome/components/satellite1_ring/ring_fx_core.h,
 * so every preview in the app (and the LED Ring page's live ring) shows the frame the device draws
 * from the same inputs. test/ringfx-golden.txt holds frames both must reproduce: change the math in
 * both files, then regenerate it with `npm run golden` and run the host test.
 *
 * Phases come from elapsed milliseconds with integer arithmetic wherever a frame boundary matters,
 * exact in a double below 2^53. The float math (cos, sin, pow) runs in float on the device and in
 * double here, which moves a channel by a unit at most; the parity tests allow two.
 */

export const N = 24;

export const FX = ['off', 'solid', 'breathe', 'pulse', 'spin', 'comet', 'orbit', 'ripple', 'twinkle', 'wave', 'flow', 'dot', 'arc'];
export const [FX_OFF, FX_SOLID, FX_BREATHE, FX_PULSE, FX_SPIN, FX_COMET, FX_ORBIT, FX_RIPPLE, FX_TWINKLE, FX_WAVE, FX_FLOW, FX_DOT, FX_ARC] = FX.map((_, i) => i);
export const CM = ['ring', 'blend', 'rainbow', 'own', 'red'];
export const [CM_RING, CM_BLEND, CM_RAINBOW, CM_OWN, CM_RED] = CM.map((_, i) => i);
export const F_REV = 1, F_FIXED = 2, F_NODIP = 4, F_IF_ON = 8;

/** The nine moments a style covers, in the firmware's order. */
export const MOMENTS = ['wake', 'listen', 'think', 'reply', 'timer', 'ring', 'vol', 'mute', 'err'];
export const [M_WAKE, M_LISTEN, M_THINK, M_REPLY, M_TIMER, M_RING, M_VOL, M_MUTE, M_ERR] = MOMENTS.map((_, i) => i);
export const PRESETS = ['classic', 'calm', 'aurora', 'party', 'minimal'];
export const MIC_LEDS = [0, 6, 12, 18];

const TAU = 6.28318530718;

export function S(fx, cm, sp, br = 100, fl = 0, p = 0) {
  return { fx, cm, n: 1, fl, sp, br, p, c: [[0, 0, 0], [0, 0, 0], [0, 0, 0]] };
}

/** Muted in every style: the red marks over what the ring shows at idle. */
const MUTED = () => S(FX_SOLID, CM_RING, 100, 100, F_IF_ON);

export const STYLES = {
  classic: [S(FX_SPIN, CM_RING, 50), S(FX_SPIN, CM_RING, 100), S(FX_ORBIT, CM_RING, 100, 100, F_FIXED),
    S(FX_SPIN, CM_RING, 100, 100, F_REV), S(FX_ARC, CM_RING, 100), S(FX_PULSE, CM_RING, 100),
    S(FX_ARC, CM_RING, 100, 100, F_NODIP), MUTED(), S(FX_PULSE, CM_RED, 100)],
  calm: [S(FX_BREATHE, CM_RING, 80), S(FX_BREATHE, CM_RING, 140), S(FX_WAVE, CM_RING, 40),
    S(FX_COMET, CM_RING, 50, 100, 0, 14), S(FX_ARC, CM_RING, 100), S(FX_BREATHE, CM_RING, 180),
    S(FX_ARC, CM_RING, 100, 100, F_NODIP), MUTED(), S(FX_BREATHE, CM_RED, 120)],
  aurora: [S(FX_COMET, CM_BLEND, 80, 100, 0, 12), S(FX_WAVE, CM_BLEND, 120), S(FX_TWINKLE, CM_BLEND, 100),
    S(FX_COMET, CM_BLEND, 110, 100, F_REV, 12), S(FX_ARC, CM_BLEND, 100), S(FX_RIPPLE, CM_BLEND, 100),
    S(FX_ARC, CM_BLEND, 100, 100, F_NODIP), MUTED(), S(FX_PULSE, CM_RED, 100)],
  party: [S(FX_RIPPLE, CM_RAINBOW, 100), S(FX_FLOW, CM_RAINBOW, 140, 100, F_REV), S(FX_TWINKLE, CM_RAINBOW, 130),
    S(FX_SPIN, CM_RAINBOW, 120, 100, F_REV, 3), S(FX_ARC, CM_RAINBOW, 100), S(FX_FLOW, CM_RAINBOW, 300, 100, F_REV),
    S(FX_ARC, CM_RAINBOW, 100, 100, F_NODIP), MUTED(), S(FX_PULSE, CM_RED, 100)],
  minimal: [S(FX_DOT, CM_RING, 100, 50), S(FX_DOT, CM_RING, 220, 50), S(FX_ORBIT, CM_RING, 50, 50),
    S(FX_COMET, CM_RING, 70, 50, 0, 1), S(FX_ARC, CM_RING, 100, 40), S(FX_DOT, CM_RING, 350, 60),
    S(FX_ARC, CM_RING, 100, 40, F_NODIP), MUTED(), S(FX_DOT, CM_RED, 300)]
};

/** The firmware's clamp_style: what each moment allows. Returns a new style. */
export function clampStyle(m, s) {
  if (m === M_MUTE) return MUTED();
  s = { ...s, c: (s.c || []).map(x => x.slice()) };
  while (s.c.length < 3) s.c.push([0, 0, 0]);
  if (!(s.fx >= 0 && s.fx < FX.length)) s.fx = FX_SOLID;
  if (m === M_TIMER || m === M_VOL) s.fx = FX_ARC;
  else if (s.fx === FX_ARC) s.fx = FX_SOLID;
  if (m === M_ERR) s.cm = CM_RED;
  else if (!(s.cm >= 0 && s.cm < CM.length) || s.cm === CM_RED) s.cm = CM_RING;
  s.n = Math.min(3, Math.max(1, s.n | 0));
  s.sp = Math.min(400, Math.max(10, s.sp | 0));
  s.br = Math.min(100, Math.max(5, s.br | 0));
  if (s.fx === FX_SPIN && s.p > 4) s.p = 4;
  if (s.fx === FX_COMET && s.p > 20) s.p = 20;
  if (s.fx === FX_RIPPLE && s.p > 3) s.p = 3;
  if (s.fx === FX_DOT && s.p > N - 1) s.p = N - 1;
  if (m === M_VOL) s.fl |= F_NODIP;
  else s.fl &= ~F_NODIP;
  return s;
}

export const scale8 = (v, k) => (v * (1 + k)) >> 8;
const wrapIndex = i => ((i % N) + N) % N;
const toU8 = v => (v <= 0 ? 0 : v >= 255 ? 255 : Math.floor(v + 0.5));

export function hash32(i, k) {
  let x = (Math.imul(i, 0x9E3779B1) + Math.imul(k, 0x85EBCA77)) >>> 0;
  x = (x ^ (x >>> 15)) >>> 0;
  x = Math.imul(x, 0x2C1B3C6D) >>> 0;
  x = (x ^ (x >>> 12)) >>> 0;
  x = Math.imul(x, 0x297A2D39) >>> 0;
  return (x ^ (x >>> 15)) >>> 0;
}

const phase = (t, sp, period) => ((t * sp) % (period * 100)) / (period * 100);

function tri8(t, sp) {
  const m = Math.floor((t * sp) / 1000) % 20;
  return 25 * (10 - (m <= 10 ? m : 20 - m));
}

export function hsvToRgb(h, s, v) {
  h = h - Math.floor(h / 360) * 360;
  const c = v * s, x = c * (1 - Math.abs(((h / 60) % 2) - 1)), m = v - c;
  const i = Math.floor(h / 60) % 6;
  const [r, g, b] = [[c, x, 0], [x, c, 0], [0, c, x], [0, x, c], [x, 0, c], [c, 0, x]][i];
  return [toU8((r + m) * 255), toU8((g + m) * 255), toU8((b + m) * 255)];
}

export function rgbToHsv([R, G, B]) {
  const r = R / 255, g = G / 255, b = B / 255;
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d > 0) {
    if (mx === r) h = ((g - b) / d) % 6;
    else if (mx === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  return [h, mx > 0 ? d / mx : 0, mx];
}

function rainbowAt(u) {
  const h = (u - Math.floor(u)) * 6, i = Math.floor(h), f = h - i, q = 1 - f;
  const [r, g, b] = [[1, f, 0], [q, 1, 0], [0, 1, f], [0, q, 1], [f, 0, 1], [1, 0, q]][i % 6];
  return [toU8(r * 255), toU8(g * 255), toU8(b * 255)];
}

/** The stops a style paints with, from its color mode and the ring color. */
export function resolve(s, ring) {
  switch (s.cm) {
    case CM_BLEND: {
      const [h, sat, v] = rgbToHsv(ring);
      const s2 = sat + (1 - sat) * 0.5;
      return { n: 3, c: [hsvToRgb(h - 60, s2, v), ring.slice(), hsvToRgb(h + 60, s2, v)] };
    }
    case CM_RAINBOW: return { rainbow: true, n: 1, c: [] };
    case CM_OWN: {
      const n = Math.min(3, Math.max(1, s.n | 0));
      return { n, c: s.c.slice(0, n) };
    }
    case CM_RED: return { n: 1, c: [[255, 0, 0]] };
    default: return { n: 1, c: [ring.slice()] };
  }
}

function palAt(p, u, wrap) {
  if (p.rainbow) return rainbowAt(u);
  if (p.n === 1) return p.c[0].slice();
  let i, j, f;
  if (wrap) {
    u -= Math.floor(u);
    const x = u * p.n;
    i = Math.floor(x) % p.n;
    j = (i + 1) % p.n;
    f = x - Math.floor(x);
  } else {
    u = u < 0 ? 0 : u > 1 ? 1 : u;
    const x = u * (p.n - 1);
    i = Math.min(Math.floor(x), p.n - 2);
    j = i + 1;
    f = x - i;
  }
  const a = p.c[i], b = p.c[j];
  return [toU8(a[0] + (b[0] - a[0]) * f), toU8(a[1] + (b[1] - a[1]) * f), toU8(a[2] + (b[2] - a[2]) * f)];
}

const putK = (c, k) => [toU8(c[0] * k), toU8(c[1] * k), toU8(c[2] * k)];
const putK8 = (c, k8) => [scale8(c[0], k8), scale8(c[1], k8), scale8(c[2], k8)];
function addK(o, c, k) {
  for (let i = 0; i < 3; i++) o[i] = Math.min(255, o[i] + toU8(c[i] * k));
}

/** Draws the animation into a cleared frame; returns the spin or comet head it drew, if any. */
function drawFx(s, t, head0, inp, out) {
  const p = resolve(s, inp.ring);
  const dir = s.fl & F_REV ? -1 : 1;
  let head = null;
  switch (s.fx) {
    case FX_SOLID:
      for (let i = 0; i < N; i++) out[i] = palAt(p, i / N, true);
      break;
    case FX_BREATHE: {
      const k = 0.12 + 0.88 * (0.5 - 0.5 * Math.cos(TAU * phase(t, s.sp, 2856)));
      const d = (t % 33333) / 33333;
      for (let i = 0; i < N; i++) out[i] = putK(palAt(p, i / N + d, true), k);
      break;
    }
    case FX_PULSE: {
      const k8 = tri8(t, s.sp);
      for (let i = 0; i < N; i++) out[i] = putK8(palAt(p, i / N, true), k8);
      break;
    }
    case FX_SPIN: {
      const heads = s.p || 2;
      const steps = Math.floor((Math.floor(t / 50) * s.sp) / 100);
      const h = wrapIndex(head0 + dir * (steps % N));
      head = h;
      for (let q = 0; q < heads; q++) {
        const b = wrapIndex(h + Math.floor((q * N) / heads));
        for (let j = 0; j < 3; j++) {
          const x = wrapIndex(b - j), c = palAt(p, x / N, true);
          out[x] = j === 0 ? c : putK8(c, j === 1 ? 192 : 128);
        }
      }
      break;
    }
    case FX_COMET: {
      const len = s.p || 10;
      const steps = Math.floor((t * s.sp * 14) / 100000);
      const h = wrapIndex(head0 + dir * (steps % N));
      head = h;
      for (let j = 0; j < len; j++) {
        const f = 1 - j / len;
        addK(out[wrapIndex(h - j * dir)], palAt(p, len > 1 ? j / (len - 1) : 0, false), f * f);
      }
      break;
    }
    case FX_ORBIT: {
      let a = 2, k8;
      if (s.fl & F_FIXED) {
        k8 = tri8(t, s.sp);
      } else {
        a += dir * (Math.floor((t * s.sp * 3) / 100000) % N);
        k8 = toU8(255 * (0.35 + 0.65 * (0.5 + 0.5 * Math.cos(TAU * phase(t, s.sp, 1111)))));
      }
      out[wrapIndex(a)] = putK8(palAt(p, 0, false), k8);
      out[wrapIndex(a + 12)] = putK8(palAt(p, 1, false), k8);
      break;
    }
    case FX_RIPPLE: {
      const ph = phase(t, s.sp, 1429), d = ph * 12;
      const o = s.p * (N / 4);
      for (let j = 0; j < 4; j++) {
        const dd = d - j * 0.8;
        if (dd < 0) continue;
        const f = ((1 << (4 - j)) / 16) * (1 - ph * 0.6);
        const c = palAt(p, dd / 12, false), r = Math.floor(dd + 0.5);
        const a = wrapIndex(o + r), b = wrapIndex(o - r);
        addK(out[a], c, f);
        if (b !== a) addK(out[b], c, f);
      }
      break;
    }
    case FX_TWINKLE: {
      const per = Math.floor(140000 / (s.sp || 1));
      for (let i = 0; i < N; i++) {
        const tt = t + (hash32(i, 1) % per);
        const cyc = Math.floor(tt / per), ph = (tt % per) / per;
        if (hash32(i, cyc + 7) % 100 < 40) continue;
        const sn = Math.sin(3.14159265359 * ph);
        out[i] = putK(palAt(p, (hash32(i, cyc + 3) % 1000) / 1000, true), sn * sn);
      }
      break;
    }
    case FX_WAVE: {
      const ph = phase(t, s.sp, 2513), d = phase(t, s.sp, 8333);
      for (let i = 0; i < N; i++) {
        const w = 0.5 + 0.5 * Math.sin((TAU * 2 * i) / N - dir * TAU * ph);
        out[i] = putK(palAt(p, i / N + dir * d, true), 0.25 + 0.75 * Math.pow(w, 1.5));
      }
      break;
    }
    case FX_FLOW: {
      const ph = phase(t, s.sp, 5556);
      for (let i = 0; i < N; i++) out[i] = palAt(p, i / N - dir * ph, true);
      break;
    }
    case FX_DOT: {
      const k = 0.25 + 0.75 * (0.5 - 0.5 * Math.cos(TAU * phase(t, s.sp, 2417)));
      out[wrapIndex(s.p)] = putK(palAt(p, 0, false), k);
      break;
    }
    case FX_ARC: {
      const x = 24 * inp.ratio, last = Math.ceil(x) - 1;
      const dip = s.fl & F_NODIP ? -1 : wrapIndex(-(Math.floor((t * s.sp) / 10000) % N));
      for (let i = 0; i < N; i++) {
        if (i > x) continue;
        const dk = i === dip && i !== last ? 0.9 : 1;
        const v = 255 * dk * (x - i), cap = 255 * dk;
        out[i] = putK8(palAt(p, i / N, false), Math.floor(v < cap ? v : cap));
      }
      break;
    }
  }
  return head;
}

function micCap(out) {
  for (const m of MIC_LEDS) {
    const c = out[m], mx = Math.max(c[0], c[1], c[2]);
    if (mx > 128) {
      const sc = (128 * 255) / mx + 0.5;
      out[m] = putK8(c, sc > 255 ? 255 : Math.floor(sc));
    }
  }
}

const blank = () => Array.from({ length: N }, () => [0, 0, 0]);
const RED = () => [255, 0, 0];
const BLACK = () => [0, 0, 0];

/**
 * One moment's frame. inp: { ring:[r,g,b] (brightest channel 255), ringOn, ratio 0-1, micMuted,
 * spkSilent }. Returns { frame: 24 [r,g,b], head } where head is the spin or comet head drawn.
 */
export function renderMoment(m, s, t, head0, inp) {
  const out = blank();
  let head = null;
  if (!(s.fl & F_IF_ON && !inp.ringOn)) head = drawFx(s, t, head0, inp, out);
  if (s.br < 100) for (const c of out) for (let k = 0; k < 3; k++) c[k] = Math.floor((c[k] * s.br + 50) / 100);
  if (m <= M_REPLY) micCap(out);
  if (m === M_TIMER && inp.micMuted) {
    for (const i of [2, 4, 8, 10]) out[i] = BLACK();
    out[3] = RED();
    out[9] = RED();
  } else if (m === M_RING && inp.micMuted) {
    out[3] = RED();
    out[9] = RED();
  } else if (m === M_MUTE) {
    if (inp.micMuted) {
      for (const c of MIC_LEDS) {
        out[wrapIndex(c - 1)] = BLACK();
        out[c] = RED();
        out[wrapIndex(c + 1)] = BLACK();
      }
    }
    if (inp.spkSilent) {
      for (const st of [1, 7, 13, 19]) {
        out[st] = BLACK();
        for (let j = 1; j <= 3; j++) out[wrapIndex(st + j)] = [200, 0, 0];
        out[wrapIndex(st + 4)] = BLACK();
      }
    }
  } else if (m === M_VOL && inp.ratio <= 0) {
    out[0] = RED();
  }
  return { frame: out, head };
}

/** "38bdf8" or "#38bdf8" to [r,g,b]; null if it is not six hex digits. */
export function hexRgb(h) {
  const m = /^#?([0-9a-f]{6})$/i.exec(String(h || '').trim());
  if (!m) return null;
  const v = parseInt(m[1], 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
}

export const rgbHex = c => '#' + c.map(v => Math.round(Math.min(255, Math.max(0, v))).toString(16).padStart(2, '0')).join('');

/** A light color scaled so its brightest channel is 255, the way the device reads it. */
export function normalizeRing(c) {
  const mx = Math.max(c[0], c[1], c[2]);
  if (!mx) return [255, 255, 255];
  return c.map(v => Math.floor((v / mx) * 255));
}

/** A moment from GET /api/sat1/ring, as a style object. */
export function fromWire(w) {
  const s = S(Math.max(0, FX.indexOf(w.fx)), Math.max(0, CM.indexOf(w.cm)), w.sp ?? 100, w.br ?? 100, w.fl ?? 0, w.p ?? 0);
  const stops = (w.c || []).map(hexRgb).filter(Boolean);
  if (stops.length) {
    s.n = stops.length;
    stops.forEach((c, i) => { if (i < 3) s.c[i] = c; });
  }
  return s;
}

/** The query string POST /api/sat1/ring (and a draft preview) takes for one moment's style. */
export function toQuery(m, s) {
  const q = { m: MOMENTS[m], fx: FX[s.fx], cm: CM[s.cm], sp: s.sp, br: s.br, dir: s.fl & F_REV ? -1 : 1, p: s.p };
  if (s.cm === CM_OWN) q.c = s.c.slice(0, s.n).map(c => rgbHex(c).slice(1)).join(',');
  return q;
}

/** How a moment differs from the style it was copied from: 'own' colors, 'edit'ed motion, or ''. */
export function momentTag(s, base) {
  if (s.cm === CM_OWN) return 'own';
  if (!base) return '';
  return s.fx !== base.fx || s.cm !== base.cm || s.sp !== base.sp || s.br !== base.br || (s.fl & F_REV) !== (base.fl & F_REV) || s.p !== base.p ? 'edit' : '';
}
