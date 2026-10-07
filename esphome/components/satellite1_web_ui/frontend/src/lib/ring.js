/**
 * The LED ring's styles as the device holds them (GET /api/sat1/ring, satellite1_ring/ring_fx.cpp
 * web_json), the writes that change them, the previews that play them on the ring, and the frame
 * the LED Ring page's live ring draws for whatever the device is showing right now.
 */
import { apiUrl, post, requestJson } from './device.js';
import {
  M_MUTE, M_RING, M_TIMER, M_VOL, MOMENTS, N, PRESETS, STYLES, fromWire, hash32, normalizeRing, renderMoment, toQuery,
} from './ringfx.js';

const qs = q => new URLSearchParams(q).toString();

/** GET /api/sat1/ring's body as the pages use it: each moment a style object, in MOMENTS order. */
export function parseRing(d) {
  if (!d || typeof d !== 'object' || !d.m) return null;
  return {
    style: String(d.style || 'classic'),
    base: PRESETS.includes(d.base) ? d.base : 'classic',
    moment: String(d.moment || ''),
    preview: String(d.preview || ''),
    in: { ratio: (Number(d.in?.ratio) || 0) / 10000, mic: !!d.in?.mic, spk: !!d.in?.spk },
    m: MOMENTS.map((k, i) => (d.m[k] ? fromWire(d.m[k]) : STYLES.classic[i])),
  };
}

/** Whether this page has started a preview since it last stopped one, so a stop is only sent when
 *  there may be something to stop: stopping re-runs control_leds on the device. */
let previewed = false;

export const ringApi = {
  load: async () => parseRing(await requestJson('/api/sat1/ring')),
  setStyle: name => post(`/api/sat1/ring?${qs({ style: name })}`),
  setMoment: (m, s) => post(`/api/sat1/ring?${qs(toQuery(m, s))}`),
  /** One moment back to the style it was copied from, or (m == null) every moment. */
  reset: m => post(`/api/sat1/ring/reset?${qs(m == null ? { all: 1 } : { m: MOMENTS[m] })}`),
  /** Plays a moment or a fixed signal on the ring for `ms`; with `draft`, the unsaved style. */
  preview: (key, ms, draft) => {
    previewed = true;
    return post(`/api/sat1/ring/preview?${qs({ ...(draft ? toQuery(MOMENTS.indexOf(key), draft) : {}), m: key, ms })}`);
  },
  stop: () => {
    if (!previewed) return Promise.resolve(null);
    previewed = false;
    return post('/api/sat1/ring/preview/stop');
  },
  /** The stop for a page that is going away, which the request queue would never get to send. */
  stopNow: () => {
    if (!previewed) return;
    previewed = false;
    fetch(apiUrl('/api/sat1/ring/preview/stop'), { method: 'POST', keepalive: true }).catch(() => {});
  },
};

/** The style each preset names, as the firmware's PRESETS table holds it. */
export const presetStyle = (name, m) => (STYLES[name] || STYLES.classic)[m];

/**
 * The inputs a preview draws with when nothing on the device supplies them: the same sample arc
 * and mute marks ring_fx.cpp's inputs_() uses while it previews, so the screen and the ring agree.
 * Muted draws with the LED Ring light off, the way most rings sit at idle: just the marks.
 */
export function sampleInputs(m, ring) {
  return {
    ring,
    ringOn: m !== M_MUTE,
    ratio: m === M_TIMER ? 0.62 : m === M_VOL ? 0.45 : 0,
    micMuted: m === M_MUTE,
    spkSilent: false,
  };
}

const dark = () => Array.from({ length: N }, () => [0, 0, 0]);
const all = (c, k = 1) => Array.from({ length: N }, () => c.map(v => Math.round(v * k)));

/**
 * The signals no style changes, drawn for the screen from the YAML effects that light them
 * (common/led_ring.yaml, in the colors control_leds gives them). The card's tap plays the real
 * effect on the device. Those effects step every 10 ms but at the main loop's pace, which is slower
 * and uneven, so the flashes here keep the count and take the pace they show at in the room.
 *
 * A signal that plays once (a few flashes, a sweep) has `once`, how long it lasts on the device,
 * and repeats on screen every `every` ms so the card keeps saying what it looks like. `still` is
 * the moment that best shows it, for the one frame drawn under reduced motion.
 */
const WARM = [255, 227, 181], AQUA = [24, 187, 242], RED = [255, 0, 0], GREEN = [0, 255, 0], BLUE = [0, 0, 255];
const SIGN_IN = [3, 168, 245];

// ESPHome's addressable_twinkle at 50%: each LED swells and fades over about a second, then waits
// a moment for its next, so most of the ring is lit at once at different brightnesses.
function twinkle(t, c) {
  return Array.from({ length: N }, (_, i) => {
    const tt = t + (hash32(i, 11) % 1400);
    const x = (tt % 1400) - (hash32(i, Math.floor(tt / 1400)) % 376);
    return x < 0 || x >= 1024 ? [0, 0, 0] : c.map(v => Math.round(v * Math.sin((Math.PI * x) / 1024)));
  });
}

// Success, Error and Warning: the whole ring flashing n times. The device fades full to dark and
// back; on screen a flash rises from dark and falls back, then holds dark for a beat, because a
// dip that only touches dark for a frame does not read as a blink on a small ring.
const BLINK_MS = 400;
const LIT = 0.6;
const flashes = (t, c, n) => {
  const x = (t % BLINK_MS) / BLINK_MS;
  return t >= n * BLINK_MS || x >= LIT ? dark() : all(c, Math.sin((Math.PI * x) / LIT));
};
const PEAK = (BLINK_MS * LIT) / 2;

// Jack Plugged (from LED 0) and Jack Unplugged (from LED 12): two lights a step every 40 ms, one
// down each side, held where they meet.
function sweep(t, c, from) {
  const k = Math.min(12, Math.floor(t / 40)), out = dark();
  out[(from + k) % N] = c.slice();
  out[(from + N - k) % N] = c.slice();
  return out;
}

// Flashing XMOS: blue, dark up to the update's progress, so the ring empties clockwise.
function drain(t, ms) {
  const index = Math.floor(((t % ms) / ms) * N);
  return Array.from({ length: N }, (_, i) => (i <= index ? [0, 0, 0] : BLUE.slice()));
}

// The sign-in window's pulse effect: 12% up to 80% and back, a 1.4 s transition every 1.5 s.
function breathe(t, c) {
  const x = Math.min(1, (t % 1500) / 1400), e = x * x * (3 - 2 * x);
  return all(c, Math.floor(t / 1500) % 2 ? 0.8 - 0.68 * e : 0.12 + 0.68 * e);
}

// Factory Reset Coming Up: one more red LED every half second while the button is held; the reset
// comes as the ring fills, 22 s into the hold.
const fillUp = t => Array.from({ length: N }, (_, i) => (i <= Math.floor(t / 500) ? RED.slice() : [0, 0, 0]));

const FIXED = {
  improv: { draw: t => twinkle(t, WARM) },
  init: { draw: t => twinkle(t, AQUA) },
  no_ha: { draw: t => twinkle(t, RED) },
  not_ready: { draw: t => twinkle(t, RED) },
  xmos: { draw: t => drain(t, 8000), still: 3000 },
  xmos_done: { draw: t => flashes(t, GREEN, 2), once: 1000, every: 3000, still: PEAK },
  xmos_fail: { draw: t => flashes(t, RED, 2), once: 1000, every: 3000, still: PEAK },
  login: { draw: t => breathe(t, SIGN_IN) },
  login_ok: { draw: t => flashes(t, GREEN, 3), once: 1500, every: 3500, still: PEAK },
  warning: { draw: t => flashes(t, RED, 5), once: 2000, every: 4500, still: PEAK },
  action: { draw: (t, ring) => all(ring) },
  jack_in: { draw: (t, ring) => sweep(t, ring, 0), once: 1000, every: 3000, still: 200 },
  jack_out: { draw: (t, ring) => sweep(t, ring, 12), once: 1000, every: 3000, still: 200 },
  factory: { draw: t => fillUp(t), once: 12000, every: 13500, still: 6000 },
};

/** A fixed signal's frame at `t` ms, null for a key that is not one. */
export function fixedFrame(key, t, ring) {
  const f = FIXED[key];
  if (!f) return null;
  if (!f.once) return f.draw(t, ring);
  const x = t % f.every;
  return x < f.once ? f.draw(x, ring) : dark();
}

/** How a one-shot signal plays: { once, every } in ms, or null for one that runs until it ends. */
export const fixedRun = key => (FIXED[key]?.once ? { once: FIXED[key].once, every: FIXED[key].every } : null);

/** The time a signal's reduced-motion still is drawn at, or undefined for the usual one. */
export const fixedStill = key => FIXED[key]?.still;

/** The ring while the device is idle: the LED Ring light's plain color when it is on, else dark. */
export const idleFrame = (ring, on) => (on ? all(ring) : dark());

/** The light entity's color, scaled the way the device reads it. */
export const ringRgb = light => normalizeRing(light?.color ? [light.color.r, light.color.g, light.color.b] : [255, 255, 255]);

/**
 * The live ring's frame source: the moment the device reports (the LED Ring Moment sensor, over
 * the event stream), started from when the page saw it change, with the spin's head carried across
 * moments the way the device carries it. `get()` returns the current inputs; it is read each frame
 * so a color pick or a fresh poll shows at once.
 */
export function liveRing(get) {
  let key = null;
  let t0 = 0;
  let head = 0;
  let head0 = 0;
  return now => {
    const { moment, data, light, on } = get();
    if (moment !== key) {
      key = moment;
      t0 = now;
      head0 = head;
    }
    const t = Math.max(0, Math.floor(now - t0));
    const ring = ringRgb(light);
    const m = MOMENTS.indexOf(moment);
    if (m >= 0 && data) {
      const inp = { ring, ringOn: on, ratio: data.in.ratio, micMuted: data.in.mic, spkSilent: data.in.spk };
      const r = renderMoment(m, data.m[m], t, head0, inp);
      if (r.head != null) head = r.head;
      return r.frame;
    }
    return fixedFrame(moment, t, ring) ?? idleFrame(ring, on);
  };
}

/** Moments whose drawing depends on the device's own data, which the page polls while they show. */
export const DATA_MOMENTS = [MOMENTS[M_TIMER], MOMENTS[M_VOL], MOMENTS[M_MUTE], MOMENTS[M_RING]];
