/**
 * The Home tab's pure logic: the orb's state from the assistant's phase, and the conversions around
 * it - sensor readings and their calibration offsets, the transcript's per-word tabs, timers, and
 * the LED ring's colours.
 */

/**
 * PHASE in src/lib/device.js (the voice_assist_*_phase_id substitutions) -> the orb's states. It is
 * the same phase the LED ring animates from, so the orb and the ring cannot disagree. 10, "Not
 * ready", is the honest reading with Home Assistant gone: the microphones work, but there is nothing
 * on the other end to answer.
 */
const PHASE_ORB = {
  1: "idle",
  2: "listening",
  3: "listening",
  4: "thinking",
  5: "speaking",
  10: "connecting",
  11: "error",
};

export const ORB_LABEL = {
  idle: "Ready",
  listening: "Listening\u2026",
  thinking: "Thinking\u2026",
  speaking: "Speaking\u2026",
  connecting: "Not ready",
  error: "Error",
  disabled: "Offline",
};

export function orbState(phase, connected) {
  if (!connected) return "disabled";
  return PHASE_ORB[phase] || "idle";
}

/**
 * What the orb draws: its state, the label under it, and whether it wears the muted look. A lost
 * event stream wins over mute, because the mute state it would show is as stale as everything else.
 */
export function orbView(phase, connected, muted) {
  const state = orbState(phase, connected);
  if (muted && state !== "disabled") return { state: "idle", label: "Mic muted", muted: true };
  return { state, label: ORB_LABEL[state], muted: false };
}

/** A switch or binary sensor from /events: `value` on some, `state` on others. */
export const isOn = (e) => !!e && (e.value === true || e.state === "ON");

export const c2f = (c) => (c * 9) / 5 + 32;

/**
 * A reading for display. The device publishes °C whatever the unit switch says, so °F is converted
 * here. A sensor with no reading yet publishes a null value, which is an em dash rather than zero.
 */
export function reading(value, digits, unit, fahrenheit) {
  const v = value == null || value === "" ? NaN : Number(value);
  if (!Number.isFinite(v)) return "\u2014";
  return fahrenheit ? `${c2f(v).toFixed(digits)}\u00B0F` : `${v.toFixed(digits)}${unit}`;
}

/** "+0.4°". An offset is a delta, so °F scales it by 1.8 and never adds 32. */
export function offsetText(offset, digits, unit, fahrenheit) {
  const v = fahrenheit ? offset * 1.8 : offset;
  let s = v.toFixed(digits);
  if (Number(s) === 0) s = (0).toFixed(digits);
  else if (v > 0) s = `+${s}`;
  return s + unit;
}

const num = (v, fallback) => {
  const n = v == null || v === "" ? NaN : Number(v);
  return Number.isFinite(n) ? n : fallback;
};

/**
 * The offset stepper's step and range: the number entity's own (web_server sends them as strings
 * with the first state of a session), never finer than the sensor's display step, so every press
 * moves the number on screen. `fallback` ({step, min, max}) covers a payload without them.
 */
export function offsetSpec(e, fallback) {
  return {
    step: Math.max(num(e?.step, 0), fallback.step),
    min: num(e?.min_value, fallback.min),
    max: num(e?.max_value, fallback.max),
  };
}

const decimals = (n) => (String(n).split(".")[1] || "").length;

/** One step up (dir 1) or down (-1), rounded to the step's precision and clamped to the range. */
export function stepOffset(offset, dir, step, min, max) {
  const next = Number((offset + dir * step).toFixed(decimals(step)));
  return Math.min(max, Math.max(min, next));
}

/**
 * The per-wake-word transcript tabs. Each line carries the word that opened its exchange (`w`); the
 * tabs are the distinct words, newest first, and the open tab is the person's pick while that word
 * is still in the ring, else the newest. Untagged lines (older firmware) show under every tab, and
 * with fewer than two words there are no tabs and every line shows.
 */
export function transcriptTabs(lines, pick) {
  const words = [];
  for (let k = lines.length - 1; k >= 0; k--) {
    const w = lines[k].w;
    if (w && !words.includes(w)) words.push(w);
  }
  const tab = pick && words.includes(pick) ? pick : words[0] || null;
  const shown = words.length > 1 ? lines.filter((l) => !l.w || l.w === tab) : lines;
  return { words, tab, shown };
}

const pad2 = (n) => String(n).padStart(2, "0");

/** "09:41", or "1:05:00" past the hour. */
export function clock(seconds) {
  const t = Math.max(0, Math.floor(seconds));
  const h = Math.floor(t / 3600);
  const m = Math.floor((t % 3600) / 60);
  return h ? `${h}:${pad2(m)}:${pad2(t % 60)}` : `${pad2(m)}:${pad2(t % 60)}`;
}

/** A timer's name if it was given one by voice, else its set duration: "10 min timer". */
export function timerLabel(t) {
  if (t.name) return t.name;
  const h = Math.floor(t.total / 3600);
  const m = Math.round((t.total % 3600) / 60);
  const parts = [];
  if (h) parts.push(`${h} h`);
  if (m) parts.push(`${m} min`);
  if (!parts.length) parts.push(`${t.total} s`);
  return `${parts.join(" ")} timer`;
}

/** Seconds left at `now`, counted down from the poll that answered at `polledAt`. A paused timer holds. */
export function timerLeft(t, polledAt, now) {
  return t.active ? Math.max(0, t.left - Math.floor((now - polledAt) / 1000)) : t.left;
}

/** "#a78bfa" -> [167, 139, 250]. */
export function hexToRgb(hex) {
  const h = hex.replace("#", "");
  const n = parseInt(h.length === 3 ? h.replace(/./g, "$&$&") : h, 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

/**
 * Hue in degrees and saturation 0-1 at full value to RGB, the colour the LED ring is written with.
 * Value stays at full on purpose: brightness is a separate control on the light, and folding it into
 * the colour makes both harder to set.
 */
export function hsvToRgb(h, s) {
  const f = (n) => {
    const k = (n + h / 60) % 6;
    return Math.round(255 * (1 - s * Math.max(0, Math.min(k, 4 - k, 1))));
  };
  return [f(5), f(3), f(1)];
}

/** The hue an RGB colour sits at, 0-359; 0 for a grey. */
export function rgbHue(r, g, b) {
  const max = Math.max(r, g, b);
  const d = max - Math.min(r, g, b);
  if (d === 0) return 0;
  let h;
  if (max === r) h = ((g - b) / d) % 6;
  else if (max === g) h = (b - r) / d + 2;
  else h = (r - g) / d + 4;
  return Math.round(h * 60 + 360) % 360;
}

/** The light's 0-255 brightness as a percent. An off ring reads 0, which is how the slider turns it off. */
export const ringPct = (light) => (light?.state === "ON" ? Math.round(((light.brightness ?? 255) / 255) * 100) : 0);

export const pctTo255 = (pct) => Math.round((pct / 100) * 255);
