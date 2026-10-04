/**
 * The Home tab's pure logic: the orb's state from the assistant's phase, and the conversions around
 * it - sensor readings and their calibration offsets, the transcript's per-word tabs and its layout,
 * timers, and the LED ring's colours.
 */
import { TEXT } from "../copy.js";
import { PIPELINE_PREFERRED } from "./device.js";

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

/**
 * How long the page keeps a pipeline error on the orb, from when it first sees it. The device's
 * error phase lasts about a second and the voice poll runs at five when idle, so going by the phase
 * alone the message flashed or never showed (owner, October 2026).
 */
export const ERROR_HOLD_MS = 6000;

export function orbState(phase, connected) {
  if (!connected) return "disabled";
  return PHASE_ORB[phase] || "idle";
}

/**
 * What the orb draws: its state, the label under it, and whether it wears the muted look. A lost
 * event stream wins over mute, because the mute state it would show is as stale as everything else.
 * `error` is the device's newest pipeline error, whose message replaces the bare "Error" - in the
 * error phase always, since the device records it before entering the phase, and on an idle device
 * while the page is `holding` it (useErrorHold).
 */
export function orbView(phase, connected, muted, error = null, holding = false) {
  let state = orbState(phase, connected);
  if (muted && state !== "disabled") return { state: "idle", label: "Mic muted", muted: true };
  if (holding && error && state === "idle") state = "error";
  const message = state === "error" ? String(error?.message || "").trim() : "";
  return { state, label: message || ORB_LABEL[state], muted: false };
}

/**
 * The hold for GET /api/sat1/voice's `error` ({n, age, message}), taken the first time the page sees
 * that error: ERROR_HOLD_MS from then, so one the idle poll finds seconds late is still up long
 * enough to read. One already older than the hold when found - a page opened after it - gets none.
 */
export function errorSeen(error, now) {
  if (!error || !Number.isFinite(error.age)) return null;
  return { n: error.n, until: error.age <= ERROR_HOLD_MS ? now + ERROR_HOLD_MS : 0 };
}

/** Until when the page holds the error `seen` saw, or 0 - none, past the hold, or dismissed (`gone` is its count). */
export function errorHoldUntil(seen, gone, now) {
  return seen && seen.n !== gone && seen.until > now ? seen.until : 0;
}

/** "the Living Room", but "Bob's Office" and "The Den" as they are. */
const theRoom = (room) => (/^the\s|'s\b|\u2019s\b/i.test(room) ? room : `the ${room}`);

/**
 * The tips a resting orb rotates through (owner's list, October 2026), each only while it is true
 * here: `f` is what the page knows -
 *   word   the wake word the orb and the front window answer to, '' for none
 *   other  the other window's word, when both slots have one
 *   room   this device's Home Assistant area, '' for none
 *   ai     that word's pipeline goes to an agent other than Home Assistant's own
 *   voice  Home Assistant is connected, so anything said gets an answer
 * and the rest are booleans for what the page has (tap, type, untuned, pick, sensors, radar,
 * noRadar, music, peers, mute, ring, stop, route). `done` is the ids this browser has acted on
 * (src/lib/tips.js); the spoken examples never retire, so there is always something to show.
 * Page tips and examples alternate, the orb's own tip first while it applies, and the examples
 * repeat to keep up with the page tips, so one is never more than a tip away.
 */
export function orbTips(f, done = []) {
  const examples = [];
  const room = f.room ? theRoom(f.room) : "";
  if (f.voice && f.word) {
    if (f.ai) {
      examples.push(room ? TEXT.tip_ai_room : TEXT.tip_ai_home);
      if (f.music) examples.push(TEXT.tip_ai_song);
      examples.push(TEXT.tip_ai_combo);
      if (room) examples.push(TEXT.tip_ai_cozy);
    } else {
      if (room) examples.push(TEXT.tip_ha_lights, TEXT.tip_ha_dim);
      examples.push(TEXT.tip_ha_list, TEXT.tip_ha_time);
    }
  }
  const tips = [];
  const add = (id, when, text) => {
    if (when && !done.includes(id)) tips.push(text);
  };
  add("orb", f.tap && f.voice, TEXT.tip_orb);
  add("timer", f.voice && f.word, TEXT.tip_timer);
  add("stop", f.voice && f.stop, TEXT.tip_stop);
  add("swipe", f.other, TEXT.tip_swipe);
  add("type", f.type, TEXT.tip_type);
  // Not retired: it goes once the word is tuned.
  if (f.untuned && f.word) tips.push(TEXT.tip_tune);
  add("pick", f.pick && f.word, TEXT.tip_pick);
  add("sensors", f.sensors, f.radar ? TEXT.tip_sensors_radar : TEXT.tip_sensors);
  add("music", f.music, TEXT.tip_music);
  add("group", f.music, TEXT.tip_group);
  add("switch", f.peers, TEXT.tip_switch);
  add("route", f.voice && f.route, TEXT.tip_route);
  add("mute", f.mute, TEXT.tip_mute);
  add("color", f.ring, TEXT.tip_color);
  add("radar", f.noRadar, TEXT.tip_radar);
  const out = [];
  for (let i = 0; i < Math.max(tips.length, examples.length); i++) {
    if (i < tips.length) out.push(tips[i]);
    if (examples.length) out.push(examples[i % examples.length]);
  }
  const fill = { word: f.word || "", other: f.other || "", room };
  return out.map((s) => s.replace(/\{(word|other|room)\}/g, (_, k) => fill[k]));
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
 * The transcript split into the Home page's two windows, one per wake word slot (owner's design,
 * October 2026). `slots` is the slots' words in order, '' for an empty one. Each line carries the
 * word that opened its exchange (`w`) and shows in that word's window; a stop word ends the
 * exchange it interrupted, so its line goes with that exchange. Untagged lines (the action button)
 * show in every window with a word, an empty slot's window shows none, and a line from a word no
 * slot holds any more shows nowhere - the ring is eight lines, so it soon goes. Until the slots are
 * known (`slots` empty) there is one window with every line.
 *
 * `newest` is the window of the newest line that has one, which is where the assistant's typing
 * bubble belongs. `cue` is the newest thing a person said there, keyed by the line, so the page can
 * bring that window forward once per new exchange rather than on every answer line.
 */
export function transcriptWindows(lines, slots = []) {
  if (!slots.length) return { newest: null, cue: null, windows: [{ word: "", lines }] };
  const filed = [];
  let last = "";
  for (const l of lines) {
    const w = l.w === "stop" && last ? last : l.w || "";
    if (w) last = w;
    filed.push(w);
  }
  const at = (k) => (filed[k] ? slots.indexOf(filed[k]) : -1);
  let newest = null;
  let cue = null;
  for (let k = filed.length - 1; k >= 0 && cue === null; k--) {
    const i = at(k);
    if (i < 0) continue;
    if (newest === null) newest = i;
    if (lines[k].heard) cue = { index: i, key: lineId(lines[k]) };
  }
  const windows = slots.map((word) => ({ word, lines: word ? lines.filter((_, k) => !filed[k] || filed[k] === word) : [] }));
  return { newest, cue, windows };
}

/**
 * `s` cut to at most `max` UTF-8 bytes without splitting a character. A text entity's limit is in
 * bytes, and the device refuses an over-long value outright rather than truncating it.
 */
export function fitBytes(s, max) {
  const enc = new TextEncoder();
  if (enc.encode(s).length <= max) return s;
  let out = "";
  let n = 0;
  for (const ch of s) {
    const b = enc.encode(ch).length;
    if (n + b > max) break;
    out += ch;
    n += b;
  }
  return out;
}

/** The agent typed messages go to when none is picked: Home Assistant's built-in one. */
export const DEFAULT_AGENT = "conversation.home_assistant";

/**
 * The name of the agent `picked` names, from the payload's `cv` rows ([entity_id, name]). Empty
 * means the default; an agent Home Assistant no longer lists is shown by its id rather than
 * passed off as the default.
 */
export function agentName(cv, picked) {
  const id = picked || DEFAULT_AGENT;
  const row = (cv || []).find((r) => r[0] === id);
  if (row) return row[1] || id;
  return id === DEFAULT_AGENT ? TEXT.ask_default_agent : id;
}

/** Letters and digits only, lowercased: how a pipeline's name is held against an agent's. */
const squash = (s) => String(s || "").toLowerCase().replace(/[^\p{L}\p{N}]/gu, "");

/**
 * A short fingerprint of all Home Assistant shows the device of its voice setup: the pipelines'
 * names and the conversation agents' ids, in any order. A pipeline can be pointed at another agent,
 * or another pipeline made the preferred one, without the device seeing it, so each answer is kept
 * with the fingerprint it was given under and asked again once that has changed (owner's decision,
 * October 2026: re-ask when something visible changes). Agents' display names are left out, since
 * renaming one sends nothing anywhere else. Four base-36 characters (FNV-1a, folded): the text
 * entity has little room, and a collision only means one question not asked.
 */
export function setupStamp(pipelines, cv) {
  const s = `${[...(pipelines || [])].sort().join("\n")}\u0000${(cv || []).map((r) => r[0]).sort().join("\n")}`;
  let h = 0x811c9dc5;
  for (const ch of s) {
    h ^= ch.codePointAt(0);
    h = Math.imul(h, 0x01000193);
  }
  return ((h >>> 0) % 1679616).toString(36).padStart(4, "0");
}

/** The person's answer for `pipeline` in `known`, as the agent's entity id and the setup stamp it
 *  was given under ('' for an answer from before stamps). */
export function savedAgent(known, pipeline) {
  const raw = known && typeof known[pipeline] === "string" ? known[pipeline] : "";
  if (!raw) return null;
  const [short, stamp = ""] = raw.split("~");
  return { agent: short.includes(".") ? short : `conversation.${short}`, stamp };
}

/**
 * The agent a message typed under `pipeline` goes to, or null when the page has to ask (owner's
 * design, October 2026: the message field's menu is the wake word's pipeline, and typing reaches
 * that pipeline's agent). Home Assistant does not tell the device which agent a pipeline uses, so
 * this is the person's answer for the pipeline in `known` (the pipeline_agents map) while that agent
 * is still listed and nothing visible has changed since (`stamp`, from setupStamp), else the one
 * agent in `cv` whose name or entity id says the pipeline's name. An answer that has gone stale is
 * asked again rather than replaced by a name match: the person once said the name was not it.
 * Preferred is never matched by name: which pipeline it stands for is not in the payload either.
 */
export function pipelineAgent(pipeline, cv, known, stamp) {
  const list = cv || [];
  const saved = savedAgent(known, pipeline);
  if (saved && (saved.agent === DEFAULT_AGENT || list.some((r) => r[0] === saved.agent))) return saved.stamp === stamp ? saved.agent : null;
  if (!pipeline || pipeline === PIPELINE_PREFERRED) return null;
  const name = squash(pipeline);
  const hits = list.filter(([id, n]) => squash(n) === name || squash(id.slice(id.indexOf(".") + 1)) === name);
  if (hits.length === 1) return hits[0][0];
  if (!hits.length && name === squash(TEXT.ask_default_agent)) return DEFAULT_AGENT;
  return null;
}

/** The pipeline_agents text entity's value as a map; anything unreadable is an empty one. */
export function readAgentMap(raw) {
  try {
    const o = JSON.parse(raw || "{}");
    return o && typeof o === "object" && !Array.isArray(o) ? o : {};
  } catch {
    return {};
  }
}

/**
 * The pipeline_agents value once `pipeline` is answered with `agent` under `stamp` (setupStamp), or
 * null if that answer alone cannot fit. The text entity holds 255 bytes, so agents are kept without
 * their "conversation." prefix, as "agent~stamp", and the oldest answers go first - those for
 * `keep`, the pipelines the slots use now, last.
 */
export function rememberAgent(map, pipeline, agent, stamp, keep = [], max = 255) {
  const next = { ...map };
  delete next[pipeline];
  next[pipeline] = `${agent.startsWith("conversation.") ? agent.slice("conversation.".length) : agent}~${stamp}`;
  const fits = () => new TextEncoder().encode(JSON.stringify(next)).length <= max;
  for (const spare of [(k) => !keep.includes(k), () => true]) {
    for (const k of Object.keys(next)) {
      if (fits()) break;
      if (k !== pipeline && spare(k)) delete next[k];
    }
  }
  return fits() ? JSON.stringify(next) : null;
}

/** A transcript line's identity across polls. The ring shifts as it fills, so its index is not one. */
export const lineId = (l) => `${l.at}|${l.heard ? "h" : "a"}|${l.text}`;

/** Seconds between two lines that earn the later one a time header of its own. */
export const STAMP_GAP = 15 * 60;

const midnight = (ms) => new Date(ms).setHours(0, 0, 0, 0);

/**
 * A time header's two parts, worded as iMessage words them: "Today" and "3:12 AM", "Yesterday",
 * the weekday within the week, and before that the date with "at 9:40 PM". The first part is the
 * one set in bold. `locale` is for tests; the page uses the browser's.
 */
export function stampLabel(ms, now = Date.now(), locale = undefined) {
  const d = new Date(ms);
  const days = Math.round((midnight(now) - midnight(ms)) / 864e5);
  const time = d.toLocaleTimeString(locale, { hour: "numeric", minute: "2-digit" });
  if (days <= 0) return { day: TEXT.transcript_today, time };
  if (days === 1) return { day: TEXT.transcript_yesterday, time };
  if (days < 7) return { day: d.toLocaleDateString(locale, { weekday: "long" }), time };
  const year = d.getFullYear() === new Date(now).getFullYear() ? undefined : "numeric";
  return {
    day: d.toLocaleDateString(locale, { month: "short", day: "numeric", year }),
    time: TEXT.transcript_at.replace("%s", time),
  };
}

/**
 * The transcript as iMessage lays a conversation out. A time header goes over the first line and
 * over any line STAMP_GAP or more after the one before it. Consecutive lines from one side form a
 * run (`run` on every line after the first), and only a run's last bubble carries the tail. The
 * typing bubble, when shown, joins an answer run it follows.
 *
 * `boot` is when the device started, in ms, which turns each line's uptime stamp into a time of
 * day; without it there are no headers. Rows are {stamp, ms, key}, {line, id, key, run, tail} or
 * {typing, key, run}.
 */
export function transcriptRows(lines, boot, typing, now = Date.now(), locale = undefined) {
  const rows = [];
  const keys = new Map();
  let prev = null;
  let prevRow = null;
  for (const line of lines) {
    const id = lineId(line);
    const n = keys.get(id) || 0;
    keys.set(id, n + 1);
    const key = n ? `${id}|${n}` : id;
    const stamped = boot != null && (!prev || line.at - prev.at >= STAMP_GAP);
    if (stamped) {
      const ms = boot + line.at * 1000;
      rows.push({ stamp: stampLabel(ms, now, locale), ms, key: `t|${key}` });
    }
    const run = !stamped && !!prev && prev.heard === line.heard;
    if (run) prevRow.tail = false;
    prevRow = { line, id, key, run, tail: true };
    rows.push(prevRow);
    prev = line;
  }
  if (typing) {
    const run = !!prev && !prev.heard;
    if (run) prevRow.tail = false;
    rows.push({ typing: true, key: "typing", run });
  }
  return rows;
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

/** "#a78bfa", "#abc" or the custom picker's "hsl(290, 70%, 70%)" to [r, g, b]; null for anything else. */
export function colorRgb(c) {
  const t = typeof c === "string" ? c.trim() : "";
  if (/^#([\da-f]{3}|[\da-f]{6})$/i.test(t)) return hexToRgb(t);
  const m = /^hsla?\(\s*([\d.]+)(?:deg)?[\s,]+([\d.]+)%[\s,]+([\d.]+)%/i.exec(t);
  return m ? hslToRgb(+m[1], +m[2] / 100, +m[3] / 100) : null;
}

function hslToRgb(h, s, l) {
  const a = s * Math.min(l, 1 - l);
  const f = (n) => {
    const k = (n + h / 30) % 12;
    return Math.round(255 * (l - a * Math.max(-1, Math.min(k - 3, 9 - k, 1))));
  };
  return [f(0), f(8), f(4)];
}

function rgbToHsl(r, g, b) {
  const mx = Math.max(r, g, b) / 255;
  const mn = Math.min(r, g, b) / 255;
  const l = (mx + mn) / 2;
  const d = mx - mn;
  return [rgbHue(r, g, b), d === 0 ? 0 : d / (1 - Math.abs(2 * l - 1)), l];
}

const luminance = (rgb) => {
  const [r, g, b] = rgb.map((v) => {
    const c = v / 255;
    return c <= 0.03928 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
  });
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
};

/** WCAG contrast ratio between two [r, g, b] colours. */
export function contrast(a, b) {
  const [hi, lo] = [luminance(a), luminance(b)].sort((x, y) => y - x);
  return (hi + 0.05) / (lo + 0.05);
}

const toHex = (rgb) => "#" + rgb.map((v) => v.toString(16).padStart(2, "0")).join("");

/** The colour at its own hue and saturation, walked darker (dir -1) or lighter (+1) until `ok`. */
function shade(rgb, dir, ok) {
  if (ok(rgb)) return rgb;
  const [h, s, l0] = rgbToHsl(...rgb);
  for (let l = l0; l >= 0 && l <= 1; l += dir * 0.01) {
    const c = hslToRgb(h, s, l);
    if (ok(c)) return c;
  }
  return dir < 0 ? [0, 0, 0] : [255, 255, 255];
}

/**
 * The hardest backdrop orb-coloured text sits on in each theme (styles/tokens.css): --surface2 in
 * dark, the lightest of the greys, and --bg in light, the darkest.
 */
export const INK_ON = { dark: [43, 43, 51], light: [232, 234, 240] };
const WHITE = [255, 255, 255];
const INK = [17, 17, 19];
const AA = 4.6;

/**
 * The orb colour as the theme's custom properties. The presets are pastels that read as light on
 * dark and wash out on white, so nothing uses the raw colour where legibility counts:
 * - fill: a deepened shade white text passes AA on - user bubbles, primary buttons, the nav pill.
 * - ink: the colour as text on the page or a card - darkened in light mode, lightened in dark.
 * - ctl: an on-state control (switch track, checkbox, slider fill) - raw in dark, where it glows
 *   against the page; the fill in light, where the raw pastel would vanish into white.
 * - ctlOn: the tick or dot drawn on ctl.
 */
export function orbTokens(color, theme) {
  const rgb = colorRgb(color) || [167, 139, 250];
  const fill = shade(rgb, -1, (c) => contrast(c, WHITE) >= AA);
  const light = theme === "light";
  const ink = shade(rgb, light ? -1 : 1, (c) => contrast(c, INK_ON[light ? "light" : "dark"]) >= AA);
  const ctl = light ? fill : rgb;
  const alpha = (a) => `rgba(${rgb.join(",")},${a})`;
  return {
    fill: toHex(fill),
    ink: toHex(ink),
    ctl: toHex(ctl),
    ctlOn: contrast(ctl, WHITE) >= contrast(ctl, INK) ? "#fff" : toHex(INK),
    a20: alpha(0.2),
    a35: alpha(0.35),
    a55: alpha(0.55),
  };
}

/** The light's 0-255 brightness as a percent. An off ring reads 0, which is how the slider turns it off. */
export const ringPct = (light) => (light?.state === "ON" ? Math.round(((light.brightness ?? 255) / 255) * 100) : 0);

export const pctTo255 = (pct) => Math.round((pct / 100) * 255);
