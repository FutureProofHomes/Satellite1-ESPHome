/**
 * The Wake Word tab's pure logic: the Living Graph's marks, the tuner's placement math, the swap
 * poll's verdict, the Home Assistant pairing write, the picker's list, and the per-slot Finished
 * Speaking Detection rule.
 *
 * Scores on the wire are quantized probabilities, 0-255; the graphs draw percent.
 */
import { NO_WAKE_WORD } from "./device.js";

export const DAY_MS = 24 * 60 * 60 * 1000;

/** The probe floor the firmware drops a model to during a tune session (WL_TUNE_FLOOR). */
export const TUNE_FLOOR = 107;

/** Where the knob may live, in percent. The floor is the firmware's WL_TUNED_MIN (100/255); the
 *  ceiling keeps the quantized value inside the loader's <= 254 acceptance. */
export const CUT_MIN = 40;
export const CUT_MAX = 95;

export const pctN = (v) => Math.round((v / 255) * 100);
export const clampCut = (c) => Math.max(CUT_MIN, Math.min(CUT_MAX, c));
export const isUrl = (s) => /^https?:\/\//.test(s || "");

/** Deterministic vertical jitter, purely for legibility - the x position is the datum. */
const jit = (i, base, span) => base + ((i * 37) % span);

/** A dot's opacity at its age (0 new, 1 a day old): the rolling window empties itself. */
export const fade = (age) => 0.32 + 0.68 * (1 - Math.min(Math.max(age ?? 0, 0), 1));

/**
 * A track's 24h `dh` ring ([msAgo, score, kind, id], kind 0 a firing and 1 a close call) as graph
 * marks. The ring is persisted on the device, so the record survives reboots. The jitter keys on
 * the entry's stable id, never its position, so a dot landing never moves the others (owner's
 * report: index-keyed jitter reshuffled the graph on every firing). Only the newest firing
 * ripples, and only while under 4.5s old.
 */
export function rowMarks(track, yBase = 14, ySpan = 28) {
  const out = [];
  let newest = null;
  let newestMs = Infinity;
  for (const [ms, sc, kind, id] of track?.dh || []) {
    if (ms >= DAY_MS || !sc) continue;
    const m = { id: `d${id}`, kind: kind ? "near" : "fire", c: pctN(sc), y: yBase + ((id * 37) % ySpan), age: ms / DAY_MS };
    if (!kind && ms < newestMs) {
      newest = m;
      newestMs = ms;
    }
    out.push(m);
  }
  if (newest && newestMs < 4500) newest.ripple = true;
  return out;
}

/** The room's hourly high-water buckets (`day`, newest first) as the tuner's amber smatter, age
 *  driving the fade. No aggregate tick and no label: the room's reach is wherever the smatter
 *  ends. */
export function roomMarks(day, yBase, ySpan) {
  const out = [];
  (day || []).forEach((v, k) => {
    if (v) out.push({ id: `r${k}`, kind: "room", c: pctN(v), y: jit(k, yBase, ySpan), age: k / 23 });
  });
  return out;
}

/** Each room-register reading a session has shown, oldest first, the newest rippling in. */
export function sessionRoomMarks(seen, yBase, ySpan) {
  return seen.map((v, k) => ({
    id: `s${k}`,
    kind: "room",
    c: pctN(v),
    y: jit(8 + k, yBase, ySpan),
    age: 0,
    ripple: k === seen.length - 1,
  }));
}

/** The session's attempts in fixed lanes rather than jitter: two attempts at the same score
 *  overlapped into one unreadable blot (owner's screenshot). */
export function attemptMarks(attempts, yBase, laneH) {
  return attempts.map((a, k) => ({
    id: `a${k}`,
    kind: "you",
    c: pctN(a.score),
    y: yBase + k * laneH,
  }));
}

const ROUNDS = ["near", "near", "far", "other"];

/**
 * A tune session's event ring ([peak, avg, vad, msAgo, mm]) as scored attempts plus the count the
 * voice gate refused. The score is the peak single-frame probability, a hardware finding (Dev12,
 * September 22 2026): the engine resets its probability window the instant a detection fires, so
 * during a floored session the max windowed mean is truncated at the floor. The peak is the one
 * number the reset cannot touch; placement()'s margin covers its overestimate of the steady-state
 * mean.
 */
export function attemptsOf(ev) {
  const attempts = ev.filter((e) => !e[2]).map((e, k) => ({ score: e[0], round: ROUNDS[k] || "other" }));
  return { attempts, vadTries: ev.length - attempts.length };
}

/**
 * Into placement: the knob seeds inside the gap between the room and the quietest attempt, so
 * Apply without dragging is a correct answer. The seed is clamped 8 (quantized) under the quietest
 * attempt, the margin for the peak score's overestimate of the steady-state mean. No usable gap is
 * diagnosed by side rather than shrugged at.
 */
export function placement(attempts, day, roomReg) {
  const floorV = Math.min(...attempts.map((a) => a.score));
  const hiV = Math.max(...attempts.map((a) => a.score));
  const noise = Math.max(0, ...(day || []), roomReg);
  if (floorV - 8 < 115) return { nogap: noise > 140 ? "room" : "voice", floorV, hiV, noise };
  const base = Math.max(noise, TUNE_FLOOR);
  const seedQ = Math.min(floorV - 8, Math.max(115, Math.round(base + 0.6 * Math.max(0, floorV - base))));
  return { floorV, hiV, noise, cutC: clampCut(pctN(seedQ)) };
}

/** The placement readout's verdict at knob `cutC`: crowding the quietest try, at or under the
 *  room's reach, or safe. `c` and `floorC` are the percentages the copy names. In quick edit
 *  `floorV` is the persisted voice stat, read as data only: it is never drawn as a band. */
export function readout(cutC, floorV, day, roomReg) {
  const floorC = floorV ? pctN(floorV) : 0;
  let roomC = roomReg ? pctN(roomReg) : 0;
  for (const v of day || []) roomC = Math.max(roomC, pctN(v));
  const c = Math.round(cutC);
  if (floorC && c >= floorC - 2) return { tone: "warn", c, floorC };
  if (roomC && c <= roomC) return { tone: "err", c, floorC };
  return { tone: "dim", c, floorC };
}

/** The cutoff write: the knob quantized into 100-250, with the session's stats when known. The
 *  engine only ever reads the threshold; `n`, `f` and `h` are advisory, persisted for the graph and
 *  read back as the track's `tn` [noise, floor, hi] that quick edit reopens placement over. */
export function cutoffPath(i, cutC, noise, floor, hi) {
  const v = Math.max(100, Math.min(250, Math.round(cutC * 2.55)));
  return `/api/sat1/wakewords/cutoff?i=${i}&v=${v}${noise ? `&n=${noise}` : ""}${floor ? `&f=${floor}` : ""}${hi ? `&h=${hi}` : ""}`;
}

/**
 * One poll's verdict on slot `slot` swapping to `spec`: {done}, {err} with the loader's error,
 * {dl, tot} while downloading, or null while still pending. A failure only counts once it is
 * about this spec or names an error, since `st` may still describe the previous attempt.
 */
export function swapStep(slot, spec) {
  if (!slot) return null;
  const arrived = spec === "none" ? slot.m === "" : slot.m === spec;
  if (arrived && slot.st === 0) return { done: true };
  if (slot.st === 2 && (arrived || slot.err)) return { err: slot.err };
  if (slot.st === 1) return { dl: slot.dl || 0, tot: slot.tot || 0 };
  return null;
}

/**
 * The one select write that brings Home Assistant's wake word selects (`asst.s` rows, [entity,
 * option, ...]) into line with slot `i` going from `prevWord` to `nextWord` ("" for none), as
 * [entity, option] - or null once they agree. The new word takes the old word's select, else an
 * empty one, else the select at the slot's own index.
 */
export function pairingWrite(sel, i, prevWord, nextWord) {
  const named = (w) => sel.findIndex((x) => w && (x[1] || "").toLowerCase() === w.toLowerCase());
  if (!nextWord) {
    const off = named(prevWord);
    return off < 0 ? null : [sel[off][0], NO_WAKE_WORD];
  }
  if (named(nextWord) >= 0) return null;
  let at = named(prevWord);
  if (at < 0) at = sel.findIndex((x) => x[1] === NO_WAKE_WORD);
  if (at < 0) at = Math.min(i, sel.length - 1);
  return [sel[at][0], nextWord];
}

/** A pasted source - a GitHub repository or a manifest .json link - as {url, label}, or null. */
export function parseSource(draft) {
  const url = String(draft || "")
    .trim()
    .replace(/\/+$/, "");
  const gh = /^https?:\/\/(?:www\.)?github\.com\/[^/]+\/[^/#?]+/.test(url);
  if (!gh && !/\.json($|\?)/.test(url)) return null;
  return { url, label: gh ? url.split("/").slice(3, 5).join("/") : url.split("/").pop() };
}

/**
 * Every pickable word, the built-ins (`builtin` rows [id, phrase]) first and then each source's in
 * order, as {key, word, spec, langs, source, ver, unverified}. `spec` is what the slot is pointed at:
 * a built-in id or a manifest URL. `ver` is only set where the source has the phrase twice.
 */
export function pickerEntries(builtin, builtinLabel, sources, cat) {
  const out = (builtin || []).map(([id, word]) => ({ key: `b|${id}`, word, spec: id, langs: [], source: builtinLabel }));
  sources.forEach((s, k) => {
    for (const e of cat[s.url]?.entries || []) {
      out.push({
        key: `${k}|${e.url}`,
        word: e.word,
        spec: e.url,
        langs: e.langs || [],
        source: s.label,
        ver: e.dup ? e.ver || "" : "",
        unverified: !!e.unverified,
      });
    }
  });
  return out;
}

/** The languages the list's words declare, for the filter. */
export const langsOf = (entries) => [...new Set(entries.flatMap((e) => e.langs))].sort();

/** A search inside ~800 words still matches a crowd: past this many, the rest are counted. */
export const SEARCH_CAP = 60;

/** The picker's rows: phrase and language matches, the slot's current word first. Only a search is
 *  capped - an unfiltered list is the browsable catalogue. `lang` "" is every language. */
export function filterEntries(entries, q, lang, current) {
  const needle = q.trim().toLowerCase();
  const hits = entries.filter((e) => (!needle || e.word.toLowerCase().includes(needle)) && (!lang || e.langs.includes(lang)));
  const at = hits.findIndex((e) => e.spec === current);
  if (at > 0) hits.unshift(hits.splice(at, 1)[0]);
  if (!needle || hits.length <= SEARCH_CAP) return { shown: hits, hidden: 0 };
  return { shown: hits.slice(0, SEARCH_CAP), hidden: hits.length - SEARCH_CAP };
}

let langNames = null;
/** "en" -> "English"; the code itself where the browser has no name for it. */
export function langLabel(code) {
  try {
    if (!langNames) langNames = new Intl.DisplayNames(["en"], { type: "language" });
    return langNames.of(code) || code;
  } catch {
    return code;
  }
}

/* ------------------------------------------------------------------ */
/* Finished Speaking Detection, per slot                               */
/* ------------------------------------------------------------------ */

/** The two device selects, Primary and Secondary slot. Before each request the device copies the
 *  firing slot's value into Home Assistant's own select. */
export const FSD_KEYS = ["fsd_1", "fsd_2"];
export const FSD_UNSET = "unset";
export const FSD_OPTIONS = [
  ["aggressive", "Aggressive"],
  ["default", "Default"],
  ["relaxed", "Relaxed"],
];

/** What each slot's picker shows: its own value, or Home Assistant's current one while `unset`.
 *  Null - the picker hidden - until both device selects have reported and HA's value is known. */
export function fsdShown(own, ha) {
  if (!ha || own.length !== 2 || own.some((v) => !v)) return null;
  return own.map((v) => (v === FSD_UNSET ? ha : v));
}

/**
 * The writes one pick makes, as [slot, option] pairs. The first pick on a device whose slots are
 * both still `unset` writes the other slot too, pinned to Home Assistant's current value: that is
 * what it was showing, and left `unset` it would start following whatever the picked slot copies
 * into HA. After that each slot is written on its own.
 */
export function fsdWrites(own, ha, i, option) {
  const first = own.every((v) => v === FSD_UNSET) && FSD_OPTIONS.some(([id]) => id === ha);
  return own.map((_, k) => (k === i ? [k, option] : first ? [k, ha] : null)).filter(Boolean);
}
