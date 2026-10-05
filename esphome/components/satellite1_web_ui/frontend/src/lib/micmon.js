/**
 * The mic monitor's runtime (Settings > Developer, developer builds only): one stream from
 * GET /api/sat1/mic, played, drawn and recorded. A module-level singleton, so listening and a
 * recording in progress survive leaving the page; the card is only a view onto it.
 *
 * Playback schedules AudioBufferSourceNodes about 200 ms ahead. AudioWorklet would be the modern
 * tool, but it needs a secure context and the device serves plain HTTP. A browser that refuses a
 * 16 kHz buffer (older WebKit) gets the samples upsampled linearly to its own rate instead.
 *
 * The stream is a plain fetch outside device.js's request queue, because it never ends and would
 * hold the queue forever. A dropped stream reconnects after 1, 2, 4, 8 and then every 10 seconds
 * while Listen is on; 409 (both listener slots taken), 503 (device low on memory), 404 and a lost
 * session stop it with a reason instead.
 */

import { apiUrl, isRemote, PHASE } from "./device.js";
import { FLAG_IDLE, RATE, createParser, createRecorder, headerEvents, recordingStem } from "./micframes.js";

/** The waveform window: 10 s in 20 ms buckets. */
const BUCKET = 320;
export const BUCKETS = (10 * RATE) / BUCKET;
/** Silence on the wire for this long means the connection is dead, keepalives being every 250 ms. */
const STALL_MS = 3000;
const BACKOFF = [1000, 2000, 4000, 8000, 10000];
/** Playback batches: fewer, larger buffer sources than one per message. */
const PLAY_BATCH = 800;

const subs = new Set();
const state = {
  /** off | connecting | streaming | reconnecting | busy | low_memory | unsupported | signed_out */
  status: "off",
  flags: 0,
  phase: 0,
  scores: [0, 0, 0],
  /** stt | ww | off - which channel plays. Both are always drawn. */
  channel: "stt",
  target: null,
  recording: null,
  recordings: [],
};

/** Per-bucket peaks and RMS (0-1) for both channels, NaN where nothing was heard, plus the three
 *  wake word scores (0-255). `head` is the next bucket to write. */
const viz = {
  head: 0,
  peak: [new Float32Array(BUCKETS).fill(NaN), new Float32Array(BUCKETS).fill(NaN)],
  rms: [new Float32Array(BUCKETS).fill(NaN), new Float32Array(BUCKETS).fill(NaN)],
  score: [new Uint8Array(BUCKETS), new Uint8Array(BUCKETS), new Uint8Array(BUCKETS)],
  clipAt: [0, 0],
};
const acc = { n: 0, peak: [0, 0], sq: [0, 0], wall: 0 };

let listening = false;
let abort = null;
let retryTimer = null;
let watchdog = null;
let attempt = 0;
let lastBytes = 0;
let droppedAt = null;
let prevHeader = null;
let notifyTimer = null;
let nextId = 1;

let ac = null;
let native16k = null;
let nextAt = 0;
let pend = new Float32Array(PLAY_BATCH * 2);
let pendLen = 0;

function notify(now = false) {
  if (now) {
    clearTimeout(notifyTimer);
    notifyTimer = null;
    for (const fn of subs) fn();
    return;
  }
  if (notifyTimer) return;
  notifyTimer = setTimeout(() => {
    notifyTimer = null;
    for (const fn of subs) fn();
  }, 250);
}

function setStatus(s) {
  if (state.status === s) return;
  state.status = s;
  notify(true);
}

export function subscribe(fn) {
  subs.add(fn);
  return () => subs.delete(fn);
}

export const micState = () => state;
export const micViz = () => viz;
/** Streaming or recording: what the nav's dot shows. */
export const micActive = () => listening || !!state.recording;

function pushBucket() {
  const h = viz.head;
  for (let c = 0; c < 2; c++) {
    viz.peak[c][h] = acc.n ? acc.peak[c] / 32768 : NaN;
    viz.rms[c][h] = acc.n ? Math.sqrt(acc.sq[c] / acc.n) / 32768 : NaN;
    acc.peak[c] = 0;
    acc.sq[c] = 0;
  }
  for (let s = 0; s < 3; s++) viz.score[s][h] = state.scores[s];
  viz.head = (h + 1) % BUCKETS;
  acc.n = 0;
}

function drawFrame(f, wall) {
  const now = wall;
  for (let i = 0; i < f.count; i++) {
    const a = f.stt[i];
    const b = f.ww[i];
    const aa = a < 0 ? -a : a;
    const bb = b < 0 ? -b : b;
    if (aa > acc.peak[0]) acc.peak[0] = aa;
    if (bb > acc.peak[1]) acc.peak[1] = bb;
    if (aa >= 32767) viz.clipAt[0] = now;
    if (bb >= 32767) viz.clipAt[1] = now;
    acc.sq[0] += a * a;
    acc.sq[1] += b * b;
    if (++acc.n === BUCKET) pushBucket();
  }
  acc.wall = wall;
}

/** The microphone is idle (an XMOS install, say): keep the window rolling with gaps. */
function drawIdle(wall) {
  if (!acc.wall) acc.wall = wall;
  const steps = Math.min(BUCKETS, Math.floor((wall - acc.wall) / 20));
  if (steps <= 0) return;
  acc.n = 0;
  for (let i = 0; i < steps; i++) pushBucket();
  acc.wall = wall;
}

/** Peak and RMS over the last 100 ms, in dBFS; -Infinity for silence or nothing heard. */
export function micLevels(c) {
  let peak = 0;
  let sq = 0;
  let n = 0;
  for (let k = 1; k <= 5; k++) {
    const i = (viz.head - k + BUCKETS) % BUCKETS;
    const p = viz.peak[c][i];
    if (Number.isNaN(p)) continue;
    peak = Math.max(peak, p);
    sq += viz.rms[c][i] ** 2;
    n++;
  }
  const db = (v) => (v > 0 ? 20 * Math.log10(v) : -Infinity);
  return { peak: db(peak), rms: n ? db(Math.sqrt(sq / n)) : -Infinity, clip: performance.now() - viz.clipAt[c] < 2000 && viz.clipAt[c] > 0 };
}

/* ------------------------------------------------------------------ */
/* Playback                                                            */
/* ------------------------------------------------------------------ */

function ensureAudio() {
  if (typeof window === "undefined") return;
  if (!ac) {
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return;
    ac = new AC();
  }
  if (ac.state === "suspended") ac.resume().catch(() => {});
}

function schedule(f32) {
  let buf = null;
  if (native16k !== false) {
    try {
      buf = ac.createBuffer(1, f32.length, RATE);
      native16k = true;
      buf.getChannelData(0).set(f32);
    } catch {
      native16k = false;
      buf = null;
    }
  }
  if (!buf) {
    const ratio = ac.sampleRate / RATE;
    const m = Math.floor(f32.length * ratio);
    buf = ac.createBuffer(1, m, ac.sampleRate);
    const out = buf.getChannelData(0);
    for (let i = 0; i < m; i++) {
      const x = i / ratio;
      const j = Math.floor(x);
      const t = x - j;
      out[i] = f32[j] * (1 - t) + (f32[Math.min(j + 1, f32.length - 1)] ?? 0) * t;
    }
  }
  const now = ac.currentTime;
  // Primes the jitter buffer after a stall, and drops latency back down after a backlog.
  if (nextAt < now + 0.03 || nextAt > now + 1.0) nextAt = now + 0.2;
  const src = ac.createBufferSource();
  src.buffer = buf;
  src.connect(ac.destination);
  src.start(nextAt);
  nextAt += buf.duration;
}

function play(samples) {
  if (state.channel === "off" || !ac || ac.state !== "running") return;
  if (pendLen + samples.length > pend.length) {
    const grown = new Float32Array((pendLen + samples.length) * 2);
    grown.set(pend.subarray(0, pendLen));
    pend = grown;
  }
  for (let i = 0; i < samples.length; i++) pend[pendLen + i] = samples[i] / 32768;
  pendLen += samples.length;
  if (pendLen < PLAY_BATCH) return;
  schedule(pend.slice(0, pendLen));
  pendLen = 0;
}

export function setChannel(ch) {
  state.channel = ch;
  pendLen = 0;
  nextAt = 0;
  if (ch !== "off") ensureAudio();
  notify(true);
}

/* ------------------------------------------------------------------ */
/* The stream                                                          */
/* ------------------------------------------------------------------ */

function onFrame(f, wall) {
  const rec = state.recording;
  if (rec) {
    for (const ev of headerEvents(prevHeader, f, (p) => PHASE[p] || `Phase ${p}`)) rec.recorder.mark(ev.kind, ev.text);
  }
  prevHeader = f;
  state.flags = f.flags;
  state.phase = f.phase;
  state.scores = f.scores;
  if (f.count > 0) {
    drawFrame(f, wall);
    play(state.channel === "ww" ? f.ww : f.stt);
  } else if (f.flags & FLAG_IDLE) {
    drawIdle(wall);
  }
  if (rec && !rec.recorder.add(f, wall)) stopRecording();
  notify();
}

function clearTimers() {
  clearTimeout(retryTimer);
  clearInterval(watchdog);
  retryTimer = null;
  watchdog = null;
}

function fail(status) {
  listening = false;
  clearTimers();
  if (state.recording) stopRecording();
  setStatus(status);
}

function retry() {
  if (!listening) return;
  droppedAt ??= Date.now();
  setStatus("reconnecting");
  const wait = BACKOFF[Math.min(attempt, BACKOFF.length - 1)];
  attempt++;
  retryTimer = setTimeout(connect, wait);
}

async function connect() {
  clearTimers();
  if (!listening) return;
  const ctl = new AbortController();
  abort = ctl;
  if (state.status !== "reconnecting") setStatus("connecting");
  let r;
  try {
    r = await fetch(apiUrl("/api/sat1/mic"), {
      signal: ctl.signal,
      cache: "no-store",
      ...(isRemote() ? { mode: "cors", credentials: "omit" } : {}),
    });
  } catch {
    if (ctl === abort) retry();
    return;
  }
  if (ctl !== abort || !listening) return;
  if (r.status === 409) return fail("busy");
  if (r.status === 503) return fail("low_memory");
  if (r.status === 404) return fail("unsupported");
  if (r.status === 401 || r.status === 403) return fail("signed_out");
  if (!r.ok || !r.body) return retry();

  const startedAt = Date.now();
  if (droppedAt != null && state.recording) state.recording.recorder.restart(startedAt - droppedAt);
  droppedAt = null;
  prevHeader = null;
  lastBytes = startedAt;
  setStatus("streaming");
  watchdog = setInterval(() => {
    if (Date.now() - lastBytes > STALL_MS) ctl.abort();
  }, 1000);

  const parser = createParser();
  const reader = r.body.getReader();
  try {
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      lastBytes = Date.now();
      const wall = performance.now();
      for (const f of parser.push(value)) onFrame(f, wall);
    }
  } catch {
    /* aborted or dropped; decided below */
  }
  clearInterval(watchdog);
  if (ctl !== abort || !listening) return;
  if (Date.now() - startedAt > 5000) attempt = 0;
  droppedAt = Date.now();
  retry();
}

/** Starts listening to the device the app is pointed at. Call from a tap, so audio may start. */
export function startListening(target) {
  ensureAudio();
  if (listening) return;
  listening = true;
  attempt = 0;
  droppedAt = null;
  state.target = target;
  connect();
}

export function stopListening() {
  listening = false;
  clearTimers();
  abort?.abort();
  abort = null;
  if (state.recording) stopRecording();
  pendLen = 0;
  setStatus("off");
}

/** The app switched to another device: whatever was streaming belonged to the old one. */
export function retarget(target) {
  if (state.target != null && target !== state.target && (listening || state.recording)) stopListening();
  state.target = target;
}

/* ------------------------------------------------------------------ */
/* Recording                                                           */
/* ------------------------------------------------------------------ */

/** `meta`: { device, fw, esphome, xmos } - for the file names and the WAVs' INFO chunk. */
export function startRecording(meta) {
  if (state.recording || !listening) return;
  const startedAt = new Date();
  state.recording = { startedAt, meta: { ...meta, xmosNow: meta.xmos || null }, recorder: createRecorder() };
  notify(true);
}

export function stopRecording() {
  const rec = state.recording;
  if (!rec) return;
  state.recording = null;
  const r = rec.recorder;
  if (r.frames > 0) {
    state.recordings = [
      {
        id: nextId++,
        stem: recordingStem(rec.meta.device, rec.meta.xmos, rec.startedAt),
        startedAt: rec.startedAt,
        seconds: r.frames / RATE,
        full: r.full,
        markers: r.markers.slice(),
        stt: r.stt,
        ww: r.ww,
        meta: rec.meta,
      },
      ...state.recordings,
    ];
  }
  notify(true);
}

export function removeRecording(id) {
  state.recordings = state.recordings.filter((r) => r.id !== id);
  notify(true);
}

/** The XMOS firmware entity changed while recording: the moment a picker install lands. Its
 *  in-between states ("Flashing Mode", "XMOS not responding") are already the XMOS flag's markers. */
export function noteXmosVersion(v) {
  const rec = state.recording;
  if (!rec || !/^v?\d+\.\d+/.test(v || "") || v === rec.meta.xmosNow) return;
  if (rec.meta.xmosNow != null) rec.recorder.mark("firmware", `XMOS firmware ${v}`);
  rec.meta.xmosNow = v;
}

/** Seconds recorded so far, for the card's clock. */
export const recordingSeconds = () => (state.recording ? state.recording.recorder.frames / RATE : 0);
