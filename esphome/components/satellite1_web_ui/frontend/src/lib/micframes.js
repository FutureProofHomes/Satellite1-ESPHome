/**
 * The mic monitor's wire format and recorder, kept free of browser APIs so node can test them.
 *
 * GET /api/sat1/mic (developer builds) is one chunked response that never ends. The browser hands
 * the body over de-chunked, so messages arrive back to back and split anywhere. Each message is a
 * 20-byte little-endian header and then frame_count interleaved int16 pairs:
 *
 *   0  u32 magic "S1M1"         12 u8  flags (1 muted, 2 mic idle, 4 XMOS not ready)
 *   4  u32 sample index         13 u8  assistant phase (device.js PHASE)
 *   8  u16 frame count          14 u8  wake sequence, bumped per detection
 *   10 u16 frames dropped       15 u8  wake slot (0-1, 2 stop word, 255 unknown)
 *                               16 u8  x3 wake word scores (Primary, Secondary, stop; 0-255)
 *                               19 u8  reserved
 *
 * Channel 0 is speech-to-text, exactly what voice_assistant receives; channel 1 is the wake word
 * channel after micro_wake_word's gain. A frame count of 0 is a keepalive. The sample index counts
 * 16 kHz frames since the stream opened, so a jump in it is audio the device had to drop.
 */

export const MAGIC = 0x314d3153;
export const HEADER = 20;
export const RATE = 16000;
export const FLAG_MUTED = 1;
export const FLAG_IDLE = 2;
export const FLAG_XMOS = 4;
export const SLOT_UNKNOWN = 255;

/** Feeds on body chunks, returns whole messages. A lost alignment resynchronises on the magic. */
export function createParser() {
  let buf = new Uint8Array(0);
  return {
    push(chunk) {
      const merged = new Uint8Array(buf.length + chunk.length);
      merged.set(buf, 0);
      merged.set(chunk, buf.length);
      const out = [];
      const view = new DataView(merged.buffer);
      let off = 0;
      while (merged.length - off >= HEADER) {
        if (view.getUint32(off, true) !== MAGIC) {
          off++;
          continue;
        }
        const count = view.getUint16(off + 8, true);
        const len = HEADER + count * 4;
        if (merged.length - off < len) break;
        const stt = new Int16Array(count);
        const ww = new Int16Array(count);
        for (let i = 0, p = off + HEADER; i < count; i++, p += 4) {
          stt[i] = view.getInt16(p, true);
          ww[i] = view.getInt16(p + 2, true);
        }
        out.push({
          seq: view.getUint32(off + 4, true),
          count,
          dropped: view.getUint16(off + 10, true),
          flags: merged[off + 12],
          phase: merged[off + 13],
          wakeSeq: merged[off + 14],
          wakeSlot: merged[off + 15],
          scores: [merged[off + 16], merged[off + 17], merged[off + 18]],
          stt,
          ww,
        });
        off += len;
      }
      buf = merged.slice(off);
      return out;
    },
  };
}

/** One message, encoded the way the device does. For tests and nothing else. */
export function encodeFrame({ seq = 0, dropped = 0, flags = 0, phase = 0, wakeSeq = 0, wakeSlot = SLOT_UNKNOWN, scores = [0, 0, 0], stt = [], ww = [] }) {
  const count = stt.length;
  const out = new Uint8Array(HEADER + count * 4);
  const v = new DataView(out.buffer);
  v.setUint32(0, MAGIC, true);
  v.setUint32(4, seq, true);
  v.setUint16(8, count, true);
  v.setUint16(10, dropped, true);
  out[12] = flags;
  out[13] = phase;
  out[14] = wakeSeq;
  out[15] = wakeSlot;
  out.set(scores, 16);
  for (let i = 0; i < count; i++) {
    v.setInt16(HEADER + i * 4, stt[i], true);
    v.setInt16(HEADER + i * 4 + 2, ww[i], true);
  }
  return out;
}

const SLOT_NAME = ["Primary", "Secondary", "Stop word"];

/**
 * What changed between two headers, as recorder markers: a wake detection, an assistant phase
 * change, the XMOS going away or coming back, the mute switching. `phaseName` maps a phase number
 * to words (device.js PHASE).
 */
export function headerEvents(prev, cur, phaseName = (p) => `phase ${p}`) {
  if (!prev) return [];
  const out = [];
  if (cur.wakeSeq !== prev.wakeSeq) {
    const slot = SLOT_NAME[cur.wakeSlot];
    out.push({ kind: "wake", text: slot ? `Wake word (${slot})` : "Wake word" });
  }
  if (cur.phase !== prev.phase && cur.phase !== 0) out.push({ kind: "phase", text: phaseName(cur.phase) });
  const xmosWas = !!(prev.flags & FLAG_XMOS);
  const xmosIs = !!(cur.flags & FLAG_XMOS);
  if (xmosIs !== xmosWas) out.push({ kind: "xmos", text: xmosIs ? "XMOS not ready" : "XMOS ready" });
  const muteWas = !!(prev.flags & FLAG_MUTED);
  const muteIs = !!(cur.flags & FLAG_MUTED);
  if (muteIs !== muteWas) out.push({ kind: "mute", text: muteIs ? "Muted" : "Unmuted" });
  return out;
}

/** Fifteen minutes: about 29 MB per channel, which a phone can still hold twice. */
export const MAX_RECORD_SECONDS = 15 * 60;
/** The longest silence a single gap may insert, so a stalled tab cannot allocate without limit. */
const MAX_GAP_SECONDS = 60;

/**
 * Collects both channels as Int16Array chunks, never concatenated, so a 15-minute take costs its
 * samples once. Missing audio becomes silence of the right length and a marker, so the timeline in
 * the file is the timeline in the room:
 *   - a sample index jump within one stream (the device dropped frames for a slow reader);
 *   - a reconnect (`restart`), measured on the browser's clock;
 *   - a stretch with the microphone idle, e.g. an XMOS install, also measured on the browser's
 *     clock because the device's index does not advance while it hears nothing.
 * Markers carry the frame they apply to.
 */
export function createRecorder({ rate = RATE, maxSeconds = MAX_RECORD_SECONDS } = {}) {
  const stt = [];
  const ww = [];
  const markers = [];
  const max = rate * maxSeconds;
  let frames = 0;
  let expect = null;
  let lastAudioWall = null;
  let idleSince = null;
  let full = false;

  const silence = (n, kind, text) => {
    n = Math.min(Math.max(0, Math.round(n)), rate * MAX_GAP_SECONDS, max - frames);
    if (n <= 0) return;
    markers.push({ at: frames, kind, text });
    stt.push(new Int16Array(n));
    ww.push(new Int16Array(n));
    frames += n;
  };

  return {
    get frames() {
      return frames;
    },
    get full() {
      return full;
    },
    markers,
    stt,
    ww,
    /** One parsed message, with the wall-clock ms it arrived. False once the take is full. */
    add(f, wall) {
      if (full) return false;
      if (f.count === 0) {
        if (f.flags & FLAG_IDLE && idleSince == null && lastAudioWall != null) idleSince = lastAudioWall;
        return true;
      }
      if (idleSince != null) {
        silence(((wall - idleSince) / 1000) * rate - f.count, "idle", "Microphone idle");
        idleSince = null;
      } else if (expect != null && f.seq !== expect) {
        const gap = (f.seq - expect) >>> 0;
        if (gap > 0 && gap < 0x80000000) silence(gap, "gap", "Audio dropped");
      }
      const room = max - frames;
      const n = Math.min(f.count, room);
      stt.push(n === f.count ? f.stt : f.stt.slice(0, n));
      ww.push(n === f.count ? f.ww : f.ww.slice(0, n));
      frames += n;
      expect = (f.seq + f.count) >>> 0;
      lastAudioWall = wall;
      if (frames >= max) full = true;
      return !full;
    },
    /** The stream dropped and came back after `gapMs`: its sample index starts over. */
    restart(gapMs) {
      expect = null;
      idleSince = null;
      if (!full) silence((gapMs / 1000) * rate, "reconnect", "Stream reconnected");
      lastAudioWall = null;
    },
    mark(kind, text) {
      if (!full) markers.push({ at: frames, kind, text });
    },
  };
}

/** "satellite1-c5ac00_xmos-1.2.3_2026-10-05T14-03" - the stem both WAVs and the zip share. */
export function recordingStem(device, xmos, date) {
  const pad = (n) => String(n).padStart(2, "0");
  const when = `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())}T${pad(date.getHours())}-${pad(date.getMinutes())}`;
  const ver = /^v?(\d+\.\d+\.\d+[\w.-]*)$/.exec(String(xmos || "").trim());
  const name = String(device || "satellite1").replace(/[^A-Za-z0-9._-]+/g, "-") || "satellite1";
  return `${name}_xmos-${ver ? ver[1] : "unknown"}_${when}`;
}
