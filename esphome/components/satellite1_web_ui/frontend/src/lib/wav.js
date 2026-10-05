/**
 * Mono 16-bit PCM WAV, built as a list of parts so the samples are never copied into one buffer:
 * the recorder's Int16Array chunks go into the Blob (or the zip) as they are.
 *
 * After the samples come two optional chunks most audio tools read:
 *   - `cue ` with a `LIST/adtl` of `labl` names - the recorder's markers, at their sample offsets;
 *   - `LIST/INFO` - where the take came from (device, firmware, XMOS version, channel).
 * The data chunk comes first so a reader that stops at it still gets the audio.
 */

const enc = new TextEncoder();

function riffChunk(id, body) {
  const pad = body.length & 1;
  const out = new Uint8Array(8 + body.length + pad);
  out.set(enc.encode(id), 0);
  new DataView(out.buffer).setUint32(4, body.length, true);
  out.set(body, 8);
  return out;
}

const zstr = (s) => {
  const b = enc.encode(String(s));
  const out = new Uint8Array(b.length + 1);
  out.set(b, 0);
  return out;
};

const concat = (parts) => {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
};

/** RIFF INFO ids for the metadata keys this app writes. */
const INFO_IDS = { name: "INAM", device: "ISRC", software: "ISFT", comment: "ICMT", date: "ICRD" };

/**
 * `chunks`: Int16Array[]. `markers`: [{at, text}] with `at` in samples. `info`: any of name,
 * device, software, comment, date. Returns the parts in file order.
 */
export function wavParts(chunks, { rate = 16000, markers = [], info = {} } = {}) {
  const samples = chunks.reduce((n, c) => n + c.length, 0);
  const dataBytes = samples * 2;

  const tail = [];
  const cues = markers.filter((m) => m.at >= 0 && m.at <= samples);
  if (cues.length) {
    const cue = new Uint8Array(4 + cues.length * 24);
    const v = new DataView(cue.buffer);
    v.setUint32(0, cues.length, true);
    cues.forEach((m, i) => {
      const o = 4 + i * 24;
      v.setUint32(o, i + 1, true); // cue point id
      v.setUint32(o + 4, m.at, true); // position (play order)
      cue.set(enc.encode("data"), o + 8);
      v.setUint32(o + 12, 0, true); // chunk start
      v.setUint32(o + 16, 0, true); // block start
      v.setUint32(o + 20, m.at, true); // sample offset
    });
    tail.push(riffChunk("cue ", cue));
    const labels = cues.map((m, i) => {
      const id = new Uint8Array(4);
      new DataView(id.buffer).setUint32(0, i + 1, true);
      return riffChunk("labl", concat([id, zstr(m.text || m.kind || "marker")]));
    });
    tail.push(riffChunk("LIST", concat([enc.encode("adtl"), ...labels])));
  }
  const infos = Object.entries(INFO_IDS)
    .filter(([k]) => info[k])
    .map(([k, id]) => riffChunk(id, zstr(info[k])));
  if (infos.length) tail.push(riffChunk("LIST", concat([enc.encode("INFO"), ...infos])));
  const tailBytes = tail.reduce((n, p) => n + p.length, 0);

  const head = new Uint8Array(44);
  const v = new DataView(head.buffer);
  head.set(enc.encode("RIFF"), 0);
  v.setUint32(4, 36 + dataBytes + tailBytes, true);
  head.set(enc.encode("WAVEfmt "), 8);
  v.setUint32(16, 16, true);
  v.setUint16(20, 1, true); // PCM
  v.setUint16(22, 1, true); // mono
  v.setUint32(24, rate, true);
  v.setUint32(28, rate * 2, true);
  v.setUint16(32, 2, true);
  v.setUint16(34, 16, true);
  head.set(enc.encode("data"), 36);
  v.setUint32(40, dataBytes, true);

  return [head, ...chunks, ...tail];
}

export function wavBlob(chunks, opts) {
  return new Blob(wavParts(chunks, opts), { type: "audio/wav" });
}
