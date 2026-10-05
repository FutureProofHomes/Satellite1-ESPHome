/**
 * The recordings' WAV files (src/lib/wav.js): a header any player accepts, the samples untouched,
 * and the markers and provenance in the chunks audio editors read.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { wavParts } from "../src/lib/wav.js";

const join = (parts) => {
  const bytes = parts.map((p) => new Uint8Array(p.buffer, p.byteOffset, p.byteLength));
  const out = new Uint8Array(bytes.reduce((n, b) => n + b.length, 0));
  let at = 0;
  for (const b of bytes) {
    out.set(b, at);
    at += b.length;
  }
  return out;
};
const ascii = (b, at, n) => String.fromCharCode(...b.subarray(at, at + n));

/** The top-level chunks after "WAVE", as {id, at, size}; fails on a size that overruns the file. */
function chunks(b) {
  const v = new DataView(b.buffer);
  const out = [];
  for (let at = 12; at < b.length; ) {
    const size = v.getUint32(at + 4, true);
    assert.ok(at + 8 + size <= b.length, "chunk overruns the file");
    out.push({ id: ascii(b, at, 4), at: at + 8, size });
    at += 8 + size + (size & 1);
  }
  return out;
}

test("a plain take is a 16 kHz mono PCM file with the samples as given", () => {
  const b = join(wavParts([new Int16Array([1, -2]), new Int16Array([3])]));
  const v = new DataView(b.buffer);
  assert.equal(ascii(b, 0, 4), "RIFF");
  assert.equal(v.getUint32(4, true), b.length - 8);
  assert.equal(ascii(b, 8, 8), "WAVEfmt ");
  assert.equal(v.getUint16(20, true), 1, "PCM");
  assert.equal(v.getUint16(22, true), 1, "mono");
  assert.equal(v.getUint32(24, true), 16000);
  assert.equal(v.getUint32(28, true), 32000);
  assert.equal(v.getUint16(34, true), 16);
  assert.equal(ascii(b, 36, 4), "data");
  assert.equal(v.getUint32(40, true), 6);
  assert.deepEqual([v.getInt16(44, true), v.getInt16(46, true), v.getInt16(48, true)], [1, -2, 3]);
  assert.equal(b.length, 50);
});

test("markers become cue points with labels, at their sample offsets", () => {
  const b = join(wavParts([new Int16Array(100)], { markers: [{ at: 10, text: "Wake word (Primary)" }, { at: 60, kind: "gap" }, { at: 500, text: "past the end" }] }));
  const v = new DataView(b.buffer);
  const list = chunks(b);
  assert.deepEqual(list.map((c) => c.id), ["fmt ", "data", "cue ", "LIST"]);
  const cue = list[2];
  assert.equal(v.getUint32(cue.at, true), 2, "the out-of-range marker is dropped");
  assert.equal(v.getUint32(cue.at + 4 + 20, true), 10);
  assert.equal(v.getUint32(cue.at + 4 + 24 + 20, true), 60);
  const adtl = list[3];
  assert.equal(ascii(b, adtl.at, 4), "adtl");
  assert.equal(ascii(b, adtl.at + 4, 4), "labl");
  const text = ascii(b, adtl.at, adtl.size);
  assert.ok(text.includes("Wake word (Primary)\0"));
  assert.ok(text.includes("gap\0"), "a marker without text is labelled by its kind");
});

test("provenance goes in LIST/INFO, and odd-length strings keep every chunk word-aligned", () => {
  const info = { name: "Kitchen speech", device: "Kitchen (firmware 2.1.0)", software: "ESPHome 2026.9.1; XMOS v1.2.3", comment: "odd", date: "2026-10-05T14:03:00.000Z" };
  const b = join(wavParts([new Int16Array(4)], { markers: [{ at: 0, text: "a" }], info }));
  const list = chunks(b);
  const infoList = list[list.length - 1];
  assert.equal(infoList.id, "LIST");
  assert.equal(ascii(b, infoList.at, 4), "INFO");
  const text = ascii(b, infoList.at, infoList.size);
  for (const id of ["INAM", "ISRC", "ISFT", "ICMT", "ICRD"]) assert.ok(text.includes(id), id);
  assert.ok(text.includes("ESPHome 2026.9.1; XMOS v1.2.3\0"));
  for (const c of list) assert.equal((c.at - 8) % 2, 0, `${c.id} starts on an even byte`);
  assert.equal(new DataView(b.buffer).getUint32(4, true), b.length - 8);
});
