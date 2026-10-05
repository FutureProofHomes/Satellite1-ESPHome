/**
 * The store-only zip behind "Both (.zip)" (src/lib/zip.js). Every offset and checksum a reader
 * checks is walked here, so an archive that opens in node's eyes opens in Finder's.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { crc32, zipParts } from "../src/lib/zip.js";

const enc = new TextEncoder();
const join = (parts) => {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.byteLength, 0));
  let at = 0;
  for (const p of parts) {
    out.set(new Uint8Array(p.buffer, p.byteOffset, p.byteLength), at);
    at += p.byteLength;
  }
  return out;
};

test("crc32 matches the IEEE check value, across part boundaries too", () => {
  assert.equal(crc32([enc.encode("123456789")]), 0xcbf43926);
  assert.equal(crc32([enc.encode("1234"), enc.encode("56789")]), 0xcbf43926);
  assert.equal(crc32([]), 0);
});

test("crc32 reads a typed array's own bytes, not its whole buffer", () => {
  const big = new Int16Array([0x3231, 0x3433, 0x3635]);
  assert.equal(crc32([big.subarray(0, 2)]), crc32([enc.encode("1234")]));
});

test("the archive's local headers, central directory and end record agree", () => {
  const a = [enc.encode("RIFF"), new Int16Array([1, 2, 3])];
  const b = [enc.encode("second file, caf\u00e9")];
  const when = new Date(2026, 9, 5, 14, 3, 58);
  const z = join(zipParts([{ name: "a_speech.wav", parts: a }, { name: "b_wakeword.wav", parts: b }], when));
  const v = new DataView(z.buffer);

  const eocd = z.length - 22;
  assert.equal(v.getUint32(eocd, true), 0x06054b50);
  assert.equal(v.getUint16(eocd + 8, true), 2);
  assert.equal(v.getUint16(eocd + 10, true), 2);
  const cdSize = v.getUint32(eocd + 12, true);
  const cdAt = v.getUint32(eocd + 16, true);
  assert.equal(cdAt + cdSize, eocd);

  const expect = [
    { name: "a_speech.wav", data: join(a) },
    { name: "b_wakeword.wav", data: join(b) },
  ];
  let at = cdAt;
  for (const e of expect) {
    assert.equal(v.getUint32(at, true), 0x02014b50);
    const nameLen = v.getUint16(at + 28, true);
    assert.equal(new TextDecoder().decode(z.subarray(at + 46, at + 46 + nameLen)), e.name);
    assert.equal(v.getUint16(at + 8, true) & 0x0800, 0x0800, "UTF-8 names");
    assert.equal(v.getUint16(at + 10, true), 0, "stored");
    assert.equal(v.getUint32(at + 20, true), e.data.length);
    assert.equal(v.getUint32(at + 24, true), e.data.length);
    assert.equal(v.getUint32(at + 16, true), crc32([e.data]));
    const time = v.getUint16(at + 12, true);
    const day = v.getUint16(at + 14, true);
    assert.deepEqual([time >> 11, (time >> 5) & 63, (time & 31) * 2], [14, 3, 58]);
    assert.deepEqual([(day >> 9) + 1980, (day >> 5) & 15, day & 31], [2026, 10, 5]);

    const local = v.getUint32(at + 42, true);
    assert.equal(v.getUint32(local, true), 0x04034b50);
    assert.equal(v.getUint32(local + 14, true), crc32([e.data]));
    const lName = v.getUint16(local + 26, true);
    const dataAt = local + 30 + lName + v.getUint16(local + 28, true);
    assert.deepEqual(z.subarray(dataAt, dataAt + e.data.length), e.data);
    at += 46 + nameLen;
  }
});
