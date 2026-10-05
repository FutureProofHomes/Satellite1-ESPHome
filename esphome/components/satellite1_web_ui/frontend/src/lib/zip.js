/**
 * A store-only ZIP: no compression, because WAV barely compresses and deflate in JS on a phone is
 * slow. Each entry is a list of parts (typed arrays), so the WAVs go in without a copy and the
 * Blob is assembled from the same buffers. Finder, Windows Explorer and iOS Files all open it.
 *
 * No zip64: every entry and the whole archive must stay under 4 GB, which two 15-minute takes do
 * by a factor of seventy.
 */

let table = null;
function crcTable() {
  if (table) return table;
  table = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    table[n] = c >>> 0;
  }
  return table;
}

const bytesOf = (p) => (p instanceof Uint8Array ? p : new Uint8Array(p.buffer, p.byteOffset, p.byteLength));

/** CRC-32 (IEEE) over a list of typed arrays, as if they were one. */
export function crc32(parts) {
  const t = crcTable();
  let c = 0xffffffff;
  for (const part of parts) {
    const b = bytesOf(part);
    for (let i = 0; i < b.length; i++) c = t[(c ^ b[i]) & 0xff] ^ (c >>> 8);
  }
  return (c ^ 0xffffffff) >>> 0;
}

function dosTime(d) {
  const time = (d.getHours() << 11) | (d.getMinutes() << 5) | (d.getSeconds() >> 1);
  const date = ((d.getFullYear() - 1980) << 9) | ((d.getMonth() + 1) << 5) | d.getDate();
  return { time, date };
}

/**
 * `entries`: [{name, parts}]. Returns the archive's parts in order: each local header followed by
 * its entry's own parts, then the central directory and its end record.
 */
export function zipParts(entries, date = new Date()) {
  const enc = new TextEncoder();
  const { time, date: day } = dosTime(date);
  const out = [];
  const central = [];
  let offset = 0;
  for (const e of entries) {
    const name = enc.encode(e.name);
    const size = e.parts.reduce((n, p) => n + p.byteLength, 0);
    const crc = crc32(e.parts);

    const local = new Uint8Array(30 + name.length);
    const lv = new DataView(local.buffer);
    lv.setUint32(0, 0x04034b50, true);
    lv.setUint16(4, 10, true); // version needed: stored
    lv.setUint16(6, 0x0800, true); // names are UTF-8
    lv.setUint16(8, 0, true); // stored
    lv.setUint16(10, time, true);
    lv.setUint16(12, day, true);
    lv.setUint32(14, crc, true);
    lv.setUint32(18, size, true);
    lv.setUint32(22, size, true);
    lv.setUint16(26, name.length, true);
    lv.setUint16(28, 0, true);
    local.set(name, 30);

    const cd = new Uint8Array(46 + name.length);
    const cv = new DataView(cd.buffer);
    cv.setUint32(0, 0x02014b50, true);
    cv.setUint16(4, 20, true); // made by: MS-DOS, spec 2.0
    cv.setUint16(6, 10, true);
    cv.setUint16(8, 0x0800, true);
    cv.setUint16(10, 0, true);
    cv.setUint16(12, time, true);
    cv.setUint16(14, day, true);
    cv.setUint32(16, crc, true);
    cv.setUint32(20, size, true);
    cv.setUint32(24, size, true);
    cv.setUint16(28, name.length, true);
    cv.setUint32(42, offset, true);
    cd.set(name, 46);
    central.push(cd);

    out.push(local, ...e.parts);
    offset += local.length + size;
  }
  const cdSize = central.reduce((n, c) => n + c.length, 0);
  const end = new Uint8Array(22);
  const ev = new DataView(end.buffer);
  ev.setUint32(0, 0x06054b50, true);
  ev.setUint16(8, entries.length, true);
  ev.setUint16(10, entries.length, true);
  ev.setUint32(12, cdSize, true);
  ev.setUint32(16, offset, true);
  out.push(...central, end);
  return out;
}

export function zipBlob(entries, date) {
  return new Blob(zipParts(entries, date), { type: "application/zip" });
}
