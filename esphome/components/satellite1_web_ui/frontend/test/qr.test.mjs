/**
 * Round-trips lib/qr.js through jsQR, an independent decoder: every version the encoder claims
 * (1 through 6) must scan back to the exact input text. This is the committed form of the
 * verification the module header promises - a QR that renders but does not scan is worse than no
 * QR, because the person holding the phone blames the phone.
 */
import assert from "node:assert/strict";
import test from "node:test";

import jsQR from "jsqr";

import { qrSvgPath } from "../src/lib/qr.js";

/** Rasterizes the encoder's output - unit squares in an SVG path - to the RGBA buffer jsQR reads,
 *  with the spec's 4-module quiet zone and a scale big enough for the decoder's sampling. */
function rasterize({ size, path }) {
  const scale = 8;
  const quiet = 4;
  const px = (size + quiet * 2) * scale;
  const data = new Uint8ClampedArray(px * px * 4).fill(255);
  for (const m of path.matchAll(/M(\d+) (\d+)h1v1h-1z/g)) {
    const c = Number(m[1]) + quiet;
    const r = Number(m[2]) + quiet;
    for (let y = r * scale; y < (r + 1) * scale; y++) {
      for (let x = c * scale; x < (c + 1) * scale; x++) {
        const at = (y * px + x) * 4;
        data[at] = data[at + 1] = data[at + 2] = 0;
      }
    }
  }
  return { data, px };
}

const roundTrip = (text) => {
  const qr = qrSvgPath(text);
  assert.ok(qr, `encoder refused ${text.length} chars`);
  const { data, px } = rasterize(qr);
  const decoded = jsQR(data, px, px);
  assert.ok(decoded, `jsQR could not read the ${text.length}-char code (version ${(qr.size - 17) / 4})`);
  assert.equal(decoded.data, text);
};

test("every version 1-6 round-trips through jsQR", () => {
  // Lengths chosen to land in each version's capacity band (data codewords minus the 2-byte
  // header): v1 <=17, v2 <=32, v3 <=53, v4 <=78, v5 <=106, v6 <=134.
  for (const len of [10, 17, 18, 32, 40, 53, 60, 78, 90, 106, 120, 134]) {
    roundTrip("x".repeat(len));
  }
});

test("the sign-in link shape round-trips", () => {
  // The real payload: longest hostname ESPHome allows (31 chars) plus the 64-hex-char key.
  const link = `http://satellite1-abcdef-longest31.local/?key=${"ab".repeat(32)}`;
  roundTrip(link);
});

test("over-capacity input is refused rather than truncated", () => {
  assert.equal(qrSvgPath("x".repeat(135)), null);
});
