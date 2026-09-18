/**
 * The hand-rolled SHA-256 and HMAC in lib/auth.js, checked against node:crypto.
 *
 * These implementations exist because crypto.subtle is unavailable on the plain-HTTP origins this
 * app lives on, and they are the kind of code that fails silently and completely: a wrong HMAC
 * means no password login works anywhere, with nothing but a 401 to debug from. The device end is
 * mbedtls, so agreeing with node:crypto is agreeing with the device.
 *
 * Boundary lengths matter more than volume here: SHA-256 padding has an edge at 55/56 bytes (where
 * the length field no longer fits the last block) and at every multiple of 64; HMAC has one at key
 * length 64 (longer keys are hashed down first). All are covered explicitly, plus a random sweep.
 */
import assert from "node:assert/strict";
import { createHash, createHmac, randomBytes } from "node:crypto";
import test from "node:test";

import { hmacSha256, sha256, toHex } from "../src/lib/auth.js";

const refSha = (bytes) => createHash("sha256").update(bytes).digest("hex");
const refHmac = (key, msg) => createHmac("sha256", key).update(msg).digest("hex");

test("sha256 matches node:crypto at padding boundaries", () => {
  for (const n of [0, 1, 3, 31, 32, 54, 55, 56, 57, 63, 64, 65, 119, 127, 128, 129, 255, 256, 1000]) {
    const msg = randomBytes(n);
    assert.equal(toHex(sha256(msg)), refSha(msg), `length ${n}`);
  }
});

test("sha256 matches node:crypto over random lengths", () => {
  for (let i = 0; i < 200; i++) {
    const msg = randomBytes(Math.floor(Math.random() * 300));
    assert.equal(toHex(sha256(msg)), refSha(msg), `length ${msg.length}`);
  }
});

test("hmacSha256 matches node:crypto around the block-size key boundary", () => {
  for (const keyLen of [0, 1, 16, 32, 63, 64, 65, 100, 128]) {
    for (const msgLen of [0, 1, 32, 64, 200]) {
      const key = randomBytes(keyLen);
      const msg = randomBytes(msgLen);
      assert.equal(toHex(hmacSha256(key, msg)), refHmac(key, msg), `key ${keyLen}, msg ${msgLen}`);
    }
  }
});

test("the login answer shape: HMAC-SHA256(SHA-256(password), nonce)", () => {
  // The exact composition loginPassword() sends and handle_login_() verifies, over realistic
  // inputs: a generated-style password and a 32-hex-char nonce.
  const utf8 = (s) => new TextEncoder().encode(s);
  const password = "kjhg8734kjhwerkjh23";
  const nonce = randomBytes(16).toString("hex");
  const expected = refHmac(createHash("sha256").update(utf8(password)).digest(), utf8(nonce));
  assert.equal(toHex(hmacSha256(sha256(utf8(password)), utf8(nonce))), expected);
});
