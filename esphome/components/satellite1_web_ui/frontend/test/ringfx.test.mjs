/**
 * The LED ring renderer: lib/ringfx.js against its golden frames (the same file the firmware's host
 * test reads, so the browser and the device stay frame-for-frame identical), Classic's looks pinned
 * to the firmware it replaced, and the wire format the /api/sat1/ring routes speak.
 */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import test from 'node:test';

import { goldenLines } from './golden.mjs';
import {
  clampStyle, CM_BLEND, CM_OWN, CM_RAINBOW, CM_RED, CM_RING, F_IF_ON, F_NODIP, F_REV, fromWire, FX_ARC, FX_COMET, FX_DOT, FX_FLOW,
  FX_ORBIT, FX_RIPPLE, FX_SOLID, FX_SPIN, FX_WAVE, hash32, hexRgb, M_ERR, M_LISTEN, M_MUTE, M_RING, M_THINK, M_TIMER, M_VOL, M_WAKE,
  momentTag, normalizeRing, renderMoment, resolve, rgbToHsv, S, scale8, STYLES, toQuery
} from '../src/lib/ringfx.js';

const ARCTIC = [57, 194, 255];
const inp = (o = {}) => ({ ring: ARCTIC, ringOn: true, ratio: 0, micMuted: false, spkSilent: false, ...o });
const frame = (m, s, t, head0 = 0, o) => renderMoment(m, s, t, head0, inp(o)).frame;

test('the golden frames still come out of ringfx.js', () => {
  const file = readFileSync(new URL('./ringfx-golden.txt', import.meta.url), 'utf8').trim().split('\n');
  const now = goldenLines();
  assert.equal(now.length, file.length, 'frame count changed: regenerate with npm run golden');
  now.forEach((line, i) => assert.equal(line, file[i], `frame ${i} drifted: ${line.slice(0, 60)}`));
});

test('Classic listening is the two-blob spin with the mic cap', () => {
  const f = frame(M_LISTEN, STYLES.classic[M_LISTEN], 0, 3);
  assert.deepEqual(f[3], ARCTIC);
  assert.deepEqual(f[2], ARCTIC.map(v => scale8(v, 192)));
  assert.deepEqual(f[1], ARCTIC.map(v => scale8(v, 128)));
  assert.deepEqual(f[15], ARCTIC);
  assert.deepEqual(f[4], [0, 0, 0]);
  // One step per 50 ms at listening speed; wake word heard moves at half that.
  assert.deepEqual(frame(M_LISTEN, STYLES.classic[M_LISTEN], 149, 3)[5], ARCTIC);
  assert.deepEqual(frame(M_WAKE, STYLES.classic[M_WAKE], 149, 3)[4], ARCTIC);
  // A head on a mic LED is held to half: brightest channel 128.
  const capped = frame(M_LISTEN, STYLES.classic[M_LISTEN], 0, 0)[0];
  assert.equal(Math.max(...capped), 128);
});

test('Classic thinking pulses LEDs 2 and 14 through the 10 ms triangle', () => {
  const s = STYLES.classic[M_THINK];
  assert.deepEqual(frame(M_THINK, s, 0)[2], ARCTIC.map(v => scale8(v, 250)));
  assert.deepEqual(frame(M_THINK, s, 100)[14], [0, 0, 0]);
  assert.deepEqual(frame(M_THINK, s, 50)[14], ARCTIC.map(v => scale8(v, 125)));
  assert.deepEqual(frame(M_THINK, s, 0)[3], [0, 0, 0]);
});

test('errors stay red and the mute marks are drawn whatever the style', () => {
  for (const key of Object.keys(STYLES)) {
    const err = frame(M_ERR, STYLES[key][M_ERR], 0);
    assert.ok(err.some(c => c[0] > 0) && err.every(c => c[1] === 0 && c[2] === 0), key);
    const mute = frame(M_MUTE, STYLES[key][M_MUTE], 1234, 0, { micMuted: true, spkSilent: true });
    for (const c of [0, 6, 12, 18]) assert.deepEqual(mute[c], [255, 0, 0], `${key} mic ${c}`);
    for (const st of [1, 7, 13, 19]) assert.deepEqual(mute[st + 1], [200, 0, 0], `${key} speaker ${st}`);
  }
});

test('muted draws its base only while the ring is on', () => {
  const s = STYLES.classic[M_MUTE];
  assert.deepEqual(frame(M_MUTE, s, 0, 0, { micMuted: true })[3], ARCTIC);
  assert.deepEqual(frame(M_MUTE, s, 0, 0, { micMuted: true, ringOn: false })[3], [0, 0, 0]);
});

test('the timer arc is the time left, with the travelling dip', () => {
  const s = STYLES.classic[M_TIMER];
  const f = frame(M_TIMER, s, 250, 0, { ratio: 0.5 });
  assert.deepEqual(f[11], ARCTIC.map(v => scale8(v, 255)));
  assert.deepEqual(f[12], [0, 0, 0]);
  assert.deepEqual(f[22], [0, 0, 0], 'the dip has moved two LEDs back from 0');
  const dipped = frame(M_TIMER, s, 250, 0, { ratio: 1 });
  assert.deepEqual(dipped[22], ARCTIC.map(v => scale8(v, 229)));
  const vol0 = frame(M_VOL, STYLES.classic[M_VOL], 0, 0, { ratio: 0 });
  assert.deepEqual(vol0[0], [255, 0, 0]);
});

test('spin heads and comets run counter-clockwise when reversed', () => {
  const rev = { ...S(FX_SPIN, CM_RING, 100), fl: F_REV };
  assert.deepEqual(frame(M_LISTEN, rev, 100, 10)[8], ARCTIC);
  const comet = { ...S(FX_COMET, CM_RING, 100), p: 4 };
  const at = frame(M_LISTEN, comet, 0, 10);
  assert.deepEqual(at[10], ARCTIC);
  assert.ok(at[9][2] > at[8][2] && at[8][2] > at[7][2] && at[6][2] === 0);
});

test('orbit, wave and flow turn clockwise, or counter-clockwise when reversed', () => {
  // Orbit steps one LED per 333 ms at speed 100, from LED 2.
  const orbit = S(FX_ORBIT, CM_RING, 100);
  const cw = frame(M_THINK, orbit, 340), ccw = frame(M_THINK, { ...orbit, fl: F_REV }, 340);
  assert.ok(cw[3][2] > 0 && cw[1][2] === 0);
  assert.ok(ccw[1][2] > 0 && ccw[3][2] === 0);
  // The wave's crest starts on LED 3, and 50 ms later leans the way it travels.
  const wave = S(FX_WAVE, CM_RING, 100);
  const wcw = frame(M_THINK, wave, 50), wccw = frame(M_THINK, { ...wave, fl: F_REV }, 50);
  assert.ok(wcw[4][2] > wcw[2][2]);
  assert.ok(wccw[2][2] > wccw[4][2]);
  // A twenty-fourth of a turn later, every LED has the color its neighbour had.
  const near = (a, b) => a.every((v, k) => Math.abs(v - b[k]) <= 1);
  const flow = S(FX_FLOW, CM_RAINBOW, 50);
  const f0 = frame(M_RING, flow, 0), fcw = frame(M_RING, flow, 463), fccw = frame(M_RING, { ...flow, fl: F_REV }, 463);
  for (let i = 0; i < 24; i++) {
    assert.ok(near(fcw[i], f0[(i + 23) % 24]), `clockwise ${i}`);
    assert.ok(near(fccw[i], f0[(i + 1) % 24]), `counter-clockwise ${i}`);
  }
});

test('flow turns the colors it is given; with one color it is a steady ring', () => {
  const f = frame(M_RING, S(FX_FLOW, CM_RING, 100), 1234);
  assert.ok(f.every(c => c.join() === ARCTIC.join()));
});

test('a ripple starts at the side it is given, and the dot sits where it is put', () => {
  for (let p = 0; p < 4; p++) {
    const f = frame(M_RING, { ...S(FX_RIPPLE, CM_RING, 100), p }, 0);
    f.forEach((c, i) => assert.deepEqual(c, i === p * 6 ? ARCTIC : [0, 0, 0], `start ${p}, LED ${i}`));
  }
  const dot = frame(M_RING, { ...S(FX_DOT, CM_RING, 100), p: 7 }, 1000);
  dot.forEach((c, i) => assert.equal(c[2] > 0, i === 7, `LED ${i}`));
});

test('clampStyle keeps each moment inside what it allows', () => {
  assert.equal(clampStyle(M_TIMER, S(FX_SPIN, CM_RING, 100)).fx, FX_ARC);
  assert.equal(clampStyle(M_LISTEN, S(FX_ARC, CM_RING, 100)).fx, FX_SOLID);
  assert.equal(clampStyle(M_ERR, S(FX_SPIN, CM_RING, 100)).cm, CM_RED);
  assert.equal(clampStyle(M_LISTEN, S(FX_SPIN, CM_RED, 100)).cm, CM_RING);
  assert.equal(clampStyle(M_VOL, S(FX_ARC, CM_RING, 100)).fl & F_NODIP, F_NODIP);
  assert.equal(clampStyle(M_TIMER, S(FX_ARC, CM_RING, 100, 100, F_NODIP | F_REV)).fl, F_REV);
  assert.equal(clampStyle(M_MUTE, S(FX_SOLID, CM_RING, 100)).fl & F_IF_ON, F_IF_ON);
  const s = clampStyle(M_LISTEN, { ...S(FX_SPIN, CM_RING, 9999, 0), p: 9 });
  assert.deepEqual([s.sp, s.br, s.p], [400, 5, 4]);
  assert.equal(clampStyle(M_LISTEN, { ...S(FX_RIPPLE, CM_RING, 100), p: 9 }).p, 3);
  assert.equal(clampStyle(M_LISTEN, { ...S(FX_DOT, CM_RING, 100), p: 30 }).p, 23);
});

test('muted is the same in every style, whatever is asked of it: the marks over the idle ring', () => {
  for (const key of Object.keys(STYLES)) assert.deepEqual(STYLES[key][M_MUTE], STYLES.classic[M_MUTE], key);
  assert.deepEqual(clampStyle(M_MUTE, { ...S(FX_WAVE, CM_BLEND, 30, 50), p: 3 }), STYLES.classic[M_MUTE]);
  assert.deepEqual(STYLES.classic[M_MUTE], S(FX_SOLID, CM_RING, 100, 100, F_IF_ON));
});

test('a moment round-trips through the API wire format', () => {
  const s = { ...S(FX_COMET, CM_OWN, 62, 85, F_REV, 12), n: 2, c: [[56, 189, 248], [167, 139, 250], [0, 0, 0]] };
  const q = toQuery(M_LISTEN, s);
  assert.deepEqual(q, { m: 'listen', fx: 'comet', cm: 'own', sp: 62, br: 85, dir: -1, p: 12, c: '38bdf8,a78bfa' });
  const back = fromWire({ fx: 'comet', cm: 'own', sp: 62, br: 85, fl: F_REV, p: 12, c: ['38bdf8', 'a78bfa'] });
  assert.deepEqual(back, s);
  assert.equal(momentTag(back, STYLES.classic[M_LISTEN]), 'own');
  assert.equal(momentTag({ ...STYLES.classic[M_LISTEN], sp: 120 }, STYLES.classic[M_LISTEN]), 'edit');
  assert.equal(momentTag(STYLES.classic[M_LISTEN], STYLES.classic[M_LISTEN]), '');
});

test('helpers', () => {
  assert.deepEqual(normalizeRing([28, 94, 124]), [57, 193, 255]);
  assert.equal(hash32(3, 7), hash32(3, 7));
  assert.ok(hash32(1, 1) >= 0 && hash32(1, 1) < 2 ** 32);
});

test('the ring blend spreads a pale ring color into three distinct colors', () => {
  const sapphire = normalizeRing(hexRgb('#60a5fa'));
  const { n, c } = resolve(S(FX_SOLID, CM_BLEND, 100), sapphire);
  assert.equal(n, 3);
  assert.deepEqual(c[1], sapphire);
  const [h, s] = rgbToHsv(sapphire);
  const [h0, s0] = rgbToHsv(c[0]), [h2, s2] = rgbToHsv(c[2]);
  assert.ok(Math.abs(h0 - (h - 60)) < 2 && Math.abs(h2 - (h + 60)) < 2, `${h0} ${h} ${h2}`);
  assert.ok(s0 > s + 0.15 && s2 > s + 0.15, `${s0} ${s} ${s2}`);
});
