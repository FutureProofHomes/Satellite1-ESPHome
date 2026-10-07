/**
 * Writes test/ringfx-golden.txt: frames lib/ringfx.js draws, which test/ringfx.test.mjs holds the
 * JavaScript to and esphome/components/satellite1_ring/test/ring_fx_core_test.cpp holds the
 * firmware's renderer to. Run with `npm run golden` after changing the math in both, never to make
 * a failing parity test pass.
 *
 * One frame per line, space-separated, the 24 LEDs as 144 hex digits at the end:
 *   P preset moment t head0 r g b ring_on ratio_bp mic spk  frame   a preset's moment
 *   C moment fx cm n fl sp br p c0..c8 t head0 r g b ring_on ratio_bp mic spk  frame   a custom style
 * ratio_bp is the timer or volume ratio in basis points.
 */
import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

import {
  CM_BLEND, CM_OWN, CM_RAINBOW, CM_RING, F_REV, FX, FX_COMET, FX_DOT, FX_FLOW, FX_ORBIT, FX_RIPPLE, FX_SPIN, FX_WAVE, M_MUTE, M_RING,
  M_TIMER, M_VOL, MOMENTS, PRESETS, renderMoment, S, STYLES,
} from '../src/lib/ringfx.js';

export const RINGS = [[57, 194, 255], [255, 217, 160], [255, 64, 128]];
export const TIMES = [0, 480, 5037, 61000];

const hex = frame => frame.map(c => c.map(v => v.toString(16).padStart(2, '0')).join('')).join('');
const inp = (ring, on, bp, mic, spk) => ({ ring, ringOn: !!on, ratio: bp / 10000, micMuted: !!mic, spkSilent: !!spk });

export function goldenLines() {
  const lines = [];
  const variants = m => m === M_TIMER || m === M_VOL ? [[1, 0, 0, 0], [1, 3700, 0, 0], [1, 6200, 1, 0], [1, 10000, 0, 0]]
    : m === M_MUTE ? [[1, 0, 1, 0], [0, 0, 1, 1], [1, 0, 0, 1]]
      : m === M_RING ? [[1, 0, 0, 0], [1, 0, 1, 0]] : [[1, 0, 0, 0]];
  PRESETS.forEach(key => {
    STYLES[key].forEach((s, m) => {
      RINGS.slice(0, key === 'classic' ? 2 : 1).forEach((ring, ri) => {
        for (const [on, bp, mic, spk] of variants(m)) {
          for (const t of TIMES) {
            const head0 = (ri * 7 + t) % 24;
            const { frame } = renderMoment(m, s, t, head0, inp(ring, on, bp, mic, spk));
            lines.push(['P', key, MOMENTS[m], t, head0, ...ring, on, bp, mic, spk, hex(frame)].join(' '));
          }
        }
      });
    });
  });
  const own = { ...S(0, CM_OWN, 100), n: 3, c: [[56, 189, 248], [167, 139, 250], [244, 114, 182]] };
  const customs = [];
  for (let fx = 1; fx < FX.length - 1; fx++) {
    for (const cm of [CM_RING, CM_BLEND, CM_RAINBOW, CM_OWN]) {
      customs.push([1, { ...(cm === CM_OWN ? own : S(0, cm, 100)), fx, cm, sp: 73, br: 85, fl: fx % 2 ? F_REV : 0, p: fx === 4 ? 3 : fx === 5 ? 7 : 0 }]);
    }
  }
  // The other direction of everything that turns, every ripple start, and dots round the dial.
  for (const fx of [FX_SPIN, FX_COMET, FX_ORBIT, FX_WAVE, FX_FLOW]) customs.push([1, { ...own, fx, sp: 120, fl: fx % 2 ? 0 : F_REV }]);
  for (let p = 0; p < 4; p++) customs.push([1, { ...S(FX_RIPPLE, CM_BLEND, 90), p }]);
  for (const p of [5, 13, 23]) customs.push([1, { ...S(FX_DOT, CM_RING, 200), p }]);
  customs.push([M_TIMER, { ...own, fx: FX.length - 1, sp: 160, br: 60 }]);
  customs.push([M_VOL, { ...S(FX.length - 1, CM_BLEND, 100), fl: 4 }]);
  customs.forEach(([m, s], i) => {
    for (const t of [777, 20000]) {
      const ring = RINGS[i % RINGS.length], head0 = (i * 5) % 24, bp = m === M_TIMER || m === M_VOL ? 5100 : 0;
      const { frame } = renderMoment(m, s, t, head0, inp(ring, 1, bp, 0, 0));
      lines.push(['C', MOMENTS[m], s.fx, s.cm, s.n, s.fl, s.sp, s.br, s.p, ...s.c.flat(), t, head0, ...ring, 1, bp, 0, 0, hex(frame)].join(' '));
    }
  });
  return lines;
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  const lines = goldenLines();
  writeFileSync(new URL('./ringfx-golden.txt', import.meta.url), lines.join('\n') + '\n');
  console.log(`ringfx-golden.txt: ${lines.length} frames`);
}
