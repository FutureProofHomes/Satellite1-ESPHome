/**
 * The speaker EQ's maths (src/lib/eq.js) against the XMOS it previews: lib_audio_dsp's own
 * coefficients, the design task's headroom walk and section budget, and the filter files people
 * bring from Music Assistant, REW and Equalizer APO.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  T, analyse, autoPreamp, biquads, clampProfile, exportText, filterCurve, filtersParam, fmtDb, fmtHz, fromWire,
  handleDb, importText, kindOf, logTrackValue, profileBody, profileCurve, profileSections, sameSound, sectionsOf, slopeOf,
  typeOf,
} from "../src/lib/eq.js";

const close = (a, b, eps, what) => assert.ok(Math.abs(a - b) <= eps, `${what}: ${a} vs ${b}`);

/** "FPH Large Desk Smart Speaker", as satellite1_dsp.cpp's make_fph() builds it. */
const FPH = {
  pre: 10, bass: 0, tre: 0, lim: -1, atk: 1, rel: 100, preset: "FPH Large Desk Smart Speaker",
  f: [
    { t: T.PEAK, on: true, hz: 138, db: -3, q: 1.93 },
    { t: T.PEAK, on: true, hz: 1600, db: -4.5, q: 4 },
    { t: T.PEAK, on: true, hz: 3455, db: -8, q: 2.66 },
    { t: T.PEAK, on: true, hz: 5700, db: 7, q: 3.42 },
    { t: T.PEAK, on: true, hz: 13700, db: -10, q: 1 },
    { t: T.HP_Q, on: true, hz: 60, db: 0, q: 1 },
  ],
};
const FLAT = { pre: 0, bass: 0, tre: 0, lim: -1, atk: 1, rel: 100, preset: "Flat", f: [] };

/** music_assistant_eq_settings_2026-10-10.txt, the tuning FPH was made from, byte for byte. */
const MA_EXPORT = `Preamp: 10.00 dB
Filter 1: ON PK Fc 138.0 Hz Gain -3.00 dB Q 1.930
Filter 2: ON PK Fc 1600.0 Hz Gain -4.50 dB Q 4.000
Filter 3: ON PK Fc 3455.0 Hz Gain -8.00 dB Q 2.660
Filter 4: ON PK Fc 5700.0 Hz Gain 7.00 dB Q 3.420
Filter 5: ON PK Fc 13700.0 Hz Gain -10.00 dB Q 1.000
Filter 6: ON HP Fc 60.0 Hz Gain 0.00 dB Q 1.000
`;

test("coefficients match lib_audio_dsp's make_biquad_* at 48 kHz", () => {
  // Printed by lib_audio_dsp v1.5.1's python/audio_dsp/dsp/biquad.py: [b0, b1, b2, -a1, -a2].
  const REF = [
    [{ t: T.PEAK, hz: 5700, q: 3.42, db: 7 }, [1.07704946, -1.3772941, 0.798548912, 1.3772941, -0.875598374]],
    [{ t: T.PEAK, hz: 138, q: 1.93, db: -3 }, [0.998384669, -1.98861364, 0.990553473, 1.98861364, -0.988938142]],
    [{ t: T.LOW_SHELF, hz: 120, q: 0.707, db: 6 }, [1.00386324, -1.98122043, 0.977702497, 1.98130655, -0.981479614]],
    [{ t: T.HIGH_SHELF, hz: 8000, q: 0.707, db: -4.5 }, [0.712328869, -0.342737703, 0.151534477, 0.752260523, -0.273386166]],
    [{ t: T.HP_Q, hz: 60, q: 1, db: 0 }, [0.996073049, -1.9921461, 0.996073049, 1.99211538, -0.99217682]],
    [{ t: T.LP_BW12, hz: 16000, q: 0.707, db: 0 }, [0.465153077, 0.930306154, 0.465153077, -0.620204102, -0.240408205]],
  ];
  for (const [f, ref] of REF) {
    const [s] = biquads({ ...f, on: true });
    const got = [s.b0, s.b1, s.b2, -s.a1, -s.a2];
    got.forEach((c, i) => close(c, ref[i], 2e-8, `type ${f.t} @ ${f.hz} Hz, coefficient ${i}`));
  }
});

test("section budget follows the XMOS: off and 0 dB cost nothing, 24 dB/oct cuts cost two", () => {
  assert.equal(sectionsOf({ t: T.PEAK, on: true, hz: 1000, db: 3, q: 1 }), 1);
  assert.equal(sectionsOf({ t: T.PEAK, on: false, hz: 1000, db: 3, q: 1 }), 0);
  assert.equal(sectionsOf({ t: T.PEAK, on: true, hz: 1000, db: 0.005, q: 1 }), 0);
  assert.equal(sectionsOf({ t: T.LOW_SHELF, on: true, hz: 100, db: 0, q: 0.7 }), 0);
  assert.equal(sectionsOf({ t: T.HP_BW12, on: true, hz: 60, db: 0, q: 0.7 }), 1, "a cut has no gain to be flat");
  for (const t of [T.HP_BW24, T.HP_LR24, T.LP_BW24, T.LP_LR24]) assert.equal(sectionsOf({ t, on: true, hz: 100, db: 0, q: 0.7 }), 2);
  assert.equal(profileSections(FPH), 6, "Bass and Treble at 0 dB cost nothing");
  assert.equal(profileSections({ ...FPH, bass: 3, tre: -2 }), 8);
});

test("the FPH net curve peaks at +14.8 dB near 5.7 kHz, so the limiter is in play", () => {
  const a = analyse(FPH);
  close(a.maxGain, 14.8, 0.1, "net maximum");
  assert.equal(a.limiter, true);
  assert.equal(a.overCap, false);
  assert.equal(a.sections, 6);
  const freqs = [5700, 600, 60, 20];
  const curve = profileCurve(FPH, freqs);
  close(curve[0], 14.8, 0.15, "at 5.7 kHz");
  close(curve[1], 10, 0.15, "about the preamp alone at 600 Hz, between the cuts");
  close(curve[2], 10, 0.3, "a Q 1 high-pass is 0 dB at its own frequency (less the 138 Hz cut's skirt)");
  assert.ok(curve[3] < 0, "20 Hz is cut");
});

test("Flat is bypassed: flat, no limiter, no sections", () => {
  const a = analyse(FLAT);
  close(a.maxGain, 0, 1e-9, "net");
  close(a.peakGain, 0, 1e-9, "peak");
  assert.equal(a.limiter, false);
  assert.equal(a.sections, 0);
  // Anything that does run at unity can still drive full-scale music into a -1 dBFS ceiling.
  assert.equal(analyse({ ...FLAT, f: [{ t: T.HP_BW12, on: true, hz: 30, db: 0, q: 0.707 }] }).limiter, true);
});

test("the +18 dB cap: a positive preamp counts after the filters, cuts run before boosts", () => {
  const boost = { ...FLAT, pre: 12, f: [{ t: T.PEAK, on: true, hz: 1000, db: 7, q: 1 }] };
  const a = analyse(boost);
  close(a.peakGain, 19, 0.01, "12 dB preamp on a 7 dB peak");
  assert.equal(a.overCap, true);
  assert.equal(analyse({ ...boost, pre: 10.9 }).overCap, false);
  // Listed boost first, but the XMOS runs the cut first, so the level never rises above the net.
  const reordered = { ...FLAT, pre: 6, f: [{ t: T.PEAK, on: true, hz: 1000, db: 12, q: 1 }, { t: T.PEAK, on: true, hz: 1000, db: -12, q: 1 }] };
  close(analyse(reordered).peakGain, 6, 0.01, "+12 then -12, then the +6 preamp");
});

test("Auto preamp puts the loudest frequency on the limiter's ceiling", () => {
  const pre = autoPreamp(FPH);
  close(pre, -5.9, 1e-9, "10 - 14.84 - 1, rounded down");
  const a = analyse({ ...FPH, pre });
  assert.ok(a.maxGain <= -1 && a.maxGain > -1.1, `net ${a.maxGain}`);
  assert.equal(a.limiter, false);
  assert.equal(autoPreamp({ ...FLAT, f: [{ t: T.PEAK, on: true, hz: 100, db: -30, q: 1 }], pre: 0, lim: -12 }), -12);
  assert.equal(autoPreamp({ ...FLAT, pre: -24, f: [{ t: T.PEAK, on: true, hz: 100, db: 12, q: 1 }, { t: T.PEAK, on: true, hz: 100, db: 12, q: 1 }] }), -24, "never below the range");
});

test("Auto preamp also clears the +18 dB cap between sections, where the net curve hides it", () => {
  // Two bass shelves lift the lows +24 dB before the high-pass takes them away again: the net
  // curve tops out near +0.7 dB, but the level between sections reaches +24.
  const p = {
    ...FLAT,
    f: [
      { t: T.LOW_SHELF, on: true, hz: 200, db: 12, q: 0.707 },
      { t: T.LOW_SHELF, on: true, hz: 150, db: 12, q: 0.707 },
      { t: T.HP_Q, on: true, hz: 1000, db: 0, q: 0.9 },
    ],
  };
  assert.ok(analyse(p).overCap);
  const pre = autoPreamp(p);
  assert.equal(pre, -6, "the net curve alone would say -1.7, still over the cap");
  const a = analyse({ ...p, pre });
  assert.equal(a.overCap, false, `between sections ${a.peakGain}`);
  assert.equal(a.limiter, false);
});

test("the graph's handle sits on the filter's own curve", () => {
  assert.equal(handleDb({ t: T.PEAK, on: true, hz: 1000, db: -4, q: 2 }), -4);
  assert.equal(handleDb({ t: T.LOW_SHELF, on: true, hz: 120, db: 6, q: 0.707 }), 3);
  close(handleDb({ t: T.HP_BW12, on: true, hz: 80, db: 0, q: 0.707 }), -3.01, 0.02, "Butterworth -3 dB point");
  close(handleDb({ t: T.HP_Q, on: false, hz: 60, db: 0, q: 1 }), 0, 0.01, "an off filter still shows where it would sit");
  const curve = filterCurve({ t: T.PEAK, on: true, hz: 1000, db: -4, q: 2 }, [1000, 20, 20000]);
  close(curve[0], -4, 1e-6, "peak centre");
  close(curve[1], 0, 0.01, "far below");
});

test("Music Assistant's export imports as exactly the FPH preset", () => {
  const r = importText(MA_EXPORT);
  assert.deepEqual(r.notes, []);
  assert.equal(r.pre, 10);
  assert.equal(r.bass, 0);
  assert.equal(r.tre, 0);
  assert.ok(sameSound({ ...FPH, ...r }, FPH), JSON.stringify(r.f));
});

test("export writes Music Assistant's own line format, and reads back the same sound", () => {
  const text = exportText(FPH);
  assert.equal(text, MA_EXPORT);
  const toned = { ...FPH, bass: 3, tre: -2.5, f: [...FPH.f, { t: T.LP_LR24, on: false, hz: 16000, db: 0, q: 0.707 }, { t: T.HP_BW24, on: true, hz: 40, db: 0, q: 0.707 }] };
  const back = importText(exportText(toned));
  assert.deepEqual(back.notes, []);
  assert.equal(back.bass, 3);
  assert.equal(back.tre, -2.5);
  assert.ok(sameSound({ ...toned, ...back }, toned), JSON.stringify(back.f));
});

test("REW and Equalizer APO files: their spellings, BW Oct, and what cannot carry over", () => {
  const text = [
    "Filter Settings file",
    "Room EQ V5.31",
    "Preamp: -3.5 dB",
    "Filter  1: ON  PK       Fc   1.2 kHz  Gain  -2.0 dB  Q  1.41",
    "Filter: ON PK Fc 300 Hz Gain 2 dB BW Oct 1.0",
    "Filter  3: ON  LS       Fc   105 Hz  Gain   4.0 dB",
    "Filter  4: OFF HPQ      Fc   35 Hz  Q 0.8",
    "Filter  5: ON  LP       Fc   18000 Hz",
    "Filter  6: ON  NO       Fc   2000 Hz",
    "Filter  7: ON  HP1      Fc   25 Hz",
    "Filter  8: ON  PK       Fc   9 Hz  Gain  20.0 dB  Q  30",
    "Filter  9: ON  None",
  ].join("\r\n");
  const r = importText(text);
  assert.equal(r.pre, -3.5);
  assert.deepEqual(r.f.map(f => f.t), [T.PEAK, T.PEAK, T.LOW_SHELF, T.HP_Q, T.LP_BW12, T.PEAK]);
  close(r.f[0].hz, 1200, 1e-9, "kHz");
  close(r.f[1].q, 1.4142, 0.001, "BW Oct 1.0");
  assert.equal(r.f[3].on, false);
  assert.equal(r.f[3].q, 0.8);
  assert.deepEqual(r.f[5], { t: T.PEAK, on: true, hz: 20, db: 12, q: 10 });
  assert.deepEqual(r.notes, [
    { k: "approx", line: 6, type: "LS" },
    { k: "skip", line: 9, type: "NO" },
    { k: "skip", line: 10, type: "HP1" },
    { k: "clamp", line: 11 },
  ]);
  assert.equal(importText("just some words\nno filters here"), null);
});

test("comma decimals, unreadable lines, and slopes the speaker doesn't have", () => {
  const r = importText([
    "Preamp: -6,5 dB",
    "Filter 1: ON PK Fc 1,2 kHz Gain -2,5 dB Q 1,41",
    "Filter 2: ON PK Fc Hz Gain 2 dB Q 1",
    "Filter 3: ON PK Fc 500 Hz Q 2",
    "Filter 4: ON HP 24dB Fc 40 Hz",
    "Filter 5: ON HP 12dB Fc 30 Hz",
  ].join("\n"));
  assert.equal(r.pre, -6.5);
  close(r.f[0].hz, 1200, 1e-9, "1,2 kHz");
  assert.equal(r.f[0].db, -2.5);
  assert.equal(r.f[0].q, 1.41);
  assert.deepEqual(r.f[1], { t: T.PEAK, on: true, hz: 500, db: 0, q: 2 });
  assert.equal(r.f[2].t, T.HP_BW12, "a 12 dB/oct cut that says so");
  assert.equal(r.f.length, 3);
  assert.deepEqual(r.notes, [
    { k: "unread", line: 3 },
    { k: "nogain", line: 4 },
    { k: "skip", line: 5, type: "HP 24dB" },
  ]);
});

test("a single cut at Butterworth's or Linkwitz-Riley's own Q comes in as that slope", () => {
  const r = importText([
    "Filter 1: ON HP Fc 60 Hz Gain 0 dB Q 0.707",
    "Filter 2: ON LP Fc 15000 Hz Gain 0 dB Q 0.500",
    "Filter 3: ON HP Fc 30 Hz Gain 0 dB Q 0.9",
  ].join("\n"));
  assert.deepEqual(r.f.map(f => f.t), [T.HP_BW12, T.LP_LR12, T.HP_Q]);
  assert.equal(r.f[2].q, 0.9);
  assert.deepEqual(r.notes, []);
  const lr = { ...FLAT, f: [{ t: T.LP_LR12, on: true, hz: 15000, db: 0, q: 0.707 }] };
  assert.ok(sameSound({ ...lr, ...importText(exportText(lr)) }, lr), "Linkwitz-Riley 12 round-trips");
});

test("an import that overflows keeps what fits", () => {
  const many = Array.from({ length: 14 }, (_, i) => `Filter ${i + 1}: ON PK Fc ${100 + i * 100} Hz Gain 1 dB Q 1`).join("\n");
  const r = importText(many);
  assert.equal(r.f.length, 12);
  assert.deepEqual(r.notes, [{ k: "many", kept: 12 }]);
  const cuts = Array.from({ length: 9 }, (_, i) => `Filter ${2 * i + 1}: ON HP Fc ${20 + i} Hz Gain 0 dB Q 0.541\nFilter ${2 * i + 2}: ON HP Fc ${20 + i} Hz Gain 0 dB Q 1.307`).join("\n");
  const c = importText(cuts);
  assert.equal(c.f.length, 8, "eight 24 dB/oct cuts are all 16 sections");
  assert.ok(c.f.every(f => f.t === T.HP_BW24));
  assert.deepEqual(c.notes, [{ k: "budget", kept: 8 }]);
});

test("the device's JSON in, its form body out, always inside its ranges", () => {
  const p = fromWire({ pre: 10, bass: 0, tre: 0, lim: -1, atk: 1, rel: 100, preset: "FPH Large Desk Smart Speaker", f: [[1, 1, 138, -3, 1.93], [4, 1, 60, 0, 1], [99, 1, 1, 1, 1], "junk"] });
  assert.equal(p.f.length, 2, "unknown types and malformed rows are dropped");
  assert.deepEqual(p.f[1], { t: T.HP_Q, on: true, hz: 60, db: 0, q: 1 });
  assert.equal(filtersParam(p.f), "1,1,138,-3,1.93;4,1,60,0,1");
  const wild = clampProfile({ ...p, pre: 40, bass: -20, lim: 3, f: [{ t: T.PEAK, on: true, hz: 5, db: -50, q: 0 }, { t: T.HP_BW12, on: true, hz: 30000, db: 6, q: 4 }] });
  assert.equal(wild.pre, 12);
  assert.equal(wild.bass, -9);
  assert.equal(wild.lim, 0);
  assert.deepEqual(wild.f[0], { t: T.PEAK, on: true, hz: 20, db: -30, q: 0.1 });
  assert.deepEqual(wild.f[1], { t: T.HP_BW12, on: true, hz: 20000, db: 0, q: 0.707 }, "a fixed-shape cut has no gain or Q");
  assert.deepEqual(profileBody(FPH), { pre: "10", bass: "0", tre: "0", lim: "-1", atk: "1", rel: "100", f: "1,1,138,-3,1.93;1,1,1600,-4.5,4;1,1,3455,-8,2.66;1,1,5700,7,3.42;1,1,13700,-10,1;4,1,60,0,1" });
});

test("sameSound ignores the preset's name and what a filter type does not use", () => {
  assert.ok(sameSound(FPH, { ...FPH, preset: "" }));
  assert.ok(!sameSound(FPH, { ...FPH, pre: 9.5 }));
  const cut = { ...FLAT, f: [{ t: T.HP_BW12, on: true, hz: 60, db: 0, q: 0.707 }] };
  assert.ok(sameSound(cut, { ...cut, f: [{ ...cut.f[0], db: 3, q: 2 }] }));
  assert.ok(!sameSound(cut, { ...cut, f: [{ ...cut.f[0], on: false }] }));
});

test("kinds and slopes round-trip every filter type", () => {
  for (const t of Object.values(T).filter(t => t !== T.NONE)) {
    assert.equal(typeOf(kindOf(t), slopeOf(t) ?? undefined), t, `type ${t}`);
  }
  assert.equal(typeOf("lowcut"), T.HP_BW12, "a new cut is Butterworth 12");
});

test("a log slider's step always moves the value, even where neighbouring steps round alike", () => {
  const r2 = v => Math.round(v * 100) / 100;
  // Q runs 0.1 to 10 over 1000 steps; near 0.1 the next ten steps all round to 0.10.
  assert.equal(logTrackValue(1, 0.1, 0, 0.1, 10, r2), 0.11);
  assert.equal(logTrackValue(999, 10, 1000, 0.1, 10, r2), 9.95);
  assert.equal(logTrackValue(1000, 9.99, 999, 0.1, 10, r2), 10, "the end of the track is reachable");
  assert.equal(logTrackValue(0, 0.1, 1, 0.1, 10, r2), 0.1, "nothing lower to go to");
});

test("numbers read the way the page prints them", () => {
  assert.deepEqual([138, 1600, 3455, 13700, 20000].map(fmtHz), ["138 Hz", "1.6 kHz", "3.46 kHz", "13.7 kHz", "20 kHz"]);
  assert.deepEqual([999.4, 999.6].map(fmtHz), ["999 Hz", "1 kHz"], "never \"1000 Hz\"");
  assert.deepEqual([7, -3, 0, -0.04].map(v => fmtDb(v)), ["+7.0 dB", "\u22123.0 dB", "0 dB", "0 dB"]);
});
