/**
 * The Home tab's pure logic: the orb's state from the assistant's phase, the sensor readings and
 * their calibration steppers, the per-word transcript tabs, timers and the LED ring's colours.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  clock,
  hexToRgb,
  hsvToRgb,
  isOn,
  offsetSpec,
  offsetText,
  orbState,
  orbView,
  pctTo255,
  reading,
  rgbHue,
  ringPct,
  stepOffset,
  timerLabel,
  timerLeft,
  transcriptTabs,
} from "../v2/lib/orb.js";

test("each voice assistant phase maps to its orb state", () => {
  assert.equal(orbState(1, true), "idle");
  assert.equal(orbState(2, true), "listening");
  assert.equal(orbState(3, true), "listening");
  assert.equal(orbState(4, true), "thinking");
  assert.equal(orbState(5, true), "speaking");
  assert.equal(orbState(10, true), "connecting");
  assert.equal(orbState(11, true), "error");
});

test("unknown and missing phases are idle; a lost stream is disabled whatever the phase", () => {
  for (const p of [0, 6, 99, undefined, null, "4"]) {
    assert.equal(orbState(p, true), p === "4" ? "thinking" : "idle", String(p));
  }
  for (const p of [1, 4, 11, undefined]) assert.equal(orbState(p, false), "disabled");
});

test("muted mics wear the paused idle look, except when the stream is gone", () => {
  assert.deepEqual(orbView(4, true, true), { state: "idle", label: "Mic muted", muted: true });
  assert.deepEqual(orbView(4, false, true), { state: "disabled", label: "Offline", muted: false });
  assert.deepEqual(orbView(5, true, false), { state: "speaking", label: "Speaking\u2026", muted: false });
  assert.deepEqual(orbView(10, true, false), { state: "connecting", label: "Not ready", muted: false });
});

test("a switch reads on from either field /events uses", () => {
  assert.equal(isOn({ value: true }), true);
  assert.equal(isOn({ state: "ON" }), true);
  assert.equal(isOn({ value: false, state: "OFF" }), false);
  assert.equal(isOn(undefined), false);
});

test("readings format in the shown unit, and a sensor without one is a dash", () => {
  assert.equal(reading("21.44", 1, "\u00B0C"), "21.4\u00B0C");
  assert.equal(reading(21.44, 0, "\u00B0C"), "21\u00B0C");
  assert.equal(reading(20, 1, "\u00B0C", true), "68.0\u00B0F");
  assert.equal(reading(46.4, 0, "%"), "46%");
  assert.equal(reading(182, 0, " lx"), "182 lx");
  for (const v of [null, undefined, "", "NaN", NaN]) assert.equal(reading(v, 1, "\u00B0C"), "\u2014", String(v));
});

test("an offset is a delta: °F scales it without adding 32, and zero carries no sign", () => {
  assert.equal(offsetText(0.5, 1, "\u00B0"), "+0.5\u00B0");
  assert.equal(offsetText(-1.2, 1, "\u00B0"), "-1.2\u00B0");
  assert.equal(offsetText(1, 1, "\u00B0", true), "+1.8\u00B0");
  assert.equal(offsetText(0, 1, "\u00B0"), "0.0\u00B0");
  assert.equal(offsetText(-0.04, 1, "\u00B0"), "0.0\u00B0");
  assert.equal(offsetText(10, 0, " lx"), "+10 lx");
});

test("the stepper uses the entity's own step and range, never finer than the display step", () => {
  const temp = { step: 0.1, min: -20, max: 20 };
  assert.deepEqual(offsetSpec({ step: "0.1", min_value: "-20", max_value: "20" }, { step: 0.1, min: -5, max: 5 }), temp);
  assert.deepEqual(offsetSpec({ step: "0.5", min_value: "-10", max_value: "10" }, { step: 0.1, min: -20, max: 20 }), { step: 0.5, min: -10, max: 10 });
  assert.deepEqual(offsetSpec({ step: "0.1", min_value: "-50", max_value: "50" }, { step: 1, min: -50, max: 50 }), { step: 1, min: -50, max: 50 });
  assert.deepEqual(offsetSpec({ value: "0" }, temp), temp);
  assert.deepEqual(offsetSpec(undefined, temp), temp);
});

test("stepping rounds to the step's precision and stops at the range ends", () => {
  assert.equal(stepOffset(0.2, 1, 0.1, -20, 20), 0.3);
  assert.equal(stepOffset(-0.1, 1, 0.1, -20, 20), 0);
  assert.equal(stepOffset(19.95, 1, 0.1, -20, 20), 20);
  assert.equal(stepOffset(-20, -1, 0.1, -20, 20), -20);
  assert.equal(stepOffset(495, 1, 5, -500, 500), 500);
  assert.equal(stepOffset(3, -1, 1, -50, 50), 2);
});

test("transcript tabs are the distinct wake words, newest first, and follow the pick", () => {
  const lines = [
    { heard: true, w: "hey jarvis", text: "a" },
    { heard: false, w: "hey jarvis", text: "b" },
    { heard: true, w: "okay nabu", text: "c" },
    { heard: false, w: "okay nabu", text: "d" },
  ];
  const fresh = transcriptTabs(lines, null);
  assert.deepEqual(fresh.words, ["okay nabu", "hey jarvis"]);
  assert.equal(fresh.tab, "okay nabu");
  assert.deepEqual(fresh.shown.map((l) => l.text), ["c", "d"]);
  const picked = transcriptTabs(lines, "hey jarvis");
  assert.equal(picked.tab, "hey jarvis");
  assert.deepEqual(picked.shown.map((l) => l.text), ["a", "b"]);
  assert.equal(transcriptTabs(lines, "stop").tab, "okay nabu");
});

test("untagged lines show under every tab, and one word means no tabs", () => {
  const mixed = [{ text: "old", w: "" }, { text: "x", w: "hey jarvis" }, { text: "y", w: "stop" }];
  assert.deepEqual(transcriptTabs(mixed, "hey jarvis").shown.map((l) => l.text), ["old", "x"]);
  const single = [{ text: "x", w: "hey jarvis" }, { text: "old" }];
  const one = transcriptTabs(single, null);
  assert.deepEqual(one.words, ["hey jarvis"]);
  assert.equal(one.shown, single);
  assert.deepEqual(transcriptTabs([], null), { words: [], tab: null, shown: [] });
});

test("timers count down between polls, hold while paused, and never go negative", () => {
  const t = { left: 90, active: true };
  assert.equal(timerLeft(t, 1000, 1000), 90);
  assert.equal(timerLeft(t, 1000, 2999), 89);
  assert.equal(timerLeft(t, 1000, 200000), 0);
  assert.equal(timerLeft({ left: 90, active: false }, 1000, 60000), 90);
  assert.equal(clock(581), "09:41");
  assert.equal(clock(0), "00:00");
  assert.equal(clock(3900), "1:05:00");
  assert.equal(clock(-3), "00:00");
});

test("an unnamed timer is labelled by its set duration", () => {
  assert.equal(timerLabel({ name: "Pizza", total: 600 }), "Pizza");
  assert.equal(timerLabel({ name: "", total: 600 }), "10 min timer");
  assert.equal(timerLabel({ name: "", total: 5400 }), "1 h 30 min timer");
  assert.equal(timerLabel({ name: "", total: 45 }), "1 min timer");
  assert.equal(timerLabel({ name: "", total: 20 }), "20 s timer");
});

test("ring colours convert between hex, hue and RGB", () => {
  assert.deepEqual(hexToRgb("#a78bfa"), [167, 139, 250]);
  assert.deepEqual(hexToRgb("#fff"), [255, 255, 255]);
  assert.deepEqual(hsvToRgb(0, 1), [255, 0, 0]);
  assert.deepEqual(hsvToRgb(120, 1), [0, 255, 0]);
  assert.deepEqual(hsvToRgb(240, 0), [255, 255, 255]);
  for (const h of [0, 45, 120, 200, 290, 359]) assert.equal(rgbHue(...hsvToRgb(h, 1)), h, String(h));
  assert.equal(rgbHue(128, 128, 128), 0);
});

test("ring brightness is a percent, and an off ring reads zero", () => {
  assert.equal(ringPct({ state: "ON", brightness: 255 }), 100);
  assert.equal(ringPct({ state: "ON", brightness: 168 }), 66);
  assert.equal(ringPct({ state: "ON" }), 100);
  assert.equal(ringPct({ state: "OFF", brightness: 255 }), 0);
  assert.equal(ringPct(undefined), 0);
  assert.equal(pctTo255(100), 255);
  assert.equal(pctTo255(66), 168);
});
