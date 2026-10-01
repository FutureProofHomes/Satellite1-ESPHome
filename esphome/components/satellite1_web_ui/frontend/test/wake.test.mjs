/**
 * The Wake Word tab's logic: what the graphs draw, where the tuner seeds its knob, how a swap and
 * its Home Assistant pairing settle, what the picker lists, and the per-slot Finished Speaking
 * Detection rule.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  DAY_MS,
  attemptsOf,
  cutoffPath,
  filterEntries,
  fsdShown,
  fsdWrites,
  langLabel,
  pairingWrite,
  parseSource,
  pickerEntries,
  placement,
  readout,
  rowMarks,
  swapStep,
} from "../src/lib/wake.js";

test("a track's graph keeps the last day, keyed on stable ids", () => {
  const track = {
    dh: [
      [DAY_MS + 1, 200, 0, 1],
      [60000, 0, 0, 2],
      [30000, 230, 0, 3],
      [2000, 150, 1, 4],
      [1000, 204, 0, 5],
    ],
  };
  const marks = rowMarks(track);
  assert.deepEqual(
    marks.map((m) => [m.id, m.kind, m.c]),
    [
      ["d3", "fire", 90],
      ["d4", "near", 59],
      ["d5", "fire", 80],
    ],
  );
  assert.equal(marks.find((m) => m.id === "d5").ripple, true);
  assert.equal(marks.filter((m) => m.ripple).length, 1);
  const again = rowMarks({ dh: [[500, 210, 0, 9], ...track.dh] });
  assert.equal(again.find((m) => m.id === "d3").y, marks.find((m) => m.id === "d3").y);
});

test("an old firing does not ripple", () => {
  assert.equal(rowMarks({ dh: [[5000, 200, 0, 1]] })[0].ripple, undefined);
  assert.deepEqual(rowMarks(null), []);
});

test("tune events split into scored attempts and voice-gate refusals", () => {
  const { attempts, vadTries } = attemptsOf([
    [220, 200, 0, 9000, 0],
    [0, 0, 1, 8000, 0],
    [210, 190, 0, 7000, 0],
    [180, 170, 0, 1000, 0],
  ]);
  assert.deepEqual(attempts, [
    { score: 220, round: "near" },
    { score: 210, round: "near" },
    { score: 180, round: "far" },
  ]);
  assert.equal(vadTries, 1);
});

test("placement seeds the knob inside the gap", () => {
  const p = placement([{ score: 220 }, { score: 200 }, { score: 190 }], [90, 120, 0], 130);
  assert.equal(p.nogap, undefined);
  assert.equal(p.floorV, 190);
  assert.equal(p.hiV, 220);
  assert.equal(p.noise, 130);
  assert.equal(p.cutC, 65);
});

test("no usable gap is diagnosed by side", () => {
  assert.equal(placement([{ score: 120 }], [150], 0).nogap, "room");
  assert.equal(placement([{ score: 120 }], [60], 0).nogap, "voice");
});

test("the readout warns near the quietest try and errs at the room's reach", () => {
  assert.deepEqual(readout(74, 190, [], 0), { tone: "warn", c: 74, floorC: 75 });
  assert.equal(readout(40, 190, [110], 0).tone, "err");
  assert.equal(readout(40, 190, [], 110).tone, "err");
  assert.deepEqual(readout(60, 190, [90], 0), { tone: "dim", c: 60, floorC: 75 });
  assert.equal(readout(60, 0, [], 0).tone, "dim");
});

test("a cutoff write is quantized and carries only the known stats", () => {
  assert.equal(cutoffPath(0, 64, 130, 190, 220), "/api/sat1/wakewords/cutoff?i=0&v=163&n=130&f=190&h=220");
  assert.equal(cutoffPath(2, 99, 0, 0, 0), "/api/sat1/wakewords/cutoff?i=2&v=250");
  assert.equal(cutoffPath(1, 30, 0, 0, 0), "/api/sat1/wakewords/cutoff?i=1&v=100");
});

test("a swap settles on the device's own report", () => {
  const url = "https://example.com/w.json";
  assert.equal(swapStep(undefined, url), null);
  assert.deepEqual(swapStep({ m: url, st: 0 }, url), { done: true });
  assert.deepEqual(swapStep({ m: "", st: 0 }, "none"), { done: true });
  assert.deepEqual(swapStep({ m: url, st: 1, dl: 2048, tot: 8192 }, url), { dl: 2048, tot: 8192 });
  assert.deepEqual(swapStep({ m: url, st: 2, err: 7 }, url), { err: 7 });
  assert.deepEqual(swapStep({ m: "okay_nabu", st: 2, err: 1 }, url), { err: 1 });
  assert.equal(swapStep({ m: "okay_nabu", st: 2, err: 0 }, url), null);
  assert.equal(swapStep({ m: "okay_nabu", st: 0 }, url), null);
});

test("the pairing write moves the word into the right select, then stops", () => {
  const sel = [
    ["select.ww1", "Okay Nabu", "select.p1", "preferred"],
    ["select.ww2", "no_wake_word", "select.p2", "preferred"],
  ];
  assert.deepEqual(pairingWrite(sel, 0, "Okay Nabu", "Hey Jarvis"), ["select.ww1", "Hey Jarvis"]);
  assert.deepEqual(pairingWrite(sel, 1, "", "Hey Jarvis"), ["select.ww2", "Hey Jarvis"]);
  assert.equal(pairingWrite(sel, 0, "Hey Jarvis", "okay nabu"), null);
  assert.deepEqual(pairingWrite(sel, 0, "okay nabu", ""), ["select.ww1", "no_wake_word"]);
  assert.equal(pairingWrite(sel, 1, "Hey Jarvis", ""), null);
  const full = [
    ["select.ww1", "Alexa"],
    ["select.ww2", "Computer"],
  ];
  assert.deepEqual(pairingWrite(full, 1, "Hey Mycroft", "Hey Jarvis"), ["select.ww2", "Hey Jarvis"]);
});

test("a pasted source is a GitHub repository or a manifest link", () => {
  assert.deepEqual(parseSource(" https://github.com/esphome/micro-wake-word-models/ "), {
    url: "https://github.com/esphome/micro-wake-word-models",
    label: "esphome/micro-wake-word-models",
  });
  assert.deepEqual(parseSource("https://example.com/models/hey_casa.json"), {
    url: "https://example.com/models/hey_casa.json",
    label: "hey_casa.json",
  });
  assert.equal(parseSource("github.com/owner/repo"), null);
  assert.equal(parseSource("https://gitlab.com/owner/repo"), null);
  assert.equal(parseSource(""), null);
});

test("the picker lists built-ins first, then each source's words", () => {
  const sources = [
    { url: "https://github.com/a/b", label: "A" },
    { url: "https://github.com/c/d", label: "C" },
  ];
  const cat = {
    "https://github.com/a/b": {
      entries: [
        { word: "Computer", url: "u1", langs: ["en"], dup: true, ver: "v2" },
        { word: "Hola Casa", url: "u2", langs: ["es"], unverified: true },
      ],
    },
    "https://github.com/c/d": { loading: true },
  };
  const list = pickerEntries([["okay_nabu", "Okay Nabu"]], "Built-In", sources, cat);
  assert.deepEqual(
    list.map((e) => [e.word, e.spec, e.source, e.ver, !!e.unverified]),
    [
      ["Okay Nabu", "okay_nabu", "Built-In", undefined, false],
      ["Computer", "u1", "A", "v2", false],
      ["Hola Casa", "u2", "A", "", true],
    ],
  );
  assert.equal(new Set(list.map((e) => e.key)).size, list.length);
});

test("the picker filters, puts the current word first, and caps a search", () => {
  const many = Array.from({ length: 70 }, (_, k) => ({ word: `Hey ${k}`, spec: `s${k}`, langs: k % 2 ? ["en"] : ["de"] }));
  const all = filterEntries(many, "", "", "s5");
  assert.equal(all.shown.length, 70);
  assert.equal(all.shown[0].spec, "s5");
  const search = filterEntries(many, " hey ", "", "");
  assert.equal(search.shown.length, 60);
  assert.equal(search.hidden, 10);
  assert.ok(filterEntries(many, "", "de", "").shown.every((e) => e.langs.includes("de")));
  assert.deepEqual(filterEntries(many, "hey 1", "en", "").shown.map((e) => e.spec), ["s1", "s11", "s13", "s15", "s17", "s19"]);
});

test("languages read as names", () => {
  assert.equal(langLabel("en"), "English");
  assert.equal(langLabel("not a tag"), "not a tag");
});

test("an unset slot shows Home Assistant's value; the picker waits for every value", () => {
  assert.deepEqual(fsdShown(["unset", "unset"], "default"), ["default", "default"]);
  assert.deepEqual(fsdShown(["relaxed", "unset"], "aggressive"), ["relaxed", "aggressive"]);
  assert.equal(fsdShown(["relaxed", undefined], "default"), null);
  assert.equal(fsdShown(["relaxed", "default"], null), null);
});

test("the first pick writes both slots, every later pick one", () => {
  assert.deepEqual(fsdWrites(["unset", "unset"], "default", 0, "relaxed"), [
    [0, "relaxed"],
    [1, "default"],
  ]);
  assert.deepEqual(fsdWrites(["unset", "unset"], "aggressive", 1, "default"), [
    [0, "aggressive"],
    [1, "default"],
  ]);
  assert.deepEqual(fsdWrites(["relaxed", "unset"], "default", 1, "aggressive"), [[1, "aggressive"]]);
  assert.deepEqual(fsdWrites(["relaxed", "default"], "default", 0, "aggressive"), [[0, "aggressive"]]);
  assert.deepEqual(fsdWrites(["unset", "unset"], "unknown", 0, "relaxed"), [[0, "relaxed"]]);
});
