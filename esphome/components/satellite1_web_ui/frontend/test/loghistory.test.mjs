/**
 * The device's log history (GET /api/sat1/log) and its join with the live stream's lines.
 */

import assert from "node:assert/strict";
import test from "node:test";

import { mergeLogHistory, parseLogHistory } from "../src/lib/loghistory.js";

const body = (head, ...recs) => [head, ...recs].join("\n") + "\n";
const ids = () => {
  let n = 1000;
  return () => n++;
};
const live = (id, ms, text, at = 50_000 + ms) => ({ id, lvl: text[1], text, at, ms, gen: 0 });

test("a history answer parses into its header and one entry per record", () => {
  const h = parseLogHistory(
    body(
      "#sat1-log boot=0a1b2c3d now=90000 end=12345",
      "1200 [I][app:100]: Running through setup()",
      "1300 [C][wifi:500]: WiFi:\x1f  SSID: 'home'\x1f  Channel: 6",
    ),
  );
  assert.equal(h.boot, "0a1b2c3d");
  assert.equal(h.now, 90000);
  assert.equal(h.end, "12345");
  assert.deepEqual(h.lines, [
    { ms: 1200, text: "[I][app:100]: Running through setup()" },
    { ms: 1300, text: "[C][wifi:500]: WiFi:\n  SSID: 'home'\n  Channel: 6" },
  ]);
});

test("anything that is not a log history parses to null", () => {
  assert.equal(parseLogHistory(""), null);
  assert.equal(parseLogHistory("<!doctype html><title>Sign in</title>"), null);
  assert.equal(parseLogHistory('{"ok":0}'), null);
  assert.equal(parseLogHistory("#sat1-log boot=1 end=5\n"), null);
});

test("the end position survives as the exact text the device sent", () => {
  const h = parseLogHistory("#sat1-log boot=ffffffff now=1 end=9007199254740993\n");
  assert.equal(h.end, "9007199254740993");
  assert.deepEqual(h.lines, []);
});

test("a line held by both the stream and the history is kept once, as the stream's object", () => {
  const streamed = live(1, 5001, "[W][wifi:200]: Signal weak");
  const hist = parseLogHistory(
    body(
      "#sat1-log boot=1 now=6000 end=99",
      "4000 [D][sensor:10]: 'Temp': 21.5",
      "5000 [W][wifi:200]: Signal weak",
    ),
  );
  const out = mergeLogHistory([streamed], hist, { receivedAt: 1_000_000, gen: 0, nextId: ids() });
  assert.equal(out.length, 2);
  assert.equal(out[1], streamed);
  assert.equal(out[0].text, "[D][sensor:10]: 'Temp': 21.5");
});

test("a line said twice keeps both copies, each stream line pairing with one record", () => {
  const text = "[D][button:20]: 'Action' Pressed";
  const hist = parseLogHistory(body("#sat1-log boot=1 now=3000 end=9", `2000 ${text}`, `2002 ${text}`));
  const out = mergeLogHistory([live(1, 2000, text)], hist, { receivedAt: 1, gen: 0, nextId: ids() });
  assert.deepEqual(
    out.map((l) => l.ms),
    [2000, 2002],
  );
});

test("history lines are dated from the device's clock and slot in by uptime", () => {
  const hist = parseLogHistory(
    body("#sat1-log boot=1 now=10000 end=9", "2500 [I][a:1]: first", "7500 [E][b:2]: third"),
  );
  const mid = live(1, 5000, "[W][c:3]: second");
  const out = mergeLogHistory([mid], hist, { receivedAt: 1_000_000, gen: 0, nextId: ids() });
  assert.deepEqual(
    out.map((l) => l.text.slice(-6)),
    [" first", "second", " third"],
  );
  assert.equal(out[0].at, 1_000_000 - 7500);
  assert.equal(out[2].at, 1_000_000 - 2500);
  assert.equal(out[2].lvl, "E");
});

test("a headerless record takes the level of the record before it", () => {
  const hist = parseLogHistory(
    body("#sat1-log boot=1 now=9 end=9", "1 [W][x:1]: start of a table", "2 continued without a header"),
  );
  const out = mergeLogHistory([], hist, { receivedAt: 100, gen: 0, nextId: ids() });
  assert.deepEqual(
    out.map((l) => l.lvl),
    ["W", "W"],
  );
});

test("records at or before the Clear floor stay cleared", () => {
  const hist = parseLogHistory(
    body("#sat1-log boot=1 now=9000 end=9", "3000 [I][a:1]: before", "4000 [I][a:1]: at", "5000 [I][a:1]: after"),
  );
  const out = mergeLogHistory([], hist, { receivedAt: 1, gen: 0, floorMs: 4000, nextId: ids() });
  assert.deepEqual(
    out.map((l) => l.ms),
    [5000],
  );
});

test("order holds across the 49.7-day wrap of the device's millis()", () => {
  const hist = parseLogHistory(
    body("#sat1-log boot=1 now=500 end=9", "4294967000 [I][a:1]: before the wrap", "200 [I][a:1]: after the wrap"),
  );
  const out = mergeLogHistory([], hist, { receivedAt: 1_000_000, gen: 0, nextId: ids() });
  assert.deepEqual(
    out.map((l) => l.ms),
    [4294967000, 200],
  );
  assert.equal(out[0].at, 1_000_000 - 796);
});
