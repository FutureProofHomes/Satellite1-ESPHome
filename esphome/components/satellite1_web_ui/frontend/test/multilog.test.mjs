/**
 * Logs from several devices on one timeline (src/lib/multilog.js): which devices are offered, how
 * each device's uptime stamps become wall-clock time, and the merged order.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { formatMerged, logDevices, mergeLines, pinLines } from "../src/lib/multilog.js";

const row = (model, name, mac, up, url, pw) => [model, name, "Kitchen", mac, null, url, up, pw, null, null, null, "192.168.0.50"];

test("this device comes first, then the other Satellite1s in Home Assistant's roster", () => {
  const ha = {
    d: {
      dev: [
        row("Satellite1", "Kitchen", "AA:BB", 1, "http://kitchen.local", "pw1"),
        row("Satellite1", "Self", "cc:dd", 1, "http://self.local", "pw2"),
        row("Voice PE", "Office", "ee:ff", 1, "http://office.local", "pw3"),
        row("Satellite1", "Garage", "11:22", 0, "http://garage.local", ""),
      ],
    },
  };
  const got = logDevices(ha, { mac: "CC:DD", name: "Self" });
  assert.deepEqual(got.map((d) => [d.id, d.name, d.self, d.up]), [
    ["cc:dd", "Self", true, true],
    ["aa:bb", "Kitchen", false, true],
    ["11:22", "Garage", false, false],
  ]);
  assert.equal(got[1].password, "pw1");
});

test("with no roster only this device is offered", () => {
  assert.deepEqual(logDevices(null, { mac: "cc:dd", name: "Self" }).map((d) => d.id), ["cc:dd"]);
  assert.deepEqual(logDevices(undefined, undefined).map((d) => d.id), ["self"]);
});

test("lines are pinned to the midpoint of the round trip, with ANSI stripped and levels carried", () => {
  const hist = {
    now: 100000,
    lines: [
      { ms: 99000, text: "\u001b[0;32m[I][wifi:123]: connected\u001b[0m" },
      { ms: 99500, text: "  continuation" },
      { ms: 100000, text: "[W][xmos:9]: not ready" },
    ],
  };
  const { rtt, lines } = pinLines(hist, 1_000_000, 1_000_040);
  assert.equal(rtt, 40);
  assert.deepEqual(lines.map((l) => [l.wall, l.lvl, l.text]), [
    [1_000_020 - 1000, "I", "[I][wifi:123]: connected"],
    [1_000_020 - 500, "I", "  continuation"],
    [1_000_020, "W", "[W][xmos:9]: not ready"],
  ]);
});

test("uptime stamps from before millis() wrapped still land in the past", () => {
  const hist = { now: 500, lines: [{ ms: 0xffffffff - 499, text: "[D][x:1]: before the wrap" }] };
  const { lines } = pinLines(hist, 10_000, 10_000);
  assert.equal(lines[0].wall, 10_000 - 1000);
});

test("the merge orders by wall clock; a tie keeps device order, then line order", () => {
  const results = [
    { name: "A", lines: [{ wall: 10, text: "a1" }, { wall: 30, text: "a2" }] },
    { name: "Down", error: "offline" },
    { name: "B", lines: [{ wall: 10, text: "b1" }, { wall: 20, text: "b2" }, { wall: 20, text: "b3" }] },
  ];
  assert.deepEqual(mergeLines(results).map((l) => `${l.device}:${l.text}`), ["A:a1", "B:b1", "B:b2", "B:b3", "A:a2"]);
});

test("the download's header gives every device's error bar, or why it is missing", () => {
  const results = [
    { name: "Kitchen", lines: [{ wall: 0, text: "x" }], rtt: 41, boot: "abcd", fw: "2.1.0" },
    { name: "Garage", error: "can't sign in" },
  ];
  const text = formatMerged(results, mergeLines(results), 0);
  assert.match(text, /# Kitchen: 1 lines, firmware 2\.1\.0, boot abcd, round trip 41 ms \(timestamps \+\/- 21 ms\)/);
  assert.match(text, /# Garage: can't sign in/);
  assert.match(text, /Kitchen x\n$/);
});
