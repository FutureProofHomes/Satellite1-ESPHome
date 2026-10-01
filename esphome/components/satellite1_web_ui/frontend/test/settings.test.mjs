/**
 * The Settings pages' logic. The device's payloads are fixed by firmware already in the field, so
 * these pin the wording and parsing against the exact shapes it sends.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { TEXT } from "../src/copy.js";
import {
  ampMode,
  crashWhen,
  filterLog,
  gainDbv,
  ingressYaml,
  kb,
  levelShows,
  logExport,
  logFileName,
  logParts,
  mb,
  passwordProblem,
  stamp,
  uptime,
  usbFact,
} from "../src/lib/settings.js";

test("sizes and durations read the way the design writes them", () => {
  assert.equal(kb(120832), "118 kB");
  assert.equal(mb(4299161), "4.1 MB");
  assert.equal(uptime(null), "\u2014");
  assert.equal(uptime(65), "1m 5s");
  assert.equal(uptime(7 * 3600 + 22 * 60 + 9), "7h 22m");
  assert.equal(uptime(3 * 86400 + 7 * 3600 + 22 * 60), "3d 7h 22m");
});

test("the USB-C contract is reworded voltage first, and anything else shows raw", () => {
  assert.deepEqual(usbFact("3.25A (max) @ 20V"), { value: "20V @ 3.25A~", sub: "65 watts" });
  assert.deepEqual(usbFact("3A (max) @ 15V"), { value: "15V @ 3A~", sub: "45 watts" });
  assert.deepEqual(usbFact("1.5A (max) @ 9V"), { value: "9V @ 1.5A~", sub: "13.5 watts" });
  assert.deepEqual(usbFact("5V default"), { value: "5V default" });
  assert.equal(usbFact(""), null);
  assert.equal(usbFact(undefined), null);
});

test("the amplifier's mode: measuring and off outrank the stale mode number", () => {
  assert.equal(ampMode(null), null);
  assert.equal(ampMode({ mode: 2, active: 1, pending: 1 }).v, "Measuring\u2026");
  assert.equal(ampMode({ mode: 2, active: 0, pending: 0 }).v, "Off");
  assert.equal(ampMode({ mode: 2, active: 1, pending: 0 }).v, "High gain");
  assert.equal(ampMode({ mode: 0, active: 1, pending: 0 }).v, "Low gain");
  assert.deepEqual(ampMode({ mode: 3, active: 1, pending: 0 }), { v: "PWR_MODE 3", d: null });
  assert.equal(gainDbv(8), "15.0 dBV");
  assert.equal(gainDbv(0), "11.0 dBV");
  assert.equal(gainDbv(20), "21.0 dBV");
});

test("log levels filter from the floor up, and config or unparsed lines always show", () => {
  assert.equal(levelShows("D", "D"), true);
  assert.equal(levelShows("D", "V"), false);
  assert.equal(levelShows("VV", "VV"), true);
  assert.equal(levelShows("W", "I"), false);
  assert.equal(levelShows("W", "E"), true);
  assert.equal(levelShows("E", "C"), true);
  assert.equal(levelShows("E", "?"), true);
  const log = [
    { lvl: "D", text: "[D][sensor:094]: 'Temperature': Sending state", at: 1 },
    { lvl: "W", text: "[W][wifi:123]: Rate limit hit", at: 2 },
    { lvl: "V", text: "[V][api:200]: Connected", at: 3 },
  ];
  assert.deepEqual(
    filterLog(log, "D", "").map((l) => l.at),
    [1, 2],
  );
  assert.deepEqual(
    filterLog(log, "VV", " WIFI ").map((l) => l.at),
    [2],
  );
});

test("a log line splits into its component tag and message", () => {
  assert.deepEqual(logParts("[D][sensor:094]: 'Temperature': Sending state 21.4"), {
    tag: "sensor",
    msg: "'Temperature': Sending state 21.4",
  });
  assert.deepEqual(logParts("[VV][api.service:42]: frame"), { tag: "api.service", msg: "frame" });
  assert.deepEqual(logParts("[C][logger]: Level: DEBUG"), { tag: "logger", msg: "Level: DEBUG" });
  assert.deepEqual(logParts("  Update Interval: 60.0s"), { tag: "", msg: "  Update Interval: 60.0s" });
});

test("the export carries each line's arrival time, and the file name is filesystem-safe", () => {
  const at = new Date(2026, 8, 24, 18, 42, 1, 118).getTime();
  assert.equal(stamp(at), "18:42:01.118");
  assert.equal(logExport([{ at, text: "[I][app:100]: Hello" }]), "[18:42:01.118] [I][app:100]: Hello");
  assert.equal(logFileName(new Date(Date.UTC(2026, 8, 24, 18, 42, 1))), "satellite1-2026-09-24-18-42-01.log");
});

test("an old crash is placed by uptime and restart distance", () => {
  assert.equal(
    crashWhen({ boot: 9, up: 3642, epoch: 0 }, 12, 100),
    TEXT.crash_restarts_ago.replace("%1", "1h 0m").replace("%2", "3"),
  );
  assert.equal(
    crashWhen({ boot: 11, up: 65, epoch: 0 }, 12, 0),
    TEXT.crash_restart_ago.replace("%1", "1m 5s"),
  );
  assert.match(crashWhen({ boot: 11, up: 65, epoch: 0 }, 12, 600, Date.now()), /^\u2248 /);
});

test("the password rules match the firmware's", () => {
  assert.equal(passwordProblem("short", "short"), "pw_len");
  assert.equal(passwordProblem("x".repeat(32), "x".repeat(32)), "pw_len");
  assert.equal(passwordProblem('has"quote', 'has"quote'), "pw_chars");
  assert.equal(passwordProblem("back\\slash", "back\\slash"), "pw_chars");
  assert.equal(passwordProblem(" leading1", " leading1"), "pw_chars");
  assert.equal(passwordProblem("caf\u00e9caf\u00e9", "caf\u00e9caf\u00e9"), "pw_chars");
  assert.equal(passwordProblem("goodpass1", "goodpass2"), "pw_mismatch");
  assert.equal(passwordProblem("good pass1", "good pass1"), null);
});

const SELF = { name: "satellite1-a4c2f8", ip: "192.168.4.31", mac: "74:4D:BD:A4:C2:F8" };
const row = (name, mac, url, ip) => ["Satellite1", name, "Kitchen", mac, "25.9.4", url, 1, "pw", "w", 2450, 0, ip];

test("the ingress YAML puts every routable peer behind this device's panel", () => {
  const yaml = ingressYaml(SELF, [
    row("Office", "74:4d:bd:c8:d4:15", "http://satellite1-c8d415.local", "192.168.4.35"),
    row("Living Room", "74:4d:bd:a4:c2:f8", "http://192.168.4.31", "192.168.4.31"),
    row('Kitchen "Main"', "74:4d:bd:b1:c3:02", "http://192.168.4.32", ""),
    row("Garage", "74:4d:bd:00:00:01", "http://garage.local", ""),
  ]);
  const lines = yaml.split("\n");
  assert.equal(lines[0], "ingress:");
  assert.equal(lines[1], "  satellite1_a4c2f8:");
  assert.ok(lines.includes("    url: http://192.168.4.31"));
  // Sorted by name, this device left out, the IP-less .local row dropped.
  const keys = lines.filter((l) => /^ {2}\S/.test(l));
  assert.deepEqual(keys, ["  satellite1_a4c2f8:", "  satellite1_b1c302:", "  satellite1_c8d415:"]);
  assert.ok(lines.includes('    title: "Kitchen Main"'));
  assert.ok(lines.includes("    url: http://192.168.4.32"));
  assert.ok(lines.includes("      host: 192.168.4.35"));
  assert.equal(lines.filter((l) => l.startsWith("    parent: satellite1_a4c2f8")).length, 2);
});

test("a device renamed off the mac-suffix convention gets its own entry only", () => {
  const yaml = ingressYaml({ ...SELF, name: "kitchen-speaker" }, [row("Office", "74:4d:bd:c8:d4:15", "", "192.168.4.35")]);
  assert.equal(yaml.split("\n").filter((l) => /^ {2}\S/.test(l)).length, 1);
  assert.ok(yaml.includes("  kitchen_speaker:"));
});
