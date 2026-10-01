/**
 * The setup wizard's step logic. Its writes only run on an un-onboarded device over its own access
 * point, so these decisions are the part of the flow that can be checked off the hardware.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  JOIN_SLOW_MS,
  afterAdd,
  entryStep,
  inCaptiveSheet,
  joinProblem,
  joinRead,
  mergeScan,
  onHomeOrigin,
  probeOrigins,
  probeStreak,
} from "../v2/lib/setup-flow.js";

const UA = {
  iosSheet: "Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Mobile/15E148",
  iosSafari:
    "Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.5 Mobile/15E148 Safari/604.1",
  iosChrome:
    "Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/126.0 Mobile/15E148 Safari/604.1",
  androidSheet:
    "Mozilla/5.0 (Linux; Android 14; Pixel 8; wv) AppleWebKit/537.36 (KHTML, like Gecko) Version/4.0 Chrome/126.0 Mobile Safari/537.36",
  androidChrome: "Mozilla/5.0 (Linux; Android 14; Pixel 8) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0 Mobile Safari/537.36",
  firefox: "Mozilla/5.0 (X11; Linux x86_64; rv:128.0) Gecko/20100101 Firefox/128.0",
};

test("captive sheets are told apart from real browsers", () => {
  assert.equal(inCaptiveSheet(UA.iosSheet), true);
  assert.equal(inCaptiveSheet(UA.androidSheet), true);
  assert.equal(inCaptiveSheet("Mozilla/5.0 CaptivePortalLogin/1.0"), true);
  for (const ua of [UA.iosSafari, UA.iosChrome, UA.androidChrome, UA.firefox]) assert.equal(inCaptiveSheet(ua), false, ua);
});

test("a connected station skips the WiFi steps wherever the page loads", () => {
  for (const [search, ua] of [["", UA.iosSheet], ["?setup=prime", UA.iosSheet], ["", UA.firefox]]) {
    assert.deepEqual(entryStep({ connected: 1 }, search, ua), { step: "haconnect", launched: false });
  }
});

test("only the captive sheet gets the launcher, and its own links route past it", () => {
  assert.deepEqual(entryStep({ connected: 0 }, "", UA.iosSheet), { step: "launcher", launched: false });
  assert.deepEqual(entryStep({ connected: 0 }, "?setup=prime", UA.iosSheet), { step: "launcher", launched: true });
  assert.deepEqual(entryStep({ connected: 0 }, "?setup=go", UA.iosSheet), { step: "network", launched: false });
  assert.deepEqual(entryStep({ connected: 0 }, "", UA.iosSafari), { step: "network", launched: false });
});

test("a device that does not answer is treated as not connected", () => {
  assert.deepEqual(entryStep(null, "", UA.androidChrome), { step: "network", launched: false });
  assert.deepEqual(entryStep(null, "", UA.androidSheet), { step: "launcher", launched: false });
});

test("an empty scan read keeps the list on screen, but a first empty read is shown", () => {
  const list = [{ ssid: "Home", rssi: -50, sec: 1 }];
  assert.deepEqual(mergeScan(null, []), []);
  assert.equal(mergeScan(list, []), list);
  assert.equal(mergeScan(list, null), list);
  assert.equal(mergeScan(null, null), null);
  const next = [{ ssid: "Other", rssi: -60, sec: 0 }];
  assert.equal(mergeScan(list, next), next);
});

test("a join needs a name and, when a key is given, at least eight characters", () => {
  assert.equal(joinProblem("", "password", true), "ssid");
  assert.equal(joinProblem("   ", "", false), "ssid");
  assert.equal(joinProblem("Home", "short", true), "short");
  assert.equal(joinProblem("Home", "12345678", true), null);
  assert.equal(joinProblem("Home", "", false), null);
  assert.equal(joinProblem("Hidden open network", "", true), null);
  assert.equal(joinProblem("Open", "abc", false), null);
});

test("the joining poll arms on a connect or a vanished device, and calls a long wait slow", () => {
  assert.equal(joinRead(null, 0), "gone");
  assert.equal(joinRead({ connected: 1 }, JOIN_SLOW_MS + 1), "connected");
  assert.equal(joinRead({ connected: 0 }, 1000), "trying");
  assert.equal(joinRead({ connected: 0 }, JOIN_SLOW_MS), "trying");
  assert.equal(joinRead({ connected: 0 }, JOIN_SLOW_MS + 1), "slow");
});

test("the redirect probes the .local name and the station IP once known", () => {
  assert.deepEqual(probeOrigins("satellite1-a4c2f8", null), ["http://satellite1-a4c2f8.local"]);
  assert.deepEqual(probeOrigins("satellite1-a4c2f8", "192.168.1.40"), ["http://satellite1-a4c2f8.local", "http://192.168.1.40"]);
  assert.deepEqual(probeOrigins("", "192.168.1.40"), ["http://192.168.1.40"]);
  assert.deepEqual(probeOrigins("", null), []);
});

test("the home origin is recognised case-insensitively", () => {
  assert.equal(onHomeOrigin("Satellite1-A4C2F8.local", "satellite1-a4c2f8"), true);
  assert.equal(onHomeOrigin("192.168.4.1", "satellite1-a4c2f8"), false);
  assert.equal(onHomeOrigin("satellite1-a4c2f8.local", ""), false);
});

test("the redirect waits for two answers in a row from the same origin", () => {
  const hit = probeStreak();
  const a = "http://satellite1.local";
  const b = "http://192.168.1.40";
  assert.equal(hit(a, true), false);
  assert.equal(hit(b, true), false);
  assert.equal(hit(a, false), false);
  assert.equal(hit(a, true), false);
  assert.equal(hit(b, true), true);
  assert.equal(hit(a, true), true);
});

test("onboarding done hands over when actions are settled, otherwise the actions step comes first", () => {
  assert.equal(afterAdd(null), null);
  assert.equal(afterAdd({ setup: 1, actions: 1 }), null);
  assert.equal(afterAdd({ setup: 0, actions: 1 }), "done");
  assert.equal(afterAdd({ setup: 0, actions: 3 }), "done");
  assert.equal(afterAdd({ setup: 0, actions: 2 }), "haactions");
  assert.equal(afterAdd({ setup: 0, actions: 0 }), "haactions");
});
