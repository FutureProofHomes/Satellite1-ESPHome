/**
 * The hash routes. Bookmarks outlive firmware (the hash never reaches the server), so the names
 * from before the redesign have to keep landing on the page they meant.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { parseRoute, routeDir, routeHash } from "../src/lib/routes.js";

test("each tab parses from its own hash", () => {
  assert.deepEqual(parseRoute("#/home"), { tab: "NOW", sub: null });
  assert.deepEqual(parseRoute("#/wake-word"), { tab: "WAKE", sub: null });
  assert.deepEqual(parseRoute("#/presence"), { tab: "PRESENCE", sub: null });
  assert.deepEqual(parseRoute("#/audio/routing"), { tab: "AUDIO", sub: "routing" });
  assert.deepEqual(parseRoute("#/settings/logs"), { tab: "SETTINGS", sub: "logs" });
});

test("audio has a routing page and a DSP page, routing first", () => {
  assert.deepEqual(parseRoute("#/audio/dsp"), { tab: "AUDIO", sub: "dsp" });
  // The Audio page's old address shows Routing, as it always did, under its new name.
  assert.deepEqual(parseRoute("#/audio"), { tab: "AUDIO", sub: "routing" });
  assert.deepEqual(parseRoute("#/audio/nope"), { tab: "AUDIO", sub: "routing" });
  assert.equal(routeHash("AUDIO"), "#/audio/routing");
  assert.equal(routeHash("AUDIO", "dsp"), "#/audio/dsp");
  assert.equal(routeHash("AUDIO", "device-info"), "#/audio/routing");
});

test("the device page's URL name differs from the design's slug", () => {
  assert.deepEqual(parseRoute("#/settings/device"), { tab: "SETTINGS", sub: "device-info" });
  assert.equal(routeHash("SETTINGS", "device-info"), "#/settings/device");
});

test("the developer page has its own hash", () => {
  assert.deepEqual(parseRoute("#/settings/developer"), { tab: "SETTINGS", sub: "developer" });
  assert.equal(routeHash("SETTINGS", "developer"), "#/settings/developer");
});

test("pre-redesign hashes land on their successors", () => {
  assert.deepEqual(parseRoute("#/controls"), { tab: "NOW", sub: null });
  assert.deepEqual(parseRoute("#/config"), { tab: "AUDIO", sub: "routing" });
  assert.deepEqual(parseRoute("#/diagnostics"), { tab: "SETTINGS", sub: "device-info" });
  // The amplifier card moved to Developer; a release build then sends this on to Device Info.
  assert.deepEqual(parseRoute("#/settings/amp"), { tab: "SETTINGS", sub: "developer" });
  assert.equal(routeHash("SETTINGS", "audio"), "#/settings/device");
});

test("unknown, empty and decorated hashes fall back sensibly", () => {
  for (const h of ["", "#", "#/", "#/nope", "#/home-assistant-connect", undefined]) {
    assert.deepEqual(parseRoute(h), { tab: "NOW", sub: null }, String(h));
  }
  assert.deepEqual(parseRoute("#/settings"), { tab: "SETTINGS", sub: "device-info" });
  assert.deepEqual(parseRoute("#/settings/nope"), { tab: "SETTINGS", sub: "device-info" });
  assert.deepEqual(parseRoute("#/Presence/"), { tab: "PRESENCE", sub: null });
  assert.deepEqual(parseRoute("#/settings/logs?level=E"), { tab: "SETTINGS", sub: "logs" });
  assert.deepEqual(parseRoute("#settings/updates"), { tab: "SETTINGS", sub: "updates" });
});

test("every route survives a round trip", () => {
  for (const h of ["#/home", "#/wake-word", "#/presence", "#/audio/routing", "#/audio/dsp"]) {
    const { tab, sub } = parseRoute(h);
    assert.equal(routeHash(tab, sub), h);
  }
  for (const p of ["device", "updates", "security", "logs", "integrations", "recovery", "developer", "community"]) {
    const h = `#/settings/${p}`;
    const { tab, sub } = parseRoute(h);
    assert.equal(routeHash(tab, sub), h);
  }
});

test("the page slides the way the nav reads", () => {
  const r = (h) => parseRoute(h);
  assert.equal(routeDir(r("#/home"), r("#/audio")), "fwd");
  assert.equal(routeDir(r("#/audio"), r("#/wake-word")), "back");
  assert.equal(routeDir(r("#/presence"), r("#/settings/logs")), "fwd");
  assert.equal(routeDir(r("#/settings/logs"), r("#/settings/updates")), "back");
  assert.equal(routeDir(r("#/settings/updates"), r("#/settings/community")), "fwd");
  assert.equal(routeDir(r("#/settings"), r("#/settings/device")), null);
  assert.equal(routeDir(r("#/controls"), r("#/home")), null);
  assert.equal(routeDir(r("#/audio/routing"), r("#/audio/dsp")), "fwd");
  assert.equal(routeDir(r("#/audio/dsp"), r("#/presence")), "back");
  assert.equal(routeDir(r("#/audio/dsp"), r("#/settings/device")), "fwd");
  assert.equal(routeDir(r("#/audio"), r("#/audio/routing")), null);
});
