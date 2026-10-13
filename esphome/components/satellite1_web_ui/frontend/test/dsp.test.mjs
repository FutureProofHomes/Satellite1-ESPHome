/**
 * When the Audio tab has its DSP page (src/lib/dsp.js): only on XMOS firmware with the speaker EQ,
 * decided from the state payload plus one re-check per XMOS connect, never by polling.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { hasDevTools } from "../src/lib/devtools.js";
import { dspRecheckDue, dspRedirect, dspSupport, dspVisible, isXmosVersion, navSubs } from "../src/lib/dsp.js";

const AUDIO = [{ slug: "routing" }, { slug: "dsp" }];
const SETTINGS = [{ slug: "device-info" }, { slug: "security" }, { slug: "developer" }];
/** Satellite1Now's own filter: Developer only where the build carries a developer tool. */
const settingsFor = device => SETTINGS.filter(r => r.slug !== "developer" || hasDevTools(device));

test("the page shows only for a definite yes", () => {
  assert.equal(dspVisible(dspSupport({ dsp: 1 })), true);
  for (const device of [null, {}, { dsp: 0 }, { dsp: 2 }, { dsp: "1" }, { dsp: 7 }]) {
    assert.equal(dspVisible(dspSupport(device)), false, JSON.stringify(device));
  }
});

test("firmware without the component reads as unsupported, a loading state as unknown", () => {
  assert.equal(dspSupport({ e: {} }), 0);
  assert.equal(dspSupport(null), null);
  assert.equal(dspSupport({ dsp: 2 }), 2);
});

test("a bookmark to the DSP page is bounced only on a definite no", () => {
  assert.equal(dspRedirect(0), true);
  assert.equal(dspRedirect(1), false);
  assert.equal(dspRedirect(2), false);
  assert.equal(dspRedirect(null), false);
});

test("Audio lists its pages only while DSP is visible", () => {
  assert.deepEqual(navSubs("AUDIO", 1, SETTINGS, AUDIO), AUDIO);
  for (const s of [0, 2, null]) assert.deepEqual(navSubs("AUDIO", s, SETTINGS, AUDIO), [], String(s));
  for (const tab of ["NOW", "PRESENCE", "WAKE"]) assert.deepEqual(navSubs(tab, 1, SETTINGS, AUDIO), [], tab);
});

test("Settings > Developer follows hasDevTools whatever the DSP answer", () => {
  const dev = { mic: 1, dsp: 1 };
  const release = { e: {}, dsp: 1 };
  for (const s of [0, 1, 2, null]) {
    assert.ok(navSubs("SETTINGS", s, settingsFor(dev), AUDIO).some(r => r.slug === "developer"), `dev build, dsp ${s}`);
    assert.ok(!navSubs("SETTINGS", s, settingsFor(release), AUDIO).some(r => r.slug === "developer"), `release build, dsp ${s}`);
  }
});

test("version strings are what a connected XMOS publishes, and nothing else", () => {
  for (const v of ["v1.1.0", "v1.1.0-dev.8", "v1.0.3-beta.2", "v12.0.1"]) assert.equal(isXmosVersion(v), true, v);
  for (const v of ["XMOS not responding", "Flashing Mode", "Flashing... (40%)", "Flashing failed", "", null, undefined]) {
    assert.equal(isXmosVersion(v), false, String(v));
  }
});

test("the re-check fires when the entity becomes a version string", () => {
  assert.equal(dspRecheckDue("XMOS not responding", "v1.1.0-dev.8", 2), true);
  // A reflash of the same local build: the version is unchanged, but the XMOS reconnected.
  assert.equal(dspRecheckDue("Flashing... (100%)", "v1.1.0-dev.8", 1), true);
  assert.equal(dspRecheckDue("Flashing Mode", "v1.1.0", 0), true);
});

test("progress updates and repeats of the same version do not re-check", () => {
  assert.equal(dspRecheckDue("Flashing... (10%)", "Flashing... (20%)", 1), false);
  assert.equal(dspRecheckDue("v1.1.0-dev.8", "Flashing Mode", 1), false);
  assert.equal(dspRecheckDue("v1.1.0-dev.8", "v1.1.0-dev.8", 1), false);
});

test("page load re-checks only when the state payload could not answer", () => {
  assert.equal(dspRecheckDue(undefined, "v1.1.0-dev.8", 1), false);
  assert.equal(dspRecheckDue(undefined, "v1.1.0-dev.8", 0), false);
  assert.equal(dspRecheckDue(undefined, "v1.1.0-dev.8", 2), true);
  assert.equal(dspRecheckDue(undefined, "v1.1.0-dev.8", null), true);
  assert.equal(dspRecheckDue(undefined, "XMOS not responding", 2), false);
});
