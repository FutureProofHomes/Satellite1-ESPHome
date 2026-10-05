/**
 * When Settings > Developer exists (src/lib/devtools.js): exactly when the firmware carries one of
 * the tools config/satellite1.dev.yaml adds, judged from the state payload alone.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { devTools, hasDevTools } from "../src/lib/devtools.js";

const XMOS = { xmos_fw_choice: 1, xmos_fw_install: 1, xmos_fw_refresh: 1, xmos_fw_list: 1, xmos_fw_status: 1 };

test("a release build has no developer page, and neither does a device still loading", () => {
  assert.equal(hasDevTools(null), false);
  assert.equal(hasDevTools({ e: { speaker_channel: 1, restart: 1 } }), false);
});

test("the XMOS picker needs all five of its entities", () => {
  assert.equal(devTools({ e: XMOS }).xmos, true);
  const { xmos_fw_refresh, ...four } = XMOS;
  assert.equal(devTools({ e: four }).xmos, false);
});

test("each tool alone is enough for the page", () => {
  assert.equal(hasDevTools({ e: XMOS }), true);
  assert.equal(hasDevTools({ e: { amp_gain: 1 } }), true);
  assert.equal(hasDevTools({ e: {}, mic: { max: 2 } }), true);
  assert.equal(hasDevTools({ e: {}, sysmon: 1 }), true);
  assert.deepEqual(devTools({ e: { amp_gain: 1 }, mic: { max: 2 } }), { xmos: false, amp: true, mic: true, sysmon: false });
});
