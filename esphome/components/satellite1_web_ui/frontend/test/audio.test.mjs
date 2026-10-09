/**
 * The Audio tab's trees and Home Assistant states. The tree arithmetic edits the exact shape the
 * device stores, and the two derived switches in Home Assistant read that shape back - so a wrong
 * carve-out here is a switch that disagrees with the box on screen.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { TEXT } from "../src/copy.js";
import {
  NEED_MEDIA,
  NEED_VOLUME,
  areaCount,
  areaLocked,
  areaState,
  haProblem,
  hasTargets,
  heldBits,
  looseCount,
  looseLocked,
  looseState,
  nudge,
  olderSatellite,
  routedLocks,
  rowOn,
  rowWhy,
  selfGroup,
  toggleArea,
  toggleLoose,
  togglePlayer,
  treePayload,
} from "../src/lib/audio.js";

const sel = (areas = [], extra = [], excluded = []) => ({
  areas: new Set(areas),
  extra: new Set(extra),
  excluded: new Set(excluded),
});
const plain = (s) => ({ areas: [...s.areas].sort(), extra: [...s.extra].sort(), excluded: [...s.excluded].sort() });

// Sonos plays and sets volume; the TV only plays; this device's own player carries bit 4.
const kitchen = {
  i: "kitchen",
  n: "Kitchen",
  p: [
    ["media_player.sonos", "Sonos", 3, 1],
    ["media_player.tv", "TV", 1, 1],
    ["media_player.self", "Satellite", 7, 1],
    ["media_player.nest", "Nest", 3, 0],
  ],
};
const loose = [
  ["media_player.all", "All", 3, 1],
  ["media_player.cast", "Cast", 1, 1],
];

test("Home Assistant states, in precedence order", () => {
  assert.deepEqual(haProblem(null, true), { text: TEXT.ha_pending });
  assert.deepEqual(haProblem({ actions: 2, rung: 1, age: 3, d: {} }, true), { text: TEXT.ha_blocked, fix: true });
  assert.deepEqual(haProblem({ actions: 0, rung: -1, age: -1 }, true), { text: TEXT.ha_blocked, fix: true });
  assert.deepEqual(haProblem({ actions: 3, rung: 0, age: -1 }, true), { text: TEXT.ha_too_old });
  assert.deepEqual(haProblem({ actions: 0, rung: 0, age: -1 }, true), { text: TEXT.ha_pending });
  assert.deepEqual(haProblem({ actions: 0, rung: 0, age: -1 }, false), { text: TEXT.ha_never });
  assert.deepEqual(haProblem({ actions: 1, rung: 1, age: 4 }, true), { text: TEXT.ha_never });
  assert.deepEqual(haProblem({ actions: 1, rung: 1, age: 4, d: { areas: [], loose: [] } }, true), { text: TEXT.ha_no_players });
  assert.deepEqual(haProblem({ actions: 1, rung: 1, age: 4, d: { areas: [kitchen] } }, true), { text: TEXT.ha_no_area, soft: true });
  assert.equal(haProblem({ actions: 1, rung: 1, age: 4, d: { area: "kitchen", areas: [kitchen] } }, true), null);
});

test("a working rung outranks an old 'too old' verdict", () => {
  assert.equal(haProblem({ actions: 3, rung: 2, age: 4, d: { area: "kitchen", loose } }, true), null);
});

test("only a device with no area keeps its lists", () => {
  const d = { areas: [kitchen] };
  assert.equal(treePayload({ d }, { text: TEXT.ha_no_area, soft: true }), d);
  assert.equal(treePayload({ d }, { text: TEXT.ha_blocked, fix: true }), null);
  assert.equal(treePayload({ d }, null), d);
  assert.equal(treePayload(null, { text: TEXT.ha_pending }), null);
});

test("each row says which job it can't do, permanent reasons first", () => {
  assert.equal(rowWhy(kitchen.p[0]), null);
  assert.equal(rowWhy(kitchen.p[1]), TEXT.cap_no_volume);
  assert.equal(rowWhy(kitchen.p[2]), TEXT.cap_self);
  assert.equal(rowWhy(kitchen.p[3]), TEXT.player_offline);
  assert.equal(rowWhy(["media_player.x", "X", 2, 0]), TEXT.cap_no_media);
  assert.equal(rowWhy(["media_player.y", "Y", 3, 0], false, true), TEXT.player_offline);
  // Older firmware only in a payload built with bits 8 and 16; the kitchen's Sonos predates them.
  assert.equal(rowWhy(den.p[2], false, true), TEXT.cap_old_firmware);
  assert.equal(rowWhy(den.p[2]), null);
  assert.equal(heldBits(house), true);
  assert.equal(heldBits({ areas: [kitchen], loose }), false);
  assert.equal(heldBits(null), false);
  // A Duck box held on by Announce needs no volume control, so the held Sonos shell says nothing.
  assert.equal(rowWhy(den.p[4]), TEXT.cap_no_volume);
  assert.equal(rowWhy(den.p[4], true), null);
  // Rows cached before the caps and availability fields existed stay usable in both columns.
  assert.equal(rowWhy(["media_player.old", "Old"]), null);
});

test("ticking a whole area clears its picks and carve-outs", () => {
  const before = sel([], ["media_player.sonos", "media_player.other"], ["kitchen:media_player.tv"]);
  assert.equal(areaState(before, kitchen, NEED_VOLUME), "mixed");
  const after = toggleArea(before, kitchen, NEED_VOLUME);
  assert.deepEqual(plain(after), { areas: ["kitchen"], extra: ["media_player.other"], excluded: [] });
  assert.equal(areaState(after, kitchen, NEED_VOLUME), "on");
  assert.equal(areaCount(after, kitchen, NEED_VOLUME), "whole area");
  assert.deepEqual(plain(before), { areas: [], extra: ["media_player.other", "media_player.sonos"], excluded: ["kitchen:media_player.tv"] });
});

test("unticking a whole area lets all of it go", () => {
  const after = toggleArea(sel(["kitchen"]), kitchen, NEED_MEDIA);
  assert.deepEqual(plain(after), { areas: [], extra: [], excluded: [] });
});

test("a carved-out whole area reads mixed and a second click takes it whole again", () => {
  const cut = togglePlayer(sel(["kitchen"]), "kitchen", "media_player.sonos");
  assert.deepEqual(plain(cut), { areas: ["kitchen"], extra: [], excluded: ["kitchen:media_player.sonos"] });
  assert.equal(areaState(cut, kitchen, NEED_MEDIA), "mixed");
  assert.equal(areaCount(cut, kitchen, NEED_MEDIA), "2/3");
  assert.equal(rowOn(cut, "kitchen", kitchen.p[0], NEED_MEDIA), false);
  assert.equal(rowOn(cut, "kitchen", kitchen.p[1], NEED_MEDIA), true);
  assert.deepEqual(plain(togglePlayer(cut, "kitchen", "media_player.sonos")), { areas: ["kitchen"], extra: [], excluded: [] });
  assert.deepEqual(plain(toggleArea(cut, kitchen, NEED_MEDIA)), { areas: ["kitchen"], extra: [], excluded: [] });
});

test("players outside a whole area toggle as individual picks", () => {
  const one = togglePlayer(sel(), "kitchen", "media_player.sonos");
  assert.deepEqual(plain(one), { areas: [], extra: ["media_player.sonos"], excluded: [] });
  // Ducking: the TV has no volume control, so the Sonos is every eligible player there is.
  assert.equal(areaState(one, kitchen, NEED_VOLUME), "mixed");
  assert.equal(areaCount(one, kitchen, NEED_VOLUME), "1/2");
  assert.deepEqual(plain(togglePlayer(one, "kitchen", "media_player.sonos")), { areas: [], extra: [], excluded: [] });
});

test("an ineligible row never reads as chosen, whatever the selection names", () => {
  const stale = sel(["kitchen"], ["media_player.self"]);
  assert.equal(rowOn(stale, "kitchen", kitchen.p[2], NEED_MEDIA), false);
  assert.equal(rowOn(stale, "kitchen", kitchen.p[1], NEED_VOLUME), false);
  // Offline is transient: a capable player's tick stays.
  assert.equal(rowOn(stale, "kitchen", kitchen.p[3], NEED_MEDIA), true);
});

test("an area with nothing eligible locks its box unless something is chosen there", () => {
  const tvOnly = { i: "den", n: "Den", p: [["media_player.tv", "TV", 1, 1]] };
  assert.equal(areaLocked(sel(), tvOnly, NEED_VOLUME), true);
  assert.equal(areaLocked(sel(["den"]), tvOnly, NEED_VOLUME), false);
  assert.equal(areaLocked(sel(), tvOnly, NEED_MEDIA), false);
});

test("the No Area Assigned group bulk-toggles eligible players only", () => {
  const all = toggleLoose(sel(), loose, NEED_VOLUME);
  assert.deepEqual(plain(all), { areas: [], extra: ["media_player.all"], excluded: [] });
  assert.equal(looseState(all, loose, NEED_VOLUME), "on");
  assert.equal(looseCount(all, loose, NEED_VOLUME), "1/1");
  const some = sel([], ["media_player.cast"]);
  assert.equal(looseState(some, loose, NEED_MEDIA), "mixed");
  assert.deepEqual(plain(toggleLoose(some, loose, NEED_MEDIA)), { areas: [], extra: ["media_player.all", "media_player.cast"], excluded: [] });
  // A stale ineligible pick survives the bulk untick; its own row removes it.
  const stale = sel([], ["media_player.all", "media_player.cast"]);
  assert.deepEqual(plain(toggleLoose(stale, loose, NEED_VOLUME)), { areas: [], extra: ["media_player.cast"], excluded: [] });
  assert.equal(looseLocked(sel(), [["media_player.cast", "Cast", 1, 1]], NEED_VOLUME), true);
});

// Caps 8: held by the silent clip (Sonos, current Satellite1). 16: turned down to Area Ducking
// Volume (any other brand). The older Satellite1 has neither; the cloud shell plays but has no
// volume control, and is held all the same.
const den = {
  i: "den",
  n: "Den",
  p: [
    ["media_player.den_sonos", "Den Sonos", 11, 1],
    ["media_player.den_sat", "Den Satellite1", 11, 1],
    ["media_player.den_old", "Den Satellite1 old", 3, 1],
    ["media_player.den_tv", "Den TV", 19, 1],
    ["media_player.den_cloud", "Den Sonos cloud", 9, 1],
  ],
};
const houseLoose = [
  ["media_player.cast_group", "Cast group", 19, 1],
  ["media_player.spare", "Spare", 19, 1],
];
const house = { areas: [den], loose: houseLoose };

test("routed players that are always ducked lock on, older Satellite1s do not", () => {
  const routing = sel(["den"], ["media_player.cast_group"], ["den:media_player.den_tv"]);
  const locks = routedLocks(house, routing);
  assert.deepEqual([...locks.ids].sort(), [
    "media_player.cast_group",
    "media_player.den_cloud",
    "media_player.den_sat",
    "media_player.den_sonos",
  ]);
  // The cast group is turned down to Remote Ducking Volume, so the slider stays live.
  assert.equal(locks.level, true);
  assert.equal(routedLocks(house, sel(["den"], [], ["den:media_player.den_tv"])).level, false);
  assert.equal(routedLocks(null, routing).ids.size, 0);
  // Rows cached before bits 8 and 16 existed never lock.
  assert.equal(routedLocks({ areas: [kitchen] }, sel(["kitchen"])).ids.size, 0);
});

test("locked rows count as ticked, and a room of them locks its box", () => {
  const locked = routedLocks(house, sel(["den"], [], ["den:media_player.den_tv"])).ids;
  assert.equal(areaState(sel(), den, NEED_VOLUME, locked), "mixed");
  assert.equal(areaCount(sel(), den, NEED_VOLUME, locked), "3/5");
  assert.equal(areaLocked(sel(), den, NEED_VOLUME, locked), false);
  const both = sel([], ["media_player.den_old", "media_player.den_tv"]);
  assert.equal(areaState(both, den, NEED_VOLUME, locked), "on");
  const sonosOnly = { i: "bath", n: "Bath", p: [den.p[0]] };
  const bathLock = new Set(["media_player.den_sonos"]);
  assert.equal(areaState(sel(), sonosOnly, NEED_VOLUME, bathLock), "on");
  assert.equal(areaLocked(sel(), sonosOnly, NEED_VOLUME, bathLock), true);
  // Taking a whole room clears picks as always; locked rows are covered by the area id like the rest.
  assert.deepEqual(plain(toggleArea(sel(), den, NEED_VOLUME, locked)), { areas: ["den"], extra: [], excluded: [] });
});

test("the No Area Assigned bulk box never picks a locked player", () => {
  const locked = new Set(["media_player.cast_group"]);
  assert.equal(looseState(sel(), houseLoose, NEED_VOLUME, locked), "mixed");
  assert.equal(looseCount(sel(), houseLoose, NEED_VOLUME, locked), "1/2");
  const all = toggleLoose(sel(), houseLoose, NEED_VOLUME, locked);
  assert.deepEqual(plain(all), { areas: [], extra: ["media_player.spare"], excluded: [] });
  assert.equal(looseState(all, houseLoose, NEED_VOLUME, locked), "on");
  assert.deepEqual(plain(toggleLoose(all, houseLoose, NEED_VOLUME, locked)), { areas: [], extra: [], excluded: [] });
  assert.equal(looseLocked(sel(), [houseLoose[0]], NEED_VOLUME, locked), true);
});

test("a column with nothing to count in a group says nothing", () => {
  const tvOnly = { i: "den", n: "Den", p: [["media_player.tv", "TV", 1, 1]] };
  assert.equal(areaCount(sel(), tvOnly, NEED_VOLUME), "");
  assert.equal(areaCount(sel(["den"]), tvOnly, NEED_VOLUME), "");
  assert.equal(areaCount(sel(), tvOnly, NEED_MEDIA), "0/1");
  const selfOnly = { i: "hall", n: "Hall", p: [kitchen.p[2]] };
  assert.equal(areaCount(sel(), selfOnly, NEED_MEDIA), "");
  assert.equal(looseCount(sel(), [["media_player.cast", "Cast", 1, 1]], NEED_VOLUME), "");
  // A locked row is counted though it takes no volume: a held Sonos shell.
  assert.equal(looseCount(sel(), [["media_player.cloud", "Cloud", 9, 1]], NEED_VOLUME, new Set(["media_player.cloud"])), "1/1");
});

test("only a Satellite1 on older firmware reads as one", () => {
  assert.equal(olderSatellite(den.p[2]), true);
  assert.equal(olderSatellite(den.p[0]), false);
  assert.equal(olderSatellite(den.p[3]), false);
  assert.equal(olderSatellite(kitchen.p[2]), false);
  // The TV plays but takes no volume, and a row cached before caps existed says nothing.
  assert.equal(olderSatellite(kitchen.p[1]), false);
  assert.equal(olderSatellite(["media_player.old", "Old"]), false);
});

test("this device's row is found in its area, in No Area Assigned, or not at all", () => {
  assert.equal(selfGroup({ areas: [den, kitchen] }), "kitchen");
  assert.equal(selfGroup({ areas: [den], loose: [["media_player.self", "Satellite", 7, 1]] }), null);
  assert.equal(selfGroup(house), undefined);
  assert.equal(selfGroup(null), undefined);
});

test("anything chosen, whole areas or picks, is an active selection", () => {
  assert.equal(hasTargets(sel()), false);
  assert.equal(hasTargets(sel(["kitchen"])), true);
  assert.equal(hasTargets(sel([], ["media_player.all"])), true);
  assert.equal(hasTargets(sel([], [], ["kitchen:media_player.tv"])), false);
});

test("the stepper moves a step at a time on the grid and stops at the ends", () => {
  assert.equal(nudge(50, 1, 0, 100, 5), 55);
  assert.equal(nudge(50, -1, 0, 100, 5), 45);
  assert.equal(nudge(97, 1, 0, 100, 5), 100);
  assert.equal(nudge(3, -1, 0, 100, 5), 0);
  assert.equal(nudge(100, 1, 0, 100, 5), 100);
  assert.equal(nudge(0, -1, 0, 100, 5), 0);
  assert.equal(nudge(42, 1, 0, 100, 5), 45);
  assert.equal(nudge(10, 1, 0, 100, 0), 11);
});
