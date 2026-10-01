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
  looseCount,
  looseLocked,
  looseState,
  nudge,
  rowOn,
  rowWhy,
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

test("each row says why it is grey, permanent reasons first", () => {
  assert.equal(rowWhy(kitchen.p[0], NEED_VOLUME), null);
  assert.equal(rowWhy(kitchen.p[1], NEED_VOLUME), TEXT.cap_no_volume);
  assert.equal(rowWhy(kitchen.p[1], NEED_MEDIA), null);
  assert.equal(rowWhy(kitchen.p[2], NEED_MEDIA), TEXT.cap_self);
  assert.equal(rowWhy(kitchen.p[3], NEED_MEDIA), TEXT.player_offline);
  assert.equal(rowWhy(["media_player.x", "X", 2, 0], NEED_MEDIA), TEXT.cap_no_media);
  // Rows cached before the caps and availability fields existed stay usable in both trees.
  assert.equal(rowWhy(["media_player.old", "Old"], NEED_MEDIA), null);
  assert.equal(rowWhy(["media_player.old", "Old"], NEED_VOLUME), null);
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
