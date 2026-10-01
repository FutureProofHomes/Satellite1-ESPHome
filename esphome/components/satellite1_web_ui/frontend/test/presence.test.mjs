/**
 * The Presence tab's helpers. The bodies are the ones satellite1_radar's handler validates, so a
 * wrong shape here is a 400 on the device, not a cosmetic slip.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  clampShift,
  direction,
  distLabel,
  feetOn,
  gateArray,
  gateLabel,
  grabCorner,
  inPolygon,
  nearest,
  presenceLabel,
  rangeLabel,
  realTargets,
  shapeBody,
  stepTrails,
} from "../v2/lib/presence.js";

const SQUARE = [
  { x: -100, y: 100 },
  { x: 100, y: 100 },
  { x: 100, y: 300 },
  { x: -100, y: 300 },
];

test("empty target slots at 0,0 are dropped and the rest keep their slot", () => {
  const live = { targets: [{ x: 0, y: 0 }, { x: -50.5, y: 210 }, { x: 0, y: 0 }] };
  assert.deepEqual(realTargets(live), [{ x: -50.5, y: 210, i: 1 }]);
  assert.deepEqual(realTargets(null), []);
  assert.deepEqual(realTargets({ targets: [{ x: 0.0, y: 0.0 }] }), []);
});

test("the nearest target and where it stands", () => {
  const near = nearest([{ x: 300, y: 400, i: 0 }, { x: 0, y: 120, i: 2 }]);
  assert.equal(near.i, 2);
  assert.equal(near.d, 120);
  assert.equal(nearest([]), null);
  assert.equal(direction({ x: 0, y: 200 }), "Ahead");
  assert.equal(direction({ x: 40, y: 400 }), "Ahead");
  assert.equal(direction({ x: -150, y: 200 }), "Left");
  assert.equal(direction({ x: 150, y: 200 }), "Right");
});

test("distances read in metres or feet, and zero range is the full reach", () => {
  assert.equal(distLabel(250, false), "2.5 m");
  assert.equal(distLabel(250, true), "8.2 ft");
  assert.equal(rangeLabel(0, false), "6 m");
  assert.equal(rangeLabel(0, true), "20 ft");
  assert.equal(rangeLabel(450, false), "450 cm");
  assert.equal(rangeLabel(450, true), "14.8 ft");
});

test("the unit switch reads from either state shape, and absent is metric", () => {
  assert.equal(feetOn(undefined), false);
  assert.equal(feetOn({ value: true }), true);
  assert.equal(feetOn({ state: "ON" }), true);
  assert.equal(feetOn({ value: false, state: "OFF" }), false);
});

test("presence names occupied zones, ignoring ones the config no longer defines", () => {
  const config = { zones: [SQUARE, [], SQUARE] };
  const states = {
    "text_sensor/Radar Target": { value: "Still" },
    "text_sensor/Radar Zone 1": { value: "Clear" },
    "text_sensor/Radar Zone 2": { value: "Approaching" },
    "text_sensor/Radar Zone 3": { value: "Moving Away" },
  };
  assert.equal(presenceLabel(config, states), "Zone 3");
  states["text_sensor/Radar Zone 1"] = { value: "Still" };
  assert.equal(presenceLabel(config, states), "Zone 1, 3");
  assert.equal(presenceLabel({ zones: [SQUARE, [], []] }, { ...states, "text_sensor/Radar Zone 1": { value: "Undefined" } }), "Yes");
  assert.equal(presenceLabel(config, { "text_sensor/Radar Target": { value: "Clear" } }), "No");
  assert.equal(presenceLabel(null, {}), "No");
});

test("gate rows are labelled with the band of distance they watch", () => {
  assert.equal(gateLabel(0, false, false), "0\u20130.75m");
  assert.equal(gateLabel(1, false, false), "0.75\u20131.5m");
  assert.equal(gateLabel(2, true, false), "0.4\u20130.6m");
  assert.equal(gateLabel(1, false, true), "2.5\u20134.9ft");
  assert.equal(gateLabel(8, true, true), "5.2\u20135.9ft");
});

test("threshold arrays always go out at nine numbers", () => {
  assert.deepEqual(gateArray([50, "40", null]), [50, 40, 0, 0, 0, 0, 0, 0, 0]);
  assert.equal(gateArray(undefined).length, 9);
});

test("saving a shape resends all three zones and the exclusion", () => {
  const config = { zones: [SQUARE, [], []], exclusion: [] };
  const tri = [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 10.4 }];
  const body = shapeBody(config, null, 1, tri);
  assert.equal(body.zones.length, 3);
  assert.deepEqual(body.zones[0], SQUARE);
  assert.deepEqual(body.zones[1], [{ x: 0, y: 0 }, { x: 10, y: 0 }, { x: 0, y: 10 }]);
  assert.deepEqual(body.zones[2], []);
  assert.deepEqual(body.exclusion, []);
});

test("converting a zone to the exclusion moves it in one body", () => {
  const config = { zones: [SQUARE, SQUARE, []], exclusion: [{ x: 1, y: 1 }, { x: 2, y: 2 }, { x: 3, y: 1 }] };
  const body = shapeBody(config, 0, "x", SQUARE);
  assert.deepEqual(body.zones, [[], SQUARE, []]);
  assert.deepEqual(body.exclusion, SQUARE);
  const back = shapeBody(config, "x", 2, config.exclusion);
  assert.deepEqual(back.exclusion, []);
  assert.deepEqual(back.zones[2], config.exclusion);
});

test("deleting lands nothing where the shape came from", () => {
  const config = { zones: [SQUARE, SQUARE, SQUARE], exclusion: SQUARE };
  assert.deepEqual(shapeBody(config, 1, 1, []).zones, [SQUARE, [], SQUARE]);
  assert.deepEqual(shapeBody(config, "x", "x", []).exclusion, []);
  assert.deepEqual(shapeBody({}, 0, 0, []), { zones: [[], [], []], exclusion: [] });
});

test("a press grabs the nearest corner in reach, not the first", () => {
  const cramped = [{ x: 0, y: 100 }, { x: 30, y: 100 }, { x: 30, y: 130 }];
  assert.equal(grabCorner(cramped, { x: 25, y: 102 }, 40), 1);
  assert.equal(grabCorner(cramped, { x: 28, y: 125 }, 40), 2);
  assert.equal(grabCorner(cramped, { x: 200, y: 300 }, 40), -1);
  assert.equal(grabCorner([], { x: 0, y: 0 }, 40), -1);
});

test("point-in-polygon and whole-shape drags", () => {
  assert.equal(inPolygon({ x: 0, y: 200 }, SQUARE), true);
  assert.equal(inPolygon({ x: 0, y: 50 }, SQUARE), false);
  assert.equal(inPolygon({ x: 150, y: 200 }, SQUARE), false);
  assert.deepEqual(clampShift(SQUARE, 50, 50, 400, 640), [50, 50]);
  assert.deepEqual(clampShift(SQUARE, 900, -500, 400, 640), [300, -100]);
  assert.deepEqual(clampShift(SQUARE, -900, 900, 400, 640), [-300, 340]);
});

test("trails grow only when a target moves, and drop with their slot", () => {
  const a = stepTrails({}, [{ x: 10, y: 10, i: 0 }], 3);
  assert.deepEqual(a, { 0: [{ x: 10, y: 10 }] });
  assert.equal(stepTrails(a, [{ x: 10, y: 10, i: 0 }], 3), a);
  let t = a;
  for (const x of [20, 30, 40]) t = stepTrails(t, [{ x, y: 10, i: 0 }], 3);
  assert.deepEqual(t[0].map((p) => p.x), [40, 30, 20]);
  t = stepTrails(t, [{ x: 5, y: 5, i: 2 }], 3);
  assert.deepEqual(Object.keys(t), ["2"]);
  assert.deepEqual(stepTrails(t, [], 3), {});
});
