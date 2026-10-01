/**
 * The Presence tab's arithmetic: what the radar API's payloads mean on screen, and the bodies the
 * zone editor and gate chart send back. Shapes are satellite1_radar's radar_tuner_handler.cpp.
 *
 * Every distance here is centimetres, as the device sends and stores them. Feet exist only in the
 * labels; the distance_unit_ft switch never changes a number the device hears.
 */

export const CM_PER_FT = 30.48;

/** The LD2410's gate count; its threshold arrays are accepted only at exactly this length. */
export const NUM_GATES = 9;

/** The distance unit switch's state. Absent on older firmware, which then reads metric. */
export const feetOn = (sw) => !!sw && (sw.value === true || sw.state === "ON");

export const distLabel = (cm, isFt) => (isFt ? `${(cm / CM_PER_FT).toFixed(1)} ft` : `${(cm / 100).toFixed(1)} m`);

/** Detection range readout. Zero is no cut-off, which on the LD2450 is its full 6 m reach. */
export const rangeLabel = (cm, isFt) =>
  cm === 0 ? (isFt ? "20 ft" : "6 m") : isFt ? `${(cm / CM_PER_FT).toFixed(1)} ft` : `${cm} cm`;

/**
 * The LD2450's targets with their slot index. It always reports three slots, and an empty one sits
 * at exactly 0,0 - there is no count field to go by.
 */
export function realTargets(live) {
  const t = (live && live.targets) || [];
  return t.map((p, i) => ({ x: Number(p.x) || 0, y: Number(p.y) || 0, i })).filter((p) => !(p.x === 0 && p.y === 0));
}

export function nearest(targets) {
  let near = null;
  for (const t of targets) {
    const d = Math.hypot(t.x, t.y);
    if (!near || d < near.d) near = { ...t, d };
  }
  return near;
}

/** A 30 degree cone straight ahead; an x threshold would call arm's length "Left" and across the room "Ahead". */
export function direction(t) {
  const ang = (Math.atan2(t.x, t.y) * 180) / Math.PI;
  return ang < -15 ? "Left" : ang > 15 ? "Right" : "Ahead";
}

/**
 * The LD2450 Presence pill: the occupied zones by number, else Yes/No from the debounced target
 * state. A zone only counts while the config defines it, because a deleted zone's sensor lives on
 * until reboot reporting "Undefined".
 */
export function presenceLabel(config, states) {
  const zones = (config && config.zones) || [];
  const busy = [0, 1, 2].filter((i) => {
    const s = (zones[i] || []).length > 2 && states[`text_sensor/Radar Zone ${i + 1}`];
    return !!s && !!s.value && s.value !== "Clear" && s.value !== "Undefined";
  });
  if (busy.length) return `Zone ${busy.map((i) => i + 1).join(", ")}`;
  const target = states["text_sensor/Radar Target"];
  return target && target.value && target.value !== "Clear" ? "Yes" : "No";
}

/** An LD2410 gate's band of distance, "0.75–1.5m": what the row watches rather than its index. */
export function gateLabel(i, fine, isFt) {
  const step = fine ? 0.2 : 0.75;
  const fmt = (m) => String(parseFloat(isFt ? ((m * 100) / CM_PER_FT).toFixed(1) : m.toFixed(2)));
  return `${fmt(i * step)}\u2013${fmt((i + 1) * step)}${isFt ? "ft" : "m"}`;
}

/** A threshold array at the endpoint's fixed length, with numbers in every slot. */
export const gateArray = (arr) => Array.from({ length: NUM_GATES }, (_, i) => Number((arr || [])[i]) || 0);

const copyPoints = (pts) => (pts || []).map(({ x, y }) => ({ x: Math.round(x), y: Math.round(y) }));

/**
 * The POST body that saves an edited shape. `zones` goes as all three polygons or not at all (the
 * handler answers 400 to any other count), so every save resends the full set. `from` is the slot
 * the shape came from (null for a new one) and is emptied; `which` is where it lands. They differ
 * when the Exclusion toggle converts a shape, and one body does both. Deleting is landing nothing
 * where it came from.
 */
export function shapeBody(config, from, which, points) {
  const zones = [0, 1, 2].map((i) => copyPoints(((config && config.zones) || [])[i]));
  let exclusion = copyPoints(config && config.exclusion);
  if (from === "x") exclusion = [];
  else if (typeof from === "number") zones[from] = [];
  if (which === "x") exclusion = copyPoints(points);
  else if (typeof which === "number") zones[which] = copyPoints(points);
  return { zones, exclusion };
}

/** Ray-cast point-in-polygon, the same test the handler runs on targets. */
export function inPolygon(p, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

/** Where a shape's name goes: the mean of its corners, close enough for room-shaped polygons. */
export function labelPoint(pts) {
  return { x: pts.reduce((a, p) => a + p.x, 0) / pts.length, y: pts.reduce((a, p) => a + p.y, 0) / pts.length };
}

/** A whole-shape drag, clamped as a group so no corner leaves the plot and the shape never distorts. */
export function clampShift(points, dx, dy, halfW, depth) {
  const xs = points.map((q) => q.x);
  const ys = points.map((q) => q.y);
  return [
    Math.max(-halfW - Math.min(...xs), Math.min(halfW - Math.max(...xs), dx)),
    Math.max(-Math.min(...ys), Math.min(depth - Math.max(...ys), dy)),
  ];
}

/**
 * Each target slot's recent positions, newest first, for the comet tail. The same object back when
 * nothing moved, so a render that is not a new poll adds nothing; a slot that stopped reporting
 * loses its tail with it.
 */
export function stepTrails(prev, targets, len) {
  const next = {};
  for (const t of targets) {
    const a = prev[t.i] || [];
    next[t.i] = a.length && a[0].x === t.x && a[0].y === t.y ? a : [{ x: t.x, y: t.y }, ...a].slice(0, len);
  }
  const same = Object.keys(next).length === Object.keys(prev).length && Object.keys(next).every((k) => next[k] === prev[k]);
  return same ? prev : next;
}
