/**
 * The Audio tab's logic: why the speaker lists are missing, and the tree arithmetic over the
 * selection the device stores at /api/sat1/sel. It mirrors src/routes/config.jsx and src/tree.jsx
 * for as long as v1 ships, so change them together; their comments carry the reasoning. The shapes
 * are the device's:
 *
 *   payload  /api/sat1/ha's `d`: {areas: [{i, n, p: [[id, name, caps, avail], ...]}], loose: [...]}
 *   sel      one tree's {areas, extra, excluded} as Sets - whole area ids, individually chosen
 *            entity ids, and `areaId:entityId` carve-outs from a whole area
 *
 * Every edit returns a fresh selection rather than mutating, so the caller can hand it straight to
 * the selection write and keep the one it had for the revert.
 */
import { TEXT } from "../../src/copy.js";
import { HA_NEVER, haBlocked, haTooOld } from "../../src/lib/device.js";

/** The capability a tree's call needs of a player: play_media for routing, volume_set for ducking. */
export const NEED_MEDIA = 1;
export const NEED_VOLUME = 2;
/** Caps bit 4: this device's own media player. */
const CAP_SELF = 4;

/**
 * Why the area and player lists are not here, or null when they are. `fix` marks the one state the
 * walk-through can act on; `soft` the one that still leaves the lists usable - a device in no area
 * can still route to and duck every other room, which is what TEXT.ha_no_area tells it.
 *
 * `deviceHa` is the state payload's `ha` flag: whether the native API is connected at all, which is
 * what separates "not asked yet" from "nobody to ask" while nothing has ever arrived.
 */
export function haProblem(ha, deviceHa) {
  if (!ha) return { text: TEXT.ha_pending };
  if (haBlocked(ha)) return { text: TEXT.ha_blocked, fix: true };
  if (haTooOld(ha)) return { text: TEXT.ha_too_old };
  if (ha.age === HA_NEVER) return { text: deviceHa ? TEXT.ha_pending : TEXT.ha_never };
  if (!ha.d) return { text: TEXT.ha_never };
  const empty = !ha.d.areas?.length && !ha.d.loose?.length;
  if (empty) return { text: TEXT.ha_no_players };
  if (!ha.d.area) return { text: TEXT.ha_no_area, soft: true };
  return null;
}

/** The payload the trees draw from: none at all while a problem empties them. */
export const treePayload = (ha, problem) => (problem && !problem.soft ? null : ha?.d || null);

/** `?? 3` keeps rows cached before the caps field existed live in both trees. */
export const capOk = (row, need) => ((row[2] ?? 3) & need) !== 0;
export const isSelf = (row) => ((row[2] ?? 0) & CAP_SELF) !== 0;
/** Whether a row can be added to the selection at all. */
export const eligible = (row, need) => capOk(row, need) && !isSelf(row);
/** `?? 1` keeps rows cached before the availability field existed reading as online. */
export const isLive = (row) => (row[3] ?? 1) !== 0;

/**
 * The grey caption on a row, or null. Permanent reasons outrank offline, so a row that is both
 * says why it stays grey after the player comes back.
 */
export function rowWhy(row, need) {
  if (isSelf(row)) return TEXT.cap_self;
  if (!capOk(row, need)) return need === NEED_VOLUME ? TEXT.cap_no_volume : TEXT.cap_no_media;
  if (!isLive(row)) return TEXT.player_offline;
  return null;
}

/** Whether anything is chosen: the device's own test for "routing (or ducking) is on". */
export const hasTargets = (sel) => sel.areas.size > 0 || sel.extra.size > 0;

/** Covered by its whole area and not carved out, or picked on its own. `areaId` null for loose rows. */
export const isOn = (sel, areaId, id) =>
  (areaId !== null && sel.areas.has(areaId) && !sel.excluded.has(`${areaId}:${id}`)) || sel.extra.has(id);

/** A player row's tick: never on for an ineligible row, whatever an old selection names. */
export const rowOn = (sel, areaId, row, need) => eligible(row, need) && isOn(sel, areaId, row[0]);

const copy = (sel) => ({ areas: new Set(sel.areas), extra: new Set(sel.extra), excluded: new Set(sel.excluded) });

/** Inside a whole area a click carves the player out (or back in); anywhere else it toggles the pick. */
export function togglePlayer(sel, areaId, id) {
  const next = copy(sel);
  if (areaId !== null && next.areas.has(areaId)) {
    const key = `${areaId}:${id}`;
    if (next.excluded.has(key)) {
      next.excluded.delete(key);
    } else {
      next.excluded.add(key);
      next.extra.delete(id);
    }
    return next;
  }
  if (next.extra.has(id)) next.extra.delete(id);
  else next.extra.add(id);
  return next;
}

/** Over eligible rows only, or an area with a greyed player in it could never read as whole. */
export function areaState(sel, area, need) {
  const ids = area.p.filter((r) => eligible(r, need)).map((r) => r[0]);
  if (sel.areas.has(area.i)) return ids.some((id) => sel.excluded.has(`${area.i}:${id}`)) ? "mixed" : "on";
  const on = ids.filter((id) => sel.extra.has(id)).length;
  return on === 0 ? "off" : on === ids.length ? "on" : "mixed";
}

/** "whole area", or "chosen/eligible". */
export function areaCount(sel, area, need) {
  const rows = area.p.filter((r) => eligible(r, need));
  if (sel.areas.has(area.i)) {
    const cut = rows.filter((r) => sel.excluded.has(`${area.i}:${r[0]}`)).length;
    return cut === 0 ? "whole area" : `${rows.length - cut}/${rows.length}`;
  }
  return `${rows.filter((r) => sel.extra.has(r[0])).length}/${rows.length}`;
}

/**
 * The area's bulk box. From on it lets the whole area go; from off or mixed it takes the whole area,
 * so a speaker added to the room later is included. Either way the area's carve-outs and individual
 * picks are cleared, since the area id now says everything about it.
 */
export function toggleArea(sel, area, need) {
  const state = areaState(sel, area, need);
  const next = copy(sel);
  for (const k of sel.excluded) if (k.startsWith(`${area.i}:`)) next.excluded.delete(k);
  for (const r of area.p) next.extra.delete(r[0]);
  if (state === "on") next.areas.delete(area.i);
  else next.areas.add(area.i);
  return next;
}

/** Nothing eligible to add, and nothing chosen to remove: the bulk box has no job. */
export const areaLocked = (sel, area, need) =>
  areaState(sel, area, need) === "off" && !area.p.some((r) => eligible(r, need));

export function looseState(sel, loose, need) {
  const rows = loose.filter((r) => eligible(r, need));
  const on = rows.filter((r) => sel.extra.has(r[0])).length;
  return on === 0 ? "off" : on === rows.length ? "on" : "mixed";
}

export function looseCount(sel, loose, need) {
  const rows = loose.filter((r) => eligible(r, need));
  return `${rows.filter((r) => sel.extra.has(r[0])).length}/${rows.length}`;
}

/** Scoped to eligible rows both ways: a stale ineligible pick is removed by its own row. */
export function toggleLoose(sel, loose, need) {
  const all = looseState(sel, loose, need) === "on";
  const next = copy(sel);
  for (const r of loose) {
    if (!eligible(r, need)) continue;
    if (all) next.extra.delete(r[0]);
    else next.extra.add(r[0]);
  }
  return next;
}

export const looseLocked = (sel, loose, need) =>
  looseState(sel, loose, need) === "off" && !loose.some((r) => eligible(r, need));

/** One press of a ± stepper: a step either way, onto the step grid, inside the range. */
export function nudge(value, dir, min, max, step) {
  const s = step > 0 ? step : 1;
  const v = Math.round((value + dir * s - min) / s) * s + min;
  return Math.min(max, Math.max(min, v));
}
