/**
 * The Audio tab's logic: why the speaker lists are missing, and the tree arithmetic over the
 * selection the device stores at /api/sat1/sel. The shapes are the device's:
 *
 *   payload  /api/sat1/ha's `d`: {areas: [{i, n, p: [[id, name, caps, avail], ...]}], loose: [...]}
 *   sel      one tree's {areas, extra, excluded} as Sets - whole area ids, individually chosen
 *            entity ids (players in no area included), and `areaId:entityId` carve-outs from a
 *            whole area
 *
 * The edits work on that stored shape directly rather than flattening it to a list of ids and
 * expanding it again on save, which is what keeps the selection small enough to store: unticking
 * one speaker in a twelve-player area adds one exclusion rather than replacing the area with eleven
 * ids - about forty characters against about four hundred and sixty, and the text entity that
 * stored the selection before /api/sat1/sel capped at 255. The area prefix on an exclusion is not
 * needed to resolve it, since an entity belongs to at most one area; it is there so the firmware
 * can answer "is my own area selected whole" without knowing area membership, which is what the
 * two derived switches in Home Assistant read.
 *
 * Every edit returns a fresh selection rather than mutating, so the caller can hand it straight to
 * the selection write and keep the one it had for the revert.
 */
import { TEXT } from "../../src/copy.js";
import { HA_NEVER, haBlocked, haTooOld } from "../../src/lib/device.js";

/** The capability a tree's call needs of a player: play_media for routing, volume_set for ducking.
 *  They differ, which is why the same tree greys different rows in each card. */
export const NEED_MEDIA = 1;
export const NEED_VOLUME = 2;
/** Caps bit 4: this device's own media player. Never eligible - routing to itself is the Local
 *  Speaker row's job, and ducking its own volume while it talks is never right. */
const CAP_SELF = 4;

/**
 * Why the area and player lists are not here, or null when they are: always one precise state,
 * never a generic failure. `fix` marks the one state the walk-through drawer can act on, through
 * the "Show fix" link; `soft` the one that still leaves the lists usable - a device in no area can
 * still route to and duck every other room, which is what TEXT.ha_no_area tells it. What it cannot
 * do is answer the two whole-my-area switches, which is why that state is still reported.
 *
 * `deviceHa` is the state payload's `ha` flag: whether the native API is connected at all, which is
 * what separates "not asked yet" from "nobody to ask" while nothing has ever arrived.
 */
export function haProblem(ha, deviceHa) {
  // No response yet is a wait, not a fault. Returning null here would say "go ahead and read ha.d"
  // about a payload that is not there, and whether the tab crashed would come down to whether
  // /api/sat1/sel answered before /api/sat1/ha, since the selection gates the first render.
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
/** `?? 1` keeps rows cached before the availability field existed reading as online. Offline only
 *  greys and labels a row: it is transient, so the tick stays editable and the stored selection is
 *  untouched, while the call-time walks in tts_routing.yaml skip the player until it returns. */
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

/** Covered by its whole area and not carved out, or picked on its own. `areaId` null for loose rows.
 *  Both hold at once, which is what lets an area be taken whole while a player in a different room
 *  is added on its own. */
export const isOn = (sel, areaId, id) =>
  (areaId !== null && sel.areas.has(areaId) && !sel.excluded.has(`${areaId}:${id}`)) || sel.extra.has(id);

/** A player row's tick: never on for an ineligible row, whatever an old selection names - the
 *  call-time walks skip it, so a tick would be a false promise. */
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

/**
 * Over eligible rows only, or an area with a greyed player in it could never read as whole and
 * "select whole area" would look broken. A whole area reads "on" unless something is carved out of
 * it, which is exactly what the firmware's two derived switches report, so this box and the switch
 * in Home Assistant agree.
 */
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
 * so a speaker added to the room later is included - from mixed too, since a second click taking
 * the whole thing is what a tri-state box trains people to expect. Either way the area's carve-outs
 * and individual picks are cleared, since the area id now says everything about it.
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

/** Scoped to eligible rows both ways, so the bulk box never adds or removes a greyed row. */
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
