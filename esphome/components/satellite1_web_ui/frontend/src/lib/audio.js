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
import { TEXT } from "../copy.js";
import { HA_NEVER, haBlocked, haTooOld } from "./device.js";

/** The capability each column's call needs of a player: play_media for Announce, volume_set for
 *  Duck. They differ, which is why a row can grey one of its boxes and not the other. */
export const NEED_MEDIA = 1;
export const NEED_VOLUME = 2;
/** Caps bit 4: this device's own media player. Never eligible - its Announce box is the "Announce
 *  on this device" setting rather than a routing pick, and ducking its own volume while it talks
 *  is never right. */
const CAP_SELF = 4;
/** Caps bits 8 and 16: ducked whenever it plays this device's responses, whatever the ducking
 *  selection says (web_ui_ha.yaml). 8 lowers its own music, held by a silent clip - a Sonos, or a
 *  Satellite1 on current firmware; 16 is turned down to Remote Ducking Volume - any other brand.
 *  Neither on a Satellite1 running older firmware, which is ducked only if ticked. */
const CAP_HELD = 8;
const CAP_LEVEL = 16;
const NONE = new Set();

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
 * The grey caption on a player row, or null. One row carries both columns, so it names whichever
 * job the player cannot do; permanent reasons outrank offline, so a row that is both says why a box
 * stays grey after the player comes back. `locked` is a Duck box held on by Announce, which needs no
 * volume control - a held Sonos shell has none - so its absence goes unsaid there. `bits` is
 * heldBits(payload), without which no row can be told apart as older firmware.
 */
export function rowWhy(row, locked = false, bits = false) {
  if (isSelf(row)) return TEXT.cap_self;
  if (!capOk(row, NEED_MEDIA)) return TEXT.cap_no_media;
  if (!capOk(row, NEED_VOLUME) && !locked) return TEXT.cap_no_volume;
  if (!isLive(row)) return TEXT.player_offline;
  if (bits && olderSatellite(row)) return TEXT.cap_old_firmware;
  return null;
}

/**
 * A Satellite1 on firmware from before routed speakers were kept ducked: it plays and takes a
 * volume, but carries neither bit 8 nor 16, which web_ui_ha.yaml withholds only from a
 * FutureProofHomes device with no Announcement Volume to hold. Its Duck box stays free while it
 * announces. Only meaningful in a payload built with the bits (heldBits): a device on firmware
 * older than them sends every row without either, and this page also drives such devices.
 */
export const olderSatellite = (row) =>
  row[2] !== undefined && !isSelf(row) && (row[2] & 3) === 3 && !(row[2] & (CAP_HELD | CAP_LEVEL));

/** Whether any row carries bit 8 or 16, which is the payload's only sign of being built with them. */
export const heldBits = (payload) =>
  [...(payload?.areas || []).flatMap((a) => a.p), ...(payload?.loose || [])].some(
    (r) => ((r[2] ?? 0) & (CAP_HELD | CAP_LEVEL)) !== 0,
  );

/**
 * Where this device's own row is: its area's id, null for "No Area Assigned", undefined when the
 * payload does not carry it - no payload yet, or one cut short (`t`). The list hangs the "Announce
 * on this device" box off that row, so the page falls back to a switch when it is missing.
 */
export function selfGroup(payload) {
  for (const area of payload?.areas || []) if (area.p.some(isSelf)) return area.i;
  return (payload?.loose || []).some(isSelf) ? null : undefined;
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

/**
 * The Duck column's locked boxes: the players ticked to Announce that are ducked for it regardless
 * (caps 8 or 16), as `ids`. `level` says whether any of them is turned down to Remote Ducking
 * Volume, which is what keeps that slider live with nothing ticked to Duck. Tested with
 * play_media's eligibility because that is what puts a player in the routing walk.
 */
export function routedLocks(payload, routing) {
  const ids = new Set();
  let level = false;
  const walk = (areaId) => (row) => {
    const caps = row[2] ?? 0;
    if (!(caps & (CAP_HELD | CAP_LEVEL)) || !eligible(row, NEED_MEDIA) || !isOn(routing, areaId, row[0])) return;
    ids.add(row[0]);
    if (caps & CAP_LEVEL) level = true;
  };
  for (const area of payload?.areas || []) area.p.forEach(walk(area.i));
  (payload?.loose || []).forEach(walk(null));
  return { ids, level };
}

/** The rows a group's box and count read over: those a person could tick, and the locked ones,
 *  which show ticked whether or not they could be - a held Sonos needs no volume control. */
const counted = (rows, need, locked) => rows.filter((r) => eligible(r, need) || locked.has(r[0]));
const ticked = (sel, areaId, row, locked) => locked.has(row[0]) || isOn(sel, areaId, row[0]);
/** A group with nothing left to change: no row a person could tick that is not locked already. */
const nothingFree = (rows, need, locked) => !rows.some((r) => eligible(r, need) && !locked.has(r[0]));

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
 * in Home Assistant agree - and "mixed" with everything carved out, since the area itself is still
 * chosen. `locked` (routedLocks) counts as ticked, so a room whose players all play responses reads
 * whole.
 */
export function areaState(sel, area, need, locked = NONE) {
  const rows = counted(area.p, need, locked);
  if (!rows.length) return sel.areas.has(area.i) ? "on" : "off";
  const on = rows.filter((r) => ticked(sel, area.i, r, locked)).length;
  if (on === rows.length) return "on";
  return on === 0 && !sel.areas.has(area.i) ? "off" : "mixed";
}

/** "whole area", or "ticked/counted" - or "" when the column has no row in this area to count, so
 *  the line under the area's name leaves that column out. */
export function areaCount(sel, area, need, locked = NONE) {
  const rows = counted(area.p, need, locked);
  if (!rows.length) return "";
  const on = rows.filter((r) => ticked(sel, area.i, r, locked)).length;
  return sel.areas.has(area.i) && on === rows.length ? "whole area" : `${on}/${rows.length}`;
}

/**
 * The area's bulk box. From on it lets the whole area go; from off or mixed it takes the whole area,
 * so a speaker added to the room later is included - from mixed too, since a second click taking
 * the whole thing is what a tri-state box trains people to expect. Either way the area's carve-outs
 * and individual picks are cleared, since the area id now says everything about it.
 */
export function toggleArea(sel, area, need, locked = NONE) {
  const state = areaState(sel, area, need, locked);
  const next = copy(sel);
  for (const k of sel.excluded) if (k.startsWith(`${area.i}:`)) next.excluded.delete(k);
  for (const r of area.p) next.extra.delete(r[0]);
  if (state === "on") next.areas.delete(area.i);
  else next.areas.add(area.i);
  return next;
}

/** The bulk box has no job: nothing it could tick that is not locked already, and either a locked
 *  row deciding the box or nothing chosen for it to remove. */
export const areaLocked = (sel, area, need, locked = NONE) =>
  nothingFree(area.p, need, locked) &&
  (area.p.some((r) => locked.has(r[0])) || areaState(sel, area, need, locked) === "off");

export function looseState(sel, loose, need, locked = NONE) {
  const rows = counted(loose, need, locked);
  const on = rows.filter((r) => ticked(sel, null, r, locked)).length;
  return on === 0 ? "off" : on === rows.length ? "on" : "mixed";
}

export function looseCount(sel, loose, need, locked = NONE) {
  const rows = counted(loose, need, locked);
  return rows.length ? `${rows.filter((r) => ticked(sel, null, r, locked)).length}/${rows.length}` : "";
}

/** Scoped to eligible rows both ways, so the bulk box never adds or removes a greyed row - and to
 *  unlocked ones, so a routed player is not quietly picked for ducking it would lose on unrouting. */
export function toggleLoose(sel, loose, need, locked = NONE) {
  const all = looseState(sel, loose, need, locked) === "on";
  const next = copy(sel);
  for (const r of loose) {
    if (!eligible(r, need) || locked.has(r[0])) continue;
    if (all) next.extra.delete(r[0]);
    else next.extra.add(r[0]);
  }
  return next;
}

export const looseLocked = (sel, loose, need, locked = NONE) =>
  nothingFree(loose, need, locked) &&
  (loose.some((r) => locked.has(r[0])) || looseState(sel, loose, need, locked) === "off");

/** One press of a ± stepper: a step either way, onto the step grid, inside the range. */
export function nudge(value, dir, min, max, step) {
  const s = step > 0 ? step : 1;
  const v = Math.round((value + dir * s - min) / s) * s + min;
  return Math.min(max, Math.max(min, v));
}
