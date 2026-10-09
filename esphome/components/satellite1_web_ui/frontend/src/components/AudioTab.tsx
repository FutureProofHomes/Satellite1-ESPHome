import React, { useState, useEffect, useRef } from 'react';
import { HINTS, TEXT } from '../copy.js';
import { entity, haSyncOnce, pathFor, post } from '../lib/device.js';
import { tipDone } from '../lib/tips.js';
import { NEED_MEDIA, NEED_VOLUME, areaCount, areaLocked, areaState, eligible, haProblem, hasTargets, heldBits, isLive, isSelf, looseCount, looseLocked, looseState, routedLocks, rowOn, rowWhy, selfGroup, toggleArea, toggleLoose, togglePlayer, treePayload } from '../lib/audio.js';
import { HintBtn } from './bits';
import { MSlider } from './MSlider';
import { Check, Switch } from './controls';
import type { CheckState } from './controls';
import type { Ctx } from '../ctx';

/**
 * Where this device's sound goes. The speaker list edits the selection the device owns at
 * /api/sat1/sel; the levels above it and the rows under it are plain entities. The card degrades in
 * parts - a missing Home Assistant empties the list and says why, while the entity rows keep
 * working, because they are device state that applies the moment Home Assistant comes back.
 *
 * The two switches Home Assistant shows for this, "Route Announcements To All Area Players" and
 * "Duck All Area Players", are projections of the selection rather than separate settings: ticking
 * this device's own area's Announce or Duck box turns the matching switch on, and unticking any
 * single player in it turns the switch off (docs/web-ui.md, "The routing and ducking selection").
 * The speaker's channel is the one amplifier setting a customer gets, so it lives here; the
 * TAS2780's analog gain and live readings are developer tools on Settings > Developer (owner call,
 * October 2026).
 */

/** [entity id, name, caps, available] - the /api/sat1/ha row. */
type Row4 = [string, string, number?, number?];
type Area = { i: string; n: string; p: Row4[] };
type Payload = { areas?: Area[]; loose?: Row4[]; t?: number } | null;
type Sel = {
  areas: Set<string>;
  extra: Set<string>;
  excluded: Set<string>;
};
type Selection = { local: boolean; area: string; routing: Sel; duck: Sel };
type Problem = { text: string; fix?: boolean; soft?: boolean } | null;

const has = (ctx: Ctx, key: string) => !!ctx.device?.e?.[key];
const switchOn = (e: any) => !!e && (e.value === true || e.state === 'ON' || e.state === 'on');
const numberOf = (e: any) => e ? Number(e.value ?? e.state) : 0;
/** A failed write is already toasted by src/lib/device.js; the catch only keeps a dropped socket
 *  quiet. */
const send = (ctx: Ctx, key: string, action: string, query?: Record<string, string | number>) => {
  const p = pathFor(ctx, key, action, query);
  if (p) post(p).catch(() => {});
};

/**
 * A just-written number, shown over the stale reads that follow it until the device's echo reaches
 * it (within `tol`) or five seconds pass (a refused write, where the stale value is the truth).
 * Without it the control snaps back to the old value on release and jumps forward when the echo
 * lands. State rather than a ref, because MSlider renders its `value` prop the moment it drops its
 * draft.
 */
function useHeld(value: number, tol: number): [number, (v: number) => void] {
  const [held, setHeld] = useState<{ v: number; at: number } | null>(null);
  const arrived = held !== null && Math.abs(value - held.v) <= tol;
  useEffect(() => {
    if (!held) return undefined;
    if (arrived) {
      setHeld(null);
      return undefined;
    }
    const t = setTimeout(() => setHeld(null), Math.max(0, held.at + 5000 - Date.now()));
    return () => clearTimeout(t);
  }, [held, arrived]);
  return [held && !arrived ? held.v : value, (v: number) => setHeld({ v, at: Date.now() })];
}

function AuSelect({
  value,
  options,
  disabled,
  label,
  onChange
}: {
  value: string;
  options: string[];
  disabled?: boolean;
  label: string;
  onChange: (v: string) => void;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const close = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('mousedown', close);
    document.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('mousedown', close);
      document.removeEventListener('keydown', onKey);
    };
  }, [open]);
  return <div ref={ref} className={`au-sel${disabled ? ' disabled' : ''}`}>
      <button className={`au-sel-btn${open ? ' open' : ''}`} disabled={disabled} aria-label={label} aria-haspopup="listbox" aria-expanded={open} onClick={() => setOpen(v => !v)}>
        <span>{value}</span>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
          <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
      </button>
      {open && <div className="au-sel-pop" role="listbox" aria-label={label}>
          {options.map(o => <button key={o} role="option" aria-selected={o === value} className={`au-sel-opt${o === value ? ' active' : ''}`} onClick={() => {
        if (o !== value) onChange(o);
        setOpen(false);
      }}>
              {o === value && <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true"><path d="M2 6l3 3 5-5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>}
              <span>{o}</span>
            </button>)}
        </div>}
    </div>;
}

/** The end of every row: one cell under the Announce heading, one under Duck. The same two fixed
 *  cells on the headings, the rooms and the players are what keep the columns straight whatever
 *  the names beside them do. */
function Boxes({
  announce,
  duck
}: {
  announce: React.ReactNode;
  duck: React.ReactNode;
}) {
  return <span className="au-boxes">
      <span className="au-cell">{announce}</span>
      <span className="au-cell">{duck}</span>
    </span>;
}

/** A room, or "No Area Assigned", as a heading with a bulk box per column and its players under
 *  it. The two behave differently enough to be worth one shared shell and two callers rather than
 *  one component with a mode flag. */
function Group({
  label,
  summary,
  expanded,
  onExpand,
  announce,
  duck,
  children
}: {
  label: string;
  summary: string;
  expanded: boolean;
  onExpand: () => void;
  announce: React.ReactNode;
  duck: React.ReactNode;
  children?: React.ReactNode;
}) {
  return <div className="au-tree-a">
      <div className="au-tree-h" onClick={onExpand}>
        <button className="au-tree-caret" aria-label={expanded ? `Collapse ${label}` : `Expand ${label}`} aria-expanded={expanded} onClick={e => {
        e.stopPropagation();
        onExpand();
      }}>
          <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
            <path d="M4 2l4 4-4 4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        </button>
        <span className="au-tree-text">
          <span className="au-tree-label">{label}</span>
          {summary && <span className="au-tree-count">{summary}</span>}
        </span>
        <Boxes announce={announce} duck={duck} />
      </div>
      {expanded && <div className="au-tree-ps">{children}</div>}
    </div>;
}

/** The line under a group's name: each column's count, leaving out a column with nothing in the
 *  group to count - a room of TVs that take no volume says nothing about Duck. */
const summary = (announce: string, duck: string) => [announce && `${TEXT.col_announce} ${announce}`, duck && `${TEXT.col_duck} ${duck}`].filter(Boolean).join(' · ');

/**
 * Every speaker Home Assistant knows, by room, with an Announce box and a Duck box on each. The two
 * columns are the selection's two halves, written separately, so each box changes exactly one
 * setting, and the firmware's two lists - and the two switches Home Assistant derives from them -
 * are what they always were. Both columns draw from the one `areas` payload, which Home Assistant
 * built from the same walk ducking itself uses. A box the player cannot use stays greyed, with the
 * reason under its name, because a hole reads as a bug. "No Area Assigned" is not an edge case -
 * on the test installation 40 of the 104 media players are in no area at all, against 37 that are
 * in one (Cast and AirPlay shadow entities, group helpers like all_sonos, laptops) - and it is why
 * there is no free-text entity id field, which would ask someone to know an id the page can simply
 * show them.
 *
 * `locked` is routedLocks' ids: speakers ticked to Announce that the firmware ducks whatever the
 * duck selection says, so their Duck box shows ticked and disabled - one that could be unticked
 * would be a false promise. The duck selection itself is left alone, so unticking Announce puts the
 * Duck box back to what it was. A Satellite1 on older firmware is not ducked that way, so its Duck
 * box stays free and its row says why.
 *
 * This device's own row carries the "Announce on this device" setting in its Announce box, ticked
 * and greyed while nothing else is ticked to Announce, because the device answers itself then. Its
 * group opens by itself so the box is found without hunting. When the row is missing - no payload
 * yet, or one cut short - the setting stands as a switch at the top of the list instead.
 */
function SpeakerList({
  ctx,
  sel,
  payload,
  active,
  locked
}: {
  ctx: Ctx;
  sel: Selection;
  payload: Payload;
  active: boolean;
  locked: Set<string>;
}) {
  const [open, setOpen] = useState<Record<string, boolean>>({});
  const areas = payload?.areas || [];
  const loose = payload?.loose || [];
  const home = selfGroup(payload);
  const bits = heldBits(payload);
  const here = !active || sel.local;
  const setRouting = (next: Sel) => ctx.selWrite({ ...sel, routing: next });
  const setDuck = (next: Sel) => ctx.selWrite({ ...sel, duck: next });
  const setLocal = (local: boolean) => ctx.selWrite({ ...sel, local });
  const expanded = (key: string, group: string | null) => open[key] ?? group === home;
  const flip = (key: string, group: string | null) => setOpen({
    ...open,
    [key]: !expanded(key, group)
  });
  const bulk = (state: CheckState, disabled: boolean, label: string, act: () => void) => <Check state={state} disabled={disabled} label={label} onClick={e => {
    e.stopPropagation();
    act();
  }} />;
  const selfRow = (row: Row4) => {
    const [id, name] = row;
    const doing = !active ? TEXT.self_always : sel.local ? TEXT.self_too : TEXT.self_remote_only;
    return <div className={`au-tree-p${active ? '' : ' au-tree-off'}`} key={id}>
        <span className="au-tree-text">
          <span className="au-tree-player-name">{name}</span>
          <span className="au-tree-why">{`${TEXT.cap_self} · ${doing}`}</span>
        </span>
        <Boxes announce={<Check state={here ? 'on' : 'off'} disabled={!active} label={TEXT.local_label} onClick={() => setLocal(!sel.local)} />} duck={<Check state="off" disabled label={`${name}: ${TEXT.col_duck}`} onClick={() => {}} />} />
      </div>;
  };
  const playerRow = (group: string | null) => (row: Row4) => {
    if (isSelf(row)) return selfRow(row);
    const [id, name] = row;
    const held = locked.has(id);
    const canAnnounce = eligible(row, NEED_MEDIA);
    const canDuck = eligible(row, NEED_VOLUME);
    const why = rowWhy(row, held, bits);
    const off = !isLive(row) || !canAnnounce && !canDuck;
    return <div className={`au-tree-p${off ? ' au-tree-off' : ''}`} key={id}>
        <span className="au-tree-text">
          <span className="au-tree-player-name">{name}</span>
          {why && <span className="au-tree-why">{why}</span>}
        </span>
        <Boxes announce={<Check state={rowOn(sel.routing, group, row, NEED_MEDIA) ? 'on' : 'off'} disabled={!canAnnounce} label={`${name}: ${TEXT.col_announce}`} onClick={() => setRouting(togglePlayer(sel.routing, group, id))} />} duck={<Check state={held || rowOn(sel.duck, group, row, NEED_VOLUME) ? 'on' : 'off'} disabled={!canDuck || held} label={`${name}: ${held ? TEXT.duck_locked : TEXT.col_duck}`} onClick={() => setDuck(togglePlayer(sel.duck, group, id))} />} />
      </div>;
  };
  return <div className="au-tree au-list">
      {home === undefined && <div className="au-tree-row">
          <div className="au-row-label"><span>{TEXT.local_label}</span><HintBtn text={HINTS.local_speaker} /></div>
          <Switch on={here} disabled={!active} label={TEXT.local_label} onChange={setLocal} />
        </div>}
      {(areas.length > 0 || loose.length > 0) && <div className="au-cols">
          <span className="au-cols-name"><span>{TEXT.col_speaker}</span><HintBtn text={HINTS.speaker_columns} /></span>
          <Boxes announce={TEXT.col_announce} duck={TEXT.col_duck} />
        </div>}
      {areas.map(area => <Group key={area.i} label={area.n} summary={summary(areaCount(sel.routing, area, NEED_MEDIA), areaCount(sel.duck, area, NEED_VOLUME, locked))} expanded={expanded(area.i, area.i)} onExpand={() => flip(area.i, area.i)} announce={bulk(areaState(sel.routing, area, NEED_MEDIA), areaLocked(sel.routing, area, NEED_MEDIA), `${area.n}: ${TEXT.col_announce}`, () => setRouting(toggleArea(sel.routing, area, NEED_MEDIA)))} duck={bulk(areaState(sel.duck, area, NEED_VOLUME, locked), areaLocked(sel.duck, area, NEED_VOLUME, locked), `${area.n}: ${TEXT.col_duck}`, () => setDuck(toggleArea(sel.duck, area, NEED_VOLUME, locked)))}>
          {area.p.map(playerRow(area.i))}
        </Group>)}
      {loose.length > 0 && <Group label="No Area Assigned" summary={summary(looseCount(sel.routing, loose, NEED_MEDIA), looseCount(sel.duck, loose, NEED_VOLUME, locked))} expanded={expanded('__loose', null)} onExpand={() => flip('__loose', null)} announce={bulk(looseState(sel.routing, loose, NEED_MEDIA), looseLocked(sel.routing, loose, NEED_MEDIA), `No Area Assigned: ${TEXT.col_announce}`, () => setRouting(toggleLoose(sel.routing, loose, NEED_MEDIA)))} duck={bulk(looseState(sel.duck, loose, NEED_VOLUME, locked), looseLocked(sel.duck, loose, NEED_VOLUME, locked), `No Area Assigned: ${TEXT.col_duck}`, () => setDuck(toggleLoose(sel.duck, loose, NEED_VOLUME, locked)))}>
          {loose.map(playerRow(null))}
        </Group>}
    </div>;
}

/**
 * Only the problems: quiet text above the speaker list, with "Show fix" on the one a person can fix
 * from here. No age line or Refresh button - the sync runs once per page load, so there is nothing
 * to operate, and a timestamp on a list of speakers answers a question nobody was asking. Renders
 * nothing rather than an empty box, which would leave its margins behind above the list.
 */
function HaState({
  problem,
  payload,
  onFix
}: {
  problem: Problem;
  payload: Payload;
  onFix: () => void;
}) {
  const truncated = payload?.t === 1;
  if (!problem && !truncated) return null;
  return <div className="au-ha-state">
      {problem && <p>
          {problem.text}
          {problem.fix && <> <button type="button" className="au-fix" onClick={onFix}>{TEXT.show_fix}</button></>}
        </p>}
      {truncated && <p>{TEXT.ha_truncated}</p>}
    </div>;
}

/** A number entity on the design's slider, written on release. */
function NumberSlider({
  ctx,
  k,
  label,
  format,
  disabled
}: {
  ctx: Ctx;
  k: string;
  label: string;
  format: (v: number) => string;
  disabled: boolean;
}) {
  const e = entity(ctx, k);
  const step = Number(e?.step ?? 1);
  const [shown, hold] = useHeld(numberOf(e), step / 2);
  return <MSlider value={shown} min={Number(e?.min_value ?? 0)} max={Number(e?.max_value ?? 100)} step={step} disabled={disabled} ariaLabel={label} format={format} onCommit={v => {
    if (v === shown) return;
    hold(v);
    send(ctx, k, 'set', {
      value: v
    });
  }} />;
}

/**
 * A level: label and reading on one line, the slider full width under them. The 0 readings are
 * whole phrases, so when the two do not fit side by side the reading drops under the label rather
 * than squeezing it (styles/audio.css). Greyed as a whole when disabled; the hint stays usable,
 * since a greyed control is exactly when someone asks what it does.
 */
function VolumeHead({
  ctx,
  k,
  label,
  hint,
  zero,
  disabled = false
}: {
  ctx: Ctx;
  k: string;
  label: string;
  hint: string;
  zero: string;
  disabled?: boolean;
}) {
  return <div className={`au-vol${disabled ? ' disabled' : ''}`}>
      <span className="au-vol-label"><span>{label}</span><HintBtn text={hint} /></span>
      <NumberSlider ctx={ctx} k={k} label={label} disabled={disabled} format={v => v === 0 ? zero : `${Math.round(v)}%`} />
    </div>;
}

/**
 * Where this device's sound goes, in one card: its three levels, then the speakers, then what else
 * follows a response to the speakers ticked to Announce. Anything ticked to Announce means a
 * response goes somewhere besides this speaker - the device's own test, ${tts_routing_active} in
 * tts_routing.yaml - so what only applies to remote speakers stands disabled while nothing is.
 * There is no master switch, so nothing can be on with an empty list.
 *
 * Announcement Volume is this speaker's own, which covers far more than routed replies, so it never
 * greys. Remote Announcement Volume is asked of every speaker ticked to Announce, with three
 * mechanisms behind it: Sonos reads the level off the announcement, another Satellite1 has its
 * Announcement Volume set and put back, and everything else has its media volume set and restored.
 * Deliberately not a per-target table: which one applies depends on Sonos membership, which is not
 * in the payload, so a table would be a confident guess per row. Its zero is hands-off -
 * tts_routing.yaml treats it as "leave every target's volume alone". Remote Ducking Volume's zero is
 * literal - area_ducking.yaml sets ducked players to 0% for the length of the interaction - so it
 * reads "Mute playback". It stays live with nothing ticked to Duck while a speaker ticked to
 * Announce is turned down to it; a routed Sonos or Satellite1 lowers its own music instead, so
 * those alone leave it greyed.
 *
 * Every label is the entity's own name in Home Assistant, so the two never disagree about what a
 * setting is called. The echo guard comes last because it is about what happens after playback
 * rather than what plays.
 */
function AudioRouting({
  ctx,
  sel,
  problem,
  payload
}: {
  ctx: Ctx;
  sel: Selection;
  problem: Problem;
  payload: Payload;
}) {
  const active = hasTargets(sel.routing);
  const locks = routedLocks(payload, sel.routing);
  const ducking = hasTargets(sel.duck) || locks.level;
  const guard = entity(ctx, 'remote_sync_guard');
  const chime = has(ctx, 'remote_wake_chime');
  const ring = has(ctx, 'remote_timer_ring');
  const levels = has(ctx, 'announcement_volume') || has(ctx, 'remote_announcement_volume') || has(ctx, 'duck_volume');
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Audio Routing</span><HintBtn text={HINTS.audio_routing} /></div>
      <p className="au-tree-title">Route assistant responses, announcements, timer rings and wake chimes to selected speakers</p>
      {levels && <div className="au-vols">
          {has(ctx, 'announcement_volume') && <VolumeHead ctx={ctx} k="announcement_volume" label="Announcement Volume" hint={HINTS.announcement_volume} zero="Follow speaker volume" />}
          {has(ctx, 'remote_announcement_volume') && <VolumeHead ctx={ctx} k="remote_announcement_volume" label="Remote Announcement Volume" hint={HINTS.remote_announcement_volume} zero="Follow speaker volume" disabled={!active} />}
          {has(ctx, 'duck_volume') && <VolumeHead ctx={ctx} k="duck_volume" label="Remote Ducking Volume" hint={HINTS.duck_volume} zero="Mute playback" disabled={!ducking} />}
        </div>}
      <HaState problem={problem} payload={payload} onFix={ctx.onShowFix} />
      <SpeakerList ctx={ctx} sel={sel} payload={payload} active={active} locked={locks.ids} />
      {(chime || ring || guard) && <p className="au-sub">{TEXT.for_announce}</p>}
      {chime && <div className="au-row">
          <div className="au-row-label"><span>Remote Wake Chime</span><HintBtn text={HINTS.remote_wake_chime} /></div>
          <Switch on={switchOn(entity(ctx, 'remote_wake_chime'))} disabled={!active} label="Remote Wake Chime" onChange={v => send(ctx, 'remote_wake_chime', v ? 'turn_on' : 'turn_off')} />
        </div>}
      {ring && <div className="au-row">
          <div className="au-row-label"><span>Remote Timer Ring</span><HintBtn text={HINTS.remote_timer_ring} /></div>
          <Switch on={switchOn(entity(ctx, 'remote_timer_ring'))} disabled={!active} label="Remote Timer Ring" onChange={v => send(ctx, 'remote_timer_ring', v ? 'turn_on' : 'turn_off')} />
        </div>}
      {guard && <div className="au-row au-row-last">
          <div className="au-row-label"><span>Remote Echo Guard</span><HintBtn text={HINTS.remote_sync_guard} /></div>
          <AuSelect value={guard.value} options={guard.option || []} disabled={!active} label="Remote Echo Guard" onChange={v => send(ctx, 'remote_sync_guard', 'set', {
          option: v
        })} />
        </div>}
    </div>;
}

/** Which side of a stereo source the built-in speaker plays. Needs no Home Assistant, so it shows
 *  even when the routing card above cannot. */
function SpeakerChannel({
  ctx
}: {
  ctx: Ctx;
}) {
  const chan = entity(ctx, 'speaker_channel');
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Speaker</span></div>
      <div className="au-row au-row-last">
        <div className="au-row-label"><span>Channel</span><HintBtn text={HINTS.speaker_channel} /></div>
        <AuSelect value={chan?.value ?? ''} options={chan?.option || []} label="Speaker channel" onChange={v => send(ctx, 'speaker_channel', 'set', {
        option: v
      })} />
      </div>
    </div>;
}
export function AudioTab({
  ctx
}: {
  ctx: Ctx;
}) {
  // Once per page load, shared with every other tab that reads the speaker list. GET /api/sat1/ha is
  // the device's cached copy; this is what asks Home Assistant for a fresh one.
  useEffect(() => {
    haSyncOnce(ctx.haRefresh);
    tipDone('route');
    // haSyncOnce guards itself, so the effect runs once by design.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const problem: Problem = haProblem(ctx.ha, ctx.device?.ha);
  const payload: Payload = treePayload(ctx.ha, problem);
  const sel: Selection | null = ctx.sel;
  // The selection gates the routing card. A missing one is reported apart from a missing device:
  // the shell is up, so it is the selection specifically.
  return <section className="control au-tab au-route">
      <span className="eyebrow">AUDIO · ROUTING</span>
      <h1><span>Sound, </span><em>directed.</em></h1>
      {!ctx.device ? <p className="au-missing">The device is not available on this firmware build.</p> : <>
          {!sel && <p className="au-missing">{ctx.selError ? 'The saved selection' : 'Settings'} is not available on this firmware build.</p>}
          <div className="au-route-cards">
            {sel && <AudioRouting ctx={ctx} sel={sel} problem={problem} payload={payload} />}
            {has(ctx, 'speaker_channel') && <SpeakerChannel ctx={ctx} />}
          </div>
        </>}
    </section>;
}
