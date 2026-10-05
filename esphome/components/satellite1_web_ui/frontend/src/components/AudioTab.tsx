import React, { useState, useEffect, useRef } from 'react';
import { HINTS, TEXT } from '../copy.js';
import { entity, haSyncOnce, pathFor, post } from '../lib/device.js';
import { tipDone } from '../lib/tips.js';
import { NEED_MEDIA, NEED_VOLUME, areaCount, areaLocked, areaState, eligible, haProblem, hasTargets, isLive, looseCount, looseLocked, looseState, nudge, rowOn, rowWhy, toggleArea, toggleLoose, togglePlayer, treePayload } from '../lib/audio.js';
import { HintBtn } from './bits';
import { MSlider } from './MSlider';
import { Check, Switch } from './controls';
import type { CheckState } from './controls';
import type { Ctx } from '../ctx';

/**
 * Where this device's sound goes. The trees edit the selection the device owns at /api/sat1/sel;
 * the rows under them are plain entities. Each card degrades on its own - a missing Home Assistant
 * empties the trees and says why, while the entity rows keep working, because they are device state
 * that applies the moment Home Assistant comes back.
 *
 * The two switches Home Assistant shows for this, "Route TTS To All Area Players" and "Duck All Area
 * Players", are projections of the selection rather than separate settings: ticking this device's
 * own area in a tree turns the matching switch on, and unticking any single player in it turns the
 * switch off (docs/web-ui.md, "The routing and ducking selection"). The speaker's channel is the one
 * amplifier setting a customer gets, so it lives here; the TAS2780's analog gain and live readings
 * are developer tools on Settings > Developer (owner call, October 2026).
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
/** A row of players under a heading with its own bulk box, for a real area and for "No Area
 *  Assigned": the two behave differently enough to be worth one shared shell and two callers rather
 *  than one component with a mode flag. */
function Group({
  label,
  count,
  state,
  expanded,
  onExpand,
  onBulk,
  disabled,
  children
}: {
  label: string;
  count: string;
  state: CheckState;
  expanded: boolean;
  onExpand: () => void;
  onBulk: () => void;
  disabled?: boolean;
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
        <Check state={state} disabled={disabled} onClick={e => {
        e.stopPropagation();
        onBulk();
      }} label={label} />
        <span className="au-tree-label">{label}</span>
        <span className="au-tree-count">{count}</span>
      </div>
      {expanded && <div className="au-tree-ps">{children}</div>}
    </div>;
}

/**
 * The area -> player tree over one of the selection's two halves. One component for both cards
 * because the two lists must never disagree about what is in an area: both draw from the same
 * `areas` payload, which Home Assistant built from the same walk ducking itself uses. `need` is the
 * capability the tree's call asks of a player (src/lib/audio.js); rows without it, and this device's
 * own player, stay listed but greyed with the reason, because a hole reads as a bug.
 *
 * Local Speaker, when offered, comes first and in the same list as every other place the answer
 * could go: one choice about which speakers speak, not a routing list plus a separate switch that
 * silences this one. "No Area Assigned" is not an edge case - on the test installation 40 of the
 * 104 media players are in no area at all, against 37 that are in one (Cast and AirPlay shadow
 * entities, group helpers like all_sonos, laptops) - and it is why there is no free-text entity id
 * field, which would ask someone to know an id the page can simply show them.
 */
function TargetTree({
  payload,
  sel,
  onSel,
  local,
  onLocal,
  localLevel,
  need
}: {
  payload: Payload;
  sel: Sel;
  onSel: (next: Sel) => void;
  local: boolean | null;
  onLocal?: (v: boolean) => void;
  localLevel?: React.ReactNode;
  need: number;
}) {
  const [open, setOpen] = useState<Record<string, boolean>>({});
  const areas = payload?.areas || [];
  const loose = payload?.loose || [];
  if (local === null && !areas.length && !loose.length) return null;
  const playerRow = (areaId: string | null) => (row: Row4) => {
    const [id, name] = row;
    const ok = eligible(row, need);
    const why = rowWhy(row, need);
    return <div className={`au-tree-p${ok && isLive(row) ? '' : ' au-tree-off'}`} key={id}>
        <Check state={rowOn(sel, areaId, row, need) ? 'on' : 'off'} disabled={!ok} onClick={() => onSel(togglePlayer(sel, areaId, id))} label={name} />
        <span className="au-tree-player-name">{name}</span>
        {why && <span className="au-tree-why">{why}</span>}
      </div>;
  };
  return <div className="au-tree">
      {local !== null && <div className="au-tree-p au-tree-self">
          <Check state={local ? 'on' : 'off'} onClick={() => onLocal && onLocal(!local)} label="Local Speaker" />
          <span className="au-tree-player-name">Local Speaker</span>
          {local && localLevel}
        </div>}
      {areas.map(area => <Group key={area.i} label={area.n} count={areaCount(sel, area, need)} state={areaState(sel, area, need)} disabled={areaLocked(sel, area, need)} expanded={!!open[area.i]} onExpand={() => setOpen({
      ...open,
      [area.i]: !open[area.i]
    })} onBulk={() => onSel(toggleArea(sel, area, need))}>
          {area.p.map(playerRow(area.i))}
        </Group>)}
      {loose.length > 0 && <Group label="No Area Assigned" count={looseCount(sel, loose, need)} state={looseState(sel, loose, need)} disabled={looseLocked(sel, loose, need)} expanded={!!open.__loose} onExpand={() => setOpen({
      ...open,
      __loose: !open.__loose
    })} onBulk={() => onSel(toggleLoose(sel, loose, need))}>
          {loose.map(playerRow(null))}
        </Group>}
    </div>;
}

/**
 * Only the problems: quiet text under the tree title, with "Show fix" on the one a person can fix
 * from here. No age line or Refresh button - the sync runs once per page load, so there is nothing
 * to operate, and a timestamp on a list of speakers answers a question nobody was asking. Renders
 * nothing rather than an empty box, which would leave its margins behind above every tree.
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

/** Voice Override, as the design's stepper on the Local Speaker row. Zero means no override. */
function LocalLevel({
  ctx
}: {
  ctx: Ctx;
}) {
  const e = entity(ctx, 'voice_override');
  const min = Number(e?.min_value ?? 0);
  const max = Number(e?.max_value ?? 100);
  const step = Number(e?.step ?? 5);
  const [shown, hold] = useHeld(numberOf(e), step / 2);
  const set = (dir: number) => {
    const v = nudge(shown, dir, min, max, step);
    if (v === shown) return;
    hold(v);
    send(ctx, 'voice_override', 'set', {
      value: v
    });
  };
  return <span className="au-step">
      <HintBtn text={HINTS.voice_override} />
      <button type="button" className="au-sel-btn" aria-label="Decrease local speaker volume" disabled={shown <= min} onClick={() => set(-1)}>−</button>
      <span className="mslider-val" aria-live="polite">{shown === 0 ? 'auto' : shown}</span>
      <button type="button" className="au-sel-btn" aria-label="Increase local speaker volume" disabled={shown >= max} onClick={() => set(1)}>+</button>
    </span>;
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
 * Anything chosen means a response goes somewhere besides this speaker - the device's own test,
 * ${tts_routing_active} in tts_routing.yaml - so the rows that only apply to remote players stand
 * disabled while nothing is. There is no master switch, so nothing can be on with an empty list.
 *
 * One volume slider has three mechanisms behind it: Sonos reads the level off the announcement,
 * another Satellite1 has its Voice Override set and put back, and everything else has its media
 * volume set and restored. Deliberately not a per-target table: which one applies depends on Sonos
 * membership and on the target's own override value, neither of which is in the payload, so a table
 * would be a confident guess per row. Zero is hands-off - tts_routing.yaml treats it as "leave every
 * target's volume alone" - and reads as such. The mic guard comes last because it is about what
 * happens after playback rather than what plays.
 *
 * The entities keep their legacy TTS names whatever the labels say, because renaming an ESPHome
 * entity orphans it; docs/TTS-Routing.md carries the compatibility note.
 */
function RemoteRouting({
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
  const guard = entity(ctx, 'remote_sync_guard');
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Voice Response Routing</span><HintBtn text={HINTS.remote_routing} /></div>
      <p className="au-tree-title">Route the assistant voice response to selected speakers</p>
      <HaState problem={problem} payload={payload} onFix={ctx.onShowFix} />
      <TargetTree payload={payload} sel={sel.routing} onSel={next => ctx.selWrite({
      ...sel,
      routing: next
    })} local={sel.local} onLocal={v => ctx.selWrite({
      ...sel,
      local: v
    })} localLevel={has(ctx, 'voice_override') && <LocalLevel ctx={ctx} />} need={NEED_MEDIA} />
      {has(ctx, 'remote_tts_volume') && <div className="au-row">
          <div className="au-row-label"><span>Remote Speaker Volume</span><HintBtn text={HINTS.remote_tts_volume} /></div>
          <NumberSlider ctx={ctx} k="remote_tts_volume" label="Remote speaker volume" disabled={!active} format={v => v === 0 ? 'Use Remote Volume' : `${Math.round(v)}%`} />
        </div>}
      {has(ctx, 'remote_wake_chime') && <div className="au-row">
          <div className="au-row-label"><span>Remote wake chime</span><HintBtn text={HINTS.remote_wake_chime} /></div>
          <Switch on={switchOn(entity(ctx, 'remote_wake_chime'))} disabled={!active} label="Remote wake chime" onChange={v => send(ctx, 'remote_wake_chime', v ? 'turn_on' : 'turn_off')} />
        </div>}
      {has(ctx, 'remote_timer_ring') && <div className="au-row">
          <div className="au-row-label"><span>Remote timer ring</span><HintBtn text={HINTS.remote_timer_ring} /></div>
          <Switch on={switchOn(entity(ctx, 'remote_timer_ring'))} disabled={!active} label="Remote timer ring" onChange={v => send(ctx, 'remote_timer_ring', v ? 'turn_on' : 'turn_off')} />
        </div>}
      {guard && <div className="au-row au-row-last">
          <div className="au-row-label"><span>Remote mic guard</span><HintBtn text={HINTS.remote_sync_guard} /></div>
          <AuSelect value={guard.value} options={guard.option || []} disabled={!active} label="Remote mic guard" onChange={v => send(ctx, 'remote_sync_guard', 'set', {
          option: v
        })} />
        </div>}
    </div>;
}

/**
 * Every area in the house is offered, not only this device's own - ducking a room this device is
 * not in is a deliberate capability, not a side effect - and there is no Local Speaker row: this
 * device's level while it talks is Voice Override's, and its own player is greyed anyway.
 *
 * The duck starts at the wake word and holds until the answer ends, so players are quiet while the
 * device listens too. Zero on Duck volume is literal, unlike Remote Speaker Volume and Voice
 * Override: area_ducking.yaml sets ducked players to 0%, silencing them for the length of the
 * interaction, so the readout says "mute" rather than implying hands-off.
 */
function AreaDucking({
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
  const active = hasTargets(sel.duck);
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Area Ducking</span><HintBtn text={HINTS.area_ducking} /></div>
      <p className="au-tree-title">Lower the volume on selected players upon wake word detection</p>
      <HaState problem={problem} payload={payload} onFix={ctx.onShowFix} />
      <TargetTree payload={payload} sel={sel.duck} onSel={next => ctx.selWrite({
      ...sel,
      duck: next
    })} local={null} need={NEED_VOLUME} />
      {has(ctx, 'duck_volume') && <div className="au-row au-row-last">
          <div className="au-row-label"><span>Duck volume</span><HintBtn text={HINTS.duck_volume} /></div>
          <NumberSlider ctx={ctx} k="duck_volume" label="Duck volume" disabled={!active} format={v => v === 0 ? 'mute' : `${Math.round(v)}%`} />
        </div>}
    </div>;
}

/** Which side of a stereo source the built-in speaker plays. Needs no Home Assistant, so it shows
 *  even when the selection cards above cannot. */
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
  // The selection gates both cards. A missing one is reported apart from a missing device: the shell
  // is up, so it is the selection specifically.
  return <section className="control au-tab au-route">
      <span className="eyebrow">AUDIO · ROUTING</span>
      <h1><span>Sound, </span><em>directed.</em></h1>
      {!ctx.device ? <p className="au-missing">The device is not available on this firmware build.</p> : <>
          {!sel && <p className="au-missing">{ctx.selError ? 'The saved selection' : 'Settings'} is not available on this firmware build.</p>}
          <div className="au-route-cards">
            {sel && <RemoteRouting ctx={ctx} sel={sel} problem={problem} payload={payload} />}
            {sel && <AreaDucking ctx={ctx} sel={sel} problem={problem} payload={payload} />}
            {has(ctx, 'speaker_channel') && <SpeakerChannel ctx={ctx} />}
          </div>
        </>}
    </section>;
}
