import React, { useState, useEffect, useRef } from 'react';
import { HINTS, TEXT } from '../../src/copy.js';
import { entity, haSyncOnce, pathFor, post } from '../../src/lib/device.js';
import { NEED_MEDIA, NEED_VOLUME, areaCount, areaLocked, areaState, eligible, haProblem, hasTargets, isLive, looseCount, looseLocked, looseState, nudge, rowOn, rowWhy, toggleArea, toggleLoose, togglePlayer, treePayload } from '../lib/audio.js';
import { HintBtn } from './bits';
import { MSlider } from './MSlider';
import type { Ctx } from '../ctx';

/**
 * Where this device's sound goes. The trees edit the selection the device owns at /api/sat1/sel;
 * the rows under them are plain entities. Each card degrades on its own - a missing Home Assistant
 * empties the trees and says why, while the entity rows keep working, because they are device state.
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
type CheckState = 'on' | 'off' | 'mixed';

/** v1's hints for these two name the page each used to sit on; here they share a card. */
const HINT = {
  voice_override: 'How loud this device speaks when the assistant replies, separate from media volume. Zero follows the media volume instead. Speakers you route answers to have their own level - Remote Speaker Volume, below.',
  remote_tts_volume: "How loud answers are on the remote speakers. This device's own level is the one on the Local Speaker row. Sonos reads the level from the announcement itself; anything else has its volume set for the answer and put back afterwards."
};

const has = (ctx: Ctx, key: string) => !!ctx.device?.e?.[key];
const switchOn = (e: any) => !!e && (e.value === true || e.state === 'ON' || e.state === 'on');
const numberOf = (e: any) => e ? Number(e.value ?? e.state) : 0;
/** A failed write is already toasted by device.js; the catch only keeps a dropped socket quiet. */
const send = (ctx: Ctx, key: string, action: string, query?: Record<string, string | number>) => {
  const p = pathFor(ctx, key, action, query);
  if (p) post(p).catch(() => {});
};

/**
 * A just-written number, shown over the stale reads that follow it until the device's echo reaches
 * it or five seconds pass (a refused write, where the stale value is the truth) - v1's useHeld, as
 * state rather than a ref, because MSlider renders its `value` prop the moment it drops its draft.
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

function AuToggle({
  checked,
  disabled,
  label,
  onChange
}: {
  checked: boolean;
  disabled?: boolean;
  label: string;
  onChange: (v: boolean) => void;
}) {
  return <button role="switch" aria-checked={checked} aria-label={label} disabled={disabled} onClick={() => onChange(!checked)} className={`au-toggle${checked ? ' on' : ''}${disabled ? ' disabled' : ''}`}>
      <span className="au-toggle-thumb" />
    </button>;
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
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true" style={{
        transform: open ? 'rotate(180deg)' : undefined,
        transition: 'transform .2s'
      }}>
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
function AuCheck({
  state,
  disabled,
  onClick,
  label
}: {
  state: CheckState;
  disabled?: boolean;
  onClick: (e: React.MouseEvent<HTMLButtonElement>) => void;
  label: string;
}) {
  return <button role="checkbox" aria-checked={state === 'mixed' ? 'mixed' : state === 'on'} aria-label={label} disabled={disabled} onClick={onClick} className={`au-check au-check-${state}${disabled ? ' disabled' : ''}`} style={{
    width: 44,
    height: 44,
    minWidth: 44,
    minHeight: 44,
    background: 'transparent',
    border: 'none',
    borderRadius: 0,
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    padding: 0
  }}>
      <span className={`au-check au-check-${state}`} style={{
      width: 22,
      height: 22,
      minWidth: 22,
      minHeight: 22,
      pointerEvents: 'none'
    }}>
        {state === 'on' && <svg width="12" height="12" viewBox="0 0 10 10"><path d="M1.5 5l2.5 2.5 4.5-4.5" stroke="white" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" fill="none" /></svg>}
        {state === 'mixed' && <span className="au-check-dash" />}
      </span>
    </button>;
}
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
      <div className="au-tree-h" onClick={onExpand} style={{
      minHeight: 48,
      cursor: 'pointer'
    }}>
        <button className="au-tree-caret" aria-label={expanded ? `Collapse ${label}` : `Expand ${label}`} aria-expanded={expanded} onClick={e => {
        e.stopPropagation();
        onExpand();
      }}>
          <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true" style={{
          transform: expanded ? 'rotate(90deg)' : undefined,
          transition: 'transform .2s'
        }}>
            <path d="M4 2l4 4-4 4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
          </svg>
        </button>
        <AuCheck state={state} disabled={disabled} onClick={e => {
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
 * The area -> player tree over one of the selection's two halves. `need` is the capability the
 * tree's call asks of a player (lib/audio.js); rows without it, and this device's own player, stay
 * listed but greyed with the reason, because a hole reads as a bug.
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
    return <div className={`au-tree-p${ok && isLive(row) ? '' : ' au-tree-off'}`} key={id} style={{
      minHeight: 44,
      paddingTop: 10,
      paddingBottom: 10
    }}>
        <AuCheck state={rowOn(sel, areaId, row, need) ? 'on' : 'off'} disabled={!ok} onClick={() => onSel(togglePlayer(sel, areaId, id))} label={name} />
        <span className="au-tree-player-name">{name}</span>
        {why && <span className="au-tree-why">{why}</span>}
      </div>;
  };
  return <div className="au-tree">
      {local !== null && <div className="au-tree-p au-tree-self">
          <AuCheck state={local ? 'on' : 'off'} onClick={() => onLocal && onLocal(!local)} label="Local Speaker" />
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

/** Only the problems: quiet text under the tree title, with "Show fix" on the one a person can fix
 *  from here. */
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
  return <span style={{
    display: 'flex',
    alignItems: 'center',
    gap: 6
  }}>
      <HintBtn text={HINT.voice_override} />
      <button type="button" className="au-sel-btn" aria-label="Decrease local speaker volume" style={{
      width: 36,
      minHeight: 36,
      padding: 0,
      justifyContent: 'center'
    }} disabled={shown <= min} onClick={() => set(-1)}>−</button>
      <span className="mslider-val" aria-live="polite" style={{
      minWidth: 30,
      textAlign: 'center'
    }}>{shown === 0 ? 'auto' : shown}</span>
      <button type="button" className="au-sel-btn" aria-label="Increase local speaker volume" style={{
      width: 36,
      minHeight: 36,
      padding: 0,
      justifyContent: 'center'
    }} disabled={shown >= max} onClick={() => set(1)}>+</button>
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
 * Anything chosen means a response goes somewhere besides this speaker - the device's own test - so
 * the rows that only apply to remote players stand disabled while nothing is.
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
          <div className="au-row-label"><span>Remote Speaker Volume</span><HintBtn text={HINT.remote_tts_volume} /></div>
          <NumberSlider ctx={ctx} k="remote_tts_volume" label="Remote speaker volume" disabled={!active} format={v => v === 0 ? 'Use Remote Volume' : `${Math.round(v)}%`} />
        </div>}
      {has(ctx, 'remote_wake_chime') && <div className="au-row">
          <div className="au-row-label"><span>Remote wake chime</span><HintBtn text={HINTS.remote_wake_chime} /></div>
          <AuToggle checked={switchOn(entity(ctx, 'remote_wake_chime'))} disabled={!active} label="Remote wake chime" onChange={v => send(ctx, 'remote_wake_chime', v ? 'turn_on' : 'turn_off')} />
        </div>}
      {has(ctx, 'remote_timer_ring') && <div className="au-row">
          <div className="au-row-label"><span>Remote timer ring</span><HintBtn text={HINTS.remote_timer_ring} /></div>
          <AuToggle checked={switchOn(entity(ctx, 'remote_timer_ring'))} disabled={!active} label="Remote timer ring" onChange={v => send(ctx, 'remote_timer_ring', v ? 'turn_on' : 'turn_off')} />
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
 * Every area in the house is offered, not only this device's own, and there is no Local Speaker
 * row: this device's level while it talks is Voice Override's, and its own player is greyed anyway.
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
export function AudioTab({
  ctx
}: {
  ctx: Ctx;
}) {
  // Once per page load, shared with every other tab that reads the speaker list.
  useEffect(() => {
    haSyncOnce(ctx.haRefresh);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const problem: Problem = haProblem(ctx.ha, ctx.device?.ha);
  const payload: Payload = treePayload(ctx.ha, problem);
  const sel: Selection | null = ctx.sel;
  return <section className="control au-tab au-route">
      <span className="eyebrow">AUDIO · ROUTING</span>
      <h1><span>Sound, </span><em>directed.</em></h1>
      {!ctx.device ? <p className="au-missing">The device is not available on this firmware build.</p> : sel ? <div className="au-route-cards">
          <RemoteRouting ctx={ctx} sel={sel} problem={problem} payload={payload} />
          <AreaDucking ctx={ctx} sel={sel} problem={problem} payload={payload} />
        </div> : <p className="au-missing">{ctx.selError ? 'The saved selection' : 'Settings'} is not available on this firmware build.</p>}
    </section>;
}
