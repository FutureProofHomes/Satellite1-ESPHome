import React, { useState, useEffect, useLayoutEffect, useRef } from 'react';
import ReactDOM from 'react-dom';
import { MSlider } from './MSlider';
import type { Ctx } from '../ctx';
type Row4 = [string, string, number, number];
const HA_AREAS = [{
  i: 'living_room',
  n: 'Living Room',
  p: [['media_player.living_room_sonos', 'Living Room Sonos', 3, 1], ['media_player.satellite1_a4c2f8', 'Living Room Satellite', 4, 1], ['media_player.tv_living_room', 'Living Room TV', 2, 1]] as Row4[]
}, {
  i: 'kitchen',
  n: 'Kitchen',
  p: [['media_player.kitchen_display', 'Kitchen Display', 3, 1], ['media_player.satellite1_kitchen', 'Kitchen Satellite', 3, 1]] as Row4[]
}, {
  i: 'office',
  n: 'Office',
  p: [['media_player.office_homepod', 'Office HomePod', 1, 0], ['media_player.satellite1_office', 'Office Satellite', 3, 1]] as Row4[]
}];
const HA_LOOSE: Row4[] = [['media_player.all_sonos', 'All Sonos', 3, 1], ['media_player.chromecast_shadow', 'Chromecast', 1, 1]];
const GUARD_OPTS = ['0.2s', '0.4s', '0.6s', '0.8s', '1s', '1.2s', '1.4s', '1.6s', '1.8s'];
const HINTS = {
  remote_routing: "Send the assistant's answers, sign-in prompts, timer rings and the wake chime to other speakers instead of (or as well as) this one.",
  remote_tts_volume: 'The level remote players answer at. Zero leaves every target volume alone.',
  remote_wake_chime: 'Play the wake chime on the selected players too, so a far room hears that the device is listening.',
  remote_timer_ring: 'Ring finished timers on the selected players as well.',
  remote_sync_guard: 'What happens to music the routing interrupted on the remote players once the answer ends.',
  area_ducking: 'Lower other speakers while you talk to this device, from the wake word until the answer ends.',
  duck_volume: 'The level ducked players drop to. Zero silences them for the length of the interaction.'
};
type Sel = {
  areas: Set<string>;
  extra: Set<string>;
  excluded: Set<string>;
};
type CheckState = 'on' | 'off' | 'mixed';
let closeActiveHint: (() => void) | null = null;
function HintBtn({
  text
}: {
  text: string;
}) {
  const [open, setOpen] = useState(false);
  const [pos, setPos] = useState<{
    top: number;
    left: number;
  } | null>(null);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const close = useRef(() => setOpen(false));
  useLayoutEffect(() => {
    if (!open) {
      setPos(null);
      return;
    }
    const id = requestAnimationFrame(() => {
      const b = btn.current?.getBoundingClientRect();
      const w = bubble.current?.offsetWidth ?? 260;
      const h = bubble.current?.offsetHeight ?? 60;
      if (!b) return;
      let left = b.left + b.width / 2 - w / 2;
      left = Math.max(8, Math.min(left, window.innerWidth - w - 8));
      let top = b.bottom + 8;
      if (top + h > window.innerHeight - 8) top = b.top - h - 8;
      setPos({
        top,
        left
      });
    });
    return () => cancelAnimationFrame(id);
  }, [open]);
  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      const t = e.target as Node;
      if (btn.current?.contains(t) || bubble.current?.contains(t)) return;
      setOpen(false);
    };
    const onScroll = () => setOpen(false);
    document.addEventListener('mousedown', onDown);
    window.addEventListener('scroll', onScroll, true);
    window.addEventListener('resize', onScroll);
    return () => {
      document.removeEventListener('mousedown', onDown);
      window.removeEventListener('scroll', onScroll, true);
      window.removeEventListener('resize', onScroll);
      if (closeActiveHint === close.current) closeActiveHint = null;
    };
  }, [open]);
  const toggle = () => {
    if (!open) {
      if (closeActiveHint && closeActiveHint !== close.current) closeActiveHint();
      closeActiveHint = close.current;
    }
    setOpen(v => !v);
  };
  return <span style={{
    display: 'inline-flex'
  }}>
      <button ref={btn} type="button" className="au-hint-btn" aria-label="More info" aria-expanded={open} onClick={toggle}>i</button>
      {open && ReactDOM.createPortal(<div ref={bubble} role="tooltip" className="au-hint-bubble" style={{
      top: pos?.top ?? -9999,
      left: pos?.left ?? -9999,
      visibility: pos ? 'visible' : 'hidden'
    }}>{text}</div>, document.body)}
    </span>;
}
function AuToggle({
  checked,
  disabled,
  onChange
}: {
  checked: boolean;
  disabled?: boolean;
  onChange: (v: boolean) => void;
}) {
  return <button role="switch" aria-checked={checked} disabled={disabled} onClick={() => onChange(!checked)} className={`au-toggle${checked ? ' on' : ''}${disabled ? ' disabled' : ''}`}>
      <span className="au-toggle-thumb" />
    </button>;
}
function AuSelect({
  value,
  options,
  disabled,
  onChange
}: {
  value: string;
  options: string[];
  disabled?: boolean;
  onChange: (v: string) => void;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const close = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener('mousedown', close);
    return () => document.removeEventListener('mousedown', close);
  }, [open]);
  return <div ref={ref} className={`au-sel${disabled ? ' disabled' : ''}`}>
      <button className={`au-sel-btn${open ? ' open' : ''}`} disabled={disabled} onClick={() => setOpen(v => !v)}>
        <span>{value}</span>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" style={{
        transform: open ? 'rotate(180deg)' : undefined,
        transition: 'transform .2s'
      }}>
          <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
      </button>
      {open && <div className="au-sel-pop">
          {options.map(o => <button key={o} className={`au-sel-opt${o === value ? ' active' : ''}`} onClick={() => {
        onChange(o);
        setOpen(false);
      }}>
              {o === value && <svg width="12" height="12" viewBox="0 0 12 12" fill="none"><path d="M2 6l3 3 5-5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>}
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
        <button className="au-tree-caret" tabIndex={-1} aria-hidden={true} aria-label={expanded ? 'Collapse' : 'Expand'} aria-expanded={expanded} onClick={onExpand}>
          <svg width="12" height="12" viewBox="0 0 12 12" fill="none" style={{
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
function TargetTree({
  sel,
  onSel,
  local,
  onLocal,
  need
}: {
  sel: Sel;
  onSel: (next: Sel) => void;
  local: boolean | null;
  onLocal?: (v: boolean) => void;
  need: number;
}) {
  const [open, setOpen] = useState<Record<string, boolean>>({});
  const [localSpeakerVolume, setLocalSpeakerVolume] = useState(50);
  const capOk = (row: Row4) => ((row[2] ?? 3) & need) !== 0;
  const isSelf = (row: Row4) => ((row[2] ?? 0) & 4) !== 0;
  const elig = (row: Row4) => capOk(row) && !isSelf(row);
  const isLive = (row: Row4) => (row[3] ?? 1) !== 0;
  const reason = need === 2 ? 'no volume control' : 'cannot play media';
  const edit = (fn: (next: Sel) => void) => {
    const next: Sel = {
      areas: new Set(sel.areas),
      extra: new Set(sel.extra),
      excluded: new Set(sel.excluded)
    };
    fn(next);
    onSel(next);
  };
  const isOn = (areaId: string | null, id: string) => areaId !== null && sel.areas.has(areaId) && !sel.excluded.has(`${areaId}:${id}`) || sel.extra.has(id);
  const clickPlayer = (areaId: string | null, id: string) => {
    edit(next => {
      if (areaId !== null && next.areas.has(areaId)) {
        const key = `${areaId}:${id}`;
        if (next.excluded.has(key)) next.excluded.delete(key);else {
          next.excluded.add(key);
          next.extra.delete(id);
        }
        return;
      }
      if (next.extra.has(id)) next.extra.delete(id);else next.extra.add(id);
    });
  };
  const areaState = (area: (typeof HA_AREAS)[0]): CheckState => {
    const ids = area.p.filter(elig).map(([id]) => id);
    if (sel.areas.has(area.i)) return ids.some(id => sel.excluded.has(`${area.i}:${id}`)) ? 'mixed' : 'on';
    const on = ids.filter(id => sel.extra.has(id)).length;
    return on === 0 ? 'off' : on === ids.length ? 'on' : 'mixed';
  };
  const clickArea = (area: (typeof HA_AREAS)[0]) => {
    const state = areaState(area);
    edit(next => {
      next.excluded.forEach(k => {
        if (k.startsWith(`${area.i}:`)) next.excluded.delete(k);
      });
      area.p.forEach(([id]) => next.extra.delete(id));
      if (state === 'on') next.areas.delete(area.i);else next.areas.add(area.i);
    });
  };
  const looseState = (): CheckState => {
    const rows = HA_LOOSE.filter(elig);
    const on = rows.filter(([id]) => sel.extra.has(id)).length;
    return on === 0 ? 'off' : on === rows.length ? 'on' : 'mixed';
  };
  const areaCount = (area: (typeof HA_AREAS)[0]) => {
    const rows = area.p.filter(elig);
    if (sel.areas.has(area.i)) {
      const cut = rows.filter(([id]) => sel.excluded.has(`${area.i}:${id}`)).length;
      return cut === 0 ? 'whole area' : `${rows.length - cut}/${rows.length}`;
    }
    return `${rows.filter(([id]) => sel.extra.has(id)).length}/${rows.length}`;
  };
  const playerRow = (areaId: string | null) => (row: Row4) => {
    const [id, name] = row;
    const capable = capOk(row);
    const self = isSelf(row);
    const eligible = capable && !self;
    const live = isLive(row);
    const on = eligible && isOn(areaId, id);
    return <div className={`au-tree-p${eligible && live ? '' : ' au-tree-off'}`} key={id} style={{
      minHeight: 44,
      paddingTop: 10,
      paddingBottom: 10
    }}>
        <AuCheck state={on ? 'on' : 'off'} disabled={!eligible} onClick={() => clickPlayer(areaId, id)} label={name} />
        <span className="au-tree-player-name">{name}</span>
        {(!eligible || !live) && <span className="au-tree-why">{self ? 'This device' : !capable ? reason : 'offline'}</span>}
      </div>;
  };
  return <div className="au-tree">
      {local !== null && <div className="au-tree-p au-tree-self">
          <AuCheck state={local ? 'on' : 'off'} onClick={() => onLocal && onLocal(!local)} label="Local Speaker" />
          <span className="au-tree-player-name">Local Speaker</span>
          {local && <span style={{
        display: 'flex',
        alignItems: 'center',
        gap: 6
      }}>
              <HintBtn text="Overrides the satellite's local speaker volume for voice responses. Adjusts in 5-point steps." />
              <button type="button" className="au-sel-btn" aria-label="Decrease local speaker volume" style={{
          width: 36,
          minHeight: 36,
          padding: 0,
          justifyContent: 'center'
        }} disabled={localSpeakerVolume <= 0} onClick={() => setLocalSpeakerVolume(v => Math.max(0, v - 5))}>−</button>
              <span className="mslider-val" aria-live="polite" style={{
          minWidth: 30,
          textAlign: 'center'
        }}>{localSpeakerVolume}</span>
              <button type="button" className="au-sel-btn" aria-label="Increase local speaker volume" style={{
          width: 36,
          minHeight: 36,
          padding: 0,
          justifyContent: 'center'
        }} disabled={localSpeakerVolume >= 100} onClick={() => setLocalSpeakerVolume(v => Math.min(100, v + 5))}>+</button>
            </span>}
        </div>}
      {HA_AREAS.map(area => <Group key={area.i} label={area.n} count={areaCount(area)} state={areaState(area)} expanded={!!open[area.i]} onExpand={() => setOpen({
      ...open,
      [area.i]: !open[area.i]
    })} onBulk={() => clickArea(area)}>
          {area.p.map(playerRow(area.i))}
        </Group>)}
      <Group label="No Area Assigned" count={`${HA_LOOSE.filter(elig).filter(([id]) => sel.extra.has(id)).length}/${HA_LOOSE.filter(elig).length}`} state={looseState()} expanded={!!open.__loose} onExpand={() => setOpen({
      ...open,
      __loose: !open.__loose
    })} onBulk={() => edit(next => {
      const state = looseState();
      HA_LOOSE.filter(elig).forEach(([id]) => state === 'on' ? next.extra.delete(id) : next.extra.add(id));
    })}>
        {HA_LOOSE.map(playerRow(null))}
      </Group>
    </div>;
}
function RemoteRouting() {
  const [sel, setSel] = useState<Sel>({
    areas: new Set(['living_room']),
    extra: new Set(),
    excluded: new Set()
  });
  const [local, setLocal] = useState(true);
  const [vol, setVol] = useState(0);
  const [chime, setChime] = useState(true);
  const [timer, setTimer] = useState(true);
  const [guard, setGuard] = useState('0.2s');
  const active = sel.areas.size > 0 || sel.extra.size > 0;
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Voice Response Routing</span><HintBtn text={HINTS.remote_routing} /></div>
      <p className="au-tree-title">Route the assistant voice response to selected speakers</p>
      <TargetTree sel={sel} onSel={setSel} local={local} onLocal={setLocal} need={1} />
      <div className="au-row">
        <div className="au-row-label"><span>Remote Speaker Volume</span><HintBtn text={HINTS.remote_tts_volume} /></div>
        <MSlider value={vol} min={0} max={100} step={1} disabled={!active} ariaLabel="Remote speaker volume" format={v => v === 0 ? 'Use Remote Volume' : `${Math.round(v)}%`} onCommit={setVol} />
      </div>
      <div className="au-row">
        <div className="au-row-label"><span>Remote wake chime</span><HintBtn text={HINTS.remote_wake_chime} /></div>
        <AuToggle checked={chime} disabled={!active} onChange={setChime} />
      </div>
      <div className="au-row">
        <div className="au-row-label"><span>Remote timer ring</span><HintBtn text={HINTS.remote_timer_ring} /></div>
        <AuToggle checked={timer} disabled={!active} onChange={setTimer} />
      </div>
      <div className="au-row au-row-last">
        <div className="au-row-label"><span>Remote mic guard</span><HintBtn text={HINTS.remote_sync_guard} /></div>
        <AuSelect value={guard} options={GUARD_OPTS} disabled={!active} onChange={setGuard} />
      </div>
    </div>;
}
function AreaDucking() {
  const [sel, setSel] = useState<Sel>({
    areas: new Set(),
    extra: new Set(['media_player.kitchen_display']),
    excluded: new Set()
  });
  const [vol, setVol] = useState(20);
  const active = sel.areas.size > 0 || sel.extra.size > 0;
  return <div className="au-card">
      <div className="au-card-head"><span className="au-card-title">Area Ducking</span><HintBtn text={HINTS.area_ducking} /></div>
      <p className="au-tree-title">Lower the volume on selected players upon wake word detection</p>
      <TargetTree sel={sel} onSel={setSel} local={null} need={2} />
      <div className="au-row au-row-last">
        <div className="au-row-label"><span>Duck volume</span><HintBtn text={HINTS.duck_volume} /></div>
        <MSlider value={vol} min={0} max={100} step={1} disabled={!active} ariaLabel="Duck volume" format={v => v === 0 ? 'mute' : `${Math.round(v)}%`} onCommit={setVol} />
      </div>
    </div>;
}
export function AudioTab({
  ctx
}: {
  ctx: Ctx;
}) {
  return <section className="control au-tab au-route">
      <span className="eyebrow">AUDIO · ROUTING</span>
      <h1><span>Sound, </span><em>directed.</em></h1>
      <div className="au-route-cards">
        <RemoteRouting />
        <AreaDucking />
      </div>
    </section>;
}