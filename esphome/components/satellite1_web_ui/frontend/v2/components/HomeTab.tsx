import { useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { HINTS, PRESENCE, TEXT } from '../../src/copy.js';
import { entity, pathFor, post, useVoice } from '../../src/lib/device.js';
import { sparkPoints } from '../../src/lib/sparkhist.js';
import { sparkPaths } from '../../src/lib/sparkline.js';
import type { Ctx, Orb } from '../ctx';
import { ChevronDown, Clock } from '../icons';
import { clock, isOn, offsetSpec, offsetText, reading, stepOffset, timerLabel, timerLeft, transcriptTabs } from '../lib/orb.js';
import { useHeld, VoiceOrb } from './VoiceOrb';
type Sensor = {
  id: string;
  key: string;
  offsetKey: string;
  label: string;
  unit: string;
  digits: number;
  step: number;
  min: number;
  max: number;
  hint: string;
};
/** GET /api/sat1/voice's timer and transcript rows (web_ui_handler.cpp handle_voice_). */
type Timer = {
  id: string;
  name: string;
  total: number;
  left: number;
  active: boolean;
};
type Line = {
  heard: boolean;
  at: number;
  w: string;
  text: string;
};

/**
 * v1's sensor table (routes/controls.jsx). `step` is the display step: the offset stepper's floor
 * and the sparkline's backfill amplitude. `min`/`max` stand in for a number payload without its range.
 */
const SENSORS: Sensor[] = [{
  id: 'temp',
  key: 'temp',
  offsetKey: 'temp_offset',
  label: 'Temperature',
  unit: '\u00B0C',
  digits: 1,
  step: 0.1,
  min: -20,
  max: 20,
  hint: HINTS.temp
}, {
  id: 'humidity',
  key: 'humidity',
  offsetKey: 'humidity_offset',
  label: 'Humidity',
  unit: '%',
  digits: 0,
  step: 1,
  min: -50,
  max: 50,
  hint: HINTS.humidity
}, {
  id: 'light',
  key: 'lux',
  offsetKey: 'lux_offset',
  label: 'Light',
  unit: ' lx',
  digits: 0,
  step: 5,
  min: -500,
  max: 500,
  hint: HINTS.lux
}];
type SparkData = {
  pts: [number, number][];
  amp: number;
  seed: string;
};
function Spark({
  pts,
  amp,
  seed
}: SparkData) {
  const made = sparkPaths(pts, amp, seed);
  if (!made) return null;
  const gid = `spkg-${seed}`.replace(/[^a-zA-Z0-9-]/g, '-');
  return <svg className="spark" viewBox="0 0 100 100" preserveAspectRatio="none" pointerEvents="none" aria-hidden="true"><defs><linearGradient id={gid} x1="0" y1="0" x2="0" y2="1"><stop offset="0" stopColor="var(--orb-a)" stopOpacity="0.22" /><stop offset="1" stopColor="var(--orb-a)" stopOpacity="0" /></linearGradient></defs><path d={made.fill} fill={`url(#${gid})`} stroke="none" /><path d={made.line} fill="none" stroke="var(--orb-a)" strokeOpacity={0.5} strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke" /></svg>;
}
function DrawerSpark({
  pts,
  amp,
  seed
}: SparkData) {
  const made = sparkPaths(pts, amp, seed);
  if (!made) return null;
  return <div className="spark"><svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true"><path d={made.line} /></svg></div>;
}
function SensorDrawer({
  label,
  onClose,
  children
}: {
  label: string;
  onClose: () => void;
  children: React.ReactNode;
}) {
  const panelRef = useRef<HTMLDivElement>(null);
  const dragStartY = useRef<number | null>(null);
  const closeRef = useRef(onClose);
  closeRef.current = onClose;
  useEffect(() => {
    document.body.classList.add('has-drawer');
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') closeRef.current();
    };
    document.addEventListener('keydown', onKey);
    return () => {
      document.body.classList.remove('has-drawer');
      document.removeEventListener('keydown', onKey);
    };
  }, []);
  return createPortal([<div key="scrim" className="scrim sensor-scrim" onClick={onClose} />, <div key="panel" ref={panelRef} className="sheet sensor-sheet" role="dialog" aria-modal="true" aria-label={label} onClick={e => e.stopPropagation()}>
      <div className="handle" style={{
      touchAction: 'none',
      cursor: 'grab'
    }} role="button" aria-label="Close" onPointerDown={e => {
      if (window.innerWidth >= 1024) return;
      dragStartY.current = e.clientY;
      (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
      if (panelRef.current) panelRef.current.style.transition = 'none';
    }} onPointerMove={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (panelRef.current) {
        panelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
        panelRef.current.style.opacity = String(Math.max(0, 1 - dy / 200));
      }
    }} onPointerUp={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (dy > 80) {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
          panelRef.current.style.opacity = '0';
          setTimeout(onClose, 210);
        } else onClose();
      } else {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%)';
          panelRef.current.style.opacity = '1';
        }
        if (dy < 6) onClose();
      }
      dragStartY.current = null;
      setTimeout(() => {
        if (panelRef.current) {
          panelRef.current.style.transition = '';
          panelRef.current.style.transform = '';
          panelRef.current.style.opacity = '';
        }
      }, 250);
    }} />
    
      {children}
    </div>], document.body);
}

/**
 * The calibration offset: a number entity whose value the sensor's filter adds, so the reading the
 * sensor publishes is already corrected. Every press writes; the pressed value holds on screen until
 * the device echoes it, so quick presses step on from each other rather than from a stale value.
 */
function useOffset(ctx: Ctx, s: Sensor) {
  const off = entity(ctx, s.offsetKey);
  const spec = offsetSpec(off, s);
  const [offset, hold] = useHeld(Number(off?.value) || 0, spec.step / 2);
  const bump = (dir: 1 | -1) => {
    const next = stepOffset(offset, dir, spec.step, spec.min, spec.max);
    if (next === offset) return;
    hold(next);
    post(`${pathFor(ctx, s.offsetKey, 'set')}?value=${next}`);
  };
  return {
    offset,
    bump,
    atMin: offset <= spec.min,
    atMax: offset >= spec.max
  };
}
function TempPopup({
  ctx,
  s,
  value,
  spark,
  close
}: {
  ctx: Ctx;
  s: Sensor;
  value: unknown;
  spark: SparkData;
  close: () => void;
}) {
  const {
    offset,
    bump,
    atMin,
    atMax
  } = useOffset(ctx, s);
  const unitF = entity(ctx, 'temp_unit_f');
  const isF = isOn(unitF);
  return <div className="sensor-drawer-body temp-pop" aria-label="Temperature settings"><div><span className="eyebrow">CALIBRATION · Temperature</span><strong>{reading(value, s.digits, s.unit, isF)}</strong></div><p className="muted cal-hint">{s.hint}</p><DrawerSpark {...spark} /><div className="temp-row"><span>Offset</span><div className="stepper"><button aria-label="Decrease offset" disabled={atMin} onClick={() => bump(-1)}>−</button><b>{offsetText(offset, s.digits, '°', isF)}</b><button aria-label="Increase offset" disabled={atMax} onClick={() => bump(1)}>+</button></div></div>{unitF && <div className="temp-row"><span>Fahrenheit</span><button role="switch" aria-checked={isF} aria-label="Use Fahrenheit" title={HINTS.temp_unit} className={'switch ' + (isF ? 'on' : '')} onClick={() => post(pathFor(ctx, 'temp_unit_f', isF ? 'turn_off' : 'turn_on'))}><i /></button></div>}<div className="cal-actions"><button className="done" onClick={close}>Done</button></div></div>;
}
function Calibration({
  ctx,
  s,
  value,
  spark,
  close
}: {
  ctx: Ctx;
  s: Sensor;
  value: unknown;
  spark: SparkData;
  close: () => void;
}) {
  const {
    offset,
    bump,
    atMin,
    atMax
  } = useOffset(ctx, s);
  return <div className="sensor-drawer-body"><div><span className="eyebrow">CALIBRATION · {s.label}</span><strong>{reading(value, s.digits, s.unit)}</strong></div><p className="muted cal-hint">{s.hint}</p><DrawerSpark {...spark} /><div className="cal-actions"><button aria-label="Decrease offset" disabled={atMin} onClick={() => bump(-1)}>−</button><span>offset {offsetText(offset, s.digits, s.unit)}</span><button aria-label="Increase offset" disabled={atMax} onClick={() => bump(1)}>+</button><button className="done" onClick={close}>Done</button></div></div>;
}

/**
 * Temperature, humidity and light, each opening its calibration drawer when the build has the
 * offset entity behind it, and the radar's presence, which leaves for the Presence tab.
 */
function SensorPills({
  ctx
}: {
  ctx: Ctx;
}) {
  const [expanded, setExpanded] = useState<string | null>(null);
  if (!ctx.device) return <div className="pills">{[0, 1, 2, 3].map(i => <div key={i}><div className="sensor skel" aria-hidden="true" style={{
        height: "87.29px"
      }}><strong style={{
          height: "33.1px"
        }}>&nbsp;</strong><small style={{
          paddingTop: "0px"
        }}>&nbsp;</small><ChevronDown size={10} className="sensor-caret" /></div></div>)}</div>;
  const isF = isOn(entity(ctx, 'temp_unit_f'));
  const mac = String(ctx.device.mac || 'local').toLowerCase();
  // Referenced by id: satellite1_radar registers it at runtime from a C++ literal, so it has no
  // config id for the entity map to point at.
  const presence = ctx.states['text_sensor/Radar Target'];
  const module = entity(ctx, 'radar_module');
  const close = () => setExpanded(null);
  return <div className="pills">{SENSORS.map(s => {
      const sensor = entity(ctx, s.key);
      if (!sensor) return null;
      const editable = !!entity(ctx, s.offsetKey);
      const spark: SparkData = {
        pts: sparkPoints(s.key),
        amp: s.step,
        seed: `${mac}:${s.key}`
      };
      const val = reading(sensor.value, 0, s.unit, s.id === 'temp' && isF);
      const face = <><Spark {...spark} /><strong style={{
          height: "33.1px"
        }}>{val}</strong><small style={{
          paddingTop: "0px"
        }}>{s.label}</small></>;
      return <div key={s.id}>{editable ? <button className={'sensor ' + (expanded === s.id ? 'active' : '')} aria-haspopup="dialog" aria-expanded={expanded === s.id} onClick={() => setExpanded(expanded === s.id ? null : s.id)} style={{
          height: "87.29px"
        }}>{face}<ChevronDown size={10} className="sensor-caret" aria-hidden="true" /></button> : <div className="sensor" style={{
          height: "87.29px"
        }}>{face}</div>}
        {editable && expanded === s.id && <SensorDrawer label={`${s.label} calibration`} onClose={close}>{s.id === 'temp' ? <TempPopup ctx={ctx} s={s} value={sensor.value} spark={spark} close={close} /> : <Calibration ctx={ctx} s={s} value={sensor.value} spark={spark} close={close} />}</SensorDrawer>}</div>;
    })}{presence && <div><a className="sensor" href="#/presence" title={[presence.value, module?.value ? `${module.value} settings` : 'Presence'].filter(Boolean).join(' \u2014 ')} onClick={e => {
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey) return;
        e.preventDefault();
        ctx.go('PRESENCE');
      }} style={{
        textDecoration: 'none'
      }}><strong>{PRESENCE[presence.value as keyof typeof PRESENCE] || presence.value || '\u2014'}</strong><small>Presence ›</small></a></div>}</div>;
}

/**
 * The last few exchanges, oldest at the top, with v1's per-wake-word tabs above them once two words
 * have spoken. The newest word's tab opens by itself, and a hand-picked tab holds until a newer word
 * fires. The box follows new lines unless the person has scrolled up to read.
 */
function Transcript({
  lines
}: {
  lines: Line[];
}) {
  const [pick, setPick] = useState<string | null>(null);
  const {
    words,
    tab,
    shown
  } = transcriptTabs(lines, pick);
  const newest = words[0] || null;
  const newestRef = useRef(newest);
  useEffect(() => {
    if (newest !== newestRef.current) {
      newestRef.current = newest;
      setPick(null);
    }
  }, [newest]);
  const box = useRef<HTMLElement>(null);
  const stuck = useRef(true);
  const last = shown[shown.length - 1];
  useLayoutEffect(() => {
    const el = box.current;
    if (el && stuck.current) el.scrollTop = el.scrollHeight;
  }, [shown.length, last?.text, tab]);
  return <>{words.length > 1 && <div className="tt-tabs" role="tablist" aria-label="Transcript by wake word">{words.map(w => <button key={w} role="tab" aria-selected={w === tab} className={'tt-tab' + (w === tab ? ' on' : '')} onClick={() => setPick(w)}>{`“${w === 'stop' ? 'Stop' : w}”`}</button>)}</div>}<section ref={box} className="transcript transcript-tall" onScroll={e => {
      const el = e.currentTarget;
      stuck.current = el.scrollHeight - el.scrollTop - el.clientHeight < 32;
    }}>{shown.length ? shown.map((l, i) => <p key={i} className={l.heard ? 'user' : 'assistant'}>{l.text}</p>) : <p className="transcript-empty">{TEXT.nothing_said}</p>}</section></>;
}

/**
 * The device's timers, read-only: Home Assistant owns Assist timers and offers no way to cancel one
 * from here, so they are managed by voice. The poll runs every second while one counts; between
 * answers the shown time counts down from the last one so seconds never stall or skip.
 */
function Timers({
  timers
}: {
  timers: Timer[];
}) {
  const polledAt = useMemo(() => Date.now(), [timers]);
  const [now, setNow] = useState(Date.now);
  const counting = timers.some(t => t.active);
  useEffect(() => {
    if (!counting) return;
    const id = setInterval(() => setNow(Date.now()), 250);
    return () => clearInterval(id);
  }, [counting]);
  return <div className="timer-list">{timers.map(t => {
      const left = timerLeft(t, polledAt, Math.max(now, polledAt));
      return <div key={t.id} className={'timer-pill' + (left === 0 ? ' done' : '') + (t.active ? '' : ' paused')} title={HINTS.timers}><Clock size={15} aria-hidden="true" /><span className="timer-pill-label">{timerLabel(t)}{!t.active && <small> · paused</small>}</span><strong className="timer-pill-time">{clock(left)}</strong></div>;
    })}</div>;
}
export function HomeTab({
  ctx,
  orb,
  onOrbColor
}: {
  ctx: Ctx;
  orb: Orb;
  onOrbColor: (from: string, to: string) => void;
}) {
  const voice = useVoice(true);
  return <section className="now now-compact"><div className="now-left" style={{
      width: '100%'
    }}><SensorPills ctx={ctx} /><VoiceOrb ctx={ctx} phase={voice?.phase} orb={orb} onOrbColor={onOrbColor} /></div><div className="now-right"><Transcript lines={voice?.transcript || []} /><Timers timers={voice?.timers || []} /></div></section>;
}
