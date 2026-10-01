import { useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import type { Ctx, Orb } from '../ctx';
import { ChevronDown, Clock, X } from '../icons';
import { VoiceOrb } from './VoiceOrb';
type Timer = {
  id: string;
  label: string;
  total: number;
  remaining: number;
};
const fmtTime = (s: number) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
const transcript = [{
  who: 'assistant',
  text: 'Good evening. The room is calm.'
}, {
  who: 'user',
  text: 'Set a timer for ten minutes.'
}, {
  who: 'assistant',
  text: 'Ten minutes, starting now.'
}, {
  who: 'assistant',
  text: 'Anything else?'
}, {
  who: 'assistant',
  text: "Your timer is set. I'll let you know when the ten minutes are up."
}, {
  who: 'user',
  text: 'Also dim the living room lights to 40 percent.'
}, {
  who: 'assistant',
  text: "Done, living room lights are now at 40%. Anything else you'd like me to adjust?"
}, {
  who: 'user',
  text: "That's all for now, thanks."
}];
function walk(seed: number, n: number, base: number, amp: number) {
  let s = seed;
  let v = base;
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    s = s * 1103515245 + 12345 & 0x7fffffff;
    v += (s / 0x7fffffff - 0.5) * amp;
    out.push(v);
  }
  return out;
}
const SPARKS: Record<string, number[]> = {
  temp: walk(7, 24, 21.4, 0.35),
  humidity: walk(13, 24, 46, 1.6),
  lux: walk(29, 24, 180, 28)
};
const SPARK_MAP: Record<string, {
  pts: number[];
  seed: string;
}> = {
  temp: {
    pts: SPARKS.temp,
    seed: 'local:temp'
  },
  humidity: {
    pts: SPARKS.humidity,
    seed: 'local:humidity'
  },
  light: {
    pts: SPARKS.lux,
    seed: 'local:lux'
  }
};
function Spark({
  pts,
  seed
}: {
  pts: number[];
  seed: string;
}) {
  const lo = Math.min(...pts);
  const hi = Math.max(...pts);
  const amp = Math.max(hi - lo, 1e-6);
  const X = (i: number) => i / (pts.length - 1) * 100;
  const Y = (v: number) => 88 - (v - lo) / amp * 62;
  let line = `M ${X(0)} ${Y(pts[0])}`;
  for (let i = 1; i < pts.length; i++) {
    const mx = (X(i - 1) + X(i)) / 2;
    const my = (Y(pts[i - 1]) + Y(pts[i])) / 2;
    line += ` Q ${X(i - 1)} ${Y(pts[i - 1])} ${mx} ${my}`;
  }
  line += ` L ${X(pts.length - 1)} ${Y(pts[pts.length - 1])}`;
  const fill = `${line} L 100 100 L 0 100 Z`;
  const gid = `spkg-${seed}`.replace(/[^a-zA-Z0-9-]/g, '-');
  return <svg className="spark" viewBox="0 0 100 100" preserveAspectRatio="none" pointerEvents="none" aria-hidden="true"><defs><linearGradient id={gid} x1="0" y1="0" x2="0" y2="1"><stop offset="0" stopColor="var(--orb-a)" stopOpacity="0.22" /><stop offset="1" stopColor="var(--orb-a)" stopOpacity="0" /></linearGradient></defs><path d={fill} fill={`url(#${gid})`} stroke="none" /><path d={line} fill="none" stroke="var(--orb-a)" strokeOpacity={0.5} strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke" /></svg>;
}
function SensorDrawer({
  onClose,
  children
}: {
  onClose: () => void;
  children: React.ReactNode;
}) {
  const panelRef = useRef<HTMLDivElement>(null);
  const dragStartY = useRef<number | null>(null);
  useEffect(() => {
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, []);
  return createPortal([<div key="scrim" className="scrim sensor-scrim" onClick={onClose} />, <div key="panel" ref={panelRef} className="sheet sensor-sheet" role="dialog" aria-modal="true" onClick={e => e.stopPropagation()}>
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
function Calibration({
  label,
  value,
  unit,
  close
}: {
  label: string;
  value: string;
  unit: string;
  close: () => void;
}) {
  return <div className="sensor-drawer-body"><div><span className="eyebrow">CALIBRATION · {label}</span><strong>{value}{unit}</strong></div><div className="spark"><svg viewBox="0 0 180 32" preserveAspectRatio="none"><polyline points="0,22 12,18 24,21 38,11 52,16 68,8 84,14 100,6 116,13 132,9 148,12 164,4 180,8" /></svg></div><div className="cal-actions"><button>−</button><span>offset 0</span><button>+</button><button className="done" onClick={close}>Done</button></div></div>;
}
function TempPopup({
  offset,
  setOffset,
  unit,
  setUnit,
  close
}: {
  offset: number;
  setOffset: (v: number) => void;
  unit: 'C' | 'F';
  setUnit: (v: 'C' | 'F') => void;
  close: () => void;
}) {
  const c = 21.4 + offset;
  const shown = unit === 'C' ? `${c.toFixed(1)}°C` : `${(c * 9 / 5 + 32).toFixed(1)}°F`;
  return <div className="sensor-drawer-body temp-pop" aria-label="Temperature settings"><div><span className="eyebrow">CALIBRATION · Temperature</span><strong>{shown}</strong></div><div className="spark"><svg viewBox="0 0 180 32" preserveAspectRatio="none"><polyline points="0,20 12,19 24,17 38,18 52,14 68,15 84,12 100,13 116,10 132,12 148,9 164,10 180,8" /></svg></div><div className="temp-row"><span>Offset</span><div className="stepper"><button aria-label="Decrease offset" onClick={() => setOffset(Math.max(-5, +(offset - 0.5).toFixed(1)))}>−</button><b>{offset > 0 ? '+' : ''}{offset.toFixed(1)}°</b><button aria-label="Increase offset" onClick={() => setOffset(Math.min(5, +(offset + 0.5).toFixed(1)))}>+</button></div></div><div className="temp-row"><span>Fahrenheit</span><button role="switch" aria-checked={unit === 'F'} aria-label="Use Fahrenheit" className={'switch ' + (unit === 'F' ? 'on' : '')} onClick={() => setUnit(unit === 'C' ? 'F' : 'C')}><i /></button></div><div className="cal-actions"><button className="done" onClick={close}>Done</button></div></div>;
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
  const [expanded, setExpanded] = useState<string | null>(null);
  const [timers, setTimers] = useState<Timer[]>([{
    id: 't1',
    label: 'Timer 1',
    total: 600,
    remaining: 581
  }]);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const [unit, setUnit] = useState<'C' | 'F'>('C');
  const [offset, setOffset] = useState(0);
  const tempC = 21.4 + offset;
  const tempText = unit === 'C' ? `${Math.round(tempC)}°C` : `${Math.round(tempC * 9 / 5 + 32)}°F`;
  useEffect(() => {
    intervalRef.current = setInterval(() => {
      setTimers(v => v.some(t => t.remaining > 0) ? v.map(t => t.remaining > 0 ? {
        ...t,
        remaining: t.remaining - 1
      } : t) : v);
    }, 1000);
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
    };
  }, []);
  const removeTimer = (id: string) => setTimers(v => v.filter(t => t.id !== id));
  return <section className="now now-compact"><div className="now-left" style={{
      width: '100%'
    }}><div className="pills">{[['temp', tempText, 'Temperature'], ['humidity', '46%', 'Humidity'], ['light', '182 lx', 'Light'], ['presence', 'Still', 'Presence']].map(([key, val, label]) => <div key={key}>{key === 'presence' ? <a className="sensor" href="#/presence" onClick={e => {
            e.preventDefault();
            ctx.go('PRESENCE');
          }} style={{
            textDecoration: 'none'
          }}><strong>{val}</strong><small>{label} ›</small></a> : <button className={'sensor ' + (expanded === key ? 'active' : '')} onClick={() => setExpanded(expanded === key ? null : key)} style={{
            height: "87.29px"
          }}>{SPARK_MAP[key] && <Spark pts={SPARK_MAP[key].pts} seed={SPARK_MAP[key].seed} />}<strong style={{
              height: "33.1px"
            }}>{val}</strong><small style={{
              paddingTop: "0px"
            }}>{label}</small><ChevronDown size={10} className="sensor-caret" aria-hidden="true" /></button>}
        {expanded === key && (key === 'temp' ? <SensorDrawer onClose={() => setExpanded(null)}><TempPopup offset={offset} setOffset={setOffset} unit={unit} setUnit={setUnit} close={() => setExpanded(null)} /></SensorDrawer> : <SensorDrawer onClose={() => setExpanded(null)}><Calibration label={label} value={val} unit="" close={() => setExpanded(null)} /></SensorDrawer>)}</div>)}</div><VoiceOrb onColorChange={onOrbColor} /></div><div className="now-right"><section className="transcript transcript-tall">{transcript.map((line, i) => <p key={i} className={line.who}>{line.text}</p>)}</section><div className="timer-list">{timers.map(t => <div key={t.id} className={'timer-pill' + (t.remaining === 0 ? ' done' : '')}><Clock size={15} aria-hidden="true" /><span className="timer-pill-label">{t.label}</span><strong className="timer-pill-time">{fmtTime(t.remaining)}</strong><button type="button" className="timer-pill-x" aria-label={`Cancel ${t.label}`} onClick={() => removeTimer(t.id)}><X size={14} /></button></div>)}</div></div></section>;
}
