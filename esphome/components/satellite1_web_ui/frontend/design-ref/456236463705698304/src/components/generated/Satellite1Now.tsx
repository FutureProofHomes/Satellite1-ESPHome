import { useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { AlertTriangle, Info, XCircle, Clock, X, LogOut, ChevronDown } from 'lucide-react';
import { VoiceOrb } from './VoiceOrb';
import { MediaBar } from './MediaBar';
import { WakeTab } from './WakeTab';
import { PresenceTab } from './PresenceTab';
import { AudioTab } from './AudioTab';
import { DiagnosticsTab, SETTINGS_ROUTES } from './DiagnosticsTab';
type Tab = 'NOW' | 'WAKE' | 'PRESENCE' | 'AUDIO' | 'SETTINGS' | 'SETUP';
type Sheet = 'media' | 'device' | 'notice' | null;
const DEV_TOASTS = true;
type Timer = {
  id: string;
  label: string;
  total: number;
  remaining: number;
};
type ToastKind = 'info' | 'warn' | 'error' | 'timer';
type Toast = {
  id: string;
  kind: ToastKind;
  message: string;
  duration?: number;
};
const TOAST_ICONS = {
  info: Info,
  warn: AlertTriangle,
  error: XCircle,
  timer: Clock
};
const fmtTime = (s: number) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
function ToastPill({
  toast,
  onDismiss,
  leaving = false
}: {
  toast: Toast;
  onDismiss: (id: string) => void;
  leaving?: boolean;
}) {
  const pillRef = useRef<HTMLDivElement>(null);
  const startX = useRef<number | null>(null);
  const [swipeDx, setSwipeDx] = useState(0);
  const [swiping, setSwiping] = useState(false);
  const [dismissed, setDismissed] = useState(false);
  useEffect(() => {
    if (leaving) return;
    const t = setTimeout(() => onDismiss(toast.id), toast.duration ?? 4000);
    return () => clearTimeout(t);
  }, [toast.id, toast.duration, onDismiss, leaving]);
  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (leaving || dismissed || (e.target as HTMLElement).closest('.toast-x')) return;
    startX.current = e.clientX;
    setSwiping(true);
    pillRef.current?.setPointerCapture(e.pointerId);
  };
  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (!swiping || startX.current === null) return;
    setSwipeDx(e.clientX - startX.current);
  };
  const onPointerUp = (e: React.PointerEvent<HTMLDivElement>) => {
    if (startX.current === null) return;
    setSwiping(false);
    const dx = e.clientX - startX.current;
    startX.current = null;
    if (Math.abs(dx) >= 72) {
      setDismissed(true);
      setSwipeDx(dx > 0 ? 400 : -400);
      setTimeout(() => onDismiss(toast.id), 250);
    } else setSwipeDx(0);
  };
  const Ico = TOAST_ICONS[toast.kind];
  return <div ref={pillRef} className={`header-toast-pill toast-${toast.kind}${leaving ? ' leaving' : ' entering'}`} role="status" style={{
    transform: `translateX(${swipeDx}px)`,
    opacity: dismissed ? 0 : Math.max(0.3, 1 - Math.abs(swipeDx) / 200),
    transition: swiping ? 'none' : 'transform 240ms ease, opacity 240ms ease',
    touchAction: 'pan-y',
    userSelect: 'none',
    cursor: 'grab'
  }} onPointerDown={onPointerDown} onPointerMove={onPointerMove} onPointerUp={onPointerUp} onPointerCancel={onPointerUp}>
      <Ico size={18} className="toast-icon" aria-hidden="true" />
      <span className="toast-msg">{toast.message}</span>
      <button type="button" className="toast-x" aria-label="Dismiss" onClick={() => onDismiss(toast.id)}><X size={14} /></button>
    </div>;
}
const art = 'https://storage.googleapis.com/storage.magicpath.ai/component-assets/454802455327313920/454809714925146112/ecc89c59771f06b4c2fa7960733c67a3eb86e4e4fb31599723751ffb6e85f432.png';
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
const peers = [{
  name: 'Kitchen Satellite',
  meta: 'Wi‑Fi · LD2450 · present',
  state: 'UP'
}, {
  name: 'Office Satellite',
  meta: 'Ethernet · LD2410 · no presence',
  state: 'UP'
}, {
  name: 'Bedroom Satellite',
  meta: 'Offline',
  state: 'OFFLINE'
}];
const logs = [{
  l: 'I',
  t: '12:04:21  voice pipeline ready'
}, {
  l: 'D',
  t: '12:04:19  radar target updated x=42 y=128'
}, {
  l: 'W',
  t: '12:03:55  Wi‑Fi signal -52 dBm'
}, {
  l: 'I',
  t: '12:03:51  media player connected'
}, {
  l: 'E',
  t: '11:58:02  chime retry succeeded'
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
function hexToRgba(hex: string, a: number): string {
  if (!hex.startsWith('#')) return `color-mix(in srgb, ${hex} ${Math.round(a * 100)}%, transparent)`;
  const h = hex.replace('#', '');
  const full = h.length === 3 ? h.split('').map(c => c + c).join('') : h;
  const n = parseInt(full, 16);
  return `rgba(${n >> 16 & 255},${n >> 8 & 255},${n & 255},${a})`;
}
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
function Icon({
  name,
  size = 18
}: {
  name: string;
  size?: number;
}) {
  const paths: Record<string, string> = {
    bell: 'M5 8a4 4 0 0 1 8 0c0 4 2 4 2 5H3c0-1 2-1 2-5Zm3 8h2',
    sun: 'M8 1v2m0 10v2M1 8h2m10 0h2M3 3l1.5 1.5m7 7L13 13M13 3l-1.5 1.5m-7 7L3 13M11 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z',
    moon: 'M13.5 9.6A5.8 5.8 0 0 1 6.4 2.5a5.8 5.8 0 1 0 7.1 7.1Z',
    play: 'm6 4 8 4-8 4V4Z',
    pause: 'M6 4v8m4-8v8',
    chevron: 'm5 7 3 3 3-3',
    x: 'm4 4 8 8m0-8-8 8',
    plus: 'M8 3v10M3 8h10',
    search: 'm11 11 3 3M6.8 11a4.2 4.2 0 1 1 0-8.4 4.2 4.2 0 0 1 0 8.4Z',
    mic: 'M8 2a2 2 0 0 1 2 2v4a2 2 0 0 1-4 0V4a2 2 0 0 1 2-2Zm-4 6a4 4 0 0 0 8 0m-4 4v3m-2 0h4'
  };
  return <svg width={size} height={size} viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d={paths[name] || paths.plus} /></svg>;
}
function Sheet({
  kind,
  close,
  children
}: {
  kind: Sheet;
  close: () => void;
  children: React.ReactNode;
}) {
  const panelRef = useRef<HTMLElement>(null);
  const dragStartY = useRef<number | null>(null);
  useEffect(() => {
    if (!kind) return;
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, [kind]);
  if (!kind) return null;
  const dismiss = close;
  return createPortal([<div key="scrim" className="scrim" onClick={close} />, <aside key="sheet" ref={panelRef} className="sheet" onClick={e => e.stopPropagation()}><div className="handle" role="button" aria-label="Close" style={{
      touchAction: 'none',
      cursor: 'grab'
    }} onPointerDown={e => {
      if (window.innerWidth >= 1024) return;
      dragStartY.current = e.clientY;
      e.currentTarget.setPointerCapture(e.pointerId);
      if (panelRef.current) panelRef.current.style.transition = 'none';
    }} onPointerMove={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (panelRef.current) {
        panelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
        panelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
      }
    }} onPointerUp={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (dy > 80) {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
          panelRef.current.style.opacity = '0';
          setTimeout(dismiss, 210);
        } else dismiss();
      } else {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%)';
          panelRef.current.style.opacity = '1';
        }
      }
      dragStartY.current = null;
      setTimeout(() => {
        if (panelRef.current) {
          panelRef.current.style.transition = '';
          panelRef.current.style.transform = '';
          panelRef.current.style.opacity = '';
        }
      }, 250);
    }} />{children}</aside>], document.body);
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
export function Satellite1Now() {
  const appRef = useRef<HTMLElement>(null);
  const headerRef = useRef<HTMLElement>(null);
  useLayoutEffect(() => {
    const measure = () => {
      const hh = headerRef.current?.offsetHeight ?? 72;
      appRef.current?.style.setProperty('--header-h', hh + 'px');
    };
    measure();
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, []);
  const [session, setSession] = useState(true);
  const [password, setPassword] = useState('');
  const [bad, setBad] = useState(false);
  const [theme, setTheme] = useState<'dark' | 'light'>('dark');
  const [tab, setTab] = useState<Tab>('NOW');
  const [settingsSubRoute, setSettingsSubRoute] = useState<string>('device-info');
  const [navLayer, setNavLayer] = useState<0 | 1>(0);
  const [sheet, setSheet] = useState<Sheet>(null);
  const [playing, setPlaying] = useState(true);
  const [listening, setListening] = useState(false);
  const [expanded, setExpanded] = useState<string | null>(null);
  const [timers, setTimers] = useState<Timer[]>([{
    id: 't1',
    label: 'Timer 1',
    total: 600,
    remaining: 581
  }]);
  const timerSeq = useRef(1);
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const [phase, setPhase] = useState<'in' | 'ha_blocked'>('in');
  const [toasts, setToasts] = useState<Toast[]>([]);
  const addToast = useCallback((t: Omit<Toast, 'id'>) => {
    setToasts(v => [...v, {
      ...t,
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 7)}`
    }]);
  }, []);
  const dismissToast = useCallback((id: string) => setToasts(v => v.filter(t => t.id !== id)), []);
  const [displayedToast, setDisplayedToast] = useState<Toast | null>(null);
  const [leavingToast, setLeavingToast] = useState<Toast | null>(null);
  const displayedRef = useRef<Toast | null>(null);
  useEffect(() => {
    const next = toasts.length ? toasts[toasts.length - 1] : null;
    const cur = displayedRef.current;
    if (next?.id === cur?.id) return;
    if (cur && next) {
      setLeavingToast(cur);
      setTimeout(() => setLeavingToast(l => l?.id === cur.id ? null : l), 300);
    }
    displayedRef.current = next;
    setDisplayedToast(next);
  }, [toasts]);
  useEffect(() => {
    if (!DEV_TOASTS) return;
    const a = setTimeout(() => addToast({
      kind: 'info',
      message: 'Satellite connected'
    }), 1500);
    const b = setTimeout(() => addToast({
      kind: 'warn',
      message: 'Microphone sensitivity is low'
    }), 3000);
    const c = setTimeout(() => addToast({
      kind: 'timer',
      message: 'Timer 1 has ended'
    }), 5000);
    return () => {
      clearTimeout(a);
      clearTimeout(b);
      clearTimeout(c);
    };
  }, [addToast]);
  const addTimer = () => {
    timerSeq.current += 1;
    const n = timerSeq.current;
    setTimers(v => [...v, {
      id: `t${n}-${Date.now()}`,
      label: `Timer ${n}`,
      total: 60,
      remaining: 60
    }]);
  };
  const removeTimer = (id: string) => setTimers(v => v.filter(t => t.id !== id));
  void addTimer;
  const [notify, setNotify] = useState(2);
  const [audioOpen, setAudioOpen] = useState(true);
  const [zones, setZones] = useState(0);
  const [loginVoice, setLoginVoice] = useState(false);
  const [unit, setUnit] = useState<'C' | 'F'>('C');
  const [offset, setOffset] = useState(0);
  const [orbFrom, setOrbFrom] = useState('#a78bfa');
  const [orbTo, setOrbTo] = useState('#818cf8');
  const handleDone = useCallback(() => setTab('NOW'), []);
  const onOrbColor = useCallback((f: string, t: string) => {
    setOrbFrom(f);
    setOrbTo(t);
  }, []);
  const orbVars = {
    '--orb-a': orbFrom,
    '--orb-b': orbTo,
    '--orb-a20': hexToRgba(orbFrom, 0.2),
    '--orb-a35': hexToRgba(orbFrom, 0.35),
    '--orb-a55': hexToRgba(orbFrom, 0.55)
  } as React.CSSProperties;
  const tempC = 21.4 + offset;
  const tempText = unit === 'C' ? `${Math.round(tempC)}°C` : `${Math.round(tempC * 9 / 5 + 32)}°F`;
  useEffect(() => {
    document.documentElement.dataset.theme = theme;
  }, [theme]);
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
  const toggleTheme = () => setTheme(v => v === 'dark' ? 'light' : 'dark');
  const logout = () => {
    setSheet(null);
    setLoginVoice(false);
    setPassword('');
    setSession(false);
  };
  if (!session) return <main className="auth auth-screen" data-theme={theme}><div className="auth-brand"><Logo cls="login-logo" /><span className="auth-eyebrow">SATELLITE1</span><h1 className="auth-headline"><span className="auth-line white">Your home.</span><span className="auth-line violet">Your voice.</span><span className="auth-line white">Your AI.</span></h1></div><div className="auth-panel"><div className="auth-form">{loginVoice ? <section className="voice-login"><div className="voice-pulse"><Icon name="mic" size={28} /></div><span className="eyebrow">LISTENING · 01:58</span><div className="challenge"><b>Hey Jarvis</b><b>Okay Nabu</b><b>Stop</b><b>Satellite</b></div><button className="text-button" onClick={() => setSession(true)}>Complete sign in</button></section> : <div className="auth-stack"><button className="voicetap-btn" onClick={() => setLoginVoice(true)}><svg className="voicetap-icon" width="20" height="20" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M8 2a2 2 0 0 1 2 2v4a2 2 0 0 1-4 0V4a2 2 0 0 1 2-2Zm-4 6a4 4 0 0 0 8 0m-4 4v3m-2 0h4" /></svg><span>Sign in with VoiceTap</span></button><div className="or"><span>or use the password</span></div><form onSubmit={e => {
            e.preventDefault();
            password ? setSession(true) : setBad(true);
          }}><input aria-label="Password" type="password" placeholder="Password" value={password} onChange={e => {
              setPassword(e.target.value);
              setBad(false);
            }} />{bad && <p className="error">Enter a password to continue.</p>}<button className="secondary wide">Sign in</button></form><button className="link" onClick={() => {
            setTab('SETUP');
            setSession(true);
          }}>First time? Set up your device</button></div>}</div></div><button className="theme-toggle auth-theme" onClick={toggleTheme} aria-label="Toggle theme"><Icon name={theme === 'dark' ? 'sun' : 'moon'} /></button><div className="login-dev">satellite1-a4c2f8</div></main>;
  if (tab === 'SETUP') return <main className="app setup-fullscreen" data-theme={theme} style={orbVars}><SetupWizard onDone={handleDone} /></main>;
  if (phase === 'ha_blocked') return <div className="ha-block-screen" data-theme={theme}>
      <div className="ha-block-card">
        <AlertTriangle size={40} className="ha-block-icon" />
        <h2 className="ha-block-title">Can't reach Home Assistant</h2>
        <p className="ha-block-body">The satellite lost its connection to Home Assistant. Check your network and HA server status.</p>
        <button className="ha-block-btn primary" onClick={() => setPhase('in')}>Retry</button>
        <button className="ha-block-btn ghost" onClick={() => {
        setPhase('in');
        setTab('SETTINGS');
        setNavLayer(1);
      }}>Open Device Settings</button>
      </div>
    </div>;
  const activeToast = displayedToast;
  return <main className="app" ref={appRef} data-theme={theme} style={orbVars}><header ref={headerRef} className={`app-header${activeToast ? ' toast-active' : ''}`}><div className="header-left"><div className="header-device-slot"><button className="device-chip" onClick={() => setSheet('device')}><span className="dot" /> <span>Living Room</span><small>192.168.4.31</small></button></div><div className="header-toast-slot" aria-live="polite">{leavingToast && <ToastPill key={`leaving-${leavingToast.id}`} toast={leavingToast} leaving onDismiss={() => {}} />}{activeToast && <ToastPill key={`active-${activeToast.id}`} toast={activeToast} onDismiss={dismissToast} />}</div></div><div className="header-actions"><button className="icon-button" onClick={() => setSheet('notice')}><Icon name="bell" /><b>{notify}</b></button><button className="theme-toggle" onClick={toggleTheme}><Icon name={theme === 'dark' ? 'sun' : 'moon'} /></button></div></header>
    <div key={tab} className="tab-content-enter">{tab === 'NOW' && <section className="now now-compact"><div className="now-left" style={{
          width: '100%'
        }}><div className="pills">{[['temp', tempText, 'Temperature'], ['humidity', '46%', 'Humidity'], ['light', '182 lx', 'Light'], ['presence', 'Still', 'Presence']].map(([key, val, label]) => <div key={key}>{key === 'presence' ? <a className="sensor" href="/presence" onClick={e => {
                e.preventDefault();
                window.history.pushState({}, '', '/presence');
                setTab('PRESENCE');
              }} style={{
                textDecoration: 'none'
              }}><strong>{val}</strong><small>{label} ›</small></a> : <button className={'sensor ' + (expanded === key ? 'active' : '')} onClick={() => setExpanded(expanded === key ? null : key)} style={{
                height: "87.29px"
              }}>{SPARK_MAP[key] && <Spark pts={SPARK_MAP[key].pts} seed={SPARK_MAP[key].seed} />}<strong style={{
                  height: "33.1px"
                }}>{val}</strong><small style={{
                  paddingTop: "0px"
                }}>{label}</small><ChevronDown size={10} className="sensor-caret" aria-hidden="true" /></button>}
            {expanded === key && (key === 'temp' ? <SensorDrawer onClose={() => setExpanded(null)}><TempPopup offset={offset} setOffset={setOffset} unit={unit} setUnit={setUnit} close={() => setExpanded(null)} /></SensorDrawer> : <SensorDrawer onClose={() => setExpanded(null)}><Calibration label={label} value={val} unit="" close={() => setExpanded(null)} /></SensorDrawer>)}</div>)}</div><VoiceOrb onColorChange={onOrbColor} /></div><div className="now-right"><section className="transcript transcript-tall">{transcript.map((line, i) => <p key={i} className={line.who}>{line.text}</p>)}</section><div className="timer-list">{timers.map(t => <div key={t.id} className={'timer-pill' + (t.remaining === 0 ? ' done' : '')}><Clock size={15} aria-hidden="true" /><span className="timer-pill-label">{t.label}</span><strong className="timer-pill-time">{fmtTime(t.remaining)}</strong><button type="button" className="timer-pill-x" aria-label={`Cancel ${t.label}`} onClick={() => removeTimer(t.id)}><X size={14} /></button></div>)}</div></div></section>}
    {tab !== 'NOW' && <ControlPanel key={tab} tab={tab} audioOpen={audioOpen} setAudioOpen={setAudioOpen} zones={zones} setZones={setZones} onDone={handleDone} onGoDevice={() => {
        setTab('SETTINGS');
        setNavLayer(1);
      }} onSimulateHaBlock={() => setPhase('ha_blocked')} settingsSubRoute={settingsSubRoute} setSettingsSubRoute={setSettingsSubRoute} />}</div>
    <nav className="side-nav" aria-label="Sections">{(['NOW', 'WAKE', 'PRESENCE', 'AUDIO', 'SETTINGS'] as Tab[]).map(item => <button key={item} className={tab === item ? 'selected' : ''} onClick={() => setTab(item)}><span>{item === 'NOW' ? 'HOME' : item}</span></button>)}{tab === 'SETTINGS' && <div className="side-sub" role="list">{SETTINGS_ROUTES.map(r => <button key={r.slug} role="listitem" className={'side-sub-item' + (settingsSubRoute === r.slug ? ' on' : '')} onClick={() => setSettingsSubRoute(r.slug)}>{r.label}</button>)}</div>}<div className="side-nav-foot" style={{
        marginTop: 'auto',
        padding: '12px 8px 88px',
        borderTop: '1px solid var(--line)',
        display: 'flex',
        flexDirection: 'column',
        gap: 2
      }}><span style={{
          fontSize: 13,
          fontWeight: 600,
          color: 'var(--text)',
          opacity: 0.85
        }}>Living Room Satellite</span><span style={{
          fontSize: 11,
          color: 'var(--muted)'
        }}>192.168.4.31 · Firmware 25.9.4</span><button type="button" onClick={logout} style={{
          marginTop: 10,
          alignSelf: 'flex-start',
          display: 'inline-flex',
          alignItems: 'center',
          gap: 8,
          background: 'transparent',
          border: '1px solid var(--line)',
          borderRadius: 999,
          padding: '6px 12px',
          minHeight: 34,
          fontSize: 12,
          fontWeight: 600,
          color: 'var(--muted)'
        }}><LogOut size={14} aria-hidden="true" /><span>Sign out</span></button></div></nav>
    <nav className="tabs" data-tab={tab} style={{
      translate: "0px -8px"
    }}><div className="tabs-track" style={{
        transform: navLayer === 0 ? 'translateX(0%)' : 'translateX(-50%)'
      }}><div className="tabs-layer tabs-main">{(['NOW', 'WAKE', 'PRESENCE', 'AUDIO', 'SETTINGS'] as Tab[]).map(item => <button key={item} className={tab === item ? 'selected' : ''} onClick={() => {
            setTab(item);
            if (item === 'SETTINGS') setNavLayer(1);else setNavLayer(0);
          }}>{item === 'NOW' ? 'HOME' : item}</button>)}</div><div className="tabs-layer tabs-sub"><button className="tabs-back" aria-label="Back to main menu" onClick={() => {
            setTab('NOW');
            setNavLayer(0);
            setSettingsSubRoute('device-info');
          }}><svg width="16" height="16" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="m10 4-5 4 5 4" /></svg></button><div className="tabs-sub-scroll">{SETTINGS_ROUTES.map(r => <button key={r.slug} className={'tabs-sub-pill' + (settingsSubRoute === r.slug ? ' on' : '')} onClick={() => setSettingsSubRoute(r.slug)}>{r.label}</button>)}</div></div></div></nav><MediaBar playing={playing} setPlaying={setPlaying} />
    <Sheet kind={sheet} close={() => setSheet(null)}>{sheet === 'device' && <DeviceSheet onLogout={logout} />}{sheet === 'notice' && <Notice notify={notify} setNotify={setNotify} />}</Sheet>
  </main>;
}
function DeviceSheet({
  onLogout
}: {
  onLogout: () => void;
}) {
  return <div className="device-sheet"><span className="eyebrow">DEVICE SWITCHER</span><h2>Living Room Satellite</h2><p className="muted">192.168.4.31 · <span className="green">●</span> LD2450 · Wi‑Fi</p><div className="peer-list">{peers.map(peer => <a href="#" key={peer.name} className={peer.state === 'OFFLINE' ? 'offline' : ''}><span className="dot" /><span><b>{peer.name}</b><small>{peer.meta}</small></span><Icon name="chevron" /></a>)}</div><button type="button" onClick={onLogout} style={{
      marginTop: 10,
      alignSelf: 'flex-start',
      display: 'inline-flex',
      alignItems: 'center',
      gap: 8,
      background: 'transparent',
      border: '1px solid var(--line)',
      borderRadius: 999,
      padding: '6px 12px',
      minHeight: 34,
      fontSize: 12,
      fontWeight: 600,
      color: 'var(--muted)'
    }}><LogOut size={14} aria-hidden="true" /><span>Sign out</span></button></div>;
}
function Notice({
  notify,
  setNotify
}: {
  notify: number;
  setNotify: (v: number) => void;
}) {
  const [filter, setFilter] = useState('All');
  return <div className="notice-sheet"><div className="sheet-top"><span className="eyebrow">NOTIFICATIONS</span><button onClick={() => setNotify(0)}>Clear all</button></div><div className="filters">{['All', 'Info', 'Warnings', 'Errors', 'Archive'].map(x => <button className={filter === x ? 'active' : ''} key={x} onClick={() => setFilter(x)}>{x}</button>)}</div>{filter !== 'Archive' && notify > 0 ? <><article className="notice info"><b>INFO</b><strong>Firmware 25.9.4 is available</strong><small>14m ago · update ready</small></article><article className="notice warn"><b>WARN ×3</b><strong>Warning from wifi</strong><small>3h ago · signal fluctuating</small></article></> : <div className="empty">Nothing here yet.</div>}</div>;
}
function ControlPanel({
  tab,
  audioOpen,
  setAudioOpen,
  zones,
  setZones,
  onDone,
  onGoDevice,
  onSimulateHaBlock,
  settingsSubRoute,
  setSettingsSubRoute
}: {
  tab: Tab;
  audioOpen: boolean;
  setAudioOpen: (v: boolean) => void;
  zones: number;
  setZones: (v: number) => void;
  onDone: () => void;
  onGoDevice: () => void;
  onSimulateHaBlock: () => void;
  settingsSubRoute: string;
  setSettingsSubRoute: (v: string) => void;
}) {
  if (tab === 'WAKE') return <WakeTab />;
  if (tab === 'PRESENCE') return <PresenceTab onGoDevice={onGoDevice} />;
  if (tab === 'AUDIO') return <AudioTab />;
  if (tab === 'SETTINGS') return <DiagnosticsTab onSimulateHaBlock={onSimulateHaBlock} subRoute={settingsSubRoute} onSubRouteChange={setSettingsSubRoute} />;
  return <SetupWizard onDone={onDone} />;
}
type WizStep = 'launcher' | 'network' | 'joining' | 'mode' | 'haconnect' | 'haactions';
const WIZ_ORDER: WizStep[] = ['launcher', 'network', 'joining', 'mode', 'haconnect', 'haactions'];
const WIZ_NETWORKS = [{
  ssid: 'Davis Home',
  rssi: -48,
  sec: 1
}, {
  ssid: 'Davis Home Guest',
  rssi: -52,
  sec: 1
}, {
  ssid: 'HP-Print-A7',
  rssi: -71,
  sec: 0
}, {
  ssid: 'NETGEAR-2G',
  rssi: -84,
  sec: 1
}];
const BAR_IDX = [{
  id: 'b0',
  i: 0
}, {
  id: 'b1',
  i: 1
}, {
  id: 'b2',
  i: 2
}, {
  id: 'b3',
  i: 3
}];
const HAC_COPY = [{
  id: 'c0',
  t: 'In your Home Assistant, go to ',
  b: false
}, {
  id: 'c1',
  t: 'Settings → Devices & Services',
  b: true
}, {
  id: 'c2',
  t: '. Your Satellite1 is waiting under ',
  b: false
}, {
  id: 'c3',
  t: 'Discovered',
  b: true
}, {
  id: 'c4',
  t: ' — tap ',
  b: false
}, {
  id: 'c5',
  t: 'Add',
  b: true
}, {
  id: 'c6',
  t: ' and follow the steps.',
  b: false
}];
const ESPHOME_LOGO = 'https://storage.googleapis.com/storage.magicpath.ai/component-assets/454802455327313920/454809714925146112/94468b329d143721ef9e4b9ca3fd46d52a305d180b732daf6ac832eab469af45.png';
const barsOf = (rssi: number) => rssi >= -55 ? 4 : rssi >= -66 ? 3 : rssi >= -77 ? 2 : rssi >= -88 ? 1 : 0;
const Bars = ({
  rssi
}: {
  rssi: number;
}) => {
  const n = barsOf(rssi);
  return <svg className="wifi-bars" viewBox="0 0 16 14" aria-hidden="true">
      {BAR_IDX.map(_mpRecord => {
      const {
        id,
        i
      } = _mpRecord;
      return <rect key={id} x={i * 4} y={11 - i * 3} width="2.6" height={3 + i * 3} rx="1" opacity={i < n ? 1 : 0.25} />;
    })}
    </svg>;
};
const LockIcon = () => <svg className="wifi-lock" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.3" aria-hidden="true">
    <rect x="2.4" y="5.2" width="7.2" height="5" rx="1.2" />
    <path d="M4 5V3.6a2 2 0 0 1 4 0V5" />
  </svg>;
function SetupWizard({
  onDone
}: {
  onDone: () => void;
}) {
  const onDoneRef = useRef(onDone);
  useEffect(() => {
    onDoneRef.current = onDone;
  }, [onDone]);
  const [step, setStep] = useState<WizStep>('launcher');
  const [prep, setPrep] = useState(true);
  const [count, setCount] = useState(3);
  const [open, setOpen] = useState<string | null>(null);
  const [pw, setPw] = useState('');
  const [err, setErr] = useState('');
  const [manual, setManual] = useState(false);
  const [mSsid, setMSsid] = useState('');
  const [ssid, setSsid] = useState('');
  const [joinPw, setJoinPw] = useState('');
  const [slow, setSlow] = useState(false);
  useEffect(() => {
    if (step !== 'launcher') return;
    setPrep(true);
    setCount(3);
    const t = setTimeout(() => setPrep(false), 1200);
    return () => clearTimeout(t);
  }, [step]);
  useEffect(() => {
    if (step !== 'launcher' || prep || count <= 0) return;
    const t = setTimeout(() => setCount(c => c - 1), 1000);
    return () => clearTimeout(t);
  }, [step, prep, count]);
  useEffect(() => {
    if (step !== 'joining') return;
    setSlow(false);
    if (/wrong/i.test(joinPw)) {
      const t = setTimeout(() => setSlow(true), 7000);
      return () => clearTimeout(t);
    }
    const t = setTimeout(() => setStep('haconnect'), 6000);
    return () => clearTimeout(t);
  }, [step, joinPw]);
  useEffect(() => {
    if (step === 'haconnect') {
      const t = setTimeout(() => setStep('haactions'), 12000);
      return () => clearTimeout(t);
    }
    if (step === 'haactions') {
      const t = setTimeout(() => onDoneRef.current(), 12000);
      return () => clearTimeout(t);
    }
  }, [step]);
  const join = (name: string, secured: boolean) => {
    if (!name.trim()) {
      setErr('Enter a network name.');
      return;
    }
    if (secured && pw.length < 8) {
      setErr('WiFi passwords are at least 8 characters.');
      return;
    }
    setErr('');
    setSsid(name);
    setJoinPw(pw);
    setStep('joining');
  };
  const idx = WIZ_ORDER.indexOf(step) + 1;
  const openHA = (e: React.MouseEvent) => e.preventDefault();
  return <section className="control setup wiz">
      <span className="eyebrow">SETUP · {String(idx).padStart(2, '0')} / 06</span>
      <Logo cls="wiz-logo" />
      <h1 className="wiz-h">Set up your Satellite1</h1>
      <div className="wiz-glass">
        {step === 'launcher' && <div className="wiz-center">
            <p className="wiz-p">Your Satellite1 is ready to meet your home.</p>
            {prep ? <div className="wiz-wait"><span className="wiz-pulse" /><span>Getting your setup ready…</span></div> : count > 0 ? <div className="wiz-wait"><span className="wiz-count">{count}</span><span>Almost there…</span></div> : <button className="primary wide" onClick={() => setStep('network')}>Setup Satellite1</button>}
          </div>}
        {step === 'network' && <div>
            <h1 className="wiz-h">Choose your WiFi network</h1>
            <p className="wiz-hint">2.4 GHz networks only - if your WiFi has separate names for 2.4 and 5 GHz, pick the 2.4 GHz one.</p>
            <ul className="wiz-nets">
              {WIZ_NETWORKS.map(n => <li key={n.ssid} className={open === n.ssid ? 'open' : ''}>
                  <button className="wiz-net" onClick={() => {
              setOpen(open === n.ssid ? null : n.ssid);
              setPw('');
              setErr('');
              setManual(false);
            }}>
                    <Bars rssi={n.rssi} /><span className="wiz-ssid">{n.ssid}</span>{n.sec ? <LockIcon /> : null}
                  </button>
                  {open === n.ssid && <form className="wiz-form" onSubmit={e => {
              e.preventDefault();
              join(n.ssid, !!n.sec);
            }}>
                      {n.sec ? <input type="password" aria-label="WiFi password" placeholder="WiFi password" value={pw} onChange={e => setPw(e.target.value)} autoFocus /> : <p className="wiz-hint">This network has no password.</p>}
                      {err && <p className="error">{err}</p>}
                      <button className="primary wide" type="submit">Join</button>
                    </form>}
                </li>)}
            </ul>
            <div className="wiz-row">
              <button className="text-button" onClick={() => {
            setManual(!manual);
            setOpen(null);
            setPw('');
            setErr('');
          }}>Join another network…</button>
              <button className="text-button">Scan again</button>
            </div>
            {manual && <form className="wiz-form" onSubmit={e => {
          e.preventDefault();
          join(mSsid, true);
        }}>
                <input aria-label="Network name" placeholder="Network name" value={mSsid} onChange={e => setMSsid(e.target.value)} />
                <input type="password" aria-label="WiFi password" placeholder="WiFi password" value={pw} onChange={e => setPw(e.target.value)} />
                {err && <p className="error">{err}</p>}
                <button className="primary wide" type="submit">Join</button>
              </form>}
          </div>}
        {step === 'joining' && <div className="wiz-center">
            <h1 className="wiz-h">Connecting to {ssid}…</h1>
            <span className="wiz-pulse lg" />
            <p className="wiz-p">Please wait while your Satellite1 connects to your network. You'll be redirected to finish setting up.</p>
            <p className="wiz-hint">If nothing happens after it connects, join your home WiFi and open <code>http://satellite1-a4c2f8.local</code></p>
            {slow && <div className="wiz-slow"><p>Still trying. If this takes much longer, the password may have been wrong - go back and re-enter it.</p><button className="secondary" onClick={() => setStep('network')}>Back</button></div>}
          </div>}
        {step === 'mode' && <div>
            <h1 className="wiz-h">How will your Satellite1 connect?</h1>
            <p className="wiz-hint">More ways to connect are on the way.</p>
            <button className="wiz-mode" disabled><span><b>Nexus AI Basestation</b><small>Connect to your 100% private Nexus AI Basestation</small></span><em className="wiz-badge">Coming soon</em></button>
            <button className="wiz-mode on" onClick={() => setStep('haconnect')}><span><b>Home Assistant</b><small>Connect to your Home Assistant server</small></span><em className="wiz-badge on">Selected</em></button>
          </div>}
        {step === 'haconnect' && <div>
            <h1 className="wiz-h">Connect to Home Assistant</h1>
            <p className="wiz-hint wiz-instruction"><span>Connect mode: <b>Home Assistant</b> · </span><button className="text-button wiz-inline" onClick={() => setStep('mode')}>Change</button></p>
            <p className="wiz-p">{HAC_COPY.map(s => s.b ? <strong key={s.id}>{s.t}</strong> : <span key={s.id}>{s.t}</span>)}</p>
            <div className="wiz-disc">
              <div className="wiz-disc-header">
                <span className="wiz-disc-title">Discovered</span>
              </div>
              <div className="wiz-disc-card">
                <button className="wiz-disc-dots" aria-label="More options">···</button>
                <img className="wiz-disc-logo" src={ESPHOME_LOGO} alt="ESPHome" />
                <span className="wiz-disc-name">Satellite1 A4C2F8 (satellite1-a4c2f8)</span>
                <span className="wiz-disc-int">ESPHome</span>
                <div className="wiz-disc-actions">
                  <button className="wiz-disc-ignore">Ignore</button>
                  <button className="wiz-disc-add">Add</button>
                </div>
              </div>
            </div>
            <a href="homeassistant://navigate/config/integrations" className="primary wide wiz-btn" onClick={openHA}>Open Home Assistant</a>
            <button className="secondary wide" onClick={() => setStep('haactions')}>I've added it in Home Assistant</button>
            <a href="http://homeassistant.local:8123/config/integrations" className="link wiz-center-link" onClick={openHA}>No Home Assistant app? Open it in your browser instead.</a>
            <div className="wiz-wait"><span className="wiz-pulse" /><span>Waiting for Home Assistant… this page continues on its own once your Satellite1 is added.</span></div>
          </div>}
        {step === 'haactions' && <div>
            <h1 className="wiz-h">One last Home Assistant setting</h1>
            <p className="wiz-p">This setting lets your Satellite1 speak announcements and route audio through Home Assistant.</p>
            <ol className="wiz-steps">
              <li><span className="wiz-step-body"><span>In Home Assistant, open </span><strong>Settings {'\u203A'} Devices &amp; services {'\u203A'} ESPHome</strong><span>.</span></span></li>
              <li><span className="wiz-step-body"><span>Tap the </span><span className="glyph" aria-label="cog">{'\u2699'}</span><span> cog next to this device — </span><strong>Satellite1 A4C2F8</strong><span>, unless you renamed it.</span></span></li>
              <li><span className="wiz-step-body"><span>Tick </span><strong>{'\u201C'}Allow the device to perform Home Assistant actions{'\u201D'}</strong><span>, then Submit.</span></span></li>
            </ol>
            <a href="homeassistant://navigate/config/integrations/integration/esphome" className="primary wide wiz-btn" onClick={openHA}>Open Home Assistant</a>
            <div className="wiz-wait"><span className="wiz-pulse" /><span>Waiting for the setting… this page continues on its own once it's allowed.</span></div>
            <button className="text-button wiz-center-link" onClick={onDone}>Skip for now</button>
          </div>}
      </div>
      <div className="login-dev">satellite1-a4c2f8</div>
    </section>;
}
const Logo = ({
  cls = 'login-logo'
}: {
  cls?: string;
}) => <svg className={cls} viewBox="0 0 79.375 79.375" fill="none" stroke="currentColor" strokeLinecap="square" aria-hidden="true">
    <g transform="matrix(1.6754,0,0,1.6754,84.9754,-16.4554)" strokeWidth="1.31">
      <path d="m -45.44,31.52 11,-10.96 11,10.96 v 16.25 l -5.82,.01" />
      <path d="m -27.49,20.19 10.94,9.29 v 18.3 l 8.35,.03 V 29.29 l -10.94,-9.66 -1.92,1.73" />
      <path strokeWidth="1.36" d="m -32.25,47.75 c 0,-7.27 -5.89,-13.42 -13.16,-13.42 h 0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -36.31,47.73 c .01,-.12 .01,-.12 .01,-.24 0,-5.03 -4.08,-9.11 -9.11,-9.11 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -40.36,47.8 c .01,-.12 .01,-.18 .01,-.31 0,-2.8 -2.27,-5.06 -5.06,-5.06 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -45.41,46.48 a 1.01,1.01 0 0 0 -.15,.01 v 1.38 h 1.09 a 1.01,1.01 0 0 0 .07,-.37 1.01,1.01 0 0 0 -1.01,-1.01 z" />
    </g>
  </svg>;