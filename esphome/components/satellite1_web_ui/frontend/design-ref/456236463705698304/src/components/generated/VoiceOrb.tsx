import { useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { Mic, MicOff } from 'lucide-react';
const PARTICLE_COUNT = 720;
const TWO_PI = Math.PI * 2;
const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));
const STATIC_TIME = 1.7;
type Rgb = [number, number, number];
function toRgb(color: string): Rgb {
  if (color.startsWith('hsl')) {
    const m = color.match(/[\d.]+/g) || ['0', '0', '0'];
    const h = +m[0] % 360 / 360,
      s = +m[1] / 100,
      l = +m[2] / 100;
    const q = l < 0.5 ? l * (1 + s) : l + s - l * s,
      p = 2 * l - q;
    const f = (t: number) => {
      if (t < 0) t += 1;
      if (t > 1) t -= 1;
      if (t < 1 / 6) return p + (q - p) * 6 * t;
      if (t < 1 / 2) return q;
      if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
      return p;
    };
    return [f(h + 1 / 3) * 255, f(h) * 255, f(h - 1 / 3) * 255];
  }
  const h = color.replace('#', '');
  const n = parseInt(h.length === 3 ? h.split('').map(c => c + c).join('') : h, 16);
  return [n >> 16 & 255, n >> 8 & 255, n & 255];
}
const mixRgb = (a: Rgb, b: Rgb, m: number): Rgb => [a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m, a[2] + (b[2] - a[2]) * m];
const approach = (cur: number, target: number, rate: number, dt: number) => cur + (target - cur) * (1 - Math.exp(-rate * dt));
interface SpherePoint {
  x: number;
  y: number;
  z: number;
  ringFrac: number;
  seed: number;
  tone: number;
}
function buildSphere(count: number): SpherePoint[] {
  const points: SpherePoint[] = [];
  for (let i = 0; i < count; i++) {
    const y = 1 - i / (count - 1) * 2;
    const r = Math.sqrt(1 - y * y);
    const th = GOLDEN_ANGLE * i;
    points.push({
      x: Math.cos(th) * r,
      y,
      z: Math.sin(th) * r,
      ringFrac: i * 0.61803398875 % 1,
      seed: i * 0.7548776662 % 1 * TWO_PI,
      tone: i * 0.5436890126 % 1
    });
  }
  return points;
}
type OrbState = 'idle' | 'connecting' | 'listening' | 'thinking' | 'speaking' | 'error' | 'disabled';
const ORB_STATES: OrbState[] = ['idle', 'connecting', 'listening', 'thinking', 'speaking', 'error', 'disabled'];
function stateMotion(s: OrbState): 'ripple' | 'pulse' | 'flow' | null {
  if (s === 'speaking') return 'ripple';
  if (s === 'listening') return 'pulse';
  if (s === 'thinking') return 'flow';
  return null;
}
function stateEnergy(s: OrbState, t: number): number {
  if (s === 'speaking') return 0.5 + 0.4 * Math.sin(t * 6.2);
  if (s === 'listening') return 0.4 + 0.35 * Math.sin(t * 4.5);
  if (s === 'thinking') return 0.3 + 0.25 * Math.sin(t * 3.1);
  return 0.1;
}
function createStateMix(initial: OrbState) {
  const weights: Record<OrbState, number> = {
    idle: 0,
    connecting: 0,
    listening: 0,
    thinking: 0,
    speaking: 0,
    error: 0,
    disabled: 0
  };
  weights[initial] = 1;
  return {
    update(target: OrbState, dt: number): Record<OrbState, number> {
      for (const s of ORB_STATES) weights[s] = approach(weights[s], s === target ? 1 : 0, 3.5, dt);
      return {
        ...weights
      };
    }
  };
}
const ERROR_FROM_RGB = toRgb('#ff4444');
const ERROR_TO_RGB = toRgb('#ff8800');
function ParticlesOrb({
  state,
  size,
  colorFrom,
  colorTo,
  paused = false
}: {
  state: OrbState;
  size: number;
  colorFrom: string;
  colorTo: string;
  paused?: boolean;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const stateRef = useRef(state);
  const pausedRef = useRef(paused);
  const colorRef = useRef({
    from: colorFrom,
    to: colorTo
  });
  useEffect(() => {
    stateRef.current = state;
    pausedRef.current = paused;
    colorRef.current = {
      from: colorFrom,
      to: colorTo
    };
  });
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const points = buildSphere(PARTICLE_COUNT);
    const center = size / 2;
    const baseRadius = center * 0.62;
    const reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const stateMix = createStateMix(stateRef.current);
    let t = reduce ? STATIC_TIME : 0;
    let angleY = 0,
      connectingPhase = 0,
      levelS = 0,
      raf = 0;
    const angleX = 0.32;
    let last: number | null = null;
    let running = true;
    const render = (dt: number, isStatic = false) => {
      const st = stateRef.current;
      const easeDt = isStatic ? 60 : dt;
      const w = stateMix.update(st, easeDt);
      let ripple = 0,
        pulse = 0,
        flow = 0;
      for (const s of ORB_STATES) {
        const k = stateMotion(s);
        if (k === 'ripple') ripple += w[s];else if (k === 'pulse') pulse += w[s];else if (k === 'flow') flow += w[s];
      }
      const wIdle = w.idle,
        wConn = w.connecting,
        wError = w.error,
        wDisabled = w.disabled;
      const motionScale = 1 - wDisabled * 0.96;
      const rawLevel = isStatic ? stateEnergy(st, t) : stateEnergy(st, t) * 0.6 + 0.1;
      levelS = approach(levelS, rawLevel, 9, easeDt);
      const level = levelS;
      const spin = (0.14 + ripple * (0.9 + level * 1.6) + flow * 0.4 + wConn * 0.3) * motionScale;
      angleY += dt * spin;
      connectingPhase = (connectingPhase + dt * 1.1) % TWO_PI;
      const breathe = 0.05 * (0.25 + wIdle * 0.75) * Math.sin(t * 1.1) * motionScale;
      const conv = pulse * (0.22 + 0.12 * Math.sin(t * 2.8 * (spin + 1)));
      const expand = flow * (0.08 - level * 0.32);
      const radius = baseRadius * (1 + breathe + level * 0.16 + expand - conv);
      const from = mixRgb(toRgb(colorRef.current.from), ERROR_FROM_RGB, wError);
      const to = mixRgb(toRgb(colorRef.current.to), ERROR_TO_RGB, wError);
      const shakeAmp = wError * radius * 0.05 * motionScale;
      const shakeX = shakeAmp * (Math.sin(t * 26) * 0.5 + Math.sin(t * 15.7));
      const shakeY = shakeAmp * (Math.sin(t * 22) * 0.5 + Math.sin(t * 13.1));
      const idleAmp = wIdle * radius * 0.055 * motionScale;
      const jitterAmp = (flow + wError * 0.7) * radius * (0.015 + level * 0.005) * motionScale;
      const rippleAmp = ripple * (0.045 + level * 0.24);
      const pulseAmp = pulse * 0.16;
      const alphaScale = 1 - wDisabled * 0.35;
      const cosY = Math.cos(angleY),
        sinY = Math.sin(angleY),
        cosX = Math.cos(angleX),
        sinX = Math.sin(angleX);
      ctx.clearRect(0, 0, size, size);
      ctx.globalCompositeOperation = !isStatic && ripple + pulse + flow > 0.5 ? 'lighter' : 'source-over';
      for (let i = 0; i < points.length; i++) {
        const p = points[i];
        const x1 = p.x * cosY - p.z * sinY;
        const z1 = p.x * sinY + p.z * cosY;
        const y1 = p.y * cosX - z1 * sinX;
        const z2 = p.y * sinX + z1 * cosX;
        const depth = (z2 + 1) / 2;
        const perspective = 0.65 + depth * 0.45;
        let pr = radius;
        if (rippleAmp > 0.002) pr *= 1 + rippleAmp * Math.sin(p.y * 4.5 - t * 6.5);
        if (pulseAmp > 0.002) pr *= 1 - pulseAmp * (0.5 + 0.5 * Math.sin(p.ringFrac * TWO_PI + t * 3.1));
        let ox = shakeX,
          oy = shakeY;
        if (idleAmp > 0.01) {
          ox += idleAmp * (Math.sin(t * 0.55 + p.seed * 3.7) + 0.5 * Math.sin(t * 1.3 + p.seed * 1.3));
          oy += idleAmp * (Math.cos(t * 0.62 + p.seed * 2.9) + 0.5 * Math.sin(t * 1.05 + p.seed * 5.1));
        }
        if (jitterAmp > 0.01) {
          ox += jitterAmp * Math.sin(t * 14 + p.seed * 9.3);
          oy += jitterAmp * Math.cos(t * 17 + p.seed * 6.1);
        }
        const sx = center + x1 * pr * perspective + ox;
        const sy = center + y1 * pr * perspective + oy;
        let alpha = (0.12 + depth * 0.78) * alphaScale;
        let dot = 0.6 + depth * 1.5;
        let X = sx,
          Y = sy;
        if (wConn > 0.004) {
          const ringAngle = 1 / points.length * TWO_PI * connectingPhase + 0.05 * Math.sin(t * 1.3 + p.seed);
          const ringR = center * (0.58 + 0.13 * p.ringFrac) * (1 + 0.05 * Math.sin(t + p.seed * 1.7));
          X = sx + (center + Math.cos(ringAngle) * ringR - sx) * wConn;
          Y = sy + (center + Math.sin(ringAngle) * ringR - sy) * wConn;
          alpha += (0.35 + p.tone * 0.5 - alpha) * wConn;
          dot += (0.75 + p.tone * 0.9 - dot) * wConn;
        }
        const cr = from[0] + (to[0] - from[0]) * p.tone;
        const cg = from[1] + (to[1] - from[1]) * p.tone;
        const cb = from[2] + (to[2] - from[2]) * p.tone;
        ctx.beginPath();
        ctx.fillStyle = `rgba(${cr | 0},${cg | 0},${cb | 0},${alpha.toFixed(3)})`;
        ctx.arc(X, Y, dot, 0, TWO_PI);
        ctx.fill();
      }
      ctx.globalCompositeOperation = 'source-over';
    };
    if (reduce) {
      render(0, true);
      return;
    }
    const frame = (now: number) => {
      raf = 0;
      const dt = last === null || pausedRef.current ? 0 : Math.min((now - last) / 1000, 0.1);
      last = now;
      t += dt;
      render(dt);
      if (running) raf = requestAnimationFrame(frame);
    };
    const observer = new IntersectionObserver(([entry]) => {
      running = entry.isIntersecting;
      if (running && raf === 0) {
        last = null;
        raf = requestAnimationFrame(frame);
      } else if (!running && raf !== 0) {
        cancelAnimationFrame(raf);
        raf = 0;
      }
    }, {
      threshold: 0
    });
    observer.observe(canvas);
    raf = requestAnimationFrame(frame);
    return () => {
      running = false;
      if (raf !== 0) cancelAnimationFrame(raf);
      observer.disconnect();
    };
  }, [size]);
  return <canvas ref={canvasRef} style={{
    width: size,
    height: size,
    display: 'block'
  }} aria-hidden="true" />;
}
type CycleState = 'idle' | 'listening' | 'processing' | 'speaking';
const cycle: {
  state: CycleState;
  orb: OrbState;
  label: string;
  ms: number;
}[] = [{
  state: 'idle',
  orb: 'idle',
  label: 'READY',
  ms: 3800
}, {
  state: 'listening',
  orb: 'listening',
  label: 'LISTENING…',
  ms: 3600
}, {
  state: 'processing',
  orb: 'thinking',
  label: 'THINKING…',
  ms: 2600
}, {
  state: 'speaking',
  orb: 'speaking',
  label: 'SPEAKING…',
  ms: 4200
}];
const ORB_PRESETS = [{
  label: 'Adaptive',
  from: '#a78bfa',
  to: '#818cf8'
}, {
  label: 'Nebula',
  from: '#c084fc',
  to: '#818cf8'
}, {
  label: 'Arctic',
  from: '#38bdf8',
  to: '#0ea5e9'
}, {
  label: 'Aurora',
  from: '#34d399',
  to: '#059669'
}, {
  label: 'Solar',
  from: '#fbbf24',
  to: '#f59e0b'
}, {
  label: 'Rose',
  from: '#fb7185',
  to: '#e11d48'
}, {
  label: 'Ember',
  from: '#fb923c',
  to: '#ef4444'
}, {
  label: 'Sapphire',
  from: '#60a5fa',
  to: '#2563eb'
}];
const hueFrom = (h: number) => `hsl(${h}, 70%, 70%)`;
const hueTo = (h: number) => `hsl(${(h + 40) % 360}, 65%, 55%)`;
const sizeFor = (w: number) => w >= 1024 ? 220 : w >= 640 ? 200 : 180;
export function VoiceOrb({
  onColorChange
}: {
  onColorChange?: (from: string, to: string) => void;
} = {}) {
  const [i, setI] = useState(0);
  const [colorPanel, setColorPanel] = useState(false);
  useEffect(() => {
    if (colorPanel) document.body.classList.add('has-drawer');else document.body.classList.remove('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, [colorPanel]);
  const [sel, setSel] = useState('Adaptive');
  const [customHue, setCustomHue] = useState(290);
  const [ledHue, setLedHue] = useState(290);
  const [ledBrightness, setLedBrightness] = useState(80);
  const [micMuted, setMicMuted] = useState(false);
  const [orbFrom, setOrbFrom] = useState(ORB_PRESETS[0].from);
  const [orbTo, setOrbTo] = useState(ORB_PRESETS[0].to);
  const [orbSize, setOrbSize] = useState(180);
  const orbPanelRef = useRef<HTMLElement>(null);
  const orbDragStartY = useRef<number | null>(null);
  useEffect(() => {
    onColorChange?.(orbFrom, orbTo);
  }, [orbFrom, orbTo, onColorChange]);
  useEffect(() => {
    const id = setTimeout(() => setI(v => (v + 1) % cycle.length), cycle[i].ms);
    return () => clearTimeout(id);
  }, [i]);
  useEffect(() => {
    const on = () => setOrbSize(sizeFor(window.innerWidth));
    on();
    window.addEventListener('resize', on);
    return () => window.removeEventListener('resize', on);
  }, []);
  const cur = cycle[i];
  const pickCustom = (h: number) => {
    setCustomHue(h);
    setOrbFrom(hueFrom(h));
    setOrbTo(hueTo(h));
  };
  return <div className="orb-wrap">
    <button onClick={() => setI(v => (v + 1) % cycle.length)} aria-label={'Voice assistant: ' + cur.label} style={{
      background: 'transparent',
      padding: 0
    }}>
      <ParticlesOrb state={micMuted ? 'idle' : cur.orb} size={orbSize} colorFrom={micMuted ? '#ef4444' : orbFrom} colorTo={micMuted ? '#b91c1c' : orbTo} paused={micMuted} />
    </button>
    <div className="orb-meta">
      <span className="orb-label" aria-live="polite" style={micMuted ? {
        color: '#ef4444'
      } : undefined}>{micMuted ? 'MIC MUTED' : cur.label}</span>
      <div style={{
        display: 'flex',
        alignItems: 'center',
        gap: 6
      }}>
      <button className="orb-hint customize-btn" onClick={() => setColorPanel(true)} aria-label="Customize orb color"><span aria-hidden="true" style={{
            width: 15,
            height: 15,
            borderRadius: '50%',
            flexShrink: 0,
            background: 'conic-gradient(red,yellow,lime,cyan,blue,magenta,red)',
            boxShadow: 'inset 0 0 0 1px rgba(0,0,0,.08)'
          }} /><span>Customize</span></button>
      <button className="orb-hint customize-btn" onClick={() => setMicMuted(m => !m)} aria-pressed={micMuted} aria-label={micMuted ? 'Unmute microphone' : 'Mute microphone'} style={{
          opacity: 1,
          color: micMuted ? '#ef4444' : 'var(--text)',
          background: micMuted ? 'rgba(239,68,68,.12)' : 'transparent',
          width: 44,
          justifyContent: 'center'
        }}>{micMuted ? <MicOff size={18} strokeWidth={2.2} /> : <Mic size={18} />}</button>
      </div>
    </div>
    {colorPanel && createPortal([<div key="scrim" className="scrim" onClick={() => setColorPanel(false)} />, <aside key="panel" ref={orbPanelRef} className="sheet orb-sheet color-panel" onClick={e => e.stopPropagation()} role="dialog" aria-label="Orb color">
        <div className="handle" role="button" aria-label="Close" style={{
        touchAction: 'none',
        cursor: 'grab'
      }} onPointerDown={e => {
        if (window.innerWidth >= 1024) return;
        orbDragStartY.current = e.clientY;
        e.currentTarget.setPointerCapture(e.pointerId);
        if (orbPanelRef.current) orbPanelRef.current.style.transition = 'none';
      }} onPointerMove={e => {
        if (orbDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - orbDragStartY.current);
        if (orbPanelRef.current) {
          orbPanelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
          orbPanelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
        }
      }} onPointerUp={e => {
        if (orbDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - orbDragStartY.current);
        const dismiss = () => setColorPanel(false);
        if (dy > 80) {
          if (orbPanelRef.current) {
            orbPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
            orbPanelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
            orbPanelRef.current.style.opacity = '0';
            setTimeout(dismiss, 210);
          } else dismiss();
        } else if (orbPanelRef.current) {
          orbPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          orbPanelRef.current.style.transform = 'translateX(-50%)';
          orbPanelRef.current.style.opacity = '1';
        }
        orbDragStartY.current = null;
        setTimeout(() => {
          if (orbPanelRef.current) {
            orbPanelRef.current.style.transition = '';
            orbPanelRef.current.style.transform = '';
            orbPanelRef.current.style.opacity = '';
          }
        }, 250);
      }} />
        <span className="eyebrow">ORB COLOR</span>
        <h2>Pick a glow.</h2>
        <p className="muted">Also updates your device LED ring.</p>
        <div className="swatches">
          {ORB_PRESETS.map(s => <button key={s.label} className={'swatch ' + (sel === s.label ? 'on' : '')} aria-pressed={sel === s.label} onClick={() => {
          setSel(s.label);
          setOrbFrom(s.from);
          setOrbTo(s.to);
          setColorPanel(false);
        }}><span className="swatch-dot" style={{
            background: `linear-gradient(135deg, ${s.from}, ${s.to})`
          }} /><small>{s.label}</small></button>)}
          <button className={'swatch ' + (sel === 'Custom' ? 'on' : '')} aria-pressed={sel === 'Custom'} onClick={() => {
          setSel('Custom');
          pickCustom(customHue);
        }}><span className="swatch-dot" style={{
            background: 'conic-gradient(red,yellow,lime,cyan,blue,magenta,red)'
          }} /><small>Custom</small></button>
        </div>
        {sel === 'Custom' && <label className="hue-label"><small className="muted" style={{
          display: 'block',
          marginBottom: 6
        }}>Orb / theme color</small><span>Hue · {customHue}°</span><input className="hue" type="range" min={0} max={359} value={customHue} onChange={e => pickCustom(Number(e.target.value))} aria-label="Custom hue" /></label>}
        {sel === 'Custom' && <label className="hue-label"><small className="muted" style={{
          display: 'block',
          marginBottom: 6
        }}>LED Ring color</small><span>Hue · {ledHue}°</span><input className="hue" type="range" min={0} max={359} value={ledHue} onChange={e => setLedHue(Number(e.target.value))} aria-label="LED ring hue" /></label>}
        {sel === 'Custom' && <label className="hue-label"><small className="muted" style={{
          display: 'block',
          marginBottom: 6
        }}>LED Brightness</small><span>{ledBrightness}%</span><input className="hue led-bright" type="range" min={0} max={100} value={ledBrightness} onChange={e => setLedBrightness(Number(e.target.value))} aria-label="LED brightness" /></label>}
        {sel === 'Custom' && <button className="primary wide" onClick={() => setColorPanel(false)}>Done</button>}
      </aside>], document.body)}
  </div>;
}