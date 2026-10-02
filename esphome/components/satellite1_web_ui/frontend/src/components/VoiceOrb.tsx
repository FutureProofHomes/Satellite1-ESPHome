import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { HINTS, TEXT } from '../copy.js';
import { entity, pathFor, post } from '../lib/device.js';
import type { Ctx, Orb } from '../ctx';
import { Mic, MicOff } from '../icons';
import { hexToRgb, hsvToRgb, isOn, orbView, pctTo255, rgbHue, ringPct } from '../lib/orb.js';
import { tipDone } from '../lib/tips.js';
import { Drawer, Presence } from './Drawer';
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
/**
 * A just-written value, held over the stale state that follows it. A control writes and then keeps
 * rendering from the entity or poll behind it, which does not know about the write for up to a poll
 * interval - so a slider's thumb snapped back to the old value and jumped forward again when the
 * echo landed (reported from Safari on the phone, September 2026, but present everywhere). The held
 * value wins until the echo comes within `tol` of it, which absorbs rounding on values that
 * round-trip through 0-255, or until five seconds pass - the escape for a write the device refused,
 * where the stale value is the truth. It re-renders on hold so callers need no state of their own.
 */
export function useHeld(value: number, tol: number): [number, (v: number) => void] {
  const held = useRef<{ v: number; at: number } | null>(null);
  const [, bump] = useState(0);
  if (held.current && (Math.abs(value - held.current.v) <= tol || Date.now() - held.current.at > 5000)) held.current = null;
  return [held.current ? held.current.v : value, v => {
    held.current = { v, at: Date.now() };
    bump(n => n + 1);
  }];
}

/**
 * The picker's LED sliders: they follow the drag on screen and write once, on the native change at
 * release, because a write per input event would queue a request per pixel of drag (MSlider has the
 * same rule). preact/compat turns onChange into input events, so the commit listens for it directly.
 */
function LedRange({
  label,
  text,
  value,
  max,
  tol,
  className,
  ariaLabel,
  onCommit
}: {
  label: string;
  text: (v: number) => string;
  value: number;
  max: number;
  tol: number;
  className?: string;
  ariaLabel: string;
  onCommit: (v: number) => void;
}) {
  const [held, hold] = useHeld(value, tol);
  const [draft, setDraft] = useState<number | null>(null);
  const shown = draft ?? held;
  const input = useRef<HTMLInputElement>(null);
  const commit = useRef(onCommit);
  commit.current = onCommit;
  useEffect(() => {
    const el = input.current;
    if (!el) return;
    const on = () => {
      const v = Number(el.value);
      hold(v);
      setDraft(null);
      commit.current(v);
    };
    el.addEventListener('change', on);
    return () => el.removeEventListener('change', on);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  return <label className="hue-label"><small className="muted hue-cap">{label}</small><span>{text(shown)}</span><input ref={input} className={'hue' + (className ? ' ' + className : '')} type="range" min={0} max={max} value={shown} onChange={e => setDraft(Number(e.currentTarget.value))} aria-label={ariaLabel} /></label>;
}
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
// The canvas keeps room the particles reach only at their widest - the speaking ripple and the
// connecting ring both stop about 13% short of its edge - so that much of it may overlap the pills
// above and the label below instead of standing as empty space around the sphere. It is transparent
// there and lets clicks through, so it covers nothing.
const ORB_OVERLAP = 0.1;
// The tap target: the resting sphere's diameter (ParticlesOrb's baseRadius, 0.62 of the half-size),
// so a tap counts where the orb is drawn and not in the transparent margin around it.
const ORB_TAP = 0.62;
const TAPPABLE = ['idle', 'listening', 'thinking', 'speaking'];
/** How long each tip under a resting orb stays, and its crossfade (.orb-tips in home.css). */
const TIP_MS = 7000;
const TIP_FADE_MS = 450;

/**
 * The tip a resting orb shows, moving on every TIP_MS while the page is in view, and the one it is
 * fading out from. The count carries across conversations, so the rotation resumes where it was
 * rather than starting over each time the orb goes back to rest.
 */
function useTip(tips: string[], resting: boolean) {
  const [n, setN] = useState(0);
  const on = resting && tips.length > 0;
  useEffect(() => {
    if (!on) return;
    const id = setInterval(() => {
      if (!document.hidden) setN(k => k + 1);
    }, TIP_MS);
    return () => clearInterval(id);
  }, [on]);
  const tip = on ? tips[n % tips.length] : null;
  const last = useRef<string | null>(null);
  const [leaving, setLeaving] = useState<string | null>(null);
  // Before paint, so the outgoing tip never misses a frame.
  useLayoutEffect(() => {
    const was = last.current;
    last.current = tip;
    if (!was || !tip || was === tip) return;
    setLeaving(was);
    const id = setTimeout(() => setLeaving(null), TIP_FADE_MS);
    return () => clearTimeout(id);
  }, [tip]);
  return { tip, leaving: tip ? leaving : null };
}

/**
 * The assistant's orb: its state follows the phase the Home tab polls, its colours are the shell's
 * persisted `orb`, and the picker writes the LED ring and keeps it lit while open. `onOrbColor`
 * fires only on a person's pick, so mounting never overwrites the saved choice. At rest, `tips`
 * (orbTips) take the label's place; the label stays in the accessibility tree, so a screen reader
 * hears the state change and not every tip.
 */
export function VoiceOrb({
  ctx,
  phase,
  orb,
  onOrbColor,
  onTalk,
  tips = []
}: {
  ctx: Ctx;
  phase: number | undefined;
  orb: Orb;
  onOrbColor: (from: string, to: string) => void;
  onTalk?: () => void;
  tips?: string[];
}) {
  const [colorPanel, setColorPanel] = useState(false);
  const [sel, setSel] = useState(() => ORB_PRESETS.find(p => p.from === orb.a && p.to === orb.b)?.label ?? 'Custom');
  const [customHue, setCustomHue] = useState(() => Number(/^hsl\(\s*([\d.]+)/.exec(orb.a)?.[1] ?? 290));
  const [orbSize, setOrbSize] = useState(180);
  useEffect(() => {
    const on = () => setOrbSize(sizeFor(window.innerWidth));
    on();
    window.addEventListener('resize', on);
    return () => window.removeEventListener('resize', on);
  }, []);
  const mute = entity(ctx, 'mute_mics');
  const muted = isOn(mute);
  const view = orbView(phase, ctx.connected, muted);
  // The sphere is the action button's press-to-talk (common/web_ui_assist.yaml), on firmware that
  // has it - `onTalk` is the Home tab's, which knows whose pipeline the open tab is. Not while muted,
  // and not while Home Assistant is gone ("Not ready") or the page has lost the device - the device
  // would refuse, and a tap that does nothing reads as a broken orb.
  const talk = !!onTalk && !view.muted && TAPPABLE.includes(view.state);
  const running = view.state !== 'idle';
  const shown = useTip(tips, !view.muted && view.state === 'idle');
  const ring = entity(ctx, 'ring');
  const hasRing = !!ring;
  // The ring is lit for as long as the picker is open, so each pick shows on the device as it is
  // made, and goes dark when the picker closes by any route - Done, the scrim, Escape, the handle,
  // or leaving the page (owner, October 2026). Requests are single-flight, so a slider commit made
  // just before Done still lands ahead of the turn_off.
  useEffect(() => {
    if (!colorPanel || !hasRing) return;
    post(pathFor(ctx, 'ring', 'turn_on'));
    return () => {
      post(pathFor(ctx, 'ring', 'turn_off'));
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [colorPanel, hasRing]);
  const ringColor = ring?.color || { r: 255, g: 255, b: 255 };
  // Every colour pick posts turn_on with the colour, so picking on a dark ring lights it in that
  // colour - one gesture instead of toggle-then-pick (owner, September 2026).
  const ringOn = (params: Record<string, number>) => post(`${pathFor(ctx, 'ring', 'turn_on')}?${new URLSearchParams(params as unknown as Record<string, string>)}`);
  const ringRgb = ([r, g, b]: number[]) => ringOn({ r, g, b });
  const pickCustom = (h: number) => {
    setCustomHue(h);
    onOrbColor(hueFrom(h), hueTo(h));
  };
  return <div className="orb-wrap">
    <div className="orb-stage" style={{
      margin: `${-Math.round(orbSize * ORB_OVERLAP)}px 0`,
      pointerEvents: 'none'
    }}>
      <ParticlesOrb state={view.state} size={orbSize} colorFrom={view.muted ? '#ef4444' : orb.a} colorTo={view.muted ? '#b91c1c' : orb.b} paused={view.muted} />
      {talk && <button className="orb-tap" style={{
        width: Math.round(orbSize * ORB_TAP),
        height: Math.round(orbSize * ORB_TAP)
      }} aria-label={running ? TEXT.orb_stop : TEXT.orb_talk} title={running ? TEXT.orb_stop : TEXT.orb_talk} onClick={onTalk} />}
    </div>
    <div className="orb-meta">
      <div className="orb-say">
        <span className={'orb-label' + (shown.tip ? ' tipped' : '') + (view.muted ? ' muted-on' : '')} aria-live="polite">{view.label}</span>
        {shown.tip && <span className="orb-tips">{shown.leaving && <span key={'out' + shown.leaving} className="out" aria-hidden="true">{shown.leaving}</span>}<span key={shown.tip} className="in">{shown.tip}</span></span>}
      </div>
      <div className="orb-tools">
      <button className="orb-hint customize-btn" onClick={() => {
          tipDone('color');
          setColorPanel(true);
        }} aria-label="Customize orb color"><span aria-hidden="true" className="customize-dot" /><span>Customize</span></button>
      {mute && <button className="orb-hint customize-btn mute-btn" onClick={() => {
          tipDone('mute');
          post(pathFor(ctx, 'mute_mics', muted ? 'turn_off' : 'turn_on'));
        }} aria-pressed={muted} aria-label={muted ? 'Unmute microphone' : 'Mute microphone'} title={HINTS.mute}>{muted ? <MicOff size={18} strokeWidth={2.2} /> : <Mic size={18} />}</button>}
      </div>
    </div>
    <Presence>{colorPanel && <Drawer label="Orb color" onClose={() => setColorPanel(false)} className="orb-sheet color-panel">
        <span className="eyebrow">ORB COLOR</span>
        <h2>Pick a glow.</h2>
        {ring && <p className="muted">Also updates your device LED ring.</p>}
        <div className="swatches">
          {ORB_PRESETS.map(s => <button key={s.label} className={'swatch ' + (sel === s.label ? 'on' : '')} aria-pressed={sel === s.label} onClick={() => {
          setSel(s.label);
          onOrbColor(s.from, s.to);
          if (ring) ringRgb(hexToRgb(s.from));
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
        {sel === 'Custom' && <label className="hue-label"><small className="muted hue-cap">Orb / theme color</small><span>Hue · {customHue}°</span><input className="hue" type="range" min={0} max={359} value={customHue} onChange={e => pickCustom(Number(e.target.value))} aria-label="Custom hue" /></label>}
        {sel === 'Custom' && ring && <LedRange label="LED Ring color" text={v => `Hue · ${v}°`} value={rgbHue(ringColor.r, ringColor.g, ringColor.b)} max={359} tol={2} ariaLabel="LED ring hue" onCommit={h => ringRgb(hsvToRgb(h, 1))} />}
        {sel === 'Custom' && ring && <LedRange label="LED Brightness" text={v => `${v}%`} value={ringPct(ring)} max={100} tol={1} className="led-bright" ariaLabel="LED brightness" onCommit={v => v === 0 ? post(pathFor(ctx, 'ring', 'turn_off')) : ringOn({
          brightness: pctTo255(v)
        })} />}
        <button className="primary wide" onClick={() => setColorPanel(false)}>Done</button>
      </Drawer>}</Presence>
  </div>;
}