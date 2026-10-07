import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { HINTS, RING, TEXT } from '../copy.js';
import { entity, pathFor, post } from '../lib/device.js';
import type { Ctx, Orb } from '../ctx';
import { ChevronLeft, ChevronRight, Mic, MicOff, X } from '../icons';
import { errorHoldUntil, errorSeen, isOn, ORB_LABEL, orbState, orbView } from '../lib/orb.js';
import { ringApi, ringRgb } from '../lib/ring.js';
import { M_LISTEN, STYLES } from '../lib/ringfx.js';
import { tipDone } from '../lib/tips.js';
import { Drawer, Presence } from './Drawer';
import { MomentRing } from './RingPreview';
import { RingStudio, ringColorName, styleName } from './RingStudio';
import type { RingData, RingView } from './RingStudio';
import { DxConfirmDialog } from './settings/dx';
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
/** A pick's dance: a ripple through the sphere for DANCE_S, with a scale pop in its first POP_S. */
const DANCE_S = 1.2;
const POP_S = 0.3;
/**
 * The sphere. A color change morphs over about half a second rather than cutting; `dance` is a
 * counter, and each change of it plays the pick's dance. Paused, it stops drawing once the morph
 * has settled - it is under a drawer, or muted, and a still frame is all anyone sees.
 */
export function ParticlesOrb({
  state,
  size,
  colorFrom,
  colorTo,
  paused = false,
  dance = 0
}: {
  state: OrbState;
  size: number;
  colorFrom: string;
  colorTo: string;
  paused?: boolean;
  dance?: number;
}) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const stateRef = useRef(state);
  const pausedRef = useRef(paused);
  const danceRef = useRef(dance);
  const colorRef = useRef({
    from: colorFrom,
    to: colorTo
  });
  const still = useRef<(() => void) | null>(null);
  useEffect(() => {
    stateRef.current = state;
    pausedRef.current = paused;
    danceRef.current = dance;
    colorRef.current = {
      from: colorFrom,
      to: colorTo
    };
  });
  // Reduced motion draws one frame, so a new color is a new frame - faded in, not moved.
  useEffect(() => {
    if (!still.current) return;
    still.current();
    canvasRef.current?.animate?.([{ opacity: 0.35 }, { opacity: 1 }], { duration: 320, easing: 'ease-out' });
  }, [colorFrom, colorTo, state]);
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
    const shownFrom = toRgb(colorRef.current.from);
    const shownTo = toRgb(colorRef.current.to);
    let danceSeen = danceRef.current;
    let danceAt = -10;
    let settled = 0;
    /** Moves the drawn colors toward the asked-for ones; returns how far they still have to go. */
    const morph = (dt: number, snap: boolean) => {
      const tf = toRgb(colorRef.current.from),
        tt = toRgb(colorRef.current.to);
      let gap = 0;
      for (let k = 0; k < 3; k++) {
        shownFrom[k] = snap ? tf[k] : approach(shownFrom[k], tf[k], 7, dt);
        shownTo[k] = snap ? tt[k] : approach(shownTo[k], tt[k], 7, dt);
        gap = Math.max(gap, Math.abs(shownFrom[k] - tf[k]), Math.abs(shownTo[k] - tt[k]));
      }
      return gap;
    };
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
      if (danceRef.current !== danceSeen) {
        danceSeen = danceRef.current;
        danceAt = t;
      }
      const since = t - danceAt;
      const dancing = !isStatic && since >= 0 && since < DANCE_S ? Math.sin(Math.PI * since / DANCE_S) : 0;
      const pop = dancing && since < POP_S ? Math.sin(Math.PI * since / POP_S) * 0.07 : 0;
      ripple += dancing * 0.9;
      morph(dt, isStatic);
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
      const radius = baseRadius * (1 + breathe + level * 0.16 + expand - conv + pop);
      const from = mixRgb(shownFrom, ERROR_FROM_RGB, wError);
      const to = mixRgb(shownTo, ERROR_TO_RGB, wError);
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
      still.current = () => render(0, true);
      render(0, true);
      return () => {
        still.current = null;
      };
    }
    const frame = (now: number) => {
      raf = 0;
      const step = last === null ? 0 : Math.min((now - last) / 1000, 0.1);
      const dt = pausedRef.current ? 0 : step;
      last = now;
      t += dt;
      // Paused, the frame only changes while a color is still morphing: draw until it lands.
      if (pausedRef.current) {
        if (morph(step, false) < 0.5) settled++;
        else settled = 0;
      } else settled = 0;
      if (settled < 3) render(dt);
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

/** GET /api/sat1/voice's newest pipeline error (web_ui_handler.cpp handle_voice_): its count, ms ago, and text. */
export type VoiceError = {
  n: number;
  age: number;
  message: string;
};
const RUNNING: OrbState[] = ['listening', 'thinking', 'speaking'];

/**
 * Whether the page is holding the device's newest error on the orb, ERROR_HOLD_MS from when the
 * page first saw it (src/lib/orb.js). The hold ends for good - that error never comes back - when it
 * runs out, when the device starts another run, or on `drop`: the orb's tap, which starts one before
 * the poll can report it. `live` is the device's own state, not the held one.
 */
function useErrorHold(error: VoiceError | undefined, live: OrbState) {
  const seen = useRef<{ n: number; until: number } | null>(null);
  if (error && seen.current?.n !== error.n) seen.current = errorSeen(error, Date.now());
  const [gone, setGone] = useState<number | null>(null);
  const [, bump] = useState(0);
  const running = RUNNING.includes(live);
  useEffect(() => {
    if (running && error) setGone(error.n);
  }, [running, error?.n]);
  const until = errorHoldUntil(seen.current, gone, Date.now());
  useEffect(() => {
    if (!until) return;
    const id = setTimeout(() => bump(k => k + 1), until - Date.now());
    return () => clearTimeout(id);
  }, [until]);
  return { holding: until > 0 && !running, drop: () => error && setGone(error.n) };
}

/** The Customize drawer's pages: the chooser, the orb's, and the LED ring's (RingStudio). */
type Page = 'choose' | 'orb' | RingView;

/**
 * The assistant's orb: its state follows the phase the Home tab polls, its colours are the shell's
 * persisted `orb`, and Customize opens the drawer that styles it and, separately, the LED ring.
 * `onOrbColor` fires only on a person's pick, so mounting never overwrites the saved choice. At
 * rest, `tips` (orbTips) take the label's place; the label stays in the accessibility tree, so a
 * screen reader hears the state change and not every tip. A pipeline error's message (`error`)
 * takes the same place, in red, so a two-line message grows up into the orb's margin rather than
 * pushing the buttons below it down.
 */
export function VoiceOrb({
  ctx,
  phase,
  error,
  orb,
  onOrbColor,
  onTalk,
  tips = []
}: {
  ctx: Ctx;
  phase: number | undefined;
  error?: VoiceError;
  orb: Orb;
  onOrbColor: (from: string, to: string) => void;
  onTalk?: () => void;
  tips?: string[];
}) {
  const [colorPanel, setColorPanel] = useState(false);
  const [page, setPage] = useState<Page>('choose');
  const [sel, setSel] = useState(() => ORB_PRESETS.find(p => p.from === orb.a && p.to === orb.b)?.label ?? 'Custom');
  const [customHue, setCustomHue] = useState(() => Number(/^hsl\(\s*([\d.]+)/.exec(orb.a)?.[1] ?? 290));
  const [dance, setDance] = useState(0);
  const [orbSize, setOrbSize] = useState(180);
  useEffect(() => {
    const on = () => setOrbSize(sizeFor(window.innerWidth));
    on();
    window.addEventListener('resize', on);
    return () => window.removeEventListener('resize', on);
  }, []);
  const mute = entity(ctx, 'mute_mics');
  const muted = isOn(mute);
  const live = orbState(phase, ctx.connected) as OrbState;
  const hold = useErrorHold(error, live);
  const view = orbView(phase, ctx.connected, muted, error, hold.holding);
  const err = view.state === 'error' && view.label !== ORB_LABEL.error ? view.label : '';
  // The sphere is the action button's press-to-talk (common/web_ui_assist.yaml), on firmware that
  // has it - `onTalk` is the Home tab's, which knows whose pipeline the open tab is. Not while muted,
  // and not while Home Assistant is gone ("Not ready") or the page has lost the device - the device
  // would refuse, and a tap that does nothing reads as a broken orb. Going by the device's own
  // state, so an error the page is still holding on an idle device can be answered with a retry.
  const talk = !!onTalk && !muted && TAPPABLE.includes(live);
  const running = live !== 'idle';
  const shown = useTip(tips, !view.muted && view.state === 'idle');
  const light = entity(ctx, 'ring');
  const hasRing = !!light;
  // The ring's styles, read when the drawer opens: undefined until they answer, null on firmware
  // without satellite1_ring, whose LED Ring page then offers color, brightness and effect only.
  const [ringData, setRingData] = useState<RingData | null | undefined>(undefined);
  const reloadRing = async () => {
    try {
      setRingData(await ringApi.load());
    } catch {
      setRingData(null);
    }
  };
  useEffect(() => {
    if (colorPanel && hasRing) reloadRing();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [colorPanel, hasRing]);
  const open = () => {
    setPage(hasRing ? 'choose' : 'orb');
    setColorPanel(true);
  };
  // The theme and the ring's color, brightness and style save as they are picked; a moment's edits
  // wait for Save. So the moment editor is the one page that can lose work, and leaving it - back,
  // or the drawer closing any of its ways - asks first. `then` is what the leave was going to do.
  const dirty = useRef(false);
  const [leaving, setLeaving] = useState<(() => void) | null>(null);
  const leave = (then: () => void) => {
    if (!dirty.current) {
      then();
      return true;
    }
    setLeaving(() => then);
    return false;
  };
  const close = () => leave(() => setColorPanel(false));
  const go = (p: Page) => {
    leave(() => setPage(p));
  };
  // Each page starts at its top, not wherever the last one was scrolled to.
  const top = useRef<HTMLDivElement>(null);
  const pageKey = typeof page === 'string' ? page : page.page + (page.m ?? '');
  useLayoutEffect(() => {
    top.current?.closest('.dw-panel')?.scrollTo(0, 0);
  }, [pageKey]);
  const pickOrb = (label: string, from: string, to: string) => {
    setSel(label);
    onOrbColor(from, to);
    setDance(d => d + 1);
  };
  const pickCustom = (h: number) => {
    setCustomHue(h);
    onOrbColor(hueFrom(h), hueTo(h));
  };
  const ringView = typeof page === 'object' ? page : null;
  const label = page === 'choose' ? RING.choose_eyebrow : page === 'orb' ? RING.orb_eyebrow : RING.ring_eyebrow;
  return <div className="orb-wrap">
    <div className="orb-stage" style={{
      margin: `${-Math.round(orbSize * ORB_OVERLAP)}px 0`,
      pointerEvents: 'none'
    }}>
      <ParticlesOrb state={view.state} size={orbSize} colorFrom={view.muted ? '#ef4444' : orb.a} colorTo={view.muted ? '#b91c1c' : orb.b} paused={view.muted || colorPanel} />
      {talk && <button className="orb-tap" style={{
        width: Math.round(orbSize * ORB_TAP),
        height: Math.round(orbSize * ORB_TAP)
      }} aria-label={running ? TEXT.orb_stop : TEXT.orb_talk} title={running ? TEXT.orb_stop : TEXT.orb_talk} onClick={() => {
          hold.drop();
          onTalk?.();
        }} />}
    </div>
    <div className="orb-meta">
      <div className="orb-say">
        <span className={'orb-label' + (shown.tip || err ? ' tipped' : '') + (view.muted ? ' muted-on' : '')} aria-live="polite">{view.label}</span>
        {err && <span className="orb-tips orb-err" aria-hidden="true"><span key={err} className="in">{err}</span></span>}
        {shown.tip && <span className="orb-tips">{shown.leaving && <span key={'out' + shown.leaving} className="out" aria-hidden="true">{shown.leaving}</span>}<span key={shown.tip} className="in">{shown.tip}</span></span>}
      </div>
      <div className="orb-tools">
      <button className="orb-hint customize-btn" onClick={() => {
          tipDone('color');
          open();
        }} aria-label={hasRing ? 'Customize the orb and LED ring' : 'Customize orb color'}><span aria-hidden="true" className="customize-dot" /><span>Customize</span></button>
      {mute && <button className="orb-hint customize-btn mute-btn" onClick={() => {
          tipDone('mute');
          post(pathFor(ctx, 'mute_mics', muted ? 'turn_off' : 'turn_on'));
        }} aria-pressed={muted} aria-label={muted ? 'Unmute microphone' : 'Mute microphone'} title={HINTS.mute}>{muted ? <MicOff size={18} strokeWidth={2.2} /> : <Mic size={18} />}</button>}
      </div>
    </div>
    <Presence>{colorPanel && <Drawer label={label} onClose={close} className={'orb-sheet color-panel' + (ringView ? ' ring-sheet' : '')}>
        <div ref={top} key={ringView ? 'ring' : pageKey} className="cz-page">
        {page === 'choose' && <div className="rs-page">
            <span className="eyebrow">{RING.choose_eyebrow}</span>
            <h2>{RING.choose_title}</h2>
            <p className="muted rs-lead">{RING.choose_lead}</p>
            <div className="cz-cards">
              <button className="cz-card" onClick={() => go('orb')}>
                <ParticlesOrb state="idle" size={110} colorFrom={orb.a} colorTo={orb.b} />
                <strong>{RING.choose_orb}</strong>
                <small>{sel}</small>
                <span className="cz-go">{RING.choose_go}<ChevronRight size={14} /></span>
              </button>
              <button className="cz-card" onClick={() => go({ page: 'ring' })}>
                <MomentRing m={M_LISTEN} style={ringData?.m[M_LISTEN] ?? STYLES.classic[M_LISTEN]} ring={ringRgb(light)} size={110} />
                <strong>{RING.choose_ring}</strong>
                <small>{ringColorName(light)}{ringData ? ` · ${styleName(ringData.style)}` : ''}</small>
                <span className="cz-go">{RING.choose_go}<ChevronRight size={14} /></span>
              </button>
            </div>
            <button className="primary wide" onClick={close}>{RING.done}</button>
          </div>}
        {page === 'orb' && <div className="rs-page">
            {hasRing ? <div className="rs-top"><button className="rs-back" onClick={() => go('choose')}><ChevronLeft size={18} />{RING.back}</button><button className="dw-x" aria-label="Close" onClick={close}><X size={18} /></button></div> : <div className="rs-top end"><button className="dw-x" aria-label="Close" onClick={close}><X size={18} /></button></div>}
            <div className="rs-hero"><ParticlesOrb state="idle" size={150} colorFrom={orb.a} colorTo={orb.b} dance={dance} /></div>
            <span className="eyebrow">{RING.orb_eyebrow}</span>
            <h2>{RING.orb_title}</h2>
            <div className="swatches">
              {ORB_PRESETS.map(s => <button key={s.label} className={'swatch ' + (sel === s.label ? 'on' : '')} aria-pressed={sel === s.label} onClick={() => pickOrb(s.label, s.from, s.to)}><span className="swatch-dot" style={{
                  background: `linear-gradient(135deg, ${s.from}, ${s.to})`
                }} /><small>{s.label}</small></button>)}
              <button className={'swatch ' + (sel === 'Custom' ? 'on' : '')} aria-pressed={sel === 'Custom'} onClick={() => {
                  setSel('Custom');
                  pickCustom(customHue);
                  setDance(d => d + 1);
                }}><span className="swatch-dot" style={{
                  background: 'conic-gradient(red,yellow,lime,cyan,blue,magenta,red)'
                }} /><small>Custom</small></button>
            </div>
            {sel === 'Custom' && <label className="hue-label"><small className="muted hue-cap">Orb / theme color</small><span>Hue · {customHue}°</span><input className="hue" type="range" min={0} max={359} value={customHue} onChange={e => pickCustom(Number(e.target.value))} aria-label="Custom hue" /></label>}
            <button className="primary wide" onClick={close}>{RING.done}</button>
          </div>}
        {ringView && <RingStudio ctx={ctx} data={ringData} reload={reloadRing} view={ringView} go={go} onBack={() => go('choose')} onClose={close} onDirty={d => {
            dirty.current = d;
          }} />}
        </div>
      </Drawer>}</Presence>
    <DxConfirmDialog open={!!leaving} title={RING.leave_title} body={RING.leave_body} confirmLabel={RING.leave} cancelLabel={RING.stay} danger onCancel={() => setLeaving(null)} onConfirm={() => {
        const then = leaving;
        setLeaving(null);
        dirty.current = false;
        then?.();
      }} />
  </div>;
}
