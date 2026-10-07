import { useEffect, useRef } from 'react';
import { MIC_LEDS, N, renderMoment } from '../lib/ringfx.js';
import { fixedFrame, fixedStill, sampleInputs } from '../lib/ring.js';

/**
 * The LED ring on screen: 24 lights round a dark face, each a core and a soft glow, drawn from the
 * frames lib/ringfx.js renders - the device's own math, so a preview is the ring it describes.
 *
 * Every ring on the page shares one animation frame loop, held to about 30 frames a second (the
 * device's effects update every 10 to 20 ms, but a phone drawing a dozen previews has better things
 * to do), and a ring scrolled out of view leaves it. With reduced motion each draws one still frame.
 */

type Rgb = number[];
export type FrameFn = (now: number) => Rgb[] | null;

const TAU = Math.PI * 2;
const STILL_MS = 900;
const FRAME_MS = 32;
const reduced = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

const live = new Set<(now: number) => void>();
let raf = 0;
let last = 0;
const tick = (now: number) => {
  raf = live.size ? requestAnimationFrame(tick) : 0;
  if (now - last < FRAME_MS) return;
  last = now;
  live.forEach(f => f(now));
};
const join = (f: (now: number) => void) => {
  live.add(f);
  if (!raf) raf = requestAnimationFrame(tick);
};

function paint(cx: CanvasRenderingContext2D, size: number, leds: Rgb[] | null, mics: boolean, bright: number) {
  const c = size / 2;
  cx.globalCompositeOperation = 'source-over';
  cx.clearRect(0, 0, size, size);
  const face = cx.createRadialGradient(c, c * 0.9, c * 0.1, c, c, c);
  face.addColorStop(0, '#23232a');
  face.addColorStop(0.72, '#141418');
  face.addColorStop(1, '#0a0a0c');
  cx.fillStyle = face;
  cx.beginPath();
  cx.arc(c, c, c * 0.98, 0, TAU);
  cx.fill();
  cx.strokeStyle = 'rgba(255,255,255,.06)';
  cx.lineWidth = Math.max(1, size * 0.006);
  cx.beginPath();
  cx.arc(c, c, c * 0.64, 0, TAU);
  cx.stroke();
  const R = c * 0.8;
  const glow = size * 0.07;
  const core = Math.max(1, size * 0.02);
  const at = (i: number, r: number) => {
    const a = -Math.PI / 2 + (i / N) * TAU;
    return [c + Math.cos(a) * r, c + Math.sin(a) * r];
  };
  if (mics) {
    cx.fillStyle = 'rgba(255,255,255,.28)';
    for (const i of MIC_LEDS) {
      const [x, y] = at(i, c * 0.56);
      cx.beginPath();
      cx.arc(x, y, Math.max(1, size * 0.011), 0, TAU);
      cx.fill();
    }
  }
  cx.fillStyle = 'rgba(255,255,255,.07)';
  for (let i = 0; i < N; i++) {
    const [x, y] = at(i, R);
    cx.beginPath();
    cx.arc(x, y, core * 0.8, 0, TAU);
    cx.fill();
  }
  if (!leds) return;
  cx.globalCompositeOperation = 'lighter';
  for (let i = 0; i < N; i++) {
    const [r, g, b] = leds[i];
    const mx = Math.max(r, g, b);
    const k = (mx / 255) * bright;
    if (k < 0.015) continue;
    const nr = (r / mx) * 255 | 0, ng = (g / mx) * 255 | 0, nb = (b / mx) * 255 | 0;
    const [x, y] = at(i, R);
    cx.fillStyle = `rgba(${nr},${ng},${nb},${(0.16 * k).toFixed(3)})`;
    cx.beginPath();
    cx.arc(x, y, glow, 0, TAU);
    cx.fill();
    cx.fillStyle = `rgba(${nr},${ng},${nb},${(0.32 * k).toFixed(3)})`;
    cx.beginPath();
    cx.arc(x, y, glow * 0.5, 0, TAU);
    cx.fill();
    const w = 0.45 * k;
    cx.fillStyle = `rgba(${nr + (255 - nr) * w | 0},${ng + (255 - ng) * w | 0},${nb + (255 - nb) * w | 0},${Math.min(1, 0.3 + k).toFixed(3)})`;
    cx.beginPath();
    cx.arc(x, y, core, 0, TAU);
    cx.fill();
  }
  cx.globalCompositeOperation = 'source-over';
}

/**
 * One ring. `frame` is called with the animation clock (performance.now()) and may change between
 * renders without restarting anything. `bright` (0-1) dims the lights the way the brightness
 * slider dims the device, kept off the floor so a dim ring still reads on screen.
 */
export function RingCanvas({
  size,
  frame,
  mics = false,
  bright = 1,
  className,
  label
}: {
  size: number;
  frame: FrameFn;
  mics?: boolean;
  bright?: number;
  className?: string;
  label?: string;
}) {
  const ref = useRef<HTMLCanvasElement>(null);
  const opts = useRef({ frame, mics, bright });
  opts.current = { frame, mics, bright: 0.35 + 0.65 * Math.max(0, Math.min(1, bright)) };
  const still = useRef<(() => void) | null>(null);
  useEffect(() => {
    const canvas = ref.current;
    const cx = canvas?.getContext('2d');
    if (!canvas || !cx) return undefined;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    cx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const draw = (now: number) => {
      const o = opts.current;
      paint(cx, size, o.frame(now), o.mics, o.bright);
    };
    if (reduced()) {
      const t = performance.now();
      still.current = () => draw(t);
      draw(t);
      return () => {
        still.current = null;
      };
    }
    draw(performance.now());
    const seen = new IntersectionObserver(([e]) => {
      if (e.isIntersecting) join(draw);
      else live.delete(draw);
    });
    seen.observe(canvas);
    return () => {
      seen.disconnect();
      live.delete(draw);
    };
  }, [size]);
  // A still ring redraws when what it shows changes; a running one picks it up on its next frame.
  useEffect(() => {
    still.current?.();
  });
  return <canvas ref={ref} className={className} style={{ width: size, height: size, display: 'block' }} role={label ? 'img' : undefined} aria-label={label} aria-hidden={label ? undefined : true} />;
}

/** The clock a preview starts from: its own mount, or under reduced motion a frame part way in. */
function useStart(still = STILL_MS) {
  const t0 = useRef(0);
  if (!t0.current) t0.current = performance.now() - (reduced() ? still : 0);
  return t0.current;
}

/** A moment playing one style, on sample data, from when it mounted. */
export function MomentRing({ m, style, ring, size, mics, bright, className }: {
  m: number;
  style: unknown;
  ring: Rgb;
  size: number;
  mics?: boolean;
  bright?: number;
  className?: string;
}) {
  const t0 = useStart();
  return <RingCanvas size={size} mics={mics} bright={bright} className={className} frame={now => renderMoment(m, style, Math.floor(now - t0), 0, sampleInputs(m, ring)).frame} />;
}

/** A fixed signal, as the YAML effect that lights it looks. */
export function FixedRing({ k, ring, size, className }: { k: string; ring: Rgb; size: number; className?: string }) {
  const t0 = useStart(fixedStill(k));
  return <RingCanvas size={size} className={className} frame={now => fixedFrame(k, Math.floor(now - t0), ring)} />;
}
