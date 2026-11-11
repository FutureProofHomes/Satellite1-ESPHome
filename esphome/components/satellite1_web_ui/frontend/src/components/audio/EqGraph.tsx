import React, { useId, useLayoutEffect, useMemo, useRef, useState } from 'react';
import type { KeyboardEvent as RKeyboardEvent, PointerEvent as RPointerEvent } from 'react';
import { DSP } from '../../copy.js';
import { RANGE, T, clamp, filterCurve, fmtDb, fmtHz, handleDb, kindOf, logFreqs, profileCurve, profileSections, slopeOf, usesGain, usesQ } from '../../lib/eq.js';

export type Filter = {
  t: number;
  on: boolean;
  hz: number;
  db: number;
  q: number;
};
export type Profile = {
  pre: number;
  bass: number;
  mid: number;
  tre: number;
  lim: number;
  atk: number;
  rel: number;
  preset: string;
  f: Filter[];
};

/**
 * One colour per filter slot, from styles/dsp.css (`--eq-c0`...`--eq-c11`), which darkens them on
 * the light theme. The number in each dot carries the same identity; `--eq-on` is the number's ink
 * on a filled dot or chip.
 */
export const filterColor = (i: number) => `var(--eq-c${i % 12})`;
export const ON_INK = 'var(--eq-on)';

export const fill = (s: string, vars: Record<string, string | number>) => s.replace(/\{(\w+)\}/g, (_, k) => String(vars[k] ?? ''));

/** What a filter does, in words: "Cut at 3.46 kHz", "Remove below 60 Hz". */
export function sayFilter(f: Filter) {
  const hz = fmtHz(f.hz);
  const flat = usesGain(f.t) && Math.abs(f.db) < 0.05;
  const s = DSP.say;
  switch (kindOf(f.t)) {
    case 'lowshelf': return fill(flat ? s.flat : f.db > 0 ? s.more_below : s.less_below, { hz });
    case 'highshelf': return fill(flat ? s.flat : f.db > 0 ? s.more_above : s.less_above, { hz });
    case 'lowcut': return fill(s.lowcut, { hz });
    case 'highcut': return fill(s.highcut, { hz });
    default: return fill(flat ? s.flat : f.db > 0 ? s.boost : s.cut, { hz });
  }
}

/** Its technical name: "Peak · −3.0 dB · Q 1.93", "Low cut · 12 dB/oct · custom Q 1.00". */
export function techFilter(f: Filter) {
  const kind = DSP.kinds[kindOf(f.t) as keyof typeof DSP.kinds];
  if (usesGain(f.t)) return `${kind} · ${fmtDb(f.db)} · Q ${f.q.toFixed(2)}`;
  const slope = DSP.slopes[slopeOf(f.t) as keyof typeof DSP.slopes];
  return f.t === T.HP_Q || f.t === T.LP_Q ? `${kind} · ${slope} ${f.q.toFixed(2)}` : `${kind} · ${slope}`;
}

/** Three significant figures: 138, 1600, 3460. Finer is the number field's job. */
export const roundHz = (hz: number) => {
  const r = 10 ** (Math.floor(Math.log10(hz)) - 2);
  return clamp(+(Math.round(hz / r) * r).toPrecision(3), RANGE.hz[0], RANGE.hz[1]);
};
const round1 = (v: number) => Math.round(v * 10) / 10;
const round2 = (v: number) => Math.round(v * 100) / 100;

const DB_SPAN = 18;
const GRID_HZ = [20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000];
const KEY_HZ = new Set([20, 100, 1000, 10000]);
const GRID_DB = [-18, -12, -6, 0, 6, 12, 18];
const hzLabel = (hz: number) => (hz >= 1000 ? `${hz / 1000}k` : String(hz));
const dbLabel = (db: number) => (db > 0 ? `+${db}` : db < 0 ? `\u2212${-db}` : '0');

/** Measured before paint, so a phone never sees a frame drawn at the fallback width. */
function useWidth(ref: React.RefObject<HTMLElement>) {
  const [w, setW] = useState(0);
  useLayoutEffect(() => {
    const el = ref.current;
    if (!el) return undefined;
    setW(el.clientWidth);
    const ro = new ResizeObserver(() => setW(el.clientWidth));
    ro.observe(el);
    return () => ro.disconnect();
  }, [ref]);
  return w;
}

/**
 * The response graph: 20 Hz-20 kHz on a log axis, ±18 dB. It draws the EQ itself - filters, Bass,
 * Midrange and Treble - so each dot sits on the curve it shapes; the preamp instead moves the limiter's line,
 * since raising everything by 10 dB is the same as bringing the ceiling 10 dB closer. Wherever the
 * curve is above that line, loud music reaches the limiter.
 *
 * `readOnly` is Basic's: the total curve alone, shorter, with no dots, no limiter line and nothing
 * to drag, since there the tone sliders are the only controls.
 *
 * Drawn at the container's own pixel width rather than scaled, so labels stay legible on a phone.
 */
export function EqGraph({
  p,
  sel = null,
  onSelect = () => {},
  onChange = () => {},
  onAdd = () => {},
  readOnly = false
}: {
  p: Profile;
  sel?: number | null;
  onSelect?: (i: number | null) => void;
  onChange?: (i: number, patch: Partial<Filter>) => void;
  onAdd?: (hz: number, db: number) => void;
  readOnly?: boolean;
}) {
  const box = useRef<HTMLDivElement>(null);
  const svg = useRef<SVGSVGElement>(null);
  const width = useWidth(box) || 640;
  const height = readOnly ? (width < 480 ? 150 : 180) : width < 480 ? 220 : 270;
  const pad = { l: 34, r: 10, t: 14, b: 24 };
  const iw = width - pad.l - pad.r;
  const ih = height - pad.t - pad.b;
  const x = (hz: number) => pad.l + (Math.log(hz / 20) / Math.log(1000)) * iw;
  const hzAt = (px: number) => 20 * 1000 ** ((px - pad.l) / iw);
  const y = (db: number) => pad.t + ((DB_SPAN - clamp(db, -DB_SPAN - 3, DB_SPAN + 3)) / (2 * DB_SPAN)) * ih;
  const dbAt = (py: number) => DB_SPAN - ((py - pad.t) / ih) * 2 * DB_SPAN;

  const n = clamp(Math.round(iw / 2.5), 120, 320);
  const freqs = useMemo(() => logFreqs(n), [n]);
  const total = useMemo(() => profileCurve({ ...p, pre: 0 }, freqs), [p.bass, p.mid, p.tre, p.f, freqs]);
  // An edit replaces only the filter it touches, so the others' curves and paths are kept by
  // identity: a drag recomputes one curve, not twelve.
  const cache = useRef<{ key: string; curve: WeakMap<Filter, number[]>; path: WeakMap<number[], string> } | null>(null);
  const size = `${width}x${height}`;
  if (cache.current?.key !== size) cache.current = { key: size, curve: new WeakMap(), path: new WeakMap() };
  const memo = cache.current;
  const curves = readOnly ? [] : p.f.map(f => {
    let c = memo.curve.get(f);
    if (!c) memo.curve.set(f, c = filterCurve({ ...f, on: true }, freqs));
    return c;
  });
  const ids = useId();
  const fillId = `eq-fill${ids}`;
  const plotId = `eq-plot${ids}`;

  const line = (vals: number[]) => {
    let d = memo.path.get(vals);
    if (!d) memo.path.set(vals, d = vals.map((v, i) => `${i ? 'L' : 'M'}${x(freqs[i]).toFixed(1)} ${y(v).toFixed(1)}`).join(''));
    return d;
  };
  const area = (vals: number[]) => `${line(vals)}L${x(20000).toFixed(1)} ${y(0).toFixed(1)}L${x(20).toFixed(1)} ${y(0).toFixed(1)}Z`;
  const limDb = p.lim - p.pre;
  // The XMOS bypasses a flat profile outright, limiter included, so there is no line to draw.
  const bypassed = p.pre === 0 && profileSections(p) === 0;

  const local = (e: { clientX: number; clientY: number }) => {
    const r = svg.current!.getBoundingClientRect();
    return [((e.clientX - r.left) * width) / r.width, ((e.clientY - r.top) * height) / r.height];
  };
  const drag = useRef<number | null>(null);
  const moveTo = (i: number, px: number, py: number) => {
    const f = p.f[i];
    const patch: Partial<Filter> = { hz: roundHz(clamp(hzAt(px), RANGE.hz[0], RANGE.hz[1])) };
    if (usesGain(f.t)) {
      const d = dbAt(py) * (f.t === T.PEAK ? 1 : 2);
      patch.db = round1(clamp(d, RANGE.db[0], RANGE.db[1]));
    }
    onChange(i, patch);
  };
  const down = (i: number) => (e: RPointerEvent<SVGGElement>) => {
    e.preventDefault();
    e.stopPropagation();
    svg.current?.setPointerCapture(e.pointerId);
    drag.current = i;
    onSelect(i);
    (e.currentTarget as unknown as HTMLElement).focus?.();
  };
  const move = (e: RPointerEvent<SVGSVGElement>) => {
    if (drag.current == null) return;
    const [px, py] = local(e);
    moveTo(drag.current, px, py);
  };
  const up = (e: RPointerEvent<SVGSVGElement>) => {
    if (drag.current == null) return;
    drag.current = null;
    svg.current?.releasePointerCapture(e.pointerId);
  };
  const key = (i: number) => (e: RKeyboardEvent<SVGGElement>) => {
    const f = p.f[i];
    const big = e.shiftKey;
    let patch: Partial<Filter> | null = null;
    if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
      const oct = (big ? 1 / 3 : 1 / 12) * (e.key === 'ArrowRight' ? 1 : -1);
      patch = { hz: roundHz(clamp(f.hz * 2 ** oct, RANGE.hz[0], RANGE.hz[1])) };
    } else if ((e.key === 'ArrowUp' || e.key === 'ArrowDown') && usesGain(f.t)) {
      const step = (big ? 3 : 0.5) * (e.key === 'ArrowUp' ? 1 : -1);
      patch = { db: round1(clamp(f.db + step, RANGE.db[0], RANGE.db[1])) };
    } else if ((e.key === 'PageUp' || e.key === 'PageDown') && usesQ(f.t)) {
      const k = big ? 1.5 : 1.1;
      patch = { q: round2(clamp(f.q * (e.key === 'PageUp' ? k : 1 / k), RANGE.q[0], RANGE.q[1])) };
    }
    if (!patch) return;
    e.preventDefault();
    onChange(i, patch);
  };
  // Only on the picked dot, so a page scrolled past the graph isn't caught by one; scaled by the
  // distance, so a trackpad flick is a nudge rather than a run to the end: x1.1 per wheel notch.
  const wheel = (i: number) => (e: WheelEvent) => {
    const f = p.f[i];
    if (i !== sel || !usesQ(f.t)) return;
    e.preventDefault();
    const px = e.deltaY * (e.deltaMode === 1 ? 33 : e.deltaMode === 2 ? 400 : 1);
    const k = 1.1 ** clamp(-px / 100, -3, 3);
    onChange(i, { q: round2(clamp(f.q * k, RANGE.q[0], RANGE.q[1])) });
  };
  const add = (e: React.MouseEvent<SVGSVGElement>) => {
    if ((e.target as Element).closest('.eq-handle')) return;
    const [px, py] = local(e);
    if (px < pad.l || px > width - pad.r || py < pad.t || py > height - pad.b) return;
    onAdd(roundHz(clamp(hzAt(px), RANGE.hz[0], RANGE.hz[1])), round1(clamp(dbAt(py), RANGE.db[0], RANGE.db[1])));
  };

  const wide = iw >= 420;
  const events = readOnly ? {} : {
    onPointerMove: move,
    onPointerUp: up,
    onPointerCancel: up,
    onDblClick: add,
    onPointerDown: (e: RPointerEvent<SVGSVGElement>) => {
      if (!(e.target as Element).closest('.eq-handle')) onSelect(null);
    }
  };
  return <div ref={box} className="eq-graph">
      <svg ref={svg} width={width} height={height} viewBox={`0 0 ${width} ${height}`} role={readOnly ? 'img' : 'group'} aria-label={DSP.graph} {...events}>
        <defs>
          <linearGradient id={fillId} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="var(--orb-ink)" stopOpacity=".28" />
            <stop offset="50%" stopColor="var(--orb-ink)" stopOpacity=".04" />
            <stop offset="100%" stopColor="var(--orb-ink)" stopOpacity=".28" />
          </linearGradient>
          <clipPath id={plotId}><rect x={pad.l} y={pad.t} width={iw} height={ih} /></clipPath>
        </defs>
        {GRID_HZ.map(hz => <g key={hz}>
            <line className="eq-grid" x1={x(hz)} x2={x(hz)} y1={pad.t} y2={pad.t + ih} />
            {(wide || KEY_HZ.has(hz)) && <text className="eq-axis" x={x(hz)} y={height - 6} textAnchor={hz === 20 ? 'start' : hz === 20000 ? 'end' : 'middle'}>{hzLabel(hz)}</text>}
          </g>)}
        {GRID_DB.map(db => <g key={db}>
            <line className={db === 0 ? 'eq-zero' : 'eq-grid'} x1={pad.l} x2={pad.l + iw} y1={y(db)} y2={y(db)} />
            {(!readOnly || db % 12 === 0) && <text className="eq-axis" x={pad.l - 6} y={y(db) + 3.5} textAnchor="end">{dbLabel(db)}</text>}
          </g>)}
        <g clipPath={`url(#${plotId})`}>
          {!readOnly && !bypassed && limDb > -DB_SPAN && limDb < DB_SPAN && <g className="eq-limit">
              <rect x={pad.l} y={pad.t} width={iw} height={Math.max(0, y(limDb) - pad.t)} />
              <line x1={pad.l} x2={pad.l + iw} y1={y(limDb)} y2={y(limDb)} />
              <text x={pad.l + 4} y={y(limDb) - 5}>{DSP.limiter_line}</text>
            </g>}
          {sel != null && curves[sel] && <path className="eq-sel-area" d={area(curves[sel])} style={{
          fill: filterColor(sel)
        }} />}
          {curves.map((c, i) => (p.f[i].on || i === sel) && <path key={i} className={`eq-part${i === sel ? ' sel' : ''}${p.f[i].on ? '' : ' off'}`} d={line(c)} style={{
          stroke: filterColor(i)
        }} />)}
          <path className="eq-total-area" d={area(total)} style={{
          fill: `url(#${fillId})`
        }} />
          <path className="eq-total" d={line(total)} />
        </g>
        {!readOnly && p.f.map((f, i) => {
        const hy = clamp(y(handleDb(f)), pad.t, pad.t + ih);
        const color = filterColor(i);
        const say = sayFilter(f);
        // Lowercase on purpose: Preact sets SVG props as attributes, as written, and an SVG
        // element ignores a camel-case tabIndex attribute - the dot would never take focus.
        return <g key={i} className={`eq-handle${i === sel ? ' sel' : ''}${f.on ? '' : ' off'}`} transform={`translate(${x(f.hz).toFixed(1)} ${hy.toFixed(1)})`} tabindex={0} role="slider" aria-label={fill(DSP.handle, {
          i: i + 1,
          say
        })} aria-valuemin={RANGE.hz[0]} aria-valuemax={RANGE.hz[1]} aria-valuenow={Math.round(f.hz)} aria-valuetext={usesGain(f.t) ? `${say}, ${fmtDb(f.db)}` : say} onPointerDown={down(i)} onKeyDown={key(i)} onWheel={wheel(i)} onFocus={() => onSelect(i)}>
              <circle className="eq-hit" r={18} />
              <circle className="eq-dot" r={i === sel ? 11 : 9} style={{
            fill: f.on ? color : 'var(--surface)',
            stroke: color
          }} />
              <text className="eq-num" y={3.5} textAnchor="middle" style={{
            fill: f.on ? ON_INK : color
          }}>{i + 1}</text>
            </g>;
      })}
      </svg>
    </div>;
}
