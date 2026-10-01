import { useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { ArrowRight, ChevronDown, Check } from '../icons';
import type { Ctx } from '../ctx';
import { HintBtn } from './bits';
import { HINTS, TEXT, WW_ERR } from '../copy.js';
import { PIPELINE_PREFERRED, STOP_SLOT, entity, haBlocked, haSyncOnce, haTooOld, pathFor, post, requestJson, useAssist, useWakeSlots } from '../lib/device.js';
import { holdPeerMutes, keepPeerMutes, releasePeerMutes } from '../lib/peermute.js';
import { DEFAULT_SOURCES, REQUEST_WORD_URL, TRAIN_URL, canSpeak, enumerateSource, readSources, speak, writeSources } from '../lib/wakesources.js';
import { CUT_MAX, CUT_MIN, FSD_KEYS, FSD_OPTIONS, attemptMarks, attemptsOf, clampCut, cutoffPath, fade, filterEntries, fsdShown, fsdWrites, isUrl, langLabel, langsOf, pairingWrite, parseSource, pctN, pickerEntries, placement, readout, roomMarks, rowMarks, sessionRoomMarks, swapStep } from '../lib/wake.js';
type MarkKind = 'fire' | 'near' | 'room' | 'you';
interface Mark {
  id: string;
  c: number;
  y: number;
  kind: MarkKind;
  age?: number;
  ripple?: boolean;
}
/** One track of GET /api/sat1/wakewords: a slot, or the stop word's `stopw` block. */
interface Track {
  i?: number;
  m?: string;
  w?: string;
  st?: number;
  err?: number;
  cut: number;
  ld?: number;
  tn?: number[];
  day?: number[];
  dh?: number[][];
  dl?: number;
  tot?: number;
}
interface Swap {
  phase: 'busy' | 'error';
  word: string;
  spec: string;
  err?: number;
  dl?: number;
  tot?: number;
}
interface Source {
  url: string;
  label: string;
}
interface CatEntry {
  loading?: boolean;
  error?: boolean;
  entries?: unknown[];
}
interface Entry {
  key: string;
  word: string;
  spec: string;
  langs: string[];
  source: string;
  ver?: string;
  unverified?: boolean;
}
interface Attempt {
  score: number;
  round: string;
}
interface Tuning {
  i: number;
  word: string;
  isStop: boolean;
  quick: boolean;
}
interface TunerState {
  phase: 'ready' | 'voice' | 'place' | 'nogap' | 'nocap' | 'gone';
  attempts: Attempt[];
  vadTries: number;
  skipped: boolean;
  roomReg: number;
  roomSeen: number[];
  day: number[];
  noise: number;
  floorV: number;
  hiV: number;
  cutC: number;
  nogap?: string;
}
const GRID_V = [{
  id: 'g10',
  p: 10
}, {
  id: 'g20',
  p: 20
}, {
  id: 'g25',
  p: 25,
  mj: true
}, {
  id: 'g30',
  p: 30
}, {
  id: 'g40',
  p: 40
}, {
  id: 'g50',
  p: 50,
  mj: true
}, {
  id: 'g60',
  p: 60
}, {
  id: 'g70',
  p: 70
}, {
  id: 'g75',
  p: 75,
  mj: true
}, {
  id: 'g80',
  p: 80
}, {
  id: 'g90',
  p: 90
}];
const TICKS = [{
  id: 't0',
  p: 0
}, {
  id: 't25',
  p: 25
}, {
  id: 't50',
  p: 50
}, {
  id: 't75',
  p: 75
}, {
  id: 't100',
  p: 100
}];
const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
/** The stop model reports its phrase lowercase; it reads capitalized everywhere (owner call,
 *  September 2026). Every other word arrives already display-cased by the loader. */
const showWord = (w: string) => w === 'stop' ? 'Stop' : w;
const kb = (n: number) => Math.round(n / 1024);
const SpeakIcon = () => <svg width="13" height="13" viewBox="0 0 13 13" fill="none"><path d="M2 5v3h2l3 3V2L4 5H2z" fill="currentColor" /><path d="M9 4.5a3 3 0 010 4M10.5 3a5 5 0 010 7" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" fill="none" /></svg>;
interface DdOption {
  id: string;
  label: string;
}
function Dropdown({
  value,
  options,
  onChange,
  label
}: {
  value: string;
  options: DdOption[];
  onChange: (v: string) => void;
  label: string;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const h = (e: PointerEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    const k = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.stopPropagation();
        setOpen(false);
      }
    };
    document.addEventListener('pointerdown', h);
    document.addEventListener('keydown', k);
    return () => {
      document.removeEventListener('pointerdown', h);
      document.removeEventListener('keydown', k);
    };
  }, [open]);
  const cur = options.find(o => o.id === value);
  return <div className="ww-dd" ref={ref}>
    <button type="button" className={open ? 'ww-dd-btn open' : 'ww-dd-btn'} aria-haspopup="listbox" aria-expanded={open} aria-label={label} onClick={() => setOpen(!open)}><span>{cur?.label ?? value}</span><svg width="14" height="14" viewBox="0 0 16 16" fill="none" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="m4 6 4 4 4-4" /></svg></button>
    {open && <ul className="ww-dd-pop" role="listbox" aria-label={label}>{options.map(o => <li key={o.id}><button type="button" role="option" aria-selected={o.id === value} className={o.id === value ? 'ww-dd-opt on' : 'ww-dd-opt'} onClick={() => {
          onChange(o.id);
          setOpen(false);
        }}><span>{o.label}</span>{o.id === value && <svg width="14" height="14" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3.2 8.6 6.4 11.8 12.8 4.8" /></svg>}</button></li>)}</ul>}
  </div>;
}
function renderMark(m: Mark, x: (c: number) => number, plotH: number) {
  const mx = x(m.c);
  const my = Math.max(8, Math.min(m.y, plotH - 8));
  if (m.kind === 'fire') return <g key={m.id} opacity={fade(m.age)}><circle className="lg-halo-a" cx={mx} cy={my} r="9" />{m.ripple && <circle className="lg-rip" cx={mx} cy={my} r="8.5" />}<circle className="lg-fire" cx={mx} cy={my} r="5" /></g>;
  if (m.kind === 'near') return <circle key={m.id} className="lg-near" cx={mx} cy={my} r="3.5" opacity={fade(m.age)} />;
  if (m.kind === 'room') return <g key={m.id} opacity={fade(m.age) * 0.85}><circle className="lg-halo-w" cx={mx} cy={my} r="7" />{m.ripple && <circle className="lg-rip-room" cx={mx} cy={my} r="8.5" />}<circle className="lg-room" cx={mx} cy={my} r="3.5" /></g>;
  return <g key={m.id}><circle className="lg-rip-you" cx={mx} cy={my} r="8.5" /><circle className="lg-you" cx={mx} cy={my} r="5" /></g>;
}
const KEY_STEP: Record<string, number> = {
  ArrowLeft: -1,
  ArrowDown: -1,
  ArrowRight: 1,
  ArrowUp: 1,
  PageDown: -5,
  PageUp: 5
};
/**
 * The Living Graph. Its marks are drawn twice and split at the knob into two color worlds: left of
 * it frosted on the amber veil (what the word ignores, literally out of focus), right of it
 * crisp on the accent tint. The two colors are the whole explanation; the split has no legend.
 * With `onCut` the knob drags and the graph is a slider; with only `onTune` the knob is a tap
 * target and the graph a button. A press moves the knob only once it has travelled past a 4px
 * slop, so a tap never nudges the line.
 */
function TouchGraph({
  gid,
  marks,
  cut,
  onCut,
  onTune,
  h = 120,
  aria
}: {
  gid: string;
  marks: Mark[];
  cut?: number;
  onCut?: (v: number) => void;
  onTune?: () => void;
  h?: number;
  aria: string;
}) {
  const W = 320;
  const ref = useRef<SVGSVGElement>(null);
  const [held, setHeld] = useState(false);
  const press = useRef<{
    x: number;
    y: number;
    moved: boolean;
  } | null>(null);
  const x = (c: number) => c / 100 * W;
  const plotH = h - 16;
  const rows = [1, 2, 3].map(i => ({
    id: `h${i}`,
    y: Math.round(plotH * i / 4)
  }));
  const setFrom = (clientX: number) => {
    const r = ref.current?.getBoundingClientRect();
    if (!r || !onCut) return;
    onCut(clampCut(Math.round((clientX - r.left) / r.width * 100)));
  };
  const release = () => {
    press.current = null;
    setHeld(false);
  };
  const onKeyDown = (e: KeyboardEvent) => {
    if (onCut && cut !== undefined) {
      const next = e.key === 'Home' ? CUT_MIN : e.key === 'End' ? CUT_MAX : KEY_STEP[e.key] ? cut + KEY_STEP[e.key] : null;
      if (next === null) return;
      e.preventDefault();
      onCut(clampCut(next));
    } else if (onTune && (e.key === 'Enter' || e.key === ' ')) {
      e.preventDefault();
      onTune();
    }
  };
  const cx = cut !== undefined ? x(cut) : 0;
  const a11y = onCut && cut !== undefined ? {
    role: 'slider',
    tabIndex: 0,
    'aria-valuemin': CUT_MIN,
    'aria-valuemax': CUT_MAX,
    'aria-valuenow': cut,
    'aria-valuetext': `${cut}%`,
    onKeyDown
  } : onTune ? {
    role: 'button',
    tabIndex: 0,
    onKeyDown
  } : {
    role: 'img'
  };
  return <svg ref={ref} className={onCut ? 'ww-graph drag' : 'ww-graph'} viewBox={`0 0 ${W} ${h}`} aria-label={aria} {...a11y} onPointerMove={e => {
    const p = press.current;
    if (!p) return;
    if (!p.moved && Math.abs(e.clientX - p.x) + Math.abs(e.clientY - p.y) < 4) return;
    p.moved = true;
    setFrom(e.clientX);
  }} onPointerUp={() => {
    if (press.current && !press.current.moved) onTune?.();
    release();
  }} onPointerCancel={release}>
    <defs>
      <filter id={`${gid}-frost`} x="-30%" y="-30%" width="160%" height="160%" colorInterpolationFilters="sRGB">
        <feGaussianBlur in="SourceGraphic" stdDeviation="6" />
      </filter>
      <clipPath id={`${gid}-veil`}><rect x="0" y="0" width={cx} height={plotH} /></clipPath>
      <clipPath id={`${gid}-clear`}><rect x={cx} y="0" width={W - cx} height={plotH} /></clipPath>
    </defs>
    <rect className="lg-track" x="0" y="0" width={W} height={plotH} rx="7" />
    {GRID_V.map(g => <line key={g.id} className={g.mj ? 'lg-paper mj' : 'lg-paper'} x1={x(g.p)} x2={x(g.p)} y1="0" y2={plotH} />)}
    {rows.map(r => <line key={r.id} className="lg-paper" x1="0" x2={W} y1={r.y} y2={r.y} />)}
    {cut !== undefined && <rect className="lg-veil" x="0" y="0" width={cx} height={plotH} rx="7" />}
    {cut !== undefined && <rect className="lg-tint" x={cx} y="0" width={W - cx} height={plotH} />}
    {cut !== undefined && <g clipPath={`url(#${gid}-veil)`} filter={`url(#${gid}-frost)`}>{marks.map(m => renderMark(m, x, plotH))}</g>}
    <g clipPath={cut !== undefined ? `url(#${gid}-clear)` : undefined}>{marks.map(m => renderMark(m, x, plotH))}</g>
    <line className="lg-axis" x1="0" x2={W} y1={plotH} y2={plotH} />
    {TICKS.map(t => <g key={t.id}><line className="lg-axis" x1={x(t.p)} x2={x(t.p)} y1={plotH} y2={plotH + 4} /><text className="lg-al" x={x(t.p)} y={h - 1} textAnchor={t.p === 0 ? 'start' : t.p === 100 ? 'end' : 'middle'}>{t.p}%</text></g>)}
    {cut !== undefined && <g>
      <line className="lg-stem" x1={cx} x2={cx} y1="0" y2={plotH} />
      {held && <circle className="lg-heldring" cx={cx} cy={plotH / 2} r="18" />}
      <rect className={held ? 'lg-knob held' : 'lg-knob'} x={cx - 7} y={plotH / 2 - 14} width="14" height="28" rx="4.5" />
      <line className="lg-grip" x1={cx - 2} x2={cx - 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
      <line className="lg-grip" x1={cx + 2} x2={cx + 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
      {(onCut || onTune) && <rect className="lg-hit" x={cx - 18} y="0" width="36" height={plotH} onPointerDown={e => {
        (e.currentTarget.ownerSVGElement as SVGSVGElement).setPointerCapture(e.pointerId);
        press.current = {
          x: e.clientX,
          y: e.clientY,
          moved: false
        };
        setHeld(true);
      }} />}
    </g>}
    {cut !== undefined && <text className="lg-al lg-confidence" x="8" y={plotH - 8} textAnchor="start" style={{
      fontSize: '10px',
      fill: 'rgba(255,255,255,0.4)',
      fontStyle: 'italic'
    }}>{TEXT.tn_axis}</text>}
  </svg>;
}
/** A drawer's page duties: the page behind blurs, focus moves into the panel and back out after,
 *  and Escape closes it. Popups inside stop their own Escape before it reaches the window. */
function useDrawer(onClose: () => void) {
  const panel = useRef<HTMLDivElement>(null);
  const close = useRef(onClose);
  close.current = onClose;
  useEffect(() => {
    const back = document.activeElement as HTMLElement | null;
    document.body.classList.add('has-drawer');
    panel.current?.focus({
      preventScroll: true
    });
    const esc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') close.current();
    };
    window.addEventListener('keydown', esc);
    return () => {
      document.body.classList.remove('has-drawer');
      window.removeEventListener('keydown', esc);
      back?.focus?.({
        preventScroll: true
      });
    };
  }, []);
  return panel;
}
const toPlace = (s: TunerState, attempts: Attempt[], vadTries: number): TunerState => {
  const p = placement(attempts, s.day, s.roomReg);
  if (p.nogap) return {
    ...s,
    attempts,
    vadTries,
    phase: 'nogap',
    nogap: p.nogap
  };
  return {
    ...s,
    attempts,
    vadTries,
    phase: 'place',
    noise: p.noise,
    floorV: p.floorV,
    hiV: p.hiV,
    cutC: p.cutC
  };
};
/**
 * The tuner: Start opens a device session (the model floored, peers muted), the voice rounds land
 * as blue dots (near, near, far, then other people, the last skippable), then placement seeds the
 * knob inside the gap for Apply. There is no room-listening phase: the engine only ever reads the
 * threshold, and the room's last 24 hours is already on the graph as the amber smatter. Nor is
 * there a confirmation phase: the first real firing confirms itself by landing on the row's graph
 * with its ripple. `quick` is the knob-tap path: placement straight over the stored stats, no
 * session, with Re-Tune as the full re-measure.
 */
function Tuner({
  ctx,
  i,
  word,
  isStop,
  quick,
  seed,
  track,
  wakeRead,
  onClose
}: {
  ctx: Ctx;
  i: number;
  word: string;
  isStop: boolean;
  quick: boolean;
  seed: {
    cut: number;
    noise: number;
    floor: number;
    hi: number;
    day: number[];
  };
  track: Track | null;
  wakeRead: () => Promise<any>;
  onClose: () => void;
}) {
  const [st, setSt] = useState<TunerState>(() => ({
    phase: quick ? 'place' : 'ready',
    attempts: [],
    vadTries: 0,
    skipped: false,
    roomReg: 0,
    roomSeen: [],
    day: seed.day,
    noise: seed.noise,
    floorV: quick ? seed.floor : 0,
    hiV: quick ? seed.hi : 0,
    cutC: quick ? clampCut(pctN(seed.cut || 130)) : 55
  }));
  // The same-area peers this session holds muted (src/lib/peermute.js), so they do not answer the
  // word being said over and over. Peers self-heal on a 60s TTL, so every release is best effort.
  const heldRef = useRef<any[]>([]);
  const [pm, setPm] = useState<{
    muted: string[];
    failed: string[];
    unknown: boolean;
  } | null>(null);
  const releasePeers = () => {
    if (heldRef.current.length) releasePeerMutes(heldRef.current);
    heldRef.current = [];
  };
  const panel = useDrawer(onClose);
  // The session is opened by Start (the ready gate is the whole point), kept alive every 20s while
  // open, and closed on unmount whatever phase the panel died in. Quick edit never opens one:
  // placement against stored data needs no floored model. The peer holds ride the same lifecycle,
  // asked for at Start, reminded on the same cadence, released wherever the session ends.
  const openRef = useRef(false);
  const mounted = useRef(true);
  useEffect(() => {
    const ka = setInterval(() => {
      if (openRef.current) {
        post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => {});
        keepPeerMutes(heldRef.current);
      }
    }, 20000);
    return () => {
      mounted.current = false;
      clearInterval(ka);
      if (openRef.current) post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
      openRef.current = false;
      releasePeers();
    };
  }, [i]);
  const start = async () => {
    const r = await post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => null);
    if (!mounted.current) return;
    if (!r || !r.ok) {
      setSt(s => ({
        ...s,
        phase: 'gone'
      }));
      return;
    }
    let cap = 1;
    try {
      cap = JSON.parse(r.text).cap ?? 1;
    } catch {
      /* firmware without `cap`: assume able */
    }
    openRef.current = true;
    // The peer holds run in parallel with the rounds; nothing here blocks the measurement. Re-Tune
    // re-enters start() with the holds already placed, and asking again would only re-run the
    // sign-ins, so the ask happens once and the keepalive carries it from there.
    if (!heldRef.current.length) {
      holdPeerMutes(ctx.ha, ctx.device?.mac).then((res: any) => {
        if (!mounted.current) {
          if (res.held.length) releasePeerMutes(res.held);
          return;
        }
        heldRef.current = res.held;
        if (res.held.length || res.failed.length || res.unknown) setPm({
          muted: res.held.map((p: any) => p.name),
          failed: res.failed,
          unknown: res.unknown
        });
      });
    }
    setSt(s => ({
      ...s,
      phase: cap === 0 ? 'nocap' : 'voice',
      attempts: [],
      vadTries: 0,
      skipped: false
    }));
  };
  // A session that died device-side, or a build that cannot score, has no live holds to justify:
  // the peers go back to their own mute states while the panel shows its message.
  useEffect(() => {
    if (st.phase === 'gone' || st.phase === 'nocap') releasePeers();
  }, [st.phase]);
  // The session's event ring drives the voice phase; the payload's day buckets keep the smatter
  // fresh in every phase.
  useEffect(() => {
    let live = true;
    const tick = async () => {
      if (!live) return;
      const d = await wakeRead();
      if (!live) return;
      const day = (i === STOP_SLOT ? d?.stopw?.day : d?.slots?.find((x: Track) => x.i === i)?.day) || null;
      const tune = d?.tune && d.tune.i === i ? d.tune : null;
      const ev = tune ? tune.ev || [] : null;
      const reg = tune ? tune.room || 0 : 0;
      setSt(s => {
        let next = s;
        if (day) next = {
          ...next,
          day
        };
        // A higher register reading lands as a new dot; earlier ones never move. The running max
        // still feeds the placement math, but one live dot sliding to each new max made the whole
        // graph appear to jump mid-session (owner's report, September 22 2026).
        if (reg > next.roomReg && (next.phase === 'voice' || next.phase === 'place')) next = {
          ...next,
          roomReg: reg,
          roomSeen: [...next.roomSeen, reg]
        };
        if (next.phase !== 'voice') return next;
        if (ev === null) return {
          ...next,
          phase: 'gone'
        };
        const {
          attempts,
          vadTries
        } = attemptsOf(ev);
        const enough = attempts.length >= 4 || attempts.length >= 3 && next.skipped;
        return enough ? toPlace(next, attempts, vadTries) : {
          ...next,
          attempts,
          vadTries
        };
      });
      setTimeout(tick, 700);
    };
    tick();
    return () => {
      live = false;
    };
  }, [i]);
  const skip = () => setSt(s => s.attempts.length >= 3 ? toPlace(s, s.attempts, s.vadTries) : {
    ...s,
    skipped: true
  });
  const apply = async () => {
    // The room's reach: the loudest of the day's buckets, the session register, and whatever a
    // previous tune persisted.
    const noise = Math.max(0, ...st.day, st.roomReg, st.noise);
    const r = await post(cutoffPath(i, st.cutC, noise, st.floorV, st.hiV)).catch(() => null);
    if (!r?.ok) return;
    if (openRef.current) {
      openRef.current = false;
      await post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
    }
    releasePeers();
    await wakeRead();
    onClose();
  };
  const clearHistory = async () => {
    await post(`/api/sat1/wakewords/clearhist?i=${i}`).catch(() => null);
    await wakeRead();
  };
  const smatter: Mark[] = [...roomMarks(st.day, 38, 80), ...sessionRoomMarks(st.roomSeen, 38, 80)];
  const tries: Mark[] = attemptMarks(st.attempts, 42, 26);
  const place = st.phase === 'place';
  const marks = st.phase === 'nocap' || st.phase === 'gone' ? [] : st.phase === 'ready' ? smatter : place && quick ? [...smatter, ...rowMarks(track, 42, 84)] : [...smatter, ...tries];
  const n = st.attempts.length;
  const prompt = n < 2 ? TEXT.tn2_near.replace('%s', word) : n < 3 ? TEXT.tn2_far : TEXT.tn2_other;
  const verdict = readout(st.cutC, st.floorV, st.day, st.roomReg);
  const readText = verdict.tone === 'warn' ? TEXT.tn2_high.replace('%s', `${verdict.floorC}%`) : verdict.tone === 'err' ? TEXT.tn2_low : `${TEXT.tn2_ok.replace('%s', `${verdict.c}%`)}${verdict.floorC ? TEXT.tn2_under.replace('%s', String(verdict.floorC - verdict.c)) : ''}.`;
  // The peer-muting status, explicit by owner decision (September 23 2026): who is muted for this
  // session, who could not be, or that nobody could even be looked for. Silent only when the roster
  // answered and named no same-area peer - there is nothing to say about an empty room.
  const pmNote = pm && <>
    {pm.muted.length > 0 && <p className="ww-note">{`${pm.muted.length === 1 ? TEXT.tn_pm_one : TEXT.tn_pm_many.replace('%s', String(pm.muted.length))} (${pm.muted.join(', ')})`}</p>}
    {pm.failed.length > 0 && <p className="ww-note warn">{TEXT.tn_pm_failed.replace('%s', pm.failed.join(', '))}</p>}
    {pm.unknown && <p className="ww-note warn">{TEXT.tn_pm_unknown}</p>}
  </>;
  const title = TEXT.tn_title.replace('%s', word);
  return createPortal([<div key="scrim" className="ww-scrim" />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={title}>
    <div className="ww-ptop"><h2 style={{
        display: 'flex',
        alignItems: 'center',
        gap: 8
      }}><span>{title}</span><HintBtn text={HINTS.living_graph} /></h2><button className="secondary" onClick={onClose}>Close</button></div>
    {/* One graph at one height through every phase: the canvas resizing between ready and the
        rounds read as a layout bug (owner's screenshots, September 22 2026). */}
    <TouchGraph gid={`tn${i}`} h={150} marks={marks} cut={place ? st.cutC : undefined} onCut={place ? c => setSt(s => ({
      ...s,
      cutC: c
    })) : undefined} aria={place ? `Trigger threshold for “${word}”` : `Tuning graph for “${word}”`} />
    {st.phase === 'ready' && <div><p className="ww-read dim">{(isStop ? TEXT.tn2_ready_stop : TEXT.tn2_ready).replace('%s', word)}</p><div className="ww-btns"><button className="primary" onClick={start}>{TEXT.tn2_start}</button><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
    {st.phase === 'voice' && <div><p className="ww-read"><strong>{prompt}</strong><span> ({Math.min(n, 4)} / 4)</span></p>{st.vadTries > 0 && <p className="ww-note warn">{TEXT.tn_vad}</p>}{pmNote}{n >= 3 && <div className="ww-btns"><button className="secondary" onClick={skip}>{TEXT.tn2_skip}</button></div>}</div>}
    {place && <div><p className={`ww-read ${verdict.tone}`}>{readText}</p>{pmNote}<div className="ww-acts">
      {/* Four verbs in this order (owner call): back into the rounds, wipe the 24h record, out,
          commit. */}
      <button className="secondary" onClick={start}>{TEXT.tn2_retune}</button>
      <button className="secondary" onClick={clearHistory}>{TEXT.tn2_clear}</button>
      <button className="secondary" onClick={onClose}>{TEXT.cancel}</button>
      <button className="primary" onClick={apply}>{TEXT.tn_apply}</button>
    </div></div>}
    {st.phase === 'nogap' && <div><p className="ww-read err">{st.nogap === 'room' ? TEXT.tn_nogap_room : TEXT.tn_nogap_voice}</p><div className="ww-btns"><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
    {(st.phase === 'nocap' || st.phase === 'gone') && <div><p className={st.phase === 'gone' ? 'ww-read warn' : 'ww-read dim'}>{st.phase === 'gone' ? TEXT.tn_gone : TEXT.tn_nocap}</p><div className="ww-btns"><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
  </div>], document.body);
}
function PipeDrawer({
  slotName,
  pipe,
  fsd,
  onClose
}: {
  slotName: string;
  pipe: {
    value: string;
    options: [string, string][];
    busy: boolean;
    onPick: (v: string) => void;
  } | null;
  fsd: {
    value: string;
    onPick: (v: string) => void;
  } | null;
  onClose: () => void;
}) {
  const panel = useDrawer(onClose);
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={`${TEXT.vp_label}, ${slotName}`} onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>{TEXT.vp_label}</h2><button className="secondary" onClick={onClose}>Close</button></div>
    {pipe && <>
      <h3 className="ww-sec">{TEXT.vp_label}<HintBtn text={<>{HINTS.voice_pipeline} <a href={TEXT.vp_docs_url} target="_blank" rel="noopener">{TEXT.vp_docs}</a></>} /></h3>
      <div className="ww-opts" role="radiogroup" aria-label={TEXT.vp_label}>{pipe.options.map(([id, label]) => <button key={id} type="button" role="radio" aria-checked={pipe.value === id} className={pipe.value === id ? 'ww-opt on' : 'ww-opt'} disabled={pipe.busy} onClick={() => pipe.value !== id && pipe.onPick(id)}><span>{label}</span><span className="ww-mark">{pipe.value === id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
    </>}
    {fsd && <>
      <h3 className={pipe ? 'ww-sec ww-sec-gap' : 'ww-sec'}>Finished Speaking Detection<HintBtn text={HINTS.finished_speaking} /></h3>
      <div className="ww-opts" role="radiogroup" aria-label="Finished Speaking Detection">{FSD_OPTIONS.map(([id, label]: [string, string]) => <button key={id} type="button" role="radio" aria-checked={fsd.value === id} className={fsd.value === id ? 'ww-opt on' : 'ww-opt'} onClick={() => fsd.value !== id && fsd.onPick(id)}><span>{label}</span><span className="ww-mark">{fsd.value === id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
      <p className="ww-foot">Controls how quickly the satellite detects end of speech.</p>
    </>}
  </div>], document.body);
}
function WordDrawer({
  slotName,
  entries,
  pending,
  current,
  taken,
  busy,
  onPick,
  onClose
}: {
  slotName: string;
  entries: Entry[];
  pending: {
    url: string;
    label: string;
    error: boolean;
  }[];
  current: string;
  taken: string;
  busy: boolean;
  onPick: (e: Entry | null) => void;
  onClose: () => void;
}) {
  const [q, setQ] = useState('');
  const [lang, setLang] = useState('all');
  const panel = useDrawer(onClose);
  const langs: string[] = langsOf(entries);
  const {
    shown,
    hidden
  } = filterEntries(entries, q, lang === 'all' ? '' : lang, current);
  const other = taken.toLowerCase();
  const speakable = canSpeak();
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={`Wake Word Picker, ${slotName}`} onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Wake Word Picker</h2><button className="secondary" onClick={onClose}>Close</button></div>
    <div className="ww-filter"><input className="ww-search" type="search" placeholder={TEXT.ww_search} aria-label={TEXT.ww_search} value={q} onChange={e => setQ(e.currentTarget.value)} />{langs.length > 1 && <Dropdown value={lang} options={[{
        id: 'all',
        label: TEXT.ww_all_langs
      }, ...langs.map(l => ({
        id: l,
        label: langLabel(l)
      }))]} onChange={setLang} label="Language" />}</div>
    <div className="ww-picker-list">
      <div className="ww-opts" role="radiogroup" aria-label="Wake Word">
        {!q.trim() && lang === 'all' && <button type="button" role="radio" aria-checked={!current} className={current ? 'ww-opt' : 'ww-opt on'} disabled={busy} onClick={() => onPick(null)}>
          <span className="ww-opt-label"><span className="ww-opt-word">{TEXT.ww_none}</span></span>
          <span className="ww-mark">{!current && <Check size={13} strokeWidth={3} />}</span>
        </button>}
        {shown.map((e: Entry) => {
          const isSelected = e.spec === current;
          const isTaken = !!other && e.word.toLowerCase() === other;
          // A disabled row names why ("on the other slot"); a grey row alone does not say.
          const note = [e.source, e.ver, e.unverified && TEXT.ww_unverified, isTaken && TEXT.ww_on_other].filter(Boolean).join(' · ');
          return <button key={e.key} type="button" role="radio" aria-checked={isSelected} className={isSelected ? 'ww-opt on' : 'ww-opt'} disabled={busy || isTaken} onClick={() => onPick(e)}>
            <span className="ww-opt-label"><span className="ww-opt-word">{e.word}</span><span className="ww-opt-source">{note}</span></span>
            {speakable && <span className="ww-speak" role="button" tabIndex={0} aria-label={`Preview ${e.word}`} onClick={ev => {
              ev.stopPropagation();
              speak(e.word);
            }} onKeyDown={ev => {
              if (ev.key !== 'Enter' && ev.key !== ' ') return;
              ev.preventDefault();
              ev.stopPropagation();
              speak(e.word);
            }}><SpeakIcon /></span>}
            <span className="ww-mark">{isSelected && <Check size={13} strokeWidth={3} />}</span>
          </button>;
        })}
      </div>
      {hidden > 0 && <p className="ww-foot">{hidden} {TEXT.ww_more}</p>}
      {pending.map(p => <p key={p.url} className="ww-foot">{p.label}: {p.error ? TEXT.ww_source_failed : TEXT.ww_source_loading}</p>)}
      <p className="ww-foot">More words come from the sources in the card below.</p>
    </div>
  </div>], document.body);
}
function Sources({
  sources,
  setSources,
  cat
}: {
  sources: Source[];
  setSources: (list: Source[]) => void;
  cat: Record<string, CatEntry>;
}) {
  const [url, setUrl] = useState('');
  const [bad, setBad] = useState(false);
  const [confirm, setConfirm] = useState<string | null>(null);
  const missing = DEFAULT_SOURCES.filter((d: Source) => !sources.some(s => s.url === d.url));
  const add = () => {
    const src = parseSource(url);
    if (!src) {
      setBad(true);
      return;
    }
    setBad(false);
    if (sources.some(s => s.url === src.url)) return;
    setSources([...sources, src]);
    setUrl('');
  };
  return <article className="ww-card">
    <div className="ww-head"><h2 className="ww-title"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M12 3v12m0 0-4-4m4 4 4-4M4 21h16" /></svg><span>{TEXT.ws_title}</span><HintBtn text={HINTS.wake_sources} /></h2></div>
    {sources.map(s => {
      const c = cat[s.url];
      return <div key={s.url}><div className="ww-src"><a href={s.url} target="_blank" rel="noreferrer">{s.label}</a>{!c?.error && <small>{c?.entries ? `${c.entries.length} ${TEXT.ws_words}` : TEXT.ww_source_loading}</small>}<button className="ww-x" aria-label={`Remove ${s.label}`} onClick={() => setConfirm(s.url)}>{TEXT.ws_remove_c}</button></div>
        {c?.error && <p className="ww-note warn">{TEXT.ww_source_failed}</p>}
        {confirm === s.url && <div className="ww-confirm"><span>{TEXT.ws_remove_t} {TEXT.ws_remove_b}</span><div><button className="secondary" onClick={() => setConfirm(null)}>Keep</button><button className="primary" onClick={() => {
              setSources(sources.filter(x => x.url !== s.url));
              setConfirm(null);
            }}>{TEXT.ws_remove_c}</button></div></div>}</div>;
    })}
    {missing.length > 0 && <p className="ww-foot"><button type="button" className="ww-link" onClick={() => setSources([...missing, ...sources])}>{TEXT.ws_restore}</button></p>}
    <form className="ww-add" noValidate onSubmit={e => {
      e.preventDefault();
      add();
    }}>
      <input type="url" placeholder={TEXT.ws_ph} value={url} onChange={e => {
        setUrl(e.currentTarget.value);
        setBad(false);
      }} aria-label="Source URL" /><button className="primary" type="submit" disabled={!url.trim()}>{TEXT.ws_add}</button>
    </form>
    {bad && <p className="ww-note err">{TEXT.ws_bad_url}</p>}
    <p className="ww-foot">{TEXT.ws_footer_q}<a href={REQUEST_WORD_URL} target="_blank" rel="noopener">{TEXT.ws_request}</a>{TEXT.ws_or}<a href={TRAIN_URL} target="_blank" rel="noopener">{TEXT.ws_train}</a>.</p>
  </article>;
}
const isOn = (e: any) => e.value === true || e.state === 'ON';
/**
 * The Wake Word tab. The device is the validator and the source of truth: the browser only
 * enumerates sources (src/lib/wakesources.js) and polls the swap it asked for, and a failed
 * download leaves the previous word listening, which the card says. A word's graph appears only
 * once it is tuned - its presence is the tuned state, with no badge - and an untuned word carries
 * the Tune it! button instead (owner call, September 2026).
 */
export function WakeTab({
  ctx
}: {
  ctx: Ctx;
}) {
  const {
    ha,
    haRefresh
  } = ctx;
  const [tuning, setTuning] = useState<Tuning | null>(null);
  const [swaps, setSwaps] = useState<Record<number, Swap | null>>({});
  const anyBusy = Object.values(swaps).some(s => s?.phase === 'busy');
  // The standing 2.5s poll is there because the graphs' dots and live landings are living facts.
  // Swaps and tune sessions run their own faster chained loops; the standing poll stands down
  // meanwhile so only one loop reads at a time.
  const {
    wake,
    wakeRead,
    setSlot
  } = useWakeSlots(tuning || anyBusy ? 0 : 2500);
  // The hook's null is both "not answered yet" and "no loader on this build" (a 404): one read of
  // our own tells them apart, so the not-available note never flashes while the first loads.
  const [settled, setSettled] = useState(false);
  const alive = useRef(true);
  useEffect(() => {
    wakeRead().then(() => alive.current && setSettled(true));
    return () => {
      alive.current = false;
    };
  }, []);
  const [sources, setSourcesState] = useState<Source[]>(readSources);
  const [cat, setCat] = useState<Record<string, CatEntry>>({});
  const setSources = (list: Source[]) => {
    setSourcesState(list);
    writeSources(list);
  };
  useEffect(() => {
    let live = true;
    for (const s of sources) {
      if (cat[s.url] && !cat[s.url].loading) continue;
      setCat(c => ({
        ...c,
        [s.url]: {
          loading: true
        }
      }));
      enumerateSource(s).then((entries: unknown[]) => live && setCat(c => ({
        ...c,
        [s.url]: {
          entries
        }
      }))).catch(() => live && setCat(c => ({
        ...c,
        [s.url]: {
          error: true
        }
      })));
    }
    return () => {
      live = false;
    };
  }, [sources]);
  const [picking, setPicking] = useState<number | null>(null);
  const [piping, setPiping] = useState<number | null>(null);
  const slots: Track[] = wake?.slots || [];
  const slotAt = (i: number) => slots.find(s => s.i === i);
  const activeWords = slots.filter(s => s.m && s.w).map(s => s.w as string);
  const assist = useAssist(ha, haRefresh, activeWords);
  useEffect(() => {
    haSyncOnce(haRefresh);
  }, []);
  // The one-shot mount repair: a word listening on the device but holding no Home Assistant select
  // (a browser closed mid-pairing, or a swap that predates the device-side reload) gets one. One
  // word per mount, because a second unslotted word would race the first for the same free select
  // on a stale view; the next visit catches it. syncSlot's own guards skip slotted words and stop
  // when no select is free. Keyed on the word list too, since HA and the slot read land in either
  // order.
  const repaired = useRef(false);
  useEffect(() => {
    if (repaired.current || !assist.ready) return;
    const orphan = activeWords.find(w => assist.pipelineFor(w) === null);
    if (!orphan) return;
    repaired.current = true;
    assist.syncSlot(orphan, true).catch(() => {});
  }, [assist.ready, activeWords.length]);

  // Home Assistant's wake word selects follow the device's slots. After a download the device asks
  // HA to reload this device's config entry, which tears down and rebuilds every entity, so the
  // deadline is 60s and the write only lands once the selects are back with the new option. An
  // empty `asst` is expected meanwhile and is waited out, not taken as a verdict: stopping on one
  // bad poll is how a download could strand the HA select on "No Wake Word" for good.
  const syncAssist = async (i: number, prevWord: string, nextWord: string) => {
    const deadline = Date.now() + 60000;
    for (;;) {
      const fresh = await requestJson('/api/sat1/ha').catch(() => null);
      const sel = fresh?.d?.asst?.s;
      if (Array.isArray(sel) && sel.length) {
        const write = pairingWrite(sel, i, prevWord, nextWord);
        if (!write) return;
        await post(`/api/sat1/ha/select?e=${encodeURIComponent(write[0])}&o=${encodeURIComponent(write[1])}`).catch(() => {});
        await haRefresh();
      }
      if (!alive.current || Date.now() > deadline) return;
      // A breather between rounds: HA needs seconds to reconnect and re-read after the reload, and
      // re-posting flat out burns the device's socket table while it does.
      await sleep(500);
    }
  };
  const choose = async (i: number, spec: string, word: string) => {
    const prevWord = slotAt(i)?.w || '';
    if (tuning && tuning.i === i) setTuning(null);
    setSwaps(s => ({
      ...s,
      [i]: {
        phase: 'busy',
        word,
        spec
      }
    }));
    const r = await setSlot(i, spec).catch(() => ({
      ok: false
    }));
    if (!r.ok) {
      if (alive.current) setSwaps(s => ({
        ...s,
        [i]: {
          phase: 'error',
          word,
          spec,
          err: 0
        }
      }));
      return;
    }
    const deadline = Date.now() + (isUrl(spec) ? 90000 : 10000);
    for (;;) {
      await sleep(isUrl(spec) ? 900 : 400);
      if (!alive.current) return;
      const d = await wakeRead();
      const s = d?.slots?.find((x: Track) => x.i === i);
      const step = swapStep(s, spec);
      if (step?.done) {
        setSwaps(prev => ({
          ...prev,
          [i]: null
        }));
        await syncAssist(i, prevWord, spec === 'none' ? '' : s.w || word);
        return;
      }
      if (step && 'err' in step) {
        setSwaps(prev => ({
          ...prev,
          [i]: {
            phase: 'error',
            word,
            spec,
            err: step.err
          }
        }));
        return;
      }
      if (step) setSwaps(prev => ({
        ...prev,
        [i]: {
          ...(prev[i] as Swap),
          dl: step.dl,
          tot: step.tot
        }
      }));
      if (Date.now() > deadline) {
        setSwaps(prev => ({
          ...prev,
          [i]: {
            phase: 'error',
            word,
            spec,
            err: 7
          }
        }));
        return;
      }
    }
  };

  // Finished Speaking Detection: the device keeps one select per slot and copies the firing slot's
  // value into Home Assistant's own before each request; `unset` follows HA. See docs/web-ui.md,
  // "Finished speaking detection, per wake word".
  const fsdRaw = ha?.d?.fsd;
  const haFsd: string | null = !haBlocked(ha) && !haTooOld(ha) && Array.isArray(fsdRaw) && fsdRaw.length === 2 ? fsdRaw[1] : null;
  const fsdOwn = FSD_KEYS.map((k: string) => entity(ctx, k)?.value);
  const fsdNow: string[] | null = fsdShown(fsdOwn, haFsd);
  const [fsdHold, setFsdHold] = useState<Record<number, string>>({});
  const pickFsd = async (i: number, option: string) => {
    const writes: [number, string][] = fsdWrites(fsdOwn, haFsd, i, option);
    setFsdHold(h => ({
      ...h,
      ...Object.fromEntries(writes)
    }));
    const done = await Promise.all(writes.map(([k, v]) => post(`${pathFor(ctx, FSD_KEYS[k], 'set')}?option=${encodeURIComponent(v)}`).catch(() => null)));
    // Held until the stream has had time to report the device's own value; a refused write
    // snaps back at once.
    const drop = () => alive.current && setFsdHold(h => {
      const next = {
        ...h
      };
      for (const [k, v] of writes) if (next[k] === v) delete next[k];
      return next;
    });
    if (done.some(r => !r?.ok)) drop();else setTimeout(drop, 2000);
  };
  const stopSwitch = entity(ctx, 'stop_word');
  const stopOn = stopSwitch ? isOn(stopSwitch) : false;
  // The switch is the preference; `stop_active` is whether the stop model runs right now, published
  // by the firmware's stop_word_arm/disarm scripts (voice_assistant.yaml) in the same instant they
  // flip the model and pushed over /events. Firmware without the sensor falls back to the switch,
  // never a false "Paused".
  const stopActive = entity(ctx, 'stop_active');
  const stopRunning: boolean | null = stopActive ? isOn(stopActive) : null;
  const wakeSound = entity(ctx, 'wake_sound');
  const chime: boolean | null = wakeSound ? isOn(wakeSound) : null;
  const heading = <><span className="eyebrow">WAKE · WAKE WORDS</span><h1>Say the <em>word.</em></h1></>;
  if (!ctx.device || !wake) return <section className="control wake-section">
    {heading}
    {ctx.device && settled && <article className="ww-card"><p className="ww-note">Wake word control is not available on this firmware build.</p></article>}
  </section>;
  const stopw: Track | null = wake.stopw || null;
  const pipeOptions: [string, string][] = [[PIPELINE_PREFERRED, TEXT.pipeline_preferred], ...assist.pipelines.map((p: string) => [p, p] as [string, string])];
  const pipeValue = (w: string): string => assist.pipelineFor(w) ?? assist.fallbackPipeline() ?? PIPELINE_PREFERRED;
  const pipeLabel = (v: string) => v === PIPELINE_PREFERRED ? TEXT.pipeline_preferred : v;
  // Everything the graph already knows about a track, so quick edit reopens placement with no
  // re-recording.
  const tuneSeed = (i: number) => {
    const t = i === STOP_SLOT ? stopw : slotAt(i);
    return {
      cut: t?.cut || 0,
      noise: t?.tn?.[0] || 0,
      floor: t?.tn?.[1] || 0,
      hi: t?.tn?.[2] || 0,
      day: t?.day || []
    };
  };
  const assistNote = !assist.ready && activeWords.length > 0 && <p className="ww-note">
    {haBlocked(ha) ? TEXT.assistant_blocked : haTooOld(ha) ? TEXT.ha_too_old : TEXT.assistant_needs_ha}
    {haBlocked(ha) && <> <button type="button" className="ww-link" onClick={ctx.onShowFix}>{TEXT.show_fix}</button></>}
  </p>;
  const wordCard = (i: number) => {
    const slot = slotAt(i);
    const swap = swaps[i];
    const swapping = swap?.phase === 'busy';
    const failed = swap?.phase === 'error';
    const word = swapping ? swap.word : slot?.w || '';
    const waiting = !swap && !!slot && (slot.st === 3 || slot.st === 2 && !slot.ld && isUrl(slot.m));
    const tuned = !swapping && !!slot && slot.cut > 0;
    const live = tuned && !!slot.ld;
    const tuneBtn = !tuned && !swapping && !failed && !!slot?.ld;
    const pipe = !swapping && word && assist.ready ? pipeValue(word) : null;
    let note: [string, string] | null = null;
    if (swapping) {
      const progress = (swap.tot || 0) > 0 ? `${kb(swap.dl || 0)} / ${kb(swap.tot as number)} KB` : isUrl(swap.spec) ? TEXT.ww_downloading : TEXT.ww_loading;
      note = ['', slot?.w && slot.w !== swap.word ? `${progress} ${TEXT.mb_swap_note.replace('%1', showWord(slot.w)).replace('%2', showWord(swap.word))}` : progress];
    } else if (failed) note = ['err', `${TEXT.ww_failed} ${WW_ERR[swap.err || 0] || ''}`.trim()];else if (waiting) note = ['warn', slot.st === 3 ? TEXT.ww_waiting : `${WW_ERR[slot.err || 0] || ''} ${TEXT.ww_retrying}`.trim()];
    return <article className="ww-card" key={`w${i}`}>
      <div className="ww-head"><h2 className="ww-title"><span>Wake Word {i + 1}</span><HintBtn text={HINTS.wake_words} /></h2>
        {i === 0 && chime !== null && <span className="ww-title"><button className={chime ? 'ww-bell' : 'ww-bell off'} aria-pressed={chime} aria-label={TEXT.ww_chime} onClick={() => post(pathFor(ctx, 'wake_sound', chime ? 'turn_off' : 'turn_on'))}>
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M6 8a6 6 0 0 1 12 0c0 7 3 9 3 9H3s3-2 3-9M10.3 21a1.94 1.94 0 0 0 3.4 0" />{!chime && <path d="m3 3 18 18" />}</svg>
        </button><HintBtn text={HINTS.wake_sound} /></span>}</div>
      <div className="ww-flow">
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Wake Word</span>
          <button className={picking === i ? 'ww-flow-btn open' : 'ww-flow-btn'} aria-haspopup="dialog" disabled={swapping} onClick={() => setPicking(i)}>
            <span>{word ? showWord(word) : 'None'}</span>
            <ChevronDown size={16} />
          </button>
        </div>
        {pipe !== null && <>
          <span className="ww-flow-arrow" aria-hidden="true"><ArrowRight size={18} /></span>
          <div className="ww-flow-cell">
            <span className="ww-flow-cap">{TEXT.vp_label}</span>
            <button className={piping === i ? 'ww-flow-btn open' : 'ww-flow-btn'} aria-haspopup="dialog" onClick={() => setPiping(i)}>
              <span>{pipeLabel(pipe)}</span>
              <ChevronDown size={16} />
            </button>
          </div>
        </>}
      </div>
      {note && <p className={note[0] ? `ww-note ${note[0]}` : 'ww-note'}>{note[1]}</p>}
      {failed && <div className="ww-btns"><button className="primary" onClick={() => choose(i, swap.spec, swap.word)}>{TEXT.ww_retry}</button></div>}
      {i === 0 && activeWords.length === 0 && !anyBusy && <p className="ww-note">{TEXT.ww_route_none}</p>}
      {i === 0 && assistNote}
      {(tuned || tuneBtn) && <div className="ww-top">
        {live && <span className="ww-live"><i />{TEXT.lg_listening}</span>}
        {tuned && <HintBtn text={HINTS.living_graph} />}
        {tuneBtn && <button className="ww-tune" onClick={() => setTuning({
          i,
          word,
          isStop: false,
          quick: false
        })}>{TEXT.ww_tune_btn}</button>}
      </div>}
      {tuned && <TouchGraph gid={`g${i}`} h={72} marks={rowMarks(slot)} cut={pctN(slot.cut)} onTune={() => setTuning({
        i,
        word,
        isStop: false,
        quick: true
      })} aria={TEXT.tn_title.replace('%s', showWord(word))} />}
    </article>;
  };
  const stopCard = () => {
    if (!stopw || !stopSwitch) return null;
    const tuned = stopOn && stopw.cut > 0;
    // Three states (owner's report, September 2026: the model is disabled in steady state, so a
    // standing "Live" was a lie). Green "Listening" whenever the model genuinely runs, which
    // wins even with the switch off because a ringing timer arms it regardless; yellow "Paused"
    // while armed but idle, tuned or not.
    const live = stopRunning === null ? tuned : stopRunning;
    const paused = stopRunning === false && stopOn;
    const tuneBtn = stopOn && !tuned;
    return <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><span>Stop Word</span><HintBtn text={HINTS.stop_word} /></h2></div>
      <button className={`wsw${stopOn ? ' on' : ''}`} style={{
        marginTop: 14,
        marginBottom: 16
      }} role="switch" aria-checked={stopOn} onClick={() => post(pathFor(ctx, 'stop_word', stopOn ? 'turn_off' : 'turn_on'))}>
        {!stopOn && <span className="wsw-knob" />}
        <span>Stop</span>
        {stopOn && <span className="wsw-knob" />}
      </button>
      {(live || paused || tuneBtn) && <div className="ww-top">
        {live ? <span className="ww-live"><i />{TEXT.lg_listening}</span> : paused ? <span className="ww-live paused"><i />{TEXT.lg_paused}</span> : null}
        {tuneBtn && <button className="ww-tune" onClick={() => setTuning({
          i: STOP_SLOT,
          word: 'stop',
          isStop: true,
          quick: false
        })}>{TEXT.ww_tune_btn}</button>}
      </div>}
      {tuned && <TouchGraph gid="gs" h={72} marks={rowMarks(stopw)} cut={pctN(stopw.cut)} onTune={() => setTuning({
        i: STOP_SLOT,
        word: 'stop',
        isStop: true,
        quick: true
      })} aria={TEXT.tn_title.replace('%s', 'Stop')} />}
    </article>;
  };
  const pickerFor = (i: number) => {
    const entries: Entry[] = pickerEntries(wake.builtin, TEXT.ww_included, sources, cat);
    const pending = sources.filter(s => cat[s.url]?.loading || cat[s.url]?.error).map(s => ({
      url: s.url,
      label: s.label,
      error: !!cat[s.url]?.error
    }));
    const current = swaps[i]?.phase === 'busy' ? (swaps[i] as Swap).spec : slotAt(i)?.m || '';
    return <WordDrawer slotName={`Wake Word ${i + 1}`} entries={entries} pending={pending} current={current} taken={slotAt(1 - i)?.w || swaps[1 - i]?.word || ''} busy={anyBusy} onPick={e => {
      setPicking(null);
      if (!e) {
        if (current) choose(i, 'none', '');
      } else if (e.spec !== current) choose(i, e.spec, e.word);
    }} onClose={() => setPicking(null)} />;
  };
  const pipeFor = (i: number) => {
    const w = slotAt(i)?.w || '';
    if (!w) return null;
    return <PipeDrawer slotName={`Wake Word ${i + 1}`} pipe={assist.ready ? {
      value: pipeValue(w),
      options: pipeOptions,
      busy: assist.busy,
      onPick: v => assist.setPipeline(w, v)
    } : null} fsd={fsdNow ? {
      value: fsdHold[i] ?? fsdNow[i],
      onPick: v => pickFsd(i, v)
    } : null} onClose={() => setPiping(null)} />;
  };
  return <section className="control wake-section">
    {heading}
    {wordCard(0)}
    {wordCard(1)}
    {stopCard()}
    <Sources sources={sources} setSources={setSources} cat={cat} />
    {picking !== null && pickerFor(picking)}
    {piping !== null && pipeFor(piping)}
    {tuning && <Tuner key={`${tuning.i}-${tuning.quick}`} ctx={ctx} i={tuning.i} word={showWord(tuning.word)} isStop={tuning.isStop} quick={tuning.quick} seed={tuneSeed(tuning.i)} track={tuning.i === STOP_SLOT ? stopw : slotAt(tuning.i) || null} wakeRead={wakeRead} onClose={() => setTuning(null)} />}
  </section>;
}
