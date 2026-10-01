import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import type { Dispatch, PointerEvent as RPointerEvent, ReactNode, SetStateAction } from 'react';
import { createPortal } from 'react-dom';
import { HINTS, PRESENCE, TEXT } from '../../src/copy.js';
import { BASE, entity, pathFor, post, RADAR_LIVE_MS, useRadar } from '../../src/lib/device.js';
import { RadarIcon } from '../icons';
import { clampShift, direction, distLabel, feetOn, gateArray, gateLabel, inPolygon, labelPoint, nearest, presenceLabel, rangeLabel, realTargets, shapeBody, stepTrails } from '../lib/presence.js';
import { MSlider } from './MSlider';
import type { Ctx } from '../ctx';

/**
 * Presence, on satellite1_radar's own JSON API rather than entities: the radar's settings live in
 * the module's config behind /api/v1/<module>/config, as v1's route reads them. useRadar owns the
 * probe, the chained live poll (only while this tab is mounted), the optimistic writes, the
 * LD2410's /apply and the debounced flash save.
 */

const PLOT_HALF_W = 400;
const PLOT_DEPTH = 640;
const FOV_X = Math.sin(60 * Math.PI / 180) * PLOT_DEPTH;
const FOV_Y = Math.cos(60 * Math.PI / 180) * PLOT_DEPTH;
/** The handler stores at most this many corners per polygon and refuses the whole POST past it. */
const MAX_POINTS = 8;
const TRAIL_LEN = 6;
const GLIDE = `transform ${RADAR_LIVE_MS}ms linear`;
const RINGS = [{
  id: 'r200',
  r: 200
}, {
  id: 'r400',
  r: 400
}, {
  id: 'r600',
  r: 600
}];
type Pt = {
  x: number;
  y: number;
};
type Target = Pt & {
  i: number;
};
type Edit = {
  from: number | 'x' | null;
  which: number | 'x';
  points: Pt[];
  sel: number | null;
  hist: Pt[][];
} | null;
type Radar = ReturnType<typeof useRadar>;
type Pill = {
  l: string;
  v: string;
  exp?: boolean;
};
type ViewProps = {
  ctx: Ctx;
  radar: Radar;
  isFt: boolean;
  setFt: ((v: boolean) => void) | null;
};
const num = (v: unknown) => Number(v) || 0;
const polygonPoints = (pts: Pt[]) => pts.map(p => `${p.x},${p.y}`).join(' ');
let _openHintSetter: ((v: boolean) => void) | null = null;
function HintBtn({
  text
}: {
  text: ReactNode;
}) {
  const [open, setOpen] = useState(false);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState<{
    left: number;
    top: number;
  } | null>(null);
  useEffect(() => {
    if (!open) return;
    if (_openHintSetter && _openHintSetter !== setOpen) _openHintSetter(false);
    _openHintSetter = setOpen;
    const dismiss = (e: PointerEvent) => {
      if (!btn.current?.contains(e.target as Node) && !bubble.current?.contains(e.target as Node)) setOpen(false);
    };
    const esc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setOpen(false);
    };
    document.addEventListener('pointerdown', dismiss, true);
    document.addEventListener('keydown', esc);
    return () => {
      document.removeEventListener('pointerdown', dismiss, true);
      document.removeEventListener('keydown', esc);
      if (_openHintSetter === setOpen) _openHintSetter = null;
    };
  }, [open]);
  useLayoutEffect(() => {
    if (!open) {
      setPos(null);
      return;
    }
    const id = requestAnimationFrame(() => {
      if (!btn.current || !bubble.current) return;
      const t = btn.current.getBoundingClientRect();
      const b = bubble.current.getBoundingClientRect();
      const GAP = 6;
      const MARGIN = 8;
      let left = t.left + t.width / 2 - b.width / 2;
      left = Math.max(MARGIN, Math.min(left, window.innerWidth - b.width - MARGIN));
      const below = t.bottom + GAP;
      const top = below + b.height + MARGIN > window.innerHeight ? Math.max(MARGIN, t.top - b.height - GAP) : below;
      setPos({
        left,
        top
      });
    });
    return () => cancelAnimationFrame(id);
  }, [open]);
  return <span className="ww-hint-wrap">
    <button ref={btn} type="button" className="ww-hint-btn" aria-label="More information" aria-expanded={open} onClick={() => setOpen(v => !v)}>i</button>
    {open && createPortal(<div ref={bubble} className="ww-hint-bubble" role="tooltip" style={pos ? {
      left: pos.left,
      top: pos.top,
      opacity: 1
    } : {
      left: -9999,
      top: -9999,
      opacity: 0
    }}>{text}</div>, document.body)}
  </span>;
}
function Switch({
  on,
  onChange,
  label,
  disabled
}: {
  on: boolean;
  onChange: (v: boolean) => void;
  label: string;
  disabled?: boolean;
}) {
  return <button type="button" role="switch" aria-checked={on} aria-label={label} disabled={disabled} onClick={() => onChange(!on)} className={`pr-switch${on ? ' on' : ''}`}>
      <span className="pr-switch-thumb" />
    </button>;
}

/**
 * Slider writes, sent only when the value changed. MSlider commits on every pointer-up and key-up,
 * Tab landing on it included, and each commit here is a config POST (and an /apply on the LD2410).
 * A previewed field has already been patched locally, so it is compared with its pre-drag value.
 */
function useSettings(config: any, write: (patch: any) => unknown, preview: (patch: any) => void) {
  const before = useRef<Record<string, unknown>>({});
  return {
    preview: (k: string, v: unknown) => {
      if (!(k in before.current)) before.current[k] = config[k];
      preview({
        [k]: v
      });
    },
    commit: (k: string, v: unknown) => {
      const was = k in before.current ? before.current[k] : config[k];
      delete before.current[k];
      if (v !== was) write({
        [k]: v
      });
    }
  };
}
function Plot({
  zones,
  exclusion,
  range,
  targets,
  trails,
  edit,
  setEdit,
  onOpen,
  isFt
}: {
  zones: Pt[][];
  exclusion: Pt[];
  range: number;
  targets: Target[];
  trails: Record<number, Pt[]>;
  edit: Edit;
  setEdit: Dispatch<SetStateAction<Edit>>;
  onOpen: (which: number | 'x') => void;
  isFt: boolean;
}) {
  const svgRef = useRef<SVGSVGElement>(null);
  const dragRef = useRef<number | 'all' | null>(null);
  const preRef = useRef<Pt[]>([]);
  const lastRef = useRef<Pt>({
    x: 0,
    y: 0
  });
  const startRef = useRef<Pt>({
    x: 0,
    y: 0
  });
  const movedRef = useRef(false);
  const addedRef = useRef(false);
  const changedRef = useRef(false);
  const toPoint = (e: RPointerEvent): Pt => {
    const r = svgRef.current!.getBoundingClientRect();
    const x = (e.clientX - r.left) / r.width * PLOT_HALF_W * 2 - PLOT_HALF_W;
    const y = (e.clientY - r.top) / r.height * PLOT_DEPTH;
    return {
      x: Math.round(Math.max(-PLOT_HALF_W, Math.min(PLOT_HALF_W, x))),
      y: Math.round(Math.max(0, Math.min(PLOT_DEPTH, y)))
    };
  };
  const down = (e: RPointerEvent) => {
    const p = toPoint(e);
    if (!edit) {
      for (let i = 0; i < 3; i++) {
        if ((zones[i] || []).length > 2 && inPolygon(p, zones[i])) {
          onOpen(i);
          return;
        }
      }
      if (exclusion.length > 2 && inPolygon(p, exclusion)) onOpen('x');
      return;
    }
    e.preventDefault();
    preRef.current = edit.points.map(q => ({
      ...q
    }));
    startRef.current = p;
    lastRef.current = p;
    movedRef.current = false;
    addedRef.current = false;
    changedRef.current = false;
    const near = edit.points.findIndex(q => Math.hypot(q.x - p.x, q.y - p.y) < 40);
    if (near >= 0) {
      dragRef.current = near;
    } else if (edit.points.length > 2 && inPolygon(p, edit.points)) {
      dragRef.current = 'all';
    } else {
      if (edit.points.length >= MAX_POINTS) return;
      setEdit(ed => ed && {
        ...ed,
        points: [...ed.points, p],
        sel: null
      });
      dragRef.current = edit.points.length;
      addedRef.current = true;
      changedRef.current = true;
    }
    svgRef.current!.setPointerCapture(e.pointerId);
  };
  const move = (e: RPointerEvent) => {
    if (dragRef.current === null || !edit) return;
    const p = toPoint(e);
    if (dragRef.current === 'all') {
      const [dx, dy] = clampShift(edit.points, p.x - lastRef.current.x, p.y - lastRef.current.y, PLOT_HALF_W, PLOT_DEPTH);
      if (dx === 0 && dy === 0) return;
      movedRef.current = true;
      changedRef.current = true;
      lastRef.current = {
        x: lastRef.current.x + dx,
        y: lastRef.current.y + dy
      };
      setEdit(ed => ed && {
        ...ed,
        points: ed.points.map(q => ({
          x: q.x + dx,
          y: q.y + dy
        }))
      });
    } else {
      if (!movedRef.current && Math.hypot(p.x - startRef.current.x, p.y - startRef.current.y) <= 12) return;
      movedRef.current = true;
      changedRef.current = true;
      const idx = dragRef.current;
      setEdit(ed => ed && {
        ...ed,
        points: ed.points.map((q, i) => i === idx ? p : q)
      });
    }
  };
  const up = () => {
    const held = dragRef.current;
    dragRef.current = null;
    if (!edit) return;
    if (changedRef.current) {
      changedRef.current = false;
      const snap = preRef.current;
      setEdit(ed => ed && {
        ...ed,
        hist: [...ed.hist, snap]
      });
    }
    if (typeof held === 'number' && !movedRef.current && !addedRef.current) {
      setEdit(ed => ed && {
        ...ed,
        sel: ed.sel === held ? null : held
      });
    }
  };
  const hideZone = (i: number) => edit && (edit.from === i || edit.which === i);
  const hideExcl = edit && (edit.from === 'x' || edit.which === 'x');
  const zoneList = zones.map((z, i) => ({
    id: `z${i}`,
    i,
    z
  }));
  const vtx = edit ? edit.points.map((p, i) => ({
    id: `v${i}-${p.x}-${p.y}`,
    i,
    p
  })) : [];
  const effectiveRange = range > 0 ? range : PLOT_DEPTH;
  return <div className="pr-plot">
    <svg ref={svgRef} viewBox={`${-PLOT_HALF_W} 0 ${PLOT_HALF_W * 2} ${PLOT_DEPTH}`} className={`pr-plot-svg${edit ? ' editing' : ''}`} style={edit ? {
      touchAction: 'none'
    } : undefined} onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up} role="img" aria-label="Radar plot of the room">
      <defs>
        <radialGradient id="pr-fovg" gradientUnits="userSpaceOnUse" cx="0" cy="0" r={PLOT_DEPTH}>
          <stop offset="0" stopColor="rgba(147,180,253,0.10)" />
          <stop offset="1" stopColor="rgba(147,180,253,0.01)" />
        </radialGradient>
        <filter id="pr-glowf" x="-150%" y="-150%" width="400%" height="400%">
          <feGaussianBlur stdDeviation="12" />
        </filter>
        <filter id="pr-fog" x="-20%" y="-20%" width="140%" height="140%">
          <feGaussianBlur stdDeviation="7" />
        </filter>
        <mask id="pr-out-mask">
          <rect x={-PLOT_HALF_W} y="0" width={PLOT_HALF_W * 2} height={PLOT_DEPTH} fill="white" />
          <circle cx="0" cy="0" r={effectiveRange} fill="black" />
        </mask>
      </defs>
      <path fill="url(#pr-fovg)" stroke="rgba(147,180,253,0.25)" strokeWidth="1" d={`M0 0 L${-FOV_X} ${FOV_Y} A${PLOT_DEPTH} ${PLOT_DEPTH} 0 0 0 ${FOV_X} ${FOV_Y} Z`} />
      {RINGS.map(r => <circle key={r.id} cx="0" cy="0" r={r.r} fill="none" stroke="rgba(147,180,253,0.30)" strokeWidth="2" strokeDasharray="10 14" />)}
      <line x1={-PLOT_HALF_W} y1="0" x2={PLOT_HALF_W} y2="0" stroke="rgba(255,255,255,0.08)" strokeWidth="1" />
      <line x1="0" y1="0" x2="0" y2={PLOT_DEPTH} stroke="rgba(255,255,255,0.08)" strokeWidth="1" />
      {zoneList.map(({
        id,
        i,
        z
      }) => !hideZone(i) && z && z.length > 2 ? <g key={id}>
        <polygon points={polygonPoints(z)} fill={`rgba(37,99,235,${0.12 + i * 0.04})`} stroke="rgba(147,180,253,0.45)" strokeWidth="1.5" />
        <text className="pr-plot-name" x={labelPoint(z).x} y={labelPoint(z).y}>{`Zone ${i + 1}`}</text>
      </g> : null)}
      {!hideExcl && exclusion.length > 2 && <g>
        <polygon points={polygonPoints(exclusion)} fill="rgba(251,191,36,0.10)" stroke="rgba(251,191,36,0.45)" strokeWidth="1.5" strokeDasharray="5 3" />
        <text className="pr-plot-name pr-excl-name" x={labelPoint(exclusion).x} y={labelPoint(exclusion).y}>Exclusion</text>
      </g>}
      {edit && edit.points.length > 0 && <g>
        {edit.points.length > 2 ? <polygon points={polygonPoints(edit.points)} fill={edit.which === 'x' ? 'rgba(251,191,36,0.15)' : 'rgba(147,180,253,0.18)'} stroke={edit.which === 'x' ? 'rgba(251,191,36,0.7)' : 'rgba(147,180,253,0.8)'} strokeWidth="1.5" strokeDasharray={edit.which === 'x' ? '5 3' : undefined} /> : <polyline points={polygonPoints(edit.points)} fill="none" stroke="rgba(147,180,253,0.6)" strokeWidth="1.5" />}
        {edit.points.length > 2 && <text className={`pr-plot-name${edit.which === 'x' ? ' pr-excl-name' : ''}`} x={labelPoint(edit.points).x} y={labelPoint(edit.points).y}>
          {edit.which === 'x' ? 'Exclusion' : `Zone ${(edit.which as number) + 1}`}
        </text>}
        {vtx.map(({
          id,
          i,
          p
        }) => <circle key={id} cx={p.x} cy={p.y} r="16" fill={edit.sel === i ? 'rgba(147,180,253,0.5)' : 'rgba(147,180,253,0.15)'} stroke={edit.sel === i ? '#93b4fd' : 'rgba(147,180,253,0.6)'} strokeWidth="2" />)}
      </g>}
      {range > 0 && <g>
        <g filter="url(#pr-fog)" mask="url(#pr-out-mask)" opacity="0.4">
          <path fill="url(#pr-fovg)" d={`M0 0 L${-FOV_X} ${FOV_Y} A${PLOT_DEPTH} ${PLOT_DEPTH} 0 0 0 ${FOV_X} ${FOV_Y} Z`} />
          {RINGS.map(r => <circle key={r.id} cx="0" cy="0" r={r.r} fill="none" stroke="rgba(147,180,253,0.30)" strokeWidth="2" strokeDasharray="10 14" />)}
        </g>
        <rect x={-PLOT_HALF_W} y="0" width={PLOT_HALF_W * 2} height={PLOT_DEPTH} fill="rgba(10,10,16,0.62)" mask="url(#pr-out-mask)" />
      </g>}
      {range > 0 && <g>
        <circle cx="0" cy="0" r={range} fill="rgba(96,165,250,0.04)" stroke="none" />
        <circle cx="0" cy="0" r={range} fill="none" stroke="#60a5fa" strokeWidth="2" opacity="0.85" />
      </g>}
      {targets.map(t => <g key={`t${t.i}`}>
        {(trails[t.i] || []).slice(1).map((p, j) => {
          const age = j + 1;
          const opacity = 1 - age / 6.5;
          const r = Math.max(4, 14 - age * 1.8);
          return <g key={`tr${age}`} style={{
            transform: `translate(${p.x}px,${p.y}px)`,
            transition: GLIDE
          }}>
            <circle cx="0" cy="0" r={r * 2.2} fill="rgba(96,165,250,0.12)" style={{
              opacity
            }} />
            <circle cx="0" cy="0" r={r} fill="#60a5fa" style={{
              opacity: opacity * 0.7
            }} />
          </g>;
        })}
        <g style={{
          transform: `translate(${t.x}px,${t.y}px)`,
          transition: GLIDE
        }}>
          <circle className="pr-tgt-halo" cx="0" cy="0" r="42" fill="rgba(96,165,250,0.10)" />
          <circle cx="0" cy="0" r="26" fill="rgba(96,165,250,0.22)" filter="url(#pr-glowf)" />
          <circle cx="0" cy="0" r="11" fill="#60a5fa" style={{
            filter: 'drop-shadow(0 0 8px rgba(96,165,250,0.9))'
          }} />
          <circle cx="-3" cy="-3" r="3.5" fill="rgba(255,255,255,0.7)" />
        </g>
      </g>)}
      {RINGS.map(r => <text key={r.id} className="pr-tick" x="14" y={Math.min(r.r + 26, PLOT_DEPTH - 10)}>
        {isFt ? `${Math.round(r.r / 30.48 * 10) / 10} ft` : `${r.r / 100}m`}
      </text>)}
    </svg>
  </div>;
}

/** The status row. The Distance pill opens the unit switch, and only when the firmware has one. */
function StatusPills({
  pills,
  isFt,
  setFt
}: {
  pills: Pill[];
  isFt: boolean;
  setFt: ((v: boolean) => void) | null;
}) {
  const [open, setOpen] = useState(false);
  return <div className="pr-card">
    <div className="pr-pills">
      {pills.map(p => p.exp && setFt ? <button key={p.l} type="button" className={`pr-pill${open ? ' open' : ''}`} style={{
        cursor: 'pointer',
        minHeight: 44
      }} aria-expanded={open} onClick={() => setOpen(o => !o)}>
          <span className="pr-pill-v">{p.v}</span><span className="pr-pill-l">{`${p.l} ›`}</span>
        </button> : <div key={p.l} className="pr-pill"><span className="pr-pill-v">{p.v}</span><span className="pr-pill-l">{p.l}</span></div>)}
    </div>
    {open && setFt && <div className="pr-pill-expand">
      <span>Feet</span>
      <HintBtn text={HINTS.distance_unit} />
      <span style={{
        flex: 1
      }} />
      <Switch on={isFt} onChange={setFt} label="Feet" />
    </div>}
  </div>;
}
function CardHead({
  title
}: {
  title: string;
}) {
  return <div className="pr-card-head">
    <span className="pr-card-title">{title}</span>
    <span className="pr-live-badge" title={`Polling every ${RADAR_LIVE_MS}ms`}>● live</span>
    <span style={{
      flex: 1
    }} />
    <HintBtn text={HINTS.presence} />
  </div>;
}

/** `/api/v1/reboot` restarts the whole device, so this shows only when the module says it needs one. */
function RestartRow({
  busy,
  onRestart
}: {
  busy: boolean;
  onRestart: () => void;
}) {
  return <div className="pr-row pr-row-last">
    <div className="pr-row-label pr-row-stack">
      <span>Restart to apply</span>
      <span className="pr-row-sub">Some of these settings only take effect after the device restarts.</span>
    </div>
    <button type="button" className="pr-btn" disabled={busy} onClick={onRestart}>Restart device</button>
  </div>;
}
function Heading({
  model,
  children
}: {
  model?: string;
  children: ReactNode;
}) {
  return <>
    <span className="eyebrow">{model ? `PRESENCE · ${model}` : 'PRESENCE'}</span>
    <h1>{children}</h1>
  </>;
}
function Looking() {
  return <section className="control pr-tab">
    <Heading>The room, <em>mapped.</em></Heading>
    <div className="pr-card"><p className="pr-wait" role="status">{'Looking for a radar module\u2026'}</p></div>
  </section>;
}
function NoRadar({
  onGoDevice
}: {
  onGoDevice?: () => void;
}) {
  return <section className="control pr-tab">
    <Heading>The room, <em>mapped.</em></Heading>
    <div className="pr-card pr-empty">
      <RadarIcon size={40} className="pr-empty-icon" aria-hidden="true" />
      <h2 className="pr-empty-title">No Radar Detected</h2>
      <p className="pr-empty-body">
        <span>{TEXT.no_sensor_lead}</span>
        <a href={TEXT.no_sensor_docs_url} target="_blank" rel="noreferrer">{TEXT.no_sensor_docs}</a>
        <span>{TEXT.no_sensor_mid}</span>
        <a href={TEXT.no_sensor_buy_url} target="_blank" rel="noreferrer">{TEXT.no_sensor_buy}</a>
        <span>.</span>
      </p>
      <img className="pr-promo" src={`${BASE}/ui/no-sensor.webp`} alt="Satellite1 with hidden mmWave radar sensor" />
      {onGoDevice && <button type="button" className="pr-btn" style={{
        width: '100%',
        maxWidth: 280,
        minHeight: 44,
        background: 'transparent'
      }} onClick={onGoDevice}>Device settings</button>}
    </div>
  </section>;
}
function LD2450View({
  ctx,
  radar,
  isFt,
  setFt
}: ViewProps) {
  const {
    radarConfig: config,
    radarLive,
    radarBusy: busy,
    radarWrite: write,
    radarPreview,
    radarReboot
  } = radar;
  const [edit, setEdit] = useState<Edit>(null);
  const settings = useSettings(config, write, radarPreview);
  const trailsRef = useRef<Record<number, Pt[]>>({});
  const targets: Target[] = realTargets(radarLive);
  trailsRef.current = stepTrails(trailsRef.current, targets, TRAIL_LEN);
  const near = nearest(targets);
  const zones: Pt[][] = config.zones || [];
  const exclusion: Pt[] = config.exclusion || [];
  const defined = [0, 1, 2].filter(i => (zones[i] || []).length > 2);
  const hasExcl = exclusion.length > 2;
  const firstFree = [0, 1, 2].find(i => (zones[i] || []).length < 3);
  const isExcl = !!edit && edit.which === 'x';
  const zoneSlot = edit && typeof edit.from === 'number' ? edit.from : firstFree;
  const savable = edit && (edit.points.length === 0 || edit.points.length >= 3);
  const zoneBtns = defined.map(i => ({
    id: `zb${i}`,
    i
  }));
  const openShape = (which: number | 'x') => {
    const src: Pt[] = which === 'x' ? exclusion : zones[which] || [];
    setEdit({
      from: which,
      which,
      points: src.map(p => ({
        x: p.x,
        y: p.y
      })),
      sel: null,
      hist: []
    });
  };
  const save = async () => {
    if (!edit) return;
    await write(shapeBody(config, edit.from, edit.which, edit.points));
    setEdit(null);
  };
  const del = async () => {
    if (!edit || edit.from == null) return;
    await write(shapeBody(config, edit.from, edit.from, []));
    setEdit(null);
  };
  const instruction = edit ? edit.points.length === 0 ? TEXT.zi_first : edit.points.length < 3 ? TEXT.zi_more : edit.sel != null ? TEXT.zi_selected : TEXT.zi_adjust : defined.length || hasExcl ? TEXT.zones_set : TEXT.zones_none;
  const pills: Pill[] = [{
    l: 'Presence',
    v: presenceLabel(config, ctx.states || {})
  }, {
    l: 'People',
    v: String(targets.length)
  }, {
    l: 'Distance',
    v: near ? distLabel(near.d, isFt) : '\u2014',
    exp: true
  }, {
    l: 'Direction',
    v: near ? direction(near) : '\u2014'
  }];
  return <section className="control pr-tab">
    <Heading model="LD2450">The room, <em>mapped.</em></Heading>

    <StatusPills pills={pills} isFt={isFt} setFt={setFt} />

    <div className="pr-card">
      <CardHead title="LD2450" />

      <Plot zones={zones} exclusion={exclusion} range={num(config.detection_range)} targets={targets} trails={trailsRef.current} edit={edit} setEdit={setEdit} onOpen={openShape} isFt={isFt} />

      <p className="pr-instr">{instruction}</p>

      {edit ? <div className="pr-editor">
        <div className="pr-editor-row">
          <span className="pr-editor-name">{isExcl ? 'Exclusion' : `Zone ${(edit.which as number) + 1}`}</span>
          <span className="pr-corner-count">{edit.points.length}/{MAX_POINTS} corners</span>
        </div>
        <div className="pr-editor-row">
          <span className="pr-label">Exclusion zone</span>
          <HintBtn text={HINTS.zone_excl} />
          <span style={{
            flex: 1
          }} />
          <Switch on={isExcl} label="Exclusion zone" disabled={busy || isExcl && zoneSlot === undefined} onChange={v => setEdit(e => e && {
            ...e,
            which: v ? 'x' : zoneSlot as number
          })} />
        </div>
        <div className="pr-actions">
          <button type="button" className="pr-btn" disabled={edit.hist.length === 0} onClick={() => setEdit(e => !e || e.hist.length === 0 ? e : {
            ...e,
            points: e.hist[e.hist.length - 1],
            hist: e.hist.slice(0, -1),
            sel: null
          })}>Undo</button>
          {edit.sel != null && <button type="button" className="pr-btn" onClick={() => setEdit(e => e && {
            ...e,
            points: e.points.filter((_, i) => i !== e.sel),
            sel: null,
            hist: [...e.hist, e.points]
          })}>Remove corner</button>}
          <button type="button" className="pr-btn" onClick={() => setEdit(null)}>Cancel</button>
          <button type="button" className="pr-btn pr-btn-solid" disabled={!savable || busy} onClick={save}>Save</button>
          {edit.from != null && <button type="button" className="pr-btn pr-btn-danger" disabled={busy} onClick={del}>Delete</button>}
        </div>
      </div> : <div className="pr-actions">
        {zoneBtns.map(b => <button key={b.id} type="button" className="pr-btn" disabled={busy} onClick={() => openShape(b.i)}>{`Zone ${b.i + 1}`}</button>)}
        {hasExcl && <button type="button" className="pr-btn" disabled={busy} onClick={() => openShape('x')}>Exclusion</button>}
        {(firstFree !== undefined || !hasExcl) && <button type="button" className="pr-btn pr-btn-solid" disabled={busy} onClick={() => setEdit({
          from: null,
          which: firstFree ?? 'x',
          points: [],
          sel: null,
          hist: []
        })}>Add zone</button>}
        <HintBtn text={HINTS.radar_zones} />
      </div>}
    </div>

    <div className="pr-card">
      <div className="pr-card-head"><span className="pr-card-title">Settings</span></div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Detection range</span><HintBtn text={HINTS.radar_range} /></div>
        <MSlider value={num(config.detection_range)} min={0} max={600} step={10} ariaLabel="Detection range" format={v => rangeLabel(v, isFt)} onPreview={v => settings.preview('detection_range', v)} onCommit={v => settings.commit('detection_range', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Stability</span><HintBtn text={HINTS.radar_stability} /></div>
        <MSlider value={num(config.stability)} min={0} max={10} step={1} ariaLabel="Stability" format={v => `${v}`} onCommit={v => settings.commit('stability', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Timeout</span><HintBtn text={HINTS.radar_timeout} /></div>
        <MSlider value={num(config.timeout)} min={0} max={300} step={5} ariaLabel="Timeout" format={v => `${v} s`} onCommit={v => settings.commit('timeout', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Multi-target</span><HintBtn text={HINTS.radar_multi} /></div>
        <Switch on={!!config.multi_target} label="Multi-target" onChange={v => write({
          multi_target: v
        })} />
      </div>
      <div className={`pr-row${config.reboot_required ? '' : ' pr-row-last'}`}>
        <div className="pr-row-label"><span>Bluetooth</span><HintBtn text={HINTS.radar_bt} /></div>
        <Switch on={!!config.bluetooth} label="Bluetooth" onChange={v => write({
          bluetooth: v
        })} />
      </div>
      {config.reboot_required && <RestartRow busy={busy} onRestart={radarReboot} />}
    </div>
  </section>;
}
const GATE_ROWS = Array.from({
  length: 9
}, (_, i) => ({
  id: `g${i}`,
  i
}));
const GATE_KEYS: Record<string, number> = {
  ArrowLeft: -1,
  ArrowDown: -1,
  ArrowRight: 1,
  ArrowUp: 1,
  PageDown: -10,
  PageUp: 10
};

/**
 * One gate group: each row's bar is the live energy at that distance and its notch the trigger
 * threshold. A drag or key run edits a local draft; release commits all nine values, the only
 * shape the endpoint takes. Rows past the furthest gate still show energy but take no input.
 */
function GateGroup({
  kind,
  label,
  hint,
  levels,
  thresholds,
  maxGate,
  fine,
  isFt,
  busy,
  onCommit
}: {
  kind: 'move' | 'still';
  label: string;
  hint: string;
  levels: number[];
  thresholds: number[];
  maxGate: number;
  fine: boolean;
  isFt: boolean;
  busy: boolean;
  onCommit: (t: number[]) => void;
}) {
  const [draft, setDraft] = useState<number[] | null>(null);
  const [drag, setDrag] = useState<number | null>(null);
  // Release can arrive before the press has re-rendered, so commit reads the draft from here.
  const draftRef = useRef<number[] | null>(null);
  const vals = draft ?? thresholds;
  const valAt = (e: RPointerEvent<HTMLDivElement>) => {
    const r = e.currentTarget.getBoundingClientRect();
    return Math.round(Math.max(0, Math.min(100, (e.clientX - r.left) / r.width * 100)));
  };
  const set = (i: number, f: (v: number) => number) => {
    const next = (draftRef.current ?? thresholds).map((x, j) => j === i ? Math.max(0, Math.min(100, f(x))) : x);
    draftRef.current = next;
    setDraft(next);
  };
  const drop = () => {
    draftRef.current = null;
    setDraft(null);
    setDrag(null);
  };
  const commit = () => {
    const d = draftRef.current;
    if (d && d.some((v, j) => v !== thresholds[j])) onCommit(d);
    drop();
  };
  return <div className="pr-gates-g">
    <span className="pr-gates-label"><span>{label}</span><HintBtn text={hint} /></span>
    {GATE_ROWS.map(({
      id,
      i
    }) => {
      const off = i > maxGate;
      const band = gateLabel(i, fine, isFt);
      return <div key={id} className={`pr-gate${off ? ' off' : ''}`}>
        <span className="pr-gate-n">{band}</span>
        <div className="pr-gate-track" role="slider" aria-label={`${label} threshold, ${band}`} aria-valuemin={0} aria-valuemax={100} aria-valuenow={vals[i]} aria-disabled={off || busy} tabIndex={off ? -1 : 0} onPointerDown={e => {
          if (off || busy) return;
          e.currentTarget.setPointerCapture(e.pointerId);
          const v = valAt(e);
          setDrag(i);
          set(i, () => v);
        }} onPointerMove={e => {
          if (drag !== i) return;
          const v = valAt(e);
          set(i, () => v);
        }} onPointerUp={commit} onPointerCancel={drop} onKeyDown={e => {
          const by = GATE_KEYS[e.key];
          if (by === undefined || off || busy) return;
          e.preventDefault();
          set(i, x => x + by);
        }} onKeyUp={e => {
          if (GATE_KEYS[e.key] !== undefined) commit();
        }}>
          <div className="pr-gate-rail">
            <div className={`pr-gate-fill pr-gate-fill-${kind}`} style={{
              width: `${Math.min(100, num(levels[i]))}%`
            }} />
          </div>
          {!off && <span className={`pr-gate-notch${drag === i ? ' dragging' : ''}`} style={{
            left: `${vals[i]}%`
          }} />}
        </div>
      </div>;
    })}
  </div>;
}
function LD2410View({
  ctx,
  radar,
  isFt,
  setFt
}: ViewProps) {
  const {
    radarConfig: config,
    radarLive,
    radarBusy: busy,
    radarWrite: write,
    radarPreview,
    radarReboot
  } = radar;
  const settings = useSettings(config, write, radarPreview);
  const fine = config.distance_resolution === '0.2m';
  const states = ctx.states || {};
  const target = states['text_sensor/Radar Target'];
  const reading = states['sensor/Radar Detection Distance'] || states['sensor/Radar Moving Distance'];
  const cm = reading ? Number(reading.value) : NaN;
  const gates = radarLive && radarLive.gates || {};
  const pills: Pill[] = [{
    l: 'Presence',
    v: target && target.value ? PRESENCE[target.value] || target.value : '\u2014'
  }, {
    l: 'Distance',
    v: Number.isFinite(cm) && cm > 0 ? distLabel(cm, isFt) : '\u2014',
    exp: true
  }];
  return <section className="control pr-tab">
    <Heading model="LD2410">The gate, <em>tuned.</em></Heading>
    <StatusPills pills={pills} isFt={isFt} setFt={setFt} />
    <div className="pr-card">
      <CardHead title="LD2410" />
      <div className="pr-gates">
        <GateGroup kind="move" label="Movement" hint={HINTS.gate_move} levels={gates.move || []} thresholds={gateArray(config.gate_move_thresholds)} maxGate={num(config.max_move_gate)} fine={fine} isFt={isFt} busy={busy} onCommit={t => write({
          gate_move_thresholds: t
        })} />
        <GateGroup kind="still" label="Stillness" hint={HINTS.gate_still} levels={gates.still || []} thresholds={gateArray(config.gate_still_thresholds)} maxGate={num(config.max_still_gate)} fine={fine} isFt={isFt} busy={busy} onCommit={t => write({
          gate_still_thresholds: t
        })} />
      </div>
      <p className="pr-gates-hint">{TEXT.gate_thresholds_help}</p>
    </div>
    <div className="pr-card">
      <div className="pr-card-head"><span className="pr-card-title">Settings</span></div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Timeout</span><HintBtn text={HINTS.radar_timeout} /></div>
        <MSlider value={num(config.timeout)} min={0} max={300} step={5} ariaLabel="Timeout" format={v => `${v} s`} onCommit={v => settings.commit('timeout', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Furthest movement gate</span><HintBtn text={HINTS.gate_max_move} /></div>
        <MSlider value={num(config.max_move_gate)} min={0} max={8} step={1} ariaLabel="Furthest movement gate" format={v => `${v}`} onPreview={v => settings.preview('max_move_gate', v)} onCommit={v => settings.commit('max_move_gate', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Furthest stillness gate</span><HintBtn text={HINTS.gate_max_still} /></div>
        <MSlider value={num(config.max_still_gate)} min={0} max={8} step={1} ariaLabel="Furthest stillness gate" format={v => `${v}`} onPreview={v => settings.preview('max_still_gate', v)} onCommit={v => settings.commit('max_still_gate', v)} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Distance resolution</span><HintBtn text={HINTS.radar_resolution} /></div>
        <MSlider value={fine ? 0 : 1} min={0} max={1} step={1} ariaLabel="Distance resolution" format={v => v === 0 ? isFt ? '0.7 ft' : '0.2 m' : isFt ? '2.5 ft' : '0.75 m'} onPreview={v => settings.preview('distance_resolution', v === 0 ? '0.2m' : '0.75m')} onCommit={v => settings.commit('distance_resolution', v === 0 ? '0.2m' : '0.75m')} />
      </div>
      <div className={`pr-row${config.reboot_required ? '' : ' pr-row-last'}`}>
        <div className="pr-row-label"><span>Bluetooth</span><HintBtn text={HINTS.radar_bt} /></div>
        <Switch on={!!config.bluetooth} label="Bluetooth" onChange={v => write({
          bluetooth: v
        })} />
      </div>
      {config.reboot_required && <RestartRow busy={busy} onRestart={radarReboot} />}
    </div>
  </section>;
}
export function PresenceTab({
  ctx,
  onGoDevice
}: {
  ctx: Ctx;
  onGoDevice?: () => void;
}) {
  const radar = useRadar(true);
  const unit = entity(ctx, 'distance_unit_ft');
  const isFt = feetOn(unit);
  const setFt = unit ? (v: boolean) => {
    post(pathFor(ctx, 'distance_unit_ft', v ? 'turn_on' : 'turn_off')).catch(() => {});
  } : null;
  if (radar.radarKind === 'none') return <NoRadar onGoDevice={onGoDevice} />;
  if (!radar.radarKind || !radar.radarConfig) return <Looking />;
  if (radar.radarKind === 'ld2410') return <LD2410View ctx={ctx} radar={radar} isFt={isFt} setFt={setFt} />;
  return <LD2450View ctx={ctx} radar={radar} isFt={isFt} setFt={setFt} />;
}
