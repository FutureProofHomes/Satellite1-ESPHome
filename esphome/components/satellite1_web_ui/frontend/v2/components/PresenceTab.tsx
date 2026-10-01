import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import type { Dispatch, PointerEvent as RPointerEvent, ReactNode, SetStateAction } from 'react';
import { createPortal } from 'react-dom';
import { BASE } from '../../src/lib/device.js';
import { RadarIcon } from '../icons';
import { MSlider } from './MSlider';
import type { Ctx } from '../ctx';
const HINTS = {
  presence: 'Presence is sensed by mmWave radar, not heard — the microphones play no part in it.',
  distance_unit: 'Display only. The module keeps measuring in centimetres either way.',
  radar_range: 'Targets beyond this ring are ignored. Zero is the module\u2019s full 6 m reach.',
  radar_stability: 'How much a reading is smoothed before it counts. Higher holds still rooms steadier.',
  radar_timeout: 'How long presence holds after the last detection before the room reads clear.',
  radar_multi: 'Track up to three people at once instead of the strongest target only.',
  radar_bt: 'The module\u2019s own Bluetooth radio, used by the vendor app. Off is the right answer once this page exists.',
  radar_zones: 'Zones turn coordinates into rooms: occupancy publishes per zone. The exclusion area silences a fan or curtain.',
  zone_excl: 'An exclusion area is ignored outright — a fan, a curtain, a pet bed. The zone slots stay free.',
  gate_move: 'How much movement the radar sees at each distance, live. Drag a notch to set the trigger level for that distance — anything above it counts as a person moving.',
  gate_still: 'How much tiny motion — like breathing — the radar sees at each distance, live. Drag a notch to set the trigger level — anything above it counts as a person holding still.',
  gate_max_move: 'The farthest distance that counts for movement. Rows past it dim in the chart above and are ignored.',
  gate_max_still: 'The farthest distance that counts for stillness. Rows past it dim in the chart above and are ignored.',
  radar_resolution: 'How finely distance is split into the nine rows above: 0.75m steps reach the whole room, 0.2m steps reach less far but with more detail up close. Changing it changes what each row means, so re-check your levels after.'
};
const TEXT = {
  zi_first: 'Tap the map to place the first corner.',
  zi_more: 'Keep tapping — a zone needs at least three corners.',
  zi_selected: 'Drag the corner to move it, or remove it below.',
  zi_adjust: 'Drag corners to adjust, tap one to select it — or drag the middle to move the whole shape.',
  zones_set: 'Tap a zone on the map or below to edit it.',
  zones_none: 'No zones yet — presence publishes for the whole field of view.'
};
const PLOT_HALF_W = 400;
const PLOT_DEPTH = 640;
const FOV_X = Math.sin(60 * Math.PI / 180) * PLOT_DEPTH;
const FOV_Y = Math.cos(60 * Math.PI / 180) * PLOT_DEPTH;
const MAX_POINTS = 8;
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
type Edit = {
  from: number | 'x' | null;
  which: number | 'x';
  points: Pt[];
  sel: number | null;
  hist: Pt[][];
} | null;
const polygonPoints = (pts: Pt[]) => pts.map(p => `${p.x},${p.y}`).join(' ');
const labelPoint = (pts: Pt[]) => ({
  x: pts.reduce((a, p) => a + p.x, 0) / pts.length,
  y: pts.reduce((a, p) => a + p.y, 0) / pts.length
});
const inPolygon = (p: Pt, poly: Pt[]) => {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a.y > p.y !== b.y > p.y && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
};
function useWanderingTarget() {
  const [t, setT] = useState({
    x: -40,
    y: 210
  });
  useEffect(() => {
    let k = 0;
    const timer = setInterval(() => {
      k += 0.25;
      setT({
        x: Math.round(Math.sin(k / 3.1) * 150 - 30),
        y: Math.round(250 + Math.sin(k / 4.7) * 110 + Math.cos(k / 2.3) * 30)
      });
    }, 250);
    return () => clearInterval(timer);
  }, []);
  return t;
}
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
  onChange
}: {
  on: boolean;
  onChange: (v: boolean) => void;
}) {
  return <button role="switch" aria-checked={on} onClick={() => onChange(!on)} className={`pr-switch${on ? ' on' : ''}`}>
      <span className="pr-switch-thumb" />
    </button>;
}
function Plot({
  zones,
  exclusion,
  range,
  target,
  trail,
  edit,
  setEdit,
  onOpen,
  isFt
}: {
  zones: Pt[][];
  exclusion: Pt[];
  range: number;
  target: Pt;
  trail: Pt[];
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
      let dx = p.x - lastRef.current.x;
      let dy = p.y - lastRef.current.y;
      const xs = edit.points.map(q => q.x);
      const ys = edit.points.map(q => q.y);
      dx = Math.max(-PLOT_HALF_W - Math.min(...xs), Math.min(PLOT_HALF_W - Math.max(...xs), dx));
      dy = Math.max(-Math.min(...ys), Math.min(PLOT_DEPTH - Math.max(...ys), dy));
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
      {trail.slice(1).map((p, i) => {
        const age = i + 1;
        const opacity = 1 - age / 6.5;
        const r = Math.max(4, 14 - age * 1.8);
        return <g key={`tr${age}`} style={{
          transform: `translate(${p.x}px,${p.y}px)`,
          transition: 'transform 0.25s linear'
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
        transform: `translate(${target.x}px,${target.y}px)`,
        transition: 'transform 0.25s linear'
      }}>
        <circle className="pr-tgt-halo" cx="0" cy="0" r="42" fill="rgba(96,165,250,0.10)" />
        <circle cx="0" cy="0" r="26" fill="rgba(96,165,250,0.22)" filter="url(#pr-glowf)" />
        <circle cx="0" cy="0" r="11" fill="#60a5fa" style={{
          filter: 'drop-shadow(0 0 8px rgba(96,165,250,0.9))'
        }} />
        <circle cx="-3" cy="-3" r="3.5" fill="rgba(255,255,255,0.7)" />
      </g>
      {RINGS.map(r => <text key={r.id} className="pr-tick" x="14" y={Math.min(r.r + 26, PLOT_DEPTH - 10)}>
        {isFt ? `${Math.round(r.r / 30.48 * 10) / 10} ft` : `${r.r / 100}m`}
      </text>)}
    </svg>
  </div>;
}
function StatusPills({
  target,
  isFt,
  setIsFt
}: {
  target: Pt;
  isFt: boolean;
  setIsFt: (v: boolean) => void;
}) {
  const [open, setOpen] = useState(false);
  const d = Math.hypot(target.x, target.y);
  const ang = Math.atan2(target.x, target.y) * 180 / Math.PI;
  const dist = isFt ? `${(d / 30.48).toFixed(1)} ft` : `${(d / 100).toFixed(1)} m`;
  return <div className="pr-card">
    <div className="pr-pills">
      <div className="pr-pill"><span className="pr-pill-v">Zone 1</span><span className="pr-pill-l">Presence</span></div>
      <div className="pr-pill"><span className="pr-pill-v">1</span><span className="pr-pill-l">People</span></div>
      <button type="button" className={`pr-pill${open ? ' open' : ''}`} style={{
        cursor: 'pointer'
      }} aria-expanded={open} onClick={() => setOpen(o => !o)}>
        <span className="pr-pill-v">{dist}</span><span className="pr-pill-l">Distance ›</span>
      </button>
      <div className="pr-pill"><span className="pr-pill-v">{ang < -15 ? 'Left' : ang > 15 ? 'Right' : 'Ahead'}</span><span className="pr-pill-l">Direction</span></div>
    </div>
    {open && <div className="pr-pill-expand">
      <span>Feet</span>
      <HintBtn text={HINTS.distance_unit} />
      <span style={{
        flex: 1
      }} />
      <Switch on={isFt} onChange={setIsFt} />
    </div>}
  </div>;
}
export function PresenceTab({
  ctx,
  onGoDevice
}: {
  ctx: Ctx;
  onGoDevice?: () => void;
}) {
  void onGoDevice;
  const [radarConnected, setRadarConnected] = useState(false);
  const [radarModel, setRadarModel] = useState<'LD2450' | 'LD2410'>('LD2450');
  const target = useWanderingTarget();
  const [isFt, setIsFt] = useState(false);
  const [cfg10, setCfg10] = useState<Ld2410Cfg>({
    timeout: 30,
    max_move_gate: 6,
    max_still_gate: 6,
    bluetooth: false,
    res02: false,
    gate_move_thresholds: [50, 50, 40, 30, 20, 15, 15, 15, 15],
    gate_still_thresholds: [0, 0, 40, 40, 30, 30, 20, 20, 20]
  });
  const live10 = useLd2410Live(cfg10.res02 ? 20 : 75);
  const [zones, setZones] = useState<Pt[][]>([[{
    x: -260,
    y: 90
  }, {
    x: 40,
    y: 90
  }, {
    x: 40,
    y: 330
  }, {
    x: -260,
    y: 330
  }], [], []]);
  const [exclusion, setExclusion] = useState<Pt[]>([]);
  const [edit, setEdit] = useState<Edit>(null);
  const [cfg, setCfg] = useState({
    detection_range: 450,
    stability: 3,
    timeout: 30,
    multi_target: true,
    bluetooth: false
  });
  const [preview, setPreview] = useState<number | null>(null);
  const [trail, setTrail] = useState<Pt[]>([]);
  useEffect(() => {
    setTrail(prev => {
      if (prev[0]?.x === target.x && prev[0]?.y === target.y) return prev;
      return [target, ...prev].slice(0, 6);
    });
  }, [target.x, target.y]);
  const defined = [0, 1, 2].filter(i => (zones[i] || []).length > 2);
  const hasExcl = exclusion.length > 2;
  const firstFree = [0, 1, 2].find(i => (zones[i] || []).length < 3);
  const isExcl = edit && edit.which === 'x';
  const zoneSlot = edit && typeof edit.from === 'number' ? edit.from : firstFree;
  const savable = edit && (edit.points.length === 0 || edit.points.length >= 3);
  const zoneBtns = defined.map(i => ({
    id: `zb${i}`,
    i
  }));
  const openShape = (which: number | 'x') => {
    const src = which === 'x' ? exclusion : zones[which] || [];
    setEdit({
      from: which,
      which,
      points: src.map(p => ({
        ...p
      })),
      sel: null,
      hist: []
    });
  };
  const save = () => {
    if (!edit) return;
    const zs = zones.map(z => z.map(p => ({
      ...p
    })));
    let ex = exclusion.map(p => ({
      ...p
    }));
    if (edit.from === 'x') ex = [];else if (typeof edit.from === 'number') zs[edit.from] = [];
    if (edit.which === 'x') ex = edit.points;else zs[edit.which as number] = edit.points;
    setZones(zs);
    setExclusion(ex);
    setEdit(null);
  };
  const del = () => {
    if (!edit || edit.from == null) return;
    const zs = zones.map(z => z.map(p => ({
      ...p
    })));
    let ex = exclusion.map(p => ({
      ...p
    }));
    if (edit.from === 'x') ex = [];else zs[edit.from] = [];
    setZones(zs);
    setExclusion(ex);
    setEdit(null);
  };
  const instruction = edit ? edit.points.length === 0 ? TEXT.zi_first : edit.points.length < 3 ? TEXT.zi_more : edit.sel != null ? TEXT.zi_selected : TEXT.zi_adjust : defined.length || hasExcl ? TEXT.zones_set : TEXT.zones_none;
  if (!radarConnected) return <section className="control pr-tab">
    <span className="eyebrow">PRESENCE</span>
    <h1>The room, <em>mapped.</em></h1>
    <div className="pr-card pr-empty">
      <RadarIcon size={40} className="pr-empty-icon" aria-hidden="true" />
      <h2 className="pr-empty-title">No Radar Detected</h2>
      <p className="pr-empty-body">
        <span>A presence sensor was not detected in your Sat1.{' '}</span>
        <a href="https://docs.futureproofhomes.net/satellite1-presence-sensors/#connecting-mmwave-sensors" target="_blank" rel="noreferrer">Read our docs to learn more</a>
        <span>, or purchase a presence sensor{' '}</span>
        <a href="https://futureproofhomes.net/products/ld2450-mmwave-human-presence-sensor" target="_blank" rel="noreferrer">here</a>
        <span>.</span>
      </p>
      <img className="pr-promo" src={`${BASE}/ui/no-sensor.webp`} alt="Satellite1 with hidden mmWave radar sensor" />
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        gap: 8,
        width: '100%',
        maxWidth: 280
      }}>
        <button type="button" className="pr-btn pr-btn-solid" style={{
          width: '100%',
          minHeight: 44
        }} onClick={() => {
          setRadarModel('LD2450');
          setRadarConnected(true);
        }}>Simulate LD2450</button>
        <button type="button" className="pr-btn" style={{
          width: '100%',
          minHeight: 44,
          background: 'transparent'
        }} onClick={() => {
          setRadarModel('LD2410');
          setRadarConnected(true);
        }}>Simulate LD2410</button>
      </div>
    </div>
  </section>;
  if (radarModel === 'LD2410') return <LD2410View cfg={cfg10} setCfg={setCfg10} live={live10} isFt={isFt} setIsFt={setIsFt} onDisconnect={() => setRadarConnected(false)} />;
  return <section className="control pr-tab">
    <span className="eyebrow">PRESENCE · LD2450</span>
    <h1>The room, <em>mapped.</em></h1>

    <StatusPills target={target} isFt={isFt} setIsFt={setIsFt} />

    <div className="pr-card">
      <div className="pr-card-head">
        <span className="pr-card-title">LD2450</span>
        <span className="pr-live-badge">● live</span>
        <span style={{
          flex: 1
        }} />
        <HintBtn text={HINTS.presence} />
      </div>

      <Plot zones={zones} exclusion={exclusion} range={preview ?? cfg.detection_range} target={target} trail={trail} edit={edit} setEdit={setEdit} onOpen={openShape} isFt={isFt} />

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
          <Switch on={!!isExcl} onChange={v => setEdit(e => e && {
            ...e,
            which: v ? 'x' : zoneSlot ?? 0
          })} />
        </div>
        <div className="pr-actions">
          <button className="pr-btn" disabled={!edit || edit.hist.length === 0} onClick={() => setEdit(e => !e || e.hist.length === 0 ? e : {
            ...e,
            points: e.hist[e.hist.length - 1],
            hist: e.hist.slice(0, -1),
            sel: null
          })}>Undo</button>
          {edit.sel != null && <button className="pr-btn" onClick={() => setEdit(e => e && {
            ...e,
            points: e.points.filter((_, i) => i !== e.sel),
            sel: null,
            hist: [...e.hist, e.points]
          })}>Remove corner</button>}
          <button className="pr-btn" onClick={() => setEdit(null)}>Cancel</button>
          <button className="pr-btn pr-btn-solid" disabled={!savable} onClick={save}>Save</button>
          {edit.from != null && <button className="pr-btn pr-btn-danger" onClick={del}>Delete</button>}
        </div>
      </div> : <div className="pr-actions">
        {zoneBtns.map(b => <button key={b.id} className="pr-btn" onClick={() => openShape(b.i)}>{`Zone ${b.i + 1}`}</button>)}
        {hasExcl && <button className="pr-btn" onClick={() => openShape('x')}>Exclusion</button>}
        {(firstFree !== undefined || !hasExcl) && <button className="pr-btn pr-btn-solid" onClick={() => {
          if (firstFree !== undefined) setEdit({
            from: null,
            which: firstFree,
            points: [],
            sel: null,
            hist: []
          });else setEdit({
            from: null,
            which: 'x',
            points: [],
            sel: null,
            hist: []
          });
        }}>Add zone</button>}
        <HintBtn text={HINTS.radar_zones} />
      </div>}
    </div>

    <div className="pr-card">
      <div className="pr-card-head"><span className="pr-card-title">Settings</span></div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Detection range</span><HintBtn text={HINTS.radar_range} /></div>
        <MSlider value={cfg.detection_range} min={0} max={600} step={10} ariaLabel="Detection range" format={v => v === 0 ? isFt ? '20 ft' : '6 m' : isFt ? `${(v / 30.48).toFixed(1)} ft` : `${v} cm`} onPreview={v => setPreview(v)} onCommit={v => {
          setPreview(null);
          setCfg(c => ({
            ...c,
            detection_range: v
          }));
        }} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Stability</span><HintBtn text={HINTS.radar_stability} /></div>
        <MSlider value={cfg.stability} min={0} max={10} step={1} ariaLabel="Stability" format={v => `${v}`} onPreview={v => setCfg(c => ({
          ...c,
          stability: v
        }))} onCommit={v => setCfg(c => ({
          ...c,
          stability: v
        }))} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Timeout</span><HintBtn text={HINTS.radar_timeout} /></div>
        <MSlider value={cfg.timeout} min={0} max={300} step={5} ariaLabel="Timeout" format={v => `${v} s`} onPreview={v => setCfg(c => ({
          ...c,
          timeout: v
        }))} onCommit={v => setCfg(c => ({
          ...c,
          timeout: v
        }))} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Multi-target</span><HintBtn text={HINTS.radar_multi} /></div>
        <Switch on={cfg.multi_target} onChange={v => setCfg(c => ({
          ...c,
          multi_target: v
        }))} />
      </div>
      <div className="pr-row pr-row-last">
        <div className="pr-row-label"><span>Bluetooth</span><HintBtn text={HINTS.radar_bt} /></div>
        <Switch on={cfg.bluetooth} onChange={v => setCfg(c => ({
          ...c,
          bluetooth: v
        }))} />
      </div>
    </div>
    <button type="button" className="text-button" style={{
      alignSelf: 'center'
    }} onClick={() => setRadarConnected(false)}>Disconnect radar</button>
  </section>;
}
type Ld2410Cfg = {
  timeout: number;
  max_move_gate: number;
  max_still_gate: number;
  bluetooth: boolean;
  res02: boolean;
  gate_move_thresholds: number[];
  gate_still_thresholds: number[];
};
type Ld2410Live = {
  move: number[];
  still: number[];
  state: string;
  dist: number;
};
const GATE_ROWS = Array.from({
  length: 9
}, (_, i) => ({
  id: `g${i}`,
  i
}));
function useLd2410Live(resCm: number) {
  const [live, setLive] = useState<Ld2410Live>({
    move: Array(9).fill(0) as number[],
    still: Array(9).fill(0) as number[],
    state: 'Still',
    dist: 210
  });
  useEffect(() => {
    let k = 0;
    const timer = setInterval(() => {
      k += 0.25;
      const dist = Math.round(260 + Math.sin(k / 4.7) * 130 + Math.cos(k / 2.3) * 40);
      const gi = Math.max(0, Math.min(8, Math.floor(dist / resCm)));
      const moving = Math.sin(k / 3.4) > -0.35;
      const gates = (peak: number, jitter: number) => Array.from({
        length: 9
      }, (_, i) => {
        const spread = Math.exp(-Math.abs(i - gi) * 1.1);
        return Math.min(100, Math.round(3 + Math.random() * 6 + (peak + Math.random() * jitter) * spread));
      });
      setLive({
        move: moving ? gates(55, 30) : gates(8, 8),
        still: moving ? gates(18, 12) : gates(45, 25),
        state: moving ? 'Moving' : 'Still',
        dist
      });
    }, 250);
    return () => clearInterval(timer);
  }, [resCm]);
  return live;
}
function GateGroup({
  kind,
  label,
  hint,
  levels,
  thresholds,
  maxGate,
  resCm,
  isFt,
  onCommit
}: {
  kind: 'move' | 'still';
  label: string;
  hint: string;
  levels: number[];
  thresholds: number[];
  maxGate: number;
  resCm: number;
  isFt: boolean;
  onCommit: (t: number[]) => void;
}) {
  const [draft, setDraft] = useState<number[] | null>(null);
  const [drag, setDrag] = useState<number | null>(null);
  const vals = draft ?? thresholds;
  const fmt = (cm: number) => isFt ? `${Math.round(cm / 30.48 * 10) / 10}` : `${Math.round(cm) / 100}`;
  const valAt = (e: RPointerEvent<HTMLDivElement>) => {
    const r = e.currentTarget.getBoundingClientRect();
    return Math.round(Math.max(0, Math.min(100, (e.clientX - r.left) / r.width * 100)));
  };
  const set = (i: number, v: number) => setDraft(d => (d ?? thresholds).map((x, j) => j === i ? v : x));
  const commit = () => {
    if (draft) onCommit(draft);
    setDraft(null);
    setDrag(null);
  };
  return <div className="pr-gates-g">
    <span className="pr-gates-label"><span>{label}</span><HintBtn text={hint} /></span>
    {GATE_ROWS.map(({
      id,
      i
    }) => {
      const off = i > maxGate;
      return <div key={id} className={`pr-gate${off ? ' off' : ''}`}>
        <span className="pr-gate-n">{`${fmt(i * resCm)}–${fmt((i + 1) * resCm)}${isFt ? 'ft' : 'm'}`}</span>
        <div className="pr-gate-track" role="slider" aria-label={`${label} threshold, row ${i + 1}`} aria-valuemin={0} aria-valuemax={100} aria-valuenow={vals[i]} tabIndex={off ? -1 : 0} onPointerDown={e => {
          e.currentTarget.setPointerCapture(e.pointerId);
          setDrag(i);
          set(i, valAt(e));
        }} onPointerMove={e => {
          if (drag === i) set(i, valAt(e));
        }} onPointerUp={commit} onPointerCancel={commit} onKeyDown={e => {
          if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
          e.preventDefault();
          const v = Math.max(0, Math.min(100, vals[i] + (e.key === 'ArrowRight' ? 1 : -1)));
          onCommit(thresholds.map((x, j) => j === i ? v : x));
        }}>
          <div className="pr-gate-rail">
            <div className={`pr-gate-fill pr-gate-fill-${kind}`} style={{
              width: `${levels[i]}%`
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
  cfg,
  setCfg,
  live,
  isFt,
  setIsFt,
  onDisconnect
}: {
  cfg: Ld2410Cfg;
  setCfg: Dispatch<SetStateAction<Ld2410Cfg>>;
  live: Ld2410Live;
  isFt: boolean;
  setIsFt: (v: boolean) => void;
  onDisconnect: () => void;
}) {
  const [distOpen, setDistOpen] = useState(false);
  const resCm = cfg.res02 ? 20 : 75;
  const dist = isFt ? `${(live.dist / 30.48).toFixed(1)} ft` : `${(live.dist / 100).toFixed(1)} m`;
  return <section className="control pr-tab">
    <span className="eyebrow">PRESENCE · LD2410</span>
    <h1>The gate, <em>tuned.</em></h1>
    <div className="pr-card">
      <div className="pr-pills">
        <div className="pr-pill"><span className="pr-pill-v">{live.state}</span><span className="pr-pill-l">Presence</span></div>
        <button type="button" className={`pr-pill${distOpen ? ' open' : ''}`} style={{
          cursor: 'pointer',
          minHeight: 44
        }} aria-expanded={distOpen} onClick={() => setDistOpen(o => !o)}>
          <span className="pr-pill-v">{dist}</span><span className="pr-pill-l">Distance ›</span>
        </button>
      </div>
      {distOpen && <div className="pr-pill-expand">
        <span>Feet</span>
        <HintBtn text={HINTS.distance_unit} />
        <span style={{
          flex: 1
        }} />
        <Switch on={isFt} onChange={setIsFt} />
      </div>}
    </div>
    <div className="pr-card">
      <div className="pr-card-head">
        <span className="pr-card-title">LD2410</span>
        <span className="pr-live-badge">● live</span>
        <span style={{
          flex: 1
        }} />
        <HintBtn text={HINTS.presence} />
      </div>
      <div className="pr-gates">
        <GateGroup kind="move" label="Movement" hint={HINTS.gate_move} levels={live.move} thresholds={cfg.gate_move_thresholds} maxGate={cfg.max_move_gate} resCm={resCm} isFt={isFt} onCommit={t => setCfg(c => ({
          ...c,
          gate_move_thresholds: t
        }))} />
        <GateGroup kind="still" label="Stillness" hint={HINTS.gate_still} levels={live.still} thresholds={cfg.gate_still_thresholds} maxGate={cfg.max_still_gate} resCm={resCm} isFt={isFt} onCommit={t => setCfg(c => ({
          ...c,
          gate_still_thresholds: t
        }))} />
      </div>
      <p className="pr-gates-hint">Each row is a band of distance. The bar shows what the radar sees there right now; drag the notch to set where it triggers. Dimmed rows are out of range and ignored. Changes save automatically.</p>
    </div>
    <div className="pr-card">
      <div className="pr-card-head"><span className="pr-card-title">Settings</span></div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Timeout</span><HintBtn text={HINTS.radar_timeout} /></div>
        <MSlider value={cfg.timeout} min={0} max={300} step={5} ariaLabel="Timeout" format={v => `${v} s`} onPreview={v => setCfg(c => ({
          ...c,
          timeout: v
        }))} onCommit={v => setCfg(c => ({
          ...c,
          timeout: v
        }))} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Furthest movement gate</span><HintBtn text={HINTS.gate_max_move} /></div>
        <MSlider value={cfg.max_move_gate} min={0} max={8} step={1} ariaLabel="Furthest movement gate" format={v => `${v}`} onPreview={v => setCfg(c => ({
          ...c,
          max_move_gate: v
        }))} onCommit={v => setCfg(c => ({
          ...c,
          max_move_gate: v
        }))} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Furthest stillness gate</span><HintBtn text={HINTS.gate_max_still} /></div>
        <MSlider value={cfg.max_still_gate} min={0} max={8} step={1} ariaLabel="Furthest stillness gate" format={v => `${v}`} onPreview={v => setCfg(c => ({
          ...c,
          max_still_gate: v
        }))} onCommit={v => setCfg(c => ({
          ...c,
          max_still_gate: v
        }))} />
      </div>
      <div className="pr-row">
        <div className="pr-row-label"><span>Distance resolution</span><HintBtn text={HINTS.radar_resolution} /></div>
        <MSlider value={cfg.res02 ? 0 : 1} min={0} max={1} step={1} ariaLabel="Distance resolution" format={v => v === 0 ? isFt ? '0.7 ft' : '0.2 m' : isFt ? '2.5 ft' : '0.75 m'} onPreview={v => setCfg(c => ({
          ...c,
          res02: v === 0
        }))} onCommit={v => setCfg(c => ({
          ...c,
          res02: v === 0
        }))} />
      </div>
      <div className="pr-row pr-row-last">
        <div className="pr-row-label"><span>Bluetooth</span><HintBtn text={HINTS.radar_bt} /></div>
        <Switch on={cfg.bluetooth} onChange={v => setCfg(c => ({
          ...c,
          bluetooth: v
        }))} />
      </div>
    </div>
    <button type="button" className="text-button" style={{
      alignSelf: 'center'
    }} onClick={onDisconnect}>Disconnect radar</button>
  </section>;
}