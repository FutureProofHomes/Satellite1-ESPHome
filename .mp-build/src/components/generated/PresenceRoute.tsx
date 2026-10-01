/**
 * The Presence route, all three faces of it: the LD2450 plot with zones and live targets
 * (zone drawing/editing, undo per gesture, delete), the LD2410 gate tuner (nine bars of live
 * movement/stillness energy with draggable thresholds), and the no-module-fitted card. The
 * real firmware picks a face by detecting the hardware; the mockup adds a preview-only
 * switcher at the top instead. Ported from routes/presence.jsx.
 */
import React, { useEffect, useRef, useState } from 'react';
import { Btn, Card, Empty, Hint, N_PRES, Pill, Row, Slider, Toggle } from './ui';
const HINTS = {
  presence: 'Presence is sensed by mmWave radar, not heard - the microphones play no part in it.',
  distance_unit: 'Display only. The module keeps measuring in centimetres either way.',
  radar_range: 'Targets beyond this ring are ignored. Zero is the module\u2019s full 6 m reach.',
  radar_stability: 'How much a reading is smoothed before it counts. Higher holds still rooms steadier.',
  radar_timeout: 'How long presence holds after the last detection before the room reads clear.',
  radar_multi: 'Track up to three people at once instead of the strongest target only.',
  radar_bt: 'The module\u2019s own Bluetooth radio, used by the vendor app. Off is the right answer once this page exists.',
  radar_zones: 'Zones turn coordinates into rooms: occupancy publishes per zone. The exclusion area silences a fan or curtain.',
  zone_excl: 'An exclusion area is ignored outright - a fan, a curtain, a pet bed. The zone slots stay free.',
  radar_resolution: 'How finely distance is split into the nine rows above: 0.75m steps reach the whole room, 0.2m steps reach less far but with more detail up close. Changing it changes what each row means, so re-check your levels after.',
  gate_move: 'How much movement the radar sees at each distance, live. Drag a notch to set the trigger level for that distance - anything above it counts as a person moving.',
  gate_still: 'How much tiny motion - like breathing - the radar sees at each distance, live. Drag a notch to set the trigger level - anything above it counts as a person holding still.',
  gate_max_move: 'The farthest distance that counts for movement. Rows past it dim in the chart above and are ignored.',
  gate_max_still: 'The farthest distance that counts for stillness. Rows past it dim in the chart above and are ignored.'
};
const TEXT = {
  zi_first: 'Tap the map to place the first corner.',
  zi_more: 'Keep tapping - a zone needs at least three corners.',
  zi_selected: 'Drag the corner to move it, or remove it below.',
  zi_adjust: 'Drag corners to adjust, tap one to select it - or drag the middle to move the whole shape.',
  zones_set: 'Tap a zone on the map or below to edit it.',
  zones_none: 'No zones yet - presence publishes for the whole field of view.',
  gate_thresholds_help: 'Each row is a band of distance. The bar shows what the radar sees there right now; drag the notch to set where it triggers. Dimmed rows are out of range and ignored. Changes save automatically.',
  no_sensor_lead: 'A presence sensor was not detected in your Sat1. Please ',
  no_sensor_docs: 'read our docs to learn more',
  no_sensor_docs_url: 'https://docs.futureproofhomes.net/satellite1-presence-sensors/#connecting-mmwave-sensors',
  no_sensor_mid: ', you can purchase a presence sensor ',
  no_sensor_buy: 'here',
  no_sensor_buy_url: 'https://futureproofhomes.net/products/ld2450-mmwave-human-presence-sensor'
};
const PLOT_HALF_W = 400;
const PLOT_DEPTH = 640;
const FOV_X = Math.sin(60 * Math.PI / 180) * PLOT_DEPTH;
const FOV_Y = Math.cos(60 * Math.PI / 180) * PLOT_DEPTH;
const MAX_POINTS = 8;
type Pt = {
  x: number;
  y: number;
};
const polygonPoints = (pts: Pt[]) => pts.map(p => `${p.x},${p.y}`).join(' ');
const labelPoint = (pts: Pt[]) => ({
  x: pts.reduce((a, p) => a + p.x, 0) / pts.length,
  y: pts.reduce((a, p) => a + p.y, 0) / pts.length
});

/** Ray-cast point-in-polygon, for opening a committed shape with a tap on the map. */
const inPolygon = (p: Pt, poly: Pt[]) => {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a.y > p.y !== b.y > p.y && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
};

/** A live target wandering the room on a slow lissajous path, so the plot breathes like the real one. */
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

/**
 * The LD2410 mock: nine gates of movement and stillness energy, refreshed at the real 250ms poll
 * rate. A simulated person drifts through the room; the gate they occupy lights up, neighbours get
 * spill-over, and every gate carries a little noise floor - so the chart breathes like the real one.
 * The person alternates between walking (movement energy high) and sitting (stillness energy high),
 * which also drives the Presence pill's Moving/Still.
 */
function useLd2410Live(resCm: number) {
  const [live, setLive] = useState({
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
type Edit = {
  from: number | 'x' | null;
  which: number | 'x';
  points: Pt[];
  sel: number | null;
  hist: Pt[][];
} | null;
function Plot({
  zones,
  exclusion,
  range,
  target,
  edit,
  setEdit,
  onOpen,
  isFt
}: {
  zones: Pt[][];
  exclusion: Pt[];
  range: number;
  target: Pt;
  edit: Edit;
  setEdit: React.Dispatch<React.SetStateAction<Edit>>;
  onOpen: (which: number | 'x') => void;
  isFt: boolean;
}) {
  const svgRef = useRef<SVGSVGElement>(null);
  // The gesture, exactly as the real plot holds it: which corner is held (or "all" for a
  // whole-shape drag), the pre-gesture snapshot for one undo step per gesture, whether the
  // pointer really moved, and whether this gesture ADDED the corner (a just-added corner must
  // not select itself - the original shipped that bug once).
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
  const toPoint = (e: React.PointerEvent): Pt => {
    const r = svgRef.current!.getBoundingClientRect();
    const x = (e.clientX - r.left) / r.width * PLOT_HALF_W * 2 - PLOT_HALF_W;
    const y = (e.clientY - r.top) / r.height * PLOT_DEPTH;
    return {
      x: Math.round(Math.max(-PLOT_HALF_W, Math.min(PLOT_HALF_W, x))),
      y: Math.round(Math.max(0, Math.min(PLOT_DEPTH, y)))
    };
  };
  const down = (e: React.PointerEvent) => {
    const p = toPoint(e);
    if (!edit) {
      // Not editing: a tap on a committed shape opens it - the same door as its button below.
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
      // Inside the shape but not on a corner: the whole shape rides the finger.
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
  const move = (e: React.PointerEvent) => {
    if (dragRef.current === null || !edit) return;
    const p = toPoint(e);
    if (dragRef.current === 'all') {
      // The delta is clamped as a group, so no corner can leave the field and the shape can
      // never distort against an edge - it just stops.
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
      // Nothing moves until the pointer travels past a threshold - otherwise a tap would nudge
      // the corner it meant to select.
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
    // One completed gesture, one undo step - whether it added, dragged, or shifted the shape.
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
  const draftClass = edit && edit.which === 'x' ? 'plot-excl draft' : 'plot-zone draft';
  const ftLabel = (cm: number) => `${Math.round(cm / 30.48 * 10) / 10} ft`;
  return <div className="plot">
      <svg ref={svgRef} viewBox={`${-PLOT_HALF_W} 0 ${PLOT_HALF_W * 2} ${PLOT_DEPTH}`} className={`plot-svg${edit ? ' editing' : ''}`} style={edit ? {
      touchAction: 'none'
    } : undefined} onPointerDown={down} onPointerMove={move} onPointerUp={up} onPointerCancel={up}>
        <defs>
          <radialGradient id="fovg" gradientUnits="userSpaceOnUse" cx="0" cy="0" r={PLOT_DEPTH}>
            <stop offset="0" className="fov-in" />
            <stop offset="1" className="fov-out" />
          </radialGradient>
          <filter id="glowf" x="-150%" y="-150%" width="400%" height="400%">
            <feGaussianBlur stdDeviation="12" />
          </filter>
        </defs>
        <g>
          <path className="plot-fov" d={`M0 0 L${-FOV_X} ${FOV_Y} A${PLOT_DEPTH} ${PLOT_DEPTH} 0 0 0 ${FOV_X} ${FOV_Y} Z`} />
          {[200, 400, 600].map(r => <circle key={r} className="plot-ring" cx="0" cy="0" r={r} />)}
          <line className="plot-axis" x1={-PLOT_HALF_W} y1="0" x2={PLOT_HALF_W} y2="0" />
          <line className="plot-axis" x1="0" y1="0" x2="0" y2={PLOT_DEPTH} />

          {range > 0 && <circle className="plot-range" cx="0" cy="0" r={range} />}

          {zones.map((z, i) => !hideZone(i) && z && z.length > 2 ? <g key={`z${i}`}>
                <polygon className="plot-zone" points={polygonPoints(z)} />
                <text className="plot-name" x={labelPoint(z).x} y={labelPoint(z).y}>{`Zone ${i + 1}`}</text>
              </g> : null)}
          {!hideExcl && exclusion.length > 2 && <g>
              <polygon className="plot-excl" points={polygonPoints(exclusion)} />
              <text className="plot-name excl" x={labelPoint(exclusion).x} y={labelPoint(exclusion).y}>
                Exclusion
              </text>
            </g>}

          {edit && edit.points.length > 0 && <>
              {edit.points.length > 2 ? <polygon className={draftClass} points={polygonPoints(edit.points)} /> : <polyline className={draftClass} points={polygonPoints(edit.points)} />}
              {edit.points.length > 2 && <text className={`plot-name${edit.which === 'x' ? ' excl' : ''}`} x={labelPoint(edit.points).x} y={labelPoint(edit.points).y}>
                  {edit.which === 'x' ? 'Exclusion' : `Zone ${(edit.which as number) + 1}`}
                </text>}
              {edit.points.map((p, i) => <circle key={`v${i}`} className={`plot-vtx${edit.sel === i ? ' sel' : ''}`} cx={p.x} cy={p.y} r="16" />)}
            </>}

          <g>
            <g className="plot-tgt" style={{
            transform: `translate(${target.x}px,${target.y}px)`
          }}>
              <circle className="plot-halo" cx="0" cy="0" r="34" />
              <circle className="plot-target" cx="0" cy="0" r="16" />
            </g>
          </g>
        </g>

        {[200, 400, 600].map(r => <text key={r} className="plot-tick" x="14" y={Math.min(r + 26, PLOT_DEPTH - 10)}>
            {isFt ? ftLabel(r) : `${r / 100}m`}
          </text>)}
      </svg>
    </div>;
}

/* ------------------------------------------------------------------ */
/* The LD2410 gate bars                                                */
/* ------------------------------------------------------------------ */

const NUM_GATES = 9;
type Ld2410Cfg = {
  timeout: number;
  max_move_gate: number;
  max_still_gate: number;
  bluetooth: boolean;
  res02: boolean;
  gate_move_thresholds: number[];
  gate_still_thresholds: number[];
};
const ftOf = (cm: number) => cm * 0.0328084;

/**
 * Nine gates of movement and stillness energy, each a bar - and on each bar, the gate's trigger
 * threshold as a draggable notch. Ported from the real Gates component: rows past the group's
 * furthest gate dim rather than disappear, and their notch goes entirely.
 */
function Gates({
  live,
  cfg,
  setCfg,
  maxMove,
  maxStill,
  isFt
}: {
  live: {
    move: number[];
    still: number[];
  };
  cfg: Ld2410Cfg;
  setCfg: React.Dispatch<React.SetStateAction<Ld2410Cfg>>;
  maxMove: number;
  maxStill: number;
  isFt: boolean;
}) {
  const [drag, setDrag] = useState<{
    field: 'gate_move_thresholds' | 'gate_still_thresholds';
    idx: number;
    val: number;
  } | null>(null);
  const res = cfg.res02 ? 0.2 : 0.75;
  const fmt = (v: number) => isFt ? String(parseFloat(ftOf(v * 100).toFixed(1))) : String(parseFloat(v.toFixed(2)));
  const groups: [string, number[], 'gate_move_thresholds' | 'gate_still_thresholds', number, string][] = [['Movement', live.move, 'gate_move_thresholds', maxMove, HINTS.gate_move], ['Stillness', live.still, 'gate_still_thresholds', maxStill, HINTS.gate_still]];
  const pct = (e: React.PointerEvent, el: HTMLElement) => {
    const r = el.getBoundingClientRect();
    return Math.max(0, Math.min(100, Math.round((e.clientX - r.left) / r.width * 100)));
  };
  const commit = (field: 'gate_move_thresholds' | 'gate_still_thresholds', idx: number, val: number) => {
    setCfg(c => {
      const arr = c[field].slice();
      arr[idx] = val;
      return {
        ...c,
        [field]: arr
      };
    });
  };
  return <div className="gates">
      {groups.map(([label, energies, field, maxGate, hint]) => <div className="gates-g" key={field}>
          <div className="ctl-label">
            <span>{label}</span>
            <Hint text={hint} />
          </div>
          {Array.from({
        length: NUM_GATES
      }, (_, i) => {
        const energy = Number(energies[i]) || 0;
        const off = i > maxGate;
        const held = drag && drag.field === field && drag.idx === i;
        const thr = held ? drag.val : Number(cfg[field][i]) || 0;
        return <div className={`gate${off ? ' off' : ''}`} key={i}>
                <span className="gate-n">{`${fmt(i * res)}\u2013${fmt((i + 1) * res)}${isFt ? 'ft' : 'm'}`}</span>
                <div className={`gate-bar${off ? '' : ' editable'}`} onPointerDown={e => {
            if (off) return;
            e.currentTarget.setPointerCapture(e.pointerId);
            setDrag({
              field,
              idx: i,
              val: pct(e, e.currentTarget)
            });
          }} onPointerMove={e => {
            if (!held) return;
            setDrag({
              field,
              idx: i,
              val: pct(e, e.currentTarget)
            });
          }} onPointerUp={e => {
            if (!held) return;
            commit(field, i, pct(e, e.currentTarget));
            setDrag(null);
          }} onPointerCancel={() => setDrag(null)}>
                  <div className="gate-fill" style={{
              width: `${Math.min(100, energy)}%`
            }} />
                  {!off && <div className="gate-thr" style={{
              left: `${thr}%`
            }} title={`Threshold ${thr}`} />}
                </div>
              </div>;
      })}
        </div>)}
      <p className="dim sm">{TEXT.gate_thresholds_help}</p>
    </div>;
}

/* ------------------------------------------------------------------ */
/* No module fitted                                                    */
/* ------------------------------------------------------------------ */

/** A doorway, not a dead end - the owner's wording and the owner's product photo. */
function NoSensor() {
  return <Card title="Presence" icon={N_PRES}>
      <p className="sm">
        {TEXT.no_sensor_lead}
        <a href={TEXT.no_sensor_docs_url} target="_blank" rel="noreferrer">
          {TEXT.no_sensor_docs}
        </a>
        {TEXT.no_sensor_mid}
        <a href={TEXT.no_sensor_buy_url} target="_blank" rel="noreferrer">
          {TEXT.no_sensor_buy}
        </a>
      </p>
      <img className="promo" src="/assets/no-sensor.webp" alt="Satellite1 with a hidden mmWave radar sensor" />
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Status pills                                                        */
/* ------------------------------------------------------------------ */

function StatusPills({
  kind,
  target,
  ld2410,
  isFt,
  setIsFt
}: {
  kind: 'ld2450' | 'ld2410';
  target: Pt;
  ld2410: {
    state: string;
    dist: number;
  };
  isFt: boolean;
  setIsFt: (v: boolean) => void;
}) {
  const [open, setOpen] = useState<string | null>(null);
  const d = kind === 'ld2450' ? Math.hypot(target.x, target.y) : ld2410.dist;
  const ang = Math.atan2(target.x, target.y) * 180 / Math.PI;
  const dist = isFt ? `${(d / 30.48).toFixed(1)} ft` : `${(d / 100).toFixed(1)} m`;
  return <Card>
      <div className="pills">
        {kind === 'ld2450' ? <>
            <span className="pill ro">
              <span className="pill-v">Zone 1</span>
              <span className="pill-l">Presence</span>
            </span>
            <span className="pill ro">
              <span className="pill-v">1</span>
              <span className="pill-l">People</span>
            </span>
            <Pill id="dist" open={open} setOpen={setOpen} label="Distance" value={dist} />
            <span className="pill ro">
              <span className="pill-v">{ang < -15 ? 'Left' : ang > 15 ? 'Right' : 'Ahead'}</span>
              <span className="pill-l">Direction</span>
            </span>
          </> : <>
            {/* The LD2410 has no zones or coordinates: the kind of presence and one distance are
                the whole story it can tell, so the row carries exactly two pills. */}
            <span className="pill ro">
              <span className="pill-v">{ld2410.state}</span>
              <span className="pill-l">Presence</span>
            </span>
            <Pill id="dist" open={open} setOpen={setOpen} label="Distance" value={dist} />
          </>}
      </div>
      {open === 'dist' && <div className="editor">
          <div className="row sm">
            <span className="dim">Feet</span>
            <Hint text={HINTS.distance_unit} />
            <span className="grow" />
            <Toggle checked={isFt} onChange={setIsFt} />
          </div>
        </div>}
    </Card>;
}

/* ------------------------------------------------------------------ */

export function PresenceRoute() {
  // Which radar module the mockup simulates. The real firmware detects the fitted module and renders
  // the matching page; here a preview-only switcher stands in for the hardware so every variant -
  // the LD2450 plot, the LD2410 gate tuner, and the no-module card - can be seen.
  const [kind, setKind] = useState<'ld2450' | 'ld2410' | 'none'>('ld2450');
  const target = useWanderingTarget();
  const [isFt, setIsFt] = useState(false);
  // The LD2410's config, defaults from the module's own datasheet values.
  const [cfg10, setCfg10] = useState<Ld2410Cfg>({
    timeout: 30,
    max_move_gate: 6,
    max_still_gate: 6,
    bluetooth: false,
    res02: false,
    gate_move_thresholds: [50, 50, 40, 30, 20, 15, 15, 15, 15],
    gate_still_thresholds: [0, 0, 40, 40, 30, 30, 20, 20, 20]
  });
  // Slider previews for the furthest-gate settings, so the chart dims in real time while dragging.
  const [pv10, setPv10] = useState<{
    m?: number;
    s?: number;
  } | null>(null);
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
  const defined = [0, 1, 2].filter(i => (zones[i] || []).length > 2);
  const hasExcl = exclusion.length > 2;
  const firstFree = [0, 1, 2].find(i => (zones[i] || []).length < 3);
  const isExcl = edit && edit.which === 'x';
  // Whether the Exclusion toggle can be switched off: the shape needs a zone slot to become.
  const zoneSlot = edit && typeof edit.from === 'number' ? edit.from : firstFree;
  const savable = edit && (edit.points.length === 0 || edit.points.length >= 3);
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

  /** Every save sends the full set, exactly as the device endpoint demands. `from` empties and
   *  `which` fills in one write - which is also how the Exclusion toggle converts a shape. */
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

  /** Deletes the committed shape outright - only offered for shapes that exist on the device;
   *  a never-saved draft has nothing to delete, and Cancel already discards it. */
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
  // The preview-only module switcher, present on every variant so there is always a way back.
  const switcher = <Card>
      <p className="dim sm">{'Mockup preview: pick which radar module the device reports. The real firmware detects the fitted module on its own.'}</p>
      <div className="row-actions">
        <Btn solid={kind === 'ld2450'} onClick={() => setKind('ld2450')}>
          LD2450
        </Btn>
        <Btn solid={kind === 'ld2410'} onClick={() => setKind('ld2410')}>
          LD2410
        </Btn>
        <Btn solid={kind === 'none'} onClick={() => setKind('none')}>
          Disconnected
        </Btn>
      </div>
    </Card>;

  // No module fitted: the real route shows only the doorway card - no pills, no settings.
  if (kind === 'none') return <>
      {switcher}
      <NoSensor />
    </>;
  return <>
      {switcher}
      <StatusPills kind={kind} target={target} ld2410={live10} isFt={isFt} setIsFt={setIsFt} />

      <Card title={kind === 'ld2450' ? 'LD2450' : 'LD2410'} icon={N_PRES} hint={HINTS.presence} right={<span className="live-dot" title="Polling every 250ms">
            live
          </span>}>
        {kind === 'ld2410' ? <Gates live={live10} cfg={cfg10} setCfg={setCfg10} maxMove={pv10?.m ?? cfg10.max_move_gate} maxStill={pv10?.s ?? cfg10.max_still_gate} isFt={isFt} /> : <>
        <Plot zones={zones} exclusion={exclusion} range={preview ?? cfg.detection_range} target={target} edit={edit} setEdit={setEdit} onOpen={openShape} isFt={isFt} />

        {!edit && !defined.length && !hasExcl ? <Empty icon={<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinejoin="round" aria-hidden="true">
                <path d="M3.2 4.6 12.6 3l.4 8.4-9.6 1.6z" strokeDasharray="2.8 2.2" />
              </svg>} text={TEXT.zones_none} /> : <p className="dim sm">{instruction}</p>}

        {edit ? <div className="editor">
            <div className="row">
              <span className="grow strong">{isExcl ? 'Exclusion' : `Zone ${(edit.which as number) + 1}`}</span>
              <span className="dim sm">{`${edit.points.length}/${MAX_POINTS} corners`}</span>
            </div>
            <div className="row sm">
              <span className="dim">Exclusion zone</span>
              <Hint text={HINTS.zone_excl} />
              <span className="grow" />
              <Toggle checked={!!isExcl} disabled={!!isExcl && zoneSlot === undefined} onChange={v => setEdit(e => e && {
            ...e,
            which: v ? 'x' : zoneSlot ?? 0
          })} />
            </div>
            <div className="row-actions">
              <Btn onClick={() => setEdit(e => !e || e.hist.length === 0 ? e : {
            ...e,
            points: e.hist[e.hist.length - 1],
            hist: e.hist.slice(0, -1),
            sel: null
          })} disabled={!edit || edit.hist.length === 0}>
                Undo
              </Btn>
              {edit.sel != null && <Btn onClick={() => setEdit(e => e && {
            ...e,
            points: e.points.filter((_, i) => i !== e.sel),
            sel: null,
            hist: [...e.hist, e.points]
          })}>
                  Remove corner
                </Btn>}
              <Btn onClick={() => setEdit(null)}>Cancel</Btn>
              <Btn onClick={save} disabled={!savable} solid>
                Save
              </Btn>
              {edit.from != null && <Btn onClick={del} danger>
                  Delete
                </Btn>}
            </div>
          </div> : <div className="row-actions">
            {defined.map(i => <Btn key={i} onClick={() => openShape(i)}>{`Zone ${i + 1}`}</Btn>)}
            {hasExcl && <Btn onClick={() => openShape('x')}>Exclusion</Btn>}
            {(firstFree !== undefined || !hasExcl) && <Btn solid onClick={() => {
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
        }}>
                Add zone
              </Btn>}
            <Hint text={HINTS.radar_zones} />
          </div>}
        </>}
      </Card>

      <Card title="Settings">
        {kind === 'ld2410' ? <>
            {/* Same control and same meaning as the LD2450's Timeout, so the same hint. */}
            <Row label="Timeout" hint={HINTS.radar_timeout}>
              <Slider value={cfg10.timeout} min={0} max={300} step={5} format={v => `${Math.round(v)} s`} onCommit={v => setCfg10(c => ({
          ...c,
          timeout: Math.round(v)
        }))} />
            </Row>

            {/* The previews dim the gate chart above in real time as these are dragged. */}
            <Row label="Furthest movement gate" hint={HINTS.gate_max_move}>
              <Slider value={cfg10.max_move_gate} min={0} max={8} step={1} format={v => `${Math.round(v)}`} onPreview={v => setPv10(p => ({
          ...p,
          m: Math.round(v)
        }))} onCommit={v => {
          setPv10(null);
          setCfg10(c => ({
            ...c,
            max_move_gate: Math.round(v)
          }));
        }} />
            </Row>

            <Row label="Furthest stillness gate" hint={HINTS.gate_max_still}>
              <Slider value={cfg10.max_still_gate} min={0} max={8} step={1} format={v => `${Math.round(v)}`} onPreview={v => setPv10(p => ({
          ...p,
          s: Math.round(v)
        }))} onCommit={v => {
          setPv10(null);
          setCfg10(c => ({
            ...c,
            max_still_gate: Math.round(v)
          }));
        }} />
            </Row>

            <Row label="Bluetooth">
              <Toggle checked={cfg10.bluetooth} onChange={v => setCfg10(c => ({
          ...c,
          bluetooth: v
        }))} />
            </Row>

            {/* A two-stop slider rather than a dropdown: its ends are the two choices. */}
            <Row label="Distance resolution" hint={HINTS.radar_resolution}>
              <Slider value={cfg10.res02 ? 0 : 1} min={0} max={1} step={1} format={v => Math.round(v) === 0 ? isFt ? '0.7 ft' : '0.2 m' : isFt ? '2.5 ft' : '0.75 m'} onCommit={v => setCfg10(c => ({
          ...c,
          res02: Math.round(v) === 0
        }))} />
            </Row>
          </> : <>
        <Row label="Detection range" hint={HINTS.radar_range}>
          <Slider value={cfg.detection_range} min={0} max={600} step={10} format={v => Number(v) === 0 ? isFt ? '20 ft' : '6 m' : isFt ? `${(v / 30.48).toFixed(1)} ft` : `${Math.round(v)} cm`} onPreview={v => setPreview(Math.round(v))} onCommit={v => {
          setPreview(null);
          setCfg(c => ({
            ...c,
            detection_range: Math.round(v)
          }));
        }} />
        </Row>

        <Row label="Stability" hint={HINTS.radar_stability}>
          <Slider value={cfg.stability} min={0} max={10} step={1} format={v => `${Math.round(v)}`} onCommit={v => setCfg(c => ({
          ...c,
          stability: Math.round(v)
        }))} />
        </Row>

        <Row label="Timeout" hint={HINTS.radar_timeout}>
          <Slider value={cfg.timeout} min={0} max={300} step={5} format={v => `${Math.round(v)} s`} onCommit={v => setCfg(c => ({
          ...c,
          timeout: Math.round(v)
        }))} />
        </Row>

        <Row label="Multi-target" hint={HINTS.radar_multi}>
          <Toggle checked={cfg.multi_target} onChange={v => setCfg(c => ({
          ...c,
          multi_target: v
        }))} />
        </Row>

        <Row label="Bluetooth" hint={HINTS.radar_bt}>
          <Toggle checked={cfg.bluetooth} onChange={v => setCfg(c => ({
          ...c,
          bluetooth: v
        }))} />
        </Row>
          </>}
      </Card>
    </>;
}