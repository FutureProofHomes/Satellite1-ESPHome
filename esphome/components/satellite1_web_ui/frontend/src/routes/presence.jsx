/**
 * Presence: what the radar sees, and the settings that change what it sees.
 *
 * Built entirely on `satellite1_radar`'s existing JSON API rather than on entities. That is not a
 * shortcut - the radar's settings are not ESPHome entities at all, they live in the module's own
 * config and are read and written over `/api/v1/<module>/config` - the same API the retired
 * `/radar_tuner` pages used, which is what made side-by-side parity testing honest while both
 * faces existed.
 *
 * Zone and exclusion polygons are edited here, on the same SVG plot that draws them - tap to add a
 * corner, drag a corner to move it. The plot's viewBox is radar centimetres, so a pointer position
 * converts to device coordinates with two divisions and no calibration, which is the whole reason
 * this needed no canvas and no library.
 *
 * Engineering mode needs no explicit enter and exit. The LD2410 only reports gate energies while it is
 * on, and the handler arms it from the live poll and lets it lapse once polling stops - so mounting
 * this route is the entire protocol. A tab closed mid-session cannot leave the radar in engineering
 * mode, which an explicit exit POST could.
 *
 * "Reboot radar" reboots the whole device. `/api/v1/reboot` calls `App.safe_reboot()`. So it is not a
 * permanent button labelled as a radar action; it appears only when the device says a restart is
 * needed, and says what it will actually do.
 *
 * When no module is detected the route stays - the owner's decision, reversing an earlier plan to
 * hide it - and shows a card that says what is missing and where to get one, with the product photo.
 * The image is served by the device itself at /ui/no-sensor.webp (fully self-contained firmware, no
 * CDN), so the card renders whole even offline; only the two links need the internet.
 */
import { useRef, useState } from "preact/hooks";

import { HINTS, PRESENCE, TEXT } from "../copy.js";
import { RADAR_LIVE_MS, useRadar } from "../lib/device.js";
import { Btn, Card, Empty, Hint, Missing, N_PRES, Row, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* The LD2450 plot                                                     */
/* ------------------------------------------------------------------ */

/** Half-width of the plotted area, in cm. The LD2450's field of view is about ±60°, so at its 600cm
 *  ceiling the lateral extent it can report is wider than the range itself. 400 keeps a person at the
 *  edge of a normal room on screen without shrinking the middle, where everyone actually is. */
const PLOT_HALF_W = 400;
const PLOT_DEPTH = 600;

/** The handler stores at most this many corners per polygon (MAX_ZONE_POINTS in ld2450_handler.h),
 *  and rejects the whole POST for more, so the editor stops appending at the same number. */
const MAX_POINTS = 8;

/** How close a tap must land to an existing corner, in plot centimetres, to grab it instead of adding
 *  a new one. 36cm is about 16px at a phone's plot width - a finger, not a cursor. */
const GRAB_CM = 36;

/** A target at exactly the origin is the module's way of saying "this slot is empty" - it reports three
 *  slots whether or not it has three people. Filtering on 0,0 rather than on a count is what the
 *  payload supports; there is no count field. */
function realTargets(live) {
  const t = (live && live.targets) || [];
  return t
    .map((p, i) => ({ ...p, i }))
    .filter((p) => !(p.x === 0 && p.y === 0));
}

function polygonPoints(points) {
  return (points || []).map((p) => `${p.x},${p.y}`).join(" ");
}

/** Where a polygon's name is drawn. The mean of the corners, which is not the true centroid of a
 *  lopsided shape, but zones are rooms and rooms are convex enough for a label. */
function labelPoint(points) {
  let x = 0;
  let y = 0;
  for (const p of points) {
    x += p.x;
    y += p.y;
  }
  return { x: x / points.length, y: y / points.length };
}

/** Ray-cast point-in-polygon, for telling "grab the whole shape" from "add a corner here". */
function inPolygon(p, pts) {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const a = pts[i];
    const b = pts[j];
    if (a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}

/**
 * SVG in radar coordinates, not pixels: the viewBox is centimetres, so every zone polygon and target
 * is plotted with the numbers the device actually sent and nothing has to be scaled by hand. The
 * sensor sits at the top centre with the field fanning downward - the legacy tuner's orientation,
 * kept at the owner's request - which happens to be SVG's native direction: radar y grows away from
 * the device and screen y grows down, so no flip transform is needed at all.
 *
 * Editing rides the same coordinate system in reverse: a pointer position maps back to centimetres
 * through the plot's bounding box, so the corner someone drags is stored with the number the device
 * will be told, not a pixel that needs converting later. One pointer handler covers every gesture -
 * down near a corner grabs it, down inside the shape grabs the whole shape, down anywhere else appends
 * a corner and grabs that - because on a phone all of these are the same finger and should not fight
 * over it. A press on a corner that never travels is a tap, and a tap selects the corner, which is
 * what arms the Remove corner button on the card below.
 */
/** The wedge's corners: the LD2450 fans ±60° from straight ahead, so at the 600cm ceiling its edges
 *  pass through (±600·sin60°, 600·cos60°). Wider than the viewBox on purpose - the SVG clips the
 *  wings, and narrowing the angle to fit would draw a field the module does not have. */
const FOV_X = Math.round(PLOT_DEPTH * Math.sin(Math.PI / 3));
const FOV_Y = PLOT_DEPTH / 2;

function Plot({ live, config, edit, onEdit, onSelect, onOpen, onHist }) {
  const svgRef = useRef(null);
  const trailRef = useRef({}); // per target slot, its last few positions - the comet tail
  const dragRef = useRef(null); // index of the held corner, "all" for the whole shape, null when idle
  const movedRef = useRef(false); // travelled past the tap threshold, so release must not select
  const addedRef = useRef(false); // this press created the corner, so release must not select either
  const lastRef = useRef(null); // previous pointer position while the whole shape is dragged
  const startRef = useRef(null); // where the press landed, for the tap-or-drag decision
  const preRef = useRef(null); // the points as they were before this gesture, for the undo history
  const changedRef = useRef(false); // whether this gesture actually mutated anything

  const targets = realTargets(live);
  const zones = (config && config.zones) || [];
  const exclusion = (config && config.exclusion) || [];
  const range = config && config.detection_range ? config.detection_range : 0;

  // The trail bookkeeping. A ref, not state: it is derived from the poll that already re-rendered
  // this component, so recording it must not schedule another render. Guarded on "did it move",
  // which also makes the mutation safe to run twice in one render. Slots the module stopped
  // reporting lose their history, so a tail never outlives its person.
  const trails = trailRef.current;
  const seen = new Set();
  for (const t of targets) {
    seen.add(t.i);
    const a = trails[t.i] || (trails[t.i] = []);
    const last = a[a.length - 1];
    if (!last || last.x !== t.x || last.y !== t.y) {
      a.push({ x: t.x, y: t.y });
      if (a.length > 4) a.shift();
    }
  }
  for (const k of Object.keys(trails)) if (!seen.has(Number(k))) delete trails[k];

  const toPoint = (e) => {
    const r = svgRef.current.getBoundingClientRect();
    const x = Math.round(((e.clientX - r.left) / r.width) * PLOT_HALF_W * 2 - PLOT_HALF_W);
    // Sensor at the top: screen y and radar y point the same way, so the fraction maps directly.
    const y = Math.round(((e.clientY - r.top) / r.height) * PLOT_DEPTH);
    return {
      x: Math.max(-PLOT_HALF_W, Math.min(PLOT_HALF_W, x)),
      y: Math.max(0, Math.min(PLOT_DEPTH, y)),
    };
  };

  const down = (e) => {
    // Not editing: a tap on a committed shape opens its editor. The plot is the picture of the zones,
    // so it is also the most direct way to say which one you mean - zones first, so a shape drawn
    // inside the ignore area still opens as itself.
    if (!edit) {
      const p = toPoint(e);
      for (let i = 0; i < 3; i++) {
        if ((zones[i] || []).length > 2 && inPolygon(p, zones[i])) return onOpen(i);
      }
      if (exclusion.length > 2 && inPolygon(p, exclusion)) return onOpen("x");
      return;
    }
    e.preventDefault();
    const p = toPoint(e);
    movedRef.current = false;
    addedRef.current = false;
    changedRef.current = false;
    startRef.current = p;
    // Snapshotted before anything mutates: one gesture is one undo step, whether it added a corner,
    // dragged one, or moved the whole shape.
    preRef.current = edit.points.map((q) => ({ ...q }));

    let idx = -1;
    let best = GRAB_CM;
    edit.points.forEach((q, i) => {
      const d = Math.hypot(q.x - p.x, q.y - p.y);
      if (d < best) {
        best = d;
        idx = i;
      }
    });

    if (idx >= 0) {
      dragRef.current = idx;
    } else if (edit.points.length > 2 && inPolygon(p, edit.points)) {
      // Inside the shape but not on a corner: the whole polygon rides the finger.
      dragRef.current = "all";
      lastRef.current = p;
    } else {
      if (edit.points.length >= MAX_POINTS) return;
      onEdit([...edit.points, p]);
      onSelect(null);
      dragRef.current = edit.points.length;
      addedRef.current = true;
      changedRef.current = true;
    }
    svgRef.current.setPointerCapture(e.pointerId);
  };

  const move = (e) => {
    if (dragRef.current === null) return;
    const p = toPoint(e);
    if (dragRef.current === "all") {
      // The delta is clamped as a group before it is applied, so no corner can leave the field and
      // the shape can never distort against an edge - it just stops.
      let dx = p.x - lastRef.current.x;
      let dy = p.y - lastRef.current.y;
      const xs = edit.points.map((q) => q.x);
      const ys = edit.points.map((q) => q.y);
      dx = Math.max(-PLOT_HALF_W - Math.min(...xs), Math.min(PLOT_HALF_W - Math.max(...xs), dx));
      dy = Math.max(-Math.min(...ys), Math.min(PLOT_DEPTH - Math.max(...ys), dy));
      if (dx === 0 && dy === 0) return;
      movedRef.current = true;
      changedRef.current = true;
      lastRef.current = { x: lastRef.current.x + dx, y: lastRef.current.y + dy };
      onEdit(edit.points.map((q) => ({ x: q.x + dx, y: q.y + dy })));
    } else {
      // A finger is never perfectly still, so nothing moves until the pointer travels past a
      // threshold - otherwise a tap would nudge the corner it meant to select, and the silent
      // sub-threshold drift would be a mutation the undo history never saw.
      if (!movedRef.current && Math.hypot(p.x - startRef.current.x, p.y - startRef.current.y) <= 12) return;
      movedRef.current = true;
      changedRef.current = true;
      onEdit(edit.points.map((q, i) => (i === dragRef.current ? p : q)));
    }
  };

  const up = () => {
    const held = dragRef.current;
    dragRef.current = null;
    // One completed gesture, one undo step: the pre-gesture snapshot goes on the history whether the
    // gesture added, dragged, or shifted the whole shape.
    if (changedRef.current) {
      changedRef.current = false;
      onHist(preRef.current);
    }
    if (typeof held === "number" && !movedRef.current && !addedRef.current) {
      onSelect(edit.sel === held ? null : held);
    }
  };

  // The polygon being edited is drawn from the draft, not from the config, and the config's copies of
  // both the shape it came from and the slot it will save into are skipped, so the screen never shows
  // a stale shape and the draft on top of each other - `from` and `which` differ when the Exclusion
  // toggle is converting a shape from one kind to the other.
  const hideZone = (i) => edit && (edit.from === i || edit.which === i);
  const hideExcl = edit && (edit.from === "x" || edit.which === "x");
  const draftClass = edit && edit.which === "x" ? "plot-excl draft" : "plot-zone draft";

  return (
    <div class="plot">
      <svg
        ref={svgRef}
        viewBox={`${-PLOT_HALF_W} 0 ${PLOT_HALF_W * 2} ${PLOT_DEPTH}`}
        class={`plot-svg${edit ? " editing" : ""}`}
        onPointerDown={down}
        onPointerMove={move}
        onPointerUp={up}
        onPointerCancel={up}
      >
        <defs>
          {/* userSpaceOnUse so the gradient is centimetres from the sensor, not a fraction of the
              wedge's own box - the glow must sit at the device however the wedge is clipped. */}
          <radialGradient id="fovg" gradientUnits="userSpaceOnUse" cx="0" cy="0" r={PLOT_DEPTH}>
            <stop offset="0" class="fov-in" />
            <stop offset="1" class="fov-out" />
          </radialGradient>
          {/* The target's glow. The filter box is grown because blur samples outside the circle's
              own bounds, and the default region clips the halo into a square. */}
          <filter id="glowf" x="-150%" y="-150%" width="400%" height="400%">
            <feGaussianBlur stdDeviation="12" />
          </filter>
        </defs>
        <g>
          {/* The field of view: what the module actually watches, washed with a gradient that is
              brightest at the sensor and gone by the far edge. Also the answer to why a corner of
              the plot never shows anyone - it is outside this wedge. */}
          <path class="plot-fov" d={`M0 0 L${-FOV_X} ${FOV_Y} A${PLOT_DEPTH} ${PLOT_DEPTH} 0 0 0 ${FOV_X} ${FOV_Y} Z`} />
          {/* Distance rings every 2m, which is how people describe a room. */}
          {[200, 400, 600].map((r) => (
            <circle key={r} class="plot-ring" cx="0" cy="0" r={r} />
          ))}
          <line class="plot-axis" x1={-PLOT_HALF_W} y1="0" x2={PLOT_HALF_W} y2="0" />
          <line class="plot-axis" x1="0" y1="0" x2="0" y2={PLOT_DEPTH} />

          {/* The configured cut-off, drawn so a target beyond it visibly sits outside. 0 means the
              module's own default rather than "no range", so nothing is drawn for it. */}
          {range > 0 && <circle class="plot-range" cx="0" cy="0" r={range} />}

          {/* Every committed shape carries its name at its middle, so three blue polygons on one plot
              are tellable apart without counting corners against the card below. */}
          {zones.map((z, i) =>
            !hideZone(i) && z && z.length > 2 ? (
              <g key={`z${i}`}>
                <polygon class="plot-zone" points={polygonPoints(z)} />
                <text class="plot-name" x={labelPoint(z).x} y={labelPoint(z).y}>{`Zone ${i + 1}`}</text>
              </g>
            ) : null
          )}
          {!hideExcl && exclusion.length > 2 && (
            <g>
              <polygon class="plot-excl" points={polygonPoints(exclusion)} />
              <text class="plot-name excl" x={labelPoint(exclusion).x} y={labelPoint(exclusion).y}>
                Exclusion
              </text>
            </g>
          )}

          {edit && edit.points.length > 0 && (
            <>
              {/* A polyline until there are three corners, because a two-point "polygon" renders as
                  nothing and the second tap would look like it did not land. */}
              {edit.points.length > 2 ? (
                <polygon class={draftClass} points={polygonPoints(edit.points)} />
              ) : (
                <polyline class={draftClass} points={polygonPoints(edit.points)} />
              )}
              {edit.points.length > 2 && (
                <text class={`plot-name${edit.which === "x" ? " excl" : ""}`} x={labelPoint(edit.points).x} y={labelPoint(edit.points).y}>
                  {edit.which === "x" ? "Exclusion" : `Zone ${edit.which + 1}`}
                </text>
              )}
              {edit.points.map((p, i) => (
                <circle key={`v${i}`} class={`plot-vtx${edit.sel === i ? " sel" : ""}`} cx={p.x} cy={p.y} r="16" />
              ))}
            </>
          )}

          {/* Drawn at the origin and placed by a CSS transform, because transforms transition and
              geometry attributes do not everywhere: the dot glides between 250ms polls instead of
              teleporting, and a walking person reads as walking. The dot is a blurred halo behind a
              solid core, and behind both, the last few polled positions as fading echoes - keyed by
              age, so when the history shifts each echo glides to the next position and the tail
              follows the person like a comet's. */}
          {targets.map((t) => (
            <g key={t.i}>
              {(trails[t.i] || []).slice(0, -1).map((p, j) => (
                <circle
                  key={`e${j}`}
                  class="plot-trail"
                  cx="0"
                  cy="0"
                  r={8 + 3 * j}
                  style={`transform:translate(${p.x}px,${p.y}px);opacity:${0.1 + 0.09 * j}`}
                />
              ))}
              <g class="plot-tgt" style={`transform:translate(${t.x}px,${t.y}px)`}>
                <circle class="plot-halo" cx="0" cy="0" r="34" />
                <circle class="plot-target" cx="0" cy="0" r="16" />
              </g>
            </g>
          ))}
        </g>

        {/* Labelled on the rings, not along an edge. The device sits at the top centre, so a row of
            ticks spread across the full width puts "0" at 400cm to the left - which is what the first
            version did, and it made the scale actively misleading. A ring of radius r crosses the
            centreline at y = r; the label sits just below that crossing, except the outermost ring's,
            which sits just above it - r + 26 for the 600cm ring is past the bottom of the viewBox,
            which is how the 6m label went missing. */}
        {[200, 400, 600].map((r) => (
          <text key={r} class="plot-tick" x="14" y={Math.min(r + 26, PLOT_DEPTH - 10)}>
            {r / 100}m
          </text>
        ))}
      </svg>

      {/* Nothing under the plot: the coordinates readout went first (the pills say it better), and the
          instruction line now lives with the zone controls below - one place for every instruction,
          per the owner. */}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The LD2450 zones                                                    */
/* ------------------------------------------------------------------ */

/**
 * Three zones and one exclusion area: a button per defined shape, an Add zone button while a slot is
 * free, and a subcard (the Controls page's offset-editor pattern) while a shape is being drawn.
 *
 * The edit object carries `from` (the slot the shape came from - null when Add zone started it) and
 * `which` (the slot Save will write). They differ only while the Exclusion toggle is converting a
 * shape from one kind to the other; Save then empties `from` and fills `which` in one POST, since the
 * handler takes `zones` and `exclusion` in the same body. `zones` must arrive as all three polygons or
 * not at all (the handler 400s on any other count), so every save sends the full set.
 *
 * Save is disabled between one and two corners, because the handler would store the shape but the
 * firmware treats anything under three corners as "not defined" - a silently dead zone is the worst
 * possible outcome of a save button. Zero corners is a legitimate save: it is how a shape is deleted.
 */
function Zones({ config, write, busy, edit, setEdit, open }) {
  const zones = config.zones || [];
  const exclusion = config.exclusion || [];

  const defined = [0, 1, 2].filter((i) => (zones[i] || []).length > 2);
  const hasExcl = exclusion.length > 2;
  const firstFree = [0, 1, 2].find((i) => (zones[i] || []).length < 3);

  const add = () => {
    if (firstFree !== undefined) setEdit({ from: null, which: firstFree, points: [], sel: null, hist: [] });
    // All three zones taken: the only shape left to add is the ignore area.
    else if (!hasExcl) setEdit({ from: null, which: "x", points: [], sel: null, hist: [] });
  };

  const isExcl = edit && edit.which === "x";
  // Whether the Exclusion toggle can be switched off: the shape needs a zone slot to become.
  const zoneSlot = edit && typeof edit.from === "number" ? edit.from : firstFree;

  const save = async () => {
    const pts = edit.points.map(({ x, y }) => ({ x, y }));
    const zs = [0, 1, 2].map((i) => (zones[i] || []).map(({ x, y }) => ({ x, y })));
    let ex = exclusion.map(({ x, y }) => ({ x, y }));
    if (edit.from === "x") ex = [];
    else if (typeof edit.from === "number") zs[edit.from] = [];
    if (edit.which === "x") ex = pts;
    else zs[edit.which] = pts;
    await write({ zones: zs, exclusion: ex });
    setEdit(null);
  };

  /** Deletes the committed shape outright - empty polygon written, editor closed. Replaced Clear at
   *  the owner's call: "clear the draft, then save the emptiness" was two steps for one intention.
   *  Only offered for shapes that exist on the device; a never-saved draft has nothing to delete,
   *  and Cancel already discards it. */
  const del = async () => {
    const zs = [0, 1, 2].map((i) => (zones[i] || []).map(({ x, y }) => ({ x, y })));
    let ex = exclusion.map(({ x, y }) => ({ x, y }));
    if (edit.from === "x") ex = [];
    else zs[edit.from] = [];
    await write({ zones: zs, exclusion: ex });
    setEdit(null);
  };

  const savable = edit && (edit.points.length === 0 || edit.points.length >= 3);

  /* The one instruction line, above whatever sits below it - buttons or editor, per the owner: it
     walks the person through drawing while the subcard stays pure controls. */
  const instruction = edit
    ? edit.points.length === 0
      ? TEXT.zi_first
      : edit.points.length < 3
        ? TEXT.zi_more
        : edit.sel != null
          ? TEXT.zi_selected
          : TEXT.zi_adjust
    : defined.length || hasExcl
      ? TEXT.zones_set
      : TEXT.zones_none;

  return (
    <>
      {/* The first-run case gets the drawn empty state; every other state of the shared instruction
          line stays a plain sentence, because mid-edit guidance changing shape per tap would strobe. */}
      {!edit && !defined.length && !hasExcl ? (
        <Empty
          icon={
            <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linejoin="round" aria-hidden="true">
              <path d="M3.2 4.6 12.6 3l.4 8.4-9.6 1.6z" stroke-dasharray="2.8 2.2" />
            </svg>
          }
          text={TEXT.zones_none}
        />
      ) : (
        <p class="dim sm">{instruction}</p>
      )}
      {edit ? (
        <div class="editor">
          <div class="row">
            <span class="grow strong">{isExcl ? "Exclusion" : `Zone ${edit.which + 1}`}</span>
            {/* .dim.sm, not .ctl-sub: ctl-sub is width:100% for the stacked labels it belongs to, and
                inside this flex row that width is what wrapped the title onto the counter. */}
            <span class="dim sm">{`${edit.points.length}/${MAX_POINTS} corners`}</span>
          </div>
          <div class="row sm">
            <span class="dim">Exclusion zone</span>
            <Hint text={HINTS.zone_excl} />
            <span class="grow" />
            <Toggle
              checked={isExcl}
              disabled={busy || (isExcl && zoneSlot === undefined)}
              onChange={(v) => setEdit((e) => ({ ...e, which: v ? "x" : zoneSlot }))}
            />
          </div>
          <div class="row-actions">
            {/* Undo replays the history the plot records - one step per gesture - so a moved corner
                goes back where it was rather than vanishing, which is what "undo a move" means. The
                selection drops because the popped snapshot may not contain the selected index. */}
            <Btn
              onClick={() =>
                setEdit((e) => (e.hist.length === 0 ? e : { ...e, points: e.hist[e.hist.length - 1], hist: e.hist.slice(0, -1), sel: null }))
              }
              disabled={edit.hist.length === 0}
            >
              Undo
            </Btn>
            {edit.sel != null && (
              <Btn
                onClick={() =>
                  setEdit((e) => ({ ...e, points: e.points.filter((_, i) => i !== e.sel), sel: null, hist: [...e.hist, e.points] }))
                }
              >
                Remove corner
              </Btn>
            )}
            <Btn onClick={() => setEdit(null)}>Cancel</Btn>
            <Btn onClick={save} disabled={!savable || busy} solid>
              Save
            </Btn>
            {edit.from != null && (
              <Btn onClick={del} disabled={busy} danger>
                Delete
              </Btn>
            )}
          </div>
        </div>
      ) : (
        <div class="row-actions">
          {defined.map((i) => (
            <Btn key={i} onClick={() => open(i)} disabled={busy}>{`Zone ${i + 1}`}</Btn>
          ))}
          {hasExcl && (
            <Btn onClick={() => open("x")} disabled={busy}>
              Exclusion
            </Btn>
          )}
          {(firstFree !== undefined || !hasExcl) && (
            <Btn onClick={add} disabled={busy} solid>
              Add zone
            </Btn>
          )}
          {/* The zones explainer rides the button row now that there is no Zones card header to carry it. */}
          <Hint text={HINTS.radar_zones} />
        </div>
      )}
    </>
  );
}

/* ------------------------------------------------------------------ */
/* The LD2410 gate bars                                                */
/* ------------------------------------------------------------------ */

const NUM_GATES = 9;

/**
 * Nine gates of movement and stillness energy, each a bar - and on each bar, the gate's trigger
 * threshold as a draggable handle. The live energy underneath is exactly the reference a person needs
 * to place a threshold, which is why the two share a bar instead of the threshold being a second
 * column of number inputs.
 *
 * Dragging updates only local state; release commits the whole nine-value array, because that is the
 * only shape the endpoint accepts, and the write hook follows it with the /apply the module needs.
 *
 * Written from the handler's payload rather than from hardware: this device has an LD2450 fitted, so
 * the shape here is `{"gates":{"move":[...],"still":[...]}}` as `handle_ld2410_live_` emits it, and the
 * layout is unverified against a real LD2410. Flagged rather than presented as tested.
 */
function Gates({ live, config, write, busy }) {
  const [drag, setDrag] = useState(null); // { field, idx, val } while a handle is held

  /* Each gate is one slice of distance, so it is labelled with the metres it actually watches rather
     than its index - "2.25-3" says what "3" never did. Computed from the configured resolution, so
     flipping 0.75m/0.2m relabels the whole column to match what the module is now measuring. */
  const res = config.distance_resolution === "0.2m" ? 0.2 : 0.75;
  const fmt = (v) => String(parseFloat(v.toFixed(2)));

  const groups = [
    ["Movement", (live && live.gates && live.gates.move) || [], "gate_move_thresholds", Number(config.max_move_gate) || 0, HINTS.gate_move],
    ["Stillness", (live && live.gates && live.gates.still) || [], "gate_still_thresholds", Number(config.max_still_gate) || 0, HINTS.gate_still],
  ];

  const pct = (e, el) => {
    const r = el.getBoundingClientRect();
    return Math.max(0, Math.min(100, Math.round(((e.clientX - r.left) / r.width) * 100)));
  };

  const commit = (field, idx, val) => {
    const arr = Array.from({ length: NUM_GATES }, (_, i) => Number((config[field] || [])[i]) || 0);
    arr[idx] = val;
    write({ [field]: arr });
  };

  return (
    <div class="gates">
      {groups.map(([label, energies, field, maxGate, hint]) => (
        <div class="gates-g" key={field}>
          <div class="ctl-label">
            <span>{label}</span>
            <Hint text={hint} />
          </div>
          {Array.from({ length: NUM_GATES }, (_, i) => {
            const energy = Number(energies[i]) || 0;
            /* Beyond this group's furthest gate the module still measures but ignores the result, so
               the bar dims rather than disappears: the faint energy is the evidence for extending the
               range, and removing rows would make the slider look like it deletes data. The notch goes
               entirely - a threshold on an ignored gate is a dead control. */
            const off = i > maxGate;
            const held = drag && drag.field === field && drag.idx === i;
            const thr = held ? drag.val : Number((config[field] || [])[i]) || 0;
            return (
              <div class={`gate${off ? " off" : ""}`} key={i}>
                <span class="gate-n">{`${fmt(i * res)}\u2013${fmt((i + 1) * res)}m`}</span>
                <div
                  class={`gate-bar${off ? "" : " editable"}`}
                  onPointerDown={(e) => {
                    if (busy || off) return;
                    e.currentTarget.setPointerCapture(e.pointerId);
                    setDrag({ field, idx: i, val: pct(e, e.currentTarget) });
                  }}
                  onPointerMove={(e) => {
                    if (!held) return;
                    setDrag({ field, idx: i, val: pct(e, e.currentTarget) });
                  }}
                  onPointerUp={(e) => {
                    if (!held) return;
                    commit(field, i, pct(e, e.currentTarget));
                    setDrag(null);
                  }}
                  onPointerCancel={() => setDrag(null)}
                >
                  <div class="gate-fill" style={`width:${Math.min(100, energy)}%`} />
                  {!off && <div class="gate-thr" style={`left:${thr}%`} title={`Threshold ${thr}`} />}
                </div>
                {/* No number on the right any more - the owner found the flickering energy readout
                    distracting, and the bar itself is the reading. The notch's position is the
                    threshold feedback while dragging. */}
              </div>
            );
          })}
        </div>
      ))}
      <p class="dim sm">{TEXT.gate_thresholds_help}</p>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Settings                                                            */
/* ------------------------------------------------------------------ */

/** Ranges are the handler's own validation bounds, so the slider cannot produce a value the device
 *  will answer 400 to: detection_range 0-600, stability 0-10, timeout is a uint16 but is offered up to
 *  300s because a presence timeout beyond five minutes is not a setting anyone wants.
 *
 *  Nothing here is disabled while a write is in flight. Writes are optimistic and the queue serialises
 *  them, so there is nothing to protect - and greying every control out for each round trip is what
 *  made the whole card flash on every slider release. */
function Ld2450Settings({ config, write, preview }) {
  return (
    <>
      {/* Zero is "no cut-off", which on this module means its full 6m reach - so the label says the
          reach rather than "default", and the ring is not drawn because nothing is being cut off. The
          preview is what lets the ring track the thumb in real time; the device hears one write, on
          release. */}
      <Row label="Detection range" hint={HINTS.radar_range}>
        <Slider
          value={config.detection_range}
          min={0}
          max={600}
          step={10}
          format={(v) => (Number(v) === 0 ? "6 m" : `${Math.round(v)} cm`)}
          onPreview={(v) => preview({ detection_range: Math.round(v) })}
          onCommit={(v) => write({ detection_range: Math.round(v) })}
        />
      </Row>

      <Row label="Stability" hint={HINTS.radar_stability}>
        <Slider
          value={config.stability}
          min={0}
          max={10}
          step={1}
          format={(v) => `${Math.round(v)}`}
          onCommit={(v) => write({ stability: Math.round(v) })}
        />
      </Row>

      <Row label="Timeout" hint={HINTS.radar_timeout}>
        <Slider
          value={config.timeout}
          min={0}
          max={300}
          step={5}
          format={(v) => `${Math.round(v)} s`}
          onCommit={(v) => write({ timeout: Math.round(v) })}
        />
      </Row>

      <Row label="Multi-target" hint={HINTS.radar_multi}>
        <Toggle checked={config.multi_target} onChange={(v) => write({ multi_target: v })} />
      </Row>

      <Row label="Bluetooth" hint={HINTS.radar_bt}>
        <Toggle checked={config.bluetooth} onChange={(v) => write({ bluetooth: v })} />
      </Row>
    </>
  );
}

/** Undisabled during writes for the same reason as the LD2450's settings above. */
function Ld2410Settings({ config, write, preview }) {
  return (
    <>
      {/* Same control and same meaning as the LD2450's, so the same hint rather than a second wording. */}
      <Row label="Timeout" hint={HINTS.radar_timeout}>
        <Slider
          value={config.timeout}
          min={0}
          max={300}
          step={5}
          format={(v) => `${Math.round(v)} s`}
          onCommit={(v) => write({ timeout: Math.round(v) })}
        />
      </Row>

      {/* The previews are what dim the gate chart above in real time as these are dragged - the device
          still hears one write, on release. */}
      <Row label="Furthest movement gate" hint={HINTS.gate_max_move}>
        <Slider
          value={config.max_move_gate}
          min={0}
          max={8}
          step={1}
          format={(v) => `${Math.round(v)}`}
          onPreview={(v) => preview({ max_move_gate: Math.round(v) })}
          onCommit={(v) => write({ max_move_gate: Math.round(v) })}
        />
      </Row>

      <Row label="Furthest stillness gate" hint={HINTS.gate_max_still}>
        <Slider
          value={config.max_still_gate}
          min={0}
          max={8}
          step={1}
          format={(v) => `${Math.round(v)}`}
          onPreview={(v) => preview({ max_still_gate: Math.round(v) })}
          onCommit={(v) => write({ max_still_gate: Math.round(v) })}
        />
      </Row>

      <Row label="Bluetooth">
        <Toggle checked={config.bluetooth} onChange={(v) => write({ bluetooth: v })} />
      </Row>

      {/* A two-stop slider rather than a dropdown, at the owner's call - it sits among sliders, and a
          slider whose ends are the two choices reads as "less reach, finer" vs "more reach, coarser"
          where a dropdown read as a form field. The handler takes the two strings verbatim. */}
      <Row label="Distance resolution" hint={HINTS.radar_resolution}>
        <Slider
          value={config.distance_resolution === "0.2m" ? 0 : 1}
          min={0}
          max={1}
          step={1}
          format={(v) => (Math.round(v) === 0 ? "0.2 m" : "0.75 m")}
          onCommit={(v) => write({ distance_resolution: Math.round(v) === 0 ? "0.2m" : "0.75m" })}
        />
      </Row>
    </>
  );
}

/* ------------------------------------------------------------------ */
/* Status pills                                                        */
/* ------------------------------------------------------------------ */

/** Zone occupancy entities exist only for zones with three or more corners - the handler registers
 *  them per shape - so these keys are probed rather than assumed. Runtime names owned by C++ literals,
 *  like "Radar Target" on the home page. */
const ZONE_STATE_KEYS = ["text_sensor/Radar Zone 1", "text_sensor/Radar Zone 2", "text_sensor/Radar Zone 3"];

/**
 * The room at a glance, above the tuner.
 *
 * Presence and zone occupancy come from the firmware's own debounced entities over the event stream -
 * the same states Home Assistant automates on, so this row can never disagree with the lights. People,
 * distance and direction come from the live target poll instead, because the entities carry no
 * coordinates; they are the raw 4Hz readings, which is right for a row whose job is "what does it see
 * this instant". Direction is where the nearest person stands as the plot draws it: the middle pill
 * and the moving dot below it always agree.
 */
function StatusPills({ ctx, kind, live }) {
  const target = ctx.states["text_sensor/Radar Target"];
  // No .on ring on any of these: on the sensor chips that ring means "selected, editor open below",
  // and borrowing it here to mean "active" made the row look like some pills were pressed - the values
  // already say what is happening.
  const pills = [];

  if (kind === "ld2450") {
    const ts = realTargets(live);
    let near = null;
    for (const t of ts) {
      const d = Math.hypot(t.x, t.y);
      if (!near || d < near.d) near = { ...t, d };
    }
    // A 30-degree cone in the middle counts as "Ahead"; a pure x threshold would call someone standing
    // 30cm to the side "Left" at arm's length and "Ahead" across the room, which reads as flicker.
    const ang = near ? (Math.atan2(near.x, near.y) * 180) / Math.PI : 0;
    // One presence pill instead of presence-plus-zone: "Zone 2" beats "Yes" because where someone is
    // says more than that they are, and Yes/No beats Approaching/Still because that was the radar's
    // vocabulary, not the room's. The owner's call, replacing the separate Zone pill.
    const occupied = ZONE_STATE_KEYS.map((k, i) => ({ n: i + 1, s: ctx.states[k] })).filter(
      (z) => z.s && z.s.value && z.s.value !== "Clear"
    );
    const present = !!(target && target.value && target.value !== "Clear");
    pills.push(
      {
        l: "Presence",
        v: occupied.length ? `Zone ${occupied.map((z) => z.n).join(", ")}` : present ? "Yes" : "No",
      },
      { l: "People", v: String(ts.length) },
      { l: "Distance", v: near ? `${(near.d / 100).toFixed(1)} m` : "\u2014" },
      { l: "Direction", v: near ? (ang < -15 ? "Left" : ang > 15 ? "Right" : "Ahead") : "\u2014" }
    );
  } else {
    // The LD2410 has no zones, so its presence pill keeps the firmware's Moving/Still/Clear - there,
    // the kind of presence is the only detail the module offers.
    pills.push({
      l: "Presence",
      v: target && target.value ? PRESENCE[target.value] || target.value : "\u2014",
    });
    // The LD2410 reports one distance, not coordinates, so this is the whole story it can tell.
    const d = ctx.states["sensor/Radar Detection Distance"] || ctx.states["sensor/Radar Moving Distance"];
    const cm = d ? Number(d.value) : NaN;
    pills.push({ l: "Distance", v: Number.isFinite(cm) && cm > 0 ? `${(cm / 100).toFixed(1)} m` : "\u2014" });
  }

  return (
    <Card>
      <div class="pills">
        {pills.map((p) => (
          <span key={p.l} class="pill ro">
            <span class="pill-v">{p.v}</span>
            <span class="pill-l">{p.l}</span>
          </span>
        ))}
      </div>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* No module fitted                                                    */
/* ------------------------------------------------------------------ */

/**
 * A doorway, not a dead end - the owner's wording and the owner's product photo. The image is served
 * from the device's own flash (see /ui/no-sensor.webp in the component), so the card renders whole on
 * a network with no internet; the two links are the only part that needs the outside world, which is
 * honest, because so does buying a sensor.
 */
function NoSensor() {
  return (
    <Card title="Presence" icon={N_PRES}>
      <p class="sm">
        {TEXT.no_sensor_lead}
        <a href={TEXT.no_sensor_docs_url} target="_blank" rel="noreferrer">
          {TEXT.no_sensor_docs}
        </a>
        {TEXT.no_sensor_mid}
        <a href={TEXT.no_sensor_buy_url} target="_blank" rel="noreferrer">
          {TEXT.no_sensor_buy}
        </a>
      </p>
      <img class="promo" src="/ui/no-sensor.webp" alt="Satellite1 with a hidden mmWave radar sensor" />
    </Card>
  );
}

/* ------------------------------------------------------------------ */

export function Presence({ ctx }) {
  const { radarKind, radarConfig, radarLive, radarBusy, radarWrite, radarPreview, radarReboot } =
    useRadar(true);
  const [edit, setEdit] = useState(null); // { which: 0|1|2|"x", points: [...] } while editing

  if (!ctx.device) return <Missing what="the device" />;
  // Braced, not bare JSX text: a \u escape in a text child is literal characters, not an escape.
  if (radarKind === null) return <Card title="Presence" icon={N_PRES}>{"Looking for a radar module\u2026"}</Card>;
  if (radarKind === "none") return <NoSensor />;
  if (!radarConfig) return <Missing what="A radar module" />;

  const is2450 = radarKind === "ld2450";

  /** Opens a committed shape for editing - one function because there are two doors to it: the shape's
   *  button on the Zones card and the shape itself on the plot. */
  const openShape = (which) => {
    const src = which === "x" ? radarConfig.exclusion || [] : (radarConfig.zones || [])[which] || [];
    setEdit({ from: which, which, points: src.map((p) => ({ x: p.x, y: p.y })), sel: null, hist: [] });
  };

  return (
    <>
      <StatusPills ctx={ctx} kind={radarKind} live={radarLive} />

      {/* HINTS.presence lives here rather than on the sensor chip that links to this route. The chip has no
          room for an ⓘ that is not also a mis-tap risk at 50-80px wide, and this card is where someone who
          followed that chip lands - so it is the first thing they can ask a question of. The answer is
          worth having in a product with microphones in it: people assume presence is heard, not sensed. */}
      <Card
        title={is2450 ? "LD2450" : "LD2410"}
        icon={N_PRES}
        hint={HINTS.presence}
        right={
          // Lit only while this route is mounted, which is also exactly when polling happens - so the
          // dot is a true statement about the device rather than decoration.
          <span class="live-dot" title={`Polling every ${RADAR_LIVE_MS}ms`}>
            live
          </span>
        }
      >
        {is2450 ? (
          <>
            {/* Functional updates, not spreads of the closure: adding a corner fires onEdit and then
                onSelect in the same handler, and a spread of the stale `edit` in the second call would
                silently revert what the first one did - which shipped once as "cannot draw shapes at
                all", every tap adding a corner and un-adding it in the same frame. */}
            <Plot
              live={radarLive}
              config={radarConfig}
              edit={edit}
              onEdit={(points) => setEdit((e) => ({ ...e, points }))}
              onSelect={(sel) => setEdit((e) => ({ ...e, sel }))}
              onOpen={openShape}
              onHist={(snap) => setEdit((e) => ({ ...e, hist: [...e.hist, snap] }))}
            />
            {/* Zone controls live in the same card as the map they draw on - plot, instruction line,
                buttons, one surface for the whole task, per the owner. The separate Zones card is gone. */}
            <Zones
              config={radarConfig}
              write={radarWrite}
              busy={radarBusy}
              edit={edit}
              setEdit={setEdit}
              open={openShape}
            />
          </>
        ) : (
          <Gates live={radarLive} config={radarConfig} write={radarWrite} busy={radarBusy} />
        )}
      </Card>

      <Card title="Settings">
        {is2450 ? (
          <Ld2450Settings config={radarConfig} write={radarWrite} preview={radarPreview} />
        ) : (
          <Ld2410Settings config={radarConfig} write={radarWrite} preview={radarPreview} />
        )}

        {/* No save button: every change is live immediately and is written to permanent storage on its
            own, ten seconds after the last edit (see useRadar). The owner's call - a save button next
            to controls that visibly already worked read as either redundant or ominous. */}

        {radarConfig.reboot_required && (
          <Row
            label="Restart to apply"
            sub="Some of these settings only take effect after the device restarts."
          >
            <Btn onClick={radarReboot} disabled={radarBusy}>
              Restart device
            </Btn>
          </Row>
        )}
      </Card>

      {/* The legacy /radar_tuner link lived here during the parity-testing window; the owner approved
          the pages' retirement in September 2026 and the route 404s now. */}
    </>
  );
}
