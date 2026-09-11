/**
 * Presence: what the radar sees, and the handful of settings that change what it sees.
 *
 * Built entirely on `satellite1_radar`'s existing JSON API rather than on entities. That is not a
 * shortcut - the radar's settings are not ESPHome entities at all, they live in the module's own
 * config and are read and written over `/api/v1/<module>/config`. This route is a second face on the
 * `/radar_tuner` page the device already serves, and the two share that API rather than agreeing by
 * accident.
 *
 * Two things the wireframe got wrong, corrected here against the handler:
 *
 * Engineering mode needs no explicit enter and exit. The LD2410 only reports gate energies while it is
 * on, and the handler arms it from the live poll and lets it lapse once polling stops - so mounting
 * this route is the entire protocol. A tab closed mid-session cannot leave the radar in engineering
 * mode, which an explicit exit POST could.
 *
 * "Reboot radar" reboots the whole device. `/api/v1/reboot` calls `App.safe_reboot()`. So it is not a
 * permanent button labelled as a radar action; it appears only when the device says a restart is
 * needed, and says what it will actually do.
 */
import { HINTS } from "../copy.js";
import { RADAR_LIVE_MS, useRadar } from "../lib/device.js";
import { Btn, Card, Missing, Row, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* The LD2450 plot                                                     */
/* ------------------------------------------------------------------ */

/** Half-width of the plotted area, in cm. The LD2450's field of view is about ±60°, so at its 600cm
 *  ceiling the lateral extent it can report is wider than the range itself. 400 keeps a person at the
 *  edge of a normal room on screen without shrinking the middle, where everyone actually is. */
const PLOT_HALF_W = 400;
const PLOT_DEPTH = 600;

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

/**
 * SVG in radar coordinates, not pixels: the viewBox is centimetres, so every zone polygon and target
 * is plotted with the numbers the device actually sent and nothing has to be scaled by hand. y grows
 * away from the device, which is why the whole group is flipped - the device sits at the bottom.
 */
function Plot({ live, config }) {
  const targets = realTargets(live);
  const zones = (config && config.zones) || [];
  const exclusion = (config && config.exclusion) || [];
  const range = config && config.detection_range ? config.detection_range : 0;

  return (
    <div class="plot">
      <svg viewBox={`${-PLOT_HALF_W} 0 ${PLOT_HALF_W * 2} ${PLOT_DEPTH}`} class="plot-svg">
        <g transform={`translate(0, ${PLOT_DEPTH}) scale(1, -1)`}>
          {/* Distance rings every 2m, which is how people describe a room. */}
          {[200, 400, 600].map((r) => (
            <circle key={r} class="plot-ring" cx="0" cy="0" r={r} />
          ))}
          <line class="plot-axis" x1={-PLOT_HALF_W} y1="0" x2={PLOT_HALF_W} y2="0" />
          <line class="plot-axis" x1="0" y1="0" x2="0" y2={PLOT_DEPTH} />

          {/* The configured cut-off, drawn so a target beyond it visibly sits outside. 0 means the
              module's own default rather than "no range", so nothing is drawn for it. */}
          {range > 0 && <circle class="plot-range" cx="0" cy="0" r={range} />}

          {zones.map((z, i) =>
            z && z.length > 2 ? (
              <polygon key={`z${i}`} class="plot-zone" points={polygonPoints(z)} />
            ) : null
          )}
          {exclusion.length > 2 && (
            <polygon class="plot-excl" points={polygonPoints(exclusion)} />
          )}

          {targets.map((t) => (
            <circle key={t.i} class="plot-target" cx={t.x} cy={t.y} r="16" />
          ))}
        </g>

        {/* Labelled on the rings, not along the bottom edge. The device sits at the bottom centre, so a
            row of ticks spread across the full width puts "0" at 400cm to the left - which is what the
            first version did, and it made the scale actively misleading. Outside the flipped group
            because text in it would be upside down; y is converted by hand instead. */}
        {[200, 400, 600].map((r) => (
          <text key={r} class="plot-tick" x="14" y={PLOT_DEPTH - r + 26}>
            {r / 100}m
          </text>
        ))}
      </svg>

      <p class="ctl-sub">
        {targets.length === 0
          ? "Nothing detected."
          : targets.map((t) => `${(t.y / 100).toFixed(1)}m at ${Math.round(t.x)}cm`).join(", ")}
      </p>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The LD2410 gate bars                                                */
/* ------------------------------------------------------------------ */

/**
 * Nine gates of movement and stillness energy, each a bar.
 *
 * Written from the handler's payload rather than from hardware: this device has an LD2450 fitted, so
 * the shape here is `{"gates":{"move":[...],"still":[...]}}` as `handle_ld2410_live_` emits it, and the
 * layout is unverified against a real LD2410. Flagged rather than presented as tested.
 */
function Gates({ live }) {
  const move = (live && live.gates && live.gates.move) || [];
  const still = (live && live.gates && live.gates.still) || [];
  if (move.length === 0 && still.length === 0) {
    return <p class="ctl-sub">Waiting for gate energies.</p>;
  }

  const rows = [
    ["Movement", move],
    ["Stillness", still],
  ];

  return (
    <div class="gates">
      {rows.map(([label, vals]) => (
        <div class="gates-g" key={label}>
          <div class="ctl-label">
            <span>{label}</span>
          </div>
          {vals.map((v, i) => (
            <div class="gate" key={i}>
              <span class="gate-n">{i}</span>
              <div class="gate-bar">
                <div class="gate-fill" style={`width:${Math.min(100, Number(v) || 0)}%`} />
              </div>
              <span class="gate-v">{Math.round(Number(v) || 0)}</span>
            </div>
          ))}
        </div>
      ))}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Settings                                                            */
/* ------------------------------------------------------------------ */

/** Ranges are the handler's own validation bounds, so the slider cannot produce a value the device
 *  will answer 400 to: detection_range 0-600, stability 0-10, timeout is a uint16 but is offered up to
 *  300s because a presence timeout beyond five minutes is not a setting anyone wants. */
function Ld2450Settings({ config, write, busy }) {
  return (
    <>
      <Row label="Detection range" hint={HINTS.radar_range}>
        <Slider
          value={config.detection_range}
          min={0}
          max={600}
          step={10}
          disabled={busy}
          format={(v) => (Number(v) === 0 ? "default" : `${Math.round(v)} cm`)}
          onCommit={(v) => write({ detection_range: Math.round(v) })}
        />
      </Row>

      <Row label="Stability" hint={HINTS.radar_stability}>
        <Slider
          value={config.stability}
          min={0}
          max={10}
          step={1}
          disabled={busy}
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
          disabled={busy}
          format={(v) => `${Math.round(v)} s`}
          onCommit={(v) => write({ timeout: Math.round(v) })}
        />
      </Row>

      <Row label="Multi-target" hint={HINTS.radar_multi}>
        <Toggle
          checked={config.multi_target}
          disabled={busy}
          onChange={(v) => write({ multi_target: v })}
        />
      </Row>

      <Row label="Bluetooth" hint={HINTS.radar_bt}>
        <Toggle checked={config.bluetooth} disabled={busy} onChange={(v) => write({ bluetooth: v })} />
      </Row>
    </>
  );
}

function Ld2410Settings({ config, write, busy }) {
  return (
    <>
      {/* Same control and same meaning as the LD2450's, so the same hint rather than a second wording. */}
      <Row label="Timeout" hint={HINTS.radar_timeout}>
        <Slider
          value={config.timeout}
          min={0}
          max={300}
          step={5}
          disabled={busy}
          format={(v) => `${Math.round(v)} s`}
          onCommit={(v) => write({ timeout: Math.round(v) })}
        />
      </Row>

      <Row label="Furthest movement gate">
        <Slider
          value={config.max_move_gate}
          min={0}
          max={8}
          step={1}
          disabled={busy}
          format={(v) => `${Math.round(v)}`}
          onCommit={(v) => write({ max_move_gate: Math.round(v) })}
        />
      </Row>

      <Row label="Furthest stillness gate">
        <Slider
          value={config.max_still_gate}
          min={0}
          max={8}
          step={1}
          disabled={busy}
          format={(v) => `${Math.round(v)}`}
          onCommit={(v) => write({ max_still_gate: Math.round(v) })}
        />
      </Row>

      <Row label="Bluetooth">
        <Toggle checked={config.bluetooth} disabled={busy} onChange={(v) => write({ bluetooth: v })} />
      </Row>

      <Row label="Distance resolution" sub={config.distance_resolution}>
        <span class="dim xs">Set from the radar tuner</span>
      </Row>
    </>
  );
}

/* ------------------------------------------------------------------ */

export function Presence({ ctx }) {
  const { radarKind, radarConfig, radarLive, radarBusy, radarWrite, radarSave, radarReboot } = useRadar(
    true
  );

  if (!ctx.device) return <Missing what="the device" />;
  // Braced, not bare JSX text: a \u escape in a text child is literal characters, not an escape.
  if (radarKind === null) return <Card title="Presence">{"Looking for a radar module\u2026"}</Card>;
  if (radarKind === "none" || !radarConfig) return <Missing what="A radar module" />;

  const is2450 = radarKind === "ld2450";

  return (
    <>
      {/* HINTS.presence lives here rather than on the sensor chip that links to this route. The chip has no
          room for an ⓘ that is not also a mis-tap risk at 50-80px wide, and this card is where someone who
          followed that chip lands - so it is the first thing they can ask a question of. The answer is
          worth having in a product with microphones in it: people assume presence is heard, not sensed. */}
      <Card
        title={is2450 ? "LD2450" : "LD2410"}
        hint={HINTS.presence}
        right={
          // Lit only while this route is mounted, which is also exactly when polling happens - so the
          // dot is a true statement about the device rather than decoration.
          <span class="live-dot" title={`Polling every ${RADAR_LIVE_MS}ms`}>
            live
          </span>
        }
      >
        {is2450 ? <Plot live={radarLive} config={radarConfig} /> : <Gates live={radarLive} />}
      </Card>

      <Card title="Settings">
        {is2450 ? (
          <Ld2450Settings config={radarConfig} write={radarWrite} busy={radarBusy} />
        ) : (
          <Ld2410Settings config={radarConfig} write={radarWrite} busy={radarBusy} />
        )}

        {/* Every change above is already live on the module; this is what survives a power cut. Two
            actions rather than one because they are genuinely different: saving persists, restarting is
            what some of the module's settings need before they take effect. */}
        <div class="row-actions">
          <Btn onClick={radarSave} disabled={radarBusy} solid>
            Save to flash
          </Btn>
        </div>

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

      {is2450 && (
        <Card title="Zones">
          <p class="ctl-sub">
            {(radarConfig.zones || []).some((z) => z && z.length > 2) || (radarConfig.exclusion || []).length > 2
              ? "Drawn on the plot above. Editing the shapes needs the full tuner."
              : "No zones set. The whole field of view counts as presence."}
          </p>
          {/* Read-only on purpose. A polygon editor is the one part of the tuner that genuinely needs
              dragging on a canvas, and a half-working one that saves the wrong shape is worse than a
              link to the page that already does it properly. */}
          <div class="row-actions">
            <a class="btn" href="/radar_tuner">
              Open the radar tuner
            </a>
          </div>
        </Card>
      )}
    </>
  );
}
