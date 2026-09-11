/**
 * Controls: the things a person adjusts, all of which work with Home Assistant switched off.
 *
 * Media is absent rather than greyed out. It needs the Home Assistant data layer, and a permanently
 * disabled card teaches people the app is broken.
 */
import { useState } from "preact/hooks";

import { HINTS } from "../copy.js";
import { entity, pathFor, PHASE, post, useVoice } from "../lib/device.js";
import { Card, Missing, Row, Select, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* Sensor pills, with calibration on the pill itself                   */
/* ------------------------------------------------------------------ */

/**
 * The offset entity publishes the correction, and the sensor publishes the already-corrected value,
 * because the offset is a filter on the sensor. So the raw reading is a subtraction, and the editor
 * can show all three figures - what the hardware reads, the correction, what the device reports -
 * which is the only presentation in which "shows" being wrong is obviously the offset's fault.
 */
function Pill({ id, open, setOpen, label, value }) {
  return (
    <button class={`pill${open === id ? " on" : ""}`} onClick={() => setOpen(open === id ? null : id)}>
      <span class={`pill-v${String(value).length > 8 ? " sm" : ""}`}>{value}</span>
      <span class="pill-l">
        {label}
        <span class="accent"> &#9998;</span>
      </span>
    </button>
  );
}

function Editor({ title, raw, unit, digits, step, offset, offsetPath, onClose }) {
  const bump = (delta) => {
    const next = Number((offset + delta).toFixed(digits + 1));
    post(`${offsetPath}?value=${next}`);
  };

  return (
    <div class="editor">
      <div class="row">
        <span class="grow strong">{title}</span>
        <button class="link" onClick={onClose}>
          Done
        </button>
      </div>
      <div class="row sm">
        <span class="grow dim">sensor reads</span>
        <span class="mono">
          {raw.toFixed(digits)}
          {unit}
        </span>
      </div>
      <div class="row sm">
        <span class="grow dim">offset</span>
        <button class="btn sq" onClick={() => bump(-step)}>
          &minus;
        </button>
        <span class="mono w44">
          {offset > 0 ? "+" : ""}
          {offset.toFixed(digits)}
        </span>
        <button class="btn sq" onClick={() => bump(step)}>
          +
        </button>
      </div>
      <hr />
      <div class="row">
        <span class="grow dim sm">shows</span>
        <span class="accent strong">
          {(raw + offset).toFixed(digits)}
          {unit}
        </span>
      </div>
    </div>
  );
}

const SENSORS = [
  { id: "temp", key: "temp", offsetKey: "temp_offset", label: "Temp", title: "Temperature", unit: "\u00B0C", digits: 1, step: 0.1, hint: HINTS.temp },
  { id: "hum", key: "humidity", offsetKey: "humidity_offset", label: "Humid", title: "Humidity", unit: " %", digits: 0, step: 1, hint: HINTS.humidity },
  { id: "lux", key: "lux", offsetKey: "lux_offset", label: "Light", title: "Ambient Light", unit: " lx", digits: 0, step: 5, hint: HINTS.lux },
];

function SensorPills({ ctx }) {
  const [open, setOpen] = useState(null);

  const rows = SENSORS.map((s) => {
    const sensor = entity(ctx, s.key);
    if (!sensor) return null;
    const off = entity(ctx, s.offsetKey);
    // A sensor with no calibration entity is still worth showing; it just is not editable.
    const offset = off ? Number(off.value) : null;
    return { ...s, value: Number(sensor.value), offset, offsetPath: pathFor(ctx, s.offsetKey, "set") };
  }).filter(Boolean);

  // Referenced by id rather than through the entity map, and deliberately so. satellite1_radar
  // registers this at runtime from a C++ string literal that both the LD2450 and LD2410 handlers
  // share, so the name is owned by code rather than by anyone's YAML - which is the thing the map
  // exists to protect against - and there is no config id to point the map at anyway.
  const presence = ctx.states["text_sensor/Radar Target"];
  const module = entity(ctx, "radar_module");
  const openRow = rows.find((r) => r.id === open);

  return (
    <div>
      <div class="pills">
        {rows.map((r) => (
          <Pill
            key={r.id}
            id={r.id}
            open={r.offset === null ? null : open}
            setOpen={r.offset === null ? () => {} : setOpen}
            label={r.label}
            value={`${r.value.toFixed(r.digits)}${r.unit}`}
          />
        ))}
        {presence && (
          // Now the Presence route rather than the legacy /radar_tuner page. Still a link rather than a
          // button, so a long-press still offers "open in new tab" - someone comparing the plot against
          // what they can see in the room wants both at once.
          <a class="pill" href="#/presence" title={module?.value ? `${module.value} settings` : "Presence"}>
            <span class={`pill-v${String(presence.value).length > 8 ? " sm" : ""}`}>{presence.value || "\u2014"}</span>
            <span class="pill-l">
              Presence<span class="accent"> &#8594;</span>
            </span>
          </a>
        )}
      </div>

      {openRow && openRow.offset !== null && (
        <Editor
          title={openRow.title}
          raw={openRow.value - openRow.offset}
          unit={openRow.unit}
          digits={openRow.digits}
          step={openRow.step}
          offset={openRow.offset}
          offsetPath={openRow.offsetPath}
          onClose={() => setOpen(null)}
        />
      )}

      {!open && (
        <p class="pills-foot">
          Tap a reading to calibrate it.
          {presence ? ` Presence opens the ${module?.value || "radar"} tuner.` : ""}
        </p>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Colour wheel                                                        */
/* ------------------------------------------------------------------ */

const hsvToRgb = (h, s) => {
  const f = (n) => {
    const k = (n + h / 60) % 6;
    return Math.round(255 * (1 - s * Math.max(0, Math.min(k, 4 - k, 1))));
  };
  return [f(5), f(3), f(1)];
};

const rgbToHs = (r, g, b) => {
  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);
  if (max === 0) return [0, 0];
  const d = max - min;
  let h = 0;
  if (d !== 0) {
    if (max === r) h = ((g - b) / d) % 6;
    else if (max === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  return [Math.round(h), d / max];
};

/**
 * Hue around the circumference, saturation from the centre out.
 *
 * A row of preset swatches looks tidier and then fails the one request people actually have, which
 * is a particular colour. Value is deliberately not on the wheel - brightness is a separate control
 * on the light, and folding it in makes both harder to set.
 */
function Wheel({ hue, sat, disabled, onPick }) {
  const R = 58;
  const rad = (hue * Math.PI) / 180;
  const x = R + Math.cos(rad) * sat * (R - 8);
  const y = R + Math.sin(rad) * sat * (R - 8);

  const pick = (e) => {
    if (disabled) return;
    const b = e.currentTarget.getBoundingClientRect();
    const dx = e.clientX - b.left - R;
    const dy = e.clientY - b.top - R;
    const dist = Math.min(Math.hypot(dx, dy) / (R - 8), 1);
    let deg = (Math.atan2(dy, dx) * 180) / Math.PI;
    if (deg < 0) deg += 360;
    onPick(Math.round(deg), dist);
  };

  return (
    <div class={`wheel-wrap${disabled ? " off" : ""}`}>
      <div class="wheel" onPointerDown={pick}>
        <span class="wheel-dot" style={{ left: `${x - 6}px`, top: `${y - 6}px`, background: `hsl(${hue} ${Math.round(sat * 100)}% 50%)` }} />
      </div>
    </div>
  );
}

function Leds({ ctx }) {
  const light = entity(ctx, "ring");
  if (!light) return <Missing what="The LED ring" />;

  const on = light.state === "ON";
  const c = light.color || { r: 255, g: 255, b: 255 };
  const [hue, sat] = rgbToHs(c.r, c.g, c.b);
  const bright = Math.round(((light.brightness ?? 255) / 255) * 100);

  const write = (params) => {
    const base = pathFor(ctx, "ring", "turn_on");
    post(`${base}?${new URLSearchParams(params)}`);
  };

  const pick = (h, s) => {
    const [r, g, b] = hsvToRgb(h, s);
    write({ r, g, b });
  };

  return (
    <Card title="LED ring" hint={HINTS.led_ring}>
      <Row label="Power">
        <Toggle
          checked={on}
          onChange={(v) => post(pathFor(ctx, "ring", v ? "turn_on" : "turn_off"))}
        />
      </Row>
      <Row label="Brightness">
        <Slider
          value={bright}
          min={1}
          max={100}
          step={1}
          disabled={!on}
          format={(v) => `${v}%`}
          onCommit={(v) => write({ brightness: Math.round((v / 100) * 255) })}
        />
      </Row>
      <Wheel hue={hue} sat={sat} disabled={!on} onPick={pick} />
      <p class="dim xs center">
        Tap the wheel to set a colour. Hue around the edge, whiter towards the middle.
      </p>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Voice and audio                                                     */
/* ------------------------------------------------------------------ */

function Voice({ ctx, voice }) {
  const mute = entity(ctx, "mute_mics");
  const chime = entity(ctx, "wake_sound");
  const sens = entity(ctx, "wake_sensitivity");
  const vol = entity(ctx, "voice_override");

  // The phase is the same global the LED ring animates from, so what the card says and what the ring
  // is doing cannot disagree. "Not ready" is the honest reading with Home Assistant gone: the
  // microphones work, but there is nothing on the other end to answer.
  const phase = voice ? PHASE[voice.phase] : null;

  return (
    <Card title="Voice" right={phase && <span class={`dim xs${voice.running ? " accent" : ""}`}>{phase}</span>}>
      {mute && (
        <Row label="Mute microphones" hint={HINTS.mute}>
          <Toggle
            checked={mute.value === true || mute.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "mute_mics", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}
      {chime && (
        <Row label="Wake chime" hint={HINTS.wake_sound}>
          <Toggle
            checked={chime.value === true || chime.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "wake_sound", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}
      {sens && (
        <Row label="Wake word sensitivity" hint={HINTS.wake_sensitivity}>
          <Select
            value={sens.value}
            options={sens.option}
            onChange={(v) => post(`${pathFor(ctx, "wake_sensitivity", "set")}?option=${encodeURIComponent(v)}`)}
          />
        </Row>
      )}
      {vol && (
        <Row label="Assistant volume" hint={HINTS.voice_override}>
          <Slider
            value={Number(vol.value)}
            min={Number(vol.min_value ?? 0)}
            max={Number(vol.max_value ?? 100)}
            step={Number(vol.step ?? 1)}
            format={(v) => (v === 0 ? "follow media" : `${v}%`)}
            onCommit={(v) => post(`${pathFor(ctx, "voice_override", "set")}?value=${v}`)}
          />
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Timers and what was said                                            */
/* ------------------------------------------------------------------ */

const mmss = (s) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;

/**
 * Timers live on the device, not in Home Assistant: they keep counting and still ring with the
 * connection gone, which is the whole reason they are worth showing on a page that works offline.
 *
 * The card is absent when there are none rather than showing an empty state, because there is no way
 * to set one from here - they are created by voice - so an empty card would be an invitation to
 * press something that does not exist.
 */
function Timers({ voice }) {
  const timers = voice?.timers || [];
  if (!timers.length) return null;

  return (
    <Card title={timers.length > 1 ? "Timers" : "Timer"} hint={HINTS.timers}>
      {timers.map((t) => (
        <div key={t.id} class="ctl">
          <div class="ctl-label">
            <span>{t.name || "Timer"}</span>
            {!t.active && <span class="ctl-sub">paused</span>}
          </div>
          <div class="ctl-body">
            <span class={`mono lg${t.active ? "" : " dim"}`}>{mmss(t.left)}</span>
          </div>
        </div>
      ))}
    </Card>
  );
}

/**
 * The last few exchanges, so a misheard command is visible without opening the log.
 *
 * Only rendered once there is something to show. On a device that has not been spoken to since boot
 * an empty panel would read as a fault rather than as an absence.
 */
function Transcript({ voice }) {
  const lines = voice?.transcript || [];
  if (!lines.length) return null;

  return (
    <Card title="Recently">
      {lines
        .slice()
        .reverse()
        .map((l, i) => (
          <p key={i} class={`utt${l.heard ? " heard" : ""}`}>
            <span class="utt-who">{l.heard ? "heard" : "said"}</span>
            {l.text}
          </p>
        ))}
    </Card>
  );
}

function Speaker({ ctx }) {
  const chan = entity(ctx, "speaker_channel");
  const lineOut = entity(ctx, "line_out");
  if (!chan && !lineOut) return null;

  return (
    <Card title="Speaker">
      {chan && (
        <Row label="Channel" hint={HINTS.speaker_channel}>
          <Select
            value={chan.value}
            options={chan.option}
            onChange={(v) => post(`${pathFor(ctx, "speaker_channel", "set")}?option=${encodeURIComponent(v)}`)}
          />
        </Row>
      )}
      {lineOut && (
        <Row label="Line out">
          <span class="dim">{lineOut.value ? "Connected" : "Nothing plugged in"}</span>
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */

export function Controls({ ctx }) {
  const voice = useVoice(true);

  return (
    <>
      <Card>
        <SensorPills ctx={ctx} />
      </Card>
      <Voice ctx={ctx} voice={voice} />
      <Timers voice={voice} />
      <Transcript voice={voice} />
      <Speaker ctx={ctx} />
      <Leds ctx={ctx} />
    </>
  );
}
