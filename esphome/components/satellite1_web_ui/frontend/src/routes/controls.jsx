/**
 * The home page (route id "home"; this file keeps its historical name): the things a person glances
 * at and adjusts daily, all of which work with Home Assistant switched off.
 *
 * Media is not a card here anymore: it is the footer the shell renders on every route (media.jsx),
 * which is also where its card's logic went.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { HINTS, PRESENCE, TEXT } from "../copy.js";
import { entity, pathFor, PHASE, post, useVoice } from "../lib/device.js";
import { Arrow, Card, Chevron, Empty, Hint, Missing, N_CHAT, Row, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* Sensor pills, with calibration on the pill itself                   */
/* ------------------------------------------------------------------ */

/**
 * The offset entity publishes the correction, and the sensor publishes the already-corrected value,
 * because the offset is a filter on the sensor. So the raw reading the editor shows is a subtraction.
 * (The editor used to also show the corrected result on a "shows" row of its own; it went, per the
 * owner, because the pill right above the editor is that number, live, as the offset moves.)
 */
function Pill({ id, open, setOpen, label, value }) {
  const on = open === id;
  return (
    <button class={`pill${on ? " on" : ""}`} onClick={() => setOpen(on ? null : id)}>
      <span class="pill-v">{value}</span>
      <span class="pill-l">{label}</span>
      {/* Under the label, not beside it. This was a pencil next to the words, and spelled out
          "Temperature ✎" wants 66px against the 60px a quarter of a 360px screen gives it - so the glyph
          wrapped to its own line on three chips of four and stayed inline on the fourth, which read as a
          rendering fault rather than as an affordance. On its own line it cannot do that at any width.
          Points down at the editor it opens and flips up while it is open, so the chip says which of the
          four is responsible for the box underneath. */}
      <Chevron down={!on} up={on} cls="pill-c" />
    </button>
  );
}

function Editor({ title, hint, raw, unit, digits, step, offset, offsetPath, onClose }) {
  const box = useRef(null);

  const bump = (delta) => {
    const next = Number((offset + delta).toFixed(digits + 1));
    post(`${offsetPath}?value=${next}`);
  };

  /**
   * Dismissed by looking away from it, which is what replaced a Done button.
   *
   * There is nothing to confirm: every press has already been written to the device, so Done only ever
   * meant "stop showing me this". Capture, and pointerdown rather than click, for the reason Hint does
   * it - a tap that closes this should not also operate whatever sits underneath.
   *
   * The pills are excluded so the one that opened this can still toggle it shut without this handler and
   * the pill's own onClick both firing and cancelling out.
   */
  useEffect(() => {
    const away = (e) => {
      if (!box.current?.contains(e.target) && !e.target.closest?.(".pills")) onClose();
    };
    document.addEventListener("pointerdown", away, true);
    return () => document.removeEventListener("pointerdown", away, true);
  }, [onClose]);

  return (
    <div class="editor" ref={box}>
      {/* Two hints, because there are two different questions and they have different answers.
          The title's is about this particular reading - why it looks wrong, or what it is good for - and it
          is per sensor. The offset row's is how calibration works at all, which is the same sentence for
          every sensor and would be three-way duplication if it were folded into each one. */}
      <div class="row">
        <span class="grow strong">{title}</span>
        {hint && <Hint text={hint} />}
      </div>
      <div class="row sm">
        <span class="grow dim">sensor reads</span>
        <span class="num">
          {raw.toFixed(digits)}
          {unit}
        </span>
      </div>
      <div class="row sm">
        <span class="dim">offset</span>
        <Hint text={HINTS.calibrate} />
        <span class="grow" />
        <button class="btn sq" onClick={() => bump(-step)}>
          &minus;
        </button>
        <span class="num w44">
          {offset > 0 ? "+" : ""}
          {offset.toFixed(digits)}
        </span>
        <button class="btn sq" onClick={() => bump(step)}>
          +
        </button>
      </div>
    </div>
  );
}

const SENSORS = [
  { id: "temp", key: "temp", offsetKey: "temp_offset", label: "Temperature", title: "Temperature", unit: "\u00B0C", digits: 1, step: 0.1, hint: HINTS.temp },
  { id: "hum", key: "humidity", offsetKey: "humidity_offset", label: "Humidity", title: "Humidity", unit: " %", digits: 0, step: 1, hint: HINTS.humidity },
  { id: "lux", key: "lux", offsetKey: "lux_offset", label: "Light", title: "Ambient Light", unit: " lx", digits: 0, step: 5, hint: HINTS.lux },
];

function SensorPills({ ctx }) {
  const [open, setOpen] = useState(null);

  // The one paint before /api/sat1/state answers: pill-shaped shimmers hold the row's geometry so
  // the readings land in place rather than snapping a card into existence. Gated on the device
  // payload, not on the rows - a device with no sensors at all should show its truthful nothing,
  // not shimmer forever.
  if (!ctx.device) {
    return (
      <div class="pills">
        {[0, 1, 2, 3].map((i) => (
          <span key={i} class="pill skel" />
        ))}
      </div>
    );
  }

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
          <a
            class="pill"
            href="#/presence"
            // The firmware's full wording lives here, since the chip itself shows the short form.
            title={[presence.value, module?.value ? `${module.value} settings` : "Presence"].filter(Boolean).join(" \u2014 ")}
          >
            <span class="pill-v">{PRESENCE[presence.value] || presence.value || "\u2014"}</span>
            <span class="pill-l">Presence</span>
            {/* An arrow rather than the chevron its three neighbours carry, because this chip leaves the
                page instead of opening a box below it. Same position and same grey, so the row still reads
                as one set of chips; the shaft is the only thing that differs, and it is the thing that
                means "elsewhere". */}
            <Arrow cls="pill-c" />
          </a>
        )}
      </div>

      {openRow && openRow.offset !== null && (
        <Editor
          title={openRow.title}
          hint={openRow.hint}
          raw={openRow.value - openRow.offset}
          unit={openRow.unit}
          digits={openRow.digits}
          step={openRow.step}
          offset={openRow.offset}
          offsetPath={openRow.offsetPath}
          onClose={() => setOpen(null)}
        />
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
function Wheel({ hue, sat, onPick }) {
  const R = 58;
  const rad = (hue * Math.PI) / 180;
  const x = R + Math.cos(rad) * sat * (R - 8);
  const y = R + Math.sin(rad) * sat * (R - 8);

  const pick = (e) => {
    const b = e.currentTarget.getBoundingClientRect();
    const dx = e.clientX - b.left - R;
    const dy = e.clientY - b.top - R;
    const dist = Math.min(Math.hypot(dx, dy) / (R - 8), 1);
    let deg = (Math.atan2(dy, dx) * 180) / Math.PI;
    if (deg < 0) deg += 360;
    onPick(Math.round(deg), dist);
  };

  return (
    <div class="wheel-wrap">
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
      {/* Never disabled: every pick posts to turn_on with the colour, so touching the wheel on a
          dark ring lights it in that colour - one gesture instead of toggle-then-pick (owner,
          September 2026). */}
      <Wheel hue={hue} sat={sat} onPick={pick} />
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Timers and what was said                                            */
/* ------------------------------------------------------------------ */

const mmss = (s) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;

/** "10 min timer", "1 h 30 min timer" - the fallback label for a timer nobody named. */
const durLabel = (s) => {
  const h = Math.floor(s / 3600);
  const m = Math.round((s % 3600) / 60);
  const parts = [];
  if (h) parts.push(`${h} h`);
  if (m) parts.push(`${m} min`);
  if (!parts.length) parts.push(`${s} s`);
  return `${parts.join(" ")} timer`;
};

/**
 * Timers live on the device, not in Home Assistant: they keep counting and still ring with the
 * connection gone, which is the whole reason they are worth showing on a page that works offline.
 *
 * The card is absent when there are none rather than showing an empty state, because there is no way
 * to set one from here - they are created by voice - so an empty card would be an invitation to
 * press something that does not exist.
 *
 * Read-only by architecture, not by choice. The owner asked for a cancel button, and there is
 * nowhere to wire one: Home Assistant owns Assist timers, the native API only pushes their events
 * device-ward, and HA offers no action that cancels one (conversation.process carries no device id,
 * and timer intents are device-scoped). Voice is the interface - "cancel the timer" - which is what
 * HINTS.timers now says. If the protocol ever grows a cancel message, this card is where it lands.
 *
 * A timer named by voice shows its name; an unnamed one shows its set duration ("10 min timer")
 * rather than the word "Timer" directly under a card title that already says it.
 */
function Timers({ voice }) {
  const timers = voice?.timers || [];
  if (!timers.length) return null;

  return (
    <Card title={timers.length > 1 ? "Timers" : "Timer"} hint={HINTS.timers}>
      {timers.map((t) => (
        <div key={t.id} class="ctl">
          <div class="ctl-label">
            <span>{t.name || durLabel(t.total)}</span>
            {!t.active && <span class="ctl-sub">paused</span>}
          </div>
          <div class="ctl-body">
            <span class={`num lg${t.active ? "" : " dim"}`}>{mmss(t.left)}</span>
          </div>
        </div>
      ))}
    </Card>
  );
}

/**
 * What the assistant is doing, the last few exchanges so a misheard command is visible without
 * opening the log, and the two controls the owner brought back to it in the September 2026 rename
 * pass: Mute microphones (from the old Voice Input card) and Assistant volume (from Audio Output).
 * Both are assistant-shaped decisions - whether it can hear you, how loud it answers - and they see
 * daily use, which is what this page is for.
 *
 * "Assistant", not "Voice". The card's subject is the thing you talk to, and its rows now say so:
 * User for what was heard, Assist for what it answered (the labels were "heard"/"said", which read
 * as verbs about the device rather than as the two parties).
 *
 * The phase is the same global the LED ring animates from, so what this says and what the ring is
 * doing cannot disagree. "Not ready" is the honest reading with Home Assistant gone: the microphones
 * work, but there is nothing on the other end to answer.
 *
 * Present as soon as the device answers. A phase is always something, so the card is never empty -
 * and on a device that has not been spoken to since boot, one grey line is a better answer than no
 * card at all, which reads as the page having failed to load a section.
 */
function VoiceStatus({ ctx, voice }) {
  const phase = voice ? PHASE[voice.phase] : null;
  const lines = voice?.transcript || [];
  const mute = entity(ctx, "mute_mics");
  const vol = entity(ctx, "voice_override");

  // One transcript tab per wake word (owner request, September 2026): each line arrives tagged
  // with the word that initiated its exchange (`w`, new firmware; untagged lines show under every
  // tab so old firmware loses nothing). The last-uttered word is the active tab - and *stays*
  // following the newest exchange until the person picks another by hand, at which point their
  // choice holds until a newer word fires.
  const words = [];
  for (let k = lines.length - 1; k >= 0; k--) {
    const w = lines[k].w;
    if (w && !words.includes(w)) words.push(w);
  }
  const newest = words[0] || null;
  const [pick, setPick] = useState(null);
  const newestRef = useRef(newest);
  useEffect(() => {
    if (newest !== newestRef.current) {
      newestRef.current = newest;
      setPick(null); // a fresh firing reclaims the tab for its word
    }
  }, [newest]);
  const tab = pick && words.includes(pick) ? pick : newest;
  const shown = words.length > 1 ? lines.filter((l) => !l.w || l.w === tab) : lines;

  if (!phase && !lines.length && !mute && !vol) return null;

  return (
    <Card title="Assistant" icon={N_CHAT} right={phase && <span class={`dim xs${voice?.running ? " accent" : ""}`}>{phase}</span>}>
      {words.length > 1 && (
        <div class="tt-tabs" role="tablist">
          {words.map((w) => (
            <button key={w} role="tab" aria-selected={w === tab} class={`tt-tab${w === tab ? " on" : ""}`} onClick={() => setPick(w)}>
              &ldquo;{w === "stop" ? "Stop" : w}&rdquo;
            </button>
          ))}
        </div>
      )}
      {/* The transcript in a subcard of its own, per the owner - the same box the calibration editor
          draws, minus its accent border, which on that editor means "editing in progress" and here
          would promise an interaction the transcript does not have. The empty state sits inside the
          same box, so the card does not change shape the first time something is said. */}
      <div class="transcript">
        {shown.length ? (
          shown
            .slice()
            .reverse()
            .map((l, i) => (
              <p key={i} class={`utt${l.heard ? " heard" : ""}`}>
                <span class="utt-who">{l.heard ? "User:" : "Assist:"}</span>
                {l.text}
              </p>
            ))
        ) : (
          <Empty
            icon={
              <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linejoin="round" aria-hidden="true">
                <path d="M3 3h10a1.5 1.5 0 0 1 1.5 1.5V9A1.5 1.5 0 0 1 13 10.5H8.2L5 13.2v-2.7H3A1.5 1.5 0 0 1 1.5 9V4.5 A1.5 1.5 0 0 1 3 3Z" />
              </svg>
            }
            text={TEXT.nothing_said}
          />
        )}
      </div>

      {mute && (
        <Row label="Mute microphones" hint={HINTS.mute}>
          <Toggle
            checked={mute.value === true || mute.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "mute_mics", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}

      {/* Last in the card per the owner: the transcript is what you read, this is what you set. Keeps
          its name and hint - the hint is what tells it apart from Remote routing's "Remote TTS volume"
          now that the two no longer sit one card apart. */}
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

/* The Media card lived here until the media footer replaced it (owner, Sept 2026) - one media
   surface on every route instead of a card only home had. The whole of it, held-pause semantics
   included, moved to media.jsx in the shell. */

/* ------------------------------------------------------------------ */
/* Physical buttons                                                    */
/* ------------------------------------------------------------------ */

// The third field marks the chip that lights red instead of accent-blue: mute is the one button
// whose being active is a warning ("why is it not listening?") rather than a confirmation.
const BUTTONS = [
  ["btn_up", "Volume up"],
  ["btn_down", "Volume down"],
  ["btn_mute", "Mute", "warn"],
  ["btn_action", "Action"],
];

/**
 * Live button states, so someone can tell a stuck button from a dead one without a serial cable.
 * The action button's press type comes from the event entity, which is the only way to see that a
 * double press is being registered as two singles.
 *
 * This was on Diagnostics. It reads as diagnostics only if you think of the device as a computer; to
 * anyone holding one it answers "is this button working", about the same volume and mute the cards above
 * it control in software. Last on the route because it is the only card here you operate by putting the
 * phone down and touching the device.
 */
function Buttons({ ctx }) {
  const ev = entity(ctx, "action_button");
  const [lastPress, setLastPress] = useState(null);

  // event entities publish the press type in `event_type` and hold no state, so the last one seen
  // has to be latched here rather than read back.
  useEffect(() => {
    if (ev?.event_type) setLastPress(ev.event_type.replace(/_/g, " "));
  }, [ev?.event_type]);

  const rows = BUTTONS.map(([key, label, tone]) => [label, entity(ctx, key), tone]).filter(([, e]) => e);
  if (!rows.length) return null;

  return (
    <Card title="Buttons" right={lastPress && <span class="dim xs">last: {lastPress}</span>}>
      <div class="btnstates">
        {rows.map(([label, e, tone]) => (
          <span key={label} class={`bstate${tone ? ` ${tone}` : ""}${e.value ? " on" : ""}`}>
            {label}
          </span>
        ))}
      </div>
      <p class="dim xs">Press a button on the device; it lights up here.</p>
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
      {/* The settings that used to sit here - mute, the wake chime, sensitivity, the speaker channel
          and the assistant's own volume - are on Config as Voice Input and Audio Output. What is left
          on this route is what you look at rather than what you set once. Media went last: it lives
          in the footer now, visible from here and everywhere else. */}
      <VoiceStatus ctx={ctx} voice={voice} />
      <Timers voice={voice} />
      <Leds ctx={ctx} />
      <Buttons ctx={ctx} />
    </>
  );
}
