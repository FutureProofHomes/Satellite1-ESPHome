/**
 * Diagnostics: what to read, and what to attach to a support request, when something is wrong.
 *
 * Almost everything here comes from GET /api/sat1/state rather than from entities. These are
 * read-only numbers that exist for one panel on one route, and turning each into an ESPHome sensor
 * would cost RAM, a polling component, and a place in the /events stream sent to every open tab
 * whether or not anyone is looking at this page.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { HINTS, TEXT } from "../copy.js";
import { entity, pathFor, post } from "../lib/device.js";
import { Btn, Card, Confirm, Fact, Missing, Row, Toggle } from "../ui.jsx";

const kb = (n) => `${Math.round(n / 1024)} KB`;
const mb = (n) => `${(n / 1048576).toFixed(1)} MB`;

const uptime = (s) => {
  if (s == null) return "\u2014";
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (d) return `${d}d ${h}h`;
  if (h) return `${h}h ${m}m`;
  return `${m}m ${s % 60}s`;
};

/* ------------------------------------------------------------------ */
/* Device                                                              */
/* ------------------------------------------------------------------ */

function Device({ ctx }) {
  const d = ctx.device;
  if (!d) return <Card title="Device">{ctx.deviceError ? <p class="t-err sm">{ctx.deviceError}</p> : <p class="dim sm">Reading&hellip;</p>}</Card>;

  // Internal RAM is what runs out first, so it gets the warning colours; PSRAM is plentiful enough
  // that colouring it would only train people to ignore the colour.
  const heapPct = (d.heap.free / d.heap.total) * 100;
  const heapTone = heapPct < 10 ? "err" : heapPct < 20 ? "warn" : null;
  const loopTone = d.loop_ms > 500 ? "err" : d.loop_ms > 150 ? "warn" : null;

  return (
    <Card title="Device">
      <div class="facts">
        <Fact label="Internal RAM free" value={kb(d.heap.free)} unit={` of ${kb(d.heap.total)}`} hint={HINTS.heap} tone={heapTone} />
        <Fact label="PSRAM free" value={mb(d.psram.free)} unit={` of ${mb(d.psram.total)}`} hint={HINTS.psram} />
        <Fact label="Longest loop" value={d.loop_ms} unit=" ms" hint={HINTS.loop} tone={loopTone} />
        <Fact label="Uptime" value={uptime(d.uptime)} />
        <Fact label="Last restart" value={d.reset} hint={HINTS.reset} />
        <Fact label="Network" value={d.net === "ethernet" ? "Ethernet" : d.rssi != null ? `Wi-Fi ${d.rssi} dBm` : "Wi-Fi"} />
      </div>
      <div class="ids">
        <span class="mono">{d.ip}</span>
        <span class="mono dim">{d.mac}</span>
        {d.psram.installed > 0 && <span class="dim xs">{mb(d.psram.installed)} PSRAM installed</span>}
      </div>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Firmware                                                           */
/* ------------------------------------------------------------------ */

function Firmware({ ctx }) {
  const d = ctx.device;
  const upd = entity(ctx, "firmware");
  const beta = entity(ctx, "beta_firmware");
  const xmos = entity(ctx, "xmos_firmware");
  const radarModule = entity(ctx, "radar_module");
  // Component-owned name, like Radar Target on Controls - registered from a C++ literal, with no
  // config id to route through the entity map.
  const radarFw = ctx.states["text_sensor/Radar Firmware"];
  const available = upd && upd.state === "UPDATE AVAILABLE";

  return (
    <Card title="Firmware">
      <div class="facts">
        <Fact label="Project" value={d?.fw || "\u2014"} />
        <Fact label="ESPHome" value={d?.esphome || "\u2014"} />
        {xmos && <Fact label="Audio chip" value={xmos.value || "\u2014"} hint={HINTS.xmos} />}
        <Fact label="Built" value={d?.built || "\u2014"} />
        {radarModule && <Fact label="Radar module" value={radarModule.value} />}
        {radarFw && <Fact label="Radar firmware" value={radarFw.value} />}
      </div>

      {upd && (
        <Row label="Update" sub={available ? `${upd.value} available` : "Up to date"}>
          {available ? (
            <Btn solid onClick={() => post(pathFor(ctx, "firmware", "install"))}>
              Install {upd.value}
            </Btn>
          ) : (
            <span class="dim sm">{upd.current_version || upd.value}</span>
          )}
        </Row>
      )}
      {available && upd.release_url && (
        <p class="sm">
          <a href={upd.release_url} target="_blank" rel="noreferrer">
            Release notes
          </a>
        </p>
      )}
      {beta && (
        <Row label="Pre-release firmware" hint={HINTS.beta}>
          <Toggle
            checked={beta.value === true || beta.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "beta_firmware", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Buttons                                                            */
/* ------------------------------------------------------------------ */

const BUTTONS = [
  ["btn_up", "Volume up"],
  ["btn_down", "Volume down"],
  ["btn_mute", "Mute"],
  ["btn_action", "Action"],
];

/**
 * Live button states, so someone can tell a stuck button from a dead one without a serial cable.
 * The action button's press type comes from the event entity, which is the only way to see that a
 * double press is being registered as two singles.
 */
function Buttons({ ctx }) {
  const ev = entity(ctx, "action_button");
  const [lastPress, setLastPress] = useState(null);

  // event entities publish the press type in `event_type` and hold no state, so the last one seen
  // has to be latched here rather than read back.
  useEffect(() => {
    if (ev?.event_type) setLastPress(ev.event_type.replace(/_/g, " "));
  }, [ev?.event_type]);

  const rows = BUTTONS.map(([key, label]) => [label, entity(ctx, key)]).filter(([, e]) => e);
  if (!rows.length) return null;

  return (
    <Card title="Buttons" right={lastPress && <span class="dim xs">last: {lastPress}</span>}>
      <div class="btnstates">
        {rows.map(([label, e]) => (
          <span key={label} class={`bstate${e.value ? " on" : ""}`}>
            {label}
          </span>
        ))}
      </div>
      <p class="dim xs">Press a button on the device; it lights up here.</p>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Log                                                                */
/* ------------------------------------------------------------------ */

const LEVELS = { E: "err", C: "err", W: "warn", I: "info", D: "dim", V: "dim", VV: "dim" };

function Log({ ctx }) {
  const { log, logSeq, pausedRef } = ctx;
  const [paused, setPaused] = useState(false);
  const [minLevel, setMinLevel] = useState("D");
  const box = useRef(null);

  const order = ["VV", "V", "D", "I", "W", "E"];
  const floor = order.indexOf(minLevel);
  const lines = log.filter((l) => order.indexOf(l.lvl) >= floor || l.lvl === "C" || l.lvl === "?");

  useEffect(() => {
    if (!paused && box.current) box.current.scrollTop = box.current.scrollHeight;
  }, [logSeq, paused]);

  // Pause is a flag on the shared stream, not local state, so leaving the page while paused would
  // otherwise stop the ring filling for the rest of the session.
  useEffect(() => () => (pausedRef.current = false), [pausedRef]);

  const copy = () => navigator.clipboard?.writeText(lines.map((l) => l.text).join("\n"));

  return (
    <Card
      title="Log"
      hint={HINTS.log}
      right={
        <span class="row gap">
          <select class="sel sm" value={minLevel} onChange={(e) => setMinLevel(e.currentTarget.value)}>
            <option value="VV">everything</option>
            <option value="D">debug</option>
            <option value="I">info</option>
            <option value="W">warnings</option>
            <option value="E">errors</option>
          </select>
          <button
            class="btn sm"
            onClick={() => {
              const next = !paused;
              setPaused(next);
              // Paused means stop accepting lines, not just stop scrolling: the ring is 1000 lines
              // and a noisy boot would otherwise push the thing being read straight out of it.
              pausedRef.current = next;
            }}
          >
            {paused ? "Resume" : "Pause"}
          </button>
          <button class="btn sm" onClick={copy}>
            Copy
          </button>
        </span>
      }
    >
      <div class="log" ref={box}>
        {lines.length === 0 && <p class="dim sm">Waiting for the device to say something.</p>}
        {lines.map((l, i) => (
          <div key={i} class={`ln t-${LEVELS[l.lvl] || "dim"}`}>
            {l.text}
          </div>
        ))}
      </div>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Maintenance                                                        */
/* ------------------------------------------------------------------ */

function Maintenance({ ctx }) {
  const restart = pathFor(ctx, "restart", "press");
  const safe = pathFor(ctx, "safe_mode", "press");
  const factory = pathFor(ctx, "factory_reset", "press");
  const xmosReset = pathFor(ctx, "xmos_reset", "press");
  const xmosFlash = pathFor(ctx, "xmos_flash", "press");
  const xmosErase = pathFor(ctx, "xmos_erase", "press");

  return (
    <>
      {(xmosReset || xmosFlash || xmosErase) && (
        <Card title="Audio chip" hint={HINTS.xmos}>
          {xmosReset && (
            <Row label="Restart the audio chip">
              <Btn onClick={() => post(xmosReset)}>Restart</Btn>
            </Row>
          )}
          {xmosFlash && (
            <Row label="Reflash its firmware" hint={HINTS.xmos_flash}>
              <Confirm label="Reflash" confirmLabel="Reflash now" onConfirm={() => post(xmosFlash)} />
            </Row>
          )}
          {xmosErase && (
            <Row label="Erase its firmware" hint={HINTS.xmos_erase}>
              <Confirm label="Erase" confirmLabel="Erase it" danger onConfirm={() => post(xmosErase)} />
            </Row>
          )}
        </Card>
      )}

      <Card title="This device">
        {restart && (
          <Row label="Restart">
            <Btn onClick={() => post(restart)}>Restart</Btn>
          </Row>
        )}
        {safe && (
          <Row label="Safe mode" hint={HINTS.safe_mode}>
            <Confirm label="Safe mode" confirmLabel="Restart into safe mode" onConfirm={() => post(safe)} />
          </Row>
        )}
        {factory && (
          <Row label="Factory reset" hint={HINTS.factory_reset}>
            <Confirm label="Factory reset" confirmLabel="Erase everything" danger onConfirm={() => post(factory)} />
          </Row>
        )}
        {!restart && !safe && !factory && <Missing what="Device maintenance" />}
      </Card>
    </>
  );
}

/* ------------------------------------------------------------------ */

export function Diagnostics({ ctx }) {
  return (
    <>
      {ctx.device && ctx.device.ha === false && <p class="banner">{TEXT.ha_disconnected_detail}</p>}
      <Device ctx={ctx} />
      <Firmware ctx={ctx} />
      <Buttons ctx={ctx} />
      <Log ctx={ctx} />
      <Maintenance ctx={ctx} />
    </>
  );
}
