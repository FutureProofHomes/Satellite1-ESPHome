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
  const espTemp = entity(ctx, "esp_temp");
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
        {/* The only row here backed by an entity rather than by /api/sat1/state, so it is omitted on a
            build without the sensor rather than showing an em dash for a reading that will never come. */}
        {espTemp?.value != null && (
          <Fact
            label="ESP32 Temp"
            value={Number(espTemp.value).toFixed(1)}
            unit=" °C"
            hint={HINTS.esp_temp}
            tone={Number(espTemp.value) > 80 ? "err" : Number(espTemp.value) > 70 ? "warn" : null}
          />
        )}
        <Fact label="Uptime" value={uptime(d.uptime)} />
        <Fact label="Last restart" value={d.reset} hint={HINTS.reset} />
        {/* The address and MAC hang off this one rather than sitting in a strip of their own along the
            bottom of the card, which is where they were and where they described nothing in particular. */}
        <Fact
          label="Network Type"
          value={d.net === "ethernet" ? "Ethernet" : d.rssi != null ? `Wi-Fi ${d.rssi} dBm` : "Wi-Fi"}
          sub={
            <>
              <span class="num">{d.ip}</span>
              <span class="num">{d.mac}</span>
            </>
          }
        />
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
  // ESPHome's update entity publishes one of "UNKNOWN", "NO UPDATE", "UPDATE AVAILABLE", "INSTALLING".
  // Installing is kept on screen rather than treated as "nothing to offer": otherwise the panel vanishes
  // the instant you press Install, which reads as the press having failed at the one moment the device is
  // busy enough not to answer for a while.
  const available = upd && upd.state === "UPDATE AVAILABLE";
  const installing = upd && upd.state === "INSTALLING";

  return (
    <Card title="Firmware">
      <div class="facts">
        {/* Named for the product rather than for ESPHome's word for it. "Project" is what the manifest
            calls this, which means nothing to anyone who did not write the manifest. */}
        {/* "Up to date" only when that is true of all three states the updater can be in. Excluding
            `installing` matters: the panel below already says it is installing, and a version labelled up
            to date directly above it contradicts that. */}
        <Fact label="Sat1 firmware" value={d?.fw || "\u2014"} sub={upd && !available && !installing ? "Up to date" : null} />
        <Fact label="ESPHome Version" value={d?.esphome || "\u2014"} />
        {xmos && <Fact label="XMOS firmware" value={xmos.value || "\u2014"} hint={HINTS.xmos} />}
        <Fact label="Built" value={d?.built || "\u2014"} />
        {radarModule && <Fact label="Radar module" value={radarModule.value} />}
        {radarFw && <Fact label="Radar firmware" value={radarFw.value} />}
      </div>

      {/* Only when there is something to do. The row this replaced was always present and said "Up to date"
          next to the version already shown two lines above it, so the common case was a whole row restating
          a fact; that state is now a word under the version itself. When there IS an update, it gets the
          accent panel: the version is the thing worth reading, and pressing it is the thing worth doing, so
          the version is on the button rather than described next to one. */}
      {(available || installing) && (
        <div class="updbox">
          <div class="row">
            <span class="grow strong">{installing ? "Installing\u2026" : "Update available"}</span>
            {upd.release_url && (
              <a class="sm" href={upd.release_url} target="_blank" rel="noreferrer">
                Release notes
              </a>
            )}
          </div>
          <Btn solid disabled={installing} onClick={() => post(pathFor(ctx, "firmware", "install"))}>
            {installing ? "Do not cut power" : `Install ${upd.value}`}
          </Btn>
        </div>
      )}
      {beta && (
        <Row label="Beta updates" hint={HINTS.beta}>
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
/* Log                                                                */
/* ------------------------------------------------------------------ */

const LEVELS = { E: "err", C: "err", W: "warn", I: "info", D: "dim", V: "dim", VV: "dim" };

/** Splits a line around every case-insensitive occurrence of `term`, so matches can be wrapped in
 *  <mark> as real elements. Built this way rather than with innerHTML: log text is device output that
 *  routinely contains angle brackets and quotes, and one entity named with a "<" would be an injection
 *  point on the one page you go to when something is already wrong. */
function highlight(text, term) {
  if (!term) return text;
  const lower = text.toLowerCase();
  const needle = term.toLowerCase();
  const out = [];
  let at = 0;
  for (;;) {
    const hit = lower.indexOf(needle, at);
    if (hit === -1) break;
    if (hit > at) out.push(text.slice(at, hit));
    out.push(<mark>{text.slice(hit, hit + needle.length)}</mark>);
    at = hit + needle.length;
  }
  if (out.length === 0) return text;
  if (at < text.length) out.push(text.slice(at));
  return out;
}

function Log({ ctx }) {
  const { log, logSeq, pausedRef, clearLog } = ctx;
  const [paused, setPaused] = useState(false);
  const [minLevel, setMinLevel] = useState("D");
  const [filter, setFilter] = useState("");
  // Hide rather than Show: the invert is the reason the filter is useful on a device that logs a lot,
  // because the useful query is far more often "everything except the chatty component" than it is
  // "only this component".
  const [hide, setHide] = useState(false);
  const [mark, setMark] = useState("");
  const box = useRef(null);
  const atBottom = useRef(true);

  const order = ["VV", "V", "D", "I", "W", "E"];
  const floor = order.indexOf(minLevel);
  const needle = filter.trim().toLowerCase();
  const lines = log.filter((l) => {
    if (!(order.indexOf(l.lvl) >= floor || l.lvl === "C" || l.lvl === "?")) return false;
    if (!needle) return true;
    return l.text.toLowerCase().includes(needle) !== hide;
  });

  // Only when the view is already at the bottom. Scrolling to the newest line unconditionally means
  // that reading anything on a chatty device is impossible - you get yanked away mid-sentence.
  useEffect(() => {
    if (!paused && atBottom.current && box.current) box.current.scrollTop = box.current.scrollHeight;
  }, [logSeq, paused, needle, hide, minLevel]);

  useEffect(() => () => (pausedRef.current = false), [pausedRef]);

  const text = () => lines.map((l) => l.text).join("\n");

  /**
   * Copies what is on screen, filters and all, same as Save.
   *
   * The old body was `navigator.clipboard?.writeText(text())` and it copied nothing on any real device. The
   * Clipboard API is gated behind a secure context, this app is served over plain HTTP, so
   * navigator.clipboard is undefined and the ?. turned the whole thing into a no-op - no copy, no error,
   * and no feedback to notice the difference. It works when developing only because 127.0.0.1 gets a
   * secure-context exemption that 192.168.x.x does not.
   *
   * So: try the modern API for the day this is served over TLS, and fall back to execCommand, which is
   * deprecated but is not restricted by origin and is the only thing that works here. Confirmation is not
   * decoration - it is the thing that would have made the original failure visible.
   */
  const [copied, setCopied] = useState(false);

  const legacyCopy = (s) => {
    const ta = document.createElement("textarea");
    ta.value = s;
    ta.setAttribute("readonly", "");
    // Off-screen rather than hidden: display:none and visibility:hidden are not selectable, and the
    // selection is what execCommand copies. Fixed, so adding it cannot scroll the log.
    ta.style.cssText = "position:fixed;top:-1000px;opacity:0";
    document.body.appendChild(ta);
    ta.select();
    let ok = false;
    try {
      ok = document.execCommand("copy");
    } catch {
      ok = false;
    }
    ta.remove();
    return ok;
  };

  const copy = async () => {
    const s = text();
    let ok = false;
    try {
      await navigator.clipboard.writeText(s);
      ok = true;
    } catch {
      // Covers both the undefined navigator.clipboard on this origin and a permission refusal on a
      // secure one.
      ok = legacyCopy(s);
    }
    setCopied(ok);
  };

  // Back to "Copy" on its own. Also clears the timer if the card closes or the route changes while it is
  // still counting, which would otherwise set state on something no longer mounted.
  useEffect(() => {
    if (!copied) return;
    const t = setTimeout(() => setCopied(false), 1400);
    return () => clearTimeout(t);
  }, [copied]);

  // Downloads what is on screen, filters and all, because that is the thing worth sending to someone
  // else. Object URL revoked immediately; the click is synchronous.
  const download = () => {
    const url = URL.createObjectURL(new Blob([text()], { type: "text/plain" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = `satellite1-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-")}.log`;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <Card
      title="Log"
      collapsible
      name="log"
      defaultOpen
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
        </span>
      }
    >
      <div class="log-bar">
        <input
          class="inp sm grow"
          type="search"
          placeholder={"Filter\u2026"}
          value={filter}
          onInput={(e) => setFilter(e.currentTarget.value)}
        />
        <button
          class={`btn sm${hide ? " on" : ""}`}
          disabled={!needle}
          title={hide ? "Hiding matching lines" : "Showing only matching lines"}
          onClick={() => setHide(!hide)}
        >
          {hide ? "Hide" : "Show"}
        </button>
      </div>

      <div class="log-bar">
        <input
          class="inp sm grow"
          type="search"
          placeholder={"Highlight\u2026"}
          value={mark}
          onInput={(e) => setMark(e.currentTarget.value)}
        />
        <button class="btn sm" onClick={copy}>
          {copied ? TEXT.copied : "Copy"}
        </button>
        <button class="btn sm" onClick={download}>
          Save
        </button>
        <button class="btn sm" onClick={clearLog}>
          Clear
        </button>
      </div>

      <div
        class="log"
        ref={box}
        onScroll={(e) => {
          const el = e.currentTarget;
          // A few pixels of slack, because a fractional scrollHeight on a zoomed display never lands
          // exactly on the bottom and the panel would stop following.
          atBottom.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
        }}
      >
        {lines.length === 0 && (
          <p class="dim sm">
            {log.length === 0
              ? "Waiting for the device to say something."
              : "No lines match. Every line is filtered out."}
          </p>
        )}
        {lines.map((l, i) => (
          <div key={i} class={`ln t-${LEVELS[l.lvl] || "dim"}`}>
            {highlight(l.text, mark.trim())}
          </div>
        ))}
      </div>

      <p class="log-count dim xs">
        {lines.length === log.length
          ? `${log.length} lines`
          : `${lines.length} of ${log.length} lines`}
        {paused ? " \u00b7 paused" : ""}
      </p>
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
  // Satellite1::status_string() returns "v1.2.3" when the chip is talking, and one of "XMOS not
  // responding" / "Flashing Mode" / "" when it is not. Only the first is a version, so only the first goes
  // in the label - "Reflash XMOS Flashing Mode" would be worse than saying nothing.
  const xmosVer = entity(ctx, "xmos_firmware")?.value;
  const flashLabel = /^v\d/.test(xmosVer || "") ? `Reflash XMOS ${xmosVer}` : "Reflash XMOS";

  return (
    <>
      {/* Named for the part rather than described, because the row below it talks about flashing firmware,
          and at that point you need to know which chip you are aiming at. HINTS.xmos opens with "The audio
          chip", so the ⓘ carries what the title used to say. */}
      {(xmosReset || xmosFlash) && (
        <Card title="XMOS" collapsible name="xmos" hint={HINTS.xmos}>
          {xmosReset && (
            <Row label="Restart XMOS">
              <Btn onClick={() => post(xmosReset)}>Restart</Btn>
            </Row>
          )}
          {/* The version is in the label rather than left to the Firmware card, because this is the one
              place where knowing what is on the chip decides whether to press the button. */}
          {xmosFlash && (
            <Row label={flashLabel} hint={HINTS.xmos_flash}>
              <Confirm label="Reflash" confirmLabel="Reflash now" onConfirm={() => post(xmosFlash)} />
            </Row>
          )}
          {/* "Erase its firmware" was here, wired to the erase_xmos_flash button. It is gone from the app
              on purpose: it leaves the audio chip blank, which takes the microphones, the speaker and the
              wake word with it, and the only route back is the Reflash row above - which needs the very
              chip it just erased to be talking. Reflash already overwrites, so erase-then-flash was never
              a step anyone needed. The ESPHome button still exists for a bench recovery over the API. */}
        </Card>
      )}

      <Card title="Sat1 Device" collapsible name="maint">
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
      {/* Buttons moved to the foot of Controls. It is the one card here that answers "does the hardware
          respond to me", which is a question about the thing you are holding rather than about its
          internals - and it belongs beside the volume and mute controls it duplicates in hardware. */}
      <Log ctx={ctx} />
      <Maintenance ctx={ctx} />
    </>
  );
}
