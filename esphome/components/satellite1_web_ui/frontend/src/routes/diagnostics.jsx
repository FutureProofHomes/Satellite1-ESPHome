/**
 * Diagnostics: what to read, and what to attach to a support request, when something is wrong.
 *
 * Almost everything here comes from GET /api/sat1/state rather than from entities. These are
 * read-only numbers that exist for one panel on one route, and turning each into an ESPHome sensor
 * would cost RAM, a polling component, and a place in the /events stream sent to every open tab
 * whether or not anyone is looking at this page.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { CONFIRM, HINTS, TEXT } from "../copy.js";
import { changePassword, logoutAll, mdnsLooksBroken, qrSignInLink, signInLink } from "../lib/auth.js";
import { BASE, entity, entityPath, pathFor, post, request, requestJson } from "../lib/device.js";
import { qrSvgPath } from "../lib/qr.js";
import { takeIntent, toast } from "../lib/toast.js";
import { Btn, Card, Chevron, Confirm, Fact, Missing, N_AUDIO, N_DIAG, ni, Row, Select, Slider, Toggle } from "../ui.jsx";

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
  const usb = entity(ctx, "usb_power");
  // The entity publishes the FUSB302B's contract string ("3.25A (max) @ 20V") - the same string
  // Home Assistant shows, kept intact there because automations may read it. This row re-words it
  // to the owner's format (September 2026): "20V @ 3.25A~" as the headline - voltage first because
  // it decides the amp's gain mode, the trailing ~ carrying what "(max)" meant (the charger's
  // ceiling, not a live draw) - and the wattage spelled out underneath, because watts are how
  // people know their chargers. A string the regex does not recognise shows raw rather than
  // hiding, so a future contract format degrades to the old rendering instead of a missing row.
  const usbFact = (() => {
    const raw = usb?.value;
    if (raw == null || raw === "") return null;
    const m = /^([\d.]+)A \(max\) @ (\d+)V$/.exec(raw);
    if (!m) return { value: raw };
    const amps = Number(m[1]);
    const volts = Number(m[2]);
    const watts = volts * amps;
    return {
      value: `${volts}V @ ${amps}A~`,
      sub: `${Number.isInteger(watts) ? watts : watts.toFixed(1)} watts`,
    };
  })();
  // Follows the home page's temperature unit toggle, so the app never shows mixed units. The tone
  // thresholds stay computed on the °C value the entity publishes.
  const unitF = entity(ctx, "temp_unit_f");
  const isF = !!unitF && (unitF.value === true || unitF.state === "ON");
  if (!d) return <Card title="Device" icon={N_DIAG}>{ctx.deviceError ? <p class="t-err sm">{ctx.deviceError}</p> : <p class="dim sm">Reading&hellip;</p>}</Card>;

  // Internal RAM is what runs out first, so it gets the warning colours; PSRAM is plentiful enough
  // that colouring it would only train people to ignore the colour.
  const heapPct = (d.heap.free / d.heap.total) * 100;
  const heapTone = heapPct < 10 ? "err" : heapPct < 20 ? "warn" : null;
  const loopTone = d.loop_ms > 500 ? "err" : d.loop_ms > 150 ? "warn" : null;

  return (
    <Card title="Device" icon={N_DIAG}>
      <div class="facts">
        <Fact label="Internal RAM free" value={kb(d.heap.free)} unit={` of ${kb(d.heap.total)}`} hint={HINTS.heap} tone={heapTone} />
        <Fact label="PSRAM free" value={mb(d.psram.free)} unit={` of ${mb(d.psram.total)}`} hint={HINTS.psram} />
        <Fact label="Longest loop" value={d.loop_ms} unit=" ms" hint={HINTS.loop} tone={loopTone} />
        {/* The only row here backed by an entity rather than by /api/sat1/state, so it is omitted on a
            build without the sensor rather than showing an em dash for a reading that will never come. */}
        {espTemp?.value != null && (
          <Fact
            label="ESP32 Temp"
            value={(isF ? (Number(espTemp.value) * 9) / 5 + 32 : Number(espTemp.value)).toFixed(1)}
            unit={isF ? " °F" : " °C"}
            hint={HINTS.esp_temp}
            tone={Number(espTemp.value) > 80 ? "err" : Number(espTemp.value) > 70 ? "warn" : null}
          />
        )}
        <Fact label="Uptime" value={uptime(d.uptime)} />
        <Fact label="Last restart" value={d.reset} hint={HINTS.reset} />
        {/* Entity-backed and conditional like ESP32 Temp above, and placed by Last Restart on purpose:
            that row's hint already points a "Brownout" at the power supply, and this is the row that
            says which supply the device actually negotiated. Published on every powered outcome
            including the plain-5V timeout - so an absent value means a build without the PD sensor,
            not a 5V supply. */}
        {usbFact && <Fact label="USB-C Power Supply" value={usbFact.value} sub={usbFact.sub} hint={HINTS.usb_power} />}
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
/* Speaker amplifier                                                   */
/* ------------------------------------------------------------------ */

/**
 * The amplifier's live state and its wiring, one card, right under Device (owner call, September
 * 2026 - it lived on the Audio route for a day): the Power gain mode row is decided by the USB-C
 * Power Supply reading in the card above, and the two belong on one screen. It absorbed the Audio
 * route's old Audio Output card on the way - the Channel select and the Line out row moved in
 * verbatim under three amplifier rows.
 *
 * The live readings come from GET /api/sat1/amp, polled every 2 s while the card is on screen: an
 * audio_dac is not an entity, so they cannot ride /events, and a dedicated poll keeps them off the
 * state payload every tab receives. The analog gain slider is different - a real number entity (a
 * setting to persist, not a reading), through the entity map like every other slider.
 *
 * Voice Volume Override stays on the home page's Assistant card - the owner wants it under the
 * transcript it sets the level for - and the Digital volume row here is deliberately read-only:
 * it shows the level the firmware computed FROM those sliders, so someone chasing "why is it
 * quiet" can see what the amp is actually being fed.
 */
function SpeakerAmp({ ctx }) {
  const [amp, setAmp] = useState(null);

  // The useVoice pattern: poll while mounted, stop when not. A null answer (404 on a build without
  // the TAS2780, or a dropped poll) leaves the last reading standing rather than blanking rows -
  // the next poll corrects it, and the stream-lost banner covers a device that is actually gone.
  useEffect(() => {
    let live = true;
    let timer = null;
    const tick = async () => {
      try {
        const json = await requestJson("/api/sat1/amp");
        if (json && live) setAmp(json);
      } catch {
        // Same as the voice poll: not worth surfacing, the next one is two seconds away.
      }
      if (live) timer = setTimeout(tick, 2000);
    };
    tick();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
  }, []);

  const chan = entity(ctx, "speaker_channel");
  const lineOut = entity(ctx, "line_out");
  // Existence from the key table (the payload arrives a beat later over /events), value from the
  // payload - the same split config.jsx's useEntity wrapper draws for the Audio route's sliders.
  const gainExists = !!ctx.device?.e?.amp_gain;
  const gain = entity(ctx, "amp_gain");
  if (!amp && !chan && !lineOut && !gainExists) return null;

  /* The mode, worded rather than numbered. `pending` first: during the ~100 ms activation window
     the reported mode is the bootstrap's, and "measuring" is the truth. `active` next, because a
     line-out selection or an XMOS flash shuts the amp down and the stale mode would lie. The two
     modes this firmware selects get names; anything else shows raw so a future firmware that uses
     PWR_MODE 1 or 3 reaches the screen without an app release. */
  const mode = !amp
    ? null
    : amp.pending
      ? { v: "Measuring\u2026", d: "Sampling the power supply" }
      : !amp.active
        ? { v: "Off", d: "Line out selected or amplifier shut down" }
        : amp.mode === 2
          ? { v: "High gain", d: "Running from the USB-PD supply" }
          : amp.mode === 0
            ? { v: "Low gain", d: "Running from the 5 V rail" }
            : { v: `PWR_MODE ${amp.mode}`, d: null };

  return (
    <Card title="Speaker amplifier" icon={N_AUDIO} hint={HINTS.speaker_amp}>
      {mode && (
        <Row label="Power gain mode" hint={HINTS.amp_mode}>
          <span class="dim">
            {mode.v}
            {mode.d && <span class="xs"> &middot; {mode.d}</span>}
          </span>
        </Row>
      )}
      {amp && (
        <Row label="Digital volume" hint={HINTS.amp_dvc}>
          <span class="dim">{amp.muted ? "Muted" : `${amp.dvc}%`}</span>
        </Row>
      )}
      {gainExists && (
        <Row label="Analog gain" hint={HINTS.amp_gain}>
          <Slider
            value={gain ? Number(gain.value ?? gain.state) : 0}
            min={gain?.min_value ?? 0}
            max={gain?.max_value ?? 20}
            step={gain?.step ?? 1}
            // The index travels; dBV is what the readout speaks (11-21 dBV in half-dBV steps). The
            // factory default (index 8, 15 dBV) is the notch on the track the drag snaps to - the
            // "(default)" tag the readout used to carry, said in geometry instead of words (owner
            // call, September 2026). The hint still names it for anyone who wants it spelled out.
            snap={8}
            format={(v) => `${(11 + v / 2).toFixed(1)} dBV`}
            onCommit={(v) => post(pathFor(ctx, "amp_gain", "set", { value: v }))}
          />
        </Row>
      )}
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
/* Crash Reports                                                       */
/* ------------------------------------------------------------------ */

/**
 * The crash history the firmware harvested across reboots: the NVS ring of records, the pre-crash
 * log tail, and the downloadable core dump. Fetched once on mount and again whenever the state
 * poll's crash count moves (a new crash can only appear with a reboot, so "moves" is rare), plus
 * after an erase. The tail is its own lazy fetch behind the Show button - it is 4KB that most
 * visits never read. Renders nothing on a build without the crash_report component, whose absence
 * makes the endpoint 404 - the same contract as every optional card.
 */
function CrashCard({ ctx, reveal }) {
  const [data, setData] = useState(null);
  const [tail, setTail] = useState(null);
  const [showLog, setShowLog] = useState(false);
  // Which record's backtrace is open, -1 none. One at a time: the addresses are for copying, and
  // two open walls of hex help nobody.
  const [openBt, setOpenBt] = useState(-1);
  const count = ctx.device?.crash;

  const load = () =>
    requestJson("/api/sat1/crash")
      .then((d) => d && setData(d))
      .catch(() => {});
  useEffect(() => {
    load();
  }, [count]);

  if (!data) return null;

  /* When it happened, best first: a real wall-clock stamp when the flight recorder knew the time;
     a browser-side estimate for the most recent crash (now minus the current uptime is the moment
     this session began, which is the moment the crash ended); and uptime-plus-distance when the
     crash is older than the last power cycle. */
  const when = (r) => {
    if (r.epoch) return new Date(r.epoch * 1000).toLocaleString();
    if (data.boot - r.boot === 1 && ctx.device?.uptime)
      return `\u2248 ${new Date(Date.now() - ctx.device.uptime * 1000).toLocaleString()}`;
    const n = data.boot - r.boot;
    const t = n === 1 ? TEXT.crash_restart_ago : TEXT.crash_restarts_ago;
    return t.replace("%1", uptime(r.up)).replace("%2", String(n));
  };

  const toggleLog = () => {
    const next = !showLog;
    setShowLog(next);
    if (next && tail === null)
      request("/api/sat1/crash/log")
        .then((r) => setTail(r.ok ? r.text : ""))
        .catch(() => setTail(""));
  };

  /* Fetched and saved as a Blob rather than navigated to as a link, for the same reason Export
     Logs works this way: Chromium-family browsers interpose a "this file may have been tampered
     with" interstitial on any download delivered over plain HTTP, which this device always is. A
     Blob minted in-page is a local file to the browser, so the save is silent. Raw fetch rather
     than the request() helper, whose .text would decode the zip's bytes as UTF-8 and corrupt them;
     the session cookie rides fetch's same-origin default. */
  const downloadDump = () =>
    fetch(`${BASE}/api/sat1/crash/dump.bin`)
      .then((r) => (r.ok ? r.blob() : Promise.reject(new Error(String(r.status)))))
      .then((b) => {
        const url = URL.createObjectURL(b);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${ctx.device?.name || "satellite1"}-coredump.bin`;
        a.click();
        URL.revokeObjectURL(url);
      })
      .catch(() => {});

  return (
    <Card title={TEXT.crash_title} collapsible name="crash" defaultOpen forceOpen={reveal} hint={HINTS.crash} id="card-crash">
      {data.records.length === 0 && <p class="dim sm">{TEXT.crash_none}</p>}

      {/* Each record is a stacked block in the transcript subcard box, not a label/control row: a
          record has no control to right-align, and the first build that used the row shape sent
          the date column crawling over the label on a phone. */}
      {data.records.map((r, k) => (
        <div class="transcript" key={k}>
          <div class="row">
            {/* The panic text ("StoreProhibited", "assert failed: ...") when the dump yielded one,
                else the reset reason - the same string the Device card's Last Restart shows. */}
            <span class="strong grow">{r.txt || r.rs}</span>
            <span class="dim xs">{TEXT.crash_ran.replace("%s", uptime(r.up))}</span>
          </div>
          <p class="dim sm">{when(r)}</p>
          {r.task && (
            <p class="dim sm num">
              {r.task} &middot; cause {r.cause} &middot; PC {r.pc}
              {r.vaddr && r.vaddr !== "0x00000000" ? ` \u00b7 addr ${r.vaddr}` : ""}
            </p>
          )}
          {r.bt && r.bt.length > 0 && (
            <>
              <button class="btn sm" aria-expanded={openBt === k} onClick={() => setOpenBt(openBt === k ? -1 : k)}>
                {r.cor ? TEXT.crash_bt_corrupt : TEXT.crash_bt} <Chevron down={openBt === k} cls="caret-s" />
              </button>
              {openBt === k && <p class="dim xs num">{r.bt.join(" ")}</p>}
            </>
          )}
        </div>
      ))}

      {data.log > 0 && (
        <>
          <Row label={TEXT.crash_log_row} hint={HINTS.crash_log}>
            <button class="btn sm" aria-expanded={showLog} onClick={toggleLog}>
              {showLog ? TEXT.crash_log_hide : TEXT.crash_log_show} <Chevron down={showLog} cls="caret-s" />
            </button>
          </Row>
          {showLog && (
            <div class="log" style="white-space:pre-wrap">
              {tail === null ? "\u2026" : tail || TEXT.crash_log_none}
            </div>
          )}
        </>
      )}

      {data.dump > 0 && (
        <Row label={TEXT.crash_dump_row} hint={HINTS.crash_dump}>
          <Btn onClick={downloadDump}>{TEXT.crash_download}</Btn>
        </Row>
      )}

      {(data.dump > 0 || data.records.length > 0) && (
        <Row label={TEXT.crash_erase_row} hint={HINTS.crash_erase}>
          <Confirm
            label={TEXT.crash_erase}
            title={CONFIRM.crash_erase.t}
            body={CONFIRM.crash_erase.b}
            confirmLabel={TEXT.crash_erase}
            danger
            onConfirm={() =>
              post("/api/sat1/crash/erase").then(() => {
                setTail(null);
                setShowLog(false);
                load();
              })
            }
          />
        </Row>
      )}

      {!data.part && <p class="dim xs">{TEXT.crash_no_part}</p>}
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
  // Component-owned name, like Radar Target on the home page - registered from a C++ literal, with no
  // config id to route through the entity map.
  const radarFw = ctx.states["text_sensor/Radar Firmware"];
  // ESPHome's update entity publishes one of "UNKNOWN", "NO UPDATE", "UPDATE AVAILABLE", "INSTALLING".
  // Installing is kept on screen rather than treated as "nothing to offer": otherwise the panel vanishes
  // the instant you press Install, which reads as the press having failed at the one moment the device is
  // busy enough not to answer for a while.
  const available = upd && upd.state === "UPDATE AVAILABLE";
  const installing = upd && upd.state === "INSTALLING";

  return (
    <Card title="Firmware" id="card-firmware">
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
          <Confirm
            solid
            disabled={installing}
            label={installing ? "Do not cut power" : `Install ${upd.value}`}
            title={CONFIRM.update.t}
            body={CONFIRM.update.b}
            confirmLabel={`Install ${upd.value}`}
            onConfirm={() => post(pathFor(ctx, "firmware", "install"))}
          />
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

/* Every level its own colour, matching the palette ESPHome's own console uses so nobody has to learn
   a second scheme: red errors, orange warnings, green info, cyan-blue debug. Verbose alone stays grey -
   it is the chatter you filter out, and colouring it would leave nothing dim to compare against. */
const LEVELS = { E: "err", C: "err", W: "warn", I: "ok", D: "dbg", V: "dim", VV: "dim" };

/** Wall-clock HH:MM:SS from the line's arrival time, padded so the column never wobbles. */
const stamp = (at) => {
  const d = new Date(at);
  const p = (n) => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
};

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

/** The level menu's text colours, same palette as the lines below so the menu doubles as the legend. */
const LEVEL_COLOR = { VV: "var(--fg-4)", D: "var(--accent)", I: "var(--ok)", W: "var(--warn)", E: "var(--err)" };

const LEVEL_OPTS = [
  ["VV", "everything"],
  ["D", "debug"],
  ["I", "info"],
  ["W", "warnings"],
  ["E", "errors"],
];

/**
 * The level filter, as its own little menu rather than a native <select>.
 *
 * It was a <select> with coloured options, and that lasted one review: Safari's native popup ignores
 * option styling entirely, so on the owner's machine the "colour coded" menu was plain text. A drawn
 * menu obeys its stylesheet everywhere, and it is also what lets the funnel sit inside the control -
 * the owner's other ask - instead of orbiting it as a separate glyph.
 */
function LevelMenu({ value, onChange }) {
  const [open, setOpen] = useState(false);
  const wrap = useRef(null);

  useEffect(() => {
    if (!open) return;
    const away = (e) => {
      if (!wrap.current?.contains(e.target)) setOpen(false);
    };
    const esc = (e) => e.key === "Escape" && setOpen(false);
    // Capture, for the reason Hint does it: a tap that closes this should not also press whatever
    // it landed on.
    document.addEventListener("pointerdown", away, true);
    document.addEventListener("keydown", esc);
    return () => {
      document.removeEventListener("pointerdown", away, true);
      document.removeEventListener("keydown", esc);
    };
  }, [open]);

  return (
    <span class="lvlmenu" ref={wrap}>
      <button class="btn sm lvlbtn" aria-expanded={open} onClick={() => setOpen(!open)}>
        {/* The funnel says "what you are reading is filtered". Inside the control, per the owner. */}
        <svg class="funnel" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round" aria-hidden="true">
          <path d="M1.2 1.8h9.6L7.4 6.4v3.4l-2.8 1V6.4L1.2 1.8z" />
        </svg>
        <span style={{ color: LEVEL_COLOR[value] }}>{LEVEL_OPTS.find(([v]) => v === value)[1]}</span>
        <Chevron down={open} cls="caret-s" />
      </button>
      {open && (
        <div class="lvlpop" role="menu">
          {LEVEL_OPTS.map(([v, l]) => (
            <button
              key={v}
              role="menuitemradio"
              aria-checked={v === value}
              class={`lvlopt${v === value ? " on" : ""}`}
              style={{ color: LEVEL_COLOR[v] }}
              onClick={() => {
                onChange(v);
                setOpen(false);
              }}
            >
              {l}
            </button>
          ))}
        </div>
      )}
    </span>
  );
}

function Log({ ctx, intent }) {
  const { log, logSeq, pausedRef, logWatch } = ctx;
  const [paused, setPaused] = useState(false);

  // Register as the log's reader for exactly as long as this card is mounted. While no reader is
  // registered the ring still fills - that is what lets this card show the recent past the moment
  // someone navigates here - but arriving lines stop costing the rest of the app renders. (This is
  // also what silences log toasts while the card is on screen - see onLogAlert in device.js.)
  useEffect(() => logWatch(), []);
  // A toast's intent seeds the level filter, so "Tap for the device log" on a warning lands on the
  // warnings, not on the debug firehose that buried them.
  const [minLevel, setMinLevel] = useState(intent?.level || "D");
  // The line the intent points at, held while its flash runs. {at, text} is the identity the ring
  // stamps in device.js, so the match is exact when the line still exists.
  const [hl, setHl] = useState(intent?.line || null);
  useEffect(() => {
    // The late-intent case: a toast tapped while this route was already standing (the write-failed
    // toast can fire here; log toasts cannot - they are suppressed while this card watches. A
    // history row from the drawer can carry a line here, though.)
    if (intent?.level) setMinLevel(intent.level);
    if (intent?.line) setHl(intent.line);
  }, [intent]);
  const [filter, setFilter] = useState("");
  // The Show/Hide invert that lived beside the filter field is gone at the owner's request; the
  // filter is match-only now.
  const [mark, setMark] = useState("");
  const box = useRef(null);
  const atBottom = useRef(true);

  const order = ["VV", "V", "D", "I", "W", "E"];
  const floor = order.indexOf(minLevel);
  const needle = filter.trim().toLowerCase();
  const lines = log.filter((l) => {
    if (!(order.indexOf(l.lvl) >= floor || l.lvl === "C" || l.lvl === "?")) return false;
    if (!needle) return true;
    return l.text.toLowerCase().includes(needle);
  });

  // Only when the view is already at the bottom. Scrolling to the newest line unconditionally means
  // that reading anything on a chatty device is impossible - you get yanked away mid-sentence.
  useEffect(() => {
    if (!paused && atBottom.current && box.current) box.current.scrollTop = box.current.scrollHeight;
  }, [logSeq, paused, needle, minLevel]);

  useEffect(() => () => (pausedRef.current = false), [pausedRef]);

  // The line reveal: once the flagged row is in the DOM (the render above marks it .hl), scroll
  // the panel so it sits mid-view, then let the flash run out and clear. Scrolling the panel by
  // hand rather than scrollIntoView because the page itself is already travelling to the card -
  // two smooth scrolls fighting over one viewport is seasickness. atBottom is parked so the
  // live stream cannot yank the view to the newest line mid-flash. When the line has churned out
  // of the 1000-line ring (or was never captured while paused), the honest fallback is the
  // freshest view at the intended level - the flash is skipped rather than lighting a stranger.
  useEffect(() => {
    if (!hl) return undefined;
    const el = box.current?.querySelector(".ln.hl");
    if (el && box.current) {
      box.current.scrollTop = Math.max(0, el.offsetTop - box.current.clientHeight / 2);
      atBottom.current = false;
    } else if (box.current) {
      box.current.scrollTop = box.current.scrollHeight;
    }
    const t = setTimeout(() => setHl(null), 2600);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [hl]);

  // The timestamp goes into the dump too: a support request's "when did it happen" deserves the same
  // answer the screen gives.
  const text = () => lines.map((l) => `[${stamp(l.at)}] ${l.text}`).join("\n");

  // The Copy button that lived down in the highlight bar is gone at the owner's request, and it is no
  // loss: this page is served over plain HTTP, where navigator.clipboard does not exist, so the button
  // only ever worked through a deprecated execCommand fallback. Export is the one way out now, and what
  // it saves is also what copy produced - the lines on screen, filters and all.
  //
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
      title="Logs"
      collapsible
      name="log"
      defaultOpen
      // Keyed so an intent arriving while the card stands collapsed can still reveal it: forceOpen
      // is an initial-render decision, and the remount is what makes it one again.
      key={intent ? "revealed" : "log"}
      forceOpen={!!intent}
      id="card-log"
      hint={HINTS.log}
      right={
        <span class="row gap">
          <LevelMenu value={minLevel} onChange={setMinLevel} />
          {/* Up here from the highlight bar below, per the owner. Still the filtered view. */}
          <button class="btn sm" onClick={download}>
            Export Logs
          </button>
        </span>
      }
    >
      {/* Two bare fields now. The Show/Hide invert and the Clear button went at the owner's request -
          both inputs are type="search", so their built-in ✕ empties them. Note what Clear actually did
          before it went: it emptied the log buffer itself, not a field. There is no in-page way to do
          that any more; a reload starts the ring fresh, which is the same gesture with one more step. */}
      <div class="log-bar">
        <input
          class="inp sm grow"
          type="search"
          placeholder={"Filter\u2026"}
          value={filter}
          onInput={(e) => setFilter(e.currentTarget.value)}
        />
      </div>

      <div class="log-bar">
        <input
          class="inp sm grow"
          type="search"
          placeholder={"Highlight\u2026"}
          value={mark}
          onInput={(e) => setMark(e.currentTarget.value)}
        />
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
          <div key={i} class={`ln t-${LEVELS[l.lvl] || "dim"}${hl && l.at === hl.at && l.text === hl.text ? " hl" : ""}`}>
            {/* The time keeps its own muted colour on every level, so a wall of red errors still has a
                readable clock running down its margin. */}
            <span class="ln-t">{stamp(l.at)}</span>
            {highlight(l.text, mark.trim())}
          </div>
        ))}
      </div>

      {/* The line count and the stream toggle share the card's last row, per the owner. The toggle
          reads as the state it is in rather than the action it offers - Live (accent) while lines
          flow, Paused once they are held - because "is this thing streaming?" is the question it
          answers at a glance. (The count used to append "· paused" too; with the button stating it
          two words apart, that was the same fact twice.) Collapsing the card takes the button with
          it, but the paused flag lives in this component, so the stream picks up exactly where the
          toggle left it. */}
      <div class="log-foot">
        <p class="log-count dim xs">
          {lines.length === log.length
            ? `${log.length} lines`
            : `${lines.length} of ${log.length} lines`}
        </p>
        <button
          class={`btn sm${paused ? "" : " on"}`}
          onClick={() => {
            const next = !paused;
            setPaused(next);
            // Paused means stop accepting lines, not just stop scrolling: the ring is 1000 lines
            // and a noisy boot would otherwise push the thing being read straight out of it.
            pausedRef.current = next;
          }}
        >
          {paused ? "Paused" : "Live"}
        </button>
      </div>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Maintenance                                                        */
/* ------------------------------------------------------------------ */

/**
 * The radar module's own recovery card, between the XMOS and ESP32 ones - same "part + what the
 * card does to it" naming, titled for whichever module is actually fitted.
 *
 * Gated on the radar_module text sensor, which satellite1_radar only sets once detection succeeds -
 * the same moment its handlers register the "Radar Restart" / "Radar Factory Reset" buttons, so the
 * gate and the endpoints agree by construction. Those buttons are runtime entities created from C++
 * literals shared by both handlers (no YAML id, so nothing for the entity map to point at); they
 * are addressed by their literal names through entityPath, exactly as the Firmware card reads
 * "text_sensor/Radar Firmware". Absent on the stock-component ld2410/ld2450 variant builds, where
 * radar_detected_text is removed - the card simply never renders there.
 */
function RadarRecovery({ ctx }) {
  const mod = entity(ctx, "radar_module")?.value;
  if (mod !== "LD2410" && mod !== "LD2450") return null;
  return (
    <Card title={`${mod} Recovery`} collapsible name="radar-rec" hint={HINTS.radar_recovery}>
      <Row label="Restart radar">
        <Confirm
          label="Restart"
          title={CONFIRM.radar_restart.t}
          body={CONFIRM.radar_restart.b}
          confirmLabel="Restart radar"
          onConfirm={() => post(entityPath("button/Radar Restart", "press"))}
        />
      </Row>
      <Row label="Factory reset radar">
        <Confirm
          label="Factory reset"
          title={CONFIRM.radar_factory.t}
          body={CONFIRM.radar_factory.b}
          confirmLabel="Reset the radar"
          danger
          onConfirm={() => post(entityPath("button/Radar Factory Reset", "press"))}
        />
      </Row>
    </Card>
  );
}

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
      {/* Named for the part plus what the card does to it, matching Power & Recovery below - a bare
          "XMOS" said which chip but not why you would open the card. HINTS.xmos still opens with "The
          audio chip", so the ⓘ carries what the name alone does not. */}
      {(xmosReset || xmosFlash) && (
        <Card title="XMOS Recovery" collapsible name="xmos" hint={HINTS.xmos}>
          {xmosReset && (
            <Row label="Restart XMOS">
              <Confirm
                label="Restart"
                title={CONFIRM.xmos_restart.t}
                body={CONFIRM.xmos_restart.b}
                confirmLabel="Restart XMOS"
                onConfirm={() => post(xmosReset)}
              />
            </Row>
          )}
          {/* The version is in the label rather than left to the Firmware card, because this is the one
              place where knowing what is on the chip decides whether to press the button. */}
          {xmosFlash && (
            <Row label={flashLabel} hint={HINTS.xmos_flash}>
              <Confirm
                label="Reflash"
                title={CONFIRM.xmos_flash.t}
                body={CONFIRM.xmos_flash.b}
                confirmLabel="Reflash now"
                onConfirm={() => post(xmosFlash)}
              />
            </Row>
          )}
          {/* "Erase its firmware" was here, wired to the erase_xmos_flash button. It is gone from the app
              on purpose: it leaves the audio chip blank, which takes the microphones, the speaker and the
              wake word with it, and the only route back is the Reflash row above - which needs the very
              chip it just erased to be talking. Reflash already overwrites, so erase-then-flash was never
              a step anyone needed. The ESPHome button still exists for a bench recovery over the API. */}
        </Card>
      )}

      <RadarRecovery ctx={ctx} />

      {/* "ESP32 Recovery", the owner's pick, and the one that finally pairs with XMOS Recovery above:
          the two cards do the same job for the two chips. (Previously "Sat1 Device", which described
          the whole page, then briefly "Maintenance" and "Power & Recovery".) Open by default, unlike
          XMOS Recovery: Restart is the row people actually come here for. The stored collapse state
          keeps the old "maint" key, so nobody's remembered preference resets over a rename. */}
      <Card title="ESP32 Recovery" collapsible name="maint" defaultOpen hint={HINTS.maintenance}>
        {restart && (
          <Row label="Restart">
            <Confirm
              label="Restart"
              title={CONFIRM.restart.t}
              body={CONFIRM.restart.b}
              confirmLabel="Restart"
              onConfirm={() => post(restart)}
            />
          </Row>
        )}
        {safe && (
          <Row label="Safe mode" hint={HINTS.safe_mode}>
            <Confirm
              label="Safe mode"
              title={CONFIRM.safe_mode.t}
              body={CONFIRM.safe_mode.b}
              confirmLabel="Restart into safe mode"
              onConfirm={() => post(safe)}
            />
          </Row>
        )}
        {factory && (
          <Row label="Factory reset" hint={HINTS.factory_reset}>
            <Confirm
              label="Factory reset"
              title={CONFIRM.factory_reset.t}
              body={CONFIRM.factory_reset.b}
              confirmLabel="Erase everything"
              danger
              onConfirm={() => post(factory)}
            />
          </Row>
        )}
        {!restart && !safe && !factory && <Missing what="Device maintenance" />}
      </Card>
    </>
  );
}

/* ------------------------------------------------------------------ */
/* Launch: the sign-in link, its QR, and the revocation                */
/* ------------------------------------------------------------------ */

/**
 * The tokenized sign-in URL, rendered only to a browser that is already signed in (the key rides
 * GET /api/sat1/state, which sits behind the session gate). Scan the QR with a phone and it lands
 * signed in with zero typing; paste the link into a Home Assistant dashboard button and it becomes
 * a true launch button.
 *
 * The two carry the same key on different origins, each matched to its lifetime. The QR is built on
 * the device's current IP: it is scanned live off this screen, so the address is fresh by
 * construction, and an IP works on every phone where a .local QR is a dead end for the ones that
 * cannot resolve mDNS (Android Chrome, the HA app's webview) - phones that can still end up on
 * .local, because the smart redirect carries ?key= along when it upgrades the origin. The copyable
 * link keeps the .local hostname: it is the form that gets pasted somewhere long-lived, and the
 * name survives DHCP handing the device a new address. Before the first state poll delivers the IP,
 * the QR falls back to the .local link rather than rendering a dead code.
 *
 * "Sign out everywhere" is the revocation half, and the reason offering the link at all is
 * defensible: the key is a bearer credential, and this is the one action that kills every copy of
 * it - every cookie, every QR, every pasted dashboard button - in one press. This browser stays
 * signed in (the device answers with a fresh cookie), and the new link takes over here within a
 * state poll.
 */
/** Copies to the clipboard, true on success. navigator.clipboard does not exist on insecure
 *  origins, which the direct device pages always are - the textarea dance is the fallback that
 *  still works everywhere. Shared by the Launch link and the Home Assistant card's YAML. */
function copyText(text) {
  try {
    if (navigator.clipboard?.writeText) {
      navigator.clipboard.writeText(text);
    } else {
      const ta = document.createElement("textarea");
      ta.value = text;
      ta.style.cssText = "position:fixed;opacity:0";
      document.body.appendChild(ta);
      ta.select();
      document.execCommand("copy");
      ta.remove();
    }
    return true;
  } catch {
    return false; /* The text is on screen; selecting it by hand still works. */
  }
}

function Launch({ ctx }) {
  const d = ctx.device;
  const [copied, setCopied] = useState(false);
  if (!d?.key) return null;

  // The copyable link prefers .local for its DHCP-proof lifetime - except on a network this
  // browser has proven cannot resolve it (the redirect probe's day-long memory), where a .local
  // link pasted into a dashboard is a guaranteed dead end and the IP form, fragile as it is, at
  // least works. The hint below the link says what that trade costs and names the fix (a DHCP
  // reservation).
  const noMdns = mdnsLooksBroken();
  const ipLink = qrSignInLink(d.ip, d.key);
  const link = noMdns && ipLink ? ipLink : signInLink(d.name, d.key);
  const qr = qrSvgPath(ipLink || link);

  const copy = () => {
    if (copyText(link)) {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    }
  };

  return (
    <Card title="Launch" collapsible name="launch" hint={HINTS.launch}>
      <div class="launch">
        {qr && (
          <svg class="launch-qr" viewBox={`-2 -2 ${qr.size + 4} ${qr.size + 4}`} role="img" aria-label="Sign-in QR code">
            <path d={qr.path} />
          </svg>
        )}
        <div class="launch-side">
          <div class="launch-link num">{link}</div>
          {noMdns && ipLink && <p class="dim sm">{TEXT.launch_mdns_hint}</p>}
          <div class="launch-actions">
            <Btn onClick={copy}>{copied ? TEXT.launch_copied : TEXT.launch_copy}</Btn>
            <Confirm
              label={TEXT.launch_regen}
              title={TEXT.launch_regen_title}
              body={TEXT.launch_regen_body}
              confirmLabel={TEXT.launch_regen_confirm}
              danger
              onConfirm={() => logoutAll().catch(() => {})}
            />
          </div>
        </div>
      </div>
      <ChangePassword ctx={ctx} />
    </Card>
  );
}

/**
 * The authenticated password change, at the foot of the Launch card - the card that already owns
 * the session and credential surface. The current password is proven with the login's nonce
 * challenge (it never crosses the wire); the new one is entered twice and checked client-side
 * before any request. A success re-keys this browser in place and kills every other session,
 * pasted link and QR - the logout_all contract - so the submit goes through a Confirm that says
 * exactly that. The Launch card's link and QR pick the new key up on the next state poll.
 *
 * Hidden entirely on pw_fixed builds: a YAML-pinned fleet password is re-imposed on every boot,
 * so a change here would silently revert - the device refuses it and the form never shows.
 */
function ChangePassword({ ctx }) {
  const [cur, setCur] = useState("");
  const [next, setNext] = useState("");
  const [again, setAgain] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState(null);
  if (ctx.device?.pw_fixed) return null;

  // Mirrors the firmware's password_acceptable_ exactly, so nothing valid here gets a 400 there:
  // 8-31 printable ASCII, no quote or backslash (they would complicate every place the password is
  // embedded - the HA payload's literal_eval path among them), no leading/trailing space.
  const check = () => {
    if (next.length < 8 || next.length > 31) return TEXT.pw_len;
    if (/["\\]/.test(next) || /[^\x20-\x7e]/.test(next) || next.trim() !== next) return TEXT.pw_chars;
    if (next !== again) return TEXT.pw_mismatch;
    return null;
  };

  const submit = async () => {
    if (busy) return;
    setBusy(true);
    setErr(null);
    try {
      const r = await changePassword(cur, next);
      if (r.ok) {
        setCur("");
        setNext("");
        setAgain("");
        toast({ kind: "ok", title: TEXT.pw_changed, sub: TEXT.pw_changed_sub, ttl: 6000 });
      } else if (r.locked) {
        setErr(TEXT.login_locked.replace("%s", String(r.retry || 60)));
      } else if (r.fixed) {
        setErr(TEXT.pw_fixed_note);
      } else if (r.invalid) {
        setErr(TEXT.pw_chars);
      } else {
        setErr(TEXT.pw_wrong);
      }
    } catch {
      setErr(TEXT.login_unreachable);
    } finally {
      setBusy(false);
    }
  };

  return (
    <div class="pwc">
      <div class="row">
        <span class="grow strong">{TEXT.pw_title}</span>
      </div>
      <input
        class="in"
        type="password"
        value={cur}
        placeholder={TEXT.pw_current}
        autocomplete="current-password"
        aria-label={TEXT.pw_current}
        onInput={(e) => setCur(e.currentTarget.value)}
      />
      <input
        class="in"
        type="password"
        value={next}
        placeholder={TEXT.pw_new}
        autocomplete="new-password"
        aria-label={TEXT.pw_new}
        onInput={(e) => setNext(e.currentTarget.value)}
      />
      <input
        class="in"
        type="password"
        value={again}
        placeholder={TEXT.pw_again}
        autocomplete="new-password"
        aria-label={TEXT.pw_again}
        onInput={(e) => setAgain(e.currentTarget.value)}
      />
      <div class="launch-actions">
        <Confirm
          label={busy ? TEXT.pw_busy : TEXT.pw_title}
          title={CONFIRM.pw_change.t}
          body={CONFIRM.pw_change.b}
          confirmLabel={TEXT.pw_title}
          danger
          disabled={busy || !cur || !next || !again}
          onConfirm={() => {
            // Client-side checks run at the moment of commitment, so a mismatch typed after the
            // modal opened is still caught; nothing leaves the browser unless they pass.
            const bad = check();
            if (bad) {
              setErr(bad);
              return;
            }
            submit();
          }}
        />
      </div>
      {err && <p class="t-err sm">{err}</p>}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Home Assistant sidebar setup                                        */
/* ------------------------------------------------------------------ */

const HASS_INGRESS_URL = "https://github.com/lovelylain/hass_ingress";

/**
 * Everything needed to put this page in the Home Assistant sidebar, through the third-party
 * hass_ingress integration's proxy mode. Right under Launch because it is the same subject - ways
 * to reach this UI - and collapsed by default because it is setup, not status: most visits to this
 * page are not setup visits.
 *
 * Proxy mode (work_mode: ingress) rather than an iframe of this device's own origin, because the
 * iframe cannot work: an https HA page may not embed a plain-http device (mixed content, blocked
 * before the request is made), and inside a cross-site iframe the browser refuses to send this
 * device's SameSite=Lax session cookie, so login loops forever. Proxied, the browser only ever
 * talks to HA's own origin - both problems vanish at once, and the same YAML serves local-http
 * and public-https installs alike. The frontend's side of the contract is BASE (lib/device.js).
 *
 * The block covers the *whole fleet*, not just this device (owner call, September 2026): the HA
 * sync roster already names every Satellite1 Home Assistant knows - mac, live IP, display name -
 * so the card mints one paste-ready arrangement with exactly one sidebar item. This device is the
 * visible entry, titled "Satellite1 Fleet"; every peer rides behind it as a hass_ingress child
 * (`parent:`), hidden from the sidebar but reachable at /<parent>/<child> - which is where the
 * device switcher's panel links land (see peerRow in shell.jsx: it probes the flat path first and
 * falls back to the nested one). Peer panel keys are minted from this device's own hostname base
 * plus each peer's mac suffix - the same name_add_mac_suffix convention the switcher relies on -
 * so the two ends of the contract cannot drift apart. A device renamed away from the convention,
 * or a roster row without a routable IP, is silently left out; with no roster at all (HA down,
 * solo device) the block degrades to this device's entry alone.
 *
 * The per-entry lines: require_admin (parent only - children are not sidebar panels) because the
 * panel exposes the devices' sign-in pages to every HA user who can see it, and admin-only is the
 * right default to hand out; expire_time because hass_ingress's own token defaults to an hour,
 * after which a standing tab is signed out of the *proxy* mid-session; the host header because
 * the proxy forwards the browser's Host, which the pairing endpoints' DNS-rebinding guard rightly
 * rejects - push-button sign-in needs a Host this device answers to.
 */
function HomeAssistant({ ctx }) {
  const d = ctx.device;
  const [copied, setCopied] = useState(false);
  if (!d?.ip) return null;

  const slugify = (s) =>
    String(s)
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, "_")
      .replace(/^_+|_+$/g, "");
  const clean = (s) => String(s || "").replace(/["\\]/g, "");
  const IPV4 = /^\d+\.\d+\.\d+\.\d+$/;

  const ownName = String(d.name || "satellite1").toLowerCase();
  const ownSlug = slugify(ownName) || "satellite1";
  const lines = [
    "ingress:",
    `  ${ownSlug}:`,
    "    work_mode: ingress",
    '    title: "Satellite1 Fleet"',
    "    icon: mdi:satellite-uplink",
    `    url: http://${d.ip}`,
    "    require_admin: true   # only admins see the panel; remove to show everyone",
    "    expire_time: 604800   # keep long-lived tabs signed in (default is 1 hour)",
    "    headers:",
    `      host: ${d.ip}   # keeps push-button sign-in working through the proxy`,
  ];

  // The peers, from the same roster the device switcher reads. The base-name proof and the
  // per-peer derivations mirror peerRow's exactly - one convention, two consumers.
  const ownSuffix = String(d.mac || "")
    .toLowerCase()
    .replace(/[^a-z0-9]/g, "")
    .slice(-6);
  const baseOk = ownSuffix.length === 6 && ownName.endsWith(ownSuffix);
  const rows = baseOk
    ? (ctx.ha?.d?.dev || [])
        .filter((r) => (r?.[3] || "").toLowerCase() !== (d.mac || "").toLowerCase())
        .sort((a, b) => String(a?.[1] || "").localeCompare(String(b?.[1] || "")))
    : [];
  for (const r of rows) {
    const suffix = String(r?.[3] || "")
      .toLowerCase()
      .replace(/[^a-z0-9]/g, "")
      .slice(-6);
    // The routable address: the roster's live IP, else an IP-literal configuration_url. A row
    // with neither cannot be proxied and is left out rather than emitted broken.
    let ip = String(r?.[11] || "");
    if (!IPV4.test(ip)) {
      try {
        const h = new URL(String(r?.[5] || "")).hostname;
        ip = IPV4.test(h) ? h : "";
      } catch {
        ip = "";
      }
    }
    if (suffix.length !== 6 || !ip) continue;
    lines.push(
      `  ${slugify(ownName.slice(0, -6) + suffix)}:`,
      `    parent: ${ownSlug}   # hidden from the sidebar; the device switcher reaches it`,
      "    work_mode: ingress",
      `    title: "${clean(r?.[1]) || "Satellite1"}"`,
      `    url: http://${ip}`,
      "    expire_time: 604800",
      "    headers:",
      `      host: ${ip}`,
    );
  }
  const yaml = lines.join("\n");

  const copy = () => {
    if (copyText(yaml)) {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    }
  };

  return (
    <Card title="Home Assistant" collapsible name="ha_ingress" hint={HINTS.ha_ingress}>
      <p class="dim sm hai-note">
        {TEXT.hai_pre}
        <a href={HASS_INGRESS_URL} target="_blank" rel="noopener">
          {TEXT.hai_link}
        </a>
        {TEXT.hai_post}
      </p>
      <pre class="hai-yaml">{yaml}</pre>
      <div class="launch-actions">
        <Btn onClick={copy}>{copied ? TEXT.hai_copied : TEXT.hai_copy}</Btn>
      </div>
      <p class="dim sm">{TEXT.hai_dhcp_hint}</p>
    </Card>
  );
}

/* The "Recent wake detections" card retired to the Wake Words route (owner call, September 2026):
   the richer card there - a time-lane per word, close calls included - lives beside the words it
   describes, and one home for the data beats two. */

/* ------------------------------------------------------------------ */
/* Community links                                                     */
/* ------------------------------------------------------------------ */

/* Four glyphs in ui.jsx's `ni` register - 16-box, stroked in currentColor - rather than the brand
   assets: pixel-true marks would bring their own colours into two themes, and everything else on
   this page is drawn this way. An open book, the octocat's outline, a play button in a rounded
   screen, and the Discord face. */
const I_BOOK = ni(
  <>
    <path d="M1.6 2.4h3.8a2.6 2.6 0 0 1 2.6 2.6v8.9a2 2 0 0 0-2-2H1.6z" />
    <path d="M14.4 2.4h-3.8A2.6 2.6 0 0 0 8 5v8.9a2 2 0 0 1 2-2h4.4z" />
  </>,
);
const I_GITHUB = ni(
  <>
    <path d="M10.67 14.67v-2.58a2.25 2.25 0 0 0-.63-1.74c2.09-.23 4.29-1.03 4.29-4.67a3.63 3.63 0 0 0-1-2.52 3.38 3.38 0 0 0-.06-2.51s-.79-.23-2.61.99a8.92 8.92 0 0 0-4.66 0c-1.82-1.22-2.61-.99-2.61-.99a3.38 3.38 0 0 0-.06 2.51 3.63 3.63 0 0 0-1 2.52c0 3.61 2.2 4.4 4.29 4.67a2.25 2.25 0 0 0-.62 1.73v2.59" />
    <path d="M6 12.67c-3.33 1-3.33-1.67-4.67-2" />
  </>,
);
const I_YOUTUBE = ni(
  <>
    <rect x="1.8" y="4" width="12.4" height="8.4" rx="2.6" />
    <path d="M6.9 6.6v3.2l3-1.6z" fill="currentColor" />
  </>,
);
const I_DISCORD = ni(
  <>
    <path d="M10.33 11.67l.67 1.33s2.78-.89 3.67-2.33c0-.67.35-5.43-2-7-1-.67-2.67-1-2.67-1l-.67 1.33h-1.33" />
    <path d="M5.69 11.67l-.67 1.33s-2.78-.89-3.67-2.33c0-.67-.35-5.43 2-7 1-.67 2.67-1 2.67-1l.67 1.33h1.33" />
    <circle cx="5.67" cy="8.33" r="1" fill="currentColor" stroke="none" />
    <circle cx="10.33" cy="8.33" r="1" fill="currentColor" stroke="none" />
  </>,
);

/* The organisation's four front doors. Organisation links, not device links - which is why the
   remote-control mode shows the same row - and held in code like the wake words Sources card's
   URLs (lib/wakesources.js), the app's other external links. */
const COMMUNITY = [
  { label: TEXT.cl_docs, url: "https://docs.futureproofhomes.net/", icon: I_BOOK },
  { label: TEXT.cl_github, url: "https://github.com/FutureProofHomes", icon: I_GITHUB },
  { label: TEXT.cl_youtube, url: "https://www.youtube.com/@futureproofhomes", icon: I_YOUTUBE },
  { label: TEXT.cl_discord, url: "https://discord.futureproofhomes.net/", icon: I_DISCORD },
];

/* The colophon under the last card - deliberately not a Card: a bordered box would compete with
   ESP32 Recovery above it, and these four words need no heading. Muted at rest (accent on hover)
   for the same reason: help lives here, but it must not out-shout the recovery buttons. */
function CommunityLinks() {
  return (
    <nav class="dx-links" aria-label={TEXT.cl_aria}>
      {COMMUNITY.map((c) => (
        <a key={c.label} href={c.url} target="_blank" rel="noopener">
          {c.icon}
          {c.label}
        </a>
      ))}
    </nav>
  );
}

/* ------------------------------------------------------------------ */

export function Diagnostics({ ctx }) {
  // The tap intent a toast carried here - "open the log at level W", "show the crash card" -
  // taken once at mount. The window event covers a toast tapped while this route was already
  // standing, where an identical hash fires no hashchange and nothing remounts.
  const [intent, setIntent] = useState(() => takeIntent());
  useEffect(() => {
    const on = () => setIntent(takeIntent());
    addEventListener("toast-intent", on);
    return () => removeEventListener("toast-intent", on);
  }, []);

  // The reveal: scroll the target card under the sticky bar - the cards' scroll margins keep it
  // clear of the top bar and the media bar - and pulse the thing to address: the firmware card's
  // update panel when one stands, else the card itself. The pulse is a class the CSS animates and
  // this removes; non-sticky by design (owner: "gentle fade in and out, not a sticky highlight").
  // The Logs card is exempt - its reveal is the line flash below, and two highlights on one card
  // is a page shouting.
  //
  // One scroll is not enough on this page, and the first build proved it (owner's screenshot,
  // September 2026: an update toast landing on Crash Reports). The cards above the target keep
  // growing after mount - the crash card renders nothing until its fetch returns, the Device card
  // fills on its first poll - so a scroll that landed correctly is wrong two hundred pixels later.
  // So the target stays pinned for the reveal window: a ResizeObserver on the body re-scrolls
  // (instantly - only the first ride is smooth) every time late content shifts the layout, and a
  // crash-card target that does not exist yet is found on the shift that creates it. The user
  // outranks the pin: one wheel tick or touch-drag and it stands down for good.
  useEffect(() => {
    if (!intent) return undefined;
    let el = null;
    let target = null;
    let userTook = false;
    let ended = false;
    const timers = [];

    const pin = (smooth) => {
      if (userTook || ended) return;
      if (!el) {
        el = document.getElementById(`card-${intent.card}`);
        if (!el) return; // not rendered yet; the resize that creates it re-enters here
        target = intent.card === "log" ? null : el.querySelector(".updbox") || el;
        target?.classList.add("reveal");
        timers.push(setTimeout(() => target?.classList.remove("reveal"), 2600));
      }
      el.scrollIntoView({ behavior: smooth ? "smooth" : "auto", block: "start" });
    };

    pin(true);
    const took = () => {
      userTook = true;
    };
    window.addEventListener("wheel", took, { passive: true });
    window.addEventListener("touchmove", took, { passive: true });
    // The observer fires once on observe() before anything has changed; that beat is skipped so
    // it cannot cut the smooth ride short for no reason.
    let first = true;
    const ro = new ResizeObserver(() => {
      if (first) {
        first = false;
        return;
      }
      pin(false);
    });
    ro.observe(document.body);
    timers.push(
      setTimeout(() => {
        ended = true;
        ro.disconnect();
      }, 2800),
    );

    return () => {
      ended = true;
      ro.disconnect();
      window.removeEventListener("wheel", took);
      window.removeEventListener("touchmove", took);
      for (const t of timers) clearTimeout(t);
      target?.classList.remove("reveal");
    };
  }, [intent]);

  return (
    <>
      {ctx.device && ctx.device.ha === false && <p class="banner">{TEXT.ha_disconnected_detail}</p>}
      <Device ctx={ctx} />
      {/* Right under Device, per the owner: its Power gain mode row is decided by the USB-C Power
          Supply reading a few rows up, and the pair tell one story about the power brick. */}
      <SpeakerAmp ctx={ctx} />
      {/* Under Device too in spirit - its Last Restart row is the question this card answers. */}
      <CrashCard ctx={ctx} reveal={intent?.card === "crash"} />
      <Firmware ctx={ctx} />
      <Launch ctx={ctx} />
      {/* Right under Launch, whose subject it shares: ways to reach this UI. */}
      <HomeAssistant ctx={ctx} />
      {/* Buttons moved to the foot of Controls. It is the one card here that answers "does the hardware
          respond to me", which is a question about the thing you are holding rather than about its
          internals - and it belongs beside the volume and mute controls it duplicates in hardware. */}
      <Log ctx={ctx} intent={intent?.card === "log" ? intent : null} />
      <Maintenance ctx={ctx} />
      <CommunityLinks />
    </>
  );
}
