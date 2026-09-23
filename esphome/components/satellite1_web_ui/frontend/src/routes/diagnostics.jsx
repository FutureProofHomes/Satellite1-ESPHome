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
import { logoutAll, qrSignInLink, signInLink } from "../lib/auth.js";
import { entity, pathFor, post, request, requestJson } from "../lib/device.js";
import { qrSvgPath } from "../lib/qr.js";
import { Btn, Card, Chevron, Confirm, Fact, Missing, N_DIAG, Row, Toggle } from "../ui.jsx";

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
function CrashCard({ ctx }) {
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
    fetch("/api/sat1/crash/dump.bin")
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
    <Card title={TEXT.crash_title} collapsible name="crash" defaultOpen hint={HINTS.crash}>
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

function Log({ ctx }) {
  const { log, logSeq, pausedRef, logWatch } = ctx;
  const [paused, setPaused] = useState(false);

  // Register as the log's reader for exactly as long as this card is mounted. While no reader is
  // registered the ring still fills - that is what lets this card show the recent past the moment
  // someone navigates here - but arriving lines stop costing the rest of the app renders.
  useEffect(() => logWatch(), []);
  const [minLevel, setMinLevel] = useState("D");
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
          <div key={i} class={`ln t-${LEVELS[l.lvl] || "dim"}`}>
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
function Launch({ ctx }) {
  const d = ctx.device;
  const [copied, setCopied] = useState(false);
  if (!d?.key) return null;

  const link = signInLink(d.name, d.key);
  const qr = qrSvgPath(qrSignInLink(d.ip, d.key) || link);

  const copy = () => {
    try {
      // navigator.clipboard does not exist on insecure origins, which this page always is - the
      // textarea dance is the fallback that still works everywhere.
      if (navigator.clipboard?.writeText) {
        navigator.clipboard.writeText(link);
      } else {
        const ta = document.createElement("textarea");
        ta.value = link;
        ta.style.cssText = "position:fixed;opacity:0";
        document.body.appendChild(ta);
        ta.select();
        document.execCommand("copy");
        ta.remove();
      }
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch {
      /* The link is on screen; selecting it by hand still works. */
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
    </Card>
  );
}

/* The "Recent wake detections" card retired to the Wake Words route (owner call, September 2026):
   the richer card there - a time-lane per word, close calls included - lives beside the words it
   describes, and one home for the data beats two. */

/* ------------------------------------------------------------------ */

export function Diagnostics({ ctx }) {
  return (
    <>
      {ctx.device && ctx.device.ha === false && <p class="banner">{TEXT.ha_disconnected_detail}</p>}
      <Device ctx={ctx} />
      {/* Right under Device, whose Last Restart row is the question this card answers. */}
      <CrashCard ctx={ctx} />
      <Firmware ctx={ctx} />
      <Launch ctx={ctx} />
      {/* Buttons moved to the foot of Controls. It is the one card here that answers "does the hardware
          respond to me", which is a question about the thing you are holding rather than about its
          internals - and it belongs beside the volume and mute controls it duplicates in hardware. */}
      <Log ctx={ctx} />
      <Maintenance ctx={ctx} />
    </>
  );
}
