/**
 * The Settings pages' pure logic: wording the device facts, reading log lines, and minting the
 * hass_ingress YAML.
 */
import { TEXT } from "../copy.js";

export const kb = (n) => `${Math.round(n / 1024)} kB`;
export const mb = (n) => `${(n / 1048576).toFixed(1)} MB`;

/** Seconds as the design words a duration: "3d 7h 22m", "7h 22m", "22m 5s". */
export function uptime(s) {
  if (s == null) return "\u2014";
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (d) return `${d}d ${h}h ${m}m`;
  if (h) return `${h}h ${m}m`;
  return `${m}m ${s % 60}s`;
}

/**
 * The FUSB302B's contract string ("3.25A (max) @ 20V") as the USB-C Power Supply fact, in the
 * owner's format (September 2026): voltage first because it decides the amplifier's gain mode, the
 * ~ standing for "(max)" - the charger's ceiling, not a live draw - and the wattage under it,
 * because watts are how people know their chargers. The entity keeps the raw string, which is what
 * Home Assistant shows and automations may read. A string that does not parse shows raw, so a
 * future contract format degrades rather than losing the row. The sensor publishes on every
 * powered outcome, the plain-5V timeout included, so nothing at all (null here) means a build
 * without the PD sensor, not a 5 V supply.
 */
export function usbFact(raw) {
  if (raw == null || raw === "") return null;
  const m = /^([\d.]+)A \(max\) @ (\d+)V$/.exec(raw);
  if (!m) return { value: String(raw) };
  const amps = Number(m[1]);
  const volts = Number(m[2]);
  const watts = volts * amps;
  return { value: `${volts}V @ ${amps}A~`, sub: `${Number.isInteger(watts) ? watts : watts.toFixed(1)} watts` };
}

/** What follows a Sat1 firmware version: " (built with ESPHome 2026.9.1)", or nothing when unknown. */
export const builtWith = (v) => (v ? ` (built with ESPHome ${v})` : "");

/** The GitHub API address of the release an update entity's release_url names, or null. */
export function releaseApi(url) {
  const m = /^https:\/\/github\.com\/([^/]+)\/([^/]+)\/releases\/tag\/([^/?#]+)/.exec(url || "");
  return m ? `https://api.github.com/repos/${m[1]}/${m[2]}/releases/tags/${m[3]}` : null;
}

/** The "- ESPHome Version: 2026.9.1" line build_release.yaml writes into every release's notes. */
export const releaseEsphome = (body) => /^\s*-\s*ESPHome Version:\s*(\S+)/m.exec(body || "")?.[1] ?? null;

/**
 * The XMOS Firmware sensor as Device Info's fact. Satellite1::status_string() shares the sensor with
 * the flasher's callbacks in satellite1.base.yaml, so it holds a state as often as a version: only a
 * version is a release to link to, a flash in progress is amber, and the two states that leave the
 * device without audio are red.
 */
export function xmosFact(text) {
  const v = String(text ?? "").trim();
  if (xmosVersionKey(v)) return { value: v, version: true };
  if (v === "XMOS not responding" || v === "Flashing failed") return { value: v, tone: "err" };
  return { value: v || "\u2014", tone: v.startsWith("Flashing") ? "warn" : undefined };
}

/**
 * GET /api/sat1/amp's power mode, worded. `pending` first: during the ~100 ms activation window the
 * reported mode is still the bootstrap's, and "measuring" is the truth. Then `active`, because line
 * out or an XMOS flash shuts the amplifier down and the stale mode would lie. The two modes this
 * firmware selects get names; any other (PWR_MODE 1 or 3) shows raw, so a future firmware that
 * picks one reaches the screen without an app release.
 */
export function ampMode(amp) {
  if (!amp) return null;
  if (amp.pending) return { v: "Measuring\u2026", d: "Sampling the power supply" };
  if (!amp.active) return { v: "Off", d: "Line out selected or amplifier shut down" };
  if (amp.mode === 2) return { v: "High gain", d: "Running from the USB-PD supply" };
  if (amp.mode === 0) return { v: "Low gain", d: "Running from the 5 V rail" };
  return { v: `PWR_MODE ${amp.mode}`, d: null };
}

/**
 * The analog gain index (0-20) as the dBV the amplifier applies: 11-21 dBV in half steps. The index
 * is what the number entity stores and the slider writes; dBV is only what the readout says.
 */
export const gainDbv = (v) => `${(11 + v / 2).toFixed(1)} dBV`;

/* ------------------------------------------------------------------ */
/* The XMOS firmware picker                                            */
/* ------------------------------------------------------------------ */

// The picker reads the developer firmware's xmos_firmware_catalog entities. The strings matched
// here are that component's own, from xmos_firmware_catalog.cpp: the first select option is
// "Built-in (<version>)", and the status sensor's wording is parsed below.
const XF_VERSION = /^v?(\d+)\.(\d+)\.(\d+)(?:-(alpha|beta|rc|dev))?(?:\.(\d+))?$/;

/** One XMOS version, however it is written: Satellite1::status_string() drops a zero build number. */
function xmosVersionKey(v) {
  const m = XF_VERSION.exec(String(v || "").trim());
  return m ? `${+m[1]}.${+m[2]}.${+m[3]}-${m[4] || ""}.${+(m[5] || 0)}` : null;
}

const sameXmosVersion = (a, b) => {
  const ka = xmosVersionKey(a);
  return ka !== null && ka === xmosVersionKey(b);
};

/** The built-in image's version, from the select's first option. */
export const xmosBuiltin = (options) => /\((v[^)]+)\)$/.exec(options?.[0] || "")?.[1] ?? "";

/**
 * The firmware select's options with the one the chip is running as the value - not the select's
 * own value, which is only what Home Assistant last picked for its Install button. A running
 * version the list doesn't carry shows as itself, and no reading at all as a dash.
 */
export function xmosChoices(options, running) {
  const builtin = xmosBuiltin(options);
  const match = builtin && sameXmosVersion(running, builtin) ? options[0] : options.slice(1).find((o) => sameXmosVersion(o, running));
  return { options, value: match ?? (xmosVersionKey(running) ? running : "\u2014") };
}

/** An option, or a status message's target, in prose: the built-in copy is named as such. */
export function xmosLabel(options, v) {
  if (v === options?.[0]) return TEXT.xf_builtin_long.replace("%s", xmosBuiltin(options));
  if (String(v).startsWith("built-in ")) return TEXT.xf_builtin_long.replace("%s", v.slice(9));
  return v;
}

const XF_STATUS = [
  ["requested", /^Stopping audio to install (.+)\.\.\.$/],
  ["downloading", /^Downloading (.+) \((\d+)%\)$/],
  ["flashing", /^Flashing (.+) \((\d+)%\)$/],
  ["starting", /^Starting (.+)\.\.\.$/],
  ["recovering", /^Restoring built-in (\S+?)(?:\.\.\.| \((\d+)%\))$/],
];

/**
 * The install status sensor's text as a stage. `result` is a finished install's outcome, which the
 * sensor keeps showing until the next refresh or install; anything else idle reads as no stage.
 */
export function xmosStatus(text) {
  const s = String(text || "");
  for (const [stage, re] of XF_STATUS) {
    const m = re.exec(s);
    if (m) return { stage, target: m[1], pct: Math.min(100, +(m[2] || 0)) };
  }
  if (s.startsWith("Installed ")) return { stage: "idle", result: s, ok: true };
  if (s.startsWith("Failed: ")) return { stage: "idle", result: s, ok: false };
  if (s.startsWith("Refresh failed: ")) return { stage: "idle", refreshing: false, refreshError: s.slice(16) };
  return { stage: "idle", refreshing: s === "Refreshing firmware list..." };
}

// Where each stage sits on the one bar, weighted by how long it takes on the device: about five
// seconds to stop audio, a few to download, most of a minute to flash, a few for the chip to start.
const XF_STAGES = { requested: [0, 5], downloading: [5, 20], flashing: [20, 95], starting: [97, 97] };
const XF_LABELS = {
  requested: "xf_stopping",
  downloading: "xf_downloading",
  flashing: "xf_flashing",
  starting: "xf_starting",
};

/**
 * The install as one bar, because each stage's own percentage restarting from zero reads as going
 * backwards. The built-in image has no download, so its flash spans the bar. `pending` covers the
 * moment between Yes and the device reporting the install. Null when nothing is running.
 */
export function xmosProgress(status, pending) {
  if (status.stage === "idle") return pending ? { label: TEXT.xf_stopping, pct: 0 } : null;
  if (status.stage === "recovering") return { label: TEXT.xf_recovering, pct: status.pct, warn: true };
  const builtin = status.target.startsWith("built-in ");
  const [from, to] = status.stage === "flashing" && builtin ? [5, 99] : XF_STAGES[status.stage];
  const moving = status.stage === "downloading" || status.stage === "flashing";
  const label = TEXT[XF_LABELS[status.stage]].replace("%s", xmosLabel(null, status.target));
  return { label, pct: Math.round(from + (moving ? ((to - from) * status.pct) / 100 : 0)) };
}

/* ------------------------------------------------------------------ */
/* The log                                                             */
/* ------------------------------------------------------------------ */

/** The level menu: [floor, label], most to least. */
export const LOG_LEVELS = [
  ["VV", "Everything"],
  ["D", "Debug"],
  ["I", "Info"],
  ["W", "Warning"],
  ["E", "Errors"],
];

const ORDER = ["VV", "V", "D", "I", "W", "E"];

/** Whether a line at `lvl` passes the `floor`. Config lines and unparsed ones always do. */
export function levelShows(floor, lvl) {
  return lvl === "C" || lvl === "?" || ORDER.indexOf(lvl) >= ORDER.indexOf(floor);
}

/** The ring's lines at or above `floor` whose text contains `needle`, case-insensitively. */
export function filterLog(log, floor, needle) {
  const n = String(needle || "").trim().toLowerCase();
  return log.filter((l) => levelShows(floor, l.lvl) && (!n || l.text.toLowerCase().includes(n)));
}

const HEAD = /^\[(?:VV|V|D|I|W|E|C)\]\[([^\]:]+)(?::\d+)?\]:?\s?/;

/** "[D][sensor:094]: 'Temperature' ..." -> { tag: "sensor", msg: "'Temperature' ..." }. */
export function logParts(text) {
  const m = HEAD.exec(text);
  return m ? { tag: m[1], msg: text.slice(m[0].length) } : { tag: "", msg: text };
}

/** A line's arrival time as HH:MM:SS.mmm, padded so the column never wobbles. */
export function stamp(at) {
  const d = new Date(at);
  const p = (n, w = 2) => String(n).padStart(w, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
}

/**
 * What Export saves: the lines on screen, filters and all, because the filtered view is the thing
 * worth sending to someone else. Each carries its arrival time, so a support request's "when did
 * it happen" gets the same answer the screen gives.
 */
export const logExport = (lines) => lines.map((l) => `[${stamp(l.at)}] ${l.text}`).join("\n");

export const logFileName = (date) => `satellite1-${date.toISOString().slice(0, 19).replace(/[:T]/g, "-")}.log`;

/* ------------------------------------------------------------------ */
/* Crash reports and the password                                      */
/* ------------------------------------------------------------------ */

/**
 * When a crash record happened, best first: the flight recorder's wall-clock stamp; for the crash
 * that ended the previous session, now minus the current uptime; else uptime and restart distance.
 */
export function crashWhen(r, boot, uptimeNow, now = Date.now()) {
  if (r.epoch) return new Date(r.epoch * 1000).toLocaleString();
  const n = boot - r.boot;
  if (n === 1 && uptimeNow) return `\u2248 ${new Date(now - uptimeNow * 1000).toLocaleString()}`;
  return (n === 1 ? TEXT.crash_restart_ago : TEXT.crash_restarts_ago).replace("%1", uptime(r.up)).replace("%2", String(n));
}

/**
 * Mirrors the firmware's password_acceptable_ exactly, so nothing valid here earns a 400 there:
 * 8-31 printable ASCII, no leading or trailing space, and no quote or backslash, which would
 * complicate every place the password is embedded (the Home Assistant payload's literal_eval path
 * among them). Returns the TEXT key of the first problem, or null.
 */
export function passwordProblem(next, again) {
  if (next.length < 8 || next.length > 31) return "pw_len";
  if (/["\\]/.test(next) || /[^\x20-\x7e]/.test(next) || next.trim() !== next) return "pw_chars";
  if (next !== again) return "pw_mismatch";
  return null;
}

/* ------------------------------------------------------------------ */
/* The hass_ingress YAML                                               */
/* ------------------------------------------------------------------ */

const slugify = (s) =>
  String(s)
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "");
const clean = (s) => String(s || "").replace(/["\\]/g, "");
const IPV4 = /^\d+\.\d+\.\d+\.\d+$/;
const macTail = (mac) =>
  String(mac || "")
    .toLowerCase()
    .replace(/[^a-z0-9]/g, "")
    .slice(-6);

/**
 * One paste-ready hass_ingress block for the whole fleet (owner call, September 2026): this device
 * as the one visible "Satellite1 Fleet" panel, every peer in the Home Assistant roster (`dev` rows:
 * [model, name, area, mac, sw, url, up, pw, net, radar, present, ip]) hidden behind it as a
 * `parent:` child, reachable at /<parent>/<child>, where the device switcher's panel links land.
 * Peer keys are this device's hostname base plus the peer's mac suffix - the name_add_mac_suffix
 * convention peerRow in src/components/Satellite1Now.tsx derives the same way, so the two ends
 * cannot drift apart. A device renamed away from it gets no peers, and with no roster at all (HA
 * down, a solo device) the block is this device's entry alone.
 *
 * Proxy mode (work_mode: ingress) rather than an iframe of the device's own origin, which cannot
 * work: an https HA page may not embed a plain-http device (mixed content), and a cross-site iframe
 * never gets the device's SameSite=Lax session cookie, so login loops forever. Proxied, the browser
 * only talks to HA's origin, on local-http and public-https installs alike; the app's side of that
 * contract is BASE in src/lib/device.js. The per-entry lines: require_admin (parent only - children
 * are not sidebar panels) because the panel exposes the devices' sign-in pages to every HA user who
 * can see it; expire_time because hass_ingress's own token defaults to an hour, after which a
 * standing tab is signed out of the proxy mid-session; the host header because the proxy forwards
 * the browser's Host, which the pairing endpoints' DNS-rebinding guard rejects (Authentication in
 * docs/web-ui.md), and push-button sign-in needs a Host this device answers to.
 */
export function ingressYaml(d, roster) {
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
  const ownSuffix = macTail(d.mac);
  const baseOk = ownSuffix.length === 6 && ownName.endsWith(ownSuffix);
  const rows = baseOk
    ? (roster || [])
        .filter((r) => (r?.[3] || "").toLowerCase() !== String(d.mac || "").toLowerCase())
        .sort((a, b) => String(a?.[1] || "").localeCompare(String(b?.[1] || "")))
    : [];
  for (const r of rows) {
    const suffix = macTail(r?.[3]);
    // The routable address: the roster's live IP, else an IP-literal configuration_url. A row with
    // neither cannot be proxied, so it is left out rather than emitted broken.
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
  return lines.join("\n");
}
