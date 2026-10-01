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
