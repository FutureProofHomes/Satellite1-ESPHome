/**
 * The rolling history behind the Home sensor chips' sparklines: per-sensor rings of [ts, value]
 * pairs, keyed by logical key ("temp", "humidity", "lux").
 *
 * Browser-side on purpose, after the same NVS-versus-browser reasoning notif.js records: the
 * device has no history to offer and no business growing one (web_ui_handler.h's transcript-ring
 * comment - 165KB of internal heap has no room for retention policies), so the buffer lives here,
 * where it costs the device zero RAM, zero flash, zero NVS wear - and where a device reboot
 * cannot touch it, because the device never held it.
 *
 * localStorage rather than sessionStorage (owner decision, September 2026): the history must
 * survive a browser restart. Namespaced per device by MAC ("sat1.spark.<mac>") for the same
 * single-origin device-switching reason notif.js gives, with the same local-bucket migration
 * dance before the first state poll delivers the MAC.
 *
 * Only real samples are ever stored. The synthetic cold-start backfill the chips draw (ui.jsx)
 * is render-side only and never lands here, so stored data stays honest.
 */

const PREFIX = "sat1.spark.";
/** Three hours of window, capped at 240 points per sensor - ~60 points/hour at the sensors' 60s
 *  publish cadence, with headroom for change-driven appends. ~11KB per device at worst. */
const WINDOW_MS = 3 * 60 * 60 * 1000;
const MAX_POINTS = 240;
/** Flat periods still accrue points on this beat, so a calm hour draws as a line, not a dot -
 *  x is time, not sample index. */
const MIN_GAP_MS = 60 * 1000;

let mac = "local";
let rings = {};

const prune = (list) => {
  const cut = Date.now() - WINDOW_MS;
  return list.filter((p) => Array.isArray(p) && Number.isFinite(p[0]) && Number.isFinite(p[1]) && p[0] >= cut).slice(-MAX_POINTS);
};

/* Guarded like every localStorage touch in the app: Safari's private mode and blocked site data
   throw rather than no-op, and chips that forget their past are survivable where a white screen
   is not. */
function load() {
  try {
    const raw = localStorage.getItem(PREFIX + mac);
    rings = raw ? JSON.parse(raw) : {};
    if (!rings || typeof rings !== "object" || Array.isArray(rings)) rings = {};
  } catch {
    rings = {};
  }
  for (const k in rings) rings[k] = Array.isArray(rings[k]) ? prune(rings[k]) : [];
}

function save() {
  try {
    localStorage.setItem(PREFIX + mac, JSON.stringify(rings));
  } catch {
    /* Not remembering is survivable; the in-memory ring keeps this session drawn. */
  }
}

load();

/**
 * Points the store at the device the app is talking to - the shell calls this beside
 * setNotifDevice, and for the same reasons. Samples recorded before the first call sit in the
 * "local" bucket and migrate once: they were about the serving device, which is the first MAC
 * this ever learns.
 */
export function setSparkDevice(nextMac) {
  const next = String(nextMac || "").toLowerCase();
  if (!next || next === mac) return;
  const orphans = mac === "local" ? rings : null;
  if (mac === "local") {
    try {
      localStorage.removeItem(PREFIX + "local");
    } catch {
      /* the merge below still happens in memory */
    }
  }
  mac = next;
  load();
  if (orphans) {
    for (const k in orphans) {
      rings[k] = prune([...(rings[k] || []), ...orphans[k]].sort((a, b) => a[0] - b[0]));
    }
  }
  save();
}

/** One real sample in: appended when the value moved since the last recorded point, or when ≥60s
 *  have passed regardless - then pruned to the window. One write a minute per sensor is nothing. */
export function sparkRecord(key, value) {
  if (!Number.isFinite(value)) return;
  const list = rings[key] || (rings[key] = []);
  const last = list[list.length - 1];
  if (last && last[1] === value && Date.now() - last[0] < MIN_GAP_MS) return;
  list.push([Date.now(), value]);
  rings[key] = prune(list);
  save();
}

/** The pruned ring for rendering: real samples only, oldest first. */
export function sparkPoints(key) {
  return prune(rings[key] || []);
}
