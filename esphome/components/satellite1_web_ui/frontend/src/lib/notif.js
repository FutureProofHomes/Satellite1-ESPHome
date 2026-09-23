/**
 * The notification history: every err/warn/info toast of the past 24 hours, behind the bell.
 *
 * localStorage on purpose, after a deliberate NVS-versus-browser discussion (September 2026,
 * recorded in the plan): these entries are browser-witnessed events - toasts this browser showed
 * this person - plus per-person handled state, which is exactly the half of the world localStorage
 * owns. Device-witnessed facts already persist device-side (the crash NVS ring, the update
 * entity); the drawer only points at them. Ok toasts are never recorded: good news needs no
 * follow-up, and an archive of "everything went fine" buries the things that did not.
 *
 * Namespaced per device by MAC ("sat1.notif.<mac>"), because the single-origin device switcher
 * serves the page from one device while controlling another, and a shared bucket would blend two
 * devices' notifications into one bell. Before the first state poll delivers the MAC, entries land
 * in the "local" bucket and migrate into the real one the moment it is known - the serving device
 * is who those early entries were about.
 *
 * Three states, and the bell's badge is defined by them: "pending" while the toast is still on
 * screen (not yet counted - it may yet be tapped), "active" once it faded away unhandled (badge
 * +1), "archived" once tapped or dismissed anywhere (badge -1; still browsable under the Archive
 * pill - archive means handled, not deleted). There is no "seen" state: the badge answers "how
 * many things still need me", not "have I peeked", so opening the drawer alone never clears it
 * (owner decision, September 2026).
 *
 * Every read and write prunes at 24 hours. Pending entries found on load - a tab closed while a
 * toast stood - become active: the toast is gone and nobody tapped it, which is the definition.
 */

const PREFIX = "sat1.notif.";
const DAY_MS = 24 * 60 * 60 * 1000;

let mac = "local";
let entries = [];
let idSeq = 0;
const subs = new Set();

/* Guarded like every localStorage touch in the app: Safari's private mode and blocked site data
   throw rather than no-op, and a bell that forgets is survivable where a white screen is not. */
function load() {
  try {
    const raw = localStorage.getItem(PREFIX + mac);
    entries = raw ? JSON.parse(raw) : [];
    if (!Array.isArray(entries)) entries = [];
  } catch {
    entries = [];
  }
  const cut = Date.now() - DAY_MS;
  entries = entries.filter((e) => e && e.ts >= cut);
  // A pending entry can only have come from a tab that closed mid-toast: unhandled by definition.
  for (const e of entries) if (e.state === "pending") e.state = "active";
  // Ids restart per session; keep new ones clear of anything loaded.
  for (const e of entries) if (e.id >= idSeq) idSeq = e.id + 1;
}

function save() {
  const cut = Date.now() - DAY_MS;
  entries = entries.filter((e) => e.ts >= cut);
  try {
    localStorage.setItem(PREFIX + mac, JSON.stringify(entries));
  } catch {
    /* Not remembering is survivable. */
  }
  for (const fn of subs) fn();
}

load();

/**
 * Points the store at the device the app is talking to. Called by the shell whenever the state
 * poll knows the MAC - which covers both the first load and a remote-control retarget (AppInner
 * remounts per device, and its effect re-runs here). Entries recorded before the first call sit
 * in the "local" bucket and migrate once: they were about the serving device, which is the first
 * MAC this ever learns.
 */
export function setNotifDevice(nextMac) {
  const next = String(nextMac || "").toLowerCase();
  if (!next || next === mac) return;
  const orphans = mac === "local" ? entries : [];
  if (mac === "local") {
    try {
      localStorage.removeItem(PREFIX + "local");
    } catch {
      /* the copy below still happens in memory */
    }
  }
  mac = next;
  load();
  if (orphans.length) {
    entries = [...entries, ...orphans].sort((a, b) => a.ts - b.ts);
  }
  save();
}

/** The drawer's view: newest first, already pruned. */
export function listNotifs() {
  const cut = Date.now() - DAY_MS;
  return entries.filter((e) => e.ts >= cut).sort((a, b) => b.ts - a.ts);
}

/** What the bell wears: how many notifications still need handling. Pending ones are not counted
 *  (their toast is still on screen and may yet be tapped) and not listed in the drawer either -
 *  the notch is their surface until they settle. */
export function notifCount() {
  const cut = Date.now() - DAY_MS;
  return entries.filter((e) => e.state === "active" && e.ts >= cut).length;
}

export function subscribeNotifs(fn) {
  subs.add(fn);
  return () => subs.delete(fn);
}

/** A new toast's entry, born pending. Returns the id the toast store keeps for settlement. */
export function recordNotif({ kind, title, sub, count, go, intent, act }) {
  const e = { id: ++idSeq, ts: Date.now(), state: "pending", kind, title, sub, count: count || 1, go, intent, act };
  entries.push(e);
  save();
  return e.id;
}

/** A coalescing burst updating its one entry: newest title/intent win, the count and clock move. */
export function touchNotif(id, patch) {
  const e = entries.find((x) => x.id === id);
  if (!e) return;
  Object.assign(e, patch, { ts: Date.now() });
  save();
}

/**
 * How a toast's on-screen life ended. "tap" and "dismiss" are the user handling it - archived,
 * never counted. "expire" is the fade-away untapped - active, badge +1. An entry already archived
 * stays archived (a sticky tapped early, then resolved by its own condition, must not resurrect
 * onto the badge).
 */
export function settleNotif(id, how) {
  const e = entries.find((x) => x.id === id);
  if (!e || e.state === "archived") return;
  e.state = how === "expire" ? "active" : "archived";
  save();
}

/** The drawer's archive: a row tapped, ✕'d or swiped away. Badge -1 by definition. */
export function archiveNotif(id) {
  const e = entries.find((x) => x.id === id);
  if (!e || e.state === "archived") return;
  e.state = "archived";
  save();
}
