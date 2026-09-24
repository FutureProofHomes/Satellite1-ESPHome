/**
 * The toast store: the app's out-of-band vocabulary, shown on the notch surface beside the route
 * tab (NotchToasts in shell.jsx renders it; the run to the tab's right was reserved for exactly
 * this - see .rtab in app.css).
 *
 * Module-scope pub/sub rather than context, for the reason onWriteError in device.js is: the
 * callers live in files the shell imports (device.js most of all), so an import the other way
 * would be a cycle. Anything may call toast(); the one host subscribes.
 *
 * Two lifetimes, matching what the old bottom surface learned the hard way:
 *
 * - Timed toasts (ttl > 0) are moments - a failed write, a log warning, news. One shows at a
 *   time (the notch is a single row), later ones queue, and errors jump the queue because "it
 *   broke" outranks "it recovered". A toast re-announcing its `key` while one with that key is
 *   already up coalesces into it - the count badge - rather than stacking: a slider dragged
 *   against a dead device fails a dozen writes a second, and a dozen identical toasts is a
 *   haranguing, not a notification. A key that just left the screen stays quiet for a few
 *   seconds (RECENT_MS) so a slow burst does not re-open the same news every frame.
 *
 * - Sticky toasts (no ttl) are standing conditions - the lost stream. They hold the surface
 *   while nothing transient is showing and leave only when their caller resolves them (or the
 *   user dismisses; the condition's owner re-raises on the next rising edge if it must).
 *
 * Every err/warn/info toast also writes itself into the notification history (lib/notif.js -
 * the bell's drawer; ok toasts are moments of good news and are not kept). How a toast leaves
 * the screen decides its entry's fate, which is the bell badge's whole arithmetic: tapped or
 * ✕'d here means handled (archived, never counted); fading out on its own timer - or being
 * preempted, or a sticky resolving unhandled - means unhandled (active, badge +1).
 */

import { recordNotif, settleNotif, touchNotif } from "./notif.js";

const subs = new Set();

let transient = null; // The visible timed toast.
const queue = []; // Timed toasts waiting their turn; errors enter at the front.
const stickies = []; // Standing conditions; the newest shows when no transient is up.
let hideTimer = null;
let idSeq = 0;

/** How long a dismissed key stays quiet before the same key may toast again. */
const RECENT_MS = 5000;
const recent = new Map(); // key -> when it last left the screen

const visible = () => transient || stickies[stickies.length - 1] || null;

function emit() {
  const v = visible();
  for (const fn of subs) fn(v);
}

/** The host's subscription: called with the toast to show (or null), now and on every change. */
export function subscribeToasts(fn) {
  subs.add(fn);
  fn(visible());
  return () => subs.delete(fn);
}

/** A toast leaving the screen: the rate floor remembers its key, the history learns its fate. */
function finish(t, how) {
  if (!t) return;
  if (t.key) recent.set(t.key, Date.now());
  if (t.nid) settleNotif(t.nid, how);
}

function endTransient(how) {
  clearTimeout(hideTimer);
  finish(transient, how);
  transient = null;
  const next = queue.shift();
  if (next) show(next);
  else emit();
}

function show(t) {
  transient = t;
  clearTimeout(hideTimer);
  hideTimer = setTimeout(() => endTransient("expire"), t.ttl);
  emit();
}

/**
 * Raise a toast. Fields:
 *   kind   "err" | "warn" | "ok" | "info" - the colour and glyph.
 *   title  the one fact; sub - optional second line, usually naming the tap action.
 *   ttl    ms on screen; omit for a sticky standing condition.
 *   key    dedupe identity: coalesces bursts into one toast with a xN badge.
 *   go     hash to navigate to on tap (e.g. "#/diagnostics").
 *   intent handoff for the destination (see toastIntent below), carried on tap alongside `go`.
 *   act    a serialisable action token instead of navigation ("fix" opens the fix drawer). A
 *          token rather than a callback because the history rows replay the same action after a
 *          reload, and a function cannot survive localStorage.
 *
 * Returns { resolve } - for sticky toasts the way the condition's owner ends it (expire
 * semantics: nobody handled it, so it counts on the bell), for timed ones an early dismiss.
 */
export function toast(t) {
  const entry = { id: ++idSeq, count: 1, ...t };
  // Deferred past the coalesce paths below: recordNotif dedupes by key+title itself, and a raise
  // that folds into a visible toast reports through touchNotif - recording it here too would
  // count the same hit twice on the entry.
  const record = () => {
    if (entry.kind !== "ok") entry.nid = recordNotif(entry);
  };
  const handle = { resolve: () => endToast(entry.id, "expire") };

  if (!entry.ttl) {
    // One sticky per key: re-raising an already-standing condition hands back the standing one.
    const dup = entry.key && stickies.find((s) => s.key === entry.key);
    if (dup) return { resolve: () => endToast(dup.id, "expire") };
    record();
    stickies.push(entry);
    emit();
    return handle;
  }

  if (entry.key) {
    if (transient && transient.key === entry.key) {
      // The burst case: same news while it is up counts up, keeps the timer running whole, and
      // adopts the newest title and intent - so a ×12 toast tapped lands on the latest line the
      // burst produced, not the first.
      transient = { ...transient, count: transient.count + 1, title: entry.title, sub: entry.sub, intent: entry.intent };
      if (transient.nid) touchNotif(transient.nid, { title: entry.title, sub: entry.sub, intent: entry.intent });
      clearTimeout(hideTimer);
      hideTimer = setTimeout(() => endTransient("expire"), entry.ttl);
      emit();
      return { resolve: () => endToast(transient.id, "expire") };
    }
    const queued = queue.find((x) => x.key === entry.key);
    if (queued) {
      queued.count += 1;
      queued.title = entry.title;
      queued.sub = entry.sub;
      queued.intent = entry.intent;
      if (queued.nid) touchNotif(queued.nid, { title: entry.title, sub: entry.sub, intent: entry.intent });
      return { resolve: () => endToast(queued.id, "expire") };
    }
    const at = recent.get(entry.key);
    if (at && Date.now() - at < RECENT_MS) {
      // The rate floor: just-told news stays told on screen - but it still happened, so the
      // history takes the hit (deduped into its standing row) as an already-settled fact.
      record();
      if (entry.nid) settleNotif(entry.nid, "expire");
      return handle;
    }
  }

  record();
  if (!transient) {
    show(entry);
  } else if (entry.kind === "err" && transient.kind !== "err") {
    // Errors jump the queue - all the way past the standing toast, which is preempted rather
    // than requeued: by the time the error clears, the milder news is stale. Preemption is not
    // the user handling it, so the preempted toast's entry counts as unhandled.
    clearTimeout(hideTimer);
    finish(transient, "expire");
    show(entry);
  } else if (entry.kind === "err") {
    queue.unshift(entry);
  } else {
    queue.push(entry);
  }
  return handle;
}

/**
 * Ends a toast wherever it stands - visible, queued or sticky - with `how` deciding the history
 * entry's fate ("tap"/"dismiss" archived, "expire" active). One exception: a *tapped* sticky
 * stays on screen, because the condition it names has not ended - only its entry settles, so the
 * bell never counts a condition the user already went and looked at.
 */
function endToast(id, how) {
  if (transient?.id === id) {
    endTransient(how);
    return;
  }
  const q = queue.findIndex((t) => t.id === id);
  if (q >= 0) {
    finish(queue[q], how);
    queue.splice(q, 1);
    return;
  }
  const s = stickies.findIndex((t) => t.id === id);
  if (s >= 0) {
    if (how === "tap") {
      // Settle the entry, keep the toast: the lost stream is still lost.
      if (stickies[s].nid) settleNotif(stickies[s].nid, "tap");
      return;
    }
    finish(stickies[s], how);
    stickies.splice(s, 1);
    emit();
  }
}

/** The ✕: dismiss without acting. Handled, so the bell never counts it. */
export function dismissToast(id) {
  endToast(id, "dismiss");
}

/** A tap that acted. Handled; the transient leaves the screen, a sticky stands (see endToast). */
export function tapToast(id) {
  endToast(id, "tap");
}

/* ------------------------------------------------------------------ */
/* The tap intent: what a toast wants the destination to do            */
/* ------------------------------------------------------------------ */

/**
 * A toast tap that navigates can also carry an intent - "open the Logs card at level W, scroll
 * to this line and flash it" - which the destination consumes on mount. In memory rather than
 * sessionStorage on purpose: a reload must not replay a navigation the user made minutes ago,
 * and the hash alone stays clean enough to bookmark.
 */
let pendingIntent = null;

export function toastIntent(intent) {
  pendingIntent = intent || null;
}

/** The pending intent, taken exactly once. Returns null when nothing is waiting. */
export function takeIntent() {
  const i = pendingIntent;
  pendingIntent = null;
  return i;
}
