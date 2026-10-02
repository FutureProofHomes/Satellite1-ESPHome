/**
 * Which of the orb's tips this browser has acted on, so they stop showing (owner, October 2026):
 * the orb tapped, the windows swiped, a drawer opened. Kept in localStorage because it is about the
 * person at this browser, not the device - someone new to the page on another phone still gets
 * them. The ids are orbTips' in src/lib/orb.js; whatever does the thing calls tipDone.
 */
const KEY = "sat1.tips";
const subs = new Set();

/* Guarded like every localStorage touch in the app: it throws with site data blocked, and then
   the tips simply never retire. */
function read() {
  try {
    const v = JSON.parse(localStorage.getItem(KEY) || "[]");
    return Array.isArray(v) ? v.filter((x) => typeof x === "string") : [];
  } catch {
    return [];
  }
}

let done = read();

export const tipsDone = () => done;

export function tipDone(id) {
  if (done.includes(id)) return;
  done = [...done, id];
  try {
    localStorage.setItem(KEY, JSON.stringify(done));
  } catch {
    /* kept for this page's life only */
  }
  subs.forEach((f) => f());
}

/** Calls `f` whenever a tip retires; returns the unsubscribe. */
export function onTips(f) {
  subs.add(f);
  return () => {
    subs.delete(f);
  };
}
