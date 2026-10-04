/**
 * The device's recent log, GET /api/sat1/log, and how it joins the lines the live stream delivered.
 *
 * The stream only carries what the device says while a page is listening. The firmware keeps the
 * rest in two PSRAM rings (log_history.h) and serves them as text. Both sides carry the device's
 * uptime in ms - the stream as each event's id, the history as each record's prefix - and that is
 * what puts the two in order and tells one line held twice from two lines that read the same.
 */

/** The level out of a line's own "[W][tag:line]:" header. */
export const LOG_LEVEL = /^\[(VV|V|D|I|W|E|C)\]/;

/** How far apart the two copies of one line are stamped. The stream's listener and the ring's run
 *  back to back on the same task, so in practice 0 or 1. */
const SAME_LINE_MS = 5;

/** a - b in uptime ms, as int32: right across the 49.7-day wrap of a u32 millis(). */
export const msSince = (a, b) => (a - b) | 0;

/**
 * The response body, or null for anything that is not a log history (a sign-in page, an older
 * firmware's 404). The first line says where the answer stands; every line after it is a record.
 *
 *   #sat1-log boot=<hex> now=<millis> end=<position>
 *   <millis> <line, its own newlines as \x1f>
 *
 * `end` stays a string: it goes back to the device verbatim as the next read's `from`.
 */
export function parseLogHistory(body) {
  const nl = body.indexOf("\n");
  const head = (nl < 0 ? body : body.slice(0, nl)).split(" ");
  if (head[0] !== "#sat1-log") return null;
  const f = Object.fromEntries(head.slice(1).map((kv) => kv.split("=")));
  const now = Number(f.now);
  if (!f.boot || !f.end || !Number.isFinite(now)) return null;
  const lines = [];
  if (nl >= 0) {
    for (const rec of body.slice(nl + 1).split("\n")) {
      const sp = rec.indexOf(" ");
      const ms = sp > 0 ? Number(rec.slice(0, sp)) : NaN;
      if (Number.isFinite(ms)) lines.push({ ms, text: rec.slice(sp + 1).replace(/\x1f/g, "\n") });
    }
  }
  return { boot: f.boot, now, end: f.end, lines };
}

/**
 * The ring's current-boot lines with a history answer joined in, oldest first.
 *
 * A line the stream delivered is usually in the history too, and is kept once - as the stream's
 * copy, so the object a toast or a highlight already points at stays the one on screen. Each
 * stream line pairs with one record at most, so a line genuinely said twice in a row stays two.
 * Records at or before `floorMs` are what Clear removed and stay removed. A record's `at` is
 * worked back from the device's own clock (`now`, when it answered), not from any browser's.
 */
export function mergeLogHistory(log, hist, { receivedAt, gen, floorMs = null, nextId }) {
  const twins = new Map();
  for (const l of log) {
    if (!Number.isFinite(l.ms)) continue;
    const same = twins.get(l.text);
    if (same) same.push(l);
    else twins.set(l.text, [l]);
  }
  const out = log.slice();
  let lvl = "?";
  for (const r of hist.lines) {
    // A headerless record continues the one before it, as on the stream.
    const m = LOG_LEVEL.exec(r.text);
    lvl = m ? m[1] : lvl;
    if (floorMs != null && msSince(r.ms, floorMs) <= 0) continue;
    const same = twins.get(r.text);
    const i = same ? same.findIndex((l) => Math.abs(msSince(l.ms, r.ms)) <= SAME_LINE_MS) : -1;
    if (i >= 0) {
      same.splice(i, 1);
      continue;
    }
    out.push({ id: nextId(), lvl, text: r.text, at: receivedAt - ((hist.now - r.ms) >>> 0), ms: r.ms, gen });
  }
  // Keyed relative to `now`, so the order survives the millis() wrap; a line with no uptime (a
  // stream without event ids) sorts after everything that has one.
  const key = (l) => (Number.isFinite(l.ms) ? msSince(l.ms, hist.now) : Infinity);
  return out
    .map((l, i) => ({ l, k: key(l), i }))
    .sort((a, b) => (a.k === b.k ? a.i - b.i : a.k - b.k))
    .map((x) => x.l);
}
