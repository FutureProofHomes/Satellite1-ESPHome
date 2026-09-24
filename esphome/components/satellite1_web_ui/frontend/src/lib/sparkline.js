/**
 * The maths behind the Home sensor chips' sparklines (ui.jsx's Spark): raw [ts, value] history in,
 * two SVG path strings out. Pure and DOM-free on purpose, so the preview tooling and any future
 * test can run the exact production pipeline under plain node.
 *
 * The pipeline is mini-graph-card's (github.com/kalkih/mini-graph-card, src/graph.js), adapted:
 * raw samples are grouped into a fixed small number of time buckets and each bucket collapses to
 * its average (their _reducer + _average with points_per_hour), and the line is their getPath
 * smoothing - quadratic béziers through consecutive bucket midpoints with the bucket points as
 * control points, which rounds every peak and can never overshoot the data. The first cut of this
 * chart skipped the bucketing and drew every stored sample: a sensor dithering between two
 * adjacent display steps stores a point per flip (sparkhist.js records every changed value on
 * arrival), so up to 240 near-vertical zigzags were squeezed into ~100px of chip and scaled to a
 * y-range the size of that dither - a barcode, not a curve. Bucket averages are what make sixty
 * noisy samples read as one gentle line; no smoothing algorithm can rescue sub-pixel spacing.
 */

/** The window and cadence mirror lib/sparkhist.js's buffer: 3 hours, 60s synthetic beat. */
const WINDOW_MS = 3 * 60 * 60 * 1000;
const STEP_MS = 60 * 1000;
/** Stored samples further apart than this are a closed-tab outage, bridged by synthetic points so
 *  the reopened chip shows one continuous quiet line instead of a cliff. */
const GAP_MS = 10 * 60 * 1000;
/** How many averaged points the curve is drawn through - mini-graph-card's hours_to_show ×
 *  points_per_hour, here 3h × 8/hour. Twenty-four is enough for a shape and few enough that
 *  neighbouring points are ~4 viewBox units apart, which is where bézier smoothing actually has
 *  room to curve. The raw ring stays at full resolution; this is render-side only. */
const BUCKETS = 24;

/** FNV-1a over the seed string ("mac:key") into two phases - the synthetic backfill is identical
 *  across renders and reloads, so a refresh never shows a different past. */
function sparkPhases(seed) {
  let h = 2166136261;
  for (let i = 0; i < seed.length; i++) {
    h ^= seed.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  const a = (h >>> 0) / 4294967296;
  const b = (Math.imul(h, 2654435761) >>> 0) / 4294967296;
  return [a * 2 * Math.PI, b * 2 * Math.PI];
}

/** The backfill's shape: two slow sinusoids, so the synthetic span undulates like a room rather
 *  than jittering like noise. Both periods sit well above the ~7.5min bucket span - the first
 *  cut's 13min second voice flipped sign nearly every bucket and survived averaging as a zigzag,
 *  which is a spike generator, not a texture. At 100min and 210min the window holds roughly two
 *  gentle hills and a slow drift. Amplitude is the sensor's own display step - deliberately
 *  sub-perceptual, less than the sensor's natural drift. */
const sparkWobble = (t, amp, p1, p2) =>
  amp * (0.6 * Math.sin((2 * Math.PI * t) / (100 * 60000) + p1) + 0.4 * Math.sin((2 * Math.PI * t) / (210 * 60000) + p2));

/**
 * The full series to bucket: real points, plus the quiet deterministic synthetic prefix from the
 * window's start to the first real point (cold start - user decision, September 2026: no growing
 * stub), plus bridges across mid-history gaps. Anchored at the neighbouring real values, so the
 * seams are invisible. The synthetic points ride the same buckets as the real ones below, which is
 * what keeps the cold-start look identical to the warmed-up look: one gentle curve.
 */
function sparkSeries(pts, amp, seed, now) {
  const start = now - WINDOW_MS;
  const real = pts.filter((p) => p[0] >= start && p[0] <= now);
  if (!real.length) return null;
  const [p1, p2] = sparkPhases(seed);
  const out = [];
  const first = real[0];
  for (let t = Math.ceil(start / STEP_MS) * STEP_MS; t < first[0]; t += STEP_MS) {
    out.push([t, first[1] + sparkWobble(t, amp, p1, p2)]);
  }
  for (let i = 0; i < real.length; i++) {
    const p = real[i];
    if (i > 0) {
      const prev = real[i - 1];
      if (p[0] - prev[0] > GAP_MS) {
        for (let t = prev[0] + STEP_MS; t < p[0]; t += STEP_MS) {
          const f = (t - prev[0]) / (p[0] - prev[0]);
          out.push([t, prev[1] + (p[1] - prev[1]) * f + sparkWobble(t, amp, p1, p2)]);
        }
      }
    }
    out.push(p);
  }
  return out;
}

/**
 * mini-graph-card's grouping (graph.js's _reducer/_calcPoints): the window sliced into BUCKETS
 * equal spans, every sample averaged into its span, empty spans carrying the last seen average
 * (their _lastValue) so an outage cannot punch a hole in the path. Returns one value per bucket,
 * oldest first.
 */
function sparkBuckets(series, start, now) {
  const span = (now - start) / BUCKETS;
  const sums = new Array(BUCKETS).fill(0);
  const counts = new Array(BUCKETS).fill(0);
  for (const [t, v] of series) {
    const i = Math.min(BUCKETS - 1, Math.max(0, Math.floor((t - start) / span)));
    sums[i] += v;
    counts[i] += 1;
  }
  const vals = [];
  let last = null;
  for (let i = 0; i < BUCKETS; i++) {
    if (counts[i]) last = sums[i] / counts[i];
    if (last !== null) vals.push(last);
  }
  return vals;
}

const f = (n) => n.toFixed(2);

/** mini-graph-card's getPath smoothing: quadratic béziers running midpoint to midpoint with the
 *  data points as controls, so every peak comes out rounded and the curve never overshoots its
 *  data the way Catmull-Rom can. The first and last half-segments are straight, exactly as their
 *  degenerate leading Z and trailing Q render. */
function sparkPathD(xy) {
  if (xy.length < 2) return `M0 ${f(xy[0][1])}L100 ${f(xy[0][1])}`;
  let d = `M${f(xy[0][0])} ${f(xy[0][1])}`;
  for (let i = 1; i < xy.length; i++) {
    const mx = (xy[i - 1][0] + xy[i][0]) / 2;
    const my = (xy[i - 1][1] + xy[i][1]) / 2;
    d += `Q${f(xy[i - 1][0])} ${f(xy[i - 1][1])} ${f(mx)} ${f(my)}`;
  }
  const last = xy[xy.length - 1];
  d += `L${f(last[0])} ${f(last[1])}`;
  return d;
}

/**
 * The whole pipeline: history in, `{ line, fill }` viewBox-space (0-100 × 0-100) path strings out,
 * or null when there is nothing real to draw.
 *
 * The y-scale belongs to the bucketed values (they are the curve; scaling to anything else could
 * clip it), padded a quarter of the range each side so peaks never slam the band's edges - and
 * when the range is smaller than the sensor's own display step (a genuinely calm room), the pad
 * becomes that step, so a near-flat series draws as a near-flat line mid-band instead of noise
 * stretched to full height.
 */
export function sparkPaths(pts, amp, seed, now = Date.now()) {
  const series = pts && pts.length ? sparkSeries(pts, amp, seed, now) : null;
  if (!series) return null;
  const start = now - WINDOW_MS;
  const vals = sparkBuckets(series, start, now);
  let lo = Infinity;
  let hi = -Infinity;
  for (const v of vals) {
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  const pad = Math.max((hi - lo) * 0.25, amp || 1);
  lo -= pad;
  hi += pad;
  const xy = vals.map((v, i) => [(i / (vals.length - 1 || 1)) * 100, 100 - ((v - lo) / (hi - lo)) * 100]);
  const line = sparkPathD(xy);
  const fill = `${line}L100 100L0 100Z`;
  return { line, fill };
}
