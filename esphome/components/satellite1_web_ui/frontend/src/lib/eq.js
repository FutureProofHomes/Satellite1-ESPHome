/**
 * The speaker EQ's maths and its text format.
 *
 * The design maths mirrors the XMOS design task (Satellite1-XMOS,
 * satellite-xmos-firmware/src/speaker_eq/speaker_eq_design.c): the same RBJ cookbook biquads at the
 * same 48 kHz, the same section count per filter, the same headroom walk. So the graph draws what
 * the speaker plays, and the page can tell before the XMOS does that a design will be refused. The
 * XMOS stays the authority; this is the preview.
 *
 * A profile here is { pre, bass, mid, tre, lim, atk, rel, preset, f }: the device's JSON
 * (GET /api/sat1/dsp), with each filter an object { t, on, hz, db, q } rather than its
 * [type, on, hz, db, q] array.
 */

export const FS = 48000;

/** speaker_eq_filter_type_t. The 24 dB/oct cuts cost two sections. */
export const T = Object.freeze({
  NONE: 0,
  PEAK: 1,
  LOW_SHELF: 2,
  HIGH_SHELF: 3,
  HP_Q: 4,
  LP_Q: 5,
  HP_BW12: 6,
  HP_BW24: 7,
  HP_LR12: 8,
  HP_LR24: 9,
  LP_BW12: 10,
  LP_BW24: 11,
  LP_LR12: 12,
  LP_LR24: 13,
});
const NUM_TYPES = 14;

/** The ranges the device accepts (satellite1_dsp/speaker_eq_protocol.h); it refuses anything outside. */
export const RANGE = Object.freeze({
  pre: [-24, 12],
  lim: [-12, 0],
  atk: [0.1, 100],
  rel: [5, 5000],
  hz: [20, 20000],
  db: [-30, 12],
  q: [0.1, 10],
});
/** Filters a profile can hold: the XMOS's 14 less the three Bass, Midrange and Treble occupy. */
export const MAX_FILTERS = 11;
export const MAX_SECTIONS = 16;
/** The worst level allowed between sections; a design over it is refused. */
export const HEADROOM_CAP_DB = 18;
/** Bass, Midrange and Treble: two shelves and a peak at fixed frequencies, sent as every slot's
 *  first three records (satellite1_dsp.h). */
export const TONE = Object.freeze({ bassHz: 120, midHz: 1000, trebleHz: 8000, q: 0.707, max: 9 });
/** A new filter, and the shape a cut's Q takes when it has none of its own. */
export const DEFAULT_Q = 0.707;

const Q_BUTTERWORTH = Math.SQRT1_2;
const Q_LR12 = 0.5;
const Q_BW24 = [0.5411961, 1.30656296];

const GRID_POINTS = 160;
const GRID_LOW_HZ = 10;
const GRID_HIGH_FRAC = 0.49;
const BOOST_MAG2 = 1.0001;
const LIMITER_SLACK_DB = 0.01;

export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export const usesGain = t => t === T.PEAK || t === T.LOW_SHELF || t === T.HIGH_SHELF;
export const usesQ = t => usesGain(t) || t === T.HP_Q || t === T.LP_Q;
export const isHighpass = t => t === T.HP_Q || (t >= T.HP_BW12 && t <= T.HP_LR24);
export const isLowpass = t => t === T.LP_Q || (t >= T.LP_BW12 && t <= T.LP_LR24);

/** How many biquad sections a filter costs: none when it is off, or a gain filter at 0 dB. */
export function sectionsOf(f) {
  if (!f || f.t === T.NONE || !f.on) return 0;
  if (usesGain(f.t) && Math.abs(f.db) < 0.01) return 0;
  return f.t === T.HP_BW24 || f.t === T.HP_LR24 || f.t === T.LP_BW24 || f.t === T.LP_LR24 ? 2 : 1;
}

/** Bass, Midrange and Treble as the slot's first three records, the way satellite1_dsp sends them. */
export const toneFilters = p => [
  { t: T.LOW_SHELF, on: true, hz: TONE.bassHz, db: p.bass, q: TONE.q },
  { t: T.PEAK, on: true, hz: TONE.midHz, db: p.mid, q: TONE.q },
  { t: T.HIGH_SHELF, on: true, hz: TONE.trebleHz, db: p.tre, q: TONE.q },
];
export const slotFilters = p => [...toneFilters(p), ...p.f];
export const profileSections = p => slotFilters(p).reduce((n, f) => n + sectionsOf(f), 0);

/* ------------------------------------------------------------------ */
/* Design                                                              */
/* ------------------------------------------------------------------ */

/** One RBJ cookbook biquad, normalised so a0 = 1: { b0, b1, b2, a1, a2, fc }. */
function rbj(kind, fc, q, db, fs) {
  const w0 = (2 * Math.PI * fc) / fs;
  const cw = Math.cos(w0);
  const alpha = Math.sin(w0) / (2 * q);
  const a = 10 ** (db / 40);
  const k = 2 * Math.sqrt(a) * alpha;
  let b0, b1, b2, a0, a1, a2;
  switch (kind) {
    case 'lp':
      b0 = 0.5 * (1 - cw);
      b1 = 1 - cw;
      b2 = b0;
      a0 = 1 + alpha;
      a1 = -2 * cw;
      a2 = 1 - alpha;
      break;
    case 'hp':
      b0 = 0.5 * (1 + cw);
      b1 = -(1 + cw);
      b2 = b0;
      a0 = 1 + alpha;
      a1 = -2 * cw;
      a2 = 1 - alpha;
      break;
    case 'peak':
      b0 = 1 + alpha * a;
      b1 = -2 * cw;
      b2 = 1 - alpha * a;
      a0 = 1 + alpha / a;
      a1 = -2 * cw;
      a2 = 1 - alpha / a;
      break;
    case 'ls':
      b0 = a * (a + 1 - (a - 1) * cw + k);
      b1 = 2 * a * (a - 1 - (a + 1) * cw);
      b2 = a * (a + 1 - (a - 1) * cw - k);
      a0 = a + 1 + (a - 1) * cw + k;
      a1 = -2 * (a - 1 + (a + 1) * cw);
      a2 = a + 1 + (a - 1) * cw - k;
      break;
    default:
      b0 = a * (a + 1 + (a - 1) * cw + k);
      b1 = -2 * a * (a - 1 + (a + 1) * cw);
      b2 = a * (a + 1 + (a - 1) * cw - k);
      a0 = a + 1 - (a - 1) * cw + k;
      a1 = 2 * (a - 1 - (a + 1) * cw);
      a2 = a + 1 - (a - 1) * cw - k;
  }
  return { b0: b0 / a0, b1: b1 / a0, b2: b2 / a0, a1: a1 / a0, a2: a2 / a0, fc };
}

/** A filter's biquad sections, as the XMOS designs them. */
export function biquads(f, fs = FS) {
  if (!sectionsOf(f)) return [];
  const { hz, q, db } = f;
  switch (f.t) {
    case T.PEAK: return [rbj('peak', hz, q, db, fs)];
    case T.LOW_SHELF: return [rbj('ls', hz, q, db, fs)];
    case T.HIGH_SHELF: return [rbj('hs', hz, q, db, fs)];
    case T.HP_Q: return [rbj('hp', hz, q, 0, fs)];
    case T.LP_Q: return [rbj('lp', hz, q, 0, fs)];
    case T.HP_BW12: return [rbj('hp', hz, Q_BUTTERWORTH, 0, fs)];
    case T.HP_LR12: return [rbj('hp', hz, Q_LR12, 0, fs)];
    case T.HP_BW24: return Q_BW24.map(qq => rbj('hp', hz, qq, 0, fs));
    case T.HP_LR24: return [rbj('hp', hz, Q_BUTTERWORTH, 0, fs), rbj('hp', hz, Q_BUTTERWORTH, 0, fs)];
    case T.LP_BW12: return [rbj('lp', hz, Q_BUTTERWORTH, 0, fs)];
    case T.LP_LR12: return [rbj('lp', hz, Q_LR12, 0, fs)];
    case T.LP_BW24: return Q_BW24.map(qq => rbj('lp', hz, qq, 0, fs));
    case T.LP_LR24: return [rbj('lp', hz, Q_BUTTERWORTH, 0, fs), rbj('lp', hz, Q_BUTTERWORTH, 0, fs)];
    default: return [];
  }
}

const trig = (hz, fs) => {
  const w = (2 * Math.PI * hz) / fs;
  const c = Math.cos(w);
  const s = Math.sin(w);
  return [c, s, 2 * c * c - 1, 2 * s * c];
};

function mag2(s, [cos1, sin1, cos2, sin2]) {
  const nr = s.b0 + s.b1 * cos1 + s.b2 * cos2;
  const ni = s.b1 * sin1 + s.b2 * sin2;
  const dr = 1 + s.a1 * cos1 + s.a2 * cos2;
  const di = s.a1 * sin1 + s.a2 * sin2;
  return (nr * nr + ni * ni) / Math.max(dr * dr + di * di, 1e-20);
}

const toDb = m2 => 10 * Math.log10(Math.max(m2, 1e-12));

/* ------------------------------------------------------------------ */
/* Response                                                            */
/* ------------------------------------------------------------------ */

/** n log-spaced frequencies from lo to hi, the graph's x axis. */
export function logFreqs(n = 200, lo = 20, hi = 20000) {
  const r = Math.log(hi / lo) / (n - 1);
  return Array.from({ length: n }, (_, i) => lo * Math.exp(r * i));
}

/** One filter's gain in dB at each frequency. */
export function filterCurve(f, freqs, fs = FS) {
  const secs = biquads(f, fs);
  return freqs.map(hz => {
    const t = trig(hz, fs);
    return toDb(secs.reduce((m, s) => m * mag2(s, t), 1));
  });
}

/** The whole profile's gain in dB at each frequency: preamp, Bass, Midrange, Treble and every filter. */
export function profileCurve(p, freqs, fs = FS) {
  const secs = slotFilters(p).flatMap(f => biquads(f, fs));
  return freqs.map(hz => {
    const t = trig(hz, fs);
    return p.pre + toDb(secs.reduce((m, s) => m * mag2(s, t), 1));
  });
}

/**
 * The XMOS's analysis of a profile, walked the same way: a 160-point log grid from 10 Hz plus every
 * section's own frequency, cuts before boosts, a negative preamp before the filters and a positive
 * one after.
 *   sections  how many of the 16 it uses
 *   maxGain   the loudest the net response gets, in dB
 *   peakGain  the loudest any point between sections gets; over HEADROOM_CAP_DB the XMOS refuses it
 *   limiter   whether loud music can reach the limiter's ceiling (the XMOS's limiter_required)
 *   overCap   whether the XMOS will refuse it
 */
export function analyse(p, fs = FS) {
  const secs = [];
  for (const f of slotFilters(p)) {
    if (secs.length + sectionsOf(f) > MAX_SECTIONS) break;
    secs.push(...biquads(f, fs));
  }
  const ratio = Math.exp(Math.log((GRID_HIGH_FRAC * fs) / GRID_LOW_HZ) / (GRID_POINTS - 1));
  const grid = Array.from({ length: GRID_POINTS }, (_, i) => GRID_LOW_HZ * ratio ** i).concat(secs.map(s => s.fc));
  const pts = grid.map(hz => trig(hz, fs));
  const peaks = secs.map(s => pts.reduce((m, t) => Math.max(m, mag2(s, t)), 0));
  const order = secs.map((_, i) => i);
  const sorted = order.filter(i => peaks[i] <= BOOST_MAG2).concat(order.filter(i => peaks[i] > BOOST_MAG2));
  const preM = 10 ** (Math.min(p.pre, 0) / 10);
  const postM = 10 ** (Math.max(p.pre, 0) / 10);
  let worst = 0;
  let net = 0;
  for (const t of pts) {
    let level = preM;
    worst = Math.max(worst, level);
    for (const i of sorted) {
      level *= mag2(secs[i], t);
      worst = Math.max(worst, level);
    }
    level *= postM;
    worst = Math.max(worst, level);
    net = Math.max(net, level);
  }
  const maxGain = toDb(net);
  const peakGain = toDb(worst);
  return {
    sections: profileSections(p),
    maxGain,
    peakGain,
    // A profile with nothing to do is bypassed outright, limiter included.
    limiter: (secs.length > 0 || p.pre !== 0) && maxGain > p.lim + LIMITER_SLACK_DB,
    overCap: peakGain > HEADROOM_CAP_DB,
  };
}

/**
 * The most preamp with which the limiter never has to act: the loudest frequency lands on the
 * limiter's ceiling. Lower still if a point between sections would pass the +18 dB cap, which the
 * net level doesn't show (a negative preamp scales every point; a positive one only the last).
 * Rounded down to a tenth of a dB.
 */
export function autoPreamp(p) {
  let pre = p.pre - analyse(p).maxGain + p.lim;
  const { peakGain } = analyse({ ...p, pre: Math.min(pre, 0) });
  if (peakGain > HEADROOM_CAP_DB) pre = Math.min(pre, Math.min(pre, 0) - (peakGain - HEADROOM_CAP_DB));
  return clamp(Math.floor(pre * 10 + 1e-6) / 10, ...RANGE.pre);
}

/**
 * A log slider's value at track position `pos` (0-1000 between min and max), rounded by `round`.
 * One track step is finer than `round` in places, so when the position's value rounds back to
 * `from` - the value at `fromPos` - it goes on to the next value that differs, in the direction of
 * travel: an arrow key always moves the value.
 */
export function logTrackValue(pos, from, fromPos, min, max, round) {
  const at = q => round(min * (max / min) ** (q / 1000));
  const dir = pos > fromPos ? 1 : -1;
  for (let q = pos; q >= 0 && q <= 1000; q += dir) {
    const v = at(q);
    if (v !== from) return v;
  }
  return from;
}

/* ------------------------------------------------------------------ */
/* Kinds, for the editor                                               */
/* ------------------------------------------------------------------ */

/** The five things a filter can be, in the editor's order. Cuts also have a slope. */
export const KINDS = ['peak', 'lowshelf', 'highshelf', 'lowcut', 'highcut'];
/** Cut slopes: Butterworth and Linkwitz-Riley at 12 and 24 dB/oct, and 12 dB/oct with its own Q. */
export const SLOPES = ['bw12', 'bw24', 'lr12', 'lr24', 'q'];
const HP_SLOPES = { bw12: T.HP_BW12, bw24: T.HP_BW24, lr12: T.HP_LR12, lr24: T.HP_LR24, q: T.HP_Q };
const LP_SLOPES = { bw12: T.LP_BW12, bw24: T.LP_BW24, lr12: T.LP_LR12, lr24: T.LP_LR24, q: T.LP_Q };

export function kindOf(t) {
  if (t === T.LOW_SHELF) return 'lowshelf';
  if (t === T.HIGH_SHELF) return 'highshelf';
  if (isHighpass(t)) return 'lowcut';
  if (isLowpass(t)) return 'highcut';
  return 'peak';
}

export function slopeOf(t) {
  const table = isHighpass(t) ? HP_SLOPES : isLowpass(t) ? LP_SLOPES : null;
  return table ? Object.keys(table).find(k => table[k] === t) : null;
}

/** The type for a kind, keeping `slope` for cuts. */
export function typeOf(kind, slope = 'bw12') {
  if (kind === 'lowshelf') return T.LOW_SHELF;
  if (kind === 'highshelf') return T.HIGH_SHELF;
  if (kind === 'lowcut') return HP_SLOPES[slope] ?? T.HP_BW12;
  if (kind === 'highcut') return LP_SLOPES[slope] ?? T.LP_BW12;
  return T.PEAK;
}

/** Where the graph puts a filter's handle: its own response at its frequency. */
export function handleDb(f) {
  if (f.t === T.PEAK) return f.db;
  if (f.t === T.LOW_SHELF || f.t === T.HIGH_SHELF) return f.db / 2;
  return filterCurve({ ...f, on: true }, [f.hz])[0];
}

/** "138 Hz", "1.6 kHz", "13.7 kHz". */
export function fmtHz(hz) {
  if (hz < 999.5) return `${Math.round(hz)} Hz`;
  const k = hz / 1000;
  return `${+k.toFixed(k < 10 ? 2 : 1)} kHz`;
}

/** "+7.0 dB", "−3.0 dB" (a true minus), "0 dB". */
export function fmtDb(db, digits = 1) {
  const v = +db.toFixed(digits);
  if (v === 0) return '0 dB';
  return `${v > 0 ? '+' : '\u2212'}${Math.abs(v).toFixed(digits)} dB`;
}

/* ------------------------------------------------------------------ */
/* The device's JSON and form body                                     */
/* ------------------------------------------------------------------ */

const num = (v, dflt = 0) => (Number.isFinite(Number(v)) ? Number(v) : dflt);

/** A profile from GET /api/sat1/dsp. */
export function fromWire(w) {
  return {
    pre: num(w?.pre),
    bass: num(w?.bass),
    mid: num(w?.mid),
    tre: num(w?.tre),
    lim: num(w?.lim, -1),
    atk: num(w?.atk, 1),
    rel: num(w?.rel, 100),
    preset: typeof w?.preset === 'string' ? w.preset : '',
    f: (Array.isArray(w?.f) ? w.f : [])
      .filter(r => Array.isArray(r) && r.length >= 5 && num(r[0]) > 0 && num(r[0]) < NUM_TYPES)
      .map(([t, on, hz, db, q]) => ({ t: num(t), on: !!on, hz: num(hz, 1000), db: num(db), q: num(q, DEFAULT_Q) || DEFAULT_Q })),
  };
}

/** Within the device's ranges, so a write is never refused for a value the page let through. */
export function clampProfile(p) {
  return {
    ...p,
    pre: clamp(p.pre, ...RANGE.pre),
    bass: clamp(p.bass, -TONE.max, TONE.max),
    mid: clamp(p.mid, -TONE.max, TONE.max),
    tre: clamp(p.tre, -TONE.max, TONE.max),
    lim: clamp(p.lim, ...RANGE.lim),
    atk: clamp(p.atk, ...RANGE.atk),
    rel: clamp(p.rel, ...RANGE.rel),
    f: p.f.slice(0, MAX_FILTERS).map(f => ({
      ...f,
      hz: clamp(f.hz, ...RANGE.hz),
      db: usesGain(f.t) ? clamp(f.db, ...RANGE.db) : 0,
      q: usesQ(f.t) ? clamp(f.q, ...RANGE.q) : DEFAULT_Q,
    })),
  };
}

const wireNum = v => String(Math.round(v * 1e4) / 1e4);

/** The f field of POST /api/sat1/dsp: "type,on,hz,db,q;..." in order. */
export const filtersParam = fs => fs.map(f => [f.t, f.on ? 1 : 0, wireNum(f.hz), wireNum(f.db), wireNum(f.q)].join(',')).join(';');

/** Every field of POST /api/sat1/dsp for one profile, clamped. */
export function profileBody(p) {
  const c = clampProfile(p);
  return {
    pre: wireNum(c.pre),
    bass: wireNum(c.bass),
    mid: wireNum(c.mid),
    tre: wireNum(c.tre),
    lim: wireNum(c.lim),
    atk: wireNum(c.atk),
    rel: wireNum(c.rel),
    f: filtersParam(c.f),
  };
}

const near = (a, b) => Math.abs(a - b) < 0.005;

/** Whether two profiles sound the same: everything but the preset's name. */
export function sameSound(a, b) {
  if (!a || !b || a.f.length !== b.f.length) return false;
  for (const k of ['pre', 'bass', 'mid', 'tre', 'lim', 'atk', 'rel']) if (!near(a[k], b[k])) return false;
  return a.f.every((f, i) => {
    const g = b.f[i];
    return f.t === g.t && f.on === g.on && near(f.hz, g.hz) && near(usesGain(f.t) ? f.db : 0, usesGain(g.t) ? g.db : 0) && near(usesQ(f.t) ? f.q : 0, usesQ(g.t) ? g.q : 0);
  });
}

/* ------------------------------------------------------------------ */
/* REW / Equalizer APO / Music Assistant text                          */
/* ------------------------------------------------------------------ */

const PAIRS = [
  { qs: [0.541, 1.307], hp: T.HP_BW24, lp: T.LP_BW24 },
  { qs: [0.707, 0.707], hp: T.HP_LR24, lp: T.LP_LR24 },
];

const typeToken = f => (f.t === T.PEAK ? 'PK' : f.t === T.LOW_SHELF ? 'LSC' : f.t === T.HIGH_SHELF ? 'HSC' : isHighpass(f.t) ? 'HP' : 'LP');

/** The Q of each line a filter exports as: a 24 dB/oct cut is two 12 dB/oct lines. */
function exportQs(f) {
  switch (f.t) {
    case T.HP_BW12: case T.LP_BW12: return [DEFAULT_Q];
    case T.HP_LR12: case T.LP_LR12: return [Q_LR12];
    case T.HP_BW24: case T.LP_BW24: return PAIRS[0].qs;
    case T.HP_LR24: case T.LP_LR24: return PAIRS[1].qs;
    default: return [f.q];
  }
}

/**
 * The profile as a REW / Equalizer APO filter file, in the exact line format Music Assistant
 * exports and imports: Bass, Midrange and Treble first, as a Q shelf, a peak and a Q shelf at their
 * fixed frequencies, then every filter. Any of the three at 0 dB is left out. The limiter has no
 * place in the format and stays behind.
 */
export function exportText(p) {
  const recs = toneFilters(p).filter(f => Math.abs(f.db) >= 0.01);
  for (const f of p.f) for (const q of exportQs(f)) recs.push({ ...f, q });
  const lines = [`Preamp: ${p.pre.toFixed(2)} dB`];
  recs.forEach((f, i) => {
    const db = usesGain(f.t) ? f.db : 0;
    lines.push(`Filter ${i + 1}: ${f.on ? 'ON' : 'OFF'} ${typeToken(f)} Fc ${f.hz.toFixed(1)} Hz Gain ${db.toFixed(2)} dB Q ${f.q.toFixed(3)}`);
  });
  return `${lines.join('\n')}\n`;
}

// Numbers take a decimal point or, from comma-decimal locales, a comma.
const N = String.raw`(\d+(?:[.,]\d*)?|[.,]\d+)`;
const LINE = /^\s*Filter(?:\s*\d+)?\s*:\s*(ON|OFF)\s+([A-Za-z]+\d?)(?:\s+(\d+)\s*dB(?![a-z]))?(.*)$/i;
const PREAMP = new RegExp(String.raw`^\s*Preamp\s*:\s*([-+]?)${N}\s*dB`, 'i');
const FC = new RegExp(String.raw`\bFc\s+${N}\s*(k?)Hz`, 'i');
const GAIN = new RegExp(String.raw`\bGain\s+([-+]?)${N}\s*dB`, 'i');
const Q = new RegExp(String.raw`\bQ\s+${N}`, 'i');
const BW_OCT = new RegExp(String.raw`\bBW\s+Oct\s+${N}`, 'i');

const dec = s => Number(s.replace(',', '.'));
const signed = (sign, s) => (sign === '-' ? -1 : 1) * dec(s);
const octToQ = n => Math.sqrt(2 ** n) / (2 ** n - 1);

/**
 * A REW / Equalizer APO / Music Assistant filter file as a profile's sound:
 * { pre, bass, mid, tre, f, notes } or null when the text has no preamp or filter line at all.
 *
 * Peaks, Q shelves and 12 dB/oct cuts carry over exactly. A cut line with a Q is "12 dB/oct, custom
 * Q", one without is Butterworth, and one at Butterworth's or Linkwitz-Riley's own Q is that slope;
 * two cut lines at one frequency with a Butterworth or Linkwitz-Riley 24 dB/oct pair of Qs become
 * that one filter, as exportText writes it. A Q shelf at Bass's or Treble's own frequency and Q, or
 * a peak at Midrange's, becomes that control.
 *
 * `notes` says what did not carry over exactly, as codes the page words:
 *   { k: 'skip', line, type }    a filter type the speaker doesn't have (notch, all-pass, 6 dB/oct...)
 *   { k: 'unread', line }        a filter whose frequency couldn't be read, left out
 *   { k: 'nogain', line }        a boost or cut whose gain couldn't be read, imported at 0 dB
 *   { k: 'approx', line, type }  a shelf given by slope rather than Q, imported with Q 0.707
 *   { k: 'clamp', line }         a value outside the speaker's range, brought inside it
 *   { k: 'many', kept }          more filters than fit; the first `kept` were kept
 *   { k: 'budget', kept }        more sections than fit; the first `kept` filters were kept
 */
export function importText(text) {
  const notes = [];
  let pre = 0;
  let seen = false;
  const raw = [];
  String(text).split(/\r?\n/).forEach((s, idx) => {
    const line = idx + 1;
    const pm = PREAMP.exec(s);
    if (pm) {
      seen = true;
      const v = signed(pm[1], pm[2]);
      pre = clamp(v, ...RANGE.pre);
      if (pre !== v) notes.push({ k: 'clamp', line });
      return;
    }
    const m = LINE.exec(s);
    if (!m) return;
    seen = true;
    const on = m[1].toUpperCase() === 'ON';
    const tok = m[2].toUpperCase();
    const slope = m[3];
    const rest = m[4];
    if (tok === 'NONE') return;
    const fc = FC.exec(rest);
    const gain = GAIN.exec(rest);
    const qm = Q.exec(rest);
    const bw = BW_OCT.exec(rest);
    const q = qm ? dec(qm[1]) : bw ? octToQ(dec(bw[1])) : null;
    // A cut is 12 dB/oct whether or not the line says so; any other slope isn't available here.
    const cut12 = !slope || slope === '12';
    let t;
    if (tok === 'PK' || tok === 'PEQ' || tok === 'MODAL') t = T.PEAK;
    else if (tok === 'LS' || tok === 'LSC' || tok === 'LSQ') t = T.LOW_SHELF;
    else if (tok === 'HS' || tok === 'HSC' || tok === 'HSQ') t = T.HIGH_SHELF;
    else if ((tok === 'HP' || tok === 'HPQ') && cut12) t = q != null ? T.HP_Q : T.HP_BW12;
    else if ((tok === 'LP' || tok === 'LPQ') && cut12) t = q != null ? T.LP_Q : T.LP_BW12;
    if (t === undefined) {
      notes.push({ k: 'skip', line, type: slope ? `${m[2]} ${slope}dB` : m[2] });
      return;
    }
    if (!fc) {
      notes.push({ k: 'unread', line });
      return;
    }
    if (usesGain(t) && !gain) notes.push({ k: 'nogain', line });
    if (usesGain(t) && q == null && t !== T.PEAK) notes.push({ k: 'approx', line, type: m[2] });
    const hz = dec(fc[1]) * (fc[2] ? 1000 : 1);
    const f = { t, on, hz, db: gain ? signed(gain[1], gain[2]) : 0, q: q ?? (t === T.PEAK ? 1 : DEFAULT_Q), line };
    const c = clampProfile({ pre: 0, bass: 0, mid: 0, tre: 0, lim: -1, atk: 1, rel: 100, f: [f] }).f[0];
    if (c.hz !== f.hz || (usesGain(t) && c.db !== f.db) || (usesQ(t) && c.q !== f.q)) notes.push({ k: 'clamp', line });
    raw.push({ ...c, line });
  });
  if (!seen) return null;

  const filters = [];
  for (let i = 0; i < raw.length; i++) {
    const a = raw[i];
    const b = raw[i + 1];
    const pair = b && a.on === b.on && Math.abs(a.hz - b.hz) < 0.05 && ((a.t === T.HP_Q && b.t === T.HP_Q) || (a.t === T.LP_Q && b.t === T.LP_Q))
      ? PAIRS.find(({ qs }) => (near2(a.q, qs[0]) && near2(b.q, qs[1])) || (near2(a.q, qs[1]) && near2(b.q, qs[0])))
      : null;
    if (pair) {
      filters.push({ t: a.t === T.HP_Q ? pair.hp : pair.lp, on: a.on, hz: a.hz, db: 0, q: DEFAULT_Q });
      i++;
    } else {
      const t = singleCut(a);
      filters.push({ t, on: a.on, hz: a.hz, db: a.db, q: t === a.t ? a.q : DEFAULT_Q });
    }
  }

  const isTone = (f, t, hz) => f.t === t && f.on && Math.abs(f.hz - hz) < 0.5 && Math.abs(f.q - TONE.q) < 0.005 && Math.abs(f.db) <= TONE.max;
  const tone = (t, hz) => {
    const i = filters.findIndex(f => isTone(f, t, hz));
    return i >= 0 ? filters.splice(i, 1)[0].db : 0;
  };
  const bass = tone(T.LOW_SHELF, TONE.bassHz);
  const mid = tone(T.PEAK, TONE.midHz);
  const tre = tone(T.HIGH_SHELF, TONE.trebleHz);

  let f = filters;
  if (f.length > MAX_FILTERS) {
    f = f.slice(0, MAX_FILTERS);
    notes.push({ k: 'many', kept: MAX_FILTERS });
  }
  const out = { pre, bass, mid, tre, lim: -1, atk: 1, rel: 100, preset: '', f };
  if (profileSections(out) > MAX_SECTIONS) {
    while (f.length && profileSections({ ...out, f }) > MAX_SECTIONS) f = f.slice(0, -1);
    notes.push({ k: 'budget', kept: f.length });
  }
  return { pre, bass, mid, tre, f, notes };
}

function near2(a, b) {
  return Math.abs(a - b) < 0.01;
}

/** A 12 dB/oct cut line at Butterworth's or Linkwitz-Riley's own Q is that slope, as exported. */
function singleCut(f) {
  if (f.t !== T.HP_Q && f.t !== T.LP_Q) return f.t;
  const hp = f.t === T.HP_Q;
  if (near2(f.q, Q_BUTTERWORTH)) return hp ? T.HP_BW12 : T.LP_BW12;
  if (near2(f.q, Q_LR12)) return hp ? T.HP_LR12 : T.LP_LR12;
  return f.t;
}
