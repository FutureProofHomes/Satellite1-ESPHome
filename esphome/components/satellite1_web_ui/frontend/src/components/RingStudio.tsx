import { useEffect, useMemo, useRef, useState } from 'react';
import { RING } from '../copy.js';
import { entity, pathFor, post } from '../lib/device.js';
import type { Ctx } from '../ctx';
import { ChevronLeft, ChevronRight, Info, Lock, Play, Plus, X } from '../icons';
import { hsvToRgb, isOn, pctTo255, rgbHue } from '../lib/orb.js';
import {
  CM, CM_BLEND, CM_OWN, CM_RAINBOW, CM_RING, FX, FX_BREATHE, FX_COMET, FX_DOT, FX_FLOW, FX_OFF, FX_ORBIT, FX_PULSE,
  FX_RIPPLE, FX_SOLID, FX_SPIN, FX_TWINKLE, FX_WAVE, F_FIXED, F_REV, MOMENTS, M_ERR, M_LISTEN, M_MUTE, M_REPLY, M_RING,
  M_THINK, M_TIMER, M_VOL, M_WAKE, N, PRESETS, STYLES, clampStyle, hexRgb, momentTag, normalizeRing, resolve, rgbHex,
} from '../lib/ringfx.js';
import { DATA_MOMENTS, fixedRun, liveRing, presetStyle, ringApi, ringRgb } from '../lib/ring.js';
import { toast } from '../lib/toast.js';
import { LedRange } from './LedRange';
import { FixedRing, MomentRing, RingCanvas } from './RingPreview';

/**
 * The LED ring's pages in the Customize drawer: the LED Ring page (color, brightness, style), the
 * animations list, one moment's editor, and the guide to what the lights mean.
 *
 * The ring's color and brightness are the LED Ring light's, written through its REST entity
 * exactly as Home Assistant writes them. Styles and moments are satellite1_ring's, through
 * /api/sat1/ring. What the device plays follows the page: the LED Ring page runs the conversation
 * round and round, so every color and style pick shows on the ring as it is made; the animations
 * list is dark until a moment is previewed or toured; the editor runs its draft until it closes.
 */

export type Style = { fx: number; cm: number; n: number; fl: number; sp: number; br: number; p: number; c: number[][] };
export type RingData = {
  style: string;
  base: string;
  moment: string;
  preview: string;
  in: { ratio: number; mic: boolean; spk: boolean };
  m: Style[];
};
export type RingView = { page: 'ring' | 'moments' | 'edit' | 'guide'; m?: number };

/** The ring's named colors: the orb's palette, plus the warm white the orb has no use for. */
export const RING_SWATCHES = [
  { label: 'Warm white', hex: '#ffd9a0' },
  { label: 'Arctic', hex: '#38bdf8' },
  { label: 'Aurora', hex: '#34d399' },
  { label: 'Solar', hex: '#fbbf24' },
  { label: 'Rose', hex: '#fb7185' },
  { label: 'Ember', hex: '#fb923c' },
  { label: 'Sapphire', hex: '#60a5fa' },
  { label: 'Nebula', hex: '#c084fc' },
];

/** The swatch the light's color is, within what a round trip through the device does to it. */
export function ringColorName(light: any): string {
  const c = ringRgb(light);
  for (const s of RING_SWATCHES) {
    const w = normalizeRing(hexRgb(s.hex));
    if (Math.max(...w.map((v: number, i: number) => Math.abs(v - c[i]))) <= 40) return s.label;
  }
  return RING.custom;
}

export const styleName = (k: string) => (RING.styles[k] || RING.styles.classic)[0];
const momentName = (k: string) => RING.moments[k]?.[0] ?? RING.fixed[k]?.[0] ?? k;
const baseOf = (d: RingData) => (d.style === 'custom' ? d.base : PRESETS.includes(d.style) ? d.style : 'classic');
const fill = (s: string, base: string) => s.replace('{base}', styleName(base));
/** The light's brightness in percent, on or off: an off light keeps it, and the moments draw with it. */
const ringLevel = (light: any) => Math.max(1, Math.round(((light?.brightness ?? 255) / 255) * 100));
const speedWord = (sp: number) => RING.speeds[sp < 60 ? 0 : sp < 90 ? 1 : sp <= 120 ? 2 : sp <= 220 ? 3 : 4];

/**
 * Writes a color or brightness to the LED Ring light without changing whether it is on. The REST
 * entity only takes them as a turn_on, so while the light is off - as it was when these pages
 * opened - each pick is followed by a turn_off: the moments draw in the new color, and the ring
 * stays off at idle. Ends any preview when the pages close or the tab goes away.
 */
function useRingLight(ctx: Ctx) {
  const light = entity(ctx, 'ring');
  const was = useRef<boolean | null>(null);
  if (was.current === null && light) was.current = isOn(light);
  useEffect(() => {
    const gone = () => ringApi.stopNow();
    window.addEventListener('pagehide', gone);
    return () => {
      window.removeEventListener('pagehide', gone);
      ringApi.stop();
    };
  }, []);
  return (q: Record<string, string | number>) => {
    post(pathFor(ctx, 'ring', 'turn_on', q));
    if (was.current === false) post(pathFor(ctx, 'ring', 'turn_off'));
  };
}

/** Whether the tab is in view: a ring left playing for a background tab is one nobody watches. */
function useShown() {
  const [shown, setShown] = useState(!document.hidden);
  useEffect(() => {
    const f = () => setShown(!document.hidden);
    document.addEventListener('visibilitychange', f);
    return () => document.removeEventListener('visibilitychange', f);
  }, []);
  return shown;
}

const CONVERSATION = [M_WAKE, M_LISTEN, M_THINK, M_REPLY];
const TOUR_MS = 3000;

/**
 * Plays the conversation on the device, one moment every TOUR_MS, round and round while `on` and
 * the tab is in view. Each preview outlasts its step, so a slow request queue never lets the ring
 * fall dark between moments; the device draws the current style and color, so picks show at once.
 */
function useConversationTour(on: boolean) {
  const shown = useShown();
  const run = on && shown;
  const [step, setStep] = useState(0);
  useEffect(() => {
    if (!run) return undefined;
    ringApi.preview(MOMENTS[CONVERSATION[step]], TOUR_MS + 2000);
    const id = setTimeout(() => setStep(s => (s + 1) % CONVERSATION.length), TOUR_MS);
    return () => clearTimeout(id);
  }, [run, step]);
  useEffect(() => {
    if (!run) ringApi.stop();
  }, [run]);
  useEffect(() => () => {
    ringApi.stop();
  }, []);
}

/** The live ring's frames and the moment it is showing, polled while the moment draws device data. */
function useLive(ctx: Ctx, data: RingData | null, reload: () => void) {
  const light = entity(ctx, 'ring');
  const moment: string = entity(ctx, 'ring_moment')?.state || data?.moment || 'idle';
  const cur = useRef({ moment, data, light, on: isOn(light) });
  cur.current = { moment, data, light, on: isOn(light) };
  const frame = useMemo(() => liveRing(() => cur.current), []);
  const polls = DATA_MOMENTS.includes(moment);
  useEffect(() => {
    if (!polls) return undefined;
    reload();
    const id = setInterval(() => {
      if (!document.hidden) reload();
    }, 800);
    return () => clearInterval(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [polls]);
  return { frame, moment };
}

function Top({ back, onBack, onClose }: { back: string; onBack: () => void; onClose: () => void }) {
  return <div className="rs-top">
    <button className="rs-back" onClick={onBack}><ChevronLeft size={18} />{back}</button>
    <button className="dw-x" aria-label="Close" onClick={onClose}><X size={18} /></button>
  </div>;
}

/** A style's colors as dots: its stops, or a rainbow. */
function Dots({ s, ring }: { s: Style; ring: number[] }) {
  const p = resolve(s, ring);
  if (p.rainbow) return <span className="rs-dots"><i className="rs-dot rainbow" /></span>;
  return <span className="rs-dots">{p.c.map((c: number[], i: number) => <i key={i} className="rs-dot" style={{ background: rgbHex(c) }} />)}</span>;
}

const fxName = (fx: number) => RING.fx[FX[fx]] ?? FX[fx];
const cmName = (cm: number) => RING.cm[CM[cm]] ?? CM[cm];

export function RingStudio({ ctx, data, reload, view, go, onBack, onClose, onDirty }: {
  ctx: Ctx;
  /** undefined while loading, null when this firmware has no ring styles. */
  data: RingData | null | undefined;
  reload: () => Promise<void>;
  view: RingView;
  go: (v: RingView) => void;
  onBack: () => void;
  onClose: () => void;
  /** Whether the moment editor holds changes it has not saved: the only page with any. */
  onDirty: (dirty: boolean) => void;
}) {
  const turnOn = useRingLight(ctx);
  const live = useLive(ctx, data ?? null, reload);
  if (view.page === 'moments' && data) return <Moments key="moments" ctx={ctx} data={data} go={go} onClose={onClose} reload={reload} />;
  if (view.page === 'edit' && data && view.m != null) return <Editor key={'edit' + view.m} ctx={ctx} data={data} m={view.m} go={go} onClose={onClose} reload={reload} onDirty={onDirty} />;
  if (view.page === 'guide' && data) return <Guide key="guide" ctx={ctx} data={data} go={go} onClose={onClose} />;
  return <RingHome key="ring" ctx={ctx} data={data ?? null} live={live} turnOn={turnOn} go={go} onBack={onBack} onClose={onClose} reload={reload} />;
}

function RingHome({ ctx, data, live, turnOn, go, onBack, onClose, reload }: {
  ctx: Ctx;
  data: RingData | null;
  live: { frame: (now: number) => number[][] | null; moment: string };
  turnOn: (q: Record<string, string | number>) => void;
  go: (v: RingView) => void;
  onBack: () => void;
  onClose: () => void;
  reload: () => Promise<void>;
}) {
  useConversationTour(!!data);
  const light = entity(ctx, 'ring');
  const ring = ringRgb(light);
  const name = ringColorName(light);
  const [custom, setCustom] = useState(name === RING.custom);
  const pick = custom ? RING.custom : name;
  const pct = ringLevel(light);
  const [seenCustom, setSeenCustom] = useState(false);
  useEffect(() => {
    if (data?.style === 'custom') setSeenCustom(true);
  }, [data?.style]);
  const styles = [...PRESETS, ...(data?.style === 'custom' || seenCustom ? ['custom'] : [])];
  const caption = [RING.live, momentName(live.moment).toUpperCase()];
  const setStyle = async (k: string) => {
    if (!data || k === data.style) return;
    await ringApi.setStyle(k);
    await reload();
  };
  return <div className="rs-page">
    <Top back={RING.back} onBack={onBack} onClose={onClose} />
    <div className="rs-hero">
      <RingCanvas size={190} frame={live.frame} bright={pct / 100} mics label={momentName(live.moment)} />
      <span className="rs-cap"><i className="rs-live" />{caption.join(' · ')}</span>
      {data && <button className="rs-link" onClick={() => go({ page: 'guide' })}>{RING.guide_link}<ChevronRight size={14} /></button>}
    </div>
    <span className="eyebrow">{RING.ring_eyebrow}</span>
    <h2>{RING.ring_title}</h2>
    <div className="rs-sec"><span className="eyebrow">{RING.color}</span><small>{pick}</small></div>
    <div className="swatches rs-swatches">
      {RING_SWATCHES.map(s => <button key={s.label} className={'swatch' + (pick === s.label ? ' on' : '')} aria-pressed={pick === s.label} onClick={() => {
          setCustom(false);
          const [r, g, b] = hexRgb(s.hex);
          turnOn({ r, g, b });
        }}><span className="swatch-dot rs-ringdot" style={{ '--c': s.hex } as any} /><small>{s.label}</small></button>)}
      <button className={'swatch' + (pick === RING.custom ? ' on' : '')} aria-pressed={pick === RING.custom} onClick={() => setCustom(true)}><span className="swatch-dot rs-ringdot rainbow" /><small>{RING.custom}</small></button>
    </div>
    {pick === RING.custom && <LedRange label={RING.custom} text={v => `${RING.hue} · ${v}°`} value={rgbHue(ring[0], ring[1], ring[2])} max={359} tol={2} ariaLabel="LED ring hue" onCommit={h => {
        const [r, g, b] = hsvToRgb(h, 1);
        turnOn({ r, g, b });
      }} />}
    <LedRange label={RING.brightness} text={v => `${v}%`} value={pct} min={1} max={100} tol={1} className="led-bright" ariaLabel="LED ring brightness" onCommit={v => turnOn({ brightness: pctTo255(v) })} />
    {data && <>
      <div className="rs-sec"><span className="eyebrow">{RING.style}</span></div>
      <div className="rs-strip">
        {styles.map(k => <button key={k} className={'rs-tile rs-wide' + (data.style === k ? ' on' : '')} aria-pressed={data.style === k} onClick={() => setStyle(k)}>
            <MomentRing m={M_LISTEN} style={k === 'custom' || k === data.style ? data.m[M_LISTEN] : presetStyle(k, M_LISTEN)} ring={ring} size={58} />
            <strong>{styleName(k)}</strong>
            <small>{RING.styles[k][1]}</small>
          </button>)}
      </div>
      <div className="rs-rows">
        <button className="rs-row" onClick={() => go({ page: 'moments' })}>
          <MomentRing m={M_THINK} style={data.m[M_THINK]} ring={ring} size={36} />
          <span><strong>{RING.customize}</strong><small>{RING.customize_sub}</small></span>
          <ChevronRight size={18} />
        </button>
      </div>
    </>}
    <button className="primary wide" onClick={onClose}>{RING.done}</button>
  </div>;
}

/** Muted is not listed: in every style it is the red marks over what the ring shows at idle. */
const GROUPS: [string, number[]][] = [
  ['group_conv', [M_WAKE, M_LISTEN, M_THINK, M_REPLY]],
  ['group_more', [M_TIMER, M_RING, M_VOL, M_ERR]],
];
const LISTED = GROUPS.flatMap(([, ms]) => ms);

const DARK = () => null;
const PREVIEW_MS = 6000;

function Moments({ ctx, data, go, onClose, reload }: { ctx: Ctx; data: RingData; go: (v: RingView) => void; onClose: () => void; reload: () => Promise<void> }) {
  const ring = ringRgb(entity(ctx, 'ring'));
  const base = baseOf(data);
  const custom = data.style === 'custom';
  const [tour, setTour] = useState(-1);
  const [solo, setSolo] = useState(-1);
  useEffect(() => {
    if (tour < 0) return undefined;
    ringApi.preview(MOMENTS[LISTED[tour]], TOUR_MS + 200);
    const id = setTimeout(() => setTour(t => (t + 1 < LISTED.length ? t + 1 : -1)), TOUR_MS);
    return () => clearTimeout(id);
  }, [tour]);
  useEffect(() => {
    if (solo < 0) return undefined;
    ringApi.preview(MOMENTS[solo], PREVIEW_MS);
    const id = setTimeout(() => setSolo(-1), PREVIEW_MS);
    return () => clearTimeout(id);
  }, [solo]);
  useEffect(() => () => {
    ringApi.stop();
  }, []);
  const preview = (m: number) => {
    setTour(-1);
    if (solo === m) {
      setSolo(-1);
      ringApi.stop();
    } else {
      setSolo(m);
    }
  };
  const hero = tour >= 0 ? LISTED[tour] : solo;
  const row = (m: number) => {
    const s = data.m[m];
    const tag = custom ? momentTag(s, presetStyle(base, m)) : '';
    const name = RING.moments[MOMENTS[m]][0];
    return <div key={m} className="rs-row split">
      <MomentRing m={m} style={s} ring={ring} size={40} />
      <span>
        <strong>{name}{tag && <em className="rs-tag">{tag === 'own' ? RING.tag_own : RING.tag_edit}</em>}</strong>
        <small>{fxName(s.fx)} · {cmName(s.cm)} <Dots s={s} ring={ring} /></small>
      </span>
      <button className={'rs-prev' + (hero === m ? ' on' : '')} aria-pressed={solo === m} aria-label={`${RING.preview} ${name}`} onClick={() => preview(m)}>
        <Play size={12} />{solo === m ? RING.preview_stop : RING.preview}
      </button>
      <button className="rs-row-go" aria-label={name} onClick={() => go({ page: 'edit', m })}><ChevronRight size={18} /></button>
    </div>;
  };
  return <div className="rs-page">
    <Top back={RING.list_back} onBack={() => go({ page: 'ring' })} onClose={onClose} />
    <span className="eyebrow">{RING.list_eyebrow} · {styleName(data.style).toUpperCase()}</span>
    <h2>{RING.list_title}</h2>
    <p className="muted rs-lead">{fill(custom ? RING.list_lead_custom : RING.list_lead, base)}</p>
    <div className="rs-hero">
      {hero >= 0 ? <MomentRing key={hero} m={hero} style={data.m[hero]} ring={ring} size={150} mics /> : <RingCanvas size={150} frame={DARK} mics />}
      <span className="rs-cap">{hero >= 0 ? `${RING.previewing} · ${RING.moments[MOMENTS[hero]][0].toUpperCase()}` : RING.list_off}</span>
      <div className="rs-actions">
        <button className={'rs-btn' + (tour >= 0 ? ' on' : '')} onClick={() => {
            setSolo(-1);
            if (tour >= 0) {
              setTour(-1);
              ringApi.stop();
            } else {
              setTour(0);
            }
          }}>{tour >= 0 ? RING.tour_stop : RING.tour}</button>
      </div>
    </div>
    {GROUPS.map(([g, ms]) => <div key={g}>
        <span className="eyebrow rs-group">{RING[g]}</span>
        <div className="rs-rows">{ms.map(row)}</div>
      </div>)}
    <div className="rs-foot">
      {custom && <button className="rs-btn big" onClick={async () => {
          await ringApi.reset(null);
          await reload();
        }}>{fill(RING.reset_all, base)}</button>}
      <button className="primary" onClick={onClose}>{RING.done}</button>
    </div>
  </div>;
}

const GRID = [FX_SOLID, FX_BREATHE, FX_PULSE, FX_SPIN, FX_COMET, FX_ORBIT, FX_RIPPLE, FX_TWINKLE, FX_WAVE, FX_FLOW, FX_DOT, FX_OFF];
const MODES = [CM_RING, CM_BLEND, CM_RAINBOW, CM_OWN];
/** Where on the dial LED i sits, as a clock reads: LED 0 is 12 o'clock, every other one a half hour. */
const clockAt = (i: number) => (i % 2 ? RING.clock_half : RING.clock).replace('{h}', String(Math.floor(i / 2) || 12));
/** Whether two styles draw alike, as the firmware's same_style compares them: the stops count only as own colors. */
const sameStyle = (a: Style, b: Style) => a.fx === b.fx && a.cm === b.cm && a.sp === b.sp && a.br === b.br && a.fl === b.fl && a.p === b.p
  && (a.cm !== CM_OWN || a.n === b.n && a.c.slice(0, a.n).every((c, i) => rgbHex(c) === rgbHex(b.c[i])));

function Editor({ ctx, data, m, go, onClose, reload, onDirty }: { ctx: Ctx; data: RingData; m: number; go: (v: RingView) => void; onClose: () => void; reload: () => Promise<void>; onDirty: (dirty: boolean) => void }) {
  const ring = ringRgb(entity(ctx, 'ring'));
  const key = MOMENTS[m];
  const base = baseOf(data);
  const preset = presetStyle(base, m);
  const [draft, setDraft] = useState<Style>(() => clampStyle(m, data.m[m]));
  const [busy, setBusy] = useState(false);
  const latest = useRef(draft);
  latest.current = draft;
  const set = (patch: Partial<Style>) => setDraft(d => clampStyle(m, { ...d, ...patch }));
  const dirty = !sameStyle(draft, clampStyle(m, data.m[m]));
  useEffect(() => {
    onDirty(dirty);
  }, [dirty]);
  useEffect(() => () => onDirty(false), []);
  // The device plays the draft for as long as the editor is open and in view, replayed as it
  // changes and renewed before the firmware's 20 second cap ends it.
  const shown = useShown();
  useEffect(() => {
    if (!shown) return undefined;
    const id = setTimeout(() => ringApi.preview(key, 20000, latest.current), 200);
    return () => clearTimeout(id);
  }, [shown, draft, key]);
  useEffect(() => {
    if (!shown) {
      ringApi.stop();
      return undefined;
    }
    const id = setInterval(() => ringApi.preview(key, 20000, latest.current), 15000);
    return () => clearInterval(id);
  }, [shown, key]);
  useEffect(() => () => {
    ringApi.stop();
  }, []);
  const arc = m === M_TIMER || m === M_VOL;
  const own = draft.cm === CM_OWN;
  const sized = draft.fx === FX_SPIN || draft.fx === FX_COMET;
  const turns = sized || draft.fx === FX_WAVE || draft.fx === FX_FLOW || (draft.fx === FX_ORBIT && !(draft.fl & F_FIXED));
  // p means something different to each animation (heads, tail, start, position), so a new one starts from its default.
  const withFx = (fx: number) => ({ fx, p: fx === draft.fx ? draft.p : 0 });
  const pickMode = (cm: number) => {
    if (cm === CM_OWN && draft.c.slice(0, draft.n).every(c => !c[0] && !c[1] && !c[2])) set({ cm, n: 1, c: [ring.slice(), [0, 0, 0], [0, 0, 0]] });
    else set({ cm });
  };
  const setStop = (i: number, hex: string) => {
    const rgb = hexRgb(hex);
    if (!rgb) return;
    set({ c: draft.c.map((c, j) => (j === i ? rgb : c)) });
  };
  const save = async () => {
    setBusy(true);
    try {
      const r = await ringApi.setMoment(m, draft);
      if (r?.ok) {
        toast({ kind: 'ok', ttl: 2500, key: 'ring-save', title: RING.saved });
        onDirty(false);
        await reload();
        go({ page: 'moments' });
      }
    } catch {
      /* The request queue has already said so. */
    }
    setBusy(false);
  };
  const reset = async () => {
    setDraft(clampStyle(m, preset));
    if (momentTag(data.m[m], preset) || data.m[m].cm === CM_OWN) {
      await ringApi.reset(m);
      await reload();
    }
  };
  return <div className="rs-page">
    <Top back={RING.edit_back} onBack={() => go({ page: 'moments' })} onClose={onClose} />
    <span className="eyebrow">{RING.edit_eyebrow}</span>
    <h2>{RING.moments[key][0]}</h2>
    <p className="muted rs-lead">{RING.moments[key][2]}</p>
    <div className="rs-hero">
      <MomentRing m={m} style={draft} ring={ring} size={190} mics />
      <span className="rs-cap"><i className="rs-live" />{RING.edit_preview}</span>
    </div>
    <div className="rs-sec"><span className="eyebrow">{RING.animation}</span>{!arc && <small>{RING.animation_aside}</small>}</div>
    {arc ? <p className="rs-note"><Info size={14} />{RING.arc_note}</p> : <div className="rs-grid">
        {GRID.map(fx => {
          const s = clampStyle(m, { ...draft, ...withFx(fx) });
          return <button key={fx} className={'rs-tile' + (draft.fx === fx ? ' on' : '')} aria-pressed={draft.fx === fx} onClick={() => set(withFx(fx))}>
            <MomentRing m={m} style={s} ring={ring} size={50} />
            <small>{fxName(fx)}</small>
          </button>;
        })}
      </div>}
    <div className="rs-sec"><span className="eyebrow">{RING.colors}</span>{m !== M_ERR && <small>{RING.colors_aside}</small>}</div>
    {m === M_ERR ? <p className="rs-note"><Info size={14} />{RING.err_note}</p> : <>
        <div className="rs-seg" role="group" aria-label={RING.colors}>
          {MODES.map(cm => <button key={cm} className={draft.cm === cm ? 'on' : ''} aria-pressed={draft.cm === cm} onClick={() => pickMode(cm)}>{cmName(cm)}</button>)}
        </div>
        {own && <>
          <p className="rs-note"><Info size={14} />{RING.own_note}</p>
          <div className="rs-bar" style={{ background: draft.n > 1 ? `linear-gradient(90deg, ${draft.c.slice(0, draft.n).map(rgbHex).join(', ')})` : rgbHex(draft.c[0]) }} />
          <div className="rs-stops">
            {draft.c.slice(0, draft.n).map((c, i) => <span key={i} className="rs-stop">
                <label><input type="color" value={rgbHex(c)} onInput={e => setStop(i, (e.target as HTMLInputElement).value)} aria-label={`Color ${i + 1}`} /><i style={{ background: rgbHex(c) }} />{rgbHex(c).slice(1).toUpperCase()}</label>
                {draft.n > 1 && <button aria-label={`Remove color ${i + 1}`} onClick={() => set({ n: draft.n - 1, c: [...draft.c.filter((_, j) => j !== i), [0, 0, 0]] })}><X size={12} /></button>}
              </span>)}
            {draft.n < 3 && <button className="rs-add" onClick={() => set({ n: draft.n + 1, c: draft.c.map((c, j) => (j === draft.n ? draft.c[draft.n - 1].slice() : c)) })}><Plus size={14} />{RING.add_color}</button>}
          </div>
          <div className="rs-palettes">
            {Object.entries(RING.palettes as Record<string, string[]>).map(([k, hexes]) => <button key={k} onClick={() => set({ n: 3, c: hexes.map(h => hexRgb(h)) })}>
                <i style={{ background: `linear-gradient(90deg, ${hexes.map(h => '#' + h).join(', ')})` }} /><small>{k}</small>
              </button>)}
          </div>
        </>}
      </>}
    <div className="rs-sec"><span className="eyebrow">{RING.motion}</span></div>
    {turns && <div className="rs-seg two" role="group" aria-label={RING.motion}>
        <button className={draft.fl & F_REV ? '' : 'on'} aria-pressed={!(draft.fl & F_REV)} onClick={() => set({ fl: draft.fl & ~F_REV })}>{RING.cw}</button>
        <button className={draft.fl & F_REV ? 'on' : ''} aria-pressed={!!(draft.fl & F_REV)} onClick={() => set({ fl: draft.fl | F_REV })}>{RING.ccw}</button>
      </div>}
    {draft.fx !== FX_SOLID && draft.fx !== FX_OFF && <label className="hue-label"><small className="muted hue-cap">{RING.speed}</small><span>{speedWord(draft.sp)}</span>
        <input type="range" min={10} max={400} value={draft.sp} onInput={e => set({ sp: Number((e.target as HTMLInputElement).value) })} aria-label={RING.speed} /></label>}
    {sized && <label className="hue-label"><small className="muted hue-cap">{draft.fx === FX_SPIN ? RING.heads : RING.tail}</small><span>{draft.p || (draft.fx === FX_SPIN ? 2 : 10)}</span>
        <input type="range" min={1} max={draft.fx === FX_SPIN ? 4 : 20} value={draft.p || (draft.fx === FX_SPIN ? 2 : 10)} onInput={e => set({ p: Number((e.target as HTMLInputElement).value) })} aria-label={draft.fx === FX_SPIN ? RING.heads : RING.tail} /></label>}
    {draft.fx === FX_RIPPLE && <div className="hue-label"><small className="muted hue-cap">{RING.start}</small>
        <div className="rs-seg" role="group" aria-label={RING.start}>
          {(RING.sides as string[]).map((side, i) => <button key={side} className={draft.p === i ? 'on' : ''} aria-pressed={draft.p === i} onClick={() => set({ p: i })}>{side}</button>)}
        </div></div>}
    {draft.fx === FX_DOT && <label className="hue-label"><small className="muted hue-cap">{RING.position}</small><span>{clockAt(draft.p)}</span>
        <input type="range" min={0} max={N - 1} value={draft.p} onInput={e => set({ p: Number((e.target as HTMLInputElement).value) })} aria-label={RING.position} /></label>}
    <label className="hue-label"><small className="muted hue-cap">{RING.brightness}</small><span>{draft.br}%</span>
      <input type="range" min={5} max={100} value={draft.br} onInput={e => set({ br: Number((e.target as HTMLInputElement).value) })} aria-label={RING.brightness} /></label>
    <div className="rs-foot">
      <button className="rs-btn big" onClick={reset}>{fill(RING.reset, base)}</button>
      <button className="primary" disabled={busy} onClick={save}>{RING.save}</button>
    </div>
  </div>;
}

/** One card per look: signals that light the ring the same way share the first one's card. */
const GUIDE_FIXED = ['improv', 'init', 'no_ha', 'xmos', 'xmos_done', 'warning', 'login', 'action', 'jack_in', 'jack_out', 'factory'];
const FILTERS = ['all', 'conv', 'timers', 'heads'];
const PLAY_MS = 5000;

function Guide({ ctx, data, go, onClose }: { ctx: Ctx; data: RingData; go: (v: RingView) => void; onClose: () => void }) {
  const ring = ringRgb(entity(ctx, 'ring'));
  const [filter, setFilter] = useState('all');
  const [playing, setPlaying] = useState<{ k: string; n: number } | null>(null);
  // A signal that plays once is shown on the device as it really runs, twice with the card's
  // pause between (once when a run is long), in step with the card, which restarts with it.
  useEffect(() => {
    if (!playing) return undefined;
    const { k } = playing;
    const run = fixedRun(k);
    const timers: number[] = [];
    let ends = PLAY_MS;
    if (run) {
      const runs = run.once > PLAY_MS ? 1 : 2;
      for (let i = 0; i < runs; i++) timers.push(window.setTimeout(() => ringApi.preview(k, run.once), i * run.every));
      ends = (runs - 1) * run.every + run.once;
    } else {
      ringApi.preview(k, PLAY_MS);
    }
    timers.push(window.setTimeout(() => setPlaying(null), ends));
    return () => timers.forEach(clearTimeout);
  }, [playing]);
  useEffect(() => () => {
    ringApi.stop();
  }, []);
  const play = (k: string) => setPlaying(p => ({ k, n: (p?.n ?? 0) + 1 }));
  const on = (k: string) => playing?.k === k;
  const card = (m: number) => {
    const k = MOMENTS[m];
    const yours = !!momentTag(data.m[m], STYLES.classic[m]);
    return <button key={k} className={'rs-card' + (on(k) ? ' on' : '')} onClick={() => play(k)}>
      <span className="rs-card-top"><MomentRing m={m} style={data.m[m]} ring={ring} size={48} />{yours && <em className="rs-tag">{RING.tag_style}</em>}</span>
      <strong>{RING.moments[k][0]}</strong>
      <small>{RING.moments[k][1]}</small>
    </button>;
  };
  const show = (f: string) => filter === 'all' || filter === f;
  return <div className="rs-page">
    <Top back={RING.list_back} onBack={() => go({ page: 'ring' })} onClose={onClose} />
    <span className="eyebrow">{RING.guide_eyebrow}</span>
    <h2>{RING.guide_title}</h2>
    <p className="muted rs-lead">{RING.guide_lead}</p>
    <div className="rs-filters" role="group">
      {FILTERS.map(f => <button key={f} className={filter === f ? 'on' : ''} aria-pressed={filter === f} onClick={() => setFilter(f)}>{RING['filter_' + f]}</button>)}
    </div>
    {show('conv') && <>
      <span className="eyebrow rs-group">{RING.group_conv}</span>
      <div className="rs-cards">{[M_WAKE, M_LISTEN, M_THINK, M_REPLY].map(card)}</div>
    </>}
    {(show('timers') || show('heads')) && <>
      <span className="eyebrow rs-group">{filter === 'timers' ? RING.group_timers : filter === 'heads' ? RING.group_status : RING.group_more}</span>
      <div className="rs-cards">
        {show('timers') && [M_TIMER, M_RING, M_VOL].map(card)}
        {show('heads') && [M_MUTE, M_ERR].map(card)}
      </div>
    </>}
    {show('heads') && <>
      <div className="rs-sec rs-group"><span className="eyebrow">{RING.guide_fixed}</span><small>{RING.guide_fixed_aside}</small></div>
      <div className="rs-cards">
        {GUIDE_FIXED.map(k => <button key={k} className={'rs-card' + (on(k) ? ' on' : '')} onClick={() => play(k)}>
            <span className="rs-card-top"><FixedRing key={on(k) ? playing?.n : 0} k={k} ring={ring} size={48} /><em className="rs-tag fixed"><Lock size={10} />{RING.tag_fixed}</em></span>
            <strong>{RING.fixed[k][0]}</strong>
            <small>{RING.fixed[k][1]}</small>
          </button>)}
      </div>
    </>}
    <button className="primary wide" onClick={onClose}>{RING.done}</button>
  </div>;
}
