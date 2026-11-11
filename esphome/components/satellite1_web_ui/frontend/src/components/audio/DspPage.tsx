import React, { useCallback, useEffect, useId, useMemo, useRef, useState } from 'react';
import { DSP, HINTS } from '../../copy.js';
import { apiUrl, entity, pathFor, post, request, requestJson } from '../../lib/device.js';
import { eqView, setEqView } from '../../lib/dsp.js';
import { DEFAULT_Q, KINDS, MAX_FILTERS, MAX_SECTIONS, RANGE, SLOPES, T, TONE, analyse, autoPreamp, clamp, exportText, fmtDb, fmtHz, fromWire, importText, kindOf, logTrackValue, profileBody, profileSections, sameSound, slopeOf, typeOf, usesGain, usesQ } from '../../lib/eq.js';
import { toast } from '../../lib/toast.js';
import { HintBtn } from '../bits';
import { Switch } from '../controls';
import { Drawer, Presence } from '../Drawer';
import { MSlider } from '../MSlider';
import { DxCard, DxConfirmDialog, DxSelect, saveBlob, useCopied } from '../settings/dx';
import { EqGraph, ON_INK, fill, filterColor, roundHz, sayFilter, techFilter } from './EqGraph';
import type { Filter, Profile } from './EqGraph';
import type { Ctx } from '../../ctx';

type Status = {
  online: boolean;
  en: boolean;
  out: number;
  sy: number[];
  byp: number | null;
  limPct: number;
};
type Preset = {
  name: string;
  builtin: boolean;
  p: Profile;
};
type Confirm = {
  t: string;
  b: string;
  go: string;
  danger?: boolean;
  back?: React.RefObject<HTMLElement>;
  run: () => void;
} | null;

const LIVE_MS = 1000;
const COMPARE_RENEW_MS = 5000;
const NAME_BYTES = 31;
const RETRY_MIN_MS = 1000;
const RETRY_MAX_MS = 5000;
/** How long a preset action waits for edits still on their way before going ahead anyway. */
const FLUSH_MAX_MS = 10000;
/** A click this soon after Space or Enter on the compare button is the keyboard's own. */
const KEY_CLICK_MS = 1000;

const parseStatus = (d: any): Status => ({
  online: !!d.online,
  en: !!d.en,
  out: d.out === 1 ? 1 : 0,
  sy: Array.isArray(d.sy) ? d.sy : [0, 0],
  byp: d.info ? d.info.byp : null,
  limPct: d.stats ? Math.max(...(d.stats.lim_pct || [0])) : 0
});
const sameStatus = (a: Status, b: Status) => JSON.stringify(a) === JSON.stringify(b);

/**
 * The switch's own value wins over a read that set out before the switch's write had landed: that
 * read still has the old value, and would flick the switch back for a second.
 */
type LocalEn = { v: boolean; at: number } | null;
const keepEn = (st: Status, local: LocalEn, sentAt: number): Status => local && sentAt <= local.at ? {
  ...st,
  en: local.v
} : st;

/** `quiet` keeps request()'s own failure toast out of it, for a caller that reports its own. */
const form = (fields: Record<string, string>, quiet = false) => ({
  method: 'POST',
  body: new URLSearchParams(fields),
  quiet
});

/**
 * The page's own width at which it splits into two columns (styles/dsp.css, `.dsp-grid.wide`). The
 * page's, not the window's: from 1024 px the sidebar takes a share, so a 1024 px window leaves the
 * page narrower than a 1000 px one does.
 */
const WIDE_PX = 820;
function useWide() {
  const [wide, setWide] = useState(false);
  const ro = useRef<ResizeObserver | null>(null);
  const ref = useCallback((el: HTMLDivElement | null) => {
    ro.current?.disconnect();
    ro.current = null;
    if (!el) return;
    const measure = () => setWide(el.clientWidth >= WIDE_PX);
    measure();
    ro.current = new ResizeObserver(measure);
    ro.current.observe(el);
  }, []);
  return [ref, wide] as const;
}

/**
 * Edits go out latest-wins, one request at a time: a drag produces an edit per pointer move, and
 * only the newest one per output is worth sending. The device's request queue sees one write in
 * flight, never forty. The two outputs take turns, so a drag on one can't hold back the other's
 * last edit. An edit that doesn't get through is tried again, backing off, until a newer one
 * replaces it. `flush` resolves once everything edited so far has reached the device.
 */
function useWriter(onRefused: () => void) {
  const pending = useRef<(Record<string, string> | null)[]>([null, null]);
  // The last body queued per output: a slider's commit repeats its final preview, and the device
  // need only hear it once.
  const last = useRef<(string | null)[]>([null, null]);
  const busy = useRef(false);
  const turn = useRef(0);
  const refused = useRef(false);
  const retry = useRef<{ ms: number; timer?: ReturnType<typeof setTimeout> }>({ ms: 0 });
  const waiters = useRef<(() => void)[]>([]);
  const [sending, setSending] = useState(-1);
  const idle = () => !busy.current && pending.current.every(b => b == null);
  const pump = useCallback(async () => {
    if (busy.current || retry.current.timer) return;
    const o = [turn.current, 1 - turn.current].find(i => pending.current[i] != null) ?? -1;
    if (o < 0) {
      setSending(-1);
      // Reloaded once nothing else is on its way, so the device's copy lands on top.
      if (refused.current) {
        refused.current = false;
        onRefused();
      }
      waiters.current.splice(0).forEach(w => w());
      return;
    }
    turn.current = 1 - o;
    const body = pending.current[o]!;
    pending.current[o] = null;
    busy.current = true;
    setSending(o);
    // The first failure in a row is reported; its retries aren't.
    const r = await request('/api/sat1/dsp', form({
      out: String(o),
      ...body
    }, retry.current.ms > 0)).catch(() => null);
    busy.current = false;
    if (!r) {
      if (pending.current[o] == null) pending.current[o] = body;
      const ms = retry.current.ms = Math.min(retry.current.ms ? retry.current.ms * 2 : RETRY_MIN_MS, RETRY_MAX_MS);
      retry.current.timer = setTimeout(() => {
        retry.current.timer = undefined;
        pump();
      }, ms);
      return;
    }
    retry.current.ms = 0;
    if (!r.ok) {
      refused.current = true;
      last.current[o] = null;
    }
    pump();
  }, [onRefused]);
  const write = useCallback((o: number, body: Record<string, string>) => {
    const key = JSON.stringify(body);
    if (key === last.current[o]) return;
    last.current[o] = key;
    pending.current[o] = body;
    pump();
  }, [pump]);
  /** After the device's profile changes under the page: the next edit goes out even if it repeats. */
  const reset = useCallback(() => {
    last.current = [null, null];
  }, []);
  const flush = useCallback(() => new Promise<void>(res => {
    if (idle()) return res();
    const t = setTimeout(res, FLUSH_MAX_MS);
    waiters.current.push(() => {
      clearTimeout(t);
      res();
    });
  }), []);
  useEffect(() => () => clearTimeout(retry.current.timer), []);
  return {
    write,
    flush,
    reset,
    sending
  };
}

/**
 * A slider over a log scale (frequency, Q, times), on MSlider's linear track of 1000 steps. A
 * typed value sits at its nearest step but keeps its own value there, so 5700 Hz reads 5700, not
 * the step's 5690. A value reached by moving the slider keeps the step it came from, and each step
 * away lands on the next value that rounds differently, so an arrow key never sticks.
 */
function LogSlider({
  value,
  min,
  max,
  format,
  round,
  onChange,
  ariaLabel
}: {
  value: number;
  min: number;
  max: number;
  format: (v: number) => string;
  round: (v: number) => number;
  onChange: (v: number) => void;
  ariaLabel: string;
}) {
  const from = useRef<{ pos: number; v: number } | null>(null);
  const toPos = (v: number) => Math.round(1000 * Math.log(clamp(v, min, max) / min) / Math.log(max / min));
  const pos = from.current?.v === value ? from.current.pos : toPos(value);
  const valAt = (p: number) => p === pos ? value : logTrackValue(p, value, pos, min, max, round);
  const go = (p: number) => {
    if (p === pos) return;
    const v = valAt(p);
    from.current = {
      pos: p,
      v
    };
    onChange(v);
  };
  return <MSlider value={pos} min={0} max={1000} step={1} format={p => format(valAt(p))} ariaLabel={ariaLabel} onPreview={go} onCommit={go} />;
}

/** "3.5", "-3,5" and "+3" alike; NaN for anything else, which puts the old value back. */
const parseNum = (s: string) => /^[-+]?(\d+([.,]\d*)?|[.,]\d+)$/.test(s) ? Number(s.replace(',', '.')) : NaN;
/** "1.6k", "1600", "1600 Hz" and "1,6 kHz" alike. */
const parseHz = (s: string) => {
  const m = /^(\d+(?:[.,]\d*)?|[.,]\d+)\s*(k?)\s*(?:hz)?$/i.exec(s);
  return m ? Number(m[1].replace(',', '.')) * (m[2] ? 1000 : 1) : NaN;
};

/**
 * A typed value, committed on Enter or leaving the field; Escape puts the old one back. `signed`
 * fields get the full keyboard on a phone: the decimal pad has no minus key.
 */
function NumField({
  value,
  digits,
  unit,
  label,
  signed = false,
  parse = parseNum,
  onCommit
}: {
  value: number;
  digits: number;
  unit: string;
  label: string;
  signed?: boolean;
  parse?: (s: string) => number;
  onCommit: (v: number) => void;
}) {
  const [text, setText] = useState<string | null>(null);
  const commit = () => {
    if (text == null) return;
    const v = parse(text.replace(/\u2212/g, '-').trim());
    setText(null);
    if (Number.isFinite(v)) onCommit(v);
  };
  return <span className="dsp-num">
      <input type="text" inputMode={signed ? 'text' : 'decimal'} autoComplete="off" spellcheck={false} aria-label={label} value={text ?? String(+value.toFixed(digits))} onInput={e => setText((e.target as HTMLInputElement).value)} onBlur={commit} onKeyDown={e => {
      if (e.key === 'Enter') (e.target as HTMLInputElement).blur();
      if (e.key === 'Escape') setText(null);
    }} />
      <span className="dsp-num-unit" aria-hidden="true">{unit}</span>
    </span>;
}

const fmtMs = (ms: number) => (ms >= 1000 ? `${+(ms / 1000).toFixed(2)} s` : `${+ms.toFixed(ms < 10 ? 1 : 0)} ms`);
const round2 = (v: number) => Math.round(v * 100) / 100;
const utf8Len = (s: string) => new TextEncoder().encode(s).length;

/**
 * A list of actions under a button, drawn like DxSelect's: a disclosure, Tab through its buttons.
 * Picking one puts focus back on the button first, so a dialog it opens returns focus there.
 */
function Menu({
  label,
  items,
  buttonRef
}: {
  label: string;
  items: ([string, () => void] | null)[];
  buttonRef: React.RefObject<HTMLButtonElement>;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return undefined;
    const close = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    const esc = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return;
      if (ref.current?.contains(document.activeElement)) buttonRef.current?.focus();
      setOpen(false);
    };
    document.addEventListener('mousedown', close);
    document.addEventListener('keydown', esc);
    return () => {
      document.removeEventListener('mousedown', close);
      document.removeEventListener('keydown', esc);
    };
  }, [open]);
  return <div ref={ref} className="dx-sel dsp-menu">
      <button ref={buttonRef} className="dx-btn sm" aria-expanded={open} onClick={() => setOpen(v => !v)}>
        {label}
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true"><path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>
      </button>
      {open && <div className="dx-sel-pop">
          {items.filter(Boolean).map(it => {
        const [l, run] = it!;
        return <button key={l} className="dx-sel-opt" onClick={() => {
          setOpen(false);
          buttonRef.current?.focus();
          run();
        }}><span>{l}</span></button>;
      })}
        </div>}
    </div>;
}

function ImportDialog({
  open,
  onClose,
  onImport
}: {
  open: boolean;
  onClose: () => void;
  onImport: (r: NonNullable<ReturnType<typeof importText>>) => void;
}) {
  const [text, setText] = useState('');
  const [bad, setBad] = useState(false);
  const file = useRef<HTMLInputElement>(null);
  const go = () => {
    const r = importText(text);
    if (!r) {
      setBad(true);
      return;
    }
    setText('');
    setBad(false);
    onImport(r);
  };
  return <Presence>{open && <Drawer label={DSP.import_title} onClose={onClose} className="dx-modal dsp-modal">
        <p className="dx-modal-title">{DSP.import_title}</p>
        <p className="dx-modal-body">{DSP.import_body}</p>
        <textarea className="dsp-text" rows={8} spellcheck={false} aria-label={DSP.import_text} aria-invalid={bad} value={text} placeholder={'Preamp: 10.00 dB\nFilter 1: ON PK Fc 138.0 Hz Gain -3.00 dB Q 1.930'} onInput={e => {
        setText((e.target as HTMLTextAreaElement).value);
        setBad(false);
      }} />
        {bad && <p className="dsp-note t-err" role="alert">{DSP.import_none}</p>}
        <input ref={file} type="file" accept=".txt,text/plain" hidden onChange={async e => {
        const f = (e.target as HTMLInputElement).files?.[0];
        if (f) setText(await f.text());
        (e.target as HTMLInputElement).value = '';
      }} />
        <div className="dx-modal-actions dsp-modal-actions">
          <button className="dx-btn" onClick={() => file.current?.click()}>{DSP.import_choose}</button>
          <span className="dsp-grow" />
          <button className="dx-btn" onClick={onClose}>{DSP.name_cancel}</button>
          <button className="dx-btn solid" disabled={!text.trim()} onClick={go}>{DSP.import_go}</button>
        </div>
      </Drawer>}</Presence>;
}

function ExportDialog({
  open,
  text,
  name,
  onClose
}: {
  open: boolean;
  text: string;
  name: string;
  onClose: () => void;
}) {
  const [copied, copy] = useCopied();
  return <Presence>{open && <Drawer label={DSP.export_title} onClose={onClose} className="dx-modal dsp-modal">
        <p className="dx-modal-title">{DSP.export_title}</p>
        <p className="dx-modal-body">{DSP.export_body}</p>
        <textarea className="dsp-text" rows={8} readOnly spellcheck={false} aria-label={DSP.export_title} value={text} onFocus={e => (e.target as HTMLTextAreaElement).select()} />
        <div className="dx-modal-actions dsp-modal-actions">
          <button className="dx-btn" onClick={() => copy(text)}>{copied ? DSP.copied : DSP.copy}</button>
          <button className="dx-btn" onClick={() => saveBlob(new Blob([text], {
          type: 'text/plain'
        }), name)}>{DSP.download}</button>
          <span className="dsp-grow" />
          <button className="dx-btn solid" onClick={onClose}>{DSP.close}</button>
        </div>
      </Drawer>}</Presence>;
}

function FilterEditor({
  f,
  onPatch,
  onRemove
}: {
  f: Filter;
  onPatch: (patch: Partial<Filter>) => void;
  onRemove: () => void;
}) {
  const kind = kindOf(f.t);
  const slope = slopeOf(f.t);
  const setKind = (k: string) => {
    const t = typeOf(k, slope ?? 'bw12');
    onPatch(usesQ(t) && !usesQ(f.t) ? {
      t,
      q: DEFAULT_Q
    } : {
      t
    });
  };
  return <div className="dsp-edit">
      <div className="dsp-edit-row">
        <span className="dsp-edit-label">{DSP.type}</span>
        <DxSelect value={kind} label={DSP.type} options={KINDS.map(k => [k, DSP.kinds[k as keyof typeof DSP.kinds]] as [string, string])} onChange={setKind} />
      </div>
      {slope && <div className="dsp-edit-row">
          <span className="dsp-edit-label">{DSP.slope}</span>
          <DxSelect value={slope} label={DSP.slope} options={SLOPES.map(s => [s, DSP.slopes[s as keyof typeof DSP.slopes]] as [string, string])} onChange={s => onPatch({
          t: typeOf(kind, s)
        })} />
        </div>}
      <div className="dsp-edit-row">
        <span className="dsp-edit-label">{DSP.freq}</span>
        <LogSlider value={f.hz} min={RANGE.hz[0]} max={RANGE.hz[1]} format={fmtHz} round={roundHz} ariaLabel={DSP.freq} onChange={hz => onPatch({
        hz
      })} />
        <NumField value={f.hz} digits={1} unit="Hz" label={DSP.freq} parse={parseHz} onCommit={hz => onPatch({
        hz: clamp(hz, RANGE.hz[0], RANGE.hz[1])
      })} />
      </div>
      {usesGain(f.t) && <div className="dsp-edit-row">
          <span className="dsp-edit-label">{DSP.gain}</span>
          <MSlider value={f.db} min={RANGE.db[0]} max={RANGE.db[1]} step={0.1} snap={0} format={v => fmtDb(v)} ariaLabel={DSP.gain} onPreview={db => onPatch({
        db
      })} onCommit={db => onPatch({
        db
      })} />
          <NumField value={f.db} digits={2} unit="dB" label={DSP.gain} signed onCommit={db => onPatch({
        db: clamp(db, RANGE.db[0], RANGE.db[1])
      })} />
        </div>}
      {usesQ(f.t) && <div className="dsp-edit-row">
          <span className="dsp-edit-label">{DSP.q}<HintBtn text={DSP.q_hint} /></span>
          <LogSlider value={f.q} min={RANGE.q[0]} max={RANGE.q[1]} format={q => `Q ${q.toFixed(2)}`} round={round2} ariaLabel={DSP.q} onChange={q => onPatch({
        q
      })} />
          <NumField value={f.q} digits={3} unit="Q" label={DSP.q} onCommit={q => onPatch({
        q: clamp(q, RANGE.q[0], RANGE.q[1])
      })} />
        </div>}
      <div className="dsp-edit-foot">
        <button className="dx-btn sm danger" onClick={onRemove}>{DSP.remove}</button>
      </div>
    </div>;
}

/**
 * The speaker EQ. Shown only while the running XMOS has it (src/lib/dsp.js). Basic, the default, is
 * the outputs, presets and tone controls; Advanced adds the filters, preamp and limiter. Both are
 * views of the same state, and the choice is this browser's, never part of the URL.
 */
export function DspPage({
  ctx
}: {
  ctx: Ctx;
}) {
  const [view, setView] = useState(() => eqView());
  const pick = (v: string) => {
    setView(v);
    setEqView(v);
  };
  const ready = ctx.dsp === 1;
  return <section className="control au-tab dsp-tab">
      <span className="eyebrow">{DSP.eyebrow}</span>
      <div className="dsp-head">
        <h1><span>{DSP.title_a}</span><em>{DSP.title_b}</em></h1>
        {ready && <div className="dsp-view" role="group" aria-label={DSP.view}>
            {(['basic', 'advanced'] as const).map(v => <button key={v} aria-pressed={view === v} className={`dsp-view-btn${view === v ? ' on' : ''}`} onClick={() => pick(v)}>{DSP.views[v]}</button>)}
          </div>}
      </div>
      {ready ? <DspEditor ctx={ctx} advanced={view === 'advanced'} /> : <p className="au-missing">{DSP.waiting}</p>}
    </section>;
}

function DspEditor({
  ctx,
  advanced
}: {
  ctx: Ctx;
  advanced: boolean;
}) {
  const [drafts, setDrafts] = useState<Profile[] | null>(null);
  const [presets, setPresets] = useState<Preset[]>([]);
  const [status, setStatus] = useState<Status | null>(null);
  const [failed, setFailed] = useState(false);
  const [gridRef, wide] = useWide();
  const [out, setOut] = useState<number | null>(null);
  const [sel, setSel] = useState<number | null>(null);
  const [noStages, setNoStages] = useState(false);
  const [notes, setNotes] = useState<string[]>([]);
  const [naming, setNaming] = useState<{
    mode: 'save' | 'rename';
    text: string;
  } | null>(null);
  const [confirm, setConfirm] = useState<Confirm>(null);
  const [importing, setImporting] = useState(false);
  const [exporting, setExporting] = useState(false);
  // Kept here because the two layouts are different trees: crossing 820 px remounts the card.
  const [limOpen, setLimOpen] = useState(false);
  const enLocal = useRef<LocalEn>(null);
  // Counts edits, so a read that set out before one doesn't put the older profile back.
  const edits = useRef(0);
  const moreRef = useRef<HTMLButtonElement>(null);
  const presetRef = useRef<HTMLButtonElement>(null);
  const nameRef = useRef<HTMLInputElement>(null);
  const nameNoteId = useId();

  const load = useCallback(async () => {
    const gen = edits.current;
    const sentAt = Date.now();
    const d = await requestJson('/api/sat1/dsp').catch(() => null);
    if (!d || !Array.isArray(d.p) || d.p.length < 2) {
      setFailed(true);
      return;
    }
    setFailed(false);
    const st = keepEn(parseStatus(d), enLocal.current, sentAt);
    setStatus(st);
    if (gen === edits.current) setDrafts(d.p.map(fromWire));
    setPresets((Array.isArray(d.presets) ? d.presets : []).map((x: any) => ({
      name: String(x.name),
      builtin: !!x.builtin,
      p: fromWire(x.p)
    })));
    setOut(o => o ?? st.out);
  }, []);
  useEffect(() => {
    load();
  }, [load]);

  // The live read keeps the device's own INFO and stats reads going (satellite1_dsp's live window)
  // and drives the sync, playing and limiter indicators. Only while the page is in front.
  useEffect(() => {
    let live = true;
    let timer: ReturnType<typeof setTimeout>;
    const tick = async () => {
      if (document.visibilityState === 'visible') {
        const sentAt = Date.now();
        const d = await requestJson('/api/sat1/dsp/live').catch(() => null);
        if (!live) return;
        if (d) {
          const next = keepEn(parseStatus(d), enLocal.current, sentAt);
          setStatus(s => s && sameStatus(s, next) ? s : next);
        }
      }
      if (live) timer = setTimeout(tick, LIVE_MS);
    };
    timer = setTimeout(tick, LIVE_MS);
    return () => {
      live = false;
      clearTimeout(timer);
    };
  }, []);

  const writer = useWriter(load);

  // Hold-to-compare: the device lets go by itself 15 s after the last renewal, so a tab closed
  // mid-hold can't leave the EQ off.
  const holding = useRef(false);
  const keyAt = useRef(0);
  const renew = useRef<ReturnType<typeof setInterval>>();
  const [held, setHeld] = useState(false);
  const compare = (on: boolean) => request('/api/sat1/dsp', form({
    cmp: on ? '1' : '0'
  })).catch(() => null);
  const startHold = () => {
    if (holding.current) return;
    holding.current = true;
    setHeld(true);
    compare(true);
    renew.current = setInterval(() => compare(true), COMPARE_RENEW_MS);
  };
  const endHold = () => {
    if (!holding.current) return;
    holding.current = false;
    setHeld(false);
    clearInterval(renew.current);
    compare(false);
  };
  useEffect(() => () => {
    if (!holding.current) return;
    clearInterval(renew.current);
    fetch(apiUrl('/api/sat1/dsp'), {
      ...form({
        cmp: '0'
      }),
      keepalive: true
    }).catch(() => {});
  }, []);
  // The button is disabled with the EQ off, and a disabled button hears no pointer-up.
  useEffect(() => {
    if (status && !status.en) endHold();
  }, [status?.en]);
  useEffect(() => {
    if (naming) nameRef.current?.focus();
  }, [!!naming]);
  // Basic has neither the name field nor the filter list.
  useEffect(() => {
    if (advanced) return;
    setNaming(null);
    setSel(null);
  }, [advanced]);

  const o = out ?? 0;
  const draft = drafts?.[o] ?? null;
  const a = useMemo(() => draft ? analyse(draft) : null, [draft]);
  if (failed && !drafts) {
    return <div className="dsp-fail">
        <p className="au-missing">{DSP.load_failed}</p>
        <button className="dx-btn sm" onClick={load}>{DSP.retry}</button>
      </div>;
  }
  if (!drafts || !draft || !a || !status) return <p className="au-missing">{DSP.waiting}</p>;

  /**
   * Applies an edit to the shown output and sends it. False, and nothing changes, when it needs
   * more stages than exist.
   */
  const edit = (next: Profile, extra: Record<string, string> = {}) => {
    if (profileSections(next) > MAX_SECTIONS) {
      setNoStages(true);
      return false;
    }
    setNoStages(false);
    edits.current++;
    setDrafts(ds => ds && ds.map((d, i) => i === o ? next : d));
    writer.write(o, {
      ...profileBody(next),
      ...extra
    });
    return true;
  };
  const patch = (i: number, p: Partial<Filter>) => edit({
    ...draft,
    f: draft.f.map((f, k) => k === i ? {
      ...f,
      ...p
    } : f)
  });
  const add = (hz = 1000, db = 0) => {
    if (draft.f.length >= MAX_FILTERS) return;
    const before = draft.f.length;
    if (edit({
      ...draft,
      f: [...draft.f, {
        t: T.PEAK,
        on: true,
        hz,
        db,
        q: 1
      }]
    })) setSel(before);
  };
  const remove = (i: number) => {
    edit({
      ...draft,
      f: draft.f.filter((_, k) => k !== i)
    });
    setSel(null);
  };
  const clearNotes = () => {
    setNotes([]);
    setNoStages(false);
  };

  /** A preset action on the shown output, once every edit before it has landed. */
  const presetOp = async (op: string, fields: Record<string, string> = {}) => {
    await writer.flush();
    writer.reset();
    clearNotes();
    const r = await request('/api/sat1/dsp/preset', form({
      op,
      out: String(o),
      ...fields
    }, true)).catch(() => null);
    await load();
    if (!r?.ok) toast({
      kind: 'err',
      title: DSP.op_failed,
      ttl: 4000
    });
    return !!r?.ok;
  };
  const current = presets.find(x => x.name === draft.preset) ?? null;
  const modified = !!current && !sameSound(draft, current.p);
  const userNames = new Set(presets.filter(x => !x.builtin).map(x => x.name));
  const builtinNames = new Set(presets.filter(x => x.builtin).map(x => x.name));
  const load_ = (name: string) => {
    if (name === draft.preset && !modified) return;
    const run = () => presetOp('load', {
      name
    });
    if (modified || !current) {
      setConfirm({
        t: DSP.confirm_load.t,
        b: fill(DSP.confirm_load.b, {
          name
        }),
        go: DSP.confirm_load_go,
        back: presetRef,
        run
      });
    } else run();
  };

  const okToast = (title: string) => toast({
    kind: 'ok',
    title,
    ttl: 2500
  });
  const renaming = naming?.mode === 'rename' && !!draft.preset;
  const nameText = naming?.text.trim() ?? '';
  const nameIssue = !naming ? null : !nameText ? DSP.name_short : utf8Len(nameText) > NAME_BYTES ? DSP.name_long : builtinNames.has(nameText) ? DSP.name_builtin : !renaming && !userNames.has(nameText) && userNames.size >= 8 ? DSP.name_full : null;
  const nameNote = naming && !nameIssue && userNames.has(nameText) && !(renaming && nameText === draft.preset) ? fill(DSP.name_replace, {
    name: nameText
  }) : null;
  const nameMsg = nameIssue && nameText ? nameIssue : nameNote;
  const submitName = async () => {
    if (!naming || nameIssue) return;
    if (renaming && nameText === draft.preset) {
      setNaming(null);
      return;
    }
    const ok = renaming ? await presetOp('rename', {
      name: draft.preset,
      to: nameText
    }) : await presetOp('save', {
      name: nameText
    });
    if (!ok) return;
    setNaming(null);
    okToast(fill(renaming ? DSP.renamed : DSP.saved, {
      name: nameText
    }));
  };
  const setEn = (v: boolean) => {
    enLocal.current = {
      v,
      at: Infinity
    };
    setStatus(s => s && {
      ...s,
      en: v
    });
    request('/api/sat1/dsp', form({
      en: v ? '1' : '0'
    })).catch(() => null).then(r => {
      if (r?.ok) {
        enLocal.current = {
          v,
          at: Date.now()
        };
        return;
      }
      enLocal.current = null;
      setStatus(s => s && {
        ...s,
        en: !v
      });
    });
  };

  const xmos = entity(ctx, 'xmos_firmware')?.value;
  const flashing = typeof xmos === 'string' && /^Flashing/i.test(xmos);
  // The amplifier's own setting rather than the EQ's, so it applies with the EQ off too.
  const chan = entity(ctx, 'speaker_channel');
  const setChannel = (option: string) => {
    const p = pathFor(ctx, 'speaker_channel', 'set', {
      option
    });
    if (p) post(p).catch(() => {});
  };
  const sync = writer.sending === o ? 1 : status.sy[o] ?? 0;
  const playingHere = status.en && status.out === o;
  const syncText = sync === 0 && !playingHere ? DSP.ready : DSP.sync[sync] ?? '';
  const limiting = playingHere && status.limPct > 0;
  const outName = DSP.outputs[o];
  const fileName = `satellite1-eq-${(draft.preset || outName).toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '')}.txt`;

  const bannerText = flashing ? DSP.updating : !status.online ? DSP.offline : status.byp === 4 ? DSP.overload : null;
  const banner = bannerText && <div className="dsp-banner" role="status">{bannerText}</div>;

  const bar = <div className="dx-card dsp-bar">
        <div className="dsp-seg-row">
          <div className="dsp-seg" role="group" aria-label={DSP.output}>
            {DSP.outputs.map((label, i) => <button key={label} aria-pressed={o === i} className={`dsp-seg-btn${o === i ? ' on' : ''}`} onClick={() => {
            setOut(i);
            setSel(null);
            setNaming(null);
            clearNotes();
          }}>
                <span>{label}</span>
                {status.out === i && <span className="dsp-playing">{DSP.playing}</span>}
              </button>)}
          </div>
          <HintBtn text={DSP.output_hint} />
        </div>
        {o === 0 && chan && <div className="dsp-bar-row dsp-channel">
            <span className="dsp-bar-label">{DSP.channel}<HintBtn text={HINTS.speaker_channel} /></span>
            <span className="dsp-grow" />
            <DxSelect value={chan.value ?? ''} label={DSP.channel_label} options={chan.option || []} onChange={setChannel} />
          </div>}
        <div className="dsp-bar-row">
          <span className="dsp-bar-label">{DSP.eq_on}<HintBtn text={DSP.eq_hint} /></span>
          <Switch on={status.en} label={DSP.eq_on} onChange={setEn} />
          <span className="dsp-grow" />
          <button className={`dx-btn sm dsp-hold${held ? ' active' : ''}`} disabled={!status.en} aria-pressed={held} onPointerDown={e => {
          (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
          startHold();
        }} onPointerUp={endHold} onPointerCancel={endHold} onLostPointerCapture={endHold} onKeyDown={e => {
          if (e.key === ' ' || e.key === 'Enter') {
            e.preventDefault();
            keyAt.current = Date.now();
            if (!e.repeat) startHold();
          }
        }} onKeyUp={e => {
          if (e.key === ' ' || e.key === 'Enter') {
            keyAt.current = Date.now();
            endHold();
          }
        }} onClick={e => {
          // A screen reader's activation is a bare click, with no press before it and nothing to
          // hold: there the button latches, and the next activation lets go.
          if (e.detail !== 0 || Date.now() - keyAt.current < KEY_CLICK_MS) return;
          if (holding.current) endHold();else startHold();
        }} onBlur={endHold} onContextMenu={e => e.preventDefault()}>{held ? DSP.comparing : DSP.compare}</button>
        </div>
        <div className="dsp-preset">
          <span className="eyebrow">{DSP.preset}</span>
          <div className="dsp-preset-row">
            <DxSelect value={draft.preset} label={DSP.preset} buttonRef={presetRef} options={[...(current ? [] : [['', DSP.custom] as [string, string]]), ...presets.map(x => x.name)]} onChange={v => v && load_(v)} />
            {modified && <span className="dsp-tag">{DSP.modified}</span>}
          </div>
          {(advanced || modified) && <div className="dsp-preset-actions">
            {advanced && current && !current.builtin && modified && <button className="dx-btn sm solid" onClick={async () => {
            if (await presetOp('save', {
              name: draft.preset
            })) okToast(fill(DSP.saved, {
              name: draft.preset
            }));
          }}>{DSP.save}</button>}
            {advanced && <button className="dx-btn sm" onClick={() => setNaming({
            mode: 'save',
            text: ''
          })}>{DSP.save_as}</button>}
            {modified && <button className="dx-btn sm" onClick={() => presetOp('load', {
            name: draft.preset
          })}>{DSP.revert}</button>}
            {advanced && <Menu label={DSP.more} buttonRef={moreRef} items={[[DSP.import, () => setImporting(true)], [DSP.export, () => setExporting(true)], current && !current.builtin && !modified ? [DSP.rename, () => setNaming({
            mode: 'rename',
            text: draft.preset
          })] : null, current && !current.builtin ? [DSP.delete, () => setConfirm({
            t: fill(DSP.confirm_delete.t, {
              name: draft.preset
            }),
            b: DSP.confirm_delete.b,
            go: DSP.confirm_delete_go,
            danger: true,
            back: moreRef,
            run: () => presetOp('delete', {
              name: draft.preset
            })
          })] : null, [DSP.reset, () => setConfirm({
            t: DSP.confirm_reset.t,
            b: fill(DSP.confirm_reset.b, {
              output: outName,
              // satellite1_dsp lists its built-ins first, each output's default at its own index.
              name: presets.filter(x => x.builtin)[o]?.name ?? ''
            }),
            go: DSP.confirm_reset_go,
            danger: true,
            back: moreRef,
            run: () => presetOp('reset')
          })]]} />}
          </div>}
          {advanced && naming && <form className="dsp-name" onSubmit={e => {
          e.preventDefault();
          submitName();
        }}>
              <input ref={nameRef} type="text" aria-label={DSP.name} aria-invalid={!!(nameIssue && nameText)} aria-describedby={nameNoteId} placeholder={DSP.name_placeholder} value={naming.text} maxLength={NAME_BYTES} onInput={e => setNaming({
            ...naming,
            text: (e.target as HTMLInputElement).value
          })} onKeyDown={e => e.key === 'Escape' && setNaming(null)} />
              <button type="submit" className="dx-btn sm solid" disabled={!!nameIssue}>{DSP.name_save}</button>
              <button type="button" className="dx-btn sm" onClick={() => setNaming(null)}>{DSP.name_cancel}</button>
              <p id={nameNoteId} className={`dsp-note${nameIssue ? ' t-err' : ''}`} hidden={!nameMsg}>{nameMsg}</p>
            </form>}
        </div>
      </div>;

  const syncPill = <span className={`dsp-sync s${sync}${playingHere ? '' : ' idle'}`}>{syncText}</span>;
  const graph = <DxCard title={DSP.graph} hint={DSP.graph_hint} right={syncPill}>
        <div className="dsp-graph">
          <EqGraph p={draft} sel={sel} onSelect={setSel} onChange={patch} onAdd={add} />
        </div>
        <div className="dsp-strip">
          <div className="dsp-loud">
            <span className="dsp-strip-label">{DSP.loudest}<HintBtn text={DSP.loudest_hint} /></span>
            <span className={`dsp-strip-v${a.overCap ? ' t-err' : ''}`}>{fmtDb(a.overCap ? a.peakGain : a.maxGain)}</span>
          </div>
          {(a.limiter || limiting) && <span className={`dsp-badge${limiting ? ' live' : ''}`}>
              {limiting ? DSP.limiter_working : DSP.limiter_on}
              <HintBtn text={DSP.limiter_hint} />
            </span>}
          <span className="dsp-grow" />
          {(a.limiter || a.overCap) && <button className="dx-btn sm" onClick={() => edit({
          ...draft,
          pre: autoPreamp(draft)
        })}>{DSP.auto}</button>}
        </div>
        {a.overCap && <p className="dsp-note t-err">{DSP.over_cap}</p>}
        {!a.overCap && status.sy[o] === 2 && <p className="dsp-note t-err">{DSP.refused}</p>}
        {noStages && <p className="dsp-note t-warn">{DSP.no_stages}</p>}
        {notes.length > 0 && <ul className="dsp-notes">
            {notes.map(n => <li key={n}>{n}</li>)}
          </ul>}
      </DxCard>;

  const toneRow = (k: 'bass' | 'mid' | 'tre', label: string, hz: number) => <div className="dsp-tone-row">
        <span className="dsp-edit-label">{label} <small>{fmtHz(hz)}</small></span>
        <MSlider value={draft[k]} min={-TONE.max} max={TONE.max} step={0.5} snap={0} format={v => fmtDb(v)} ariaLabel={label} onPreview={v => edit({
        ...draft,
        [k]: v
      })} onCommit={v => edit({
        ...draft,
        [k]: v
      })} />
      </div>;
  const toneRows = <>
      {toneRow('bass', DSP.bass, TONE.bassHz)}
      {toneRow('mid', DSP.mid, TONE.midHz)}
      {toneRow('tre', DSP.treble, TONE.trebleHz)}
    </>;

  const basicTone = <DxCard title={DSP.tone} hint={DSP.basic_hint} right={syncPill}>
        <div className="dsp-graph">
          <EqGraph p={draft} readOnly />
        </div>
        <div className="dsp-tone">{toneRows}</div>
        {a.overCap && <p className="dsp-note t-err">{DSP.basic_over}</p>}
        {!a.overCap && status.sy[o] === 2 && <p className="dsp-note t-err">{DSP.refused}</p>}
        {noStages && <p className="dsp-note t-warn">{DSP.basic_no_stages}</p>}
      </DxCard>;

  const tone = <DxCard title={DSP.tone}>
        <div className="dsp-tone">
          {toneRows}
          <div className="dsp-tone-row">
            <span className="dsp-edit-label">{DSP.preamp}<HintBtn text={DSP.preamp_hint} /></span>
            <MSlider value={draft.pre} min={RANGE.pre[0]} max={RANGE.pre[1]} step={0.1} snap={0} format={v => fmtDb(v)} ariaLabel={DSP.preamp} onPreview={pre => edit({
            ...draft,
            pre
          })} onCommit={pre => edit({
            ...draft,
            pre
          })} />
          </div>
        </div>
      </DxCard>;

  const filters = <DxCard title={DSP.filters} right={<button className="dx-btn sm" disabled={draft.f.length >= MAX_FILTERS} onClick={() => add()}>{DSP.add}</button>}>
        {draft.f.length === 0 && <p className="dsp-empty">{DSP.none}</p>}
        <div className="dsp-filters">
          {draft.f.map((f, i) => <div key={i} className={`dsp-filter${sel === i ? ' sel' : ''}${f.on ? '' : ' off'}`}>
              <div className="dsp-filter-row">
                <button className="dsp-filter-head" aria-expanded={sel === i} onClick={() => setSel(sel === i ? null : i)}>
                  <span className="dsp-chip" aria-hidden="true" style={{
                background: f.on ? filterColor(i) : 'transparent',
                borderColor: filterColor(i),
                color: f.on ? ON_INK : filterColor(i)
              }}>{i + 1}</span>
                  <span className="dsp-filter-text">
                    <span className="dsp-filter-say">{sayFilter(f)}</span>
                    <span className="dsp-filter-tech">{techFilter(f)}</span>
                  </span>
                </button>
                <Switch on={f.on} label={`${DSP.on}: ${sayFilter(f)}`} onChange={on => patch(i, {
              on
            })} />
              </div>
              {sel === i && <FilterEditor f={f} onPatch={p => patch(i, p)} onRemove={() => remove(i)} />}
            </div>)}
        </div>
        {draft.f.length >= MAX_FILTERS && <p className="dsp-note">{fill(DSP.full, {
          n: MAX_FILTERS
        })}</p>}
      </DxCard>;

  const limiter = <DxCard title={DSP.limiter} hint={DSP.limiter_card_hint} collapsible defaultOpen={limOpen} onToggle={setLimOpen}>
        <div className="dsp-tone">
          <div className="dsp-tone-row">
            <span className="dsp-edit-label">{DSP.ceiling}</span>
            <MSlider value={draft.lim} min={RANGE.lim[0]} max={RANGE.lim[1]} step={0.5} snap={-1} format={v => `${fmtDb(v)}FS`} ariaLabel={DSP.ceiling} onCommit={lim => edit({
            ...draft,
            lim
          })} />
          </div>
          <div className="dsp-tone-row">
            <span className="dsp-edit-label">{DSP.attack}</span>
            <LogSlider value={draft.atk} min={RANGE.atk[0]} max={RANGE.atk[1]} format={fmtMs} round={v => +v.toPrecision(2)} ariaLabel={DSP.attack} onChange={atk => edit({
            ...draft,
            atk
          })} />
          </div>
          <div className="dsp-tone-row">
            <span className="dsp-edit-label">{DSP.release}</span>
            <LogSlider value={draft.rel} min={RANGE.rel[0]} max={RANGE.rel[1]} format={fmtMs} round={v => +v.toPrecision(2)} ariaLabel={DSP.release} onChange={rel => edit({
            ...draft,
            rel
          })} />
          </div>
        </div>
      </DxCard>;

  // Two independent columns on a wide window, so neither waits on the other's row heights; one
  // list in reading order on a phone, so focus order matches what is on screen. The curve is on the
  // left in both views.
  const layout = advanced ? wide ? <>
          <div className="dsp-col">{graph}{filters}</div>
          <div className="dsp-col">{bar}{tone}{limiter}</div>
        </> : <>{bar}{graph}{tone}{filters}{limiter}</> : wide ? <>
          <div className="dsp-col">{basicTone}</div>
          <div className="dsp-col">{bar}</div>
        </> : <>{bar}{basicTone}</>;
  return <div ref={gridRef} className={`dsp-grid${wide ? ' wide' : ''}`}>
      {banner}
      {layout}

      <ImportDialog open={importing} onClose={() => setImporting(false)} onImport={r => {
      setImporting(false);
      if (!edit({
        ...draft,
        pre: r.pre,
        bass: r.bass,
        mid: r.mid,
        tre: r.tre,
        f: r.f,
        preset: ''
      }, {
        preset: ''
      })) return;
      setSel(null);
      setNotes(r.notes.map(n => fill(DSP.notes[n.k as keyof typeof DSP.notes], n as any)));
      okToast(DSP.imported);
    }} />
      <ExportDialog open={exporting} text={exportText(draft)} name={fileName} onClose={() => setExporting(false)} />
      <DxConfirmDialog open={!!confirm} title={confirm?.t ?? ''} body={confirm?.b ?? ''} confirmLabel={confirm?.go ?? ''} danger={confirm?.danger} returnFocus={confirm?.back} onCancel={() => setConfirm(null)} onConfirm={() => {
      const run = confirm?.run;
      setConfirm(null);
      run?.();
    }} />
    </div>;
}
