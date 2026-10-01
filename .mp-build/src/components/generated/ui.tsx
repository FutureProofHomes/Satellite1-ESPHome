/**
 * Shared primitives, ported from the Satellite1 Web UI's ui.jsx (Preact -> React).
 * Class names match the ported stylesheet exactly, so the visuals come from the
 * device firmware's own CSS.
 */
import React, { useEffect, useLayoutEffect, useRef, useState } from 'react';

/* ------------------------------------------------------------------ */
/* Drawn glyphs                                                        */
/* ------------------------------------------------------------------ */

export const ni = (children: React.ReactNode) => <svg className="ni" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    {children}
  </svg>;
export const N_HOME = ni(<>
    <path d="M2.8 8.3 8 3.6l5.2 4.7" />
    <path d="M4.4 7.6V13h7.2V7.6" />
  </>);
export const N_WAKE = ni(<>
    <path d="M2.8 6.8v2.4" />
    <path d="M5.4 4.8v6.4" />
    <path d="M8 3v10" />
    <path d="M10.6 4.8v6.4" />
    <path d="M13.2 6.8v2.4" />
  </>);
export const N_AUDIO = ni(<>
    <path d="M2.8 6.4h2.4L8.6 3.8v8.4L5.2 9.6H2.8z" />
    <path d="M11.2 6a3.1 3.1 0 0 1 0 4" />
  </>);
export const N_PRES = ni(<>
    <path d="M13.2 8A5.2 5.2 0 1 1 8 2.8" />
    <path d="M8 8l3.7-3.7" />
    <circle cx="8" cy="8" r="1" fill="currentColor" stroke="none" />
  </>);
export const N_DIAG = ni(<path d="M2 8.5h2.8L6.4 5l3.2 6.5 1.6-3H14" />);
export const N_CHAT = ni(<path d="M3 3.5h10v6.5H8.2L5.4 12.6V10H3z" />);
export const N_OUT = ni(<>
    <path d="M6.5 3H3.5v10h3" />
    <path d="M6.8 8h6" />
    <path d="M10.6 5.8 12.8 8l-2.2 2.2" />
  </>);
export const N_SEARCH = ni(<>
    <circle cx="7" cy="7" r="4.4" />
    <path d="M10.4 10.4 14 14" />
  </>);
export const N_BELL = ni(<>
    <path d="M8 2.6a3.9 3.9 0 0 0-3.9 3.9v2.9L2.9 11.4h10.2L11.9 9.4V6.5A3.9 3.9 0 0 0 8 2.6Z" />
    <path d="M6.7 13.4a1.4 1.4 0 0 0 2.6 0" />
  </>);

/* The toast/notification glyphs, one per kind. */
export const T_ICONS: Record<string, React.ReactNode> = {
  err: ni(<>
      <circle cx="8" cy="8" r="6.1" />
      <path d="M8 4.9v3.7" />
      <path d="M8 11.2h.01" />
    </>),
  warn: ni(<>
      <path d="M8 2.5 14.1 13H1.9L8 2.5Z" />
      <path d="M8 6.6v2.8" />
      <path d="M8 11.4h.01" />
    </>),
  ok: ni(<>
      <circle cx="8" cy="8" r="6.1" />
      <path d="M5.1 8.3 7 10.2l3.9-4.1" />
    </>),
  info: ni(<>
      <circle cx="8" cy="8" r="6.1" />
      <path d="M8 7.4v3.4" />
      <path d="M8 5.1h.01" />
    </>)
};

/** The FutureProofHomes mark, inlined from the vector master. */
export const Logo = ({
  cls = 'login-logo'
}: {
  cls?: string;
}) => <svg className={cls} viewBox="0 0 79.375 79.375" fill="none" stroke="currentColor" strokeLinecap="square" aria-hidden="true">
    <g transform="matrix(1.6754,0,0,1.6754,84.9754,-16.4554)" strokeWidth="1.31">
      <path d="m -45.44,31.52 11,-10.96 11,10.96 v 16.25 l -5.82,.01" />
      <path d="m -27.49,20.19 10.94,9.29 v 18.3 l 8.35,.03 V 29.29 l -10.94,-9.66 -1.92,1.73" />
      <path strokeWidth="1.36" d="m -32.25,47.75 c 0,-7.27 -5.89,-13.42 -13.16,-13.42 h 0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -36.31,47.73 c .01,-.12 .01,-.12 .01,-.24 0,-5.03 -4.08,-9.11 -9.11,-9.11 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -40.36,47.8 c .01,-.12 .01,-.18 .01,-.31 0,-2.8 -2.27,-5.06 -5.06,-5.06 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -45.41,46.48 a 1.01,1.01 0 0 0 -.15,.01 v 1.38 h 1.09 a 1.01,1.01 0 0 0 .07,-.37 1.01,1.01 0 0 0 -1.01,-1.01 z" />
    </g>
  </svg>;

/* ------------------------------------------------------------------ */
/* Hint                                                                */
/* ------------------------------------------------------------------ */

let openHintSetter: ((v: boolean) => void) | null = null;
export function Hint({
  text
}: {
  text: React.ReactNode;
}) {
  const [open, setOpen] = useState(false);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState<{
    left: number;
    top: number;
  } | null>(null);
  useEffect(() => {
    if (!open) return;
    if (openHintSetter && openHintSetter !== setOpen) openHintSetter(false);
    openHintSetter = setOpen;
    const dismiss = (e: PointerEvent) => {
      if (!btn.current?.contains(e.target as Node) && !bubble.current?.contains(e.target as Node)) setOpen(false);
    };
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('pointerdown', dismiss, true);
    document.addEventListener('keydown', esc);
    return () => {
      document.removeEventListener('pointerdown', dismiss, true);
      document.removeEventListener('keydown', esc);
      if (openHintSetter === setOpen) openHintSetter = null;
    };
  }, [open]);
  useLayoutEffect(() => {
    if (!open || !bubble.current || !btn.current) return;
    const t = btn.current.getBoundingClientRect();
    const b = bubble.current.getBoundingClientRect();
    const m = 8;
    let left = t.left + t.width / 2 - b.width / 2;
    left = Math.max(m, Math.min(left, window.innerWidth - b.width - m));
    const below = t.bottom + 6;
    const top = below + b.height + m > window.innerHeight ? Math.max(m, t.top - b.height - 6) : below;
    setPos({
      left,
      top
    });
  }, [open]);
  return <>
      <button ref={btn} className="hint-btn" aria-label="Explain" aria-expanded={open} onClick={() => setOpen(v => !v)}>
        i
      </button>
      {open && <div ref={bubble} className="hint" role="tooltip" style={pos ? {
      left: `${pos.left}px`,
      top: `${pos.top}px`
    } : {
      opacity: 0
    }}>
          {text}
        </div>}
    </>;
}

/* ------------------------------------------------------------------ */
/* Layout                                                              */
/* ------------------------------------------------------------------ */

export function Chevron({
  down,
  up,
  cls
}: {
  down?: boolean;
  up?: boolean;
  cls?: string;
}) {
  return <svg className={`chev${down ? ' down' : ''}${up ? ' up' : ''}${cls ? ` ${cls}` : ''}`} viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.9" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M4.2 2.4 8.3 6l-4.1 3.6" />
    </svg>;
}
export function Arrow({
  cls
}: {
  cls?: string;
}) {
  return <svg className={`chev${cls ? ` ${cls}` : ''}`} viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.9" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M1.9 6h8" />
      <path d="M6.6 2.9 9.9 6l-3.3 3.1" />
    </svg>;
}
export function Card({
  title,
  icon,
  hint,
  right,
  children,
  collapsible,
  defaultOpen = false,
  ...rest
}: {
  title?: React.ReactNode;
  icon?: React.ReactNode;
  hint?: React.ReactNode;
  right?: React.ReactNode;
  children?: React.ReactNode;
  collapsible?: boolean;
  defaultOpen?: boolean;
  [key: string]: any;
}) {
  const [open, setOpen] = useState(() => collapsible ? defaultOpen : true);
  return <section className={`card${collapsible ? ' card-c' : ''}`} {...rest}>
      {title && <h2>
          {icon && <span className="cico">{icon}</span>}
          {collapsible ? <button className="card-t" aria-expanded={open} onClick={() => setOpen(!open)}>
              <span>{title}</span>
              <Chevron down={open} cls="caret-s" />
            </button> : <span>{title}</span>}
          {hint && <Hint text={hint} />}
          {right && <span className="card-right">{right}</span>}
        </h2>}
      {open && children}
    </section>;
}

/* ------------------------------------------------------------------ */
/* The chip sparkline                                                  */
/* ------------------------------------------------------------------ */

/** Bucket the history into points and smooth a quadratic bezier through them (mini-graph-card style). */
function sparkPaths(pts: number[]) {
  if (!pts || pts.length < 3) return null;
  const lo = Math.min(...pts);
  const hi = Math.max(...pts);
  const amp = Math.max(hi - lo, 1e-6);
  const X = (i: number) => i / (pts.length - 1) * 100;
  const Y = (v: number) => 88 - (v - lo) / amp * 62;
  let line = `M ${X(0)} ${Y(pts[0])}`;
  for (let i = 1; i < pts.length; i++) {
    const mx = (X(i - 1) + X(i)) / 2;
    const my = (Y(pts[i - 1]) + Y(pts[i])) / 2;
    line += ` Q ${X(i - 1)} ${Y(pts[i - 1])} ${mx} ${my}`;
  }
  line += ` L ${X(pts.length - 1)} ${Y(pts[pts.length - 1])}`;
  const fill = `${line} L 100 100 L 0 100 Z`;
  return {
    line,
    fill
  };
}
function Spark({
  pts,
  seed
}: {
  pts: number[];
  seed: string;
}) {
  const made = sparkPaths(pts);
  if (!made) return null;
  const gid = `spkg-${seed.replace(/[^a-z0-9]/gi, '')}`;
  return <svg className="spark" viewBox="0 0 100 100" preserveAspectRatio="none" pointerEvents="none" aria-hidden="true">
      <defs>
        <linearGradient id={gid} x1="0" y1="0" x2="0" y2="1">
          <stop offset="0" stopColor="var(--accent)" stopOpacity="0.18" />
          <stop offset="1" stopColor="var(--accent)" stopOpacity="0" />
        </linearGradient>
      </defs>
      <path d={made.fill} fill={`url(#${gid})`} stroke="none" />
      <path d={made.line} fill="none" stroke="var(--accent)" strokeWidth="1.5" opacity="0.45" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke" />
    </svg>;
}
export function Pill({
  id,
  open,
  setOpen,
  label,
  value,
  spark
}: {
  id: string;
  open: string | null;
  setOpen: (v: string | null) => void;
  label: string;
  value: string;
  spark?: {
    pts: number[];
    seed: string;
  };
}) {
  const on = open === id;
  return <button className={`pill${on ? ' on' : ''}`} onClick={() => setOpen(on ? null : id)}>
      {spark && <Spark {...spark} />}
      <span className="pill-v">{value}</span>
      <span className="pill-l">{label}</span>
      <Chevron down={!on} up={on} cls="pill-c" />
    </button>;
}
export function Row({
  label,
  hint,
  children,
  sub
}: {
  label: React.ReactNode;
  hint?: React.ReactNode;
  children?: React.ReactNode;
  sub?: React.ReactNode;
}) {
  return <div className="ctl">
      <div className="ctl-label">
        <span>{label}</span>
        {hint && <Hint text={hint} />}
        {sub && <span className="ctl-sub">{sub}</span>}
      </div>
      <div className="ctl-body">{children}</div>
    </div>;
}
export function Fact({
  label,
  value,
  unit,
  hint,
  tone,
  sub
}: {
  label: React.ReactNode;
  value: React.ReactNode;
  unit?: React.ReactNode;
  hint?: React.ReactNode;
  tone?: string | null;
  sub?: React.ReactNode;
}) {
  return <div className="fact">
      <div className="fact-label">
        <span>{label}</span>
        {hint && <Hint text={hint} />}
      </div>
      <div className={`fact-value${tone ? ` t-${tone}` : ''}`}>
        {value}
        {unit && <span className="fact-unit">{unit}</span>}
      </div>
      {sub && <div className="fact-sub">{sub}</div>}
    </div>;
}

/* ------------------------------------------------------------------ */
/* Inputs                                                              */
/* ------------------------------------------------------------------ */

export function Toggle({
  checked,
  disabled,
  onChange
}: {
  checked: boolean;
  disabled?: boolean;
  onChange: (v: boolean) => void;
}) {
  return <button className={`sw${checked ? ' on' : ''}`} role="switch" aria-checked={!!checked} disabled={disabled} onClick={() => onChange(!checked)}>
      <span className="sw-knob" />
    </button>;
}

/** The track's filled portion as a custom property; the CSS gradient can't know the value itself. */
export const rangeFill = (shown: number, min: number, max: number) => ({
  ['--p' as string]: `${((shown - min) / (max - min || 1) * 100).toFixed(1)}%`
}) as React.CSSProperties;
export function Slider({
  value,
  min,
  max,
  step,
  disabled,
  format,
  onCommit,
  onPreview,
  snap
}: {
  value: number;
  min: number;
  max: number;
  step?: number;
  disabled?: boolean;
  format?: (v: number) => string;
  onCommit: (v: number) => void;
  onPreview?: (v: number) => void;
  snap?: number;
}) {
  const [local, setLocal] = useState<number | null>(null);
  const shown = local ?? value;
  const detent = (v: number) => {
    if (snap == null) return v;
    const s = step || 1;
    return Math.abs(v - snap) <= s && Math.abs(shown - snap) > s ? snap : v;
  };
  const snapFrac = snap == null ? 0 : (snap - min) / (max - min || 1);
  return <div className="slider">
      <div className="slider-rail">
        <input type="range" min={min} max={max} step={step} value={shown} disabled={disabled} style={rangeFill(shown, min, max)} onInput={e => {
        const v = detent(Number((e.target as HTMLInputElement).value));
        setLocal(v);
        if (onPreview) onPreview(v);
      }} onChange={e => {
        const v = local ?? Number((e.target as HTMLInputElement).value);
        setLocal(null);
        onCommit(v);
      }} />
        {snap != null && shown !== snap && <span className="slider-notch" style={{
        left: `calc(11px + (100% - 22px) * ${snapFrac.toFixed(3)})`
      }} />}
      </div>
      <span className="slider-val num">{format ? format(shown) : shown}</span>
    </div>;
}
export function Select({
  value,
  options,
  disabled,
  onChange
}: {
  value: string;
  options: (string | [string, string])[];
  disabled?: boolean;
  onChange: (v: string) => void;
}) {
  return <select className="sel" disabled={disabled} value={value} onChange={e => onChange(e.target.value)}>
      {(options || []).map(o => {
      const [v, label] = Array.isArray(o) ? o : [o, o];
      return <option key={v} value={v}>
            {label}
          </option>;
    })}
    </select>;
}

/** A button that asks first, as a modal. */
export function Confirm({
  label,
  title,
  body,
  confirmLabel,
  danger,
  solid,
  disabled,
  onConfirm
}: {
  label: React.ReactNode;
  title?: string;
  body?: React.ReactNode;
  confirmLabel?: string;
  danger?: boolean;
  solid?: boolean;
  disabled?: boolean;
  onConfirm: () => void;
}) {
  const [open, setOpen] = useState(false);
  useEffect(() => {
    if (!open) return;
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('keydown', esc);
    return () => document.removeEventListener('keydown', esc);
  }, [open]);
  return <>
      <button className={`btn${danger ? ' danger' : ''}${solid ? ' solid' : ''}`} disabled={disabled} onClick={() => setOpen(true)}>
        {label}
      </button>
      {open && <div className="scrim center" onClick={() => setOpen(false)}>
          <div className="modal" role="alertdialog" aria-modal="true" aria-label={title} onClick={e => e.stopPropagation()}>
            <h3 className="modal-t">{title || 'Are you sure?'}</h3>
            <p className="modal-b">{body}</p>
            <div className="modal-btns">
              <button className="btn ghost" onClick={() => setOpen(false)}>
                Cancel
              </button>
              <button className={`btn solid${danger ? ' danger' : ''}`} onClick={() => {
            setOpen(false);
            onConfirm();
          }}>
                {confirmLabel || 'Confirm'}
              </button>
            </div>
          </div>
        </div>}
    </>;
}
export function Btn({
  children,
  onClick,
  disabled,
  danger,
  solid,
  cls
}: {
  children: React.ReactNode;
  onClick?: () => void;
  disabled?: boolean;
  danger?: boolean;
  solid?: boolean;
  cls?: string;
}) {
  return <button className={`btn${danger ? ' danger' : ''}${solid ? ' solid' : ''}${cls ? ` ${cls}` : ''}`} disabled={disabled} onClick={onClick}>
      {children}
    </button>;
}
export function Empty({
  icon,
  text
}: {
  icon: React.ReactNode;
  text: string;
}) {
  return <div className="empty">
      {icon}
      <p className="dim sm">{text}</p>
    </div>;
}

/** The tri-state checkbox the player trees and wake word pickers share. */
export function Check({
  state,
  disabled,
  onClick,
  label
}: {
  state: 'on' | 'off' | 'mixed';
  disabled?: boolean;
  onClick?: () => void;
  label?: string;
}) {
  const on = state === 'on';
  const mixed = state === 'mixed';
  return <button className={`cb ${state}${disabled ? ' dim' : ''}`} role="checkbox" aria-checked={on ? 'true' : mixed ? 'mixed' : 'false'} aria-label={label} disabled={disabled} onClick={onClick}>
      <svg className="cb-i" viewBox="0 0 16 16" aria-hidden="true">
        <rect className="cb-box" x="1.6" y="1.6" width="12.8" height="12.8" rx="3.4" />
        {on && <path className="cb-mark" d="M4.5 8.2 6.9 10.6l4.6-5.2" />}
        {mixed && <path className="cb-mark" d="M4.8 8h6.4" />}
      </svg>
    </button>;
}

/* ------------------------------------------------------------------ */
/* Drawer citizenship                                                  */
/* ------------------------------------------------------------------ */

let drawersOpen = 0;

/** Escape closes the drawer; only one drawer stands at a time, app-wide. */
export function useDrawer(id: string, open: boolean, onClose: () => void) {
  useEffect(() => {
    if (!open) return undefined;
    if (++drawersOpen === 1) document.documentElement.classList.add('held');
    window.dispatchEvent(new CustomEvent('drawer', {
      detail: id
    }));
    const other = (e: Event) => (e as CustomEvent).detail !== id && onClose();
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && onClose();
    window.addEventListener('drawer', other);
    document.addEventListener('keydown', esc);
    return () => {
      if (--drawersOpen === 0) document.documentElement.classList.remove('held');
      window.removeEventListener('drawer', other);
      document.removeEventListener('keydown', esc);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [id, open]);
}

/** Swipe-to-dismiss for the sliding drawers. `dir`: 1 bottom drawers (down), -1 the top one (up). */
export function useSheetDrag(onClose: () => void, dir = 1): [React.CSSProperties | null, any] {
  const [dy, setDy] = useState(0);
  const st = useRef<{
    y: number;
    t: number;
    id: number;
    held: boolean;
  } | null>(null);
  return [dy ? {
    transform: `translateY(${dy * dir}px)`,
    transition: 'none'
  } as React.CSSProperties : null, {
    onPointerDown: (e: React.PointerEvent) => {
      if (!(e.target as HTMLElement).closest('[data-grab]')) return;
      st.current = {
        y: e.clientY,
        t: Date.now(),
        id: e.pointerId,
        held: false
      };
    },
    onPointerMove: (e: React.PointerEvent) => {
      const s = st.current;
      if (!s) return;
      if (!s.held) {
        if (Math.abs(e.clientY - s.y) < 7) return;
        s.held = true;
        try {
          (e.currentTarget as HTMLElement).setPointerCapture(s.id);
        } catch {
          /* keep following the bubbled events instead */
        }
      }
      setDy(Math.max(0, (e.clientY - s.y) * dir));
    },
    onPointerUp: () => {
      const s = st.current;
      st.current = null;
      if (!s || !s.held) return;
      const flick = dy > 24 && Date.now() - s.t < 250;
      setDy(0);
      if (dy > 80 || flick) onClose();
    },
    onPointerCancel: () => {
      st.current = null;
      setDy(0);
    }
  }];
}

/* Media glyphs (from media.jsx). */
export const mi = (path: React.ReactNode, extra?: React.ReactNode) => <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    {path}
    {extra}
  </svg>;
export const I_NOTE = <svg className="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M11.8 1.6 6.4 3v7.2a2.3 2.3 0 1 0 1.2 2V6.2l4.2-1.1v3.8a2.3 2.3 0 1 0 1.2 2V1.9a.3.3 0 0 0-.4-.3H11.8Z" />
  </svg>;
export const I_PLAY = <svg className="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M5.4 3c0-.5.6-.9 1-.6l7 4.5c.4.3.4.9 0 1.2l-7 4.5c-.4.3-1 0-1-.6V3Z" />
  </svg>;
export const I_PAUSE = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" aria-hidden="true">
    <path d="M5.4 3.6v8.8M10.6 3.6v8.8" />
  </svg>;
export const I_SPK = mi(<rect x="4" y="1.8" width="8" height="12.4" rx="1.6" />, <>
    <circle cx="8" cy="10.3" r="2.1" />
    <circle cx="8" cy="4.9" r="0.4" />
  </>);
export const I_PLUS = mi(<path d="M8 3.5v9M3.5 8h9" />);
export const I_MINUS = mi(<path d="M3.5 8h9" />);
export const I_VOL = mi(<path d="M2.8 6.4h2.4L8.6 3.8v8.4L5.2 9.6H2.8z" />, <path d="M11.2 6a3.1 3.1 0 0 1 0 4" />);
export const I_SHUFFLE = mi(<path d="M1.5 4.5h2.6c3.6 0 4.2 7 7.8 7h2.1M1.5 11.5h2.6c1.3 0 2.2-.9 2.9-2M14 4.5h-2.1c-1.4 0-2.3 1-3 2.1" />, <path d="M12.2 2.7l1.8 1.8-1.8 1.8M12.2 9.7l1.8 1.8-1.8 1.8" />);
export const I_REPEAT = mi(<path d="M2.5 6.5v-.4A2.6 2.6 0 0 1 5.1 3.5h7.4M13.5 9.5v.4a2.6 2.6 0 0 1-2.6 2.6H3.5" />, <path d="M10.8 1.8 12.6 3.5l-1.8 1.7M5.2 14.2 3.4 12.5l1.8-1.7" />);
export const I_PREV = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden="true">
    <path d="M3.7 3.6v8.8" />
    <path fill="currentColor" stroke="none" d="M12.9 3.9v8.2c0 .5-.6.8-1 .5L6.3 8.5a.6.6 0 0 1 0-1l5.6-4.1c.4-.3 1 0 1 .5Z" />
  </svg>;
export const I_NEXT = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" aria-hidden="true">
    <path d="M12.3 3.6v8.8" />
    <path fill="currentColor" stroke="none" d="M3.1 3.9v8.2c0 .5.6.8 1 .5l5.6-4.1a.6.6 0 0 0 0-1L4.1 3.4c-.4-.3-1 0-1 .5Z" />
  </svg>;

/** The mdi:cog Home Assistant draws on a device row - the setup wizard's step 2 points at it. */
export const Cog = () => <svg className="fix-cog" viewBox="0 0 24 24" aria-hidden="true">
    <path d="M12,15.5A3.5,3.5 0 0,1 8.5,12A3.5,3.5 0 0,1 12,8.5A3.5,3.5 0 0,1 15.5,12A3.5,3.5 0 0,1 12,15.5M19.43,12.97C19.47,12.65 19.5,12.33 19.5,12C19.5,11.67 19.47,11.34 19.43,11L21.54,9.37C21.73,9.22 21.78,8.95 21.66,8.73L19.66,5.27C19.54,5.05 19.27,4.96 19.05,5.05L16.56,6.05C16.04,5.66 15.5,5.32 14.87,5.07L14.5,2.42C14.46,2.18 14.25,2 14,2H10C9.75,2 9.54,2.18 9.5,2.42L9.13,5.07C8.5,5.32 7.96,5.66 7.44,6.05L4.95,5.05C4.73,4.96 4.46,5.05 4.34,5.27L2.34,8.73C2.21,8.95 2.27,9.22 2.46,9.37L4.57,11C4.53,11.34 4.5,11.67 4.5,12C4.5,12.33 4.53,12.65 4.57,12.97L2.46,14.63C2.27,14.78 2.21,15.05 2.34,15.27L4.34,18.73C4.46,18.95 4.73,19.03 4.95,18.95L7.44,17.94C7.96,18.34 8.5,18.68 9.13,18.93L9.5,21.58C9.54,21.82 9.75,22 10,22H14C14.25,22 14.46,21.82 14.5,21.58L14.87,18.93C15.5,18.67 16.04,18.34 16.56,17.94L19.05,18.95C19.27,19.03 19.54,18.95 19.66,18.73L21.66,15.27C21.78,15.05 21.73,14.78 21.54,14.63L19.43,12.97Z" />
  </svg>;

/** The checkbox walk-through's step-2 renderer: the string's %c becomes the cog, %s the name. */
export const cogStep = (tpl: string, name: string) => {
  const [before, after] = tpl.split('%c');
  return <>
      {before}
      <Cog />
      {after.replace('%s', name)}
    </>;
};