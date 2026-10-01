import React, { useEffect, useId, useLayoutEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom';
import { TEXT } from '../../../src/copy.js';

/** The Settings pages' building blocks: the design's card, row, fact, confirm, toggle and select. */

let closeActiveHint: (() => void) | null = null;
export function HintBtn({
  text
}: {
  text: string;
}) {
  const [open, setOpen] = useState(false);
  const [pos, setPos] = useState<{
    top: number;
    left: number;
  } | null>(null);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const close = useRef(() => setOpen(false));
  useLayoutEffect(() => {
    if (!open) {
      setPos(null);
      return;
    }
    const id = requestAnimationFrame(() => {
      const b = btn.current?.getBoundingClientRect();
      const w = bubble.current?.offsetWidth ?? 260;
      const h = bubble.current?.offsetHeight ?? 60;
      if (!b) return;
      let left = b.left + b.width / 2 - w / 2;
      left = Math.max(8, Math.min(left, window.innerWidth - w - 8));
      let top = b.bottom + 8;
      if (top + h > window.innerHeight - 8) top = b.top - h - 8;
      setPos({
        top,
        left
      });
    });
    return () => cancelAnimationFrame(id);
  }, [open]);
  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      const t = e.target as Node;
      if (btn.current?.contains(t) || bubble.current?.contains(t)) return;
      setOpen(false);
    };
    const onScroll = () => setOpen(false);
    const onKey = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('mousedown', onDown);
    document.addEventListener('keydown', onKey);
    window.addEventListener('scroll', onScroll, true);
    window.addEventListener('resize', onScroll);
    return () => {
      document.removeEventListener('mousedown', onDown);
      document.removeEventListener('keydown', onKey);
      window.removeEventListener('scroll', onScroll, true);
      window.removeEventListener('resize', onScroll);
      if (closeActiveHint === close.current) closeActiveHint = null;
    };
  }, [open]);
  const toggle = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (!open) {
      if (closeActiveHint && closeActiveHint !== close.current) closeActiveHint();
      closeActiveHint = close.current;
    }
    setOpen(v => !v);
  };
  return <span style={{
    display: 'inline-flex'
  }}>
      <button ref={btn} type="button" className="dx-hint-btn" aria-label="More info" aria-expanded={open} onClick={toggle}>i</button>
      {open && ReactDOM.createPortal(<div ref={bubble} role="tooltip" className="dx-hint-bubble" style={{
      top: pos?.top ?? -9999,
      left: pos?.left ?? -9999,
      visibility: pos ? 'visible' : 'hidden'
    }}>{text}</div>, document.body)}
    </span>;
}

export const Caret = ({
  open,
  size = 14,
  style
}: {
  open: boolean;
  size?: number;
  style?: React.CSSProperties;
}) => <svg className="dx-caret" width={size} height={size} viewBox="0 0 14 14" fill="none" aria-hidden="true" style={{
  transform: open ? 'rotate(180deg)' : undefined,
  transition: 'transform .2s',
  ...style
}}>
    <path d="M3 5l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
  </svg>;

/**
 * `forceOpen` reopens a collapsed card - what a toast's tap intent uses so it never lands on a
 * folded one. `id` is the anchor the intent scrolls to.
 */
export function DxCard({
  title,
  children,
  collapsible = false,
  defaultOpen = true,
  forceOpen = false,
  hint,
  right,
  id
}: {
  title: string;
  children?: React.ReactNode;
  collapsible?: boolean;
  defaultOpen?: boolean;
  forceOpen?: boolean;
  hint?: string;
  right?: React.ReactNode;
  id?: string;
}) {
  const [open, setOpen] = useState(defaultOpen || forceOpen);
  const bodyId = useId();
  useEffect(() => {
    if (forceOpen) setOpen(true);
  }, [forceOpen]);
  // The whole head is the mouse target; the title's button is the keyboard and screen-reader one,
  // its click bubbling up here. The hint button stops its own click.
  return <div className="dx-card" id={id}>
      <div className={`dx-card-head${collapsible ? ' clickable' : ''}`} onClick={collapsible ? () => setOpen(v => !v) : undefined}>
        <h2 className="dx-card-title" style={{
        margin: 0
      }}>{collapsible ? <button type="button" className="dx-card-toggle" aria-expanded={open} aria-controls={bodyId}>{title}</button> : title}</h2>
        {hint && <HintBtn text={hint} />}
        {right}
        {collapsible && <Caret open={open} style={{
        marginLeft: 'auto'
      }} />}
      </div>
      {(!collapsible || open) && <div className="dx-card-body" id={bodyId}>{children}</div>}
    </div>;
}
export function DxRow({
  label,
  hint,
  children
}: {
  label: string;
  hint?: string;
  children?: React.ReactNode;
}) {
  return <div className="dx-row">
      <div className="dx-row-label"><span>{label}</span>{hint && <HintBtn text={hint} />}</div>
      <div className="dx-row-val">{children}</div>
    </div>;
}
/** `tone` colours the value for a reading past its warning or danger line. */
export function DxFact({
  label,
  value,
  unit,
  sub,
  hint,
  tone
}: {
  label: string;
  value: React.ReactNode;
  unit?: string;
  sub?: React.ReactNode;
  hint?: string;
  tone?: 'warn' | 'err' | null;
}) {
  return <div className="dx-fact">
      <div className="dx-fact-label"><span>{label}</span>{hint && <HintBtn text={hint} />}</div>
      <div className="dx-fact-val">
        <span className={`dx-fact-v${tone ? ` t-${tone}` : ''}`}>{value}</span>
        {unit && <span className="dx-fact-unit">{unit}</span>}
        {sub && <div className="dx-fact-sub">{sub}</div>}
      </div>
    </div>;
}
export function DxFacts({
  children
}: {
  children: React.ReactNode;
}) {
  return <div className="dx-facts">{children}</div>;
}

/**
 * A button that asks first, for everything that interrupts or erases. Escape and the scrim cancel.
 * Focus starts on Cancel, the safe answer, and returns to the trigger when the dialog closes.
 */
export function DxConfirm({
  label,
  ariaLabel,
  title,
  body,
  confirmLabel,
  danger = false,
  disabled = false,
  onConfirm,
  solid = false
}: {
  label: string;
  ariaLabel?: string;
  title: string;
  body: string;
  confirmLabel: string;
  danger?: boolean;
  disabled?: boolean;
  onConfirm: () => void;
  solid?: boolean;
}) {
  const [open, setOpen] = useState(false);
  const trigger = useRef<HTMLButtonElement>(null);
  const cancel = useRef<HTMLButtonElement>(null);
  const modalPanelRef = useRef<HTMLDivElement>(null);
  const modalDragStartY = useRef<number | null>(null);
  useEffect(() => {
    if (!open) return;
    document.body.classList.add('has-drawer');
    cancel.current?.focus();
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('keydown', esc);
    return () => {
      document.body.classList.remove('has-drawer');
      document.removeEventListener('keydown', esc);
      trigger.current?.focus();
    };
  }, [open]);
  return <span style={{
    display: 'contents'
  }}>
      <button ref={trigger} className={`dx-btn${solid ? ' solid' : ''}${danger ? ' danger' : ''}`} disabled={disabled} aria-label={ariaLabel} onClick={() => setOpen(true)}>{label}</button>
      {open && ReactDOM.createPortal([<div key="scrim" className="dx-modal-over" onClick={() => setOpen(false)} />, <div key="modal" ref={modalPanelRef} className="dx-modal" role="alertdialog" aria-modal="true" aria-label={title} onClick={e => e.stopPropagation()}>
            <div className="handle" role="button" aria-label="Close" style={{
        touchAction: 'none',
        cursor: 'grab'
      }} onPointerDown={e => {
        if (window.innerWidth >= 1024) return;
        modalDragStartY.current = e.clientY;
        e.currentTarget.setPointerCapture(e.pointerId);
        if (modalPanelRef.current) modalPanelRef.current.style.transition = 'none';
      }} onPointerMove={e => {
        if (modalDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - modalDragStartY.current);
        if (modalPanelRef.current) {
          modalPanelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
          modalPanelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
        }
      }} onPointerUp={e => {
        if (modalDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - modalDragStartY.current);
        const dismiss = () => setOpen(false);
        if (dy > 80) {
          if (modalPanelRef.current) {
            modalPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
            modalPanelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
            modalPanelRef.current.style.opacity = '0';
            setTimeout(dismiss, 210);
          } else dismiss();
        } else if (modalPanelRef.current) {
          modalPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          modalPanelRef.current.style.transform = 'translateX(-50%)';
          modalPanelRef.current.style.opacity = '1';
        }
        modalDragStartY.current = null;
        setTimeout(() => {
          if (modalPanelRef.current) {
            modalPanelRef.current.style.transition = '';
            modalPanelRef.current.style.transform = '';
            modalPanelRef.current.style.opacity = '';
          }
        }, 250);
      }} />
            <p className="dx-modal-title">{title}</p>
            <p className="dx-modal-body">{body}</p>
            <div className="dx-modal-actions">
              <button ref={cancel} className="dx-btn" onClick={() => setOpen(false)}>{TEXT.cancel}</button>
              <button className={`dx-btn solid${danger ? ' danger' : ''}`} onClick={() => {
          setOpen(false);
          onConfirm();
        }}>{confirmLabel}</button>
            </div>
          </div>], document.body)}
    </span>;
}
export function DxToggle({
  checked,
  onChange,
  label
}: {
  checked: boolean;
  onChange: (v: boolean) => void;
  label?: string;
}) {
  return <button role="switch" aria-checked={checked} aria-label={label} className={`dx-toggle${checked ? ' on' : ''}`} onClick={() => onChange(!checked)}>
      <span className="dx-toggle-thumb" />
    </button>;
}

/** Options are plain strings, or [value, label] pairs where the two differ. */
export function DxSelect({
  value,
  options,
  onChange,
  label
}: {
  value: string;
  options: (string | [string, string])[];
  onChange: (v: string) => void;
  label?: string;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const close = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && setOpen(false);
    document.addEventListener('mousedown', close);
    document.addEventListener('keydown', esc);
    return () => {
      document.removeEventListener('mousedown', close);
      document.removeEventListener('keydown', esc);
    };
  }, [open]);
  const pairs = options.map(o => Array.isArray(o) ? o : [o, o]);
  const shown = pairs.find(([v]) => v === value)?.[1] ?? value;
  return <div ref={ref} className="dx-sel" style={{
    position: 'relative',
    zIndex: 20
  }}>
      <button className={`dx-sel-btn${open ? ' open' : ''}`} aria-haspopup="listbox" aria-expanded={open} aria-label={label} onClick={() => setOpen(v => !v)}>
        <span>{shown}</span>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true" style={{
        transform: open ? 'rotate(180deg)' : undefined,
        transition: 'transform .2s'
      }}>
          <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
      </button>
      {open && <div className="dx-sel-pop" role="listbox" aria-label={label}>
          {pairs.map(([v, l]) => <button key={v} role="option" aria-selected={v === value} className={`dx-sel-opt${v === value ? ' active' : ''}`} onClick={() => {
        onChange(v);
        setOpen(false);
      }}>
              {v === value && <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true"><path d="M2 6l3 3 5-5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>}
              <span>{l}</span>
            </button>)}
        </div>}
    </div>;
}

/**
 * Copies to the clipboard, true on success. navigator.clipboard does not exist on the plain-HTTP
 * origins the device pages always are, so the textarea fallback is what usually runs.
 */
export function copyText(text: string) {
  try {
    if (navigator.clipboard?.writeText) {
      navigator.clipboard.writeText(text);
    } else {
      const ta = document.createElement('textarea');
      ta.value = text;
      ta.style.cssText = 'position:fixed;opacity:0';
      document.body.appendChild(ta);
      ta.select();
      document.execCommand('copy');
      ta.remove();
    }
    return true;
  } catch {
    return false;
  }
}

/**
 * Saves as an in-page Blob rather than a link to the device: Chromium interposes a "may have been
 * tampered with" interstitial on any download delivered over plain HTTP, and a Blob is local.
 */
export function saveBlob(blob: Blob, name: string) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
  URL.revokeObjectURL(url);
}

/** "Copied" for two seconds after a successful copy. */
export function useCopied(): [boolean, (text: string) => void] {
  const [copied, setCopied] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => () => {
    if (timer.current) clearTimeout(timer.current);
  }, []);
  return [copied, (text: string) => {
    if (!copyText(text)) return;
    setCopied(true);
    if (timer.current) clearTimeout(timer.current);
    timer.current = setTimeout(() => setCopied(false), 2000);
  }];
}
