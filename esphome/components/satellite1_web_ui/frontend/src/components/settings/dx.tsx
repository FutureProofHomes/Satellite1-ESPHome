import React, { useEffect, useId, useRef, useState } from 'react';
import { Drawer, Presence } from '../Drawer';
import { TEXT } from '../../copy.js';
import { HintBtn } from '../bits';

/** The Settings pages' building blocks: the design's card, row, fact, confirm, toggle and select. */

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
  return <>
      <button ref={trigger} className={`dx-btn${solid ? ' solid' : ''}${danger ? ' danger' : ''}`} disabled={disabled} aria-label={ariaLabel} onClick={() => setOpen(true)}>{label}</button>
      <DxConfirmDialog open={open} title={title} body={body} confirmLabel={confirmLabel} danger={danger} returnFocus={trigger} onCancel={() => setOpen(false)} onConfirm={() => {
      setOpen(false);
      onConfirm();
    }} />
    </>;
}

/** DxConfirm's dialog on its own, for a confirm something other than a button opens. */
export function DxConfirmDialog({
  open,
  title,
  body,
  confirmLabel,
  danger = false,
  returnFocus,
  onCancel,
  onConfirm
}: {
  open: boolean;
  title: string;
  body: string;
  confirmLabel: string;
  danger?: boolean;
  returnFocus?: React.RefObject<HTMLElement>;
  onCancel: () => void;
  onConfirm: () => void;
}) {
  const cancel = useRef<HTMLButtonElement>(null);
  return <Presence>{open && <Drawer label={title} role="alertdialog" initialFocus={cancel} returnFocus={returnFocus} onClose={onCancel} className="dx-modal">
        <p className="dx-modal-title">{title}</p>
        <p className="dx-modal-body">{body}</p>
        <div className="dx-modal-actions">
          <button ref={cancel} className="dx-btn" onClick={onCancel}>{TEXT.cancel}</button>
          <button className={`dx-btn solid${danger ? ' danger' : ''}`} onClick={onConfirm}>{confirmLabel}</button>
        </div>
      </Drawer>}</Presence>;
}

/**
 * Options are plain strings, or [value, label] pairs where the two differ. Drawn rather than a
 * native <select>, whose popup Safari renders ignoring option styling entirely.
 */
export function DxSelect({
  value,
  options,
  onChange,
  label,
  buttonRef
}: {
  value: string;
  options: (string | [string, string])[];
  onChange: (v: string) => void;
  label?: string;
  buttonRef?: React.RefObject<HTMLButtonElement>;
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
  return <div ref={ref} className="dx-sel">
      <button ref={buttonRef} className={`dx-sel-btn${open ? ' open' : ''}`} aria-haspopup="listbox" aria-expanded={open} aria-label={label} onClick={() => setOpen(v => !v)}>
        <span>{shown}</span>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
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
    // The text is on screen, so selecting it by hand still works.
    return false;
  }
}

/**
 * Saves as an in-page Blob rather than a link to the device: Chromium interposes a "may have been
 * tampered with" interstitial on any download delivered over plain HTTP, and a Blob is local
 * (Crash reports in docs/web-ui.md says why nothing else worked).
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
