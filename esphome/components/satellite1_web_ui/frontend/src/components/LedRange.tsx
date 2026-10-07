import { useEffect, useRef, useState } from 'react';

/**
 * A just-written value, held over the stale state that follows it. A control writes and then keeps
 * rendering from the entity or poll behind it, which does not know about the write for up to a poll
 * interval - so a slider's thumb snapped back to the old value and jumped forward again when the
 * echo landed (reported from Safari on the phone, September 2026, but present everywhere). The held
 * value wins until the echo comes within `tol` of it, which absorbs rounding on values that
 * round-trip through 0-255, or until five seconds pass - the escape for a write the device refused,
 * where the stale value is the truth. It re-renders on hold so callers need no state of their own.
 */
export function useHeld(value: number, tol: number): [number, (v: number) => void] {
  const held = useRef<{ v: number; at: number } | null>(null);
  const [, bump] = useState(0);
  if (held.current && (Math.abs(value - held.current.v) <= tol || Date.now() - held.current.at > 5000)) held.current = null;
  return [held.current ? held.current.v : value, v => {
    held.current = { v, at: Date.now() };
    bump(n => n + 1);
  }];
}

/**
 * The LED sliders: they follow the drag on screen and write once, on the native change at release,
 * because a write per input event would queue a request per pixel of drag (MSlider has the same
 * rule). preact/compat turns onChange into input events, so the commit listens for it directly.
 * `onDrag` sees every step, for a preview that follows the thumb without writing.
 */
export function LedRange({
  label,
  text,
  value,
  min = 0,
  max,
  tol,
  className,
  ariaLabel,
  onCommit,
  onDrag
}: {
  label: string;
  text: (v: number) => string;
  value: number;
  min?: number;
  max: number;
  tol: number;
  className?: string;
  ariaLabel: string;
  onCommit: (v: number) => void;
  onDrag?: (v: number) => void;
}) {
  const [held, hold] = useHeld(value, tol);
  const [draft, setDraft] = useState<number | null>(null);
  const shown = draft ?? held;
  const input = useRef<HTMLInputElement>(null);
  const commit = useRef(onCommit);
  commit.current = onCommit;
  useEffect(() => {
    const el = input.current;
    if (!el) return;
    const on = () => {
      const v = Number(el.value);
      hold(v);
      setDraft(null);
      commit.current(v);
    };
    el.addEventListener('change', on);
    return () => el.removeEventListener('change', on);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  return <label className="hue-label"><small className="muted hue-cap">{label}</small><span>{text(shown)}</span><input ref={input} className={'hue' + (className ? ' ' + className : '')} type="range" min={min} max={max} value={shown} onChange={e => {
      const v = Number(e.currentTarget.value);
      setDraft(v);
      onDrag?.(v);
    }} aria-label={ariaLabel} /></label>;
}
