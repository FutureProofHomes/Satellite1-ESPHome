import { useEffect, useRef, useState } from 'react';
import type { CSSProperties } from 'react';
interface MSliderProps {
  value: number;
  min: number;
  max: number;
  step?: number;
  format?: (v: number) => string;
  snap?: number;
  disabled?: boolean;
  onCommit: (v: number) => void;
  onPreview?: (v: number) => void;
  className?: string;
  ariaLabel?: string;
}
const pctOf = (v: number, min: number, max: number) => (max === min ? 0 : (v - min) / (max - min) * 100).toFixed(1) + '%';

/**
 * Reports while dragging and writes on release - src/ui.jsx's Slider in the design's markup.
 *
 * The write rides the native `change` event, which fires once per release and only when the value
 * moved, so focusing the slider or a cancelled touch costs the device nothing. It is bound by hand
 * because preact/compat turns any onChange on an input into `input`, the per-pixel event.
 *
 * The committed value holds the thumb until the device's echo reaches it (within a step) or five
 * seconds pass, so the thumb doesn't snap back for the length of a poll.
 */
export function MSlider({
  value,
  min,
  max,
  step = 1,
  format,
  snap,
  disabled,
  onCommit,
  onPreview,
  className,
  ariaLabel
}: MSliderProps) {
  const [draft, setDraft] = useState<number | null>(null);
  // The draft is read again from the native handler, which can run before the render that set it.
  const draftRef = useRef<number | null>(null);
  const held = useRef<{ v: number; at: number } | null>(null);
  if (held.current && (Math.abs(value - held.current.v) <= step || Date.now() - held.current.at > 5000)) held.current = null;
  const shown = draft ?? held.current?.v ?? value;
  const setLocal = (v: number | null) => {
    draftRef.current = v;
    setDraft(v);
  };
  // A value one step from `snap` becomes `snap` only when arriving from further away, so stepping
  // off the notch reaches its neighbours instead of being pulled straight back.
  const detent = (v: number) => snap !== undefined && Math.abs(v - snap) <= step && Math.abs(shown - snap) > step ? snap : v;
  const inputRef = useRef<HTMLInputElement>(null);
  const commitRef = useRef(onCommit);
  commitRef.current = onCommit;
  useEffect(() => {
    const el = inputRef.current;
    if (!el) return undefined;
    const onChange = () => {
      const v = draftRef.current ?? Number(el.value);
      held.current = { v, at: Date.now() };
      setLocal(null);
      commitRef.current(v);
    };
    el.addEventListener('change', onChange);
    return () => el.removeEventListener('change', onChange);
  }, []);
  return <div className={`mslider${disabled ? ' disabled' : ''}${className ? ` ${className}` : ''}`}>
      {format && <div className="mslider-top"><span className="mslider-val">{format(shown)}</span></div>}
      <div className="mslider-track">
        {snap !== undefined && <span className="mslider-notch" aria-hidden="true" style={{
        left: `calc(${pctOf(snap, min, max)})`
      }} />}
        <input ref={inputRef} type="range" className="mslider-input" min={min} max={max} step={step} value={shown} disabled={disabled} aria-label={ariaLabel} style={{
        '--pct': pctOf(shown, min, max)
      } as CSSProperties} onInput={e => {
        const v = detent(Number((e.target as HTMLInputElement).value));
        setLocal(v);
        onPreview?.(v);
      }} onPointerCancel={() => setLocal(null)} />
      </div>
    </div>;
}
