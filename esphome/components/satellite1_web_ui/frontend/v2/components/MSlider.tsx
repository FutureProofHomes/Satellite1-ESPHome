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
/**
 * A value's place along the track, for the fill's custom property: the CSS gradient cannot know the
 * value itself.
 */
const pctOf = (v: number, min: number, max: number) => (max === min ? 0 : (v - min) / (max - min) * 100).toFixed(1) + '%';

/**
 * Reports while dragging and writes on release. A number entity write is a round trip to the
 * device and dragging fires an input event per pixel, so sending each one would turn a gesture into
 * forty queued requests against seven sockets.
 *
 * The write rides the native `change` event, which fires once per release and only when the value
 * moved, so focusing the slider or a cancelled touch costs the device nothing. It is bound by hand
 * because preact/compat turns any onChange on an input into `input`, the per-pixel event.
 *
 * `onPreview` is for controls whose effect is drawn elsewhere on the page - the radar plot's
 * detection-range ring follows the drag through it. It must stay local-state-only in the caller; the
 * device still hears nothing until release.
 *
 * `snap` marks a value as the control's home: a notch on the track, and a detent the drag pulls to.
 * The Analog gain slider is the one user - its default matters and lives mid-range, where nothing
 * else on the track says "this is where it was before you touched it".
 *
 * The committed value holds the thumb until the device's echo reaches it (within a step) or five
 * seconds pass, so the thumb doesn't snap back for the length of a poll. The five seconds are the
 * escape for a write the device refused, where the stale value is the truth.
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
      // The displayed value, not the input's: the detent may have coerced the display while the
      // pointer sat a step off it, and committing what the eye saw is the contract.
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
        {/* A native thumb travels between half its own width from either end (26px, app.css), so
            the notch is placed along that travel rather than the bare track. */}
        {snap !== undefined && <span className="mslider-notch" aria-hidden="true" style={{
        left: `calc(13px + (100% - 26px) * ${pctOf(snap, min, max).slice(0, -1)} / 100)`
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
