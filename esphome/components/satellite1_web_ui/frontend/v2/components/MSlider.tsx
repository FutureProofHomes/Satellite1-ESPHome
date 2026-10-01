import { useState } from 'react';
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
  const shown = draft ?? value;
  const commit = (raw: number) => {
    let v = raw;
    if (snap !== undefined && Math.abs(v - snap) <= step) v = snap;
    setDraft(null);
    onCommit(v);
  };
  return <div className={`mslider${disabled ? ' disabled' : ''}${className ? ` ${className}` : ''}`}>
      {format && <div className="mslider-top"><span className="mslider-val">{format(shown)}</span></div>}
      <div className="mslider-track">
        {snap !== undefined && <span className="mslider-notch" aria-hidden="true" style={{
        left: `calc(${pctOf(snap, min, max)})`
      }} />}
        <input type="range" className="mslider-input" min={min} max={max} step={step} value={shown} disabled={disabled} aria-label={ariaLabel} style={{
        '--pct': pctOf(shown, min, max)
      } as CSSProperties} onChange={e => {
        const v = Number(e.target.value);
        setDraft(v);
        onPreview?.(v);
      }} onPointerUp={e => commit(Number((e.target as HTMLInputElement).value))} onKeyUp={e => commit(Number((e.target as HTMLInputElement).value))} />
      </div>
    </div>;
}