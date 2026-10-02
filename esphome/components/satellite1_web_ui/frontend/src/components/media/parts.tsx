import type { CSSProperties, ReactNode } from 'react';
import { useState } from 'react';
import { pct } from '../../lib/media.js';
import { useHeld } from './model';
import type { Model } from './model';

export const Svg = ({
  children,
  size = 16
}: {
  children: ReactNode;
  size?: number;
}) => <svg width={size} height={size} viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{children}</svg>;
export const mi = (p: ReactNode) => <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{p}</svg>;

/* Glyphs as geometry rather than codepoints: nothing here ships a webfont, and the transport,
   shuffle and repeat codepoints render as coloured, inconsistent emoji on iOS. */
export const I_PREV = <path d="M3.5 4v8M5 8l7-4v8L5 8z" />;
export const I_NEXT = <path d="M12.5 4v8M11 8L4 4v8l7-4z" />;
export const I_SHUFFLE = <g><path d="M1.5 4.5h2.6c3.6 0 4.2 7 7.8 7h2.1M1.5 11.5h2.6c1.3 0 2.2-.9 2.9-2M14 4.5h-2.1c-1.4 0-2.3 1-3 2.1" /><path d="M12.2 2.7l1.8 1.8-1.8 1.8M12.2 9.7l1.8 1.8-1.8 1.8" /></g>;
export const I_REPEAT = <g><path d="M2.5 6.5v-.4A2.6 2.6 0 0 1 5.1 3.5h7.4M13.5 9.5v.4a2.6 2.6 0 0 1-2.6 2.6H3.5" /><path d="M10.8 1.8 12.6 3.5l-1.8 1.7M5.2 14.2 3.4 12.5l1.8-1.7" /></g>;
/** Marks a slider as a volume (owner's report, September 2026: unlabelled, the bar's second row
 *  read as a scrubber). */
export const I_VOL = <g><path d="M9 4.5 5.5 7H3a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h2.5L9 13.5V4.5z" /><path d="M11.5 7.5a2 2 0 0 1 0 3" /></g>;
export const I_SPK_BOX = <g><rect x="3.5" y="1.75" width="9" height="12.5" rx="2" /><circle cx="8" cy="4.6" r=".6" fill="currentColor" stroke="none" /><circle cx="8" cy="9.6" r="2.4" /></g>;
export const I_PLUS = <path d="M8 4v8M4 8h8" />;
/** A member row's remove: a minus, the inverse of the add rows' +, rather than a ✕ (owner's
 *  request, September 2026), which read as "close" rather than "remove". */
export const I_MINUS = <path d="M4 8h8" />;
export const I_PAUSE = <path d="M5.5 4v8M10.5 4v8" />;
export const I_PLAY = <path d="M5 4l8 4-8 4V4z" />;
export const I_NOTE = <g><path d="M6 13V4l7-1.5V11" /><circle cx="4.5" cy="13" r="1.5" /><circle cx="11.5" cy="11" r="1.5" /></g>;
export const I_SEARCH = <path d="m11 11 3 3M6.8 11a4.2 4.2 0 1 1 0-8.4 4.2 4.2 0 0 1 0 8.4Z" />;

/**
 * A slider that previews while dragging (so a moving playhead does not fight the thumb) and
 * commits once on release, holding the committed value over the stale polls that follow (a write
 * per input event would queue dozens of requests on a device with seven sockets). Returns what to
 * show and the props for the <input>.
 */
export function useRange(value: number, tol: number, onCommit: (v: number) => void) {
  const [drag, setDrag] = useState<number | null>(null);
  const [base, hold] = useHeld(value, tol);
  const shown = drag ?? base;
  return [shown, {
    value: shown,
    onInput: (e: { currentTarget: HTMLInputElement }) => setDrag(Number(e.currentTarget.value)),
    // preact/compat renames onChange to onInput on range inputs; the capture form reaches the
    // native change event, which fires once, on release.
    onChangeCapture: (e: { currentTarget: HTMLInputElement }) => {
      const v = Number(e.currentTarget.value);
      hold(v);
      setDrag(null);
      onCommit(v);
    }
  }] as const;
}

/** A 0-100 volume slider in the design's dress. */
export function Vol({
  value,
  label,
  onCommit
}: {
  value: number;
  label: string;
  onCommit: (v: number) => void;
}) {
  const [shown, props] = useRange(value, 1, onCommit);
  return <input type="range" min={0} max={100} step={1} aria-label={label} style={{
    '--pct': pct(shown, 100)
  } as CSSProperties} {...props} />;
}

/** The filled play/pause circle, ringed while its command awaits the poll's echo. */
export function PlayButton({
  model,
  sm
}: {
  model: Model;
  sm?: boolean;
}) {
  const busy = !!model.pending.play;
  return <button className={'mplay' + (sm ? ' sm' : '') + (busy ? ' busy' : '')} aria-label={model.playing ? 'Pause' : 'Play'} disabled={busy} onClick={e => {
    e.stopPropagation();
    model.playPause();
  }}><Svg size={sm ? 16 : 22}>{model.playing ? I_PAUSE : I_PLAY}</Svg></button>;
}
