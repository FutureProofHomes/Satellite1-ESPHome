import type { MouseEvent as RMouseEvent } from 'react';

/** The app's one on/off switch; styles/controls.css draws it in the orb's colour when on. */
export function Switch({
  on,
  onChange,
  label,
  disabled
}: {
  on: boolean;
  onChange: (v: boolean) => void;
  label: string;
  disabled?: boolean;
}) {
  return <button type="button" role="switch" aria-checked={on} aria-label={label} disabled={disabled} className="sw" onClick={() => onChange(!on)}>
      <span className="sw-knob" />
    </button>;
}

export type CheckState = 'on' | 'off' | 'mixed';

/**
 * The tri-state box, a button rather than an input: an indeterminate checkbox needs a ref to set
 * the property, and there is no attribute for it. Its marks are drawn - an SVG tick, a CSS dash -
 * rather than typed, because the device serves no webfont and U+25EA, the half-filled square a typed
 * third state would use, is missing from most system fonts and arrives as an empty rectangle. A
 * tri-state control whose third state renders as a blank box is worse than none.
 */
export function Check({
  state,
  onClick,
  label,
  disabled
}: {
  state: CheckState;
  onClick: (e: RMouseEvent<HTMLButtonElement>) => void;
  label: string;
  disabled?: boolean;
}) {
  return <button type="button" role="checkbox" aria-checked={state === 'mixed' ? 'mixed' : state === 'on'} aria-label={label} disabled={disabled} className="ck" onClick={onClick}>
      <span className="ck-box">
        {state === 'on' && <svg width="12" height="12" viewBox="0 0 10 10" aria-hidden="true"><path d="M1.5 5l2.5 2.5 4.5-4.5" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" fill="none" /></svg>}
        {state === 'mixed' && <i className="ck-dash" />}
      </span>
    </button>;
}
