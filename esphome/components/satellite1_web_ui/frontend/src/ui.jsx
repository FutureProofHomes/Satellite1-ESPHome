/** Shared primitives. Small on purpose: every one of these is paid for in the flash budget. */
import { useEffect, useLayoutEffect, useRef, useState } from "preact/hooks";

import { TEXT } from "./copy.js";

/* ------------------------------------------------------------------ */
/* Hint                                                                */
/* ------------------------------------------------------------------ */

let openHintSetter = null;

/**
 * The "i" beside a label, and the bubble it opens.
 *
 * Positioned with fixed coordinates measured from the trigger rather than with absolute positioning
 * inside the card, because a card can scroll and can sit close enough to either edge that a bubble
 * anchored to it would be clipped. Clamped into the viewport on both axes after layout.
 *
 * Only one is open at a time: on a phone two open bubbles cover the content they explain.
 */
export function Hint({ text }) {
  const [open, setOpen] = useState(false);
  const btn = useRef(null);
  const bubble = useRef(null);
  const [pos, setPos] = useState(null);

  useEffect(() => {
    if (!open) return;
    if (openHintSetter && openHintSetter !== setOpen) openHintSetter(false);
    openHintSetter = setOpen;

    const dismiss = (e) => {
      if (!btn.current?.contains(e.target) && !bubble.current?.contains(e.target)) setOpen(false);
    };
    const esc = (e) => e.key === "Escape" && setOpen(false);
    // Capture, so a tap that also lands on a toggle closes the bubble instead of flipping a switch.
    document.addEventListener("pointerdown", dismiss, true);
    document.addEventListener("keydown", esc);
    return () => {
      document.removeEventListener("pointerdown", dismiss, true);
      document.removeEventListener("keydown", esc);
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
    // Above the trigger when there is not room below, which on a phone is most of the page.
    const below = t.bottom + 6;
    const top = below + b.height + m > window.innerHeight ? Math.max(m, t.top - b.height - 6) : below;
    setPos({ left, top });
  }, [open]);

  return (
    <>
      <button
        ref={btn}
        class="hint-btn"
        aria-label="Explain"
        aria-expanded={open}
        onClick={() => setOpen((v) => !v)}
      >
        i
      </button>
      {open && (
        <div ref={bubble} class="hint" role="tooltip" style={pos ? { left: `${pos.left}px`, top: `${pos.top}px` } : { opacity: 0 }}>
          {text}
        </div>
      )}
    </>
  );
}

/* ------------------------------------------------------------------ */
/* Layout                                                              */
/* ------------------------------------------------------------------ */

/**
 * Remembered per section, so a collapse survives a reload and a firmware update.
 *
 * Wrapped because localStorage throws rather than no-ops in a few real situations - Safari's private
 * mode historically, and any browser with site data blocked - and a diagnostics page that white-screens
 * because it could not remember a disclosure triangle would be a poor trade.
 */
function readOpen(key, fallback) {
  try {
    const v = localStorage.getItem(`sat1.open.${key}`);
    return v === null ? fallback : v === "1";
  } catch {
    return fallback;
  }
}

function writeOpen(key, open) {
  try {
    localStorage.setItem(`sat1.open.${key}`, open ? "1" : "0");
  } catch {
    /* Not remembering is survivable; failing to render is not. */
  }
}

/**
 * `collapsible` opts a card in, and needs a stable `name` to remember itself by - deliberately not the
 * title, which is copy and will be reworded.
 *
 * The header becomes a button only when collapsible. Making every card's header a button would put a
 * dozen useless tab stops in front of a keyboard user before they reach a control.
 */
export function Card({ title, hint, right, children, collapsible, name, defaultOpen = false, ...rest }) {
  const [open, setOpen] = useState(() => (collapsible ? readOpen(name, defaultOpen) : true));

  const toggle = () => {
    const next = !open;
    setOpen(next);
    writeOpen(name, next);
  };

  return (
    <section class={`card${collapsible ? " card-c" : ""}`} {...rest}>
      {title && (
        <h2>
          {collapsible ? (
            <button class="card-t" aria-expanded={open} onClick={toggle}>
              <span class="caret-s">{open ? "\u25BE" : "\u25B8"}</span>
              <span>{title}</span>
            </button>
          ) : (
            <span>{title}</span>
          )}
          {hint && <Hint text={hint} />}
          {/* Kept mounted while collapsed: on the Log card this is the level filter and Pause, and
              Pause is a flag on the shared stream that must stay reachable. */}
          {right && <span class="card-right">{right}</span>}
        </h2>
      )}
      {open && children}
    </section>
  );
}

export function Row({ label, hint, children, sub }) {
  return (
    <div class="ctl">
      <div class="ctl-label">
        <span>{label}</span>
        {hint && <Hint text={hint} />}
        {sub && <span class="ctl-sub">{sub}</span>}
      </div>
      <div class="ctl-body">{children}</div>
    </div>
  );
}

/** A labelled read-only fact. The unit is separated so it can be dimmed. */
export function Fact({ label, value, unit, hint, tone }) {
  return (
    <div class="fact">
      <div class="fact-label">
        <span>{label}</span>
        {hint && <Hint text={hint} />}
      </div>
      <div class={`fact-value${tone ? ` t-${tone}` : ""}`}>
        {value}
        {unit && <span class="fact-unit">{unit}</span>}
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Inputs                                                              */
/* ------------------------------------------------------------------ */

export function Toggle({ checked, disabled, onChange }) {
  return (
    <button
      class={`sw${checked ? " on" : ""}`}
      role="switch"
      aria-checked={!!checked}
      disabled={disabled}
      onClick={() => onChange(!checked)}
    >
      <span class="sw-knob" />
    </button>
  );
}

/**
 * A slider that reports continuously while dragging but only writes on release.
 *
 * A number entity write is a round trip to the device, and dragging fires an input event per pixel.
 * Sending each one is what turns a gesture into forty queued requests against seven sockets.
 */
export function Slider({ value, min, max, step, disabled, format, onCommit }) {
  const [local, setLocal] = useState(null);
  const shown = local ?? value;

  return (
    <div class="slider">
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={shown}
        disabled={disabled}
        onInput={(e) => setLocal(Number(e.currentTarget.value))}
        onChange={(e) => {
          const v = Number(e.currentTarget.value);
          setLocal(null);
          onCommit(v);
        }}
      />
      <span class="slider-val mono">{format ? format(shown) : shown}</span>
    </div>
  );
}

export function Select({ value, options, disabled, onChange }) {
  return (
    <select class="sel" disabled={disabled} value={value} onChange={(e) => onChange(e.currentTarget.value)}>
      {(options || []).map((o) => (
        <option key={o} value={o}>
          {o}
        </option>
      ))}
    </select>
  );
}

/**
 * A button that will not fire on the first press.
 *
 * Used for everything unrecoverable - factory reset, erasing the audio chip's firmware. The second
 * press has to be a deliberate second decision, so the confirm state times out rather than sitting
 * armed indefinitely, and the label says what will happen rather than "Are you sure?".
 */
export function Confirm({ label, confirmLabel, danger, disabled, onConfirm }) {
  const [armed, setArmed] = useState(false);

  useEffect(() => {
    if (!armed) return;
    const t = setTimeout(() => setArmed(false), 5000);
    return () => clearTimeout(t);
  }, [armed]);

  if (!armed) {
    return (
      <button class={`btn${danger ? " danger" : ""}`} disabled={disabled} onClick={() => setArmed(true)}>
        {label}
      </button>
    );
  }
  return (
    <span class="confirm">
      <button
        class={`btn${danger ? " danger" : ""} solid`}
        onClick={() => {
          setArmed(false);
          onConfirm();
        }}
      >
        {confirmLabel || TEXT.confirm}
      </button>
      <button class="btn ghost" onClick={() => setArmed(false)}>
        {TEXT.cancel}
      </button>
    </span>
  );
}

export function Btn({ children, onClick, disabled, danger, solid }) {
  return (
    <button class={`btn${danger ? " danger" : ""}${solid ? " solid" : ""}`} disabled={disabled} onClick={onClick}>
      {children}
    </button>
  );
}

/** Shown in place of a card's body when the entity behind it is not present on this build. */
export function Missing({ what }) {
  return <p class="dim sm">{what} is not available on this firmware build.</p>;
}
