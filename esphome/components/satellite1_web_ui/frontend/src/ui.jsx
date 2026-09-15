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
 * The disclosure chevron, drawn rather than typed.
 *
 * This was U+25BE and U+25B8 in three places. Nothing here ships a webfont - the device has no network
 * at runtime and no CDN is allowed - so those glyphs came from whatever the browser happened to have
 * installed, and on the phone they arrived as a mark a few pixels wide that read as a speck of dirt
 * rather than a control. Geometry renders identically everywhere, which a codepoint does not.
 *
 * Points right by default and rotates down when `down` is set, so a collapsed row and an expanded one
 * are told apart by the same shape turning rather than by two different characters.
 */
export function Chevron({ down, up, cls }) {
  return (
    <svg
      class={`chev${down ? " down" : ""}${up ? " up" : ""}${cls ? ` ${cls}` : ""}`}
      viewBox="0 0 12 12"
      fill="none"
      stroke="currentColor"
      stroke-width="1.9"
      stroke-linecap="round"
      stroke-linejoin="round"
      aria-hidden="true"
    >
      <path d="M4.2 2.4 8.3 6l-4.1 3.6" />
    </svg>
  );
}

/**
 * An arrow, which is a chevron with a shaft, and the distinction is the whole point of it existing.
 *
 * A chevron on a control means "this opens, here" - it points at the disclosure and flips when open. An
 * arrow means "this takes you somewhere else". The sensor chips need both meanings in one row: three of
 * them open an editor underneath and one of them is a link to another route, and drawing them with the
 * same glyph is what made the row read as four identical things that behave differently.
 *
 * Shares .chev for its box so the two line up on the same 12px grid.
 */
export function Arrow({ cls }) {
  return (
    <svg
      class={`chev${cls ? ` ${cls}` : ""}`}
      viewBox="0 0 12 12"
      fill="none"
      stroke="currentColor"
      stroke-width="1.9"
      stroke-linecap="round"
      stroke-linejoin="round"
      aria-hidden="true"
    >
      <path d="M1.9 6h8" />
      <path d="M6.6 2.9 9.9 6l-3.3 3.1" />
    </svg>
  );
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
              <Chevron down={open} cls="caret-s" />
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
export function Fact({ label, value, unit, hint, tone, sub }) {
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
      {/* Detail that belongs to this reading rather than beside it. The address and MAC used to sit in a
          strip along the bottom of the Device card, which put them equally close to every fact above and so
          to none of them; under "Network Type" they are obviously the address on that network. */}
      {sub && <div class="fact-sub">{sub}</div>}
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
 *
 * `onPreview` is the escape hatch for controls whose effect is drawn elsewhere on the page - the
 * radar's detection-range ring follows the drag through it. It must be local-state-only in the
 * caller; the device still hears nothing until release.
 */
export function Slider({ value, min, max, step, disabled, format, onCommit, onPreview }) {
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
        onInput={(e) => {
          const v = Number(e.currentTarget.value);
          setLocal(v);
          if (onPreview) onPreview(v);
        }}
        onChange={(e) => {
          const v = Number(e.currentTarget.value);
          setLocal(null);
          onCommit(v);
        }}
      />
      <span class="slider-val num">{format ? format(shown) : shown}</span>
    </div>
  );
}

/**
 * Options are plain strings, or [value, label] pairs where the two differ.
 *
 * Every entity-backed select passes strings, because an ESPHome select's options are the labels. The
 * pair form exists for the assistant list, where Home Assistant stores `preferred` and displays
 * "Preferred": mapping that by label instead would break for anyone who named a pipeline "Preferred",
 * and silently send their wake word to the wrong one. Pairs are already how the Home Assistant payload
 * carries an entity id beside a name, so this is the house shape rather than a new one.
 */
export function Select({ value, options, disabled, onChange }) {
  return (
    <select class="sel" disabled={disabled} value={value} onChange={(e) => onChange(e.currentTarget.value)}>
      {(options || []).map((o) => {
        const [v, label] = Array.isArray(o) ? o : [o, o];
        return (
          <option key={v} value={v}>
            {label}
          </option>
        );
      })}
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
