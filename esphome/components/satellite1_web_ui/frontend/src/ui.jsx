/** Shared primitives. Small on purpose: every one of these is paid for in the flash budget. */
import { useEffect, useLayoutEffect, useRef, useState } from "preact/hooks";

import { TEXT } from "./copy.js";

/* ------------------------------------------------------------------ */
/* Drawn glyphs                                                        */
/* ------------------------------------------------------------------ */

/* The route glyphs, drawn like media.jsx's `mi` set: 16-box, stroked in currentColor so wherever one
   sits, its surroundings' colour is the icon's colour too. One per route: a house, the waveform a
   wake word is, a speaker for what remains on Audio, a radar sweep, and a pulse line for Diagnostics
   - a wrench was considered and drew worse at 17px than the vitals it actually shows.
   They lived in shell.jsx while only the nav drawer used them; they moved here when each route's top
   card started wearing its glyph as a colour anchor (routes import ui.jsx, and shell.jsx imports the
   routes, so shell.jsx exporting them would have been a cycle). */
export const ni = (children) => (
  <svg
    class="ni"
    viewBox="0 0 16 16"
    fill="none"
    stroke="currentColor"
    stroke-width="1.5"
    stroke-linecap="round"
    stroke-linejoin="round"
    aria-hidden="true"
  >
    {children}
  </svg>
);
export const N_HOME = ni(
  <>
    <path d="M2.8 8.3 8 3.6l5.2 4.7" />
    <path d="M4.4 7.6V13h7.2V7.6" />
  </>,
);
export const N_WAKE = ni(
  <>
    <path d="M2.8 6.8v2.4" />
    <path d="M5.4 4.8v6.4" />
    <path d="M8 3v10" />
    <path d="M10.6 4.8v6.4" />
    <path d="M13.2 6.8v2.4" />
  </>,
);
export const N_AUDIO = ni(
  <>
    <path d="M2.8 6.4h2.4L8.6 3.8v8.4L5.2 9.6H2.8z" />
    <path d="M11.2 6a3.1 3.1 0 0 1 0 4" />
  </>,
);
export const N_PRES = ni(
  <>
    <path d="M13.2 8A5.2 5.2 0 1 1 8 2.8" />
    <path d="M8 8l3.7-3.7" />
    <circle cx="8" cy="8" r="1" fill="currentColor" stroke="none" />
  </>,
);
export const N_DIAG = ni(<path d="M2 8.5h2.8L6.4 5l3.2 6.5 1.6-3H14" />);
/* Not a route: the speech bubble on Home's Assistant card, whose route glyph (the house) belongs to
   the page rather than to any one card on it. */
export const N_CHAT = ni(<path d="M3 3.5h10v6.5H8.2L5.4 12.6V10H3z" />);

/** The FutureProofHomes mark, inlined from the vector master (Documentation repo) and stroked in
 *  currentColor so one copy serves both themes. ~700 bytes gzipped, against two PNGs that would
 *  not gzip at all. It lived in login.jsx while the login screen was its only wearer; it moved here
 *  when the splash overlay (splash.jsx) started opening every session with the same mark - one copy,
 *  and the two screens cannot drift apart. `cls` sizes it per surface; the login size is the default. */
export const Logo = ({ cls = "login-logo" }) => (
  <svg class={cls} viewBox="0 0 79.375 79.375" fill="none" stroke="currentColor" stroke-linecap="square" aria-hidden="true">
    <g transform="matrix(1.6754,0,0,1.6754,84.9754,-16.4554)" stroke-width="1.31">
      <path d="m -45.44,31.52 11,-10.96 11,10.96 v 16.25 l -5.82,.01" />
      <path d="m -27.49,20.19 10.94,9.29 v 18.3 l 8.35,.03 V 29.29 l -10.94,-9.66 -1.92,1.73" />
      <path stroke-width="1.36" d="m -32.25,47.75 c 0,-7.27 -5.89,-13.42 -13.16,-13.42 h 0 c -.05,0 -.1,0 -.15,.01" />
      <path stroke-width="1.36" d="m -36.31,47.73 c .01,-.12 .01,-.12 .01,-.24 0,-5.03 -4.08,-9.11 -9.11,-9.11 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path stroke-width="1.36" d="m -40.36,47.8 c .01,-.12 .01,-.18 .01,-.31 0,-2.8 -2.27,-5.06 -5.06,-5.06 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path stroke-width="1.36" d="m -45.41,46.48 a 1.01,1.01 0 0 0 -.15,.01 v 1.38 h 1.09 a 1.01,1.01 0 0 0 .07,-.37 1.01,1.01 0 0 0 -1.01,-1.01 z" />
    </g>
  </svg>
);

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
export function Card({ title, icon, hint, right, children, collapsible, name, defaultOpen = false, ...rest }) {
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
          {/* The colour anchor: a drawn glyph in a small accent-tinted circle, carried by each
              route's top card - one drop of colour per page without painting any surface. */}
          {icon && <span class="cico">{icon}</span>}
          {/* Title first, caret after. The caret used to lead, which indented a collapsible title by
              its width and made these headers visibly different from every other card's - the owner
              asked for them to match. The whole header is still one button, so the hit target did not
              shrink to the caret. */}
          {collapsible ? (
            <button class="card-t" aria-expanded={open} onClick={toggle}>
              <span>{title}</span>
              <Chevron down={open} cls="caret-s" />
            </button>
          ) : (
            <span>{title}</span>
          )}
          {hint && <Hint text={hint} />}
          {/* Kept mounted while collapsed: on the Logs card this is the level menu and Export, which
              would otherwise pop in and out of the header as the card opens and shuts. */}
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
 * Holds a just-committed slider value over the stale reads that follow it.
 *
 * Every slider commits on release and then keeps rendering from the entity or poll behind it, which
 * does not know about the write for up to a poll interval - so the thumb snapped back to the old
 * value and jumped forward again when the echo landed (reported from Safari on the phone, September
 * 2026, but present everywhere). The committed value wins until the incoming one reaches it (within
 * `tol`, which absorbs rounding on values that round-trip through 0-255 or seconds) or until five
 * seconds pass - the escape for a write the device refused, where the stale value is the truth.
 */
export function useHeld(value, tol) {
  const h = useRef(null);
  if (h.current !== null && (Math.abs(value - h.current.v) <= tol || Date.now() - h.current.at > 5000)) {
    h.current = null;
  }
  return [
    h.current !== null ? h.current.v : value,
    (v) => {
      h.current = { v, at: Date.now() };
    },
  ];
}

/** The track's filled portion as a custom property; the CSS gradient can't know the value itself. */
export const rangeFill = (shown, min, max) => `--p:${(((shown - min) / (max - min || 1)) * 100).toFixed(1)}%`;

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
  const [base, hold] = useHeld(value, step || 1);
  const shown = local ?? base;

  return (
    <div class="slider">
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={shown}
        disabled={disabled}
        style={rangeFill(shown, min, max)}
        onInput={(e) => {
          const v = Number(e.currentTarget.value);
          setLocal(v);
          if (onPreview) onPreview(v);
        }}
        onChange={(e) => {
          const v = Number(e.currentTarget.value);
          hold(v);
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
 * A button that asks first, as a modal.
 *
 * Used for everything that interrupts or erases - restarts, reflashes, the factory reset, installing
 * an update. It began as an inline two-press arm (press once, the button becomes its consequence for
 * five seconds), which made the second press deliberate but never said what the action would actually
 * do - and these are exactly the buttons whose consequences a person cannot be expected to know. The
 * modal carries a sentence or two of what happens next (copy in CONFIRM in copy.js), and the
 * proceed button still names the outcome rather than saying "OK".
 *
 * The scrim tap and Escape both cancel, because backing out must always be the easy gesture.
 */
export function Confirm({ label, title, body, confirmLabel, danger, solid, disabled, onConfirm }) {
  const [open, setOpen] = useState(false);

  useEffect(() => {
    if (!open) return;
    const esc = (e) => e.key === "Escape" && setOpen(false);
    document.addEventListener("keydown", esc);
    return () => document.removeEventListener("keydown", esc);
  }, [open]);

  return (
    <>
      <button
        class={`btn${danger ? " danger" : ""}${solid ? " solid" : ""}`}
        disabled={disabled}
        onClick={() => setOpen(true)}
      >
        {label}
      </button>
      {open && (
        <div class="scrim center" onClick={() => setOpen(false)}>
          <div class="modal" role="alertdialog" aria-modal="true" aria-label={title} onClick={(e) => e.stopPropagation()}>
            <h3 class="modal-t">{title || TEXT.confirm_title}</h3>
            <p class="modal-b">{body}</p>
            <div class="modal-btns">
              <button class="btn ghost" onClick={() => setOpen(false)}>
                {TEXT.cancel}
              </button>
              <button
                class={`btn solid${danger ? " danger" : ""}`}
                onClick={() => {
                  setOpen(false);
                  onConfirm();
                }}
              >
                {confirmLabel || TEXT.confirm}
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}

/* The modal Sheet component that lived here (the wake word picker's tall centered surface) is
   retired: the v2 redesign expands the picker inline under the word's pill, and no route opens a
   modal for it any more (owner call, September 2026). The .sheet CSS survives - the device
   switcher wears it as a top-hung drawer. */

export function Btn({ children, onClick, disabled, danger, solid, cls }) {
  return (
    <button
      class={`btn${danger ? " danger" : ""}${solid ? " solid" : ""}${cls ? ` ${cls}` : ""}`}
      disabled={disabled}
      onClick={onClick}
    >
      {children}
    </button>
  );
}

/** Shown in place of a card's body when the entity behind it is not present on this build. */
export function Missing({ what }) {
  return <p class="dim sm">{what} is not available on this firmware build.</p>;
}

/**
 * Drawer citizenship, shared by everything that slides over the page: the nav pane, the device
 * switcher, the expanded media view and the players panel. Two duties. Escape closes it - the same
 * listener each drawer used to carry alone. And only one drawer stands at a time, app-wide (the
 * owner's rule, September 2026): opening announces itself on a window event, every other open
 * drawer hears a name that is not its own and closes under the new one. An event rather than
 * shared state because the drawers live in different components with no common owner - the
 * switcher is shell.jsx's, the players panel is media.jsx's - and threading a context through
 * both for four booleans would cost more than a whisper on the window.
 *
 * `open` gates everything: a shut drawer holds no listeners and cannot be closed twice. The
 * announcement fires only on the open transition, so re-renders of an open drawer never re-close
 * a sibling that opened after it.
 */
/* How many drawers are open right now - normally 0 or 1, but the exclusion handoff holds two for a
   frame (the new one mounts, then the old one hears the event and closes), so the page lock below
   counts rather than toggles or the handoff would unlock a page with a drawer still standing. */
let drawersOpen = 0;

export function useDrawer(id, open, onClose) {
  useEffect(() => {
    if (!open) return undefined;
    // Freeze the page while any drawer stands. Found on iOS Safari: a swipe that missed the
    // players panel's handle scrolled and rubber-banded the whole document behind the scrim
    // (owner's screenshot, September 2026). The class lands on <html> so both it and <body>
    // can pin their overflow in CSS.
    if (++drawersOpen === 1) document.documentElement.classList.add("held");
    window.dispatchEvent(new CustomEvent("drawer", { detail: id }));
    const other = (e) => e.detail !== id && onClose();
    const esc = (e) => e.key === "Escape" && onClose();
    window.addEventListener("drawer", other);
    document.addEventListener("keydown", esc);
    return () => {
      if (--drawersOpen === 0) document.documentElement.classList.remove("held");
      window.removeEventListener("drawer", other);
      document.removeEventListener("keydown", esc);
    };
    // onClose is a setState arrow, new each render; re-running on it would re-announce the open.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [id, open]);
}

/**
 * Swipe-to-dismiss for the sliding drawers, following the finger. `dir` is which way dismissal
 * moves: 1 for the bottom drawers (down), -1 for the top one (up).
 *
 * The handlers go on the drawer itself but a drag only starts inside a `data-grab` zone - the
 * handle and the header row - because the drawer bodies scroll and carry sliders, and a drag that
 * started anywhere would fight both. Grab zones set touch-action: none in CSS, so the browser
 * never claims the gesture for scrolling. While held, the drawer rides the finger through an
 * inline transform (dismissal direction only - a drawer cannot be pushed further open); release
 * past 80px, or a quick flick past 24px, closes it, and anything less springs back through the
 * drawer's own transform transition.
 */
export function useSheetDrag(onClose, dir = 1) {
  const [dy, setDy] = useState(0);
  const st = useRef(null);
  return [
    // A leading semicolon so it can be appended blindly to whatever inline style the drawer
    // already carries (the media surfaces carry the artwork tint).
    dy ? `;transform:translateY(${dy * dir}px);transition:none` : "",
    {
      onPointerDown: (e) => {
        if (!e.target.closest("[data-grab]")) return;
        st.current = { y: e.clientY, t: Date.now(), id: e.pointerId, held: false };
      },
      onPointerMove: (e) => {
        const s = st.current;
        if (!s) return;
        // The pointer is not captured until the finger has clearly moved, because capture retargets
        // the eventual click to the drawer root - which would eat every plain tap on the ✕, the
        // title and the handle. Inside the slop it is a tap and stays the browser's; past it the
        // gesture is ours and the capture keeps it through descendants and out of them.
        if (!s.held) {
          if (Math.abs(e.clientY - s.y) < 7) return;
          s.held = true;
          // Guarded: capture throws for a pointer that stopped existing between events (a finger
          // that lifted mid-gesture), and losing the capture is survivable where a throw is not.
          try {
            e.currentTarget.setPointerCapture(s.id);
          } catch {
            /* keep following the bubbled events instead */
          }
        }
        setDy(Math.max(0, (e.clientY - s.y) * dir));
      },
      onPointerUp: () => {
        const s = st.current;
        st.current = null;
        if (!s || !s.held) return;
        const flick = dy > 24 && Date.now() - s.t < 250;
        setDy(0);
        if (dy > 80 || flick) onClose();
      },
      onPointerCancel: () => {
        st.current = null;
        setDy(0);
      },
    },
  ];
}

/**
 * An empty state: a small drawn glyph centred over one dim line. The glyph is passed in, drawn where
 * the state lives, because each empty thing has its own shape - a speech bubble for the transcript,
 * a dashed zone for the plot. Layout only; the words stay in copy.js like every other string.
 */
export function Empty({ icon, text }) {
  return (
    <div class="empty">
      {icon}
      <p class="dim sm">{text}</p>
    </div>
  );
}
