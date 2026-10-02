import { createContext, useContext, useEffect, useLayoutEffect, useRef, useState } from 'react';
import type { ComponentChildren } from 'preact';
import type { KeyboardEvent as PKeyboardEvent, RefObject } from 'react';
import { createPortal } from 'react-dom';

/**
 * The app's one drawer: a bottom sheet on phones and tablets, a centred dialog from 1024px. Every
 * panel that slides over the page is one of these, so they all close the same ways - the handle
 * (tap or drag), a pull down on the content while it is scrolled to the top, the scrim, Escape -
 * and all of them animate out, whichever way they were closed. styles/drawer.css draws it.
 *
 * The exit needs the drawer to outlive its parent's "closed" state by a quarter second, which is
 * what Presence is for: wrap the conditional that mounts a drawer in it, and the last open render
 * is held while the drawer animates out. Without one a drawer still works; it just vanishes.
 */

const Exit = createContext<{ closing: boolean; done: () => void } | null>(null);

/** The exit's length; drawer.css's .dw-out transitions match it. */
const OUT_MS = 260;

export function Presence({
  children
}: {
  children?: ComponentChildren;
}) {
  const live = children != null && children !== false && children !== true && children !== '';
  const last = useRef<ComponentChildren>(null);
  const [, bump] = useState(0);
  if (live) last.current = children;
  const closing = !live && last.current != null;
  const done = useRef(() => {});
  done.current = () => {
    if (live || last.current == null) return;
    last.current = null;
    bump(n => n + 1);
  };
  // The drawer reports its exit's end; this is the backstop for a held render that has none.
  useEffect(() => {
    if (!closing) return undefined;
    const t = setTimeout(() => done.current(), OUT_MS * 3);
    return () => clearTimeout(t);
  }, [closing]);
  if (!live && last.current == null) return null;
  return <Exit.Provider value={{
    closing,
    done: () => done.current()
  }}>{live ? children : last.current}</Exit.Provider>;
}

/** Open drawers, oldest first: Escape closes the newest, and each stacks above the one it opened over. */
const stack: object[] = [];
let locks = 0;
/** body.has-drawer blurs the page behind; counted, so closing a drawer over another keeps the blur. */
const lock = (on: boolean) => {
  locks += on ? 1 : -1;
  document.body.classList.toggle('has-drawer', locks > 0);
};
const FOCUSABLE = 'button:not([disabled]),[href],input:not([disabled]),select:not([disabled]),textarea:not([disabled]),[tabindex]:not([tabindex="-1"])';
const docked = () => window.innerWidth >= 1024;
const reduced = () => matchMedia('(prefers-reduced-motion: reduce)').matches;

/**
 * Whether a touch at `t` belongs to something under it rather than to the drawer: anything scrolled
 * away from its top (the pull would be a scroll back up), a text field, or any element that claims
 * its own gestures with touch-action: none - sliders, the tuner's graph, the gate track.
 */
function ownsTouch(t: Element | null, panel: HTMLElement) {
  for (let n = t; n && n !== panel; n = n.parentElement) {
    if (n.scrollTop > 0 || n.matches('input,textarea,select,[contenteditable="true"]')) return true;
    if (getComputedStyle(n).touchAction === 'none') return true;
  }
  return panel.scrollTop > 0;
}

type Drag = { src: 'ptr' | 'touch'; x0: number; y0: number; y: number; t: number; v: number; on: boolean; h: number };

export function Drawer({
  label,
  onClose,
  className = '',
  role = 'dialog',
  scrimClose = true,
  initialFocus,
  returnFocus,
  children
}: {
  label: string;
  onClose: () => void;
  /** The panel's own classes, for what sits inside it. */
  className?: string;
  role?: 'dialog' | 'alertdialog';
  /** False where a stray tap outside would throw away work - the wake word tuner. */
  scrimClose?: boolean;
  initialFocus?: RefObject<HTMLElement>;
  /** Where focus goes on close, when it is not whatever had it at open: Safari does not focus a
   *  tapped button, so the trigger has to be named. */
  returnFocus?: RefObject<HTMLElement>;
  children?: ComponentChildren;
}) {
  const exit = useContext(Exit);
  const closing = !!exit?.closing;
  const root = useRef<HTMLDivElement>(null);
  const panel = useRef<HTMLElement>(null);
  const close = useRef(onClose);
  close.current = onClose;
  const doneRef = useRef(exit?.done);
  doneRef.current = exit?.done;
  const me = useRef({}).current;
  const [depth] = useState(() => stack.length);
  const drag = useRef<Drag | null>(null);

  // Open: blur the page, take focus, join the Escape stack. Closing (or unmounting) gives all three
  // back at the start of the exit, so the page is live again while the panel is still sliding away.
  useLayoutEffect(() => {
    if (closing) return undefined;
    const back = returnFocus?.current || document.activeElement as HTMLElement | null;
    lock(true);
    stack.push(me);
    (initialFocus?.current || panel.current)?.focus({ preventScroll: true });
    return () => {
      lock(false);
      const i = stack.indexOf(me);
      if (i >= 0) stack.splice(i, 1);
      const at = document.activeElement;
      if (back?.isConnected && (!at || at === document.body || panel.current?.contains(at))) back.focus({ preventScroll: true });
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [closing]);

  // The exit: let go of any drag position so the CSS transition runs from wherever the panel is.
  useLayoutEffect(() => {
    if (!closing) return undefined;
    drag.current = null;
    root.current?.classList.remove('dw-drag');
    root.current?.style.removeProperty('--dw-p');
    if (panel.current) panel.current.style.transform = '';
    const t = setTimeout(() => doneRef.current?.(), reduced() ? 0 : OUT_MS);
    return () => clearTimeout(t);
  }, [closing]);

  // Escape stops at a hint bubble or an open dropdown first: both stop it on its way to the window.
  useEffect(() => {
    if (closing) return undefined;
    const esc = (e: KeyboardEvent) => {
      if (e.key !== 'Escape' || stack[stack.length - 1] !== me) return;
      e.preventDefault();
      close.current();
    };
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [closing, me]);

  const begin = (src: Drag['src'], x: number, y: number) => {
    if (closing || docked() || !panel.current) return false;
    drag.current = { src, x0: x, y0: y, y, t: performance.now(), v: 0, on: false, h: panel.current.offsetHeight };
    return true;
  };
  const follow = (y: number) => {
    const d = drag.current;
    if (!d || !panel.current || !root.current) return;
    if (!d.on) {
      d.on = true;
      root.current.classList.add('dw-drag');
    }
    const now = performance.now();
    if (now > d.t) d.v = 0.7 * ((y - d.y) / (now - d.t)) + 0.3 * d.v;
    d.y = y;
    d.t = now;
    const dy = y - d.y0;
    // Upward is resisted rather than refused, so the panel answers the finger either way.
    const off = dy >= 0 ? dy : -Math.min(20, Math.sqrt(-dy) * 2);
    panel.current.style.transform = `translate3d(0,${off}px,0)`;
    root.current.style.setProperty('--dw-p', String(Math.max(0, 1 - Math.max(0, off) / d.h)));
  };
  /** Ends a drag: past a quarter of the panel, or flicked, it closes; otherwise it springs back. */
  const release = () => {
    const d = drag.current;
    drag.current = null;
    if (!d?.on) return false;
    root.current?.classList.remove('dw-drag');
    const dy = d.y - d.y0;
    const v = performance.now() - d.t > 90 ? 0 : d.v;
    if (dy > d.h * 0.25 || v > 0.5 && dy > 12) close.current();else {
      if (panel.current) panel.current.style.transform = '';
      root.current?.style.removeProperty('--dw-p');
    }
    return true;
  };

  // Pull-to-close from the content, touch only: a mouse wheel or trackpad has no "pull".
  useEffect(() => {
    const p = panel.current;
    if (!p || closing) return undefined;
    const start = (e: TouchEvent) => {
      const t = e.target as Element;
      if (e.touches.length !== 1 || t.closest('.dw-grab') || ownsTouch(t, p)) return;
      begin('touch', e.touches[0].clientX, e.touches[0].clientY);
    };
    const move = (e: TouchEvent) => {
      const d = drag.current;
      if (d?.src !== 'touch') return;
      const pt = e.touches[0];
      if (!d.on) {
        const dy = pt.clientY - d.y0;
        const dx = Math.abs(pt.clientX - d.x0);
        if (Math.abs(dy) < 8 && dx < 8) return;
        if (dy <= 0 || dx > dy || !e.cancelable) {
          drag.current = null;
          return;
        }
      }
      e.preventDefault();
      follow(pt.clientY);
    };
    const end = () => {
      if (drag.current?.src === 'touch') release();
    };
    p.addEventListener('touchstart', start, { passive: true });
    p.addEventListener('touchmove', move, { passive: false });
    p.addEventListener('touchend', end);
    p.addEventListener('touchcancel', end);
    return () => {
      p.removeEventListener('touchstart', start);
      p.removeEventListener('touchmove', move);
      p.removeEventListener('touchend', end);
      p.removeEventListener('touchcancel', end);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [closing]);

  const trap = (e: PKeyboardEvent<HTMLElement>) => {
    if (e.key !== 'Tab' || !panel.current) return;
    const f = Array.from(panel.current.querySelectorAll<HTMLElement>(FOCUSABLE)).filter(el => el.offsetParent !== null);
    if (!f.length) {
      e.preventDefault();
      return;
    }
    const at = document.activeElement;
    if (e.shiftKey && (at === f[0] || at === panel.current)) {
      e.preventDefault();
      f[f.length - 1].focus();
    } else if (!e.shiftKey && at === f[f.length - 1]) {
      e.preventDefault();
      f[0].focus();
    }
  };

  return createPortal(<div ref={root} className={closing ? 'dw dw-out' : 'dw'} style={{
    zIndex: 200 + depth * 10
  }}>
    <div className="dw-scrim" onClick={scrimClose && !closing ? () => close.current() : undefined} />
    <section ref={panel} className={className ? `dw-panel ${className}` : 'dw-panel'} role={role} aria-modal="true" aria-label={label} tabIndex={-1} onKeyDown={trap}>
      <div className="dw-grab" onPointerDown={e => {
        if (e.pointerType === 'mouse' && e.button !== 0 || !begin('ptr', e.clientX, e.clientY)) return;
        e.currentTarget.setPointerCapture(e.pointerId);
      }} onPointerMove={e => {
        const d = drag.current;
        if (d?.src === 'ptr' && (d.on || Math.abs(e.clientY - d.y0) > 4)) follow(e.clientY);
      }} onPointerUp={() => {
        // A tap on the strip, with no drag in it, closes - what the handle has always done.
        if (drag.current?.src === 'ptr' && !release()) close.current();
      }} onPointerCancel={() => {
        if (drag.current?.src === 'ptr') release();
      }}>
        {/* Pointer taps close through the strip above; this answers the keyboard (detail 0). */}
        <button type="button" className="dw-handle" aria-label="Close" onClick={e => {
          if (e.detail === 0) close.current();
        }} />
      </div>
      {children}
    </section>
  </div>, document.body);
}
