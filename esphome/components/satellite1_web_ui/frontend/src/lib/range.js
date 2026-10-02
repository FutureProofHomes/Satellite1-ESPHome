/**
 * Range inputs on touch screens. iOS Safari only moves a native slider when the touch lands on its
 * thumb, so a tap on the track does nothing and a drag that starts a few pixels off the thumb
 * scrolls the page instead. Every slider gets the Android behaviour instead: a tap jumps there, and
 * a sideways drag from anywhere on it follows the finger.
 */

/**
 * The value under `x` on a range input `width` wide starting at `left`, snapped to `step`. The
 * thumb's centre travels between half its width from either end, so that is the usable span.
 */
export function valueAt(x, left, width, min, max, step, thumb) {
  const span = Math.max(1, width - thumb);
  const r = Math.min(1, Math.max(0, (x - left - thumb / 2) / span));
  const n = Math.min(Math.round((r * (max - min)) / step), Math.floor((max - min) / step + 1e-9));
  const dp = (String(step).split(".")[1] || "").length;
  return Number((min + n * step).toFixed(dp));
}

/**
 * Installs the touch handling once, on the document. A drag decides its direction after a few
 * pixels: sideways takes the slider, anything else is left to scroll the page. The value is written
 * the way a native drag writes it - an input event per move and one change on release - so every
 * slider's own handlers (MSlider's commit-on-release among them) see nothing new.
 */
export function installRangeTouch(doc = document) {
  let on = null;
  const set = (t, x) => {
    const el = on.el;
    const r = el.getBoundingClientRect();
    const thumb = parseFloat(getComputedStyle(el).getPropertyValue("--thumb")) || 28;
    const v = valueAt(x, r.left, r.width, Number(el.min || 0), Number(el.max || 100), Number(el.step) || 1, thumb);
    if (String(v) === el.value) return;
    el.value = String(v);
    on.moved = true;
    el.dispatchEvent(new Event("input", { bubbles: true }));
  };
  doc.addEventListener(
    "touchstart",
    (e) => {
      const el = e.target instanceof Element && e.target.closest('input[type="range"]');
      on = el && !el.disabled && e.touches.length === 1 ? { el, x: e.touches[0].clientX, y: e.touches[0].clientY, mode: null, moved: false } : null;
    },
    { passive: true },
  );
  doc.addEventListener(
    "touchmove",
    (e) => {
      if (!on || on.mode === "page") return;
      const t = e.touches[0];
      if (!on.mode) {
        const dx = Math.abs(t.clientX - on.x);
        const dy = Math.abs(t.clientY - on.y);
        if (dx < 4 && dy < 4) return;
        on.mode = dx >= dy && e.cancelable ? "slide" : "page";
        if (on.mode === "page") return;
      }
      e.preventDefault();
      set(t, t.clientX);
    },
    { passive: false },
  );
  const end = (e) => {
    if (!on) return;
    const was = on;
    if (e.type === "touchend" && !was.mode) set(null, was.x);
    if (e.type === "touchend" && was.mode !== "page" && was.moved) was.el.dispatchEvent(new Event("change", { bubbles: true }));
    on = null;
  };
  doc.addEventListener("touchend", end);
  doc.addEventListener("touchcancel", end);
}
