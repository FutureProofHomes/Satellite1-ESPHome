/**
 * The on-screen keyboard. A phone never resizes the page for it: it shrinks the visual viewport and
 * pans the page up under it, which carries the fixed nav and media bar up between the field and the
 * keys. While it is up, <html> carries data-kb with the visible strip's height and how far the page
 * is panned (--vvh, --vvt), and styles/shell.css and styles/home.css lay the page out in that strip.
 */

/** How much shorter than the page the visible strip must be to be a keyboard: more than an iPad's
 *  shortcut bar over a hardware keyboard, less than the smallest phone keyboard. */
export const KB_MIN = 150;

/** How long a field focused on a touch screen holds the keyboard layout before the keyboard shows,
 *  so the bars go as the keys come up rather than riding up with them first. */
export const KB_WAIT_MS = 800;

const TEXT_TYPES = new Set(["text", "search", "url", "email", "password", "tel", "number"]);

/** Whether focusing `el` brings up the keyboard: a field that takes typing. */
export function typesText(el) {
  if (!el || el.disabled || el.readOnly) return false;
  if (el.isContentEditable) return true;
  const tag = String(el.tagName || "").toUpperCase();
  if (tag === "TEXTAREA") return true;
  return tag === "INPUT" && TEXT_TYPES.has(String(el.type || "text").toLowerCase());
}

/** Whether the visible strip is short of the page by a keyboard. `scale` is the pinch zoom, which
 *  shrinks the strip in CSS pixels with no keyboard at all. */
export const keyboardUp = (layoutH, visualH, scale = 1) => layoutH - visualH * scale >= KB_MIN;

/**
 * Keeps data-kb, --vvh and --vvt on <html> true for fields inside `root`; drawers are portaled to
 * <body>, so their fields leave the page as it is. Returns the cleanup.
 */
export function watchKeyboard(root) {
  const vv = window.visualViewport;
  if (!root || !vv) return () => {};
  const html = document.documentElement;
  const touch = matchMedia("(pointer: coarse)");
  let until = 0;
  let wait = 0;
  let frame = 0;
  const typing = () => {
    const el = document.activeElement;
    return typesText(el) && root.contains(el);
  };
  const clear = () => {
    delete html.dataset.kb;
    html.style.removeProperty("--vvh");
    html.style.removeProperty("--vvt");
  };
  const update = () => {
    frame = 0;
    const up = keyboardUp(html.clientHeight, vv.height, vv.scale);
    if (typing() && (up || performance.now() < until)) {
      html.style.setProperty("--vvh", `${Math.round(vv.height)}px`);
      html.style.setProperty("--vvt", `${Math.round(vv.offsetTop)}px`);
      html.dataset.kb = "1";
    } else if (html.dataset.kb) clear();
  };
  const onFocus = () => {
    if (!typing() || !touch.matches) return;
    until = performance.now() + KB_WAIT_MS;
    clearTimeout(wait);
    wait = window.setTimeout(update, KB_WAIT_MS + 20);
    update();
  };
  // Focus moving from one field to the next blurs the first before the second takes it.
  const onBlur = () => {
    if (!frame) frame = requestAnimationFrame(update);
  };
  document.addEventListener("focusin", onFocus);
  document.addEventListener("focusout", onBlur);
  vv.addEventListener("resize", update);
  vv.addEventListener("scroll", update);
  return () => {
    document.removeEventListener("focusin", onFocus);
    document.removeEventListener("focusout", onBlur);
    vv.removeEventListener("resize", update);
    vv.removeEventListener("scroll", update);
    clearTimeout(wait);
    cancelAnimationFrame(frame);
    clear();
  };
}
