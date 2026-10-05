import { useEffect, useLayoutEffect, useRef } from 'react';
import type { ReactNode } from 'react';

/**
 * Cards side by side in a strip the person swipes through, on the native scroll's own momentum and
 * snap: the Home tab's wake word windows (owner's design, October 2026: two windows rather than two
 * tabs) and the media bar's speakers. The card off to the side recedes - smaller, dimmer - in step
 * with the finger, and the page dots stretch into a pill as it moves. Tapping a peeking card or a
 * dot, tabbing into a card, or the arrow keys bring it forward, and so does `view` changing from
 * outside, on the browser's smooth scroll. The scroll drives the look through CSS variables set
 * straight on the elements (--p on the root, --k per card, --d per dot), so a swipe never
 * re-renders the page, and `view` follows only once the strip comes to rest. `cue` lights a dot:
 * something new in that card while the person was in another.
 *
 * Class names are `prefix` plus -pager, -track, -page, -card, -dots and -dot, so each caller styles
 * its own; the dots are an overlay the caller positions (both put them on top). `nudge` adds
 * .nudge to the track for the caller's one-shot "there is more" animation - an animation rather
 * than a scripted scroll, because mandatory snap fights a scroll that stops partway.
 */
export function Pager({
  prefix,
  label,
  view,
  onView,
  labels,
  cards,
  keys,
  cue = null,
  cardClass = '',
  article = false,
  nudge = false
}: {
  prefix: string;
  label: string;
  view: number;
  onView: (i: number) => void;
  labels: string[];
  cards: ReactNode[];
  keys?: string[];
  cue?: number | null;
  cardClass?: string;
  article?: boolean;
  nudge?: boolean;
}) {
  const root = useRef<HTMLDivElement>(null);
  const track = useRef<HTMLDivElement>(null);
  const dots = useRef<HTMLDivElement>(null);
  const viewRef = useRef(view);
  viewRef.current = view;
  const onViewRef = useRef(onView);
  onViewRef.current = onView;
  // A finger on the strip: nothing from outside moves it, and where it lands is decided on release.
  const held = useRef(false);
  const rest = useRef(0);
  const placed = useRef(false);
  const n = cards.length;
  const span = () => {
    const el = track.current;
    return el ? el.scrollWidth - el.clientWidth : 0;
  };
  const progress = () => {
    const el = track.current;
    const max = span();
    return el && max > 0 ? el.scrollLeft / max * (n - 1) : 0;
  };
  const paint = () => {
    const el = track.current;
    if (!el) return;
    const at = progress();
    root.current?.style.setProperty('--p', at.toFixed(4));
    Array.from(el.children).forEach((page, i) => {
      const card = page.firstElementChild as HTMLElement | null;
      if (!card) return;
      card.style.setProperty('--k', Math.min(1, Math.abs(at - i)).toFixed(4));
      card.style.transformOrigin = i < at ? '100% 50%' : '0% 50%';
    });
    Array.from(dots.current?.children || []).forEach((dot, i) => {
      (dot as HTMLElement).style.setProperty('--d', Math.max(0, 1 - Math.abs(at - i)).toFixed(4));
    });
  };
  const land = () => {
    const i = Math.round(progress());
    if (!held.current && i !== viewRef.current) onViewRef.current(i);
  };
  useLayoutEffect(() => {
    const el = track.current;
    if (!el || held.current) return;
    const left = n > 1 ? view / (n - 1) * span() : 0;
    if (Math.abs(el.scrollLeft - left) >= 2) {
      const still = !placed.current || window.matchMedia('(prefers-reduced-motion: reduce)').matches;
      el.scrollTo({
        left,
        behavior: still ? 'auto' : 'smooth'
      });
    }
    placed.current = true;
    paint();
  }, [view, n]);
  useEffect(() => {
    const el = track.current;
    if (!el) return;
    // A new width (a phone turned, the window resized) keeps the front card where it was.
    const ro = new ResizeObserver(() => {
      if (!held.current) el.scrollLeft = n > 1 ? viewRef.current / (n - 1) * span() : 0;
      paint();
    });
    ro.observe(el);
    el.addEventListener('scrollend', land);
    return () => {
      ro.disconnect();
      el.removeEventListener('scrollend', land);
      clearTimeout(rest.current);
    };
  }, [n]);
  const release = () => {
    held.current = false;
    clearTimeout(rest.current);
    rest.current = window.setTimeout(land, 140);
  };
  const Card = article ? 'article' : 'div';
  return <div ref={root} className={`${prefix}-pager`}>
    <div ref={track} className={`${prefix}-track` + (nudge ? ' nudge' : '')} role="region" aria-roledescription="carousel" aria-label={label} onScroll={() => {
      requestAnimationFrame(paint);
      clearTimeout(rest.current);
      rest.current = window.setTimeout(land, 140);
    }} onTouchStart={() => {
      held.current = true;
    }} onTouchEnd={release} onTouchCancel={release} onKeyDown={e => {
      if ((e.target as HTMLElement).closest('input, select, textarea')) return;
      const to = e.key === 'ArrowRight' ? view + 1 : e.key === 'ArrowLeft' ? view - 1 : -1;
      if (to < 0 || to >= n) return;
      e.preventDefault();
      onView(to);
    }}>{cards.map((card, i) => <div key={keys?.[i] ?? i} className={`${prefix}-page`}><Card className={(cardClass ? cardClass + ' ' : '') + `${prefix}-card` + (i === view ? ' on' : '')} role="group" aria-roledescription="slide" aria-label={labels[i]} aria-current={i === view || undefined} onClick={() => i !== viewRef.current && onView(i)} onFocusCapture={() => i !== viewRef.current && onView(i)}>{card}</Card></div>)}</div>
    {n > 1 && <div ref={dots} className={`${prefix}-dots`}>{labels.map((l, i) => <button key={keys?.[i] ?? i} type="button" className={`${prefix}-dot` + (i === view ? ' on' : '') + (i === cue ? ' new' : '')} aria-label={l} aria-current={i === view || undefined} onClick={() => onView(i)} />)}</div>}
  </div>;
}
