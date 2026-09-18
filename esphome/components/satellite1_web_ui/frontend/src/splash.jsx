/**
 * The verdict overlay and the fix drawer: the whole surface of the Home Assistant actions
 * onboarding.
 *
 * The overlay mounts with AppInner - the first authenticated moment, right after the login screen
 * hands over or straight away for a returning cookie - and holds the reveal while the boot calls
 * land. On the happy path it is a sub-second fade over an app that is already painted underneath.
 * On the paths that need a person, it says exactly which one this is: the actions checkbox is off
 * (the star of this work, with the walk-through and a watching poll that fades in on its own once
 * the box is ticked), Home Assistant is not connected, Home Assistant predates 2025.12, or the
 * device is slow or answering errors. Every one of those carries "Continue without Home Assistant"
 * into the degraded app, because the device's own controls owe nothing to any of them.
 *
 * Deliberately not on the login screen. That surface is unauthenticated by design - /api/sat1/ha
 * answers it 401 - and telling a stranger on the LAN which of this house's devices cannot perform
 * actions is exactly what the session gate exists to prevent. The checkbox also does not gate
 * sign-in, so a warning there would be about something the visitor can neither act on nor is
 * blocked by.
 *
 * The verdict is read, not guessed: `actions` on /api/sat1/ha is tts_routing's own probe conclusion
 * (see haBlocked/haTooOld in lib/device.js), which is what lets this screen say "tick this
 * checkbox" or "update Home Assistant" instead of hedging between them.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { TEXT } from "./copy.js";
import { deviceIdentity, haBlocked, haTooOld } from "./lib/device.js";
import { Logo, useDrawer, useSheetDrag } from "./ui.jsx";

/**
 * What the splash should show right now, from the same ctx AppInner hands every route.
 *
 * "loading" gates on the fast local calls only - the state endpoint, the SSE stream, the selection -
 * which are sub-second in practice; the minimum display time below is what stops a flash-of-splash,
 * not these. Everything after is the Home Assistant verdict, and the order matters: blocked and
 * too-old are read off `actions` before the connection flag, because ticking the checkbox makes
 * Home Assistant reload the config entry - a bounce this screen must read as "still blocked, still
 * watching" rather than flickering into the not-connected card at the moment of success.
 */
function verdict(ctx) {
  // A 401 never lands here - the shell's onAuthLost flips back to the login screen and this
  // overlay unmounts with AppInner - so an error with no device is the device actually failing.
  if (ctx.deviceError && !ctx.device) return "error";
  if (!ctx.device || !ctx.connected || (!ctx.sel && !ctx.selError) || !ctx.ha) return "loading";
  if (haBlocked(ctx.ha)) return "blocked";
  if (haTooOld(ctx.ha)) return "old";
  // A payload has landed at least once, however old: the app can paint real lists.
  if (ctx.ha.rung === 1 || ctx.ha.rung === 2) return "ready";
  if (!ctx.device.ha) return "noha";
  // Connected, nothing asked yet: the device's own 5s post-connect wait, or the ladder mid-climb.
  return "asking";
}

/** How long "asking" may hold the screen before it admits something is slow. Generous on purpose:
 *  a fresh boot legitimately takes the 5s post-connect wait plus a rung or two. */
const SLOW_MS = 15000;

/** The floor under the happy path, so a fast device gets a fade instead of a flash. */
const MIN_MS = 500;

/** The fade's length; matches the .splash transition in app.css. Under reduced motion the CSS
 *  transition is off and this is just the unmount delay behind an already-invisible overlay. */
const FADE_MS = 500;

/**
 * The Open Home Assistant tap, on phones only: try the companion app's own homeassistant:// scheme,
 * which lands inside the app at the integrations page - one hop, no interstitial tab, no internet
 * needed - and fall back to the My Home Assistant web redirect if nothing claims it. "Nothing
 * claimed it" is read off the page still being visible when the timer fires: an app switch hides
 * the page (and freezes timers - the elapsed check catches one sleeping through the switch and
 * firing on the way back). Desktops skip all of this and let the anchor open the web redirect in a
 * new tab as before; the scheme is a phone thing, and some desktop browsers greet an unclaimed one
 * with their own error dialog.
 */
function openHomeAssistant(e) {
  if (!/iphone|ipad|ipod|android/i.test(navigator.userAgent)) return;
  e.preventDefault();
  const at = Date.now();
  setTimeout(() => {
    if (document.hidden || Date.now() - at > 2500) return;
    // No app took the tap. The gesture window has passed on some browsers, so if the popup is
    // refused, walk this tab over instead - Back returns to the device.
    const w = window.open(TEXT.blocked_open_ha_url, "_blank");
    if (!w) location.assign(TEXT.blocked_open_ha_url);
  }, 1400);
  location.href = TEXT.blocked_open_ha_app_url;
}

/** The cog in step 2: the same mdi:cog glyph Home Assistant draws on the device's row, inline so
 *  the customer matches it by sight instead of by the word "cog". */
const Cog = () => (
  <svg class="fix-cog" viewBox="0 0 24 24" aria-hidden="true">
    <path d="M12,15.5A3.5,3.5 0 0,1 8.5,12A3.5,3.5 0 0,1 12,8.5A3.5,3.5 0 0,1 15.5,12A3.5,3.5 0 0,1 12,15.5M19.43,12.97C19.47,12.65 19.5,12.33 19.5,12C19.5,11.67 19.47,11.34 19.43,11L21.54,9.37C21.73,9.22 21.78,8.95 21.66,8.73L19.66,5.27C19.54,5.05 19.27,4.96 19.05,5.05L16.56,6.05C16.04,5.66 15.5,5.32 14.87,5.07L14.5,2.42C14.46,2.18 14.25,2 14,2H10C9.75,2 9.54,2.18 9.5,2.42L9.13,5.07C8.5,5.32 7.96,5.66 7.44,6.05L4.95,5.05C4.73,4.96 4.46,5.05 4.34,5.27L2.34,8.73C2.21,8.95 2.27,9.22 2.46,9.37L4.57,11C4.53,11.34 4.5,11.67 4.5,12C4.5,12.33 4.53,12.65 4.57,12.97L2.46,14.63C2.27,14.78 2.21,15.05 2.34,15.27L4.34,18.73C4.46,18.95 4.73,19.03 4.95,18.95L7.44,17.94C7.96,18.34 8.5,18.68 9.13,18.93L9.5,21.58C9.54,21.82 9.75,22 10,22H14C14.25,22 14.46,21.82 14.5,21.58L14.87,18.93C15.5,18.67 16.04,18.34 16.56,17.94L19.05,18.95C19.27,19.03 19.54,18.95 19.66,18.73L21.66,15.27C21.78,15.05 21.73,14.78 21.54,14.63L19.43,12.97Z" />
  </svg>
);

/** Step 2's renderer: the string's %c becomes the cog, %s the device's name. Kept as markers in
 *  copy.js so the sentence stays reviewable as one piece of text there. */
const step2 = (tpl, name) => {
  const [before, after] = tpl.split("%c");
  return (
    <>
      {before}
      <Cog />
      {after.replace("%s", name)}
    </>
  );
};

/**
 * The checkbox walk-through, shared verbatim by the splash's blocked card and the fix drawer so the
 * two can never drift apart. `named` picks step 2's form: Home Assistant's own name for the device
 * gets the short sentence, the firmware fallback gets the "unless you renamed it" hedge - see
 * blocked_step2's note in copy.js for the catch-22 behind the two.
 */
function BlockedGuide({ name, named, haRefresh, haRefreshing }) {
  return (
    <>
      <p class="splash-b">{TEXT.blocked_body}</p>
      <ol class="fix-steps">
        <li>{TEXT.blocked_step1}</li>
        <li>{step2(named ? TEXT.blocked_step2 : TEXT.blocked_step2_unnamed, name)}</li>
        <li>{TEXT.blocked_step3}</li>
      </ol>
      {/* The href is the My Home Assistant web redirect, but on phones the click intercepts and
          tries the companion app first - see openHomeAssistant above, and blocked_open_ha's note in
          copy.js for why neither can land on the Configure dialog itself. */}
      <a class="btn solid fix-open" href={TEXT.blocked_open_ha_url} target="_blank" rel="noreferrer" onClick={openHomeAssistant}>
        {TEXT.blocked_open_ha}
      </a>
      {/* One row: a small ring (spinning = actively watching, where the old breathing dot read as
          merely alive) and a sentence that leads with the auto-advance, with the manual check inside
          it as a link rather than standing as a button (owner, September 2026). */}
      <div class="fix-watch">
        <span class="spl-spin" aria-hidden="true" />
        <span class="dim sm">
          {TEXT.blocked_watching}{" "}
          <button class="linkish" onClick={haRefresh} disabled={haRefreshing}>
            {TEXT.blocked_recheck}
          </button>
          .
        </span>
      </div>
    </>
  );
}

export function Splash({ ctx, onDone }) {
  const [leaving, setLeaving] = useState(false);
  const [slow, setSlow] = useState(false);
  // Whether any waiting card was ever shown: what earns the "Connected" beat on the way out, so a
  // recovery reads as one ("it worked") while the everyday load just fades.
  const waited = useRef(false);
  const mountAt = useRef(Date.now());

  const raw = verdict(ctx);
  // `slow` is presentation, not a verdict: the raw state is still "asking" and keeps being polled,
  // this just stops the spinner claiming everything is fine after fifteen seconds of it.
  const v = raw === "asking" && slow ? "slow" : raw;

  useEffect(() => {
    if (raw !== "loading" && raw !== "ready") waited.current = true;
  }, [raw]);

  useEffect(() => {
    if (raw !== "asking") return undefined;
    const t = setTimeout(() => setSlow(true), SLOW_MS);
    return () => clearTimeout(t);
  }, [raw]);

  // The watching poll: one cheap read of the device's cache while a card that can self-heal is up.
  // "asking" is not here because useHaData's own 3s chain already covers it, and "old" is not
  // because nothing short of upgrading Home Assistant changes that answer.
  useEffect(() => {
    if (leaving || (v !== "blocked" && v !== "noha")) return undefined;
    const t = setInterval(() => ctx.haRead(), 3000);
    return () => clearInterval(t);
    // haRead is stable for the life of the app.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [v, leaving]);

  // Ready: fade out - after the "Connected" beat if anyone had to wait, after the minimum display
  // if nobody did.
  useEffect(() => {
    if (leaving || raw !== "ready") return undefined;
    const wait = waited.current ? 900 : Math.max(0, MIN_MS - (Date.now() - mountAt.current));
    const t = setTimeout(() => setLeaving(true), wait);
    return () => clearTimeout(t);
  }, [raw, leaving]);

  useEffect(() => {
    if (!leaving) return undefined;
    const t = setTimeout(onDone, FADE_MS);
    return () => clearTimeout(t);
    // onDone is a setState arrow, new each render; the fade must not restart on it.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [leaving]);

  const identity = deviceIdentity(ctx.device, ctx.ha);
  const card = v === "blocked" || v === "old" || v === "noha" || v === "slow" || v === "error";
  const spinning = v === "loading" || v === "asking" || raw === "ready";
  const status =
    v === "loading" ? TEXT.splash_connecting
    : v === "asking" ? TEXT.splash_asking
    : raw === "ready" ? TEXT.splash_connected
    : null;

  return (
    <div class={`splash${leaving ? " out" : ""}`} role="status">
      <div class="login-glow" aria-hidden="true" />
      <div class="login-hero">
        {/* The same mark the login screen wears, and below it the same ring the boot splash spins
            (owner's call, September 2026: one spinner everywhere - an orbiting-dot flourish lived
            here briefly and read as a different screen). The ring only turns while something is
            genuinely in flight, so a card asking the user for something is not decorated like
            progress. */}
        <Logo />
        <h1 class="login-name">Satellite1</h1>
        {status && <div class="splash-status">{status}</div>}
      </div>
      {spinning && <div class="spl-spin" aria-hidden="true" />}

      {card && (
        <div class="card login-card splash-card">
          {v === "blocked" && (
            <>
              <h2 class="splash-t">{TEXT.blocked_title}</h2>
              <BlockedGuide
                name={identity.name || "this device"}
                named={identity.named}
                haRefresh={ctx.haRefresh}
                haRefreshing={ctx.haRefreshing}
              />
            </>
          )}
          {v === "noha" && (
            <>
              <h2 class="splash-t">{TEXT.noha_title}</h2>
              <p class="splash-b">{TEXT.noha_body}</p>
              <div class="fix-watch">
                <span class="login-pulse" aria-hidden="true" />
                <span class="dim sm">{TEXT.splash_asking}</span>
              </div>
            </>
          )}
          {v === "old" && (
            <>
              <h2 class="splash-t">{TEXT.old_ha_title}</h2>
              <p class="splash-b">{TEXT.old_ha_body}</p>
            </>
          )}
          {v === "slow" && (
            <>
              <p class="splash-b">{TEXT.splash_slow}</p>
              <button class="btn sm fix-check" onClick={ctx.haRefresh} disabled={ctx.haRefreshing}>
                {TEXT.blocked_check}
              </button>
            </>
          )}
          {v === "error" && (
            <>
              <p class="splash-b">{`${TEXT.splash_error} (${ctx.deviceError})`}</p>
              <button class="btn sm fix-check" onClick={() => location.reload()}>
                {TEXT.splash_retry}
              </button>
            </>
          )}
        </div>
      )}

      {card && (
        <div class="splash-skip">
          {/* A real button face, not the ghost it wore first - it did not read as pressable (owner,
              September 2026). The label admits only what is actually missing: on the cards where
              Home Assistant is fine and the actions channel alone is shut, "without Home Assistant"
              would overclaim. */}
          <button class="btn" onClick={() => setLeaving(true)}>
            {v === "blocked" || v === "old" ? TEXT.splash_continue_actions : TEXT.splash_continue}
          </button>
          <p class="dim sm">{TEXT.splash_continue_sub}</p>
        </div>
      )}
    </div>
  );
}

/**
 * The fix drawer: the blocked card's content on the app's unified drawer pattern, opened by the
 * blocked toast and by every in-place "Show fix" link, so the walk-through stays one tap away after
 * the splash's Continue. Polls the cache while open and closes itself into the healthy app the
 * moment a payload lands - which is also what makes "Check again" and the checkbox tick feel like
 * they worked.
 */
export function FixDrawer({ ctx, onClose }) {
  useDrawer("fix", true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose);

  useEffect(() => {
    const t = setInterval(() => ctx.haRead(), 3000);
    return () => clearInterval(t);
    // haRead is stable for the life of the app.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const healthy = ctx.ha && !haBlocked(ctx.ha) && (ctx.ha.rung === 1 || ctx.ha.rung === 2);
  useEffect(() => {
    if (healthy) onClose();
    // onClose is a setState arrow, new each render.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [healthy]);

  const identity = deviceIdentity(ctx.device, ctx.ha);
  return (
    <>
      <div class="scrim mpanel-scrim" onClick={onClose} />
      <div class="mpanel fixpanel" style={dragStyle || undefined} {...drag} role="dialog" aria-label={TEXT.blocked_title}>
        <button class="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
        <div class="mgroup-head dim sm" data-grab>
          {TEXT.blocked_title}
        </div>
        <BlockedGuide
          name={identity.name || "this device"}
          named={identity.named}
          haRefresh={ctx.haRefresh}
          haRefreshing={ctx.haRefreshing}
        />
      </div>
    </>
  );
}
