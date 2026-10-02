import { useEffect, useRef, useState } from 'react';
import { TEXT } from '../copy.js';
import { deviceIdentity, haBlocked, haTooOld } from '../lib/device.js';
import { openHomeAssistant } from '../lib/openha.js';
import type { Ctx } from '../ctx';
import { AlertTriangle } from '../icons';
import { Logo, cogStep } from './bits';

/**
 * The Home Assistant verdict, in the design's "Can't reach Home Assistant" screen: the onboarding
 * splash and the actions fix in one (docs/web-ui.md, "The actions verdict and the onboarding
 * splash").
 *
 * As the splash it covers the app from the first signed-in moment - right after the sign-in screen
 * hands over, or straight away for a returning cookie - until the verdict is in. On the everyday
 * path that is a short fade over an app already painted underneath; when Home Assistant needs a
 * person, it says which case this is (the actions checkbox off, not connected, older than 2025.12,
 * or the device slow or failing), and every case can continue into the app without it, because the
 * device's own controls owe Home Assistant nothing.
 *
 * As the fix (`fix`) it shows the actions walk-through on demand - from the blocked toast and every
 * in-place "Show fix" link, so it stays one tap away after Continue - polls the cache while open,
 * and closes itself into the healthy app the moment a payload lands, which is also what makes the
 * recheck and the checkbox tick feel like they worked.
 *
 * Deliberately not on the sign-in screen. That surface is unauthenticated by design -
 * /api/sat1/ha answers it 401 - and telling a stranger on the LAN which of this house's devices
 * cannot perform actions is exactly what the session gate exists to prevent. The checkbox does not
 * gate sign-in either, so a warning there would be about something the visitor can neither act on
 * nor is blocked by.
 *
 * The verdict is read off the device, not guessed: `actions` on /api/sat1/ha is tts_routing's own
 * conclusion (see haBlocked/haTooOld in src/lib/device.js), which is what lets the copy say "tick
 * this checkbox" or "update Home Assistant" instead of hedging between them.
 */
type Verdict = 'error' | 'loading' | 'blocked' | 'old' | 'ready' | 'noha' | 'asking';

/**
 * What the gate should show right now. "loading" waits on the fast local calls only - the state
 * endpoint, the SSE stream, the selection - which are sub-second in practice; MIN_MS below is what
 * stops a flash of the gate, not these.
 */
function verdict(ctx: Ctx): Verdict {
  // A 401 never lands here (the shell goes back to sign-in), so an error with no device is the
  // device actually failing.
  if (ctx.deviceError && !ctx.device) return 'error';
  if (!ctx.device || !ctx.connected || !ctx.sel && !ctx.selError || !ctx.ha) return 'loading';
  // Before the connection flag: ticking the checkbox reloads the config entry, a bounce that must
  // read as "still watching" rather than flicker into "not connected" at the moment of success.
  if (haBlocked(ctx.ha)) return 'blocked';
  if (haTooOld(ctx.ha)) return 'old';
  // A payload has landed at least once, however old: the app can paint real lists.
  if (ctx.ha.rung === 1 || ctx.ha.rung === 2) return 'ready';
  if (!ctx.device.ha) return 'noha';
  // Connected, nothing asked yet: the device's own 5s post-connect wait, or the ladder mid-climb.
  return 'asking';
}

/** How long "asking" may hold before it admits something is slow; a fresh boot legitimately takes
 *  the device's 5s post-connect wait plus a rung or two. */
const SLOW_MS = 15000;
/** The floor under the happy path, so a fast device gets a fade instead of a flash. */
const MIN_MS = 500;
/** The fade's length; matches the .gate opacity transition in src/styles/shell.css. */
const FADE_MS = 300;

export function HaGate({
  ctx,
  fix = false,
  onDone
}: {
  ctx: Ctx;
  fix?: boolean;
  onDone: () => void;
}) {
  const [leaving, setLeaving] = useState(false);
  const [slow, setSlow] = useState(false);
  // Whether any waiting card was ever shown: what earns the "Connected" beat on the way out, so a
  // recovery reads as one ("it worked") while the everyday load just fades.
  const waited = useRef(false);
  const mountAt = useRef(Date.now());
  const raw = verdict(ctx);
  // `slow` is presentation, not a verdict: the raw state is still "asking" and keeps being polled;
  // this only stops the screen claiming all is well after fifteen seconds of it.
  const v = fix ? 'blocked' : raw === 'asking' && slow ? 'slow' : raw;
  const healthy = raw === 'ready';
  useEffect(() => {
    if (raw !== 'loading' && raw !== 'ready') waited.current = true;
  }, [raw]);
  useEffect(() => {
    if (raw !== 'asking') return undefined;
    const t = setTimeout(() => setSlow(true), SLOW_MS);
    return () => clearTimeout(t);
  }, [raw]);
  // One cheap read of the device's cache while a card that can heal itself is up. "asking" is
  // covered by useHaData's own chain; "old" changes only when Home Assistant is upgraded.
  useEffect(() => {
    if (leaving || v !== 'blocked' && v !== 'noha') return undefined;
    const t = setInterval(() => ctx.haRead(), 3000);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [v, leaving]);
  // Healthy: fade out - after the "Connected" beat if anyone had to wait, after the minimum display
  // if nobody did, and at once for the fix, whose whole job is done.
  useEffect(() => {
    if (leaving || !healthy) return undefined;
    const wait = fix ? 0 : waited.current ? 900 : Math.max(0, MIN_MS - (Date.now() - mountAt.current));
    const t = setTimeout(() => setLeaving(true), wait);
    return () => clearTimeout(t);
  }, [healthy, leaving, fix]);
  useEffect(() => {
    if (!leaving) return undefined;
    const t = setTimeout(onDone, FADE_MS);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [leaving]);

  // `named` picks step 2's form below: Home Assistant's own name for the device gets the short
  // sentence, the firmware fallback gets the "unless you renamed it" hedge - see blocked_step2's
  // note in src/copy.js for the catch-22 behind the two. The Open Home Assistant link is the My Home
  // Assistant web redirect, but on phones the click opens the companion app instead (see
  // src/lib/openha.js); the web path is the separate visible link, never an automatic fallback.
  const identity = deviceIdentity(ctx.device, ctx.ha);
  const name = identity.name || 'this device';
  const card = v === 'blocked' || v === 'old' || v === 'noha' || v === 'slow' || v === 'error';
  const status = v === 'loading' ? TEXT.splash_connecting : v === 'asking' ? TEXT.splash_asking : healthy ? TEXT.splash_connected : null;
  const watching = <div className="wiz-wait"><span className="wiz-pulse" /><span>{TEXT.blocked_watching} <button className="text-button gate-inline" onClick={ctx.haRefresh} disabled={ctx.haRefreshing}>{TEXT.blocked_recheck}</button>.</span></div>;
  // The label admits only what is actually missing: where Home Assistant is fine and the actions
  // channel alone is shut, "without Home Assistant" would overclaim.
  const proceed = <button className="ha-block-btn ghost" onClick={() => setLeaving(true)}>{fix ? 'Close' : v === 'blocked' || v === 'old' ? TEXT.splash_continue_actions : TEXT.splash_continue}</button>;
  return <div className={'ha-block-screen gate' + (leaving ? ' out' : '')} role="status">
      {!card ? <div className="gate-wait"><Logo cls="login-logo" /><div className="wiz-wait"><span className="wiz-pulse" /><span>{status}</span></div></div> : <div className="ha-block-card">
        <AlertTriangle size={40} className="ha-block-icon" />
        {v === 'blocked' && <>
          <h2 className="ha-block-title">{TEXT.blocked_title}</h2>
          <p className="ha-block-body">{TEXT.blocked_body}</p>
          <ol className="gate-steps">
            <li>{TEXT.blocked_step1}</li>
            <li>{cogStep(identity.named ? TEXT.blocked_step2 : TEXT.blocked_step2_unnamed, name)}</li>
            <li>{TEXT.blocked_step3}</li>
          </ol>
          <a className="ha-block-btn primary gate-link" href={TEXT.blocked_open_ha_url} target="_blank" rel="noreferrer" onClick={e => openHomeAssistant(e, TEXT.blocked_open_ha_app_url)}>{TEXT.blocked_open_ha}</a>
          <a className="text-button" href={TEXT.blocked_open_ha_url} target="_blank" rel="noreferrer">{TEXT.blocked_open_ha_web}</a>
          {watching}
        </>}
        {v === 'noha' && <>
          <h2 className="ha-block-title">{TEXT.noha_title}</h2>
          <p className="ha-block-body">{TEXT.noha_body}</p>
          <div className="wiz-wait"><span className="wiz-pulse" /><span>{TEXT.splash_asking}</span></div>
        </>}
        {v === 'old' && <>
          <h2 className="ha-block-title">{TEXT.old_ha_title}</h2>
          <p className="ha-block-body">{TEXT.old_ha_body}</p>
        </>}
        {v === 'slow' && <>
          <h2 className="ha-block-title">Can't reach Home Assistant</h2>
          <p className="ha-block-body">{TEXT.splash_slow}</p>
          <button className="ha-block-btn primary" onClick={ctx.haRefresh} disabled={ctx.haRefreshing}>{TEXT.blocked_check}</button>
        </>}
        {v === 'error' && <>
          <h2 className="ha-block-title">Can't reach your Satellite1</h2>
          <p className="ha-block-body">{`${TEXT.splash_error} (${ctx.deviceError})`}</p>
          <button className="ha-block-btn primary" onClick={() => location.reload()}>{TEXT.splash_retry}</button>
        </>}
        {proceed}
        {!fix && <p className="ha-block-body gate-sub">{TEXT.splash_continue_sub}</p>}
      </div>}
    </div>;
}
