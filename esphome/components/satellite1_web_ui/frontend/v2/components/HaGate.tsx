import { useEffect, useRef, useState } from 'react';
import { TEXT } from '../../src/copy.js';
import { deviceIdentity, haBlocked, haTooOld } from '../../src/lib/device.js';
import { openHomeAssistant } from '../../src/lib/openha.js';
import type { Ctx } from '../ctx';
import { AlertTriangle } from '../icons';
import { Logo, cogStep } from './bits';

/**
 * The Home Assistant verdict, in the design's "Can't reach Home Assistant" screen: src/splash.jsx's
 * overlay and fix drawer in one.
 *
 * As the splash it covers the app from the first signed-in moment until the verdict is in. On the
 * everyday path that is a short fade over an app already painted underneath; when Home Assistant
 * needs a person, it says which case this is, and every case can continue into the app without it,
 * because the device's own controls owe Home Assistant nothing. As the fix (`fix`) it shows the
 * actions walk-through on demand and closes itself once the checkbox is ticked.
 *
 * The verdict is read off the device, not guessed: `actions` on /api/sat1/ha is tts_routing's own
 * conclusion, which is what lets the copy say "tick this checkbox" or "update Home Assistant".
 */
type Verdict = 'error' | 'loading' | 'blocked' | 'old' | 'ready' | 'noha' | 'asking';

function verdict(ctx: Ctx): Verdict {
  // A 401 never lands here (the shell goes back to sign-in), so an error with no device is the
  // device actually failing.
  if (ctx.deviceError && !ctx.device) return 'error';
  if (!ctx.device || !ctx.connected || !ctx.sel && !ctx.selError || !ctx.ha) return 'loading';
  // Before the connection flag: ticking the checkbox reloads the config entry, a bounce that must
  // read as "still watching" rather than flicker into "not connected" at the moment of success.
  if (haBlocked(ctx.ha)) return 'blocked';
  if (haTooOld(ctx.ha)) return 'old';
  if (ctx.ha.rung === 1 || ctx.ha.rung === 2) return 'ready';
  if (!ctx.device.ha) return 'noha';
  return 'asking';
}

/** How long "asking" may hold before it admits something is slow; a fresh boot legitimately takes
 *  the device's 5s post-connect wait plus a rung or two. */
const SLOW_MS = 15000;
/** The floor under the happy path, so a fast device gets a fade instead of a flash. */
const MIN_MS = 500;
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
  // Whether anyone had to wait, which earns the "Connected" beat on the way out.
  const waited = useRef(false);
  const mountAt = useRef(Date.now());
  const raw = verdict(ctx);
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

  const identity = deviceIdentity(ctx.device, ctx.ha);
  const name = identity.name || 'this device';
  const card = v === 'blocked' || v === 'old' || v === 'noha' || v === 'slow' || v === 'error';
  const status = v === 'loading' ? TEXT.splash_connecting : v === 'asking' ? TEXT.splash_asking : healthy ? TEXT.splash_connected : null;
  const watching = <div className="wiz-wait"><span className="wiz-pulse" /><span>{TEXT.blocked_watching} <button className="text-button gate-inline" onClick={ctx.haRefresh} disabled={ctx.haRefreshing}>{TEXT.blocked_recheck}</button>.</span></div>;
  const proceed = <button className="ha-block-btn ghost" onClick={() => setLeaving(true)}>{fix ? 'Close' : v === 'blocked' || v === 'old' ? TEXT.splash_continue_actions : TEXT.splash_continue}</button>;
  return <div className={'ha-block-screen gate' + (leaving ? ' out' : '')} role="status" data-theme={document.documentElement.dataset.theme}>
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
