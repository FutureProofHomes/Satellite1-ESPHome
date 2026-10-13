import React, { useEffect } from 'react';
import { dspRedirect } from '../lib/dsp.js';
import { routeHash } from '../lib/routes.js';
import { DspPage } from './audio/DspPage';
import { RoutingPage } from './audio/RoutingPage';
import type { Ctx } from '../ctx';

/** Audio's pages, in nav order. The shell lists them only while the Equalizer is visible: with
 *  Announcement Routing alone there is nothing to choose between, and Audio looks exactly as it did
 *  before the Equalizer existed. */
export const AUDIO_ROUTES = [{
  slug: 'routing',
  label: 'Announcement Routing'
}, {
  slug: 'dsp',
  label: 'Equalizer'
}];

export function AudioTab({
  ctx,
  sub
}: {
  ctx: Ctx;
  sub: string | null;
}) {
  // The Equalizer is hidden from the nav on XMOS firmware without the speaker EQ, but a bookmark can
  // still land on it; replace() keeps that dead end out of history, as the Developer page does. Only
  // a definite no bounces: while the XMOS is still connecting, the page waits for its answer.
  const bounce = sub === 'dsp' && dspRedirect(ctx.dsp);
  useEffect(() => {
    if (bounce) location.replace(routeHash('AUDIO', 'routing'));
  }, [bounce]);
  if (sub === 'dsp') return bounce ? null : <DspPage ctx={ctx} />;
  return <RoutingPage ctx={ctx} />;
}
