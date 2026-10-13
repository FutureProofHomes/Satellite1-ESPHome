import React, { useEffect, useState } from 'react';
import { DSP, HINTS } from '../../copy.js';
import { request } from '../../lib/device.js';
import { fmtDb } from '../../lib/eq.js';
import { kb } from '../../lib/settings.js';
import { DxCard, DxFact, DxFacts } from './dx';

/** GET_INFO's bypass_reason, in the order docs/device-control-spi-protocol.md lists them. */
const BYPASS = ['Processing', 'Switched off', 'No profile', 'Flat, passed straight through', 'CPU overload'];
/** GET_INFO's slot_status. */
const SLOT = ['Empty', 'OK', 'Rejected for headroom', 'Designing'];

/**
 * GET /api/sat1/dsp/live every second while on screen and the tab is in front, each read chained
 * to the last. A build without satellite1_dsp answers 404, which ends the poll; any other miss
 * leaves the last reading standing for the next one to correct.
 */
function useLive() {
  const [live, setLive] = useState<any>(null);
  useEffect(() => {
    let on = true;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const tick = async () => {
      if (document.visibilityState === 'visible') {
        try {
          const r: any = await request('/api/sat1/dsp/live');
          if (!on || r.status === 404) return;
          if (r.ok) setLive(JSON.parse(r.text));
        } catch {
          /* the next read retries */
        }
      }
      if (on) timer = setTimeout(tick, 1000);
    };
    tick();
    return () => {
      on = false;
      clearTimeout(timer);
    };
  }, []);
  return live;
}

/**
 * The speaker EQ's health as the audio chip reports it (GET_INFO and GET_STATS), for checking an
 * XMOS firmware swap or a heavy preset.
 */
export function SpeakerEqCard() {
  const d = useLive();
  const info = d?.info;
  const s = d?.stats;
  const worst = (a: number[] | undefined) => (a ? Math.max(a[0], a[1]) : 0);
  const missing = !d ? 'Waiting for the audio chip…' : d.dsp === 0 ? 'This XMOS firmware has no speaker EQ.' : d.online ? 'The speaker EQ has not reported yet.' : 'The audio chip is not answering.';
  return <DxCard title="Speaker EQ" collapsible defaultOpen={true} hint={HINTS.speaker_eq}>
      {info ? <DxFacts>
          <DxFact label="Speaker EQ" value={BYPASS[info.byp] ?? `Reason ${info.byp}`} tone={info.byp >= 4 ? 'warn' : null} sub={`Playing to ${DSP.outputs[d.out] ?? '?'}${info.limreq ? ', limiter armed' : ''}`} />
          {DSP.outputs.map((name: string, i: number) => <DxFact key={name} label={name} value={SLOT[info.st[i]] ?? `Status ${info.st[i]}`} tone={info.st[i] === 2 ? 'err' : null} sub={info.st[i] === 1 ? `${info.sec[i]} sections, loudest ${fmtDb(info.max[i])}` : undefined} />)}
          {s && <>
              <DxFact label="Time per frame" value={worst(s.us_avg)} unit="µs" sub={`worst ${worst(s.us_max)} µs since boot`} />
              <DxFact label="Overruns" value={s.over} tone={s.over > 0 ? 'warn' : null} sub={s.trips > 0 ? `${s.trips} watchdog trips` : 'no watchdog trips'} />
              <DxFact label="Limiter" value={`${s.lim_pct[0]}% / ${s.lim_pct[1]}%`} sub={`deepest ${s.gr.toFixed(1)} dB since boot`} hint={HINTS.xmos_limiter} />
              <DxFact label="Designs" value={info.designs} sub={`${s.rejects} rejected, last took ${(s.design_us / 1000).toFixed(1)} ms`} tone={s.rejects > 0 ? 'warn' : null} />
              <DxFact label="Tile 1 heap" value={kb(s.heap)} sub={`lowest ${kb(s.heap_min)}`} />
            </>}
        </DxFacts> : <p className="dx-muted">{missing}</p>}
    </DxCard>;
}
