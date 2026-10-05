import React, { useMemo, useState } from 'react';
import { HINTS, TEXT } from '../../copy.js';
import { collectLogs, formatMerged, logDevices, mergeLines, wallStamp } from '../../lib/multilog.js';
import type { Ctx } from '../../ctx';
import { DxCard, saveBlob } from './dx';
import { LVL_CLASS } from './Logs';

/** The merged view draws only the newest lines; the download carries all of them. */
const SHOWN = 1500;

type Result = { name: string; lines?: { wall: number; lvl: string; text: string }[]; rtt?: number; error?: string };
type Merged = { wall: number; lvl: string; text: string; device: string };

export function MultiLogCard({ ctx }: { ctx: Ctx }) {
  const devices = useMemo(() => logDevices(ctx.ha, ctx.device), [ctx.ha, ctx.device?.mac, ctx.device?.name]);
  // null until the first tick: just this device, whatever its id turns out to be once state loads.
  const [picked, setPicked] = useState<Set<string> | null>(null);
  const on = picked ?? new Set([devices[0].id]);
  const [busy, setBusy] = useState(false);
  const [out, setOut] = useState<{ results: Result[]; merged: Merged[]; at: number } | null>(null);
  const chosen = devices.filter(d => on.has(d.id));
  const toggle = (id: string) => {
    const n = new Set(on);
    if (n.has(id)) n.delete(id);
    else n.add(id);
    setPicked(n);
  };
  const collect = async () => {
    setBusy(true);
    try {
      const results: Result[] = await collectLogs(chosen, { selfFw: ctx.device?.fw });
      setOut({ results, merged: mergeLines(results), at: Date.now() });
    } finally {
      setBusy(false);
    }
  };
  const download = () => {
    if (!out) return;
    const stamp = wallStamp(out.at).slice(0, 16).replace(/[ :]/g, '-');
    saveBlob(new Blob([formatMerged(out.results, out.merged, out.at)], { type: 'text/plain' }), `satellite1-logs_${stamp}.txt`);
  };
  const shown = out ? out.merged.slice(-SHOWN) : [];
  return <DxCard title={TEXT.ml_title} collapsible defaultOpen={true} hint={HINTS.multilog}>
      <div className="ml-devs">
        {devices.map(d => <label key={d.id} className={`ml-dev${d.up ? '' : ' off'}`}>
            <input type="checkbox" checked={on.has(d.id)} onChange={() => toggle(d.id)} />
            <span>{d.name}</span>
            {d.self && <span className="dx-muted dx-xs">{TEXT.ml_this}</span>}
            {!d.up && <span className="dx-muted dx-xs">{TEXT.ml_offline}</span>}
          </label>)}
      </div>
      {!Array.isArray(ctx.ha?.d?.dev) && <p className="dx-muted dx-sm ml-note">{TEXT.ml_no_roster}</p>}
      <div className="dx-row mm-top">
        <button className="dx-btn solid" disabled={busy || chosen.length === 0} onClick={collect}>{busy ? TEXT.ml_collecting : TEXT.ml_collect}</button>
        {out && <button className="dx-btn" onClick={download}>{TEXT.ml_download}</button>}
        {chosen.length === 0 && <span className="dx-muted dx-sm">{TEXT.ml_pick}</span>}
      </div>
      {out && <ul className="ml-results">
          {out.results.map((r, i) => <li key={i}>
              <b>{r.name}</b>{' '}
              {r.lines ? <span className="dx-muted">{TEXT.ml_dev_ok.replace('%n', String(r.lines.length)).replace('%r', String(Math.round(r.rtt ?? 0)))}</span> : <span className="ml-bad">{r.error}</span>}
            </li>)}
        </ul>}
      {shown.length > 0 && <>
          {out!.merged.length > SHOWN && <p className="dx-muted dx-xs ml-note">{TEXT.ml_shown.replace('%n', String(SHOWN))}</p>}
          <div className="dx-log ml-view">
            {shown.map((l, i) => <div key={i} className={`dx-log-line ${LVL_CLASS[l.lvl] || ''}`}>
                <span className="dx-log-ts">{wallStamp(l.wall).slice(11)}</span>
                <span className="dx-log-tag">{l.device}</span>
                <span className="dx-log-txt">{l.text}</span>
              </div>)}
          </div>
        </>}
    </DxCard>;
}
