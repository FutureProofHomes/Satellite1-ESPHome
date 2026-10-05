import React, { useEffect, useRef, useState } from 'react';
import { HINTS, TEXT } from '../../copy.js';
import { requestJson } from '../../lib/device.js';
import { kb, mb } from '../../lib/settings.js';
import type { Ctx } from '../../ctx';
import { DxCard, DxFact, DxFacts } from './dx';

/** [uptime s, internal free, internal largest block, internal low-water, PSRAM free, flags] */
type Sample = [number, number, number, number, number, number];
type Task = [string, number, number, number];
type History = { boot: string; next: number; samples: Sample[]; tasks: Task[] | null };

const MAX_SAMPLES = 12 * 360;
const POLL_MS = 10000;
const TASK_STATE = ['Running', 'Ready', 'Blocked', 'Suspended', 'Deleted'];

/* Per device, so a trip to another page does not refetch twelve hours. */
const histories = new Map<string, History>();

/**
 * GET /api/sat1/sysmon every ten seconds while the card is on screen: only what is newer than the
 * last answer, paged two hours at a time on the first read. A different boot id is a reboot since,
 * and the history starts over.
 */
function useSysmon(key: string) {
  const [, setN] = useState(0);
  useEffect(() => {
    let live = true;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const tick = async () => {
      const h: History = histories.get(key) ?? { boot: '', next: 0, samples: [], tasks: null };
      for (let page = 0; page < 8 && live; page++) {
        const q = h.boot ? `?boot=${h.boot}&since=${h.next}&tasks=1` : '?tasks=1';
        const j: any = await requestJson(`/api/sat1/sysmon${q}`).catch(() => null);
        if (!live || !j || !Array.isArray(j.s)) break;
        if (j.boot !== h.boot) {
          h.boot = j.boot;
          h.samples = [];
        }
        h.samples.push(...j.s);
        if (h.samples.length > MAX_SAMPLES) h.samples.splice(0, h.samples.length - MAX_SAMPLES);
        h.next = j.next;
        h.tasks = Array.isArray(j.tasks) ? j.tasks : null;
        histories.set(key, h);
        setN(n => n + 1);
        if (!(j.next < j.end)) break;
      }
      if (live) timer = setTimeout(tick, POLL_MS);
    };
    tick();
    return () => {
      live = false;
      clearTimeout(timer);
    };
  }, [key]);
  return histories.get(key) ?? null;
}

const cssVar = (el: Element, name: string, fallback: string) => getComputedStyle(el).getPropertyValue(name).trim() || fallback;
const BANDS: [number, string][] = [[1, '--orb-ink'], [2, '--mm-stop'], [4, '--green']];

/** Lines over the history with a value axis; `bands` adds the flag strips along the bottom. */
function Chart({ samples, series, unit, scale, bands = false, label }: {
  samples: Sample[];
  series: { i: number; color: string; dash?: boolean }[];
  unit: string;
  scale: number;
  bands?: boolean;
  label: string;
}) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    c.width = Math.round(c.clientWidth * dpr);
    c.height = Math.round(c.clientHeight * dpr);
    const g = c.getContext('2d');
    if (!g) return;
    const w = c.width;
    const h = c.height;
    g.clearRect(0, 0, w, h);
    if (samples.length < 2) return;
    const bandH = bands ? 4 * dpr * 3 + 4 * dpr : 0;
    const top = 14 * dpr;
    const plotH = h - top - bandH - 2 * dpr;
    let max = 0;
    for (const s of samples) for (const { i } of series) max = Math.max(max, s[i]);
    max = Math.max(1, max * 1.1);
    const t0 = samples[0][0];
    const t1 = samples[samples.length - 1][0];
    const x = (t: number) => ((t - t0) / Math.max(1, t1 - t0)) * w;
    const y = (v: number) => top + plotH - (v / max) * plotH;

    g.font = `${10 * dpr}px -apple-system, sans-serif`;
    g.fillStyle = cssVar(c, '--muted', '#a1a1ad');
    g.strokeStyle = cssVar(c, '--line', 'rgba(255,255,255,.09)');
    g.lineWidth = 1;
    for (const f of [0.5, 1]) {
      const v = (max / 1.1) * f;
      g.beginPath();
      g.moveTo(0, y(v));
      g.lineTo(w, y(v));
      g.stroke();
      g.fillText(`${(v / scale).toFixed(scale > 1e5 ? 1 : 0)} ${unit}`, 2 * dpr, y(v) - 2 * dpr);
    }
    const span = (t1 - t0) / 3600;
    const spanText = span >= 1 ? `${span.toFixed(1)} h` : `${Math.round(span * 60)} min`;
    g.textAlign = 'right';
    g.fillText(spanText, w - 2 * dpr, top - 3 * dpr);
    g.textAlign = 'left';

    for (const s of series) {
      g.strokeStyle = cssVar(c, s.color, '#a78bfa');
      g.lineWidth = 1.5 * dpr;
      g.setLineDash(s.dash ? [4 * dpr, 3 * dpr] : []);
      g.beginPath();
      samples.forEach((p, k) => (k ? g.lineTo(x(p[0]), y(p[s.i])) : g.moveTo(x(p[0]), y(p[s.i]))));
      g.stroke();
    }
    g.setLineDash([]);
    if (bands) {
      BANDS.forEach(([bit, color], b) => {
        g.fillStyle = cssVar(c, color, '#888');
        const by = h - (b + 1) * 5 * dpr;
        for (let k = 0; k < samples.length - 1; k++) {
          if (!(samples[k][5] & bit)) continue;
          const x0 = x(samples[k][0]);
          g.fillRect(x0, by, Math.max(1, x(samples[k + 1][0]) - x0), 4 * dpr);
        }
      });
    }
  });
  return <canvas ref={ref} className="sm-chart" role="img" aria-label={label} />;
}

export function SysMonCard({ ctx }: { ctx: Ctx }) {
  const h = useSysmon(String(ctx.device?.mac || ''));
  const samples = h?.samples ?? [];
  const last = samples[samples.length - 1];
  const tasks = (h?.tasks ?? []).slice().sort((a, b) => a[2] - b[2]);
  return <DxCard title={TEXT.sm_title} collapsible defaultOpen={true} hint={HINTS.sysmon}>
      {last ? <DxFacts>
          <DxFact label={`${TEXT.sm_internal} · ${TEXT.sm_free}`} value={kb(last[1])} />
          <DxFact label={TEXT.sm_block} value={kb(last[2])} />
          <DxFact label={TEXT.sm_min} value={kb(last[3])} />
          <DxFact label={`${TEXT.sm_psram} · ${TEXT.sm_free}`} value={mb(last[4])} />
        </DxFacts> : <p className="dx-muted">{TEXT.sm_waiting}</p>}
      {samples.length > 1 && <div className="sm-charts">
          <div className="sm-legend">
            <span className="mm-legend"><i className="s0" />{TEXT.sm_free}</span>
            <span className="mm-legend"><i className="s1" />{TEXT.sm_block}</span>
            <span className="mm-legend"><i className="sm-min" />{TEXT.sm_min}</span>
          </div>
          <Chart samples={samples} series={[{ i: 1, color: '--orb-ink' }, { i: 2, color: '--accent2' }, { i: 3, color: '--muted', dash: true }]} unit="KB" scale={1024} bands label={TEXT.sm_internal} />
          <div className="sm-legend">
            <span className="mm-legend"><i className="s0" />{TEXT.sm_band_va}</span>
            <span className="mm-legend"><i className="s2" />{TEXT.sm_band_xmos}</span>
            <span className="mm-legend"><i className="sm-g" />{TEXT.sm_band_mic}</span>
          </div>
          <Chart samples={samples} series={[{ i: 4, color: '--orb-ink' }]} unit="MB" scale={1048576} label={TEXT.sm_psram} />
        </div>}
      <div className="dx-row sm-tasks-head">
        <span className="dx-row-label"><span>{TEXT.sm_tasks}</span></span>
        <span className="dx-muted dx-xs">{HINTS.sysmon_tasks}</span>
      </div>
      {h && h.tasks == null ? <p className="dx-muted dx-sm">{TEXT.sm_no_tasks}</p> : <div className="sm-tasks">
          <table>
            <thead><tr><th>{TEXT.sm_task_name}</th><th>{TEXT.sm_task_prio}</th><th>{TEXT.sm_task_stack}</th><th>{TEXT.sm_task_state}</th></tr></thead>
            <tbody>
              {tasks.map((t, i) => <tr key={`${t[0]}-${i}`} className={t[2] < 512 ? 'err' : t[2] < 1024 ? 'warn' : undefined}>
                  <td className="dx-mono">{t[0]}</td>
                  <td>{t[1]}</td>
                  <td>{t[2]} B</td>
                  <td>{TASK_STATE[t[3]] ?? t[3]}</td>
                </tr>)}
            </tbody>
          </table>
        </div>}
    </DxCard>;
}
