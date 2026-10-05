import React, { useEffect, useRef, useState } from 'react';
import { CONFIRM, HINTS, TEXT } from '../../copy.js';
import { entity, pathFor, post, request, requestJson } from '../../lib/device.js';
import { devTools } from '../../lib/devtools.js';
import { ampMode, gainDbv, xmosChoices, xmosLabel, xmosProgress, xmosStatus } from '../../lib/settings.js';
import { toast } from '../../lib/toast.js';
import type { Ctx } from '../../ctx';
import { MSlider } from '../MSlider';
import { DxCard, DxConfirmDialog, DxFact, DxFacts, DxRow, DxSelect } from './dx';
import { MicMonitorCard } from './MicMonitor';
import { MultiLogCard } from './MultiLog';
import { SysMonCard } from './SysMon';

/**
 * GET /api/sat1/amp every two seconds while on screen, each read chained to the last. An audio_dac
 * is not an entity, so these readings cannot ride /events, and a poll of their own keeps them off
 * the state payload every tab receives. A build without the amp answers 404, which ends the poll
 * for good; any other miss leaves the last reading standing for the next read to correct.
 */
function useAmp() {
  const [amp, setAmp] = useState<any>(null);
  useEffect(() => {
    let live = true;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const tick = async () => {
      try {
        const r: any = await request('/api/sat1/amp');
        if (!live || r.status === 404) return;
        if (r.ok) setAmp(JSON.parse(r.text));
      } catch {
        /* the next read retries */
      }
      if (live) timer = setTimeout(tick, 2000);
    };
    tick();
    return () => {
      live = false;
      clearTimeout(timer);
    };
  }, []);
  return amp;
}

/**
 * The TAS2780's live state and its analog gain. The power gain mode follows the USB-C supply, so it
 * is a reading here, not a picker. Digital volume is read-only on purpose: it is the level the
 * firmware computed from the volume sliders, shown so someone chasing "why is it quiet" can see
 * what the amplifier is actually fed. The analog gain's notch is the factory default (index 8,
 * 15 dBV), and the drag snaps to it (owner call, September 2026). The speaker channel is a
 * customer setting and lives on the Audio tab.
 */
function SpeakerAmpCard({ ctx }: { ctx: Ctx }) {
  const amp = useAmp();
  const lineOut = entity(ctx, 'line_out');
  const gain = entity(ctx, 'amp_gain');
  const gainV = Number(gain?.value ?? gain?.state);
  const mode = ampMode(amp);
  return <DxCard title="TAS2780 Amplifier Control" collapsible defaultOpen={true} hint={HINTS.speaker_amp}>
      {mode && <DxRow label="Power gain mode" hint={HINTS.amp_mode}>
          <span className="dx-dim" title={mode.d || undefined}>{mode.v}</span>
        </DxRow>}
      {amp && <DxRow label="Digital volume" hint={HINTS.amp_dvc}><span className="dx-dim">{amp.muted ? 'Muted' : `${amp.dvc}%`}</span></DxRow>}
      {gain && <DxRow label="Analog gain" hint={HINTS.amp_gain}>
          <MSlider value={Number.isFinite(gainV) ? gainV : 0} min={gain.min_value ?? 0} max={gain.max_value ?? 20} step={gain.step ?? 1} snap={8} format={gainDbv} ariaLabel="Analog gain" onCommit={v => post(pathFor(ctx, 'amp_gain', 'set', { value: v }))} />
        </DxRow>}
      {lineOut && <DxRow label="Line out"><span className="dx-dim">{lineOut.value ? 'Connected' : 'Nothing plugged in'}</span></DxRow>}
    </DxCard>;
}

/**
 * The developer XMOS firmware picker. config/satellite1.dev.yaml is what maps the xmos_fw_*
 * entities, so on any other build they are absent and this renders nothing. Choosing a firmware is
 * the install: the dropdown opens the confirm itself, and Yes sets the select and presses Install,
 * then swaps the row for one progress bar driven by the status sensor. An install started from
 * Home Assistant shows the same bar.
 *
 * web_server sends a select's options only when the event stream opens, so the list is re-read
 * whenever the catalog sensor says it changed. The device ignores an install while another flash
 * runs, so a Yes it never acts on is reported after ten seconds rather than leaving the bar at
 * "Stopping audio" for good. Refresh is the same press as Home Assistant's button, and is ignored
 * the same way during an install or within two seconds of the last one.
 */
function XmosFirmwareRow({ ctx }: { ctx: Ctx }) {
  const choice = entity(ctx, 'xmos_fw_choice');
  const status = entity(ctx, 'xmos_fw_status');
  const listText: string | undefined = entity(ctx, 'xmos_fw_list')?.value;
  const choicePath = pathFor(ctx, 'xmos_fw_choice');
  const installPath = pathFor(ctx, 'xmos_fw_install', 'press');
  const refreshPath = pathFor(ctx, 'xmos_fw_refresh', 'press');
  const [fresh, setFresh] = useState<string[] | null>(null);
  const [ask, setAsk] = useState<string | null>(null);
  const [pending, setPending] = useState<{ status: string } | null>(null);
  const [stalled, setStalled] = useState(false);
  const [check, setCheck] = useState<{ running: boolean; status: string; list: string | undefined } | null>(null);
  const [checkStalled, setCheckStalled] = useState(false);
  const pick = useRef<HTMLButtonElement>(null);
  const statusText: string = status?.value ?? '';
  const st = xmosStatus(statusText);
  useEffect(() => {
    if (!check || !check.running && statusText === check.status) return;
    if (st.refreshing) {
      if (!check.running) setCheck({ ...check, running: true });
      return;
    }
    setCheck(null);
    if (check.running && !st.refreshError) toast({
      kind: 'ok',
      key: 'xf-checked',
      ttl: 4000,
      title: listText === check.list ? TEXT.xf_checked_none : TEXT.xf_checked_new
    });
  }, [check, statusText]);
  useEffect(() => {
    if (!check || check.running) return undefined;
    const t = setTimeout(() => {
      setCheck(null);
      setCheckStalled(true);
    }, 10000);
    return () => clearTimeout(t);
  }, [check]);
  useEffect(() => {
    if (!choicePath) return undefined;
    let live = true;
    requestJson(`${choicePath}?detail=all`).then((j: any) => {
      if (live && Array.isArray(j?.option)) setFresh(j.option);
    }).catch(() => {});
    return () => {
      live = false;
    };
  }, [choicePath, listText]);
  useEffect(() => {
    if (pending && statusText !== pending.status) setPending(null);
  }, [pending, statusText]);
  useEffect(() => {
    if (!pending) return undefined;
    const t = setTimeout(() => {
      setPending(null);
      setStalled(true);
    }, 10000);
    return () => clearTimeout(t);
  }, [pending]);
  const all: string[] = fresh ?? choice?.option ?? [];
  if (!choicePath || !installPath || !status || !all.length) return null;
  const { options, value } = xmosChoices(all, entity(ctx, 'xmos_firmware')?.value);
  const progress = xmosProgress(st, !!pending);
  const listError = st.refreshError || (listText?.startsWith('Unavailable: ') ? listText.slice(13) : null);
  const refresh = () => {
    setCheckStalled(false);
    setCheck({ running: false, status: statusText, list: listText });
    post(refreshPath!).catch(() => {});
  };
  const install = (v: string) => {
    setAsk(null);
    setStalled(false);
    setPending({ status: statusText });
    post(`${choicePath}/set?option=${encodeURIComponent(v)}`).then((r: any) => r.ok && post(installPath)).catch(() => {});
  };
  return <>
      {progress ? <div className="dx-row dx-xf" role="status">
          <div className="dx-xf-head"><span>{progress.label}</span><span className="dx-xf-pct">{progress.pct}%</span></div>
          <div className={`dx-progress${progress.warn ? ' warn' : ''}`} role="progressbar" aria-label={progress.label} aria-valuemin={0} aria-valuemax={100} aria-valuenow={progress.pct}>
            <div style={{ width: `${progress.pct}%` }} />
          </div>
        </div> : <DxRow label={TEXT.xf_row} hint={HINTS.xmos_install}>
          <DxSelect value={value} options={options} label={TEXT.xf_row} buttonRef={pick} onChange={v => v !== value && setAsk(v)} />
        </DxRow>}
      {stalled && <p className="dx-err" role="alert">{TEXT.xf_not_started}</p>}
      {!progress && !stalled && st.result && <p className={st.ok ? 'dx-muted dx-sm' : 'dx-err'}>{st.result}</p>}
      {!progress && refreshPath && <DxRow label={TEXT.xf_refresh_row} hint={HINTS.xmos_refresh}>
          <button className="dx-btn" disabled={!!check || st.refreshing} onClick={refresh}>{TEXT.xf_refresh}</button>
        </DxRow>}
      {!progress && st.refreshing && <p className="dx-muted dx-sm">{TEXT.xf_checking}</p>}
      {!progress && !st.refreshing && listError && <p className="dx-err">{TEXT.xf_list_error.replace('%s', listError)}</p>}
      {!progress && checkStalled && <p className="dx-err" role="alert">{TEXT.xf_check_not_started}</p>}
      <DxConfirmDialog open={ask !== null} title={CONFIRM.xmos_install.t.replace('%s', xmosLabel(options, ask))} body={CONFIRM.xmos_install.b} confirmLabel={TEXT.xf_yes} returnFocus={pick} onCancel={() => setAsk(null)} onConfirm={() => ask !== null && install(ask)} />
    </>;
}

/**
 * Settings > Developer, drawn only for firmware built from config/satellite1.dev.yaml (lib/devtools.js
 * decides from what the device reports). Each card is gated on its own piece being present, so a
 * dev build with one piece removed still shows the rest. The XMOS picker comes first, then the
 * microphones it changes, so a firmware swap and its effect are on one screen.
 */
export function DeveloperCards({ ctx }: { ctx: Ctx }) {
  const t = devTools(ctx.device);
  const xmos = entity(ctx, 'xmos_firmware');
  return <div>
      {t.xmos && <DxCard title="XMOS Firmware" collapsible defaultOpen={true} hint={HINTS.xmos}>
          {xmos && <DxFacts>
              <DxFact label="XMOS Firmware" value={xmos.value || '\u2014'} hint={HINTS.xmos} />
            </DxFacts>}
          <XmosFirmwareRow ctx={ctx} />
        </DxCard>}
      {t.mic && <MicMonitorCard ctx={ctx} />}
      {t.amp && <SpeakerAmpCard ctx={ctx} />}
      <MultiLogCard ctx={ctx} />
      {t.sysmon && <SysMonCard ctx={ctx} />}
    </div>;
}
