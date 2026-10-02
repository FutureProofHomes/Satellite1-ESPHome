import React, { useEffect, useRef, useState } from 'react';
import { CONFIRM, HINTS, TEXT } from '../copy.js';
import { entity, entityPath, pathFor, post, request, requestJson } from '../lib/device.js';
import { takeIntent, toast } from '../lib/toast.js';
import type { Ctx } from '../ctx';
import { ampMode, gainDbv, ingressYaml, kb, mb, uptime, usbFact, xmosChoices, xmosLabel, xmosProgress, xmosStatus } from '../lib/settings.js';
import { MSlider } from './MSlider';
import { DxCard, DxConfirm, DxConfirmDialog, DxFact, DxFacts, DxRow, DxSelect, useCopied } from './settings/dx';
import { Switch } from './controls';
import { CrashCard, LogsCard } from './settings/Logs';
import { AuthTokenCard, ChangePasswordCard } from './settings/Security';

const RELEASES_URL = 'https://github.com/FutureProofHomes/Satellite1-ESPHome/releases';
const HASS_INGRESS_URL = 'https://github.com/lovelylain/hass_ingress';
const isOn = (e: any) => !!e && (e.value === true || e.state === 'ON');

/** Stands in for a card whose every fact comes from the first state poll. */
function Pending({
  ctx,
  title
}: {
  ctx: Ctx;
  title: string;
}) {
  return <DxCard title={title} collapsible defaultOpen={true}>
      {ctx.deviceError ? <p className="dx-err" role="alert">{ctx.deviceError}</p> : <p className="dx-muted">Reading…</p>}
    </DxCard>;
}

/**
 * The live facts from GET /api/sat1/state (the shell polls it every two seconds), plus the
 * entities that ride /events. The state facts deliberately have no entities: each would cost RAM,
 * a polling component and a place in the /events stream every open tab receives. Only internal RAM
 * gets the warning colours, because it is what runs out first; PSRAM is plentiful enough that
 * colouring it would only train people to ignore the colour.
 */
function DeviceCard({
  ctx,
  onUpdates
}: {
  ctx: Ctx;
  onUpdates: () => void;
}) {
  const d = ctx.device;
  if (!d) return <Pending ctx={ctx} title="Device" />;
  const upd = entity(ctx, 'firmware');
  const available = upd?.state === 'UPDATE AVAILABLE';
  const installing = upd?.state === 'INSTALLING';
  // ESP32 Temp and USB-C Power Supply are the entity-backed facts, so a build without the sensor
  // omits the row rather than showing a dash for a reading that will never come. Temperature
  // follows temp_unit_f so the app never shows mixed units; its tone stays on the published °C.
  const temp = entity(ctx, 'esp_temp');
  const tempC = Number(temp?.value);
  const isF = isOn(entity(ctx, 'temp_unit_f'));
  const usb = usbFact(entity(ctx, 'usb_power')?.value);
  const heapPct = d.heap.total ? d.heap.free / d.heap.total * 100 : 100;
  return <DxCard title="Device" collapsible defaultOpen={true}>
      <DxFacts>
        <DxFact label="Sat1 Firmware" value={<a href={RELEASES_URL} target="_blank" rel="noopener noreferrer" className="dx-link">{d.fw || '—'}</a>} sub={available ? <button type="button" className="dx-inline" onClick={onUpdates}>Update available: {upd.value}</button> : installing ? 'Installing…' : upd ? 'Up to date' : undefined} />
        <DxFact label="Internal RAM free" value={kb(d.heap.free)} unit={` of ${kb(d.heap.total)}`} hint={HINTS.heap} tone={heapPct < 10 ? 'err' : heapPct < 20 ? 'warn' : undefined} />
        {d.psram?.installed !== false && <DxFact label="PSRAM free" value={mb(d.psram.free)} unit={` of ${mb(d.psram.total)}`} hint={HINTS.psram} />}
        <DxFact label="Longest loop" value={d.loop_ms} unit=" ms" hint={HINTS.loop} tone={d.loop_ms > 500 ? 'err' : d.loop_ms > 150 ? 'warn' : undefined} />
        {Number.isFinite(tempC) && <DxFact label="ESP32 Temp" value={(isF ? tempC * 9 / 5 + 32 : tempC).toFixed(1)} unit={isF ? ' °F' : ' °C'} hint={HINTS.esp_temp} tone={tempC > 80 ? 'err' : tempC > 70 ? 'warn' : undefined} />}
        <DxFact label="Uptime" value={uptime(d.uptime)} />
        <DxFact label="Last restart" value={d.reset || '—'} hint={HINTS.reset} />
        {usb && <DxFact label="USB-C Power Supply" value={usb.value} sub={usb.sub} hint={HINTS.usb_power} />}
        <DxFact label="Network Type" value={d.net === 'ethernet' ? 'Ethernet' : d.rssi != null ? `Wi-Fi ${d.rssi} dBm` : 'Wi-Fi'} sub={<span><span className="dx-mono">{d.ip}</span><span className="dx-mono">{d.mac}</span></span>} />
      </DxFacts>
    </DxCard>;
}
const BUTTONS: [string, string, string?][] = [['btn_up', 'Volume up'], ['btn_down', 'Volume down'], ['btn_mute', 'Mute', 'warn'], ['btn_action', 'Action']];

/**
 * The four physical buttons, live from their binary_sensors, with the last gesture the action
 * button's event entity named. A way to tell a dead button from a dead automation.
 */
function ButtonsCard({
  ctx
}: {
  ctx: Ctx;
}) {
  const ev = entity(ctx, 'action_button');
  const [last, setLast] = useState<string | null>(null);
  useEffect(() => {
    if (ev?.event_type) setLast(String(ev.event_type).replace(/_/g, ' '));
  }, [ev?.event_type]);
  const rows = BUTTONS.map(([key, label, tone]) => ({
    label,
    tone,
    e: entity(ctx, key)
  })).filter(r => r.e);
  if (!rows.length) return null;
  return <DxCard title="Buttons" collapsible defaultOpen={true} right={last && <span className="dx-card-last">last: {last}</span>}>
      <div className="dx-bstates">
        {rows.map(r => <span key={r.label} className={`dx-bstate${r.tone ? ` ${r.tone}` : ''}${r.e.value ? ' on' : ''}`}>{r.label}</span>)}
      </div>
      <p className="dx-muted dx-xs">Press a button on the device; it lights up here.</p>
    </DxCard>;
}

/**
 * The firmware update entity and the beta channel switch, both optional (satellite1.yaml adds
 * them). Install is a POST to the update entity; the device restarts itself when it finishes.
 * ESPHome's update entity reports UNKNOWN, NO UPDATE, UPDATE AVAILABLE or INSTALLING, and
 * INSTALLING keeps the update row standing: treated as nothing to offer, it would vanish the
 * instant Install is pressed, which reads as a failed press at the one moment the device is too
 * busy to answer for a while.
 */
function FirmwareCard({
  ctx,
  reveal
}: {
  ctx: Ctx;
  reveal: boolean;
}) {
  const upd = entity(ctx, 'firmware');
  const beta = entity(ctx, 'beta_firmware');
  const available = upd?.state === 'UPDATE AVAILABLE';
  const installing = upd?.state === 'INSTALLING';
  const fw = ctx.device?.fw || upd?.current_version || '—';
  return <DxCard title="Updates" collapsible defaultOpen={true} forceOpen={reveal} id="card-firmware">
      <div className={`dx-row${available || installing ? ' dx-upd-row' : ''}`} style={{
      alignItems: 'center'
    }}>
        <div style={{
        flex: 1,
        minWidth: 0,
        display: 'flex',
        flexDirection: 'column'
      }}>
          <div className="dx-fact-label"><span>Sat1 Firmware</span></div>
          <span className="dx-fact-v"><a href={RELEASES_URL} target="_blank" rel="noopener noreferrer" className="dx-link">{fw}</a></span>
          {upd && <div className="dx-fact-sub">
              {installing ? <span>Installing {upd.value}…</span> : available ? <span>{upd.value} available</span> : <span>Up to date</span>}
              {(available || installing) && upd.release_url && <a className="dx-link" href={upd.release_url} target="_blank" rel="noopener noreferrer">Release notes</a>}
            </div>}
        </div>
        {(available || installing) && <DxConfirm solid label={installing ? 'Do not cut power' : 'Update'} disabled={installing} title={CONFIRM.update.t} body={CONFIRM.update.b} confirmLabel={`Install ${upd.value}`} onConfirm={() => post(pathFor(ctx, 'firmware', 'install'))} />}
      </div>
      {!upd && <p className="dx-muted dx-sm">This firmware build does not check for updates.</p>}
      {beta && <DxRow label="Beta updates" hint={HINTS.beta}>
          <Switch on={isOn(beta)} label="Beta updates" onChange={v => post(pathFor(ctx, 'beta_firmware', v ? 'turn_on' : 'turn_off'))} />
        </DxRow>}
    </DxCard>;
}

/**
 * The hass_ingress YAML for this device and every peer Home Assistant knows about. Needs the IP,
 * because the proxy cannot resolve mDNS.
 */
function HomeAssistantCard({
  ctx
}: {
  ctx: Ctx;
}) {
  const d = ctx.device;
  const [copied, copy] = useCopied();
  if (!d) return <Pending ctx={ctx} title="HA Side Panel" />;
  if (!d.ip) return null;
  const yaml = ingressYaml(d, ctx.ha?.d?.dev);
  return <DxCard title="HA Side Panel" collapsible defaultOpen={true} hint={HINTS.ha_ingress}>
      <p className="dx-muted">
        <span>{TEXT.hai_pre}</span>
        <a className="dx-link" href={HASS_INGRESS_URL} target="_blank" rel="noopener noreferrer">{TEXT.hai_link}</a>
        <span>{TEXT.hai_post}</span>
      </p>
      <pre className="dx-yaml">{yaml}</pre>
      <div style={{
      padding: '0 16px'
    }}>
        <button className="dx-btn" onClick={() => copy(yaml)}>{copied ? TEXT.hai_copied : TEXT.hai_copy}</button>
      </div>
      <p className="dx-muted dx-sm">{TEXT.hai_dhcp_hint}</p>
    </DxCard>;
}

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
 * The TAS2780's live state and its two user settings. The power gain mode follows the USB-C supply,
 * so it is a reading here, not a picker. Digital volume is read-only on purpose: it is the level the
 * firmware computed from the volume sliders, shown so someone chasing "why is it quiet" can see
 * what the amplifier is actually fed. The analog gain is a real number entity, the person's to set;
 * its notch is the factory default (index 8, 15 dBV), and the drag snaps to it (owner call,
 * September 2026).
 */
function SpeakerAmpCard({
  ctx
}: {
  ctx: Ctx;
}) {
  const amp = useAmp();
  const chan = entity(ctx, 'speaker_channel');
  const lineOut = entity(ctx, 'line_out');
  const gain = entity(ctx, 'amp_gain');
  const gainV = Number(gain?.value ?? gain?.state);
  const mode = ampMode(amp);
  // Existence from the key table, which arrives with the device; the values ride /events a beat
  // later, and judging by them would say "not available" on every load.
  const has = (key: string) => !!ctx.device?.e?.[key];
  if (!amp && !has('speaker_channel') && !has('line_out') && !has('amp_gain')) {
    if (!ctx.device) return <Pending ctx={ctx} title="TAS2780 Amplifier Control" />;
    return <DxCard title="TAS2780 Amplifier Control" collapsible defaultOpen={true} hint={HINTS.speaker_amp}>
        <p className="dx-muted">The speaker amplifier is not available on this firmware build.</p>
      </DxCard>;
  }
  return <DxCard title="TAS2780 Amplifier Control" collapsible defaultOpen={true} hint={HINTS.speaker_amp}>
      {mode && <DxRow label="Power gain mode" hint={HINTS.amp_mode}>
          <span className="dx-dim" title={mode.d || undefined}>{mode.v}</span>
        </DxRow>}
      {amp && <DxRow label="Digital volume" hint={HINTS.amp_dvc}><span className="dx-dim">{amp.muted ? 'Muted' : `${amp.dvc}%`}</span></DxRow>}
      {gain && <DxRow label="Analog gain" hint={HINTS.amp_gain}>
          <MSlider value={Number.isFinite(gainV) ? gainV : 0} min={gain.min_value ?? 0} max={gain.max_value ?? 20} step={gain.step ?? 1} snap={8} format={gainDbv} ariaLabel="Analog gain" onCommit={v => post(pathFor(ctx, 'amp_gain', 'set', { value: v }))} />
        </DxRow>}
      {chan && <DxRow label="Channel" hint={HINTS.speaker_channel}>
          <DxSelect value={chan.value} options={chan.option || []} label="Channel" onChange={v => v !== chan.value && post(`${pathFor(ctx, 'speaker_channel', 'set')}?option=${encodeURIComponent(v)}`)} />
        </DxRow>}
      {lineOut && <DxRow label="Line out"><span className="dx-dim">{lineOut.value ? 'Connected' : 'Nothing plugged in'}</span></DxRow>}
    </DxCard>;
}

/**
 * The developer XMOS firmware picker, drawn only by firmware built from config/satellite1.dev.yaml:
 * web_ui_xmos_catalog.yaml is what maps the xmos_fw_* entities, so on any other build they are
 * absent and this renders nothing. Choosing a firmware is the install: the dropdown opens the
 * confirm itself, and Yes sets the select and presses Install, then swaps the row for one progress
 * bar driven by the status sensor. An install started from Home Assistant shows the same bar.
 *
 * web_server sends a select's options only when the event stream opens, so the list is re-read
 * whenever the catalog sensor says it changed. The device ignores an install while another flash
 * runs, so a Yes it never acts on is reported after ten seconds rather than leaving the bar at
 * "Stopping audio" for good. Refresh is the same press as Home Assistant's button, and is ignored
 * the same way during an install or within two seconds of the last one.
 */
function XmosFirmwareRow({
  ctx
}: {
  ctx: Ctx;
}) {
  const choice = entity(ctx, 'xmos_fw_choice');
  const status = entity(ctx, 'xmos_fw_status');
  const listText: string | undefined = entity(ctx, 'xmos_fw_list')?.value;
  const choicePath = pathFor(ctx, 'xmos_fw_choice');
  const installPath = pathFor(ctx, 'xmos_fw_install', 'press');
  const refreshPath = pathFor(ctx, 'xmos_fw_refresh', 'press');
  const [fresh, setFresh] = useState<string[] | null>(null);
  const [ask, setAsk] = useState<string | null>(null);
  const [pending, setPending] = useState<{
    status: string;
  } | null>(null);
  const [stalled, setStalled] = useState(false);
  const [check, setCheck] = useState<{
    running: boolean;
    status: string;
    list: string | undefined;
  } | null>(null);
  const [checkStalled, setCheckStalled] = useState(false);
  const pick = useRef<HTMLButtonElement>(null);
  const statusText: string = status?.value ?? '';
  const st = xmosStatus(statusText);
  useEffect(() => {
    if (!check || !check.running && statusText === check.status) return;
    if (st.refreshing) {
      if (!check.running) setCheck({
        ...check,
        running: true
      });
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
  const {
    options,
    value
  } = xmosChoices(all, entity(ctx, 'xmos_firmware')?.value);
  const progress = xmosProgress(st, !!pending);
  const listError = st.refreshError || (listText?.startsWith('Unavailable: ') ? listText.slice(13) : null);
  const refresh = () => {
    setCheckStalled(false);
    setCheck({
      running: false,
      status: statusText,
      list: listText
    });
    post(refreshPath!).catch(() => {});
  };
  const install = (v: string) => {
    setAsk(null);
    setStalled(false);
    setPending({
      status: statusText
    });
    post(`${choicePath}/set?option=${encodeURIComponent(v)}`).then((r: any) => r.ok && post(installPath)).catch(() => {});
  };
  return <>
      {progress ? <div className="dx-row dx-xf" role="status">
          <div className="dx-xf-head"><span>{progress.label}</span><span className="dx-xf-pct">{progress.pct}%</span></div>
          <div className={`dx-progress${progress.warn ? ' warn' : ''}`} role="progressbar" aria-label={progress.label} aria-valuemin={0} aria-valuemax={100} aria-valuenow={progress.pct}>
            <div style={{
          width: `${progress.pct}%`
        }} />
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
 * Only the buttons the firmware maps (web_ui.yaml's entity table) are drawn; erase_xmos_flash is
 * deliberately unmapped, for the reason recorded beside it in config/common/web_ui.yaml. The radar
 * card uses satellite1_radar's generic buttons, which exist on whichever module was detected,
 * rather than the per-model ones. It is gated on radar_module, which satellite1_radar sets only
 * once detection succeeds - the moment it registers "Radar Restart" and "Radar Factory Reset" - so
 * the gate and the endpoints agree by construction. Those buttons and "Radar Firmware" are runtime
 * entities named by C++ literals, with no YAML id for the entity map, so they are addressed by
 * name. The stock-component ld2410/ld2450 variant builds remove radar_detected_text (the
 * radar_module key), and the card never renders there.
 */
function RecoveryCards({
  ctx
}: {
  ctx: Ctx;
}) {
  const d = ctx.device;
  if (!d) return <Pending ctx={ctx} title="ESP32 System Control" />;
  const restart = pathFor(ctx, 'restart', 'press');
  const safe = pathFor(ctx, 'safe_mode', 'press');
  const factory = pathFor(ctx, 'factory_reset', 'press');
  const xmosReset = pathFor(ctx, 'xmos_reset', 'press');
  const xmosFlash = pathFor(ctx, 'xmos_flash', 'press');
  // Satellite1::status_string(): "v1.2.3" while the chip is talking, else "XMOS not responding",
  // "Flashing Mode" or "" - so the XMOS Firmware fact is a state as often as a version.
  const xmos = entity(ctx, 'xmos_firmware');
  const mod = entity(ctx, 'radar_module')?.value;
  const radar = mod === 'LD2410' || mod === 'LD2450' ? mod : null;
  const radarFw = ctx.states['text_sensor/Radar Firmware'];
  return <div>
      <DxCard title="ESP32 System Control" collapsible defaultOpen={true} hint={HINTS.maintenance}>
        <DxFacts>
          <DxFact label="ESPHome Version" value={d.esphome || '—'} />
          <DxFact label="Built" value={d.built || '—'} />
        </DxFacts>
        {restart && <DxRow label="Restart">
            <DxConfirm label="Restart" title={CONFIRM.restart.t} body={CONFIRM.restart.b} confirmLabel="Restart" onConfirm={() => post(restart)} />
          </DxRow>}
        {safe && <DxRow label="Safe mode" hint={HINTS.safe_mode}>
            <DxConfirm label="Safe mode" title={CONFIRM.safe_mode.t} body={CONFIRM.safe_mode.b} confirmLabel="Restart into safe mode" onConfirm={() => post(safe)} />
          </DxRow>}
        {factory && <DxRow label="Factory reset" hint={HINTS.factory_reset}>
            <DxConfirm label="Factory reset" danger title={CONFIRM.factory_reset.t} body={CONFIRM.factory_reset.b} confirmLabel="Erase everything" onConfirm={() => post(factory)} />
          </DxRow>}
        {!restart && !safe && !factory && <p className="dx-muted dx-sm">Device maintenance is not available on this firmware build.</p>}
      </DxCard>
      {(xmosReset || xmosFlash) && <DxCard title="XMOS Audio Control" collapsible defaultOpen={true} hint={HINTS.xmos}>
          {xmos && <DxFacts>
              <DxFact label="XMOS Firmware" value={xmos.value || '—'} hint={HINTS.xmos} />
            </DxFacts>}
          {xmosReset && <DxRow label="Restart XMOS">
              <DxConfirm label="Restart" ariaLabel="Restart XMOS" title={CONFIRM.xmos_restart.t} body={CONFIRM.xmos_restart.b} confirmLabel="Restart XMOS" onConfirm={() => post(xmosReset)} />
            </DxRow>}
          {xmosFlash && <DxRow label="Reflash XMOS firmware" hint={HINTS.xmos_flash}>
              <DxConfirm label="Reflash" ariaLabel="Reflash XMOS firmware" danger title={CONFIRM.xmos_flash.t} body={CONFIRM.xmos_flash.b} confirmLabel="Reflash now" onConfirm={() => post(xmosFlash)} />
            </DxRow>}
          <XmosFirmwareRow ctx={ctx} />
        </DxCard>}
      {radar && <DxCard title={`${radar} Radar Control`} collapsible defaultOpen={true} hint={HINTS.radar_recovery}>
          <DxFacts>
            <DxFact label="Radar Module" value={radar} />
            <DxFact label="Radar Firmware" value={radarFw?.value || '—'} />
          </DxFacts>
          <DxRow label="Restart radar">
            <DxConfirm label="Restart" ariaLabel="Restart radar" title={CONFIRM.radar_restart.t} body={CONFIRM.radar_restart.b} confirmLabel="Restart radar" onConfirm={() => post(entityPath('button/Radar Restart', 'press'))} />
          </DxRow>
          <DxRow label="Factory reset radar">
            <DxConfirm label="Factory reset" ariaLabel="Factory reset radar" danger title={CONFIRM.radar_factory.t} body={CONFIRM.radar_factory.b} confirmLabel="Reset the radar" onConfirm={() => post(entityPath('button/Radar Factory Reset', 'press'))} />
          </DxRow>
        </DxCard>}
    </div>;
}
/**
 * The organisation's four front doors: organisation links, not device links, so they read the same
 * while a peer is being controlled. Line glyphs stroked in currentColor rather than the brand
 * assets, whose own colours would clash with both themes.
 */
const COMMUNITY: {
  label: string;
  url: string;
  icon: React.ReactNode;
}[] = [{
  label: TEXT.cl_docs,
  url: 'https://docs.futureproofhomes.net/',
  icon: <><path d="M1.6 2.4h3.8a2.6 2.6 0 0 1 2.6 2.6v8.9a2 2 0 0 0-2-2H1.6z" /><path d="M14.4 2.4h-3.8A2.6 2.6 0 0 0 8 5v8.9a2 2 0 0 1 2-2h4.4z" /></>
}, {
  label: TEXT.cl_github,
  url: 'https://github.com/FutureProofHomes',
  icon: <><path d="M10.67 14.67v-2.58a2.25 2.25 0 0 0-.63-1.74c2.09-.23 4.29-1.03 4.29-4.67a3.63 3.63 0 0 0-1-2.52 3.38 3.38 0 0 0-.06-2.51s-.79-.23-2.61.99a8.92 8.92 0 0 0-4.66 0c-1.82-1.22-2.61-.99-2.61-.99a3.38 3.38 0 0 0-.06 2.51 3.63 3.63 0 0 0-1 2.52c0 3.61 2.2 4.4 4.29 4.67a2.25 2.25 0 0 0-.62 1.73v2.59" /><path d="M6 12.67c-3.33 1-3.33-1.67-4.67-2" /></>
}, {
  label: TEXT.cl_youtube,
  url: 'https://www.youtube.com/@futureproofhomes',
  icon: <><rect x="1.8" y="4" width="12.4" height="8.4" rx="2.6" /><path d="M6.9 6.6v3.2l3-1.6z" fill="currentColor" /></>
}, {
  label: TEXT.cl_discord,
  url: 'https://discord.futureproofhomes.net/',
  icon: <><path d="M10.33 11.67l.67 1.33s2.78-.89 3.67-2.33c0-.67.35-5.43-2-7-1-.67-2.67-1-2.67-1l-.67 1.33h-1.33" /><path d="M5.69 11.67l-.67 1.33s-2.78-.89-3.67-2.33c0-.67-.35-5.43 2-7 1-.67 2.67-1 2.67-1l.67 1.33h1.33" /><circle cx="5.67" cy="8.33" r="1" fill="currentColor" stroke="none" /><circle cx="10.33" cy="8.33" r="1" fill="currentColor" stroke="none" /></>
}];
function CommunityLinks() {
  return <nav className="dx-links" aria-label={TEXT.cl_aria}>
      {COMMUNITY.map(c => <a key={c.url} href={c.url} target="_blank" rel="noopener noreferrer">
          <svg viewBox="0 0 16 16" aria-hidden="true">{c.icon}</svg>
          <span>{c.label}</span>
        </a>)}
    </nav>;
}
const ROUTE_HEADLINES: Record<string, {
  prefix: string;
  em: string;
}> = {
  'device-info': {
    prefix: 'Know your ',
    em: 'hardware.'
  },
  'updates': {
    prefix: 'Stay ',
    em: 'current.'
  },
  'security': {
    prefix: 'Lock it ',
    em: 'down.'
  },
  'logs': {
    prefix: 'Read the ',
    em: 'tape.'
  },
  'integrations': {
    prefix: 'Connect ',
    em: 'everything.'
  },
  'recovery': {
    prefix: 'Back from the ',
    em: 'brink.'
  },
  'audio': {
    prefix: 'Shape the ',
    em: 'sound.'
  },
  'community': {
    prefix: "You're not ",
    em: 'alone.'
  }
};
const DEFAULT_HEADLINE = {
  prefix: 'Under the ',
  em: 'hood.'
};
export const SETTINGS_ROUTES = [{
  slug: 'device-info',
  label: 'Device Info'
}, {
  slug: 'updates',
  label: 'Updates'
}, {
  slug: 'security',
  label: 'Security'
}, {
  slug: 'logs',
  label: 'Logs'
}, {
  slug: 'integrations',
  label: 'Integrations'
}, {
  slug: 'recovery',
  label: 'Recovery'
}, {
  slug: 'audio',
  label: 'Audio'
}, {
  slug: 'community',
  label: 'Community'
}];

/** The settings page each toast intent's card lives on. */
const INTENT_PAGE: Record<string, string> = {
  log: 'logs',
  crash: 'logs',
  firmware: 'updates'
};

/**
 * Brings a toast's card into view once its page is on screen. One scroll is not enough - an owner's
 * screenshot (September 2026) caught a toast landing on the wrong card - because cards above the
 * target keep filling in after mount (the crash card waits on its own fetch). So the scroll is
 * re-pinned as the page grows, for a moment, and a card that does not exist yet is found when it
 * appears. The person outranks the pin: one wheel tick or touch-drag stands it down for good. The
 * pulse marks the card; the log card flashes its line instead, since two highlights on one card is
 * a page shouting.
 */
function useReveal(intent: any, page: string, root: React.RefObject<HTMLElement>) {
  useEffect(() => {
    const card = intent?.card;
    const host = root.current;
    if (!card || INTENT_PAGE[card] !== page || !host) return undefined;
    let userTook = false;
    let pulsed: Element | null = null;
    let unpulse: ReturnType<typeof setTimeout> | undefined;
    const pin = () => {
      if (userTook) return;
      const el = document.getElementById(`card-${card}`);
      if (!el) return;
      el.scrollIntoView({
        block: 'start',
        behavior: 'auto'
      });
      if (!pulsed && card !== 'log') {
        const p = card === 'firmware' && el.querySelector('.dx-upd-row') || el;
        p.classList.add('dx-reveal');
        pulsed = p;
        unpulse = setTimeout(() => p.classList.remove('dx-reveal'), 2600);
      }
    };
    const standDown = () => {
      userTook = true;
    };
    const mo = new MutationObserver(pin);
    mo.observe(host, {
      childList: true,
      subtree: true
    });
    host.addEventListener('wheel', standDown, {
      passive: true
    });
    host.addEventListener('touchmove', standDown, {
      passive: true
    });
    pin();
    const stop = setTimeout(() => mo.disconnect(), 2800);
    return () => {
      mo.disconnect();
      clearTimeout(stop);
      clearTimeout(unpulse);
      (pulsed as Element | null)?.classList.remove('dx-reveal');
      host.removeEventListener('wheel', standDown);
      host.removeEventListener('touchmove', standDown);
    };
  }, [intent, page]);
}
export function DiagnosticsTab({
  ctx,
  subRoute = 'device-info',
  onSubRouteChange
}: {
  ctx: Ctx;
  subRoute?: string;
  onSubRouteChange?: (v: string) => void;
}) {
  const [visible, setVisible] = useState(true);
  const [displayedRoute, setDisplayedRoute] = useState(subRoute);
  useEffect(() => {
    if (subRoute === displayedRoute) return;
    setVisible(false);
    const t = setTimeout(() => {
      setDisplayedRoute(subRoute);
      setVisible(true);
    }, 160);
    return () => clearTimeout(t);
  }, [subRoute, displayedRoute]);

  // A toast's action sets its intent and then the hash, so a new page takes it on arrival; a toast
  // pointing at the page already open dispatches toast-intent instead, because an identical hash
  // fires no hashchange and nothing remounts. Taking it on every page change also drops one that
  // went stale.
  const [intent, setIntent] = useState<any>(null);
  useEffect(() => {
    const i = takeIntent();
    setIntent(i ? {
      ...i
    } : null);
  }, [subRoute]);
  useEffect(() => {
    const on = () => {
      const i = takeIntent();
      if (i) setIntent({
        ...i
      });
    };
    window.addEventListener('toast-intent', on);
    return () => window.removeEventListener('toast-intent', on);
  }, []);
  const root = useRef<HTMLElement>(null);
  useReveal(intent, displayedRoute, root);
  const active = SETTINGS_ROUTES.find(r => r.slug === displayedRoute) ?? SETTINGS_ROUTES[0];
  const headline = ROUTE_HEADLINES[active.slug] ?? DEFAULT_HEADLINE;
  return <section className="control dx-tab" ref={root}>
      <div style={{
      opacity: visible ? 1 : 0,
      transform: visible ? 'translateY(0)' : 'translateY(6px)',
      transition: 'opacity 0.16s ease, transform 0.16s ease'
    }}>
      <span className="eyebrow">SETTINGS · {active.label.toUpperCase()}</span>
      <h1><span>{headline.prefix}</span><em>{headline.em}</em></h1>
      {active.slug === 'device-info' && <DeviceCard ctx={ctx} onUpdates={() => onSubRouteChange?.('updates')} />}
      {active.slug === 'device-info' && <ButtonsCard ctx={ctx} />}
      {active.slug === 'updates' && <FirmwareCard ctx={ctx} reveal={intent?.card === 'firmware'} />}
      {active.slug === 'security' && <AuthTokenCard ctx={ctx} />}
      {active.slug === 'security' && <ChangePasswordCard ctx={ctx} />}
      {active.slug === 'logs' && <LogsCard ctx={ctx} intent={intent?.card === 'log' ? intent : null} />}
      {active.slug === 'logs' && <CrashCard ctx={ctx} reveal={intent?.card === 'crash'} />}
      {active.slug === 'integrations' && <HomeAssistantCard ctx={ctx} />}
      {active.slug === 'recovery' && <RecoveryCards ctx={ctx} />}
      {active.slug === 'audio' && <SpeakerAmpCard ctx={ctx} />}
      {active.slug === 'community' && <CommunityLinks />}
      </div>
    </section>;
}
