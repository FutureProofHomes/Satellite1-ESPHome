import React, { useEffect, useRef, useState } from 'react';
import { HINTS, TEXT } from '../../copy.js';
import { entity, useWakeSlots } from '../../lib/device.js';
import { FLAG_IDLE, FLAG_MUTED, FLAG_XMOS, RATE } from '../../lib/micframes.js';
import { BUCKETS, micLevels, micState, micViz, noteXmosVersion, removeRecording, retarget, setChannel, startListening, startRecording, stopListening, stopRecording, recordingSeconds, subscribe } from '../../lib/micmon.js';
import { wavParts } from '../../lib/wav.js';
import { zipParts } from '../../lib/zip.js';
import type { Ctx } from '../../ctx';
import { DxCard, DxRow, DxSelect, saveBlob } from './dx';

/** Re-renders on the monitor's own notifications (status at once, levels four times a second). */
function useMic() {
  const [, setN] = useState(0);
  useEffect(() => subscribe(() => setN(n => n + 1)), []);
  return micState();
}

const cssVar = (el: Element, name: string, fallback: string) => getComputedStyle(el).getPropertyValue(name).trim() || fallback;

/** Sizes the canvas to its box at the screen's pixel ratio; returns the 2D context or null. */
function fit(c: HTMLCanvasElement) {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(c.clientWidth * dpr);
  const h = Math.round(c.clientHeight * dpr);
  if (c.width !== w || c.height !== h) {
    c.width = w;
    c.height = h;
  }
  return c.getContext('2d');
}

/** Redraws on animation frames, but only when the ring has moved. */
function useRingCanvas(draw: (c: HTMLCanvasElement, g: CanvasRenderingContext2D) => void, deps: unknown[] = []) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    let raf = 0;
    let last = -1;
    let lastW = -1;
    const tick = () => {
      const c = ref.current;
      const head = micViz().head;
      if (c && (head !== last || c.clientWidth !== lastW)) {
        const g = fit(c);
        if (g) draw(c, g);
        last = head;
        lastW = c.clientWidth;
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, deps);
  return ref;
}

/** One channel's last ten seconds: peaks as a faint envelope, RMS solid, gaps left empty. */
function Wave({ ch, label }: { ch: 0 | 1; label: string }) {
  const ref = useRingCanvas((c, g) => {
    const { head, peak, rms } = micViz();
    const w = c.width;
    const h = c.height;
    const mid = h / 2;
    const color = cssVar(c, '--orb-ink', '#a78bfa');
    g.clearRect(0, 0, w, h);
    g.fillStyle = cssVar(c, '--line', 'rgba(255,255,255,.09)');
    g.fillRect(0, Math.floor(mid), w, 1);
    const step = w / BUCKETS;
    for (let k = 0; k < BUCKETS; k++) {
      const i = (head + k) % BUCKETS;
      const p = peak[ch][i];
      if (Number.isNaN(p)) continue;
      const x = k * step;
      const ph = Math.max(1, p * mid);
      const rh = Math.max(1, rms[ch][i] * mid * 1.8);
      g.globalAlpha = 0.35;
      g.fillStyle = color;
      g.fillRect(x, mid - ph, Math.max(1, step), ph * 2);
      g.globalAlpha = 1;
      g.fillRect(x, mid - Math.min(mid, rh), Math.max(1, step), Math.min(mid, rh) * 2);
    }
  }, [ch]);
  return <canvas ref={ref} className="mm-wave" role="img" aria-label={label} />;
}

const SLOT_COLORS = ['--orb-ink', '--accent2', '--mm-stop'];

/** The three wake word scores over the same ten seconds, with each tuned cutoff dashed. */
function ScoreLane({ cutoffs }: { cutoffs: number[] }) {
  const ref = useRingCanvas((c, g) => {
    const { head, score } = micViz();
    const w = c.width;
    const h = c.height;
    g.clearRect(0, 0, w, h);
    const step = w / BUCKETS;
    const y = (v: number) => h - 2 - (v / 255) * (h - 4);
    for (let s = 0; s < 3; s++) {
      const color = cssVar(c, SLOT_COLORS[s], '#fbbf24');
      g.strokeStyle = color;
      if (cutoffs[s] > 0) {
        g.setLineDash([4, 4]);
        g.globalAlpha = 0.6;
        g.beginPath();
        g.moveTo(0, y(cutoffs[s]));
        g.lineTo(w, y(cutoffs[s]));
        g.stroke();
        g.setLineDash([]);
        g.globalAlpha = 1;
      }
      g.lineWidth = Math.max(1, window.devicePixelRatio || 1);
      g.beginPath();
      for (let k = 0; k < BUCKETS; k++) {
        const v = score[s][(head + k) % BUCKETS];
        if (k === 0) g.moveTo(0, y(v));
        else g.lineTo(k * step, y(v));
      }
      g.stroke();
    }
  }, [cutoffs.join(',')]);
  return <canvas ref={ref} className="mm-lane" role="img" aria-label={TEXT.mm_scores} />;
}

/** -60 to 0 dBFS: RMS filled, peak as a tick, the numbers beside it. */
function Meter({ ch, label }: { ch: 0 | 1; label: string }) {
  const lv = micLevels(ch);
  const pos = (db: number) => (Number.isFinite(db) ? Math.max(0, Math.min(100, (db + 60) / 60 * 100)) : 0);
  const fmt = (db: number) => (Number.isFinite(db) ? db.toFixed(1) : '-\u221e');
  return <div className="mm-meter">
      <span className="mm-meter-l">{label}</span>
      <div className="mm-meter-bar" aria-hidden="true">
        <div className="mm-meter-rms" style={{ width: `${pos(lv.rms)}%` }} />
        <div className="mm-meter-pk" style={{ left: `${pos(lv.peak)}%` }} />
      </div>
      <span className="mm-meter-v">{fmt(lv.peak)} / {fmt(lv.rms)} dBFS</span>
      {lv.clip && <span className="mm-clip">{TEXT.mm_clip}</span>}
    </div>;
}

const STATUS: Record<string, string> = {
  off: TEXT.mm_st_off,
  connecting: TEXT.mm_st_connecting,
  streaming: TEXT.mm_st_streaming,
  reconnecting: TEXT.mm_st_reconnecting,
  busy: TEXT.mm_st_busy,
  low_memory: TEXT.mm_st_low_memory,
  unsupported: TEXT.mm_st_unsupported,
  signed_out: TEXT.mm_st_signed_out
};

const clock = (s: number) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const mb = (seconds: number) => (seconds * RATE * 4 / 1e6).toFixed(1);

type Recording = ReturnType<typeof micState>['recordings'][number];

function wavFor(r: Recording, which: 'stt' | 'ww') {
  const label = which === 'stt' ? TEXT.mm_dl_speech : TEXT.mm_dl_ww;
  return wavParts(which === 'stt' ? r.stt : r.ww, {
    rate: RATE,
    markers: r.markers,
    info: {
      name: `${r.stem} ${label}`,
      device: `${r.meta.device || 'Satellite1'} (firmware ${r.meta.fw || '?'})`,
      software: `ESPHome ${r.meta.esphome || '?'}; XMOS ${r.meta.xmos || '?'}`,
      comment: which === 'stt' ? 'Speech-to-text channel, as the voice assistant receives it' : "Wake word channel, after the wake word engine's gain",
      date: r.startedAt.toISOString()
    }
  });
}

function RecordingRow({ r }: { r: Recording }) {
  const [open, setOpen] = useState(false);
  const save = (which: 'stt' | 'ww') => saveBlob(new Blob(wavFor(r, which), { type: 'audio/wav' }), `${r.stem}_${which === 'stt' ? 'speech' : 'wakeword'}.wav`);
  const both = () => saveBlob(new Blob(zipParts([
    { name: `${r.stem}_speech.wav`, parts: wavFor(r, 'stt') },
    { name: `${r.stem}_wakeword.wav`, parts: wavFor(r, 'ww') }
  ], r.startedAt), { type: 'application/zip' }), `${r.stem}.zip`);
  return <div className="mm-rec">
      <div className="mm-rec-head">
        <span className="mm-rec-t">{r.startedAt.toLocaleTimeString()}</span>
        <span className="dx-muted dx-sm">{clock(r.seconds)} · {mb(r.seconds)} MB{r.full ? ` · ${TEXT.mm_full.replace('%s', clock(r.seconds))}` : ''}</span>
      </div>
      <div className="mm-rec-btns">
        <button className="dx-btn sm" onClick={() => save('stt')}>{TEXT.mm_dl_speech}</button>
        <button className="dx-btn sm" onClick={() => save('ww')}>{TEXT.mm_dl_ww}</button>
        <button className="dx-btn sm solid" onClick={both}>{TEXT.mm_dl_both}</button>
        <button className="dx-btn sm danger" onClick={() => removeRecording(r.id)}>{TEXT.mm_delete}</button>
      </div>
      {r.markers.length > 0 && <button className="mm-rec-mk dx-link" onClick={() => setOpen(v => !v)} aria-expanded={open}>{TEXT.mm_markers} ({r.markers.length})</button>}
      {open && <ul className="mm-marks">
          {r.markers.map((m, i) => <li key={i}><span className="dx-mono">{clock(m.at / RATE)}</span> {m.text}</li>)}
        </ul>}
    </div>;
}

export function MicMonitorCard({ ctx }: { ctx: Ctx }) {
  const st = useMic();
  const d = ctx.device;
  const target = String(d?.mac || '');
  useEffect(() => retarget(target), [target]);
  const xmosV: string | undefined = entity(ctx, 'xmos_firmware')?.value;
  useEffect(() => noteXmosVersion(xmosV || ''), [xmosV]);
  const { wake } = useWakeSlots(0);
  const cutoffs = [wake?.slots?.[0]?.cut || 0, wake?.slots?.[1]?.cut || 0, wake?.stopw?.cut || 0];

  const on = st.status === 'connecting' || st.status === 'streaming' || st.status === 'reconnecting';
  const live = st.status === 'streaming';
  const notes: string[] = [];
  if (live && st.flags & FLAG_MUTED) notes.push(TEXT.mm_muted);
  if (live && st.flags & FLAG_XMOS) notes.push(TEXT.mm_xmos);
  else if (live && st.flags & FLAG_IDLE) notes.push(TEXT.mm_idle);
  const failed = ['busy', 'low_memory', 'unsupported', 'signed_out'].includes(st.status);

  return <DxCard title={TEXT.mm_title} collapsible defaultOpen={true} hint={HINTS.mic_monitor}>
      <div className="dx-row mm-top">
        <button className={`dx-btn${on ? '' : ' solid'}`} onClick={() => (on ? stopListening() : startListening(target))}>{on ? TEXT.mm_stop : TEXT.mm_listen}</button>
        <span className={`mm-status${failed ? ' err' : ''}`} role="status">
          {live && <span className="mm-live" aria-hidden="true" />}
          {[STATUS[st.status] ?? st.status, ...notes].join(' · ')}
        </span>
      </div>
      <DxRow label={TEXT.mm_play_row}>
        <DxSelect value={st.channel} options={[['stt', TEXT.mm_ch_stt], ['ww', TEXT.mm_ch_ww], ['off', TEXT.mm_ch_off]]} label={TEXT.mm_play_row} onChange={v => setChannel(v)} />
      </DxRow>
      <div className="mm-chan">
        <Meter ch={0} label={TEXT.mm_ch_stt} />
        <Wave ch={0} label={TEXT.mm_ch_stt} />
        <Meter ch={1} label={TEXT.mm_ch_ww} />
        <Wave ch={1} label={TEXT.mm_ch_ww} />
      </div>
      <div className="mm-chan">
        <div className="mm-meter">
          <span className="mm-meter-l">{TEXT.mm_scores}</span>
          <span className="mm-legend"><i className="s0" />{TEXT.mm_slot_primary} {Math.round(st.scores[0] / 2.55)}%</span>
          <span className="mm-legend"><i className="s1" />{TEXT.mm_slot_secondary} {Math.round(st.scores[1] / 2.55)}%</span>
          <span className="mm-legend"><i className="s2" />{TEXT.mm_slot_stop} {Math.round(st.scores[2] / 2.55)}%</span>
        </div>
        <ScoreLane cutoffs={cutoffs} />
        <p className="dx-muted dx-xs">{HINTS.mic_scores}</p>
      </div>
      <div className="dx-row mm-top">
        {st.recording ? <button className="dx-btn danger solid" onClick={stopRecording}>{TEXT.mm_record_stop}</button> : <button className="dx-btn" disabled={!live} onClick={() => startRecording({ device: d?.name, fw: d?.fw, esphome: d?.esphome, xmos: xmosV })}>{TEXT.mm_record}</button>}
        {st.recording && <span className="mm-status"><span className="mm-rec-dot" aria-hidden="true" />{clock(recordingSeconds())}</span>}
      </div>
      {st.recordings.length > 0 && <div className="mm-recs">
          <p className="dx-muted dx-sm mm-recs-t">{TEXT.mm_recordings} · {TEXT.mm_recording_note}</p>
          {st.recordings.map(r => <RecordingRow key={r.id} r={r} />)}
        </div>}
    </DxCard>;
}
