import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import type { ReactNode } from 'react';
import { ArrowRight, ChevronDown, Check } from '../icons';
import type { Ctx } from '../ctx';
import { HINTS, TEXT, WW_ERR } from '../../src/copy.js';
import { PIPELINE_PREFERRED, STOP_SLOT, entity, haBlocked, haSyncOnce, haTooOld, pathFor, post, requestJson, useAssist, useWakeSlots } from '../../src/lib/device.js';
import { holdPeerMutes, keepPeerMutes, releasePeerMutes } from '../../src/lib/peermute.js';
import { DEFAULT_SOURCES, REQUEST_WORD_URL, TRAIN_URL, canSpeak, enumerateSource, readSources, speak, writeSources } from '../../src/lib/wakesources.js';
import { CUT_MAX, CUT_MIN, FSD_KEYS, FSD_OPTIONS, attemptMarks, attemptsOf, clampCut, cutoffPath, fade, filterEntries, fsdShown, fsdWrites, isUrl, langLabel, langsOf, pairingWrite, parseSource, pctN, pickerEntries, placement, readout, roomMarks, rowMarks, sessionRoomMarks, swapStep } from '../lib/wake.js';
type MarkKind = 'fire' | 'near' | 'room' | 'you';
interface Mark {
  id: string;
  c: number;
  y: number;
  kind: MarkKind;
  age?: number;
  ripple?: boolean;
}
/** One track of GET /api/sat1/wakewords: a slot, or the stop word's `stopw` block. */
interface Track {
  i?: number;
  m?: string;
  w?: string;
  st?: number;
  err?: number;
  cut: number;
  ld?: number;
  tn?: number[];
  day?: number[];
  dh?: number[][];
  dl?: number;
  tot?: number;
}
interface Swap {
  phase: 'busy' | 'error';
  word: string;
  spec: string;
  err?: number;
  dl?: number;
  tot?: number;
}
interface Source {
  url: string;
  label: string;
}
interface CatEntry {
  loading?: boolean;
  error?: boolean;
  entries?: unknown[];
}
interface Entry {
  key: string;
  word: string;
  spec: string;
  langs: string[];
  source: string;
  ver?: string;
  unverified?: boolean;
}
interface Attempt {
  score: number;
  round: string;
}
interface Tuning {
  i: number;
  word: string;
  isStop: boolean;
  quick: boolean;
}
interface TunerState {
  phase: 'ready' | 'voice' | 'place' | 'nogap' | 'nocap' | 'gone';
  attempts: Attempt[];
  vadTries: number;
  skipped: boolean;
  roomReg: number;
  roomSeen: number[];
  day: number[];
  noise: number;
  floorV: number;
  hiV: number;
  cutC: number;
  nogap?: string;
}
const GRID_V = [{
  id: 'g10',
  p: 10
}, {
  id: 'g20',
  p: 20
}, {
  id: 'g25',
  p: 25,
  mj: true
}, {
  id: 'g30',
  p: 30
}, {
  id: 'g40',
  p: 40
}, {
  id: 'g50',
  p: 50,
  mj: true
}, {
  id: 'g60',
  p: 60
}, {
  id: 'g70',
  p: 70
}, {
  id: 'g75',
  p: 75,
  mj: true
}, {
  id: 'g80',
  p: 80
}, {
  id: 'g90',
  p: 90
}];
const TICKS = [{
  id: 't0',
  p: 0
}, {
  id: 't25',
  p: 25
}, {
  id: 't50',
  p: 50
}, {
  id: 't75',
  p: 75
}, {
  id: 't100',
  p: 100
}];
const CSS = `
.wake-section>*{grid-column:1/-1}
.wake-section{display:flex!important;flex-direction:column;gap:14px}
.wake-section>.eyebrow{margin:0 0 calc(4px - 14px)}
.wake-section>h1{margin:0}
.ww-card{background:linear-gradient(135deg,rgba(255,255,255,.06),rgba(255,255,255,.02));border:1px solid rgba(255,255,255,.08);border-radius:20px;backdrop-filter:blur(12px);padding:18px}
[data-theme='light'] .ww-card{background:#fff;border-color:rgba(0,0,0,.08)}
.ww-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:6px}
.ww-title{display:flex;align-items:center;gap:8px;font-size:17px;font-weight:700;letter-spacing:-.02em;margin:0}
.ww-hint{min-height:0!important;width:18px;height:18px;border-radius:50%;border:1px solid var(--muted)!important;background:transparent;color:var(--muted);font-size:10px;font-weight:700;display:inline-grid;place-items:center;padding:0;cursor:help}
.ww-slot{padding:16px 0;border-bottom:1px solid rgba(255,255,255,.06)}
.ww-slot:last-child{border-bottom:0;padding-bottom:0}
.ww-top{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:12px}
.wpill{display:inline-flex;align-items:center;gap:8px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.1)!important;border-radius:12px;padding:8px 12px;font-weight:600;font-size:14px}
.wpill.open{border-color:var(--accent2)!important}
.ww-live{display:inline-flex;align-items:center;gap:6px;font-size:12px;color:rgba(34,197,94,.9);font-weight:600}
.ww-live i{width:7px;height:7px;border-radius:50%;background:#22c55e;animation:wwPulse 1.6s ease-in-out infinite}
@keyframes wwPulse{50%{opacity:.35;box-shadow:0 0 0 5px rgba(34,197,94,.15)}}
.ww-tune{margin-left:auto;width:100%;flex:0 0 100%;margin-top:20px;background:linear-gradient(135deg,var(--accent),var(--accent2));color:#fff;border-radius:10px;padding:8px 14px;font-weight:700;font-size:13px;box-shadow:0 0 16px rgba(147,180,253,.35)}
.ww-note{font-size:12px;color:var(--muted);line-height:1.5;margin:0 0 12px}
.ww-row{display:flex;align-items:center;justify-content:space-between;gap:12px;font-size:13px;margin-top:12px}
.ww-row label{display:flex;align-items:center;gap:6px;font-size:13px;color:var(--text)}
.ww-row select{width:auto;margin:0;min-width:150px}
.ww-graph{width:100%;display:block;touch-action:none;user-select:none}
@media (min-width:640px){.ww-graph{max-width:520px}}
.lg-track{fill:rgba(255,255,255,.04)}
.lg-paper{stroke:rgba(255,255,255,.06);stroke-width:1}.lg-paper.mj{stroke:rgba(255,255,255,.10)}
.lg-fire{fill:var(--accent2)}.lg-halo-a{fill:rgba(147,180,253,.18)}
.lg-rip{fill:none;stroke:var(--accent2);stroke-width:1;animation:lgRip 2.4s ease-out infinite}
.lg-near{fill:none;stroke:var(--accent2);stroke-width:1.5}
.lg-room{fill:rgba(255,190,60,.7)}.lg-halo-w{fill:rgba(255,190,60,.12)}
.lg-rip-room{fill:none;stroke:rgba(255,190,60,.8);stroke-width:1;animation:lgRip 3s ease-out infinite}
.lg-you{fill:#60a5fa}.lg-rip-you{fill:none;stroke:#60a5fa;stroke-width:1;animation:lgRip 1.8s ease-out infinite}
.lg-tint{fill:rgba(147,180,253,.10)}.lg-veil{fill:rgba(0,0,0,.22)}
.lg-stem{stroke:var(--accent2);stroke-width:1.5;stroke-dasharray:3 3}
.lg-knob{fill:var(--accent2);cursor:ew-resize}.lg-knob.held{fill:#c7d7ff}
.lg-grip{stroke:rgba(0,0,0,.4);stroke-width:1.5;stroke-linecap:round;pointer-events:none}
.lg-heldring{fill:rgba(147,180,253,.15)}
.lg-axis{stroke:rgba(255,255,255,.15);stroke-width:1}
.lg-al{fill:rgba(255,255,255,.5);font-size:9px}
@keyframes lgRip{0%{r:8.5px;opacity:.7}100%{r:18px;opacity:0}}
.wsw{min-height:0!important;position:relative;width:78px;height:32px;border-radius:999px;background:var(--surface2);border:1px solid rgba(255,255,255,.1)!important;font-weight:700;font-size:13px;color:var(--muted);transition:background .2s}
.wsw.on{background:var(--accent);color:#fff}
.ww-bell{min-height:0!important;width:36px;height:36px;border-radius:50%;background:rgba(255,255,255,.06);display:grid;place-items:center;color:var(--text)}
.ww-bell.off{color:var(--muted)}
.ww-src{display:flex;align-items:center;gap:10px;padding:12px 0;border-bottom:1px solid rgba(255,255,255,.06);font-size:13px}
.ww-src a{flex:1;min-width:0;color:var(--accent2);text-decoration:none;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-height:0}
.ww-src small{color:var(--muted);font-size:12px;white-space:nowrap}
.ww-x{background:transparent;color:var(--muted);font-size:12px;padding:4px 8px;min-height:32px}
.ww-add{display:flex;gap:8px;margin-top:14px}.ww-add input{flex:1;min-width:0;padding:10px 12px}
.ww-confirm{background:rgba(255,255,255,.04);border-radius:12px;padding:12px;margin-top:10px;font-size:12px;color:var(--muted);line-height:1.5}
.ww-confirm div{display:flex;gap:8px;margin-top:10px}
.ww-over{position:fixed;inset:0;z-index:200;background:rgba(0,0,0,.75);animation:wwFade .25s ease}
.ww-scrim{position:fixed;inset:0;z-index:200;background:rgba(0,0,0,.55);animation:wwFade .25s ease;will-change:backdrop-filter;transform:translateZ(0)}
.ww-panel{background:var(--surface);border-radius:24px 24px 0 0;position:fixed;z-index:201;bottom:0;left:0;right:0;max-height:92dvh;overflow-y:auto;padding:24px;animation:wwUp .32s cubic-bezier(.2,.8,.2,1);max-width:640px;margin:0 auto;color:var(--text)}
@keyframes wwFade{from{opacity:0}}@keyframes wwUp{from{transform:translateY(40px);opacity:0}}
.ww-ptop{display:flex;align-items:center;justify-content:space-between;margin-bottom:18px}
.ww-ptop h2{margin:0;font-size:24px;letter-spacing:-.03em}
.ww-filter{display:flex;gap:8px;margin-bottom:16px}
.ww-search{flex:1;min-width:0;padding:12px 14px;font-size:15px!important;border-radius:12px!important}
.ww-filter select{width:auto;margin:0;border-radius:12px}
.ww-grp{width:100%;display:flex;align-items:center;gap:8px;background:transparent;font-size:12px;color:var(--muted);letter-spacing:.04em;padding:10px 0;font-weight:600;text-align:left}
.ww-grp svg{transition:transform .2s}.ww-grp.shut svg{transform:rotate(-90deg)}
.ww-ent{display:flex;align-items:center;gap:12px;padding:10px 4px;border-bottom:1px solid rgba(255,255,255,.05)}
.ww-pick{flex:1;display:flex;align-items:center;gap:12px;background:transparent;text-align:left;padding:0;min-width:0}
.ww-pick:disabled{opacity:.4;cursor:not-allowed}
.ww-chk{width:20px;height:20px;border-radius:6px;border:1.5px solid rgba(255,255,255,.2);display:grid;place-items:center;flex-shrink:0}
.ww-chk.on{background:var(--accent2);border-color:var(--accent2);color:#0d0d0f}
.ww-ent b{font-size:14px;font-weight:600}
.ww-tag{font-size:10px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);background:rgba(255,255,255,.06);padding:3px 6px;border-radius:6px}
.ww-tag.warn{color:#ffc45e}
.ww-speak{min-height:36px;width:36px;border-radius:50%;background:rgba(255,255,255,.06);display:grid;place-items:center;padding:0}
.ww-foot{font-size:12px;color:var(--muted);margin:18px 0 0}
.ww-read{font-size:13px;line-height:1.5;margin:14px 0 18px}.ww-read.warn{color:#ffb35e}.ww-read.err{color:#ff7185}.ww-read.dim{color:var(--muted)}
.ww-acts{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.ww-acts button{padding:10px 6px;font-size:12px}
.ww-btns{display:flex;gap:8px;margin-top:18px}.ww-btns button{flex:1}
.ww-dd{position:relative;display:inline-block}
.ww-dd-btn{min-height:40px!important;background:rgba(255,255,255,.07);border:1px solid rgba(255,255,255,.13)!important;border-radius:10px;padding:8px 12px;font-size:13px;font-weight:500;display:flex;align-items:center;justify-content:space-between;gap:8px;min-width:160px;color:var(--text);cursor:pointer;transition:background .15s,border-color .15s,box-shadow .15s}
.ww-dd-btn:hover{border-color:rgba(255,255,255,.25)!important;background:rgba(255,255,255,.10)}
.ww-dd-btn.open,.ww-dd-btn:focus-visible{border-color:var(--accent2)!important;box-shadow:0 0 0 3px rgba(147,180,253,.15);outline:none}
.ww-dd-btn svg{stroke:var(--muted);transition:transform .2s;flex-shrink:0}.ww-dd-btn.open svg{transform:rotate(180deg)}
.ww-dd-pop{position:absolute;top:calc(100% + 6px);right:0;min-width:100%;background:var(--surface2);border:1px solid rgba(255,255,255,.10);border-radius:12px;box-shadow:0 8px 32px rgba(0,0,0,.4);z-index:50;overflow:hidden;padding:4px;animation:wwUp .2s ease;list-style:none;margin:0}
.ww-dd-opt{min-height:0!important;width:100%;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 14px;border-radius:8px;font-size:13px;background:transparent;color:var(--text);text-align:left;white-space:nowrap}
.ww-dd-opt:hover,.ww-dd-opt:focus-visible{background:rgba(255,255,255,.08);outline:none}
.ww-dd-opt.on{font-weight:600}.ww-dd-opt svg{color:var(--accent2)}
[data-theme='light'] .ww-dd-btn{background:rgba(0,0,0,.04);border-color:rgba(0,0,0,.12)!important;color:#111113}
[data-theme='light'] .ww-dd-btn:hover{background:rgba(0,0,0,.07);border-color:rgba(0,0,0,.20)!important}
[data-theme='light'] .ww-dd-btn.open{border-color:var(--accent2)!important}
[data-theme='light'] .ww-dd-pop{background:#fff;border-color:rgba(0,0,0,.10);box-shadow:0 8px 28px rgba(0,0,0,.14)}
[data-theme='light'] .ww-dd-opt{color:#111113}
[data-theme='light'] .ww-dd-opt:hover,[data-theme='light'] .ww-dd-opt:focus-visible{background:rgba(0,0,0,.05)}
[data-theme='light'] .ww-card{background:#fff;border-color:rgba(0,0,0,.10);box-shadow:0 1px 4px rgba(0,0,0,.07)}
[data-theme='light'] .ww-slot{border-bottom-color:rgba(0,0,0,.07)}
[data-theme='light'] .ww-top{color:#111113}
[data-theme='light'] .ww-hint{border-color:#9a9bab!important;color:#6b6b78}
[data-theme='light'] .wpill{background:rgba(0,0,0,.04);border-color:rgba(0,0,0,.12)!important;color:#111113}
[data-theme='light'] .wpill.open{border-color:var(--accent2)!important}
[data-theme='light'] .ww-row label{color:#111113}
[data-theme='light'] .ww-note{color:#6b6b78}
[data-theme='light'] .lg-track{fill:rgba(0,0,0,.04)}
[data-theme='light'] .lg-paper{stroke:rgba(0,0,0,.10)}
[data-theme='light'] .lg-paper.mj{stroke:rgba(0,0,0,.18)}
[data-theme='light'] .lg-veil{fill:rgba(0,0,0,.06)}
[data-theme='light'] .lg-tint{fill:rgba(37,99,235,.08)}
[data-theme='light'] .lg-axis{stroke:rgba(0,0,0,.20)}
[data-theme='light'] .lg-al{fill:rgba(0,0,0,.5)}
[data-theme='light'] .lg-halo-a{fill:rgba(29,78,216,.12)}
[data-theme='light'] .lg-halo-w{fill:rgba(180,120,0,.12)}
[data-theme='light'] .lg-fire{fill:var(--accent2)}
[data-theme='light'] .lg-near{stroke:var(--accent2)}
[data-theme='light'] .lg-rip{stroke:var(--accent2)}
[data-theme='light'] .lg-room{fill:rgba(180,110,0,.75)}
[data-theme='light'] .lg-rip-room{stroke:rgba(160,100,0,.7)}
[data-theme='light'] .lg-stem{stroke:var(--accent2)}
[data-theme='light'] .lg-knob{fill:var(--accent2)}
[data-theme='light'] .lg-knob.held{fill:var(--accent)}
[data-theme='light'] .lg-grip{stroke:rgba(255,255,255,.6)}
[data-theme='light'] .lg-heldring{fill:rgba(37,99,235,.12)}
[data-theme='light'] .ww-src{border-bottom-color:rgba(0,0,0,.07)}
[data-theme='light'] .ww-ent{border-bottom-color:rgba(0,0,0,.07)}
[data-theme='light'] .ww-foot{color:#6b6b78}
[data-theme='light'] .ww-confirm{background:rgba(0,0,0,.04);color:#6b6b78}
[data-theme='light'] .ww-read.dim{color:#6b6b78}
[data-theme='light'] .ww-tag{background:rgba(0,0,0,.06);color:#6b6b78}
[data-theme='light'] .ww-grp{color:#6b6b78}
[data-theme='light'] .ww-chk{border-color:rgba(0,0,0,.25)}
[data-theme='light'] .ww-chk.on{background:var(--accent2);border-color:var(--accent2);color:#fff}
[data-theme='light'] .ww-speak{background:rgba(0,0,0,.06)}
[data-theme='light'] .ww-bell{background:rgba(0,0,0,.05);color:#111113}
[data-theme='light'] .ww-bell.off{color:#9a9bab}
[data-theme='light'] .ww-x{color:#6b6b78}
[data-theme='light'] .wsw{background:#e9e9ef;border-color:rgba(0,0,0,.12)!important;color:#6b6b78}
[data-theme='light'] .wsw.on{background:var(--accent);color:#fff}
[data-theme='light'] .ww-panel{background:#fff;color:#111113}
[data-theme='light'] .ww-over{background:rgba(0,0,0,.45)}
[data-theme='light'] .ww-scrim{background:rgba(0,0,0,.35)}
[data-theme='light'] .ww-search{background:#f0f0f4;color:#111113;border-color:rgba(0,0,0,.12)}
.ww-hint-wrap{display:inline-flex;align-items:center}
.ww-hint-btn{flex:none;width:18px;height:18px;min-height:0!important;min-width:0!important;border-radius:9999px;border:1px solid rgba(255,255,255,0.25);background:transparent;color:rgba(255,255,255,0.45);font:italic 600 11px/1 serif;cursor:pointer;padding:0;display:inline-flex;align-items:center;justify-content:center;transition:border-color .15s,color .15s}
.ww-hint-btn[aria-expanded="true"]{border-color:var(--accent2,#93b4fd);color:var(--accent2,#93b4fd)}
.ww-hint-btn:hover{border-color:rgba(255,255,255,0.45);color:rgba(255,255,255,0.7)}
[data-theme='light'] .ww-hint-btn{border-color:rgba(0,0,0,0.25);color:rgba(0,0,0,0.45)}
[data-theme='light'] .ww-hint-btn:hover{border-color:rgba(0,0,0,0.45);color:rgba(0,0,0,0.7)}
[data-theme='light'] .ww-hint-btn[aria-expanded="true"]{border-color:var(--accent2,#2563eb);color:var(--accent2,#2563eb)}
.ww-hint-bubble{position:fixed;z-index:9999;max-width:min(280px,calc(100vw - 16px));padding:10px 14px;background:#1e1e26;border:1px solid rgba(255,255,255,0.12);border-radius:12px;box-shadow:0 8px 32px rgba(0,0,0,0.5);font-size:13px;line-height:1.5;font-weight:400;letter-spacing:0;color:rgba(255,255,255,0.85);pointer-events:auto;text-align:left}
[data-theme='light'] .ww-hint-bubble{background:#fff;border-color:rgba(0,0,0,0.10);box-shadow:0 4px 20px rgba(0,0,0,0.15);color:#111113}
[data-theme='light'] .lg-confidence{fill:rgba(0,0,0,0.35)!important}
.ww-tree{background:rgba(255,255,255,0.025);border:1px solid rgba(255,255,255,0.07);border-radius:14px;overflow:hidden;margin:12px 0}
[data-theme='light'] .ww-tree{background:rgba(0,0,0,0.025);border-color:rgba(0,0,0,0.08)}
.ww-tree-a{border-bottom:1px solid rgba(255,255,255,0.05)}.ww-tree-a:last-child{border-bottom:none}
[data-theme='light'] .ww-tree-a{border-bottom-color:rgba(0,0,0,0.05)}
.ww-tree-h{display:flex;align-items:center;gap:10px;padding:10px 14px;cursor:pointer;background:transparent;transition:background .15s;user-select:none}
.ww-tree-h:hover{background:rgba(255,255,255,0.04)}
[data-theme='light'] .ww-tree-h{background:transparent}[data-theme='light'] .ww-tree-h:hover{background:rgba(0,0,0,0.03)}
.ww-tree-caret{color:var(--muted);display:flex;align-items:center;flex-shrink:0;transition:transform .2s ease;width:16px}
.ww-tree-label{flex:1;font-size:13px;font-weight:600;color:var(--text);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ww-tree-count{font-size:11px;color:var(--muted);white-space:nowrap}
.ww-tree-ps{padding:0 0 4px 32px}
.ww-tree-ent{display:flex;align-items:center;gap:9px;padding:9px 14px 9px 0;border-top:1px solid rgba(255,255,255,0.04);transition:background .12s}
.ww-tree-ent:hover{background:rgba(255,255,255,0.04)}.ww-tree-ent.selected{background:rgba(147,180,253,0.08)}.ww-tree-ent-other{opacity:.45}
[data-theme='light'] .ww-tree-ent{border-top-color:rgba(0,0,0,0.04)}[data-theme='light'] .ww-tree-ent:hover{background:rgba(0,0,0,0.03)}[data-theme='light'] .ww-tree-ent.selected{background:rgba(37,99,235,0.06)}
.ww-tree-word{flex:1;font-size:13px;font-weight:500;color:var(--text);background:transparent;border:none;text-align:left;cursor:pointer;padding:0}
.ww-tree-word:disabled{cursor:default}
.ww-tree-lang{font-size:11px;color:var(--muted);font-weight:600;letter-spacing:.04em;text-transform:uppercase;white-space:nowrap}
.ww-tree-size{font-size:11px;color:var(--muted);white-space:nowrap}
.ww-tree-unv,.ww-tree-other{font-size:10px;padding:2px 7px;border-radius:999px;background:rgba(251,191,36,0.15);color:rgba(251,191,36,0.9);white-space:nowrap}
.ww-tree-other{background:rgba(255,255,255,0.07);color:var(--muted)}
[data-theme='light'] .ww-tree-unv{background:rgba(180,110,0,0.12);color:rgba(160,90,0,0.9)}[data-theme='light'] .ww-tree-other{background:rgba(0,0,0,0.06)}
.ww-chk2{width:18px;height:18px;min-height:18px!important;border-radius:5px;flex-shrink:0;border:1.5px solid rgba(255,255,255,0.22);background:rgba(255,255,255,0.05);display:inline-flex;align-items:center;justify-content:center;cursor:pointer;padding:0;transition:background .15s,border-color .15s}
.ww-chk2:disabled{opacity:.4;cursor:default}
.ww-chk2-on{background:var(--accent);border-color:var(--accent)}
.ww-chk2-mixed{background:rgba(147,180,253,0.2);border-color:var(--accent2)}
.ww-chk2-dash{width:8px;height:2px;background:var(--accent2);border-radius:1px}
[data-theme='light'] .ww-chk2{border-color:rgba(0,0,0,0.22);background:rgba(0,0,0,0.04)}
[data-theme='light'] .ww-chk2-on{background:var(--accent);border-color:var(--accent)}
[data-theme='light'] .ww-chk2-mixed{background:rgba(37,99,235,0.12);border-color:var(--accent)}
.ww-flow{display:flex;flex-direction:row;align-items:flex-end;gap:8px;width:100%;margin:14px 0 4px}
.ww-flow-cell{flex:1;min-width:0;display:flex;flex-direction:column;gap:6px}
.ww-flow-cap{font-size:11px;font-weight:600;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
.ww-flow-btn{width:100%;min-height:44px;display:flex;align-items:center;justify-content:space-between;gap:8px;padding:10px 14px;border-radius:999px;background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.12)!important;font-size:14px;font-weight:600;color:var(--text);text-align:left}
.ww-flow-btn>span{flex:1;min-width:0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ww-flow-btn svg{flex-shrink:0;color:var(--muted)}
.ww-flow-btn.open,.ww-flow-btn:hover{border-color:var(--accent2)!important}
.ww-flow-arrow{flex:0 0 24px;width:24px;height:44px;display:grid;place-items:center;color:var(--muted)}
[data-theme='light'] .ww-flow-btn{background:rgba(0,0,0,.04);border-color:rgba(0,0,0,.12)!important;color:#111113}
.ww-sec{margin:22px 0 4px;font-size:11px;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}
.ww-sec:first-of-type{margin-top:0}
.ww-sec-gap{margin-top:28px;padding-top:22px;border-top:1px solid rgba(255,255,255,.08)}
[data-theme='light'] .ww-sec-gap{border-top-color:rgba(0,0,0,.08)}
.ww-opts{display:flex;flex-direction:column}
.ww-opt{width:100%;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:14px 2px;background:transparent;border-bottom:1px solid rgba(255,255,255,.07)!important;font-size:15px;font-weight:500;color:var(--text);text-align:left}
.ww-opt:last-child{border-bottom:0!important}
.ww-opt:disabled{opacity:.4;cursor:not-allowed}
[data-theme='light'] .ww-opt{border-bottom-color:rgba(0,0,0,.07)!important;color:#111113}
.ww-mark{width:22px;height:22px;border-radius:50%;border:1.5px solid rgba(128,128,140,.45);display:grid;place-items:center;flex-shrink:0;color:#fff}
.ww-mark.sq{border-radius:6px}
.ww-opt.on .ww-mark{background:var(--accent);border-color:var(--accent)}
.ww-panel{display:flex;flex-direction:column;overflow-y:hidden}
.ww-picker-list{flex:1;min-height:0;overflow-y:auto;padding-bottom:16px}
.ww-filter{position:sticky;top:0;background:var(--surface);z-index:2;padding-bottom:12px;margin-bottom:0}
[data-theme='light'] .ww-filter{background:#fff}
.ww-opt-label{display:flex;flex-direction:column;gap:2px;flex:1;text-align:left;min-width:0}
.ww-opt-word{font-size:15px;font-weight:500;color:var(--text)}
.ww-opt-source{font-size:11px;color:var(--muted);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:100%}
[data-theme='light'] .ww-opt-source{color:#6b6b78}
.ww-opt .ww-speak{flex-shrink:0;cursor:pointer}
`;
const sleep = (ms: number) => new Promise(r => setTimeout(r, ms));
/** The stop model reports its phrase lowercase; it reads capitalized everywhere. */
const showWord = (w: string) => w === 'stop' ? 'Stop' : w;
const kb = (n: number) => Math.round(n / 1024);
const SpeakIcon = () => <svg width="13" height="13" viewBox="0 0 13 13" fill="none"><path d="M2 5v3h2l3 3V2L4 5H2z" fill="currentColor" /><path d="M9 4.5a3 3 0 010 4M10.5 3a5 5 0 010 7" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" fill="none" /></svg>;
interface DdOption {
  id: string;
  label: string;
}
function Dropdown({
  value,
  options,
  onChange,
  label
}: {
  value: string;
  options: DdOption[];
  onChange: (v: string) => void;
  label: string;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const h = (e: PointerEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    const k = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.stopPropagation();
        setOpen(false);
      }
    };
    document.addEventListener('pointerdown', h);
    document.addEventListener('keydown', k);
    return () => {
      document.removeEventListener('pointerdown', h);
      document.removeEventListener('keydown', k);
    };
  }, [open]);
  const cur = options.find(o => o.id === value);
  return <div className="ww-dd" ref={ref}>
    <button type="button" className={open ? 'ww-dd-btn open' : 'ww-dd-btn'} aria-haspopup="listbox" aria-expanded={open} aria-label={label} onClick={() => setOpen(!open)}><span>{cur?.label ?? value}</span><svg width="14" height="14" viewBox="0 0 16 16" fill="none" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="m4 6 4 4 4-4" /></svg></button>
    {open && <ul className="ww-dd-pop" role="listbox" aria-label={label}>{options.map(o => <li key={o.id}><button type="button" role="option" aria-selected={o.id === value} className={o.id === value ? 'ww-dd-opt on' : 'ww-dd-opt'} onClick={() => {
          onChange(o.id);
          setOpen(false);
        }}><span>{o.label}</span>{o.id === value && <svg width="14" height="14" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3.2 8.6 6.4 11.8 12.8 4.8" /></svg>}</button></li>)}</ul>}
  </div>;
}
function renderMark(m: Mark, x: (c: number) => number, plotH: number) {
  const mx = x(m.c);
  const my = Math.max(8, Math.min(m.y, plotH - 8));
  if (m.kind === 'fire') return <g key={m.id} opacity={fade(m.age)}><circle className="lg-halo-a" cx={mx} cy={my} r="9" />{m.ripple && <circle className="lg-rip" cx={mx} cy={my} r="8.5" />}<circle className="lg-fire" cx={mx} cy={my} r="5" /></g>;
  if (m.kind === 'near') return <circle key={m.id} className="lg-near" cx={mx} cy={my} r="3.5" opacity={fade(m.age)} />;
  if (m.kind === 'room') return <g key={m.id} opacity={fade(m.age) * 0.85}><circle className="lg-halo-w" cx={mx} cy={my} r="7" />{m.ripple && <circle className="lg-rip-room" cx={mx} cy={my} r="8.5" />}<circle className="lg-room" cx={mx} cy={my} r="3.5" /></g>;
  return <g key={m.id}><circle className="lg-rip-you" cx={mx} cy={my} r="8.5" /><circle className="lg-you" cx={mx} cy={my} r="5" /></g>;
}
const KEY_STEP: Record<string, number> = {
  ArrowLeft: -1,
  ArrowDown: -1,
  ArrowRight: 1,
  ArrowUp: 1,
  PageDown: -5,
  PageUp: 5
};
/**
 * The Living Graph. With `onCut` the knob drags and the graph is a slider; with only `onTune` the
 * knob is a tap target and the graph a button. A press moves the knob only once it has travelled
 * past a 4px slop, so a tap never nudges the line.
 */
function TouchGraph({
  gid,
  marks,
  cut,
  onCut,
  onTune,
  h = 120,
  aria
}: {
  gid: string;
  marks: Mark[];
  cut?: number;
  onCut?: (v: number) => void;
  onTune?: () => void;
  h?: number;
  aria: string;
}) {
  const W = 320;
  const ref = useRef<SVGSVGElement>(null);
  const [held, setHeld] = useState(false);
  const press = useRef<{
    x: number;
    y: number;
    moved: boolean;
  } | null>(null);
  const x = (c: number) => c / 100 * W;
  const plotH = h - 16;
  const rows = [1, 2, 3].map(i => ({
    id: `h${i}`,
    y: Math.round(plotH * i / 4)
  }));
  const setFrom = (clientX: number) => {
    const r = ref.current?.getBoundingClientRect();
    if (!r || !onCut) return;
    onCut(clampCut(Math.round((clientX - r.left) / r.width * 100)));
  };
  const release = () => {
    press.current = null;
    setHeld(false);
  };
  const onKeyDown = (e: KeyboardEvent) => {
    if (onCut && cut !== undefined) {
      const next = e.key === 'Home' ? CUT_MIN : e.key === 'End' ? CUT_MAX : KEY_STEP[e.key] ? cut + KEY_STEP[e.key] : null;
      if (next === null) return;
      e.preventDefault();
      onCut(clampCut(next));
    } else if (onTune && (e.key === 'Enter' || e.key === ' ')) {
      e.preventDefault();
      onTune();
    }
  };
  const cx = cut !== undefined ? x(cut) : 0;
  const a11y = onCut && cut !== undefined ? {
    role: 'slider',
    tabIndex: 0,
    'aria-valuemin': CUT_MIN,
    'aria-valuemax': CUT_MAX,
    'aria-valuenow': cut,
    'aria-valuetext': `${cut}%`,
    onKeyDown
  } : onTune ? {
    role: 'button',
    tabIndex: 0,
    onKeyDown
  } : {
    role: 'img'
  };
  return <svg ref={ref} className={onCut ? 'ww-graph drag' : 'ww-graph'} viewBox={`0 0 ${W} ${h}`} aria-label={aria} {...a11y} onPointerMove={e => {
    const p = press.current;
    if (!p) return;
    if (!p.moved && Math.abs(e.clientX - p.x) + Math.abs(e.clientY - p.y) < 4) return;
    p.moved = true;
    setFrom(e.clientX);
  }} onPointerUp={() => {
    if (press.current && !press.current.moved) onTune?.();
    release();
  }} onPointerCancel={release}>
    <defs>
      <filter id={`${gid}-frost`} x="-30%" y="-30%" width="160%" height="160%" colorInterpolationFilters="sRGB">
        <feGaussianBlur in="SourceGraphic" stdDeviation="6" />
      </filter>
      <clipPath id={`${gid}-veil`}><rect x="0" y="0" width={cx} height={plotH} /></clipPath>
      <clipPath id={`${gid}-clear`}><rect x={cx} y="0" width={W - cx} height={plotH} /></clipPath>
    </defs>
    <rect className="lg-track" x="0" y="0" width={W} height={plotH} rx="7" />
    {GRID_V.map(g => <line key={g.id} className={g.mj ? 'lg-paper mj' : 'lg-paper'} x1={x(g.p)} x2={x(g.p)} y1="0" y2={plotH} />)}
    {rows.map(r => <line key={r.id} className="lg-paper" x1="0" x2={W} y1={r.y} y2={r.y} />)}
    {cut !== undefined && <rect className="lg-veil" x="0" y="0" width={cx} height={plotH} rx="7" />}
    {cut !== undefined && <rect className="lg-tint" x={cx} y="0" width={W - cx} height={plotH} />}
    {cut !== undefined && <g clipPath={`url(#${gid}-veil)`} filter={`url(#${gid}-frost)`}>{marks.map(m => renderMark(m, x, plotH))}</g>}
    <g clipPath={cut !== undefined ? `url(#${gid}-clear)` : undefined}>{marks.map(m => renderMark(m, x, plotH))}</g>
    <line className="lg-axis" x1="0" x2={W} y1={plotH} y2={plotH} />
    {TICKS.map(t => <g key={t.id}><line className="lg-axis" x1={x(t.p)} x2={x(t.p)} y1={plotH} y2={plotH + 4} /><text className="lg-al" x={x(t.p)} y={h - 1} textAnchor={t.p === 0 ? 'start' : t.p === 100 ? 'end' : 'middle'}>{t.p}%</text></g>)}
    {cut !== undefined && <g>
      <line className="lg-stem" x1={cx} x2={cx} y1="0" y2={plotH} />
      {held && <circle className="lg-heldring" cx={cx} cy={plotH / 2} r="18" />}
      <rect className={held ? 'lg-knob held' : 'lg-knob'} x={cx - 7} y={plotH / 2 - 14} width="14" height="28" rx="4.5" />
      <line className="lg-grip" x1={cx - 2} x2={cx - 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
      <line className="lg-grip" x1={cx + 2} x2={cx + 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
      {(onCut || onTune) && <rect className="lg-hit" x={cx - 18} y="0" width="36" height={plotH} onPointerDown={e => {
        (e.currentTarget.ownerSVGElement as SVGSVGElement).setPointerCapture(e.pointerId);
        press.current = {
          x: e.clientX,
          y: e.clientY,
          moved: false
        };
        setHeld(true);
      }} />}
    </g>}
    {cut !== undefined && <text className="lg-al lg-confidence" x="8" y={plotH - 8} textAnchor="start" style={{
      fontSize: '10px',
      fill: 'rgba(255,255,255,0.4)',
      fontStyle: 'italic'
    }}>Confidence</text>}
  </svg>;
}
let _openHintSetter: ((v: boolean) => void) | null = null;
function HintBtn({
  text
}: {
  text: ReactNode;
}) {
  const [open, setOpen] = useState(false);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState<{
    left: number;
    top: number;
  } | null>(null);
  useEffect(() => {
    if (!open) return;
    if (_openHintSetter && _openHintSetter !== setOpen) _openHintSetter(false);
    _openHintSetter = setOpen;
    const dismiss = (e: PointerEvent) => {
      if (!btn.current?.contains(e.target as Node) && !bubble.current?.contains(e.target as Node)) setOpen(false);
    };
    const esc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.stopPropagation();
        setOpen(false);
      }
    };
    document.addEventListener('pointerdown', dismiss, true);
    document.addEventListener('keydown', esc);
    return () => {
      document.removeEventListener('pointerdown', dismiss, true);
      document.removeEventListener('keydown', esc);
      if (_openHintSetter === setOpen) _openHintSetter = null;
    };
  }, [open]);
  useLayoutEffect(() => {
    if (!open) {
      setPos(null);
      return;
    }
    const id = requestAnimationFrame(() => {
      if (!btn.current || !bubble.current) return;
      const t = btn.current.getBoundingClientRect();
      const b = bubble.current.getBoundingClientRect();
      const GAP = 6;
      const MARGIN = 8;
      let left = t.left + t.width / 2 - b.width / 2;
      left = Math.max(MARGIN, Math.min(left, window.innerWidth - b.width - MARGIN));
      const below = t.bottom + GAP;
      const top = below + b.height + MARGIN > window.innerHeight ? Math.max(MARGIN, t.top - b.height - GAP) : below;
      setPos({
        left,
        top
      });
    });
    return () => cancelAnimationFrame(id);
  }, [open]);
  return <span className="ww-hint-wrap">
    <button ref={btn} type="button" className="ww-hint-btn" aria-label="More information" aria-expanded={open} onClick={() => setOpen(v => !v)}>i</button>
    {open && createPortal(<div ref={bubble} className="ww-hint-bubble" role="tooltip" style={pos ? {
      left: pos.left,
      top: pos.top,
      opacity: 1
    } : {
      left: -9999,
      top: -9999,
      opacity: 0
    }}>{text}</div>, document.body)}
  </span>;
}
/** A drawer's page duties: the page behind blurs, focus moves into the panel and back out after,
 *  and Escape closes it. Popups inside stop their own Escape before it reaches the window. */
function useDrawer(onClose: () => void) {
  const panel = useRef<HTMLDivElement>(null);
  const close = useRef(onClose);
  close.current = onClose;
  useEffect(() => {
    const back = document.activeElement as HTMLElement | null;
    document.body.classList.add('has-drawer');
    panel.current?.focus({
      preventScroll: true
    });
    const esc = (e: KeyboardEvent) => {
      if (e.key === 'Escape') close.current();
    };
    window.addEventListener('keydown', esc);
    return () => {
      document.body.classList.remove('has-drawer');
      window.removeEventListener('keydown', esc);
      back?.focus?.({
        preventScroll: true
      });
    };
  }, []);
  return panel;
}
const toPlace = (s: TunerState, attempts: Attempt[], vadTries: number): TunerState => {
  const p = placement(attempts, s.day, s.roomReg);
  if (p.nogap) return {
    ...s,
    attempts,
    vadTries,
    phase: 'nogap',
    nogap: p.nogap
  };
  return {
    ...s,
    attempts,
    vadTries,
    phase: 'place',
    noise: p.noise,
    floorV: p.floorV,
    hiV: p.hiV,
    cutC: p.cutC
  };
};
/**
 * The tuner: Start opens a device session (the model floored, peers muted), the voice rounds land
 * as blue dots, then placement seeds the knob inside the gap for Apply. `quick` is the knob-tap
 * path: placement straight over the stored stats, no session.
 */
function Tuner({
  ctx,
  i,
  word,
  isStop,
  quick,
  seed,
  track,
  wakeRead,
  onClose
}: {
  ctx: Ctx;
  i: number;
  word: string;
  isStop: boolean;
  quick: boolean;
  seed: {
    cut: number;
    noise: number;
    floor: number;
    hi: number;
    day: number[];
  };
  track: Track | null;
  wakeRead: () => Promise<any>;
  onClose: () => void;
}) {
  const [st, setSt] = useState<TunerState>(() => ({
    phase: quick ? 'place' : 'ready',
    attempts: [],
    vadTries: 0,
    skipped: false,
    roomReg: 0,
    roomSeen: [],
    day: seed.day,
    noise: seed.noise,
    floorV: quick ? seed.floor : 0,
    hiV: quick ? seed.hi : 0,
    cutC: quick ? clampCut(pctN(seed.cut || 130)) : 55
  }));
  const heldRef = useRef<any[]>([]);
  const [pm, setPm] = useState<{
    muted: string[];
    failed: string[];
    unknown: boolean;
  } | null>(null);
  const releasePeers = () => {
    if (heldRef.current.length) releasePeerMutes(heldRef.current);
    heldRef.current = [];
  };
  const panel = useDrawer(onClose);
  // Peers self-heal on a 60s TTL, so every release here is best effort.
  const openRef = useRef(false);
  const mounted = useRef(true);
  useEffect(() => {
    const ka = setInterval(() => {
      if (openRef.current) {
        post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => {});
        keepPeerMutes(heldRef.current);
      }
    }, 20000);
    return () => {
      mounted.current = false;
      clearInterval(ka);
      if (openRef.current) post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
      openRef.current = false;
      releasePeers();
    };
  }, [i]);
  const start = async () => {
    const r = await post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => null);
    if (!mounted.current) return;
    if (!r || !r.ok) {
      setSt(s => ({
        ...s,
        phase: 'gone'
      }));
      return;
    }
    let cap = 1;
    try {
      cap = JSON.parse(r.text).cap ?? 1;
    } catch {
      /* firmware without `cap`: assume able */
    }
    openRef.current = true;
    if (!heldRef.current.length) {
      holdPeerMutes(ctx.ha, ctx.device?.mac).then((res: any) => {
        if (!mounted.current) {
          if (res.held.length) releasePeerMutes(res.held);
          return;
        }
        heldRef.current = res.held;
        if (res.held.length || res.failed.length || res.unknown) setPm({
          muted: res.held.map((p: any) => p.name),
          failed: res.failed,
          unknown: res.unknown
        });
      });
    }
    setSt(s => ({
      ...s,
      phase: cap === 0 ? 'nocap' : 'voice',
      attempts: [],
      vadTries: 0,
      skipped: false
    }));
  };
  useEffect(() => {
    if (st.phase === 'gone' || st.phase === 'nocap') releasePeers();
  }, [st.phase]);
  useEffect(() => {
    let live = true;
    const tick = async () => {
      if (!live) return;
      const d = await wakeRead();
      if (!live) return;
      const day = (i === STOP_SLOT ? d?.stopw?.day : d?.slots?.find((x: Track) => x.i === i)?.day) || null;
      const tune = d?.tune && d.tune.i === i ? d.tune : null;
      const ev = tune ? tune.ev || [] : null;
      const reg = tune ? tune.room || 0 : 0;
      setSt(s => {
        let next = s;
        if (day) next = {
          ...next,
          day
        };
        // A higher register reading lands as a new dot; earlier ones never move.
        if (reg > next.roomReg && (next.phase === 'voice' || next.phase === 'place')) next = {
          ...next,
          roomReg: reg,
          roomSeen: [...next.roomSeen, reg]
        };
        if (next.phase !== 'voice') return next;
        if (ev === null) return {
          ...next,
          phase: 'gone'
        };
        const {
          attempts,
          vadTries
        } = attemptsOf(ev);
        const enough = attempts.length >= 4 || attempts.length >= 3 && next.skipped;
        return enough ? toPlace(next, attempts, vadTries) : {
          ...next,
          attempts,
          vadTries
        };
      });
      setTimeout(tick, 700);
    };
    tick();
    return () => {
      live = false;
    };
  }, [i]);
  const skip = () => setSt(s => s.attempts.length >= 3 ? toPlace(s, s.attempts, s.vadTries) : {
    ...s,
    skipped: true
  });
  const apply = async () => {
    const noise = Math.max(0, ...st.day, st.roomReg, st.noise);
    const r = await post(cutoffPath(i, st.cutC, noise, st.floorV, st.hiV)).catch(() => null);
    if (!r?.ok) return;
    if (openRef.current) {
      openRef.current = false;
      await post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
    }
    releasePeers();
    await wakeRead();
    onClose();
  };
  const clearHistory = async () => {
    await post(`/api/sat1/wakewords/clearhist?i=${i}`).catch(() => null);
    await wakeRead();
  };
  const smatter: Mark[] = [...roomMarks(st.day, 38, 80), ...sessionRoomMarks(st.roomSeen, 38, 80)];
  const tries: Mark[] = attemptMarks(st.attempts, 42, 26);
  const place = st.phase === 'place';
  const marks = st.phase === 'nocap' || st.phase === 'gone' ? [] : st.phase === 'ready' ? smatter : place && quick ? [...smatter, ...rowMarks(track, 42, 84)] : [...smatter, ...tries];
  const n = st.attempts.length;
  const prompt = n < 2 ? TEXT.tn2_near.replace('%s', word) : n < 3 ? TEXT.tn2_far : TEXT.tn2_other;
  const verdict = readout(st.cutC, st.floorV, st.day, st.roomReg);
  const readText = verdict.tone === 'warn' ? TEXT.tn2_high.replace('%s', `${verdict.floorC}%`) : verdict.tone === 'err' ? TEXT.tn2_low : `${TEXT.tn2_ok.replace('%s', `${verdict.c}%`)}${verdict.floorC ? TEXT.tn2_under.replace('%s', String(verdict.floorC - verdict.c)) : ''}.`;
  const pmNote = pm && <>
    {pm.muted.length > 0 && <p className="ww-note">{`${pm.muted.length === 1 ? TEXT.tn_pm_one : TEXT.tn_pm_many.replace('%s', String(pm.muted.length))} (${pm.muted.join(', ')})`}</p>}
    {pm.failed.length > 0 && <p className="ww-note warn">{TEXT.tn_pm_failed.replace('%s', pm.failed.join(', '))}</p>}
    {pm.unknown && <p className="ww-note warn">{TEXT.tn_pm_unknown}</p>}
  </>;
  const title = TEXT.tn_title.replace('%s', word);
  return createPortal([<div key="scrim" className="ww-scrim" />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={title}>
    <div className="ww-ptop"><h2 style={{
        display: 'flex',
        alignItems: 'center',
        gap: 8
      }}><span>{title}</span><HintBtn text={HINTS.living_graph} /></h2><button className="secondary" onClick={onClose}>Close</button></div>
    <TouchGraph gid={`tn${i}`} h={150} marks={marks} cut={place ? st.cutC : undefined} onCut={place ? c => setSt(s => ({
      ...s,
      cutC: c
    })) : undefined} aria={place ? `Trigger threshold for “${word}”` : `Tuning graph for “${word}”`} />
    {st.phase === 'ready' && <div><p className="ww-read dim">{(isStop ? TEXT.tn2_ready_stop : TEXT.tn2_ready).replace('%s', word)}</p><div className="ww-btns"><button className="primary" onClick={start}>{TEXT.tn2_start}</button><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
    {st.phase === 'voice' && <div><p className="ww-read"><strong>{prompt}</strong><span> ({Math.min(n, 4)} / 4)</span></p>{st.vadTries > 0 && <p className="ww-note warn">{TEXT.tn_vad}</p>}{pmNote}{n >= 3 && <div className="ww-btns"><button className="secondary" onClick={skip}>{TEXT.tn2_skip}</button></div>}</div>}
    {place && <div><p className={`ww-read ${verdict.tone}`}>{readText}</p>{pmNote}<div className="ww-acts">
      <button className="secondary" onClick={start}>{TEXT.tn2_retune}</button>
      <button className="secondary" onClick={clearHistory}>{TEXT.tn2_clear}</button>
      <button className="secondary" onClick={onClose}>{TEXT.cancel}</button>
      <button className="primary" onClick={apply}>{TEXT.tn_apply}</button>
    </div></div>}
    {st.phase === 'nogap' && <div><p className="ww-read err">{st.nogap === 'room' ? TEXT.tn_nogap_room : TEXT.tn_nogap_voice}</p><div className="ww-btns"><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
    {(st.phase === 'nocap' || st.phase === 'gone') && <div><p className={st.phase === 'gone' ? 'ww-read warn' : 'ww-read dim'}>{st.phase === 'gone' ? TEXT.tn_gone : TEXT.tn_nocap}</p><div className="ww-btns"><button className="secondary" onClick={onClose}>{TEXT.cancel}</button></div></div>}
  </div>], document.body);
}
function PipeDrawer({
  slotName,
  pipe,
  fsd,
  onClose
}: {
  slotName: string;
  pipe: {
    value: string;
    options: [string, string][];
    busy: boolean;
    onPick: (v: string) => void;
  } | null;
  fsd: {
    value: string;
    onPick: (v: string) => void;
  } | null;
  onClose: () => void;
}) {
  const panel = useDrawer(onClose);
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={`Voice Pipeline, ${slotName}`} onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Voice Pipeline</h2><button className="secondary" onClick={onClose}>Close</button></div>
    {pipe && <>
      <h3 className="ww-sec">Voice Pipeline<HintBtn text={<>{HINTS.voice_pipeline} <a href={TEXT.vp_docs_url} target="_blank" rel="noopener">{TEXT.vp_docs}</a></>} /></h3>
      <div className="ww-opts" role="radiogroup" aria-label="Voice Pipeline">{pipe.options.map(([id, label]) => <button key={id} type="button" role="radio" aria-checked={pipe.value === id} className={pipe.value === id ? 'ww-opt on' : 'ww-opt'} disabled={pipe.busy} onClick={() => pipe.value !== id && pipe.onPick(id)}><span>{label}</span><span className="ww-mark">{pipe.value === id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
    </>}
    {fsd && <>
      <h3 className={pipe ? 'ww-sec ww-sec-gap' : 'ww-sec'}>Finished Speaking Detection<HintBtn text={HINTS.finished_speaking} /></h3>
      <div className="ww-opts" role="radiogroup" aria-label="Finished Speaking Detection">{FSD_OPTIONS.map(([id, label]: [string, string]) => <button key={id} type="button" role="radio" aria-checked={fsd.value === id} className={fsd.value === id ? 'ww-opt on' : 'ww-opt'} onClick={() => fsd.value !== id && fsd.onPick(id)}><span>{label}</span><span className="ww-mark">{fsd.value === id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
      <p className="ww-foot">Controls how quickly the satellite detects end of speech.</p>
    </>}
  </div>], document.body);
}
function WordDrawer({
  slotName,
  entries,
  pending,
  current,
  taken,
  busy,
  onPick,
  onClose
}: {
  slotName: string;
  entries: Entry[];
  pending: {
    url: string;
    label: string;
    error: boolean;
  }[];
  current: string;
  taken: string;
  busy: boolean;
  onPick: (e: Entry | null) => void;
  onClose: () => void;
}) {
  const [q, setQ] = useState('');
  const [lang, setLang] = useState('all');
  const panel = useDrawer(onClose);
  const langs: string[] = langsOf(entries);
  const {
    shown,
    hidden
  } = filterEntries(entries, q, lang === 'all' ? '' : lang, current);
  const other = taken.toLowerCase();
  const speakable = canSpeak();
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" ref={panel} tabIndex={-1} className="ww-panel" role="dialog" aria-modal="true" aria-label={`Wake Word Picker, ${slotName}`} onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Wake Word Picker</h2><button className="secondary" onClick={onClose}>Close</button></div>
    <div className="ww-filter"><input className="ww-search" type="search" placeholder="Search words" aria-label="Search words" value={q} onChange={e => setQ(e.currentTarget.value)} />{langs.length > 1 && <Dropdown value={lang} options={[{
        id: 'all',
        label: TEXT.ww_all_langs
      }, ...langs.map(l => ({
        id: l,
        label: langLabel(l)
      }))]} onChange={setLang} label="Language" />}</div>
    <div className="ww-picker-list">
      <div className="ww-opts" role="radiogroup" aria-label="Wake Word">
        {!q.trim() && lang === 'all' && <button type="button" role="radio" aria-checked={!current} className={current ? 'ww-opt' : 'ww-opt on'} disabled={busy} onClick={() => onPick(null)}>
          <span className="ww-opt-label"><span className="ww-opt-word">No wake word</span></span>
          <span className="ww-mark">{!current && <Check size={13} strokeWidth={3} />}</span>
        </button>}
        {shown.map((e: Entry) => {
          const isSelected = e.spec === current;
          const isTaken = !!other && e.word.toLowerCase() === other;
          const note = [e.source, e.ver, e.unverified && TEXT.ww_unverified, isTaken && TEXT.ww_on_other].filter(Boolean).join(' · ');
          return <button key={e.key} type="button" role="radio" aria-checked={isSelected} className={isSelected ? 'ww-opt on' : 'ww-opt'} disabled={busy || isTaken} onClick={() => onPick(e)}>
            <span className="ww-opt-label"><span className="ww-opt-word">{e.word}</span><span className="ww-opt-source">{note}</span></span>
            {speakable && <span className="ww-speak" role="button" tabIndex={0} aria-label={`Preview ${e.word}`} onClick={ev => {
              ev.stopPropagation();
              speak(e.word);
            }} onKeyDown={ev => {
              if (ev.key !== 'Enter' && ev.key !== ' ') return;
              ev.preventDefault();
              ev.stopPropagation();
              speak(e.word);
            }}><SpeakIcon /></span>}
            <span className="ww-mark">{isSelected && <Check size={13} strokeWidth={3} />}</span>
          </button>;
        })}
      </div>
      {hidden > 0 && <p className="ww-foot">{hidden} {TEXT.ww_more}</p>}
      {pending.map(p => <p key={p.url} className="ww-foot">{p.label}: {p.error ? TEXT.ww_source_failed : TEXT.ww_source_loading}</p>)}
      <p className="ww-foot">More words come from the sources in the card below.</p>
    </div>
  </div>], document.body);
}
function Sources({
  sources,
  setSources,
  cat
}: {
  sources: Source[];
  setSources: (list: Source[]) => void;
  cat: Record<string, CatEntry>;
}) {
  const [url, setUrl] = useState('');
  const [bad, setBad] = useState(false);
  const [confirm, setConfirm] = useState<string | null>(null);
  const missing = DEFAULT_SOURCES.filter((d: Source) => !sources.some(s => s.url === d.url));
  const add = () => {
    const src = parseSource(url);
    if (!src) {
      setBad(true);
      return;
    }
    setBad(false);
    if (sources.some(s => s.url === src.url)) return;
    setSources([...sources, src]);
    setUrl('');
  };
  return <article className="ww-card">
    <div className="ww-head"><h2 className="ww-title"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M12 3v12m0 0-4-4m4 4 4-4M4 21h16" /></svg><span>Wake Word Sources</span><HintBtn text={HINTS.wake_sources} /></h2></div>
    {sources.map(s => {
      const c = cat[s.url];
      return <div key={s.url}><div className="ww-src"><a href={s.url} target="_blank" rel="noreferrer">{s.label}</a>{!c?.error && <small>{c?.entries ? `${c.entries.length} ${TEXT.ws_words}` : TEXT.ww_source_loading}</small>}<button className="ww-x" aria-label={`Remove ${s.label}`} onClick={() => setConfirm(s.url)}>Remove</button></div>
        {c?.error && <p className="ww-note warn">{TEXT.ww_source_failed}</p>}
        {confirm === s.url && <div className="ww-confirm"><span>Remove this source? Its words disappear from the picker. Words already installed keep working.</span><div><button className="secondary" onClick={() => setConfirm(null)}>Keep</button><button className="primary" onClick={() => {
              setSources(sources.filter(x => x.url !== s.url));
              setConfirm(null);
            }}>Remove</button></div></div>}</div>;
    })}
    {missing.length > 0 && <p className="ww-foot"><button type="button" className="ww-link" onClick={() => setSources([...missing, ...sources])}>{TEXT.ws_restore}</button></p>}
    <form className="ww-add" noValidate onSubmit={e => {
      e.preventDefault();
      add();
    }}>
      <input type="url" placeholder="https://github.com/owner/repo" value={url} onChange={e => {
        setUrl(e.currentTarget.value);
        setBad(false);
      }} aria-label="Source URL" /><button className="primary" type="submit" disabled={!url.trim()}>Add</button>
    </form>
    {bad && <p className="ww-note err">{TEXT.ws_bad_url}</p>}
    <p className="ww-foot">{TEXT.ws_footer_q}<a href={REQUEST_WORD_URL} target="_blank" rel="noopener">{TEXT.ws_request}</a>{TEXT.ws_or}<a href={TRAIN_URL} target="_blank" rel="noopener">{TEXT.ws_train}</a>.</p>
  </article>;
}
const isOn = (e: any) => e.value === true || e.state === 'ON';
export function WakeTab({
  ctx
}: {
  ctx: Ctx;
}) {
  const {
    ha,
    haRefresh
  } = ctx;
  const [tuning, setTuning] = useState<Tuning | null>(null);
  const [swaps, setSwaps] = useState<Record<number, Swap | null>>({});
  const anyBusy = Object.values(swaps).some(s => s?.phase === 'busy');
  // Swaps and tune sessions run their own faster chained loops; the standing poll stands down
  // meanwhile so only one loop reads at a time.
  const {
    wake,
    wakeRead,
    setSlot
  } = useWakeSlots(tuning || anyBusy ? 0 : 2500);
  // The hook's null is both "not answered yet" and "no loader on this build" (a 404): one read of
  // our own tells them apart, so the not-available note never flashes while the first loads.
  const [settled, setSettled] = useState(false);
  const alive = useRef(true);
  useEffect(() => {
    wakeRead().then(() => alive.current && setSettled(true));
    return () => {
      alive.current = false;
    };
  }, []);
  const [sources, setSourcesState] = useState<Source[]>(readSources);
  const [cat, setCat] = useState<Record<string, CatEntry>>({});
  const setSources = (list: Source[]) => {
    setSourcesState(list);
    writeSources(list);
  };
  useEffect(() => {
    let live = true;
    for (const s of sources) {
      if (cat[s.url] && !cat[s.url].loading) continue;
      setCat(c => ({
        ...c,
        [s.url]: {
          loading: true
        }
      }));
      enumerateSource(s).then((entries: unknown[]) => live && setCat(c => ({
        ...c,
        [s.url]: {
          entries
        }
      }))).catch(() => live && setCat(c => ({
        ...c,
        [s.url]: {
          error: true
        }
      })));
    }
    return () => {
      live = false;
    };
  }, [sources]);
  const [picking, setPicking] = useState<number | null>(null);
  const [piping, setPiping] = useState<number | null>(null);
  const slots: Track[] = wake?.slots || [];
  const slotAt = (i: number) => slots.find(s => s.i === i);
  const activeWords = slots.filter(s => s.m && s.w).map(s => s.w as string);
  const assist = useAssist(ha, haRefresh, activeWords);
  useEffect(() => {
    haSyncOnce(haRefresh);
  }, []);
  // One word per mount: a word listening on the device but holding no Home Assistant select (a
  // browser closed mid-pairing) gets one. Keyed on the word list too, since HA and the slot read
  // land in either order.
  const repaired = useRef(false);
  useEffect(() => {
    if (repaired.current || !assist.ready) return;
    const orphan = activeWords.find(w => assist.pipelineFor(w) === null);
    if (!orphan) return;
    repaired.current = true;
    assist.syncSlot(orphan, true).catch(() => {});
  }, [assist.ready, activeWords.length]);

  // Home Assistant's wake word selects follow the device's slots. An empty `asst` is expected
  // while HA reloads this device's config entry after a download, so it is waited out, not
  // taken as a verdict.
  const syncAssist = async (i: number, prevWord: string, nextWord: string) => {
    const deadline = Date.now() + 60000;
    for (;;) {
      const fresh = await requestJson('/api/sat1/ha').catch(() => null);
      const sel = fresh?.d?.asst?.s;
      if (Array.isArray(sel) && sel.length) {
        const write = pairingWrite(sel, i, prevWord, nextWord);
        if (!write) return;
        await post(`/api/sat1/ha/select?e=${encodeURIComponent(write[0])}&o=${encodeURIComponent(write[1])}`).catch(() => {});
        await haRefresh();
      }
      if (!alive.current || Date.now() > deadline) return;
      await sleep(500);
    }
  };
  const choose = async (i: number, spec: string, word: string) => {
    const prevWord = slotAt(i)?.w || '';
    if (tuning && tuning.i === i) setTuning(null);
    setSwaps(s => ({
      ...s,
      [i]: {
        phase: 'busy',
        word,
        spec
      }
    }));
    const r = await setSlot(i, spec).catch(() => ({
      ok: false
    }));
    if (!r.ok) {
      if (alive.current) setSwaps(s => ({
        ...s,
        [i]: {
          phase: 'error',
          word,
          spec,
          err: 0
        }
      }));
      return;
    }
    const deadline = Date.now() + (isUrl(spec) ? 90000 : 10000);
    for (;;) {
      await sleep(isUrl(spec) ? 900 : 400);
      if (!alive.current) return;
      const d = await wakeRead();
      const s = d?.slots?.find((x: Track) => x.i === i);
      const step = swapStep(s, spec);
      if (step?.done) {
        setSwaps(prev => ({
          ...prev,
          [i]: null
        }));
        await syncAssist(i, prevWord, spec === 'none' ? '' : s.w || word);
        return;
      }
      if (step && 'err' in step) {
        setSwaps(prev => ({
          ...prev,
          [i]: {
            phase: 'error',
            word,
            spec,
            err: step.err
          }
        }));
        return;
      }
      if (step) setSwaps(prev => ({
        ...prev,
        [i]: {
          ...(prev[i] as Swap),
          dl: step.dl,
          tot: step.tot
        }
      }));
      if (Date.now() > deadline) {
        setSwaps(prev => ({
          ...prev,
          [i]: {
            phase: 'error',
            word,
            spec,
            err: 7
          }
        }));
        return;
      }
    }
  };

  // Finished Speaking Detection: the device keeps one select per slot and copies the firing slot's
  // value into Home Assistant's own before each request; `unset` follows HA.
  const fsdRaw = ha?.d?.fsd;
  const haFsd: string | null = !haBlocked(ha) && !haTooOld(ha) && Array.isArray(fsdRaw) && fsdRaw.length === 2 ? fsdRaw[1] : null;
  const fsdOwn = FSD_KEYS.map((k: string) => entity(ctx, k)?.value);
  const fsdNow: string[] | null = fsdShown(fsdOwn, haFsd);
  const [fsdHold, setFsdHold] = useState<Record<number, string>>({});
  const pickFsd = async (i: number, option: string) => {
    const writes: [number, string][] = fsdWrites(fsdOwn, haFsd, i, option);
    setFsdHold(h => ({
      ...h,
      ...Object.fromEntries(writes)
    }));
    const done = await Promise.all(writes.map(([k, v]) => post(`${pathFor(ctx, FSD_KEYS[k], 'set')}?option=${encodeURIComponent(v)}`).catch(() => null)));
    // Held until the stream has had time to report the device's own value; a refused write
    // snaps back at once.
    const drop = () => alive.current && setFsdHold(h => {
      const next = {
        ...h
      };
      for (const [k, v] of writes) if (next[k] === v) delete next[k];
      return next;
    });
    if (done.some(r => !r?.ok)) drop();else setTimeout(drop, 2000);
  };
  const stopSwitch = entity(ctx, 'stop_word');
  const stopOn = stopSwitch ? isOn(stopSwitch) : false;
  // The switch is the preference; `stop_active` is whether the stop model runs right now. Firmware
  // without the sensor falls back to the switch, never a false "Paused".
  const stopActive = entity(ctx, 'stop_active');
  const stopRunning: boolean | null = stopActive ? isOn(stopActive) : null;
  const wakeSound = entity(ctx, 'wake_sound');
  const chime: boolean | null = wakeSound ? isOn(wakeSound) : null;
  const heading = <><span className="eyebrow">WAKE · WAKE WORDS</span><h1>Say the <em>word.</em></h1></>;
  if (!ctx.device || !wake) return <section className="control wake-section">
    <style>{CSS}</style>
    {heading}
    {ctx.device && settled && <article className="ww-card"><p className="ww-note">Wake word control is not available on this firmware build.</p></article>}
  </section>;
  const stopw: Track | null = wake.stopw || null;
  const pipeOptions: [string, string][] = [[PIPELINE_PREFERRED, TEXT.pipeline_preferred], ...assist.pipelines.map((p: string) => [p, p] as [string, string])];
  const pipeValue = (w: string): string => assist.pipelineFor(w) ?? assist.fallbackPipeline() ?? PIPELINE_PREFERRED;
  const pipeLabel = (v: string) => v === PIPELINE_PREFERRED ? TEXT.pipeline_preferred : v;
  const tuneSeed = (i: number) => {
    const t = i === STOP_SLOT ? stopw : slotAt(i);
    return {
      cut: t?.cut || 0,
      noise: t?.tn?.[0] || 0,
      floor: t?.tn?.[1] || 0,
      hi: t?.tn?.[2] || 0,
      day: t?.day || []
    };
  };
  const assistNote = !assist.ready && activeWords.length > 0 && <p className="ww-note">
    {haBlocked(ha) ? TEXT.assistant_blocked : haTooOld(ha) ? TEXT.ha_too_old : TEXT.assistant_needs_ha}
    {haBlocked(ha) && <> <button type="button" className="ww-link" onClick={ctx.onShowFix}>{TEXT.show_fix}</button></>}
  </p>;
  const wordCard = (i: number) => {
    const slot = slotAt(i);
    const swap = swaps[i];
    const swapping = swap?.phase === 'busy';
    const failed = swap?.phase === 'error';
    const word = swapping ? swap.word : slot?.w || '';
    const waiting = !swap && !!slot && (slot.st === 3 || slot.st === 2 && !slot.ld && isUrl(slot.m));
    const tuned = !swapping && !!slot && slot.cut > 0;
    const live = tuned && !!slot.ld;
    const tuneBtn = !tuned && !swapping && !failed && !!slot?.ld;
    const pipe = !swapping && word && assist.ready ? pipeValue(word) : null;
    let note: [string, string] | null = null;
    if (swapping) {
      const progress = (swap.tot || 0) > 0 ? `${kb(swap.dl || 0)} / ${kb(swap.tot as number)} KB` : isUrl(swap.spec) ? TEXT.ww_downloading : TEXT.ww_loading;
      note = ['', slot?.w && slot.w !== swap.word ? `${progress} ${TEXT.mb_swap_note.replace('%1', showWord(slot.w)).replace('%2', showWord(swap.word))}` : progress];
    } else if (failed) note = ['err', `${TEXT.ww_failed} ${WW_ERR[swap.err || 0] || ''}`.trim()];else if (waiting) note = ['warn', slot.st === 3 ? TEXT.ww_waiting : `${WW_ERR[slot.err || 0] || ''} ${TEXT.ww_retrying}`.trim()];
    return <article className="ww-card" key={`w${i}`}>
      <div className="ww-head"><h2 className="ww-title"><span>Wake Word {i + 1}</span><HintBtn text={HINTS.wake_words} /></h2>
        {i === 0 && chime !== null && <span className="ww-title"><button className={chime ? 'ww-bell' : 'ww-bell off'} aria-pressed={chime} aria-label={TEXT.ww_chime} onClick={() => post(pathFor(ctx, 'wake_sound', chime ? 'turn_off' : 'turn_on'))}>
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M6 8a6 6 0 0 1 12 0c0 7 3 9 3 9H3s3-2 3-9M10.3 21a1.94 1.94 0 0 0 3.4 0" />{!chime && <path d="m3 3 18 18" />}</svg>
        </button><HintBtn text={HINTS.wake_sound} /></span>}</div>
      <div className="ww-flow">
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Wake Word</span>
          <button className={picking === i ? 'ww-flow-btn open' : 'ww-flow-btn'} aria-haspopup="dialog" disabled={swapping} onClick={() => setPicking(i)}>
            <span>{word ? showWord(word) : 'None'}</span>
            <ChevronDown size={16} />
          </button>
        </div>
        {pipe !== null && <>
          <span className="ww-flow-arrow" aria-hidden="true"><ArrowRight size={18} /></span>
          <div className="ww-flow-cell">
            <span className="ww-flow-cap">Voice Pipeline</span>
            <button className={piping === i ? 'ww-flow-btn open' : 'ww-flow-btn'} aria-haspopup="dialog" onClick={() => setPiping(i)}>
              <span>{pipeLabel(pipe)}</span>
              <ChevronDown size={16} />
            </button>
          </div>
        </>}
      </div>
      {note && <p className={note[0] ? `ww-note ${note[0]}` : 'ww-note'}>{note[1]}</p>}
      {failed && <div className="ww-btns"><button className="primary" onClick={() => choose(i, swap.spec, swap.word)}>{TEXT.ww_retry}</button></div>}
      {i === 0 && activeWords.length === 0 && !anyBusy && <p className="ww-note">{TEXT.ww_route_none}</p>}
      {i === 0 && assistNote}
      {(tuned || tuneBtn) && <div className="ww-top">
        {live && <span className="ww-live"><i />{TEXT.lg_listening}</span>}
        {tuned && <HintBtn text={HINTS.living_graph} />}
        {tuneBtn && <button className="ww-tune" onClick={() => setTuning({
          i,
          word,
          isStop: false,
          quick: false
        })}>{TEXT.ww_tune_btn}</button>}
      </div>}
      {tuned && <TouchGraph gid={`g${i}`} h={72} marks={rowMarks(slot)} cut={pctN(slot.cut)} onTune={() => setTuning({
        i,
        word,
        isStop: false,
        quick: true
      })} aria={TEXT.tn_title.replace('%s', showWord(word))} />}
    </article>;
  };
  const stopCard = () => {
    if (!stopw || !stopSwitch) return null;
    const tuned = stopOn && stopw.cut > 0;
    const live = stopRunning === null ? tuned : stopRunning;
    const paused = stopRunning === false && stopOn;
    const tuneBtn = stopOn && !tuned;
    return <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><span>Stop Word</span><HintBtn text={HINTS.stop_word} /></h2></div>
      <button className={`wsw${stopOn ? ' on' : ''}`} style={{
        marginTop: 14,
        marginBottom: 16
      }} role="switch" aria-checked={stopOn} onClick={() => post(pathFor(ctx, 'stop_word', stopOn ? 'turn_off' : 'turn_on'))}>
        {!stopOn && <span className="wsw-knob" />}
        <span>Stop</span>
        {stopOn && <span className="wsw-knob" />}
      </button>
      {(live || paused || tuneBtn) && <div className="ww-top">
        {live ? <span className="ww-live"><i />{TEXT.lg_listening}</span> : paused ? <span className="ww-live paused"><i />{TEXT.lg_paused}</span> : null}
        {tuneBtn && <button className="ww-tune" onClick={() => setTuning({
          i: STOP_SLOT,
          word: 'stop',
          isStop: true,
          quick: false
        })}>{TEXT.ww_tune_btn}</button>}
      </div>}
      {tuned && <TouchGraph gid="gs" h={72} marks={rowMarks(stopw)} cut={pctN(stopw.cut)} onTune={() => setTuning({
        i: STOP_SLOT,
        word: 'stop',
        isStop: true,
        quick: true
      })} aria={TEXT.tn_title.replace('%s', 'Stop')} />}
    </article>;
  };
  const pickerFor = (i: number) => {
    const entries: Entry[] = pickerEntries(wake.builtin, TEXT.ww_included, sources, cat);
    const pending = sources.filter(s => cat[s.url]?.loading || cat[s.url]?.error).map(s => ({
      url: s.url,
      label: s.label,
      error: !!cat[s.url]?.error
    }));
    const current = swaps[i]?.phase === 'busy' ? (swaps[i] as Swap).spec : slotAt(i)?.m || '';
    return <WordDrawer slotName={`Wake Word ${i + 1}`} entries={entries} pending={pending} current={current} taken={slotAt(1 - i)?.w || swaps[1 - i]?.word || ''} busy={anyBusy} onPick={e => {
      setPicking(null);
      if (!e) {
        if (current) choose(i, 'none', '');
      } else if (e.spec !== current) choose(i, e.spec, e.word);
    }} onClose={() => setPicking(null)} />;
  };
  const pipeFor = (i: number) => {
    const w = slotAt(i)?.w || '';
    if (!w) return null;
    return <PipeDrawer slotName={`Wake Word ${i + 1}`} pipe={assist.ready ? {
      value: pipeValue(w),
      options: pipeOptions,
      busy: assist.busy,
      onPick: v => assist.setPipeline(w, v)
    } : null} fsd={fsdNow ? {
      value: fsdHold[i] ?? fsdNow[i],
      onPick: v => pickFsd(i, v)
    } : null} onClose={() => setPiping(null)} />;
  };
  return <section className="control wake-section">
    <style>{CSS}</style>
    {heading}
    {wordCard(0)}
    {wordCard(1)}
    {stopCard()}
    <Sources sources={sources} setSources={setSources} cat={cat} />
    {picking !== null && pickerFor(picking)}
    {piping !== null && pipeFor(piping)}
    {tuning && <Tuner key={`${tuning.i}-${tuning.quick}`} ctx={ctx} i={tuning.i} word={showWord(tuning.word)} isStop={tuning.isStop} quick={tuning.quick} seed={tuneSeed(tuning.i)} track={tuning.i === STOP_SLOT ? stopw : slotAt(tuning.i) || null} wakeRead={wakeRead} onClose={() => setTuning(null)} />}
  </section>;
}
