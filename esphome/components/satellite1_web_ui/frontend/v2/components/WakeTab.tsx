import { useEffect, useLayoutEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import type { ReactNode } from 'react';
import { ArrowRight, ChevronDown, Check } from '../icons';
type MarkKind = 'fire' | 'near' | 'room' | 'you';
interface Mark {
  id: string;
  c: number;
  y: number;
  kind: MarkKind;
}
interface Slot {
  id: string;
  word: string;
  tuned: boolean;
  cut: number;
  pipe: string;
  eos: string;
}
interface LibEntry {
  id: string;
  word: string;
  lang: string;
  kb: number;
  unverified?: boolean;
}
interface LibGroup {
  key: string;
  label: string;
  entries: LibEntry[];
}
interface Source {
  id: string;
  url: string;
  count: number;
}
interface Attempt {
  score: number;
  round: 'near' | 'far' | 'them';
}
const CUT_MIN = 40;
const CUT_MAX = 95;
const ROOM_SCORE = 35;
const HINTS = {
  wake_words: 'Up to two wake words listen at once, each pointed at its own Voice Pipeline. The graph under each word is its last 24 hours.',
  living_graph: 'Solid dots are firings, hollow ones close calls, amber smatter is the room. Drag the knob to move the trigger threshold.',
  wake_sound: 'The chime the device plays when it hears a wake word.',
  stop_word: 'Say Stop to interrupt the assistant mid-response. When disabled, timers and alarms will keep ringing until the assistant is asked to stop them.',
  voice_pipeline: 'Which Home Assistant Assist pipeline answers this word. Untuned words fire on the model default — a two-minute tune fits the word to this room.',
  wake_sources: 'Where the word list comes from. Add a GitHub repo or a manifest URL to offer more words. Words already installed keep working even if a source is removed.'
};
const PIPES = [{
  id: 'preferred',
  label: 'Preferred'
}, {
  id: 'Home Assistant Cloud',
  label: 'Home Assistant Cloud'
}, {
  id: 'Local Whisper',
  label: 'Local Whisper'
}];
const EOS_OPTS = [{
  id: 'default',
  label: 'Default'
}, {
  id: 'aggressive',
  label: 'Aggressive'
}, {
  id: 'relaxed',
  label: 'Relaxed'
}, {
  id: 'manual',
  label: 'Manual'
}];
const LANGS = [{
  id: 'all',
  label: 'All languages'
}, {
  id: 'en',
  label: 'English'
}, {
  id: 'es',
  label: 'Spanish'
}, {
  id: 'fr',
  label: 'French'
}, {
  id: 'de',
  label: 'German'
}];
const LIBRARY: LibGroup[] = [{
  key: 'esphome',
  label: 'esphome/micro-wake-word-models',
  entries: [{
    id: 'on',
    word: 'Okay Nabu',
    lang: 'en',
    kb: 276
  }, {
    id: 'hj',
    word: 'Hey Jarvis',
    lang: 'en',
    kb: 267
  }, {
    id: 'hm',
    word: 'Hey Mycroft',
    lang: 'en',
    kb: 262
  }, {
    id: 'al',
    word: 'Alexa',
    lang: 'en',
    kb: 294
  }, {
    id: 'co',
    word: 'Computer',
    lang: 'en',
    kb: 259
  }, {
    id: 'oc',
    word: 'Okay Casita',
    lang: 'es',
    kb: 264
  }]
}, {
  key: 'fph',
  label: 'FutureProofHomes/wakewords',
  entries: [{
    id: 'hs',
    word: 'Hey Satellite',
    lang: 'en',
    kb: 270
  }, {
    id: 'bm',
    word: 'Bonjour Maison',
    lang: 'fr',
    kb: 265,
    unverified: true
  }, {
    id: 'hh',
    word: 'Hallo Haus',
    lang: 'de',
    kb: 263,
    unverified: true
  }]
}];
const WORD_MARKS: Mark[] = [{
  id: 'f1',
  c: 88,
  y: 22,
  kind: 'fire'
}, {
  id: 'f2',
  c: 81,
  y: 35,
  kind: 'fire'
}, {
  id: 'n1',
  c: 64,
  y: 28,
  kind: 'near'
}, {
  id: 'n2',
  c: 55,
  y: 43,
  kind: 'near'
}, {
  id: 'r1',
  c: 24,
  y: 31,
  kind: 'room'
}, {
  id: 'r2',
  c: 31,
  y: 46,
  kind: 'room'
}];
const STOP_MARKS: Mark[] = [{
  id: 's1',
  c: 86,
  y: 25,
  kind: 'fire'
}, {
  id: 's2',
  c: 20,
  y: 32,
  kind: 'room'
}];
const ROOM_SMATTER: Mark[] = [{
  id: 'rs1',
  c: 22,
  y: 48,
  kind: 'room'
}, {
  id: 'rs2',
  c: 31,
  y: 84,
  kind: 'room'
}, {
  id: 'rs3',
  c: 18,
  y: 104,
  kind: 'room'
}, {
  id: 'rs4',
  c: 27,
  y: 64,
  kind: 'room'
}, {
  id: 'rs5',
  c: 35,
  y: 96,
  kind: 'room'
}];
const ATTEMPT_PLAN: Attempt[] = [{
  score: 87,
  round: 'near'
}, {
  score: 84,
  round: 'near'
}, {
  score: 76,
  round: 'far'
}, {
  score: 79,
  round: 'them'
}];
const ROUND_TEXT: Record<string, string> = {
  near: 'Stand near the satellite and say it normally',
  far: 'Walk across the room and say it again',
  them: 'Have someone else in the house say it'
};
const INITIAL_SLOTS: Slot[] = [{
  id: 'a',
  word: 'Okay Nabu',
  tuned: true,
  cut: 72,
  pipe: 'preferred',
  eos: 'default'
}, {
  id: 'b',
  word: 'Hey Jarvis',
  tuned: false,
  cut: 68,
  pipe: 'Local Whisper',
  eos: 'default'
}];
const INITIAL_SOURCES: Source[] = [{
  id: 'src1',
  url: 'https://github.com/esphome/micro-wake-word-models',
  count: 6
}, {
  id: 'src2',
  url: 'https://github.com/FutureProofHomes/wakewords',
  count: 3
}];
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
interface FlatEntry {
  id: string;
  word: string;
  lang: string;
  source: string;
}
const FLAT: FlatEntry[] = LIBRARY.flatMap(g => g.entries.map(e => ({
  id: e.id,
  word: e.word,
  lang: e.lang,
  source: g.label
})));
const filterFlat = (q: string, lang: string, sel: string[]) => FLAT.filter(e => e.word.toLowerCase().includes(q.toLowerCase()) && (lang === 'all' || e.lang === lang)).sort((a, b) => Number(sel.includes(b.word)) - Number(sel.includes(a.word)));
const speakWord = (w: string, l: string) => {
  if ('speechSynthesis' in window) {
    window.speechSynthesis.cancel();
    const u = new SpeechSynthesisUtterance(w);
    u.lang = l === 'fr' ? 'fr-FR' : l === 'de' ? 'de-DE' : l === 'es' ? 'es-ES' : 'en-US';
    window.speechSynthesis.speak(u);
  }
};
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
      if (e.key === 'Escape') setOpen(false);
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
  if (m.kind === 'fire') return <g key={m.id}><circle className="lg-halo-a" cx={mx} cy={my} r="9" /><circle className="lg-rip" cx={mx} cy={my} r="8.5" /><circle className="lg-fire" cx={mx} cy={my} r="5" /></g>;
  if (m.kind === 'near') return <circle key={m.id} className="lg-near" cx={mx} cy={my} r="3.5" />;
  if (m.kind === 'room') return <g key={m.id}><circle className="lg-halo-w" cx={mx} cy={my} r="7" /><circle className="lg-rip-room" cx={mx} cy={my} r="8.5" /><circle className="lg-room" cx={mx} cy={my} r="3.5" /></g>;
  return <g key={m.id}><circle className="lg-rip-you" cx={mx} cy={my} r="8.5" /><circle className="lg-you" cx={mx} cy={my} r="5" /></g>;
}
function TouchGraph({
  marks,
  cut,
  onCut,
  onTune,
  h = 120,
  aria
}: {
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
  const dragDelta = useRef(0);
  const pointerStart = useRef({
    x: 0,
    y: 0
  });
  const x = (c: number) => c / 100 * W;
  const plotH = h - 16;
  const rows = [1, 2, 3].map(i => ({
    id: `h${i}`,
    y: Math.round(plotH * i / 4)
  }));
  const setFrom = (clientX: number) => {
    const r = ref.current?.getBoundingClientRect();
    if (!r || !onCut) return;
    const v = Math.round((clientX - r.left) / r.width * 100);
    onCut(Math.max(CUT_MIN, Math.min(CUT_MAX, v)));
  };
  const cx = cut !== undefined ? x(cut) : 0;
  const uid = aria.replace(/\s+/g, '');
  return <svg ref={ref} className="ww-graph" viewBox={`0 0 ${W} ${h}`} role="img" aria-label={aria} onPointerMove={e => {
    if (held) {
      const dx = Math.abs(e.clientX - pointerStart.current.x);
      const dy = Math.abs(e.clientY - pointerStart.current.y);
      dragDelta.current = Math.max(dragDelta.current, dx + dy);
      setFrom(e.clientX);
    }
  }} onPointerUp={() => {
    if (held && dragDelta.current < 4 && onTune) onTune();
    setHeld(false);
    dragDelta.current = 0;
  }} onPointerCancel={() => setHeld(false)}>
    <defs>
      <filter id="lg-frost" x="-30%" y="-30%" width="160%" height="160%" colorInterpolationFilters="sRGB">
        <feGaussianBlur in="SourceGraphic" stdDeviation="6" />
      </filter>
      <clipPath id={`lg-veil-clip-${uid}`}><rect x="0" y="0" width={cx} height={plotH} /></clipPath>
      <clipPath id={`lg-clear-clip-${uid}`}><rect x={cx} y="0" width={W - cx} height={plotH} /></clipPath>
    </defs>
    <rect className="lg-track" x="0" y="0" width={W} height={plotH} rx="7" />
    {GRID_V.map(g => <line key={g.id} className={g.mj ? 'lg-paper mj' : 'lg-paper'} x1={x(g.p)} x2={x(g.p)} y1="0" y2={plotH} />)}
    {rows.map(r => <line key={r.id} className="lg-paper" x1="0" x2={W} y1={r.y} y2={r.y} />)}
    {cut !== undefined && <rect className="lg-veil" x="0" y="0" width={cx} height={plotH} rx="7" />}
    {cut !== undefined && <rect className="lg-tint" x={cx} y="0" width={W - cx} height={plotH} />}
    {cut !== undefined && <g clipPath={`url(#lg-veil-clip-${uid})`} filter="url(#lg-frost)">{marks.map(m => renderMark(m, x, plotH))}</g>}
    <g clipPath={cut !== undefined ? `url(#lg-clear-clip-${uid})` : undefined}>{marks.map(m => renderMark(m, x, plotH))}</g>
    <line className="lg-axis" x1="0" x2={W} y1={plotH} y2={plotH} />
    {TICKS.map(t => <g key={t.id}><line className="lg-axis" x1={x(t.p)} x2={x(t.p)} y1={plotH} y2={plotH + 4} /><text className="lg-al" x={x(t.p)} y={h - 1} textAnchor={t.p === 0 ? 'start' : t.p === 100 ? 'end' : 'middle'}>{t.p}%</text></g>)}
    {cut !== undefined && <g>
      <line className="lg-stem" x1={cx} x2={cx} y1="0" y2={plotH} />
      {held && <circle className="lg-heldring" cx={cx} cy={plotH / 2} r="18" />}
      <rect className={held ? 'lg-knob held' : 'lg-knob'} x={cx - 7} y={plotH / 2 - 14} width="14" height="28" rx="4.5" onPointerDown={e => {
        if (!onCut) return;
        (e.currentTarget.ownerSVGElement as SVGSVGElement).setPointerCapture(e.pointerId);
        setHeld(true);
        dragDelta.current = 0;
        pointerStart.current = {
          x: e.clientX,
          y: e.clientY
        };
      }} />
      <line className="lg-grip" x1={cx - 2} x2={cx - 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
      <line className="lg-grip" x1={cx + 2} x2={cx + 2} y1={plotH / 2 - 6} y2={plotH / 2 + 6} />
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
      if (e.key === 'Escape') setOpen(false);
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
const Chev = () => <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5"><path d="m6 9 6 6 6-6" /></svg>;
function useDrawerBlur() {
  useEffect(() => {
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, []);
}
function Picker({
  current,
  other,
  onPick,
  onClose
}: {
  current: string;
  other: string;
  onPick: (w: string) => void;
  onClose: () => void;
}) {
  const [q, setQ] = useState('');
  const [lang, setLang] = useState('all');
  const list = filterFlat(q, lang, [current]);
  useDrawerBlur();
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" className="ww-panel" role="dialog" aria-label="Wake Word Picker" onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Wake Word Picker</h2><button className="secondary" onClick={onClose}>Close</button></div>
    <div className="ww-filter"><input className="ww-search" placeholder="Search words" value={q} onChange={e => setQ(e.target.value)} /><Dropdown value={lang} options={LANGS} onChange={setLang} label="Language" /></div>
    <div className="ww-picker-list">
      <div className="ww-opts" role="radiogroup" aria-label="Wake Word">{list.map(e => {
          const isSelected = e.word === current;
          const isTaken = e.word === other;
          return <button key={e.id} type="button" role="radio" aria-checked={isSelected} className={isSelected ? 'ww-opt on' : 'ww-opt'} disabled={isTaken} onClick={() => !isTaken && onPick(e.word)}>
            <span className="ww-opt-label"><span className="ww-opt-word">{e.word}</span><span className="ww-opt-source">{e.source}</span></span>
            <span className="ww-speak" role="button" tabIndex={0} aria-label={`Preview ${e.word}`} onClick={ev => {
              ev.stopPropagation();
              speakWord(e.word, e.lang);
            }}><SpeakIcon /></span>
            <span className="ww-mark">{isSelected && <Check size={13} strokeWidth={3} />}</span>
          </button>;
        })}</div>
      <p className="ww-foot">More words come from the sources in the card below.</p>
    </div>
  </div>], document.body);
}
function Tuner({
  word,
  onApply,
  onClose
}: {
  word: string;
  onApply: (cut: number) => void;
  onClose: () => void;
}) {
  const [phase, setPhase] = useState<'ready' | 'voice' | 'place'>('ready');
  const [n, setN] = useState(0);
  const [cut, setCut] = useState(60);
  const [room, setRoom] = useState<Mark[]>(ROOM_SMATTER);
  useDrawerBlur();
  useEffect(() => {
    if (phase !== 'voice') return;
    if (n >= ATTEMPT_PLAN.length) {
      const floor = Math.min(...ATTEMPT_PLAN.map(a => a.score));
      setCut(Math.round(ROOM_SCORE + 0.6 * (floor - ROOM_SCORE)));
      setPhase('place');
      return;
    }
    const t = setTimeout(() => setN(v => v + 1), 2800);
    return () => clearTimeout(t);
  }, [phase, n]);
  const you: Mark[] = ATTEMPT_PLAN.slice(0, n).map((a, i) => ({
    id: `y${a.score}${a.round}`,
    c: a.score,
    y: 30 + i * 22,
    kind: 'you' as MarkKind
  }));
  const floor = Math.min(...ATTEMPT_PLAN.map(a => a.score));
  const cur = ATTEMPT_PLAN[Math.min(n, ATTEMPT_PLAN.length - 1)];
  let read = {
    cls: 'dim',
    t: `Fires above ${cut}% · every room dot in the frost · ${floor - cut} pts under your quietest try.`
  };
  if (cut >= floor - 2) read = {
    cls: 'warn',
    t: `Above your quietest try (${floor}%) - real calls from across the room will be missed.`
  };else if (cut <= ROOM_SCORE) read = {
    cls: 'err',
    t: 'Amber dots sit past your line - the room has scored this high in the last day. Expect false firings.'
  };
  return createPortal([<div key="scrim" className="ww-scrim" />, <div key="panel" className="ww-panel" role="dialog" aria-label={`Tune ${word}`}>
    <div className="ww-ptop"><h2 style={{
        display: 'flex',
        alignItems: 'center',
        gap: 8
      }}><span>Tune “{word}”</span><HintBtn text={HINTS.living_graph} /></h2><button className="secondary" onClick={onClose}>Close</button></div>
    <TouchGraph h={150} marks={[...room, ...you]} cut={phase === 'place' ? cut : undefined} onCut={phase === 'place' ? setCut : undefined} aria={`Tuning graph for ${word}`} />
    {phase === 'ready' && <div><p className="ww-read dim">Follow the prompts and walk the room saying “{word}”. Blue dots are live detections; amber dots are the past 24 hours. You'll use them to place the firing line - and you can move it any time.</p><div className="ww-btns"><button className="primary" onClick={() => {
          setN(0);
          setPhase('voice');
        }}>Start</button><button className="secondary" onClick={onClose}>Cancel</button></div></div>}
    {phase === 'voice' && <div><p className="ww-read"><strong>{ROUND_TEXT[cur.round]}</strong><span> ({Math.min(n + 1, 4)} / 4)</span></p><p className="ww-note">Muted 1 nearby satellite for this session (Kitchen Satellite)</p></div>}
    {phase === 'place' && <div><p className={`ww-read ${read.cls}`}>{read.t}</p><div className="ww-acts">
      <button className="secondary" onClick={() => {
          setN(0);
          setPhase('voice');
        }}>Re-Tune</button>
      <button className="secondary" onClick={() => setRoom([])}>Clear history</button>
      <button className="secondary" onClick={onClose}>Cancel</button>
      <button className="primary" onClick={() => onApply(cut)}>Apply</button>
    </div></div>}
  </div>], document.body);
}
function PipeDrawer({
  slot,
  onChange,
  onClose
}: {
  slot: Slot;
  onChange: (p: Partial<Slot>) => void;
  onClose: () => void;
}) {
  useDrawerBlur();
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" className="ww-panel" role="dialog" aria-label="Voice Pipeline" onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Voice Pipeline</h2><button className="secondary" onClick={onClose}>Close</button></div>
    <h3 className="ww-sec">Voice Pipeline</h3>
    <div className="ww-opts" role="radiogroup" aria-label="Voice Pipeline">{PIPES.map(p => <button key={p.id} type="button" role="radio" aria-checked={slot.pipe === p.id} className={slot.pipe === p.id ? 'ww-opt on' : 'ww-opt'} onClick={() => onChange({
        pipe: p.id
      })}><span>{p.label}</span><span className="ww-mark">{slot.pipe === p.id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
    <h3 className="ww-sec ww-sec-gap">Finished Speaking Detection</h3>
    <div className="ww-opts" role="radiogroup" aria-label="Finished Speaking Detection">{EOS_OPTS.map(o => <button key={o.id} type="button" role="radio" aria-checked={slot.eos === o.id} className={slot.eos === o.id ? 'ww-opt on' : 'ww-opt'} onClick={() => onChange({
        eos: o.id
      })}><span>{o.label}</span><span className="ww-mark">{slot.eos === o.id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
    <p className="ww-foot">Controls how quickly the satellite detects end of speech.</p>
  </div>], document.body);
}
function WordDrawer({
  selected,
  slotId,
  onToggle,
  onClose
}: {
  selected: string[];
  slotId: string;
  onToggle: (w: string) => void;
  onClose: () => void;
}) {
  const [q, setQ] = useState('');
  const [lang, setLang] = useState('all');
  const current = selected[0] ?? '';
  const list = filterFlat(q, lang, selected);
  useDrawerBlur();
  return createPortal([<div key="scrim" className="ww-scrim" onClick={onClose} />, <div key="panel" className="ww-panel" role="dialog" aria-label={`Wake Word for slot ${slotId}`} onClick={e => e.stopPropagation()}>
    <div className="ww-ptop"><h2>Wake Word Picker</h2><button className="secondary" onClick={onClose}>Close</button></div>
    <div className="ww-filter"><input className="ww-search" placeholder="Search words" value={q} onChange={e => setQ(e.target.value)} /><Dropdown value={lang} options={LANGS} onChange={setLang} label="Language" /></div>
    <div className="ww-picker-list">
      <div className="ww-opts" role="radiogroup" aria-label="Wake Word">{list.map(e => {
          const isSelected = e.word === current;
          const isTaken = selected.includes(e.word) && e.word !== current;
          return <button key={e.id} type="button" role="radio" aria-checked={isSelected} className={isSelected ? 'ww-opt on' : 'ww-opt'} disabled={isTaken} onClick={() => !isTaken && onToggle(e.word)}>
            <span className="ww-opt-label"><span className="ww-opt-word">{e.word}</span><span className="ww-opt-source">{e.source}</span></span>
            <span className="ww-speak" role="button" tabIndex={0} aria-label={`Preview ${e.word}`} onClick={ev => {
              ev.stopPropagation();
              speakWord(e.word, e.lang);
            }}><SpeakIcon /></span>
            <span className="ww-mark">{isSelected && <Check size={13} strokeWidth={3} />}</span>
          </button>;
        })}</div>
      <p className="ww-foot">More words come from the sources in the card below.</p>
    </div>
  </div>], document.body);
}
export function WakeTab() {
  const [slots, setSlots] = useState<Slot[]>(INITIAL_SLOTS);
  const [stopOn, setStopOn] = useState(true);
  const [stopCut, setStopCut] = useState(75);
  const [chime, setChime] = useState(true);
  const [picking, setPicking] = useState<string | null>(null);
  const [tuning, setTuning] = useState<string | null>(null);
  const [sources, setSources] = useState<Source[]>(INITIAL_SOURCES);
  const [url, setUrl] = useState('');
  const [confirm, setConfirm] = useState<string | null>(null);
  const [piping, setPiping] = useState<string | null>(null);
  const [wordsOpen, setWordsOpen] = useState<string | null>(null);
  const upd = (id: string, p: Partial<Slot>) => setSlots(s => s.map(x => x.id === id ? {
    ...x,
    ...p
  } : x));
  const toggleWord = (w: string) => setSlots(s => {
    if (s.some(x => x.word === w)) return s.length > 1 ? s.filter(x => x.word !== w) : s;
    if (s.length >= 2) return s;
    const base = s[0];
    return [...s, {
      id: `s${Date.now()}`,
      word: w,
      tuned: false,
      cut: 68,
      pipe: base?.pipe ?? 'preferred',
      eos: base?.eos ?? 'default'
    }];
  });
  const pickSlot = slots.find(s => s.id === picking);
  const tuneSlot = slots.find(s => s.id === tuning);
  const pipeSlot = slots.find(s => s.id === piping);
  const wordSlot = slots.find(s => s.id === wordsOpen);
  return <section className="control wake-section">
    <style>{CSS}</style>
    <span className="eyebrow">WAKE · WAKE WORDS</span>
    <h1>Say the <em>word.</em></h1>
    <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><span>Wake Word 1</span><HintBtn text={HINTS.wake_words} /></h2>
        <span className="ww-title"><button className={chime ? 'ww-bell' : 'ww-bell off'} aria-pressed={chime} aria-label="Wake chime" onClick={() => setChime(!chime)}>
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M6 8a6 6 0 0 1 12 0c0 7 3 9 3 9H3s3-2 3-9M10.3 21a1.94 1.94 0 0 0 3.4 0" />{!chime && <path d="m3 3 18 18" />}</svg>
        </button><HintBtn text={HINTS.wake_sound} /></span></div>
      <div className="ww-flow">
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Wake Word</span>
          <button className={wordsOpen === slots[0].id ? 'ww-flow-btn open' : 'ww-flow-btn'} onClick={() => setWordsOpen(slots[0].id)}>
            <span>{slots[0].word}</span>
            <ChevronDown size={16} />
          </button>
        </div>
        <span className="ww-flow-arrow" aria-hidden="true"><ArrowRight size={18} /></span>
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Voice Pipeline</span>
          <button className={piping === slots[0].id ? 'ww-flow-btn open' : 'ww-flow-btn'} onClick={() => setPiping(slots[0].id)}>
            <span>{PIPES.find(p => p.id === slots[0].pipe)?.label ?? 'Select...'}</span>
            <ChevronDown size={16} />
          </button>
        </div>
      </div>
      <div className="ww-top">
        {slots[0].tuned && <HintBtn text={HINTS.living_graph} />}
        {!slots[0].tuned && <button className="ww-tune" onClick={() => setTuning(slots[0].id)}>Tune it!</button>}
      </div>
      {slots[0].tuned ? <TouchGraph h={72} marks={WORD_MARKS} cut={slots[0].cut} onCut={v => upd(slots[0].id, {
        cut: v
      })} onTune={() => setTuning(slots[0].id)} aria={`Scores for ${slots[0].word}`} /> : null}
    </article>
    <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><span>Wake Word 2</span><HintBtn text={HINTS.wake_words} /></h2></div>
      <div className="ww-flow">
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Wake Word</span>
          <button className={wordsOpen === slots[1].id ? 'ww-flow-btn open' : 'ww-flow-btn'} onClick={() => setWordsOpen(slots[1].id)}>
            <span>{slots[1].word}</span>
            <ChevronDown size={16} />
          </button>
        </div>
        <span className="ww-flow-arrow" aria-hidden="true"><ArrowRight size={18} /></span>
        <div className="ww-flow-cell">
          <span className="ww-flow-cap">Voice Pipeline</span>
          <button className={piping === slots[1].id ? 'ww-flow-btn open' : 'ww-flow-btn'} onClick={() => setPiping(slots[1].id)}>
            <span>{PIPES.find(p => p.id === slots[1].pipe)?.label ?? 'Select...'}</span>
            <ChevronDown size={16} />
          </button>
        </div>
      </div>
      <div className="ww-top">
        {slots[1].tuned && <HintBtn text={HINTS.living_graph} />}
        {!slots[1].tuned && <button className="ww-tune" onClick={() => setTuning(slots[1].id)}>Tune it!</button>}
      </div>
      {slots[1].tuned ? <TouchGraph h={72} marks={WORD_MARKS} cut={slots[1].cut} onCut={v => upd(slots[1].id, {
        cut: v
      })} onTune={() => setTuning(slots[1].id)} aria={`Scores for ${slots[1].word}`} /> : null}
    </article>
    <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><span>Stop Word</span><HintBtn text={HINTS.stop_word} /></h2></div>
      <button className={`wsw${stopOn ? ' on' : ''}`} style={{
        marginTop: 14,
        marginBottom: 16
      }} role="switch" aria-checked={stopOn} onClick={() => setStopOn(s => !s)}>
        {!stopOn && <span className="wsw-knob" />}
        <span>Stop</span>
        {stopOn && <span className="wsw-knob" />}
      </button>
      {stopOn ? <TouchGraph h={72} marks={STOP_MARKS} cut={stopCut} onCut={setStopCut} onTune={() => {}} aria="Scores for Stop" /> : null}
    </article>
    <article className="ww-card">
      <div className="ww-head"><h2 className="ww-title"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M12 3v12m0 0-4-4m4 4 4-4M4 21h16" /></svg><span>Wake Word Sources</span><HintBtn text={HINTS.wake_sources} /></h2></div>
      {sources.map(s => <div key={s.id}><div className="ww-src"><a href={s.url} target="_blank" rel="noreferrer">{s.url.replace('https://github.com/', '')}</a><small>{s.count} words</small><button className="ww-x" onClick={() => setConfirm(s.id)}>Remove</button></div>
        {confirm === s.id && <div className="ww-confirm"><span>Remove this source? Its words disappear from the picker. Words already installed keep working.</span><div><button className="secondary" onClick={() => setConfirm(null)}>Keep</button><button className="primary" onClick={() => {
              setSources(sources.filter(x => x.id !== s.id));
              setConfirm(null);
            }}>Remove</button></div></div>}</div>)}
      <form className="ww-add" onSubmit={e => {
        e.preventDefault();
        if (!url.trim()) return;
        setSources([...sources, {
          id: `src${Date.now()}`,
          url: url.trim(),
          count: 0
        }]);
        setUrl('');
      }}>
        <input type="url" placeholder="https://github.com/owner/repo" value={url} onChange={e => setUrl(e.target.value)} aria-label="Source URL" /><button className="primary" type="submit">Add</button>
      </form>
    </article>
    {pipeSlot && <PipeDrawer slot={pipeSlot} onChange={p => upd(pipeSlot.id, p)} onClose={() => setPiping(null)} />}
    {wordSlot && <WordDrawer selected={[wordSlot.word, ...slots.filter(s => s.id !== wordSlot.id).map(s => s.word)]} slotId={wordSlot.id} onToggle={w => {
      if (w !== wordSlot.word) upd(wordSlot.id, {
        word: w,
        tuned: false
      });
      setWordsOpen(null);
    }} onClose={() => setWordsOpen(null)} />}
    {tuneSlot && <Tuner word={tuneSlot.word} onClose={() => setTuning(null)} onApply={c => {
      upd(tuneSlot.id, {
        tuned: true,
        cut: c
      });
      setTuning(null);
    }} />}
  </section>;
}