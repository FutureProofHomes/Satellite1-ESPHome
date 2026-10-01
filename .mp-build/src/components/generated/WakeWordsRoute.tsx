/**
 * The Wake Words route: the two word slots plus the Stop word, each wearing its living graph;
 * the pill opens the inline word picker, untuned words carry "Tune it!" into the tuner flow,
 * and the Wake Word Sources card ends the route. Ported from routes/wakewords.jsx with the
 * device's scoring session simulated.
 */
import React, { useEffect, useRef, useState } from 'react';
import { Btn, Card, Check, Chevron, Confirm, Hint, N_WAKE, ni, Select } from './ui';
const HINTS = {
  wake_words: 'Up to two wake words listen at once, each pointed at its own Voice Pipeline. The graph under each word is its last 24 hours.',
  living_graph: 'Solid dots are firings, hollow ones close calls, amber smatter is the room. Drag the knob to move the trigger threshold.',
  wake_sound: 'The chime the device plays when it hears a wake word.',
  stop_word: 'Always-on "Stop": halts timers, alarms and the assistant mid-answer without waking it.',
  voice_pipeline: 'Which Home Assistant Assist pipeline answers this word.',
  wake_sources: 'Where the word list comes from. Add a GitHub repo or a manifest URL to offer more words.'
};
const TEXT = {
  tune_btn: 'Tune it!',
  tn_title: 'Tune \u201c%s\u201d',
  tn_ready: 'Follow the prompts and walk the room saying \u201c%s\u201d. Blue dots are live detections; amber dots are the past 24 hours. You\u2019ll use them to place the firing line - and you can move it any time.',
  tn_start: 'Start',
  tn_near: 'Round 1 of 3 - say \u201c%s\u201d from where you usually are. Twice is plenty.',
  tn_far: 'Round 2 of 3 - now once from across the room.',
  tn_other: 'Round 3 of 3 - hand it to anyone else who uses this device.',
  tn_skip: 'Skip this round',
  tn_ok: 'Fires above %s \u00b7 every room dot in the frost',
  tn_under: ' \u00b7 %s pts under your quietest try',
  tn_high: 'Above your quietest try (%s) - real calls from across the room will be missed.',
  tn_low: 'Amber dots sit past your line - the room has scored this high in the last day. Expect false firings.',
  tn_apply: 'Apply',
  tn_retune: 'Re-Tune',
  tn_clear: 'Clear history',
  tn_pm_one: 'Muted 1 nearby satellite for this session (Kitchen Satellite)',
  cancel: 'Cancel',
  untuned: 'Untuned - firing on the model default. A two-minute tune fits it to this room.',
  ww_on_other: 'on the other word'
};
const N_WSRC = ni(<>
    <path d="M8 2.5v7.2" />
    <path d="M5.2 7.2 8 10l2.8-2.8" />
    <path d="M2.8 10.6v2.2h10.4v-2.2" />
  </>);

/* ------------------------------------------------------------------ */
/* The chime bell                                                      */
/* ------------------------------------------------------------------ */

function BellIcon({
  slash
}: {
  slash?: boolean;
}) {
  return <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
      <path d="M13.73 21a2 2 0 0 1-3.46 0" />
      {slash && <path d="M4.5 3.5l15 17" />}
    </svg>;
}
function ChimeBell() {
  const [on, setOn] = useState(true);
  return <span className="chimewrap">
      <Hint text={HINTS.wake_sound} />
      <button className={`wsw${on ? ' on' : ''}`} role="switch" aria-checked={on} aria-label="Wake chime" onClick={() => setOn(!on)}>
        {!on && <span className="wsw-knob" />}
        <span className="wsw-bell">
          <BellIcon slash={!on} />
        </span>
        {on && <span className="wsw-knob" />}
      </button>
    </span>;
}

/* ------------------------------------------------------------------ */
/* The living graph / tuner graph                                      */
/* ------------------------------------------------------------------ */

type Mark = {
  kind: 'fire' | 'near' | 'room' | 'you';
  c: number;
  y: number;
  label?: string;
  ripple?: boolean;
};
const CUT_MIN = 40;
const CUT_MAX = 95;

/**
 * The graph both surfaces share: paper grid, the story dots, the tinted accepted region and the
 * draggable threshold knob when a cut rides in. `h` picks the row strip or the tuner canvas.
 */
function TouchGraph({
  marks,
  cut,
  onCut,
  h = 96,
  aria
}: {
  marks: Mark[];
  cut?: number | null;
  onCut?: (v: number) => void;
  h?: number;
  aria: string;
}) {
  const topY = 10;
  const axisY = h - 18;
  const gx = (c: number) => 12 + c / 100 * 276;
  const svgRef = useRef<SVGSVGElement>(null);
  const [held, setHeld] = useState(false);
  const hasKnob = cut != null && !!onCut;
  const cutX = cut != null ? gx(cut) : 0;
  const knobY = topY + (axisY - topY) / 2 - 14;
  const knobH = 28;
  const pctFromEvent = (e: React.PointerEvent) => {
    const r = svgRef.current!.getBoundingClientRect();
    const frac = (e.clientX - r.left) / r.width * 300;
    return Math.round(Math.max(CUT_MIN, Math.min(CUT_MAX, (frac - 12) / 276 * 100)));
  };
  const dot = (m: Mark, i: number) => {
    if (m.kind === 'room') return <g key={`m${i}`}>
          <circle cx={gx(m.c)} cy={m.y} r="6" className="lg-halo-w" />
          {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="8.5" className="lg-rip lg-rip-room" />}
          <circle cx={gx(m.c)} cy={m.y} r="3.2" className="lg-room" />
        </g>;
    if (m.kind === 'near') return <g key={`m${i}`}>
          <circle cx={gx(m.c)} cy={m.y} r="7" className="lg-halo-w" />
          <circle cx={gx(m.c)} cy={m.y} r="4" className="lg-near" />
        </g>;
    if (m.kind === 'you') return <g key={`m${i}`}>
          <circle cx={gx(m.c)} cy={m.y} r="8.5" className="lg-halo-a" />
          {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="10" className="lg-rip" />}
          {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="15" className="lg-rip lg-rip2" />}
          <circle cx={gx(m.c)} cy={m.y} r="5" className="lg-you" />
          {m.label && <text x={Math.min(Math.max(gx(m.c), 26), 274)} y={m.y - 10} textAnchor="middle" className="lg-al">
              {m.label}
            </text>}
        </g>;
    return <g key={`m${i}`}>
        <circle cx={gx(m.c)} cy={m.y} r="7.5" className="lg-halo-a" />
        {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="9" className="lg-rip" />}
        {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="13.5" className="lg-rip lg-rip2" />}
        <circle cx={gx(m.c)} cy={m.y} r="4.3" className="lg-fire" />
      </g>;
  };
  return <svg ref={svgRef} viewBox={`0 0 300 ${h}`} className="lg" role="img" aria-label={aria} style={hasKnob ? {
    touchAction: 'none'
  } : undefined} onPointerDown={hasKnob ? e => {
    setHeld(true);
    (e.currentTarget as SVGSVGElement).setPointerCapture(e.pointerId);
    onCut!(pctFromEvent(e));
  } : undefined} onPointerMove={hasKnob ? e => held && onCut!(pctFromEvent(e)) : undefined} onPointerUp={hasKnob ? () => setHeld(false) : undefined} onPointerCancel={hasKnob ? () => setHeld(false) : undefined}>
      <rect className="lg-track" x="12" y={topY} width="276" height={axisY - topY} rx="7" />
      {[10, 20, 30, 40, 50, 60, 70, 80, 90].map(c => <line key={`gv${c}`} x1={gx(c)} y1={topY} x2={gx(c)} y2={axisY} className={`lg-paper${c % 25 === 0 ? ' mj' : ''}`} />)}
      {Array.from({
      length: Math.floor((axisY - topY) / 22)
    }, (_, k) => topY + 22 * (k + 1)).map(y => <line key={`gh${y}`} x1="12" y1={y} x2="288" y2={y} className="lg-paper" />)}
      {cut != null && <>
          <rect className="lg-tint" x={cutX} y={topY} width={Math.max(0, 288 - cutX)} height={axisY - topY} rx="7" />
          <rect className="lg-veil" x="12" y={topY} width={Math.max(0, cutX - 12)} height={axisY - topY} rx="7" />
        </>}

      {marks.map(dot)}

      {hasKnob && <g className="lg-knobg">
          <line x1={cutX} y1={topY - 3} x2={cutX} y2={axisY + 3} className={`lg-stem${held ? ' held' : ''}`} />
          {held && <rect x={cutX - 11} y={knobY - 4} width="22" height={knobH + 8} rx="7" className="lg-heldring" />}
          <rect x={cutX - 7} y={knobY} width="14" height={knobH} rx="4.5" className={`lg-knob${held ? ' held' : ''}`} />
          {[-2.5, 2.5].map(dx => <line key={dx} x1={cutX + dx} y1={knobY + knobH * 0.28} x2={cutX + dx} y2={knobY + knobH * 0.72} className="lg-grip" />)}
        </g>}

      <g className="lg-ticks">
        <line x1="12" y1={axisY + 5} x2="288" y2={axisY + 5} className="lg-axis" />
        {[25, 50, 75].map(c => <line key={c} x1={gx(c)} y1={axisY + 2} x2={gx(c)} y2={axisY + 8} className="lg-axis" />)}
      </g>
    </svg>;
}

/* ------------------------------------------------------------------ */
/* The word picker                                                     */
/* ------------------------------------------------------------------ */

type Entry = {
  word: string;
  langs: string[];
  size: number;
  unverified?: boolean;
};
type Group = {
  key: string;
  label: string;
  entries: Entry[];
};
const LIBRARY: Group[] = [{
  key: 'esphome',
  label: 'esphome/micro-wake-word-models',
  entries: [{
    word: 'Okay Nabu',
    langs: ['en'],
    size: 282624
  }, {
    word: 'Hey Jarvis',
    langs: ['en'],
    size: 273408
  }, {
    word: 'Hey Mycroft',
    langs: ['en'],
    size: 268288
  }, {
    word: 'Alexa',
    langs: ['en'],
    size: 301056
  }, {
    word: 'Computer',
    langs: ['en'],
    size: 265216
  }, {
    word: 'Okay Casita',
    langs: ['es'],
    size: 270336
  }]
}, {
  key: 'fph',
  label: 'FutureProofHomes/wakewords',
  entries: [{
    word: 'Hey Satellite',
    langs: ['en'],
    size: 276480
  }, {
    word: 'Bonjour Maison',
    langs: ['fr'],
    size: 271360,
    unverified: true
  }, {
    word: 'Hallo Haus',
    langs: ['de'],
    size: 269312,
    unverified: true
  }]
}];
const canSpeak = () => typeof window !== 'undefined' && 'speechSynthesis' in window;
function SpeakBtn({
  word
}: {
  word: string;
}) {
  if (!canSpeak()) return null;
  return <button className="ww-speak" aria-label={`Say "${word}"`} onClick={e => {
    e.stopPropagation();
    try {
      window.speechSynthesis.speak(new SpeechSynthesisUtterance(word));
    } catch {
      /* a browser without voices just stays quiet */
    }
  }}>
      <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <path d="M3 6.4h2L8 4v8l-3-2.4H3z" />
        <path d="M10.6 5.8a3.1 3.1 0 0 1 0 4.4" />
      </svg>
    </button>;
}
function PickRow({
  entry,
  selected,
  disabled,
  note,
  onPick
}: {
  entry: Entry;
  selected: boolean;
  disabled?: boolean;
  note?: string | null;
  onPick: () => void;
}) {
  return <div className={`tree-p${disabled ? ' tree-off' : ''}`}>
      <Check state={selected ? 'on' : 'off'} disabled={disabled} onClick={onPick} label={entry.word} />
      <button className="ww-name grow" disabled={disabled} onClick={onPick}>
        &ldquo;{entry.word}&rdquo;
      </button>
      {entry.langs.map(l => <span className="tree-n" key={l}>
          {l}
        </span>)}
      <span className="tree-n">{`${Math.round(entry.size / 1024)} KB`}</span>
      {entry.unverified && <span className="tree-n">unverified</span>}
      {note && <span className="tree-n">{note}</span>}
      <SpeakBtn word={entry.word} />
    </div>;
}
function Picker({
  current,
  otherWord,
  onPick
}: {
  current: string;
  otherWord: string | null;
  onPick: (word: string | null) => void;
}) {
  const [q, setQ] = useState('');
  const [lang, setLang] = useState('');
  const [open, setOpen] = useState<Set<string>>(() => new Set([LIBRARY[0].key]));
  const langs = new Set<string>();
  for (const g of LIBRARY) for (const e of g.entries) for (const l of e.langs) langs.add(l);
  const match = (e: Entry) => (!q || e.word.toLowerCase().includes(q.toLowerCase())) && (!lang || e.langs.includes(lang));
  const searching = q.trim().length > 0;
  return <div className="tree ww-pick">
      <div className="ww-tools">
        <input className="ww-search" type="search" placeholder="Search wake words" value={q} onInput={e => setQ((e.target as HTMLInputElement).value)} />
        <select className="sel sm" value={lang} onChange={e => setLang(e.target.value)}>
          <option value="">All languages</option>
          {[...langs].sort().map(l => <option key={l} value={l}>
              {l}
            </option>)}
        </select>
      </div>

      {LIBRARY.map(g => {
      const entries = g.entries.filter(match);
      const expanded = searching ? entries.length > 0 : open.has(g.key);
      if (searching && !entries.length) return null;
      const toggle = () => setOpen(prev => {
        const next = new Set(prev);
        if (next.has(g.key)) next.delete(g.key);else next.add(g.key);
        return next;
      });
      return <div key={g.key} className="tree-a">
            <div className="tree-h">
              <button className="caret tree-x" aria-label={expanded ? 'Collapse' : 'Expand'} aria-expanded={expanded} onClick={toggle}>
                <Chevron down={expanded} />
              </button>
              <button className="ww-name grow" onClick={toggle}>
                {g.label}
              </button>
              <span className="tree-n">{g.entries.length}</span>
            </div>
            {expanded && <div className="tree-ps">
                {entries.map(e => {
            const selected = current.toLowerCase() === e.word.toLowerCase();
            const held = !!otherWord && e.word.toLowerCase() === otherWord.toLowerCase();
            return <PickRow key={e.word} entry={e} selected={selected} disabled={held} note={held ? TEXT.ww_on_other : null} onPick={() => onPick(selected ? null : e.word)} />;
          })}
              </div>}
          </div>;
    })}
    </div>;
}

/** The inline expansion the pill's chevron opens: the picker in a sunken well where the graph sits. */
function InlinePicker(props: {
  current: string;
  otherWord: string | null;
  onPick: (word: string | null) => void;
}) {
  return <div className="ww-inline">
      <Picker {...props} />
      <p className="dim sm ww-inline-foot">More words come from the sources in the card below.</p>
    </div>;
}

/* ------------------------------------------------------------------ */
/* One word's row                                                      */
/* ------------------------------------------------------------------ */

function WordRow({
  first,
  brk,
  word,
  isStop,
  stopOn,
  onStopToggle,
  pillOpen,
  onPill,
  tuneBtn,
  onTune,
  live,
  right,
  graph,
  picker,
  sub,
  pipeline
}: {
  first?: boolean;
  brk?: boolean;
  word: string;
  isStop?: boolean;
  stopOn?: boolean;
  onStopToggle?: () => void;
  pillOpen?: boolean;
  onPill?: () => void;
  tuneBtn?: boolean;
  onTune?: () => void;
  live?: boolean;
  right?: string;
  graph?: React.ReactNode;
  picker?: React.ReactNode;
  sub?: string | null;
  pipeline?: React.ReactNode;
}) {
  return <div className={`mb${first ? ' first' : ''}${brk ? ' brk' : ''}`}>
      <div className="mb-t">
        {isStop ? <>
            <button className={`wsw${stopOn ? ' on' : ''}`} role="switch" aria-checked={stopOn} onClick={onStopToggle}>
              {!stopOn && <span className="wsw-knob" />}
              &ldquo;{word}&rdquo;
              {stopOn && <span className="wsw-knob" />}
            </button>
            <Hint text={HINTS.stop_word} />
          </> : <button className="wpill" aria-expanded={pillOpen} onClick={onPill}>
            &ldquo;{word}&rdquo;
            <Chevron down={pillOpen} cls="caret-s" />
          </button>}
        {live ? <span className="mb-live">
            <span className="mb-live-dot" />
            Listening
          </span> : <span className="mb-right">{right || ''}</span>}
        {tuneBtn && <span className="tunebtn-halo">
            <button className="tunebtn" onClick={onTune}>
              {TEXT.tune_btn}
            </button>
          </span>}
      </div>
      {pillOpen ? picker : graph}
      {sub && <p className="mb-sub">{sub}</p>}
      {pipeline}
    </div>;
}
function PipelineRow({
  value,
  onChange
}: {
  value: string;
  onChange: (v: string) => void;
}) {
  return <div className="mb-pipe">
      <span>Voice Pipeline</span>
      <Hint text={HINTS.voice_pipeline} />
      <span className="grow" />
      <Select value={value} options={[['preferred', 'Preferred'], 'Home Assistant Cloud', 'Local Whisper']} onChange={onChange} />
    </div>;
}

/* ------------------------------------------------------------------ */
/* The tuner flow                                                      */
/* ------------------------------------------------------------------ */

const ROOM_SMATTER: Mark[] = [{
  kind: 'room',
  c: 22,
  y: 48
}, {
  kind: 'room',
  c: 31,
  y: 84
}, {
  kind: 'room',
  c: 18,
  y: 104
}, {
  kind: 'room',
  c: 27,
  y: 64
}, {
  kind: 'room',
  c: 35,
  y: 96
}];
const ATTEMPT_PLAN = [{
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
function TunerFlow({
  word,
  onApply,
  onClose
}: {
  word: string;
  onApply: (cut: number) => void;
  onClose: () => void;
}) {
  const [phase, setPhase] = useState<'ready' | 'voice' | 'place'>('ready');
  const [attempts, setAttempts] = useState<{
    score: number;
    round: string;
  }[]>([]);
  const [cut, setCut] = useState(55);

  /* The rounds: an attempt "lands" every few seconds once Start is pressed, then placement seeds
     the knob inside the gap between the room and the quietest try. */
  useEffect(() => {
    if (phase !== 'voice') return;
    const t = setInterval(() => {
      setAttempts(a => {
        if (a.length >= ATTEMPT_PLAN.length) return a;
        return [...a, ATTEMPT_PLAN[a.length]];
      });
    }, 2800);
    return () => clearInterval(t);
  }, [phase]);
  useEffect(() => {
    if (phase !== 'voice' || attempts.length < 4) return;
    const floor = Math.min(...attempts.map(a => a.score));
    const room = 35;
    setCut(Math.max(CUT_MIN, Math.min(CUT_MAX, Math.round(room + 0.6 * (floor - room)))));
    setPhase('place');
  }, [phase, attempts]);
  const attemptDots = (): Mark[] => attempts.map((a, k) => ({
    kind: 'you' as const,
    c: a.score,
    y: 42 + k * 26,
    label: a.round,
    ripple: k === attempts.length - 1
  }));
  const prompt = attempts.length < 2 ? TEXT.tn_near.replace('%s', word) : attempts.length < 3 ? TEXT.tn_far : TEXT.tn_other;
  const readout = () => {
    const floorC = Math.min(...attempts.map(a => a.score));
    const roomC = 35;
    if (cut >= floorC - 2) return {
      tone: 'warn',
      text: TEXT.tn_high.replace('%s', `${floorC}%`)
    };
    if (cut <= roomC) return {
      tone: 'err',
      text: TEXT.tn_low
    };
    return {
      tone: 'dim',
      text: `${TEXT.tn_ok.replace('%s', `${cut}%`)}${TEXT.tn_under.replace('%s', String(floorC - cut))}.`
    };
  };
  return <div className="tn-flow">
      {phase === 'ready' && <>
          <TouchGraph h={150} marks={ROOM_SMATTER} aria="The room's last 24 hours" />
          <p className="dim sm">{TEXT.tn_ready.replace('%s', word)}</p>
          <div className="row-actions">
            <Btn solid onClick={() => {
          setAttempts([]);
          setPhase('voice');
        }}>
              {TEXT.tn_start}
            </Btn>
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>}

      {phase === 'voice' && <>
          <TouchGraph h={150} marks={[...ROOM_SMATTER, ...attemptDots()]} aria="Live detections" />
          <p className="dim sm">
            {prompt} ({Math.min(attempts.length, 4)} / 4)
          </p>
          <p className="dim sm">{TEXT.tn_pm_one}</p>
          <div className="row-actions">
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>}

      {phase === 'place' && <>
          <TouchGraph h={150} marks={[...ROOM_SMATTER, ...attemptDots()]} cut={cut} onCut={setCut} aria="Place the firing line" />
          <p className={`sm t-${readout().tone}`}>{readout().text}</p>
          <div className="row-actions one4">
            <Btn onClick={() => {
          setAttempts([]);
          setPhase('voice');
        }}>
              {TEXT.tn_retune}
            </Btn>
            <Btn danger onClick={() => {}}>
              {TEXT.tn_clear}
            </Btn>
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
            <Btn solid onClick={() => onApply(cut)}>
              {TEXT.tn_apply}
            </Btn>
          </div>
        </>}
    </div>;
}

/* ------------------------------------------------------------------ */
/* The Wake Word Sources card                                          */
/* ------------------------------------------------------------------ */

function SourcesCard() {
  const [sources, setSources] = useState([{
    url: 'https://github.com/esphome/micro-wake-word-models',
    label: 'esphome/micro-wake-word-models',
    words: 6
  }, {
    url: 'https://github.com/FutureProofHomes/wakewords',
    label: 'FutureProofHomes/wakewords',
    words: 3
  }]);
  const [draft, setDraft] = useState('');
  return <Card title="Wake Word Sources" icon={N_WSRC} hint={HINTS.wake_sources}>
      {sources.map(s => <div key={s.url} className="ws-row">
          <div className="ws-name">
            <a href={s.url} target="_blank" rel="noopener noreferrer" onClick={e => e.preventDefault()}>
              {s.label}
            </a>
            <span className="dim sm">{s.words} words</span>
          </div>
          <Confirm label={'\u2715'} title="Remove this source?" body="Its words disappear from the picker. Words already installed keep working." confirmLabel="Remove" onConfirm={() => setSources(sources.filter(x => x.url !== s.url))} />
        </div>)}

      <div className="ws-add">
        <input className="ww-search" type="url" placeholder={'GitHub repo or manifest URL\u2026'} value={draft} onInput={e => setDraft((e.target as HTMLInputElement).value)} />
        <Btn onClick={() => {
        const url = draft.trim();
        if (!url || sources.some(s => s.url === url)) return;
        setSources([...sources, {
          url,
          label: url.split('/').slice(3, 5).join('/') || url,
          words: 0
        }]);
        setDraft('');
      }}>
          Add
        </Btn>
      </div>
    </Card>;
}

/* ------------------------------------------------------------------ */

const NABU_MARKS: Mark[] = [{
  kind: 'fire',
  c: 88,
  y: 26,
  ripple: true
}, {
  kind: 'fire',
  c: 79,
  y: 48
}, {
  kind: 'fire',
  c: 92,
  y: 62
}, {
  kind: 'near',
  c: 61,
  y: 38
}, {
  kind: 'near',
  c: 55,
  y: 58
}, {
  kind: 'room',
  c: 22,
  y: 30
}, {
  kind: 'room',
  c: 31,
  y: 52
}, {
  kind: 'room',
  c: 18,
  y: 64
}, {
  kind: 'room',
  c: 27,
  y: 42
}];
const STOP_MARKS: Mark[] = [{
  kind: 'fire',
  c: 86,
  y: 36
}, {
  kind: 'room',
  c: 20,
  y: 52
}];
type Slot = {
  word: string;
  tuned: boolean;
  cut: number;
  pipe: string;
};
export function WakeWordsRoute() {
  const [slots, setSlots] = useState<Record<number, Slot>>({
    0: {
      word: 'Okay Nabu',
      tuned: true,
      cut: 72,
      pipe: 'preferred'
    },
    1: {
      word: 'Hey Jarvis',
      tuned: false,
      cut: 68,
      pipe: 'Local Whisper'
    }
  });
  const [stopOn, setStopOn] = useState(true);
  const [stopCut, setStopCut] = useState(75);
  const [picking, setPicking] = useState<Set<number>>(new Set());
  const [tuning, setTuning] = useState<{
    i: number;
    word: string;
  } | null>(null);
  const setSlot = (i: number, patch: Partial<Slot>) => setSlots(s => ({
    ...s,
    [i]: {
      ...s[i],
      ...patch
    }
  }));
  const togglePick = (i: number) => setPicking(p => {
    const next = new Set(p);
    if (next.has(i)) next.delete(i);else next.add(i);
    return next;
  });
  const wordRow = (i: number, first: boolean) => {
    const slot = slots[i];
    const other = slots[i === 0 ? 1 : 0]?.word || null;
    return <WordRow key={i} first={first} word={slot.word} live pillOpen={picking.has(i)} onPill={() => togglePick(i)} tuneBtn={!slot.tuned && !picking.has(i)} onTune={() => setTuning({
      i,
      word: slot.word
    })} graph={slot.tuned ? <TouchGraph marks={i === 0 ? NABU_MARKS : [{
      kind: 'fire',
      c: 83,
      y: 40
    }, {
      kind: 'room',
      c: 25,
      y: 56
    }]} cut={slot.cut} onCut={v => setSlot(i, {
      cut: v
    })} aria={`${slot.word}'s last 24 hours`} /> : null} picker={<InlinePicker current={slot.word} otherWord={other} onPick={word => {
      if (word) setSlot(i, {
        word,
        tuned: false
      });
      togglePick(i);
    }} />} sub={slot.tuned || picking.has(i) ? null : TEXT.untuned} pipeline={<PipelineRow value={slot.pipe} onChange={v => setSlot(i, {
      pipe: v
    })} />} />;
  };
  return <>
      {tuning ? <Card title={TEXT.tn_title.replace('%s', tuning.word)} icon={N_WAKE} hint={HINTS.living_graph}>
          <TunerFlow word={tuning.word} onApply={cut => {
        setSlot(tuning.i, {
          tuned: true,
          cut
        });
        setTuning(null);
      }} onClose={() => setTuning(null)} />
        </Card> : <Card title="Wake Words" icon={N_WAKE} hint={HINTS.wake_words} right={<ChimeBell />}>
          {wordRow(0, true)}
          {wordRow(1, false)}
          <WordRow brk word="Stop" isStop stopOn={stopOn} onStopToggle={() => setStopOn(!stopOn)} live={stopOn} right={stopOn ? '' : 'off'} graph={stopOn ? <TouchGraph marks={STOP_MARKS} cut={stopCut} onCut={setStopCut} aria="Stop's last 24 hours" /> : null} sub={stopOn ? null : 'Timers and alarms keep ringing until the assistant is asked to stop them.'} />
        </Card>}

      <SourcesCard />
    </>;
}