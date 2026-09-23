/**
 * The Wake Words route, rebuilt around the Living Graph (owner's v2 redesign, September 2026; the
 * `wake-words-tuner-v2` canvas is the design of record and this file follows it screen for
 * screen). One idea carries the page: the graph stops being a report you read after tuning and
 * becomes the single object you tune, watch, and correct. Collapsed on the word's row it is the
 * standing record - two color worlds split at the threshold knob, every real firing landing as a
 * dot at its measured confidence for a rolling 24 hours. Expanded, the same drawing is the tuner.
 *
 * The row model (owner calls, September 2026): no chevron menu, no modal, no "tuned" badge - the
 * graph's presence IS the tuned state. The pill's own chevron expands the word picker inline where
 * the graph sits; the knob reopens placement; an untuned word carries the one loud button on the
 * route ("Tune it!"); Stop's pill is itself an iOS-style switch; the wake chime is a bell toggle
 * in the card's header, which retired the Wake Word Settings card. The Recent detections card is
 * gone - each word carries its own record, and tapping a dot tells its story.
 *
 * The device is the validator and the source of truth - the browser only enumerates sources
 * (lib/wakesources.js) and polls the swap it asked for. A failed download leaves the previous word
 * listening, and the row says so.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { HINTS, TEXT, WW_ERR } from "../copy.js";
import {
  NO_WAKE_WORD,
  PIPELINE_PREFERRED,
  STOP_SLOT,
  entity,
  haBlocked,
  haSyncOnce,
  haTooOld,
  pathFor,
  post,
  requestJson,
  useAssist,
  useWakeSlots,
} from "../lib/device.js";
import { holdPeerMutes, keepPeerMutes, releasePeerMutes } from "../lib/peermute.js";
import {
  DEFAULT_SOURCES,
  REQUEST_WORD_URL,
  TRAIN_URL,
  canSpeak,
  enumerateSource,
  readSources,
  speak,
  writeSources,
} from "../lib/wakesources.js";
import { Check } from "../tree.jsx";
import { Btn, Card, Chevron, Confirm, Hint, Missing, N_WAKE, Select, ni } from "../ui.jsx";

/** Sources: an arrow arriving into a tray - the list is fetched from elsewhere. */
const N_WSRC = ni(
  <>
    <path d="M8 3v6" />
    <path d="M5.4 6.6 8 9.2l2.6-2.6" />
    <path d="M3 11.4V13h10v-1.6" />
  </>
);

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const isUrl = (s) => /^https?:\/\//.test(s || "");

/** Quantized probability (0-255) as the percentage people read. */
const pct = (v) => `${Math.round((v / 255) * 100)}%`;
const pctN = (v) => Math.round((v / 255) * 100);

/** The stop word displays capitalized everywhere (owner call, September 2026); the model reports
 *  it lowercase. Every other word arrives already display-cased by the loader. */
const showWord = (w) => (w === "stop" ? "Stop" : w);

/** The probe floor the firmware drops to during a session (WL_TUNE_FLOOR): where a VAD-refused
 *  attempt draws, since its log line carries no score of its own. */
const TUNE_FLOOR = 107;

/** A tune event's effective score: the PEAK single-frame probability. Hardware-found (Dev12,
 *  September 22 2026): the engine resets the probability window the instant a detection fires, so
 *  during a floored session the max windowed mean is truncated at the floor. The peak is the one
 *  number the reset cannot touch; the placement margin (-8 quantized at the clamp) covers peak's
 *  overestimate of the steady-state mean. `ev` is [peak, avg, vad, msAgo, mm?]. */
const evScore = (e) => e[0];

/** The rolling window every dot lives (and fades) inside. */
const DAY_MS = 24 * 60 * 60 * 1000;

const ago = (ms) => {
  if (ms < 60000) return TEXT.det_just_now;
  if (ms < 3600000) return `${Math.round(ms / 60000)}m ago`;
  return `${Math.round(ms / 3600000)}h ago`;
};

/* ------------------------------------------------------------------ */
/* The picker: the grouped, searchable list (unchanged grammar).       */
/* It expands INLINE under a word's pill now - the modal sheet is      */
/* retired (owner call: no modal anywhere on this route).              */
/* ------------------------------------------------------------------ */

const SEARCH_CAP = 60;

function SpeakBtn({ word }) {
  if (!canSpeak()) return null;
  return (
    <button
      class="ww-speak"
      aria-label={`Say "${word}"`}
      onClick={(e) => {
        e.stopPropagation();
        speak(word);
      }}
    >
      <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
        <path d="M3 6.4h2L8 4v8l-3-2.4H3z" />
        <path d="M10.6 5.8a3.1 3.1 0 0 1 0 4.4" />
      </svg>
    </button>
  );
}

/** One pickable word, on the /audio tree's row grammar. `note` names why a disabled row is
 *  disabled ("on the other slot"), which a grey row alone never said. */
function PickRow({ entry, selected, disabled, note, onPick }) {
  return (
    <div class={`tree-p${disabled ? " tree-off" : ""}`}>
      <Check state={selected ? "on" : "off"} disabled={disabled} onClick={() => onPick(entry)} label={entry.word} />
      <button class="ww-name grow" disabled={disabled} onClick={() => onPick(entry)}>
        &ldquo;{entry.word}&rdquo;
      </button>
      {(entry.langs || []).map((l) => (
        <span class="tree-n" key={l}>
          {l}
        </span>
      ))}
      {entry.size > 0 && <span class="tree-n">{`${Math.round(entry.size / 1024)} KB`}</span>}
      {entry.unverified && <span class="tree-n">{TEXT.ww_unverified}</span>}
      {entry.dup && entry.ver && <span class="tree-n">{entry.ver}</span>}
      {note && <span class="tree-n">{note}</span>}
      <SpeakBtn word={entry.word} />
    </div>
  );
}

/** The grouped list: sticky search + language filter, .tree-a groups, rows under a guide line. */
function Picker({ groups, current, otherWord, busy, onPick }) {
  const [q, setQ] = useState("");
  const [lang, setLang] = useState("");
  const [open, setOpen] = useState(() => new Set([groups[0]?.key]));

  const langs = new Set();
  for (const g of groups) for (const e of g.entries || []) for (const l of e.langs || []) langs.add(l);

  const match = (e) =>
    (!q || e.word.toLowerCase().includes(q.toLowerCase())) && (!lang || (e.langs || []).includes(lang));
  const heldByOther = (e) => !!otherWord && e.word.toLowerCase() === otherWord.toLowerCase();

  const searching = q.trim().length > 0;
  let shown = 0;
  let hidden = 0;

  return (
    <div class="tree ww-pick">
      <div class="ww-tools">
        <input
          class="ww-search"
          type="search"
          placeholder={TEXT.ww_search}
          value={q}
          onInput={(e) => setQ(e.currentTarget.value)}
        />
        {langs.size > 1 && (
          <select class="sel sm" value={lang} onChange={(e) => setLang(e.currentTarget.value)}>
            <option value="">{TEXT.ww_all_langs}</option>
            {[...langs].sort().map((l) => (
              <option key={l} value={l}>
                {l}
              </option>
            ))}
          </select>
        )}
      </div>

      {groups.map((g) => {
        const entries = (g.entries || []).filter(match);
        const expanded = searching ? entries.length > 0 : open.has(g.key);
        if (searching && !entries.length && !g.loading && !g.error) return null;
        let rows = entries;
        if (searching && shown + rows.length > SEARCH_CAP) {
          rows = rows.slice(0, Math.max(0, SEARCH_CAP - shown));
          hidden += entries.length - rows.length;
        }
        shown += rows.length;
        return (
          <div key={g.key} class="tree-a">
            <div class="tree-h">
              <button
                class="caret tree-x"
                aria-label={expanded ? "Collapse" : "Expand"}
                aria-expanded={expanded ? "true" : "false"}
                onClick={() =>
                  setOpen((prev) => {
                    const next = new Set(prev);
                    next.has(g.key) ? next.delete(g.key) : next.add(g.key);
                    return next;
                  })
                }
              >
                <Chevron down={expanded} />
              </button>
              <button
                class="ww-name grow"
                onClick={() =>
                  setOpen((prev) => {
                    const next = new Set(prev);
                    next.has(g.key) ? next.delete(g.key) : next.add(g.key);
                    return next;
                  })
                }
              >
                {g.label}
              </button>
              {g.entries && <span class="tree-n">{g.entries.length}</span>}
            </div>
            {expanded && (
              <div class="tree-ps">
                {g.loading && <p class="dim sm ww-note">{TEXT.ww_source_loading}</p>}
                {g.error && <p class="dim sm ww-note">{TEXT.ww_source_failed}</p>}
                {rows.map((e) => {
                  const selected = current === (e.spec || e.url);
                  const held = heldByOther(e);
                  return (
                    <PickRow
                      key={e.url || e.spec}
                      entry={e}
                      selected={selected}
                      disabled={busy || held}
                      note={held ? TEXT.ww_on_other : null}
                      onPick={() => onPick(selected ? null : e)}
                    />
                  );
                })}
              </div>
            )}
          </div>
        );
      })}
      {hidden > 0 && (
        <p class="dim sm ww-note">
          {hidden} {TEXT.ww_more}
        </p>
      )}
    </div>
  );
}

/** The inline expansion the pill's chevron opens: the picker in a sunken well where the graph
 *  sits, with the sources footer under it. */
function InlinePicker({ groups, current, otherWord, busy, onPick }) {
  return (
    <div class="ww-inline">
      <Picker groups={groups} current={current} otherWord={otherWord} busy={busy} onPick={onPick} />
      <p class="dim sm ww-inline-foot">
        {TEXT.ws_footer_q}
        <a href={REQUEST_WORD_URL} target="_blank" rel="noopener">
          {TEXT.ws_request}
        </a>
        {TEXT.ws_or}
        <a href={TRAIN_URL} target="_blank" rel="noopener">
          {TEXT.ws_train}
        </a>
        .
      </p>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The Living Graph: one SVG, two sizes, two color worlds              */
/* ------------------------------------------------------------------ */

/** viewBox x for a confidence percentage. 300 wide, 12px margins. */
const gx = (c) => 12 + 2.76 * c;

/** Deterministic vertical jitter, purely for legibility - the x position is the datum. */
const jit = (i, base, span) => base + ((i * 37) % span);

/** How opaque a dot is at its age: newest nearly solid, a day old nearly gone. The rolling window
 *  visually empties itself. */
const fade = (age) => 0.32 + 0.68 * (1 - Math.min(Math.max(age ?? 0, 0), 1));

/**
 * The graph itself, render only (TouchGraph below owns the pointer). Everything it knows is drawn
 * once - then split at the threshold into two worlds, one color each: the left copy clipped,
 * gaussian-blurred and sunk under a translucent amber veil (dots hidden under the color - ignored,
 * literally out of focus); the right copy crisp on a whisper of accent blue. The two colors are
 * the entire explanation, no legend for the split.
 *
 * marks: [{kind: "fire"|"near"|"room", c, y, age, ripple}] - solid firings, hollow close calls,
 * the room's faint hourly smatter (tuner views only). dots: [{c, y, label}] - live tuning
 * attempts. vads: [{y}] - attempts the voice gate refused, hollow at the probe floor.
 */
function LivingGraph({ gid, h = 56, axis, cutC, held, marks = [], dots = [], vads = [], pop, aria }) {
  const topY = 5;
  const axisY = axis ? h - 14 : h - 5;
  const cutX = cutC != null ? gx(cutC) : null;
  const knobH = Math.max(20, Math.min(32, (axisY - topY) * 0.66));
  const knobY = topY + (axisY - topY) / 2 - knobH / 2;

  /* Every dot wears a soft halo of its own color under it (owner call, September 22 2026) - the
     lift that separates it from the track without a border. */
  const mark = (m, i) => {
    if (m.kind === "near") {
      return (
        <g key={`m${i}`} opacity={fade(m.age)}>
          <circle cx={gx(m.c)} cy={m.y} r="7" class="lg-halo-w" />
          <circle cx={gx(m.c)} cy={m.y} r="4" class="lg-near" />
        </g>
      );
    }
    if (m.kind === "room") {
      return (
        <g key={`m${i}`} opacity={fade(m.age) * 0.85}>
          <circle cx={gx(m.c)} cy={m.y} r="6" class="lg-halo-w" />
          {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="8.5" class="lg-rip lg-rip-room" />}
          <circle cx={gx(m.c)} cy={m.y} r="3.2" class="lg-room" />
        </g>
      );
    }
    return (
      <g key={`m${i}`} opacity={fade(m.age)}>
        <circle cx={gx(m.c)} cy={m.y} r="7.5" class="lg-halo-a" />
        {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="9" class="lg-rip" />}
        {m.ripple && <circle cx={gx(m.c)} cy={m.y} r="13.5" class="lg-rip lg-rip2" />}
        <circle cx={gx(m.c)} cy={m.y} r="4.3" class="lg-fire" />
      </g>
    );
  };

  const content = (
    <g>
      {marks.map(mark)}
      {dots.map((d, i) => (
        <g key={`d${i}`}>
          <circle cx={gx(d.c)} cy={d.y} r="8.5" class="lg-halo-a" />
          {d.ripple && <circle cx={gx(d.c)} cy={d.y} r="10" class="lg-rip" />}
          {d.ripple && <circle cx={gx(d.c)} cy={d.y} r="15" class="lg-rip lg-rip2" />}
          <circle cx={gx(d.c)} cy={d.y} r="5" class="lg-you" />
          {d.label && (
            /* Clamped so a near-100% attempt's label never clips the edge (owner's screenshot). */
            <text x={Math.min(Math.max(gx(d.c), 26), 274)} y={d.y - 10} text-anchor="middle" class="lg-al">
              {d.label}
            </text>
          )}
        </g>
      ))}
      {vads.map((v, i) => (
        <g key={`v${i}`}>
          <circle cx={gx(pctN(TUNE_FLOOR))} cy={v.y} r="7.5" class="lg-halo-w" />
          <circle cx={gx(pctN(TUNE_FLOOR))} cy={v.y} r="4.5" class="lg-vad" />
        </g>
      ))}
    </g>
  );

  return (
    <svg viewBox={`0 0 300 ${h}`} class="lg" role="img" aria-label={aria || HINTS.living_graph}>
      <defs>
        <filter id={`${gid}b`} x="-20%" y="-20%" width="140%" height="140%">
          <feGaussianBlur stdDeviation="2.1" />
        </filter>
        {cutX != null && (
          <clipPath id={`${gid}L`}>
            <rect x="12" y={topY - 18} width={Math.max(0, cutX - 12)} height={axisY - topY + 36} />
          </clipPath>
        )}
        {cutX != null && (
          <clipPath id={`${gid}R`}>
            <rect x={cutX} y={topY - 18} width={Math.max(0, 288 - cutX)} height={axisY - topY + 36} />
          </clipPath>
        )}
        <clipPath id={`${gid}T`}>
          <rect x="12" y={topY} width="276" height={axisY - topY} rx="7" />
        </clipPath>
      </defs>
      <rect class="lg-track" x="12" y={topY} width="276" height={axisY - topY} rx="7" />
      {/* The graph paper, under everything: faint rules every 5%, a touch stronger on the quarter
          lines, matching horizontals - clipped to the track's rounded corners so the sheet and the
          track are one shape. The veil and the tint wash over it, so it ghosts through both worlds. */}
      <g clip-path={`url(#${gid}T)`}>
        {Array.from({ length: 19 }, (_, k) => (k + 1) * 5).map((c) => (
          <line key={`gv${c}`} x1={gx(c)} y1={topY} x2={gx(c)} y2={axisY} class={`lg-paper${c % 25 === 0 ? " mj" : ""}`} />
        ))}
        {Array.from({ length: Math.max(0, Math.floor((axisY - topY) / 11.5) - 0) }, (_, k) => topY + (k + 1) * 11.5)
          .filter((y) => y < axisY - 2)
          .map((y) => (
            <line key={`gh${y}`} x1="12" y1={y} x2="288" y2={y} class="lg-paper" />
          ))}
      </g>
      {cutX != null ? (
        <>
          {/* right of the knob: a whisper of the accent - the zone where things happen */}
          <rect class="lg-tint" x={cutX} y={topY} width={Math.max(0, 288 - cutX)} height={axisY - topY} rx="7" />
          {/* left of the knob: dots first, blurred - then the amber veil over them */}
          <g clip-path={`url(#${gid}L)`} filter={`url(#${gid}b)`} opacity="0.5">
            {content}
          </g>
          <rect class="lg-veil" x="12" y={topY} width={Math.max(0, cutX - 12)} height={axisY - topY} rx="7" />
          <g clip-path={`url(#${gid}R)`}>{content}</g>
        </>
      ) : (
        content
      )}
      {/* the knob: stem + gripped handle, the radar gate bars' drag lineage */}
      {cutX != null && (
        <g class="lg-knobg">
          <line x1={cutX} y1={topY - 3} x2={cutX} y2={axisY + 3} class={`lg-stem${held ? " held" : ""}`} />
          {held && <rect x={cutX - 11} y={knobY - 4} width="22" height={knobH + 8} rx="7" class="lg-heldring" />}
          <rect x={cutX - 7} y={knobY} width="14" height={knobH} rx="4.5" class={`lg-knob${held ? " held" : ""}`} />
          {[-2.5, 0, 2.5].map((dx) => (
            <line key={dx} x1={cutX + dx} y1={knobY + knobH * 0.28} x2={cutX + dx} y2={knobY + knobH * 0.72} class="lg-grip" />
          ))}
        </g>
      )}
      {/* The axis baseline and its five tick marks. Their LABELS are HTML below the svg (.lg-ticks):
          SVG text misses the subpixel antialiasing every other font in the app gets, and the ticks
          read blurry beside it (owner's report, September 22 2026). Geometry here, type outside. */}
      {axis && (
        <g>
          <line x1="12" y1={axisY + 5} x2="288" y2={axisY + 5} class="lg-axis" />
          {[0, 25, 50, 75, 100].map((c) => (
            <line key={c} x1={gx(c)} y1={axisY + 2} x2={gx(c)} y2={axisY + 8} class="lg-axis" />
          ))}
        </g>
      )}
      {/* tap-a-dot detail: the retired detections list's exact-time story, told in place */}
      {pop && (
        <g class="lg-popg">
          <line x1={gx(pop.c)} y1={pop.y - 5} x2={gx(pop.c)} y2={pop.y - 13} class="lg-popline" />
          <rect x={Math.min(164, Math.max(14, gx(pop.c) - 66))} y={Math.max(2, pop.y - 36)} width="132" height="22" rx="7" class="lg-pop" />
          <text x={Math.min(164, Math.max(14, gx(pop.c) - 66)) + 66} y={Math.max(2, pop.y - 36) + 14.5} text-anchor="middle" class="lg-popt num">
            {pop.text}
          </text>
        </g>
      )}
    </svg>
  );
}

/**
 * The pointer layer over a graph, following the radar controls' arbitration (presence.jsx): the
 * pointer maps through the box straight into confidence percent - never pixels - one handler
 * covers every gesture, and a press that never travels past the slop is a tap, so reading a dot's
 * detail can never nudge the threshold.
 *
 * Also the graph's frame: the legend above and the tick labels below are HTML, not SVG text, so
 * they render with the same subpixel crispness as every other font in the app (owner's report:
 * SVG type read blurry beside the body text). Their positions reuse the SVG's own x math, so the
 * two layers cannot drift. Every axis graph carries the "Confidence" label inside its lower-left.
 *
 * `onCut`/`onCutEnd` make the knob draggable (placement); absent, a knob press is a tap and fires
 * `onKnobTap` (the collapsed row's way back into the tuner). `onMarkTap` opens a dot's popover.
 */
function TouchGraph({ spec, onCut, onCutEnd, onKnobTap, onMarkTap }) {
  const ref = useRef(null);
  const st = useRef(null);

  const toView = (e) => {
    const r = ref.current.getBoundingClientRect();
    return {
      c: ((e.clientX - r.left) / r.width) * 300,
      y: ((e.clientY - r.top) / r.height) * (spec.h || 56),
    };
  };
  const toC = (vx) => Math.max(CUT_MIN_C, Math.min(CUT_MAX_C, (vx - 12) / 2.76));

  const down = (e) => {
    const p = toView(e);
    const cutX = spec.cutC != null ? gx(spec.cutC) : null;
    if (cutX != null && Math.abs(p.c - cutX) <= 16 && (onCut || onKnobTap)) {
      st.current = { mode: "knob", moved: false, x0: p.c };
      try {
        e.currentTarget.setPointerCapture(e.pointerId);
      } catch {
        /* a pointer that vanished mid-press; follow bubbled events instead */
      }
      e.preventDefault();
      return;
    }
    if (onMarkTap) {
      // The nearest dot within a thumb's reach, crisp or frosted - the frost hides, not disables.
      let best = null;
      let bd = 18;
      for (const m of spec.marks || []) {
        const d = Math.hypot(gx(m.c) - p.c, m.y - p.y);
        if (d < bd) {
          bd = d;
          best = m;
        }
      }
      st.current = { mode: best ? "mark" : "none", mark: best };
    }
  };
  const move = (e) => {
    const s = st.current;
    if (!s || s.mode !== "knob" || !onCut) return;
    const p = toView(e);
    // A finger is never perfectly still: nothing moves until the pointer travels past the slop,
    // so a tap cannot nudge the line it meant to reopen.
    if (!s.moved && Math.abs(p.c - s.x0) < 4) return;
    s.moved = true;
    onCut(toC(p.c));
  };
  const up = () => {
    const s = st.current;
    st.current = null;
    if (!s) return;
    if (s.mode === "knob") {
      if (s.moved) onCutEnd?.();
      else onKnobTap?.();
    } else if (s.mode === "mark" && s.mark) {
      onMarkTap(s.mark);
    } else if (s.mode === "none") {
      onMarkTap?.(null);
    }
  };

  const h = spec.h || 56;
  return (
    <div class="lg-wrap">
      {spec.legend && (
        <div class="lg-legend">
          <span class="lg-leg">
            <span class="lg-leg-you" />
            {TEXT.tn_legend_you}
          </span>
          {(spec.marks || []).some((m) => m.kind === "room") && (
            <span class="lg-leg">
              <span class="lg-leg-room" />
              {TEXT.tn_legend_hist}
            </span>
          )}
        </div>
      )}
      <div
        ref={ref}
        class={`lg-box${onCut ? " draggable" : ""}`}
        onPointerDown={down}
        onPointerMove={move}
        onPointerUp={up}
        onPointerCancel={() => (st.current = null)}
      >
        <LivingGraph gid={spec.gid} {...spec} />
        {spec.axis && (
          <span class="lg-conf" style={{ bottom: `${((14 / h) * 100 + 4).toFixed(1)}%` }}>
            {TEXT.tn_axis}
          </span>
        )}
      </div>
      {spec.axis && (
        <div class="lg-ticks num">
          {[0, 25, 50, 75, 100].map((c) => (
            <span key={c} style={{ left: `${((12 + 2.76 * c) / 3).toFixed(2)}%` }}>
              {c}%
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

/** Where the knob may live, in percent. The floor matches the firmware's WL_TUNED_MIN (100/255);
 *  the ceiling leaves the quantized value inside the loader's <= 254 acceptance. */
const CUT_MIN_C = 40;
const CUT_MAX_C = 95;

/** A track's 24h story from its `dh` ring (persisted on the device, so it survives reboots):
 *  solid firings and hollow close calls, each at the confidence the register measured. The
 *  vertical jitter keys on the entry's own stable id - never its list position - so a new dot
 *  landing can never move the old ones (owner's report: index-keyed jitter reshuffled the graph
 *  on every firing). Only the newest firing still ripples. The y band is a parameter because the
 *  same story draws on the collapsed strip and the expanded quick-edit. */
function rowMarks(track, yBase = 14, ySpan = 28) {
  const out = [];
  let newest = null;
  for (const e of track?.dh || []) {
    const [ms, sc, kind, id] = e;
    if (ms >= DAY_MS || !sc) continue;
    const m = { kind: kind ? "near" : "fire", c: pctN(sc), y: yBase + ((id * 37) % ySpan), age: ms / DAY_MS, ms, sc };
    if (!kind && (newest === null || ms < newest.ms)) newest = m;
    out.push(m);
  }
  if (newest && newest.ms < 4500) newest.ripple = true;
  return out;
}

/** The room's last 24 hours as the tuner shows it: one faint amber dot per hourly high-water
 *  bucket (`day`, newest first), age driving the fade. No aggregate tick, no label - the room's
 *  reach is wherever the smatter ends. */
function roomMarks(day, yBase, ySpan) {
  const out = [];
  (day || []).forEach((v, k) => {
    if (!v) return;
    out.push({ kind: "room", c: pctN(v), y: jit(k, yBase, ySpan), age: k / 23 });
  });
  return out;
}

/* ------------------------------------------------------------------ */
/* One word's row                                                      */
/* ------------------------------------------------------------------ */

/** The bell inside the chime toggle: the classic flat-outline notification bell. */
function BellIcon({ slash }) {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
      <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
      <path d="M13.73 21a2 2 0 0 1-3.46 0" />
      {slash && <path d="M4.5 3.5l15 17" />}
    </svg>
  );
}

/**
 * The wake chime, promoted into the Wake Words card's own header (owner call - this retired the
 * Wake Word Settings card, whose only control it was): a toggle with the bell inside, green and
 * ringing when on, red with the bell struck through when off. Same height and grammar as the Stop
 * pill switch - the two are one control family.
 */
function ChimeBell({ ctx }) {
  const chime = entity(ctx, "wake_sound");
  if (!chime) return null;
  const on = chime.value === true || chime.state === "ON";
  return (
    <span class="chimewrap">
      <Hint text={HINTS.wake_sound} />
      <button
        class={`wsw${on ? " on" : ""}`}
        role="switch"
        aria-checked={on}
        aria-label={TEXT.ww_chime}
        onClick={() => post(pathFor(ctx, "wake_sound", on ? "turn_off" : "turn_on"))}
      >
        {!on && <span class="wsw-knob" />}
        <span class="wsw-bell">
          <BellIcon slash={!on} />
        </span>
        {on && <span class="wsw-knob" />}
      </button>
    </span>
  );
}

/**
 * One word's row: the pill (title + inline-picker chevron; Stop: the switch itself), the transient
 * right slot (LIVE dot, download progress, "off"), the untuned row's Tune it! button, then the
 * living graph - or the inline picker in its place while the pill is open - the sub line, and the
 * Voice Pipeline select.
 */
function WordRow({
  first,
  brk,
  hot,
  dimmed,
  word,
  isStop,
  stopOn,
  onStopToggle,
  pillOpen,
  onPill,
  tuneBtn,
  onTune,
  btn,
  onBtn,
  right,
  live,
  graph,
  picker,
  sub,
  subTone,
  pipeline,
}) {
  return (
    <div class={`mb${first ? " first" : ""}${brk ? " brk" : ""}${dimmed ? " off" : ""}${hot ? " hot" : ""}`}>
      <div class="mb-t">
        {isStop ? (
          <>
            <button class={`wsw${stopOn ? " on" : ""}`} role="switch" aria-checked={stopOn} onClick={onStopToggle}>
              {!stopOn && <span class="wsw-knob" />}
              &ldquo;{word}&rdquo;
              {stopOn && <span class="wsw-knob" />}
            </button>
            <Hint text={HINTS.stop_word} />
          </>
        ) : onPill ? (
          <button class="wpill" aria-expanded={pillOpen} onClick={onPill}>
            &ldquo;{word}&rdquo;
            <Chevron down={pillOpen} cls="caret-s" />
          </button>
        ) : (
          <span class="wpill quiet">&ldquo;{word}&rdquo;</span>
        )}
        {live ? (
          <span class="mb-live">
            <span class="mb-live-dot" />
            {TEXT.lg_live}
          </span>
        ) : (
          <span class="mb-right">{right || ""}</span>
        )}
        {tuneBtn && (
          <span class="tunebtn-halo">
            <button class="tunebtn" onClick={onTune}>
              {TEXT.ww_tune_btn}
            </button>
          </span>
        )}
        {btn && (
          <Btn solid onClick={onBtn}>
            {btn}
          </Btn>
        )}
      </div>
      {pillOpen ? picker : graph}
      {sub && <p class={`mb-sub${subTone ? ` t-${subTone}` : ""}`}>{sub}</p>}
      {pipeline}
    </div>
  );
}

/** The Voice Pipeline select, on the row itself (owner call: seeing the pairing and changing it
 *  are the same gesture - no panel to discover). */
function PipelineRow({ assist, word, disabled }) {
  if (!assist.ready) return null;
  return (
    <div class="mb-pipe">
      <span>{TEXT.vp_label}</span>
      <Hint
        text={
          <>
            {HINTS.voice_pipeline}{" "}
            <a href={TEXT.vp_docs_url} target="_blank" rel="noopener">
              {TEXT.vp_docs}
            </a>
          </>
        }
      />
      <span class="grow" />
      <Select
        value={assist.pipelineFor(word) ?? assist.fallbackPipeline() ?? PIPELINE_PREFERRED}
        options={[[PIPELINE_PREFERRED, TEXT.pipeline_preferred], ...assist.pipelines.map((p) => [p, p])]}
        disabled={assist.busy || disabled}
        onChange={(v) => assist.setPipeline(word, v)}
      />
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The tuner: two user-paced phases on the Living Graph                */
/* ------------------------------------------------------------------ */

/**
 * The v2 tuner (canvas: tabs 1-2). No room-listening phase - the engine only ever reads the
 * threshold, so nothing blocks on sampling; the room's last 24 hours is already on the graph as
 * the amber smatter when the tuner opens. Phase 1 is the voice rounds (near / far / other people,
 * round 3 skippable); phase 2 is placement - a draggable knob between two color worlds, a live
 * readout narrating the tradeoff, Apply and done. No confirmation phase: the first real firing
 * confirms itself by landing on the row's graph with its ripple.
 *
 * `quick` is the knob-tap path: placement reopens directly over everything the graph already
 * knows (the persisted voice stats, the room's smatter, the 24h markers) - no re-recording; the
 * "Redo voice rounds" button beside Apply is the full re-measure.
 */
function TunerFlow({ ctx, i, word, isStop, quick, seed, track, wakeRead, onClose }) {
  const [st, setSt] = useState(() => ({
    phase: quick ? "place" : "ready",
    attempts: [],
    vadTries: 0,
    skipped: false,
    roomReg: 0,
    roomSeen: [], // each register reading the session has shown, oldest first - dots land, never move
    day: seed.day || [],
    noise: seed.noise || 0,
    floorV: quick ? seed.floor || 0 : 0,
    hiV: quick ? seed.hi || 0 : 0,
    cutC: quick ? Math.max(CUT_MIN_C, Math.min(CUT_MAX_C, pctN(seed.cut || 130))) : 55,
    held: false,
  }));
  const evRef = useRef([]);

  // The same-area peers this session is holding muted (lib/peermute.js) - they must not answer
  // the word being said over and over. `pm` is the explicit status line (owner decision: name
  // what got muted and what could not be); heldRef is what the keepalive reminds and the release
  // frees. Peers self-heal on a 60s TTL, so every release here is best effort by design.
  const heldRef = useRef([]);
  const [pm, setPm] = useState(null); // { muted: [names], failed: [names], unknown }
  const releasePeers = () => {
    if (heldRef.current.length) releasePeerMutes(heldRef.current);
    heldRef.current = [];
  };

  // Session lifecycle: opened by Start (the ready gate is the whole point), keepalive while open,
  // closed on unmount whatever phase the panel died in. Quick edit never opens one - placement
  // against stored data needs no floored model. The peer holds ride the same lifecycle: asked for
  // at Start, reminded on the same 20s cadence, released wherever the session ends.
  const openRef = useRef(false);
  useEffect(() => {
    const ka = setInterval(() => {
      if (openRef.current) {
        post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => {});
        keepPeerMutes(heldRef.current);
      }
    }, 20000);
    return () => {
      clearInterval(ka);
      if (openRef.current) post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
      releasePeers();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [i]);

  const start = async () => {
    const r = await post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => null);
    if (!r || !r.ok) {
      setSt((s) => ({ ...s, phase: "gone" }));
      return;
    }
    let cap = 1;
    try {
      cap = JSON.parse(r.text).cap ?? 1;
    } catch {
      /* an old firmware answers without cap; assume able and let the board speak */
    }
    openRef.current = true;
    evRef.current = [];
    // The peer holds, in parallel with the rounds - nothing here blocks the measurement. Retune
    // re-enters start() with the holds already placed; asking again would only re-run the
    // sign-ins, so the ask happens once and the keepalive carries it from there.
    if (!heldRef.current.length) {
      holdPeerMutes(ctx?.ha, ctx?.device?.mac).then((res) => {
        heldRef.current = res.held;
        if (res.held.length || res.failed.length || res.unknown)
          setPm({ muted: res.held.map((p) => p.name), failed: res.failed, unknown: res.unknown });
      });
    }
    setSt((s) => ({ ...s, phase: cap === 0 ? "nocap" : "voice", attempts: [], vadTries: 0, skipped: false }));
  };

  // A session that died device-side (or a build that cannot score) has no live holds to justify:
  // the peers go back to their own mute states while the panel shows its message.
  useEffect(() => {
    if (st.phase === "gone" || st.phase === "nocap") releasePeers();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [st.phase]);

  // The poll: the session's event ring drives the voice phase; the payload's day buckets keep the
  // smatter fresh in every phase.
  useEffect(() => {
    let live = true;
    const tick = async () => {
      if (!live) return;
      const d = await wakeRead();
      if (!live) return;
      const day = (i === STOP_SLOT ? d?.stopw?.day : d?.slots?.find((x) => x.i === i)?.day) || null;
      const ev = d?.tune && d.tune.i === i ? d.tune.ev || [] : null;
      const reg = d?.tune && d.tune.i === i ? d.tune.room || 0 : 0;
      if (ev) evRef.current = ev;
      setSt((s) => {
        let next = s;
        if (day) next = { ...next, day };
        // A higher register reading lands as a NEW amber dot; the old ones stay where they fell.
        // The running max still feeds the placement math, but no dot ever moves - the first cut
        // slid one live dot to each new max, and the whole graph appeared to jump mid-session
        // (owner's report, September 22 2026).
        if (reg > next.roomReg && (next.phase === "voice" || next.phase === "place"))
          next = { ...next, roomReg: reg, roomSeen: [...next.roomSeen, reg] };
        if (next.phase !== "voice") return next;
        if (ev === null) return { ...next, phase: "gone" };
        const rounds = ["near", "near", "far", "other"];
        const attempts = ev.filter((e) => !e[2]).map((e, k) => ({ score: evScore(e), round: rounds[k] || "other" }));
        const vadTries = ev.filter((e) => e[2]).length;
        const enough = attempts.length >= 4 || (attempts.length >= 3 && next.skipped);
        if (!enough) return { ...next, attempts, vadTries };
        return place_(next, attempts, vadTries);
      });
      if (live) setTimeout(tick, 700);
    };
    tick();
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [i]);

  /** The room's reach, in quantized units: the loudest of the day's buckets, the session
   *  register, and whatever a previous tune persisted. Advisory only - the human reads it off the
   *  smatter; this feeds the readout's honesty. */
  const roomQ = () => Math.max(0, ...(st.day || []), st.roomReg, st.noise);

  /** Into placement: the knob seeds at a sensible spot inside the gap, so Apply-without-dragging
   *  is a correct answer. Weak scores are diagnosed rather than shrugged at. */
  const place_ = (s, attempts, vadTries) => {
    const floorV = Math.min(...attempts.map((a) => a.score));
    const hiV = Math.max(...attempts.map((a) => a.score));
    const noise = Math.max(0, ...(s.day || []), s.roomReg);
    if (floorV - 8 < 115) {
      return { ...s, attempts, vadTries, phase: "nogap", nogapSide: noise > 140 ? "room" : "voice" };
    }
    const gap = Math.max(0, floorV - Math.max(noise, TUNE_FLOOR));
    const seedQ = Math.min(floorV - 8, Math.max(115, Math.round(Math.max(noise, TUNE_FLOOR) + 0.6 * gap)));
    const cutC = Math.max(CUT_MIN_C, Math.min(CUT_MAX_C, pctN(seedQ)));
    return { ...s, attempts, vadTries, noise, floorV, hiV, phase: "place", cutC };
  };

  const skip = () =>
    setSt((s) => {
      if (s.attempts.length >= 3) return place_(s, s.attempts, s.vadTries);
      return { ...s, skipped: true };
    });

  const apply = async () => {
    const v = Math.max(100, Math.min(250, Math.round(st.cutC * 2.55)));
    const n = roomQ();
    const f = st.floorV || 0;
    const hi = st.hiV || 0;
    await post(`/api/sat1/wakewords/cutoff?i=${i}&v=${v}${n ? `&n=${n}` : ""}${f ? `&f=${f}` : ""}${hi ? `&h=${hi}` : ""}`);
    if (openRef.current) {
      openRef.current = false;
      await post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
    }
    releasePeers();
    await wakeRead();
    onClose();
  };

  /* ---- what each phase draws ---- */

  const smatter = (yBase, ySpan) => {
    const out = roomMarks(st.day, yBase, ySpan);
    // The session's own register readings join the smatter as they land, newest rippling in.
    st.roomSeen.forEach((v, k) =>
      out.push({ kind: "room", c: pctN(v), y: jit(8 + k, yBase, ySpan), age: 0, ripple: k === st.roomSeen.length - 1 })
    );
    return out;
  };

  // Fixed lanes rather than jitter: two attempts landing at the same score overlapped into one
  // unreadable blot (owner's screenshot). The newest attempt ripples in, like every fresh event.
  const attemptsDots = (yBase, laneH) =>
    st.attempts.map((a, k) => ({
      c: pctN(a.score),
      y: yBase + k * laneH,
      label: TEXT[`tn_r_${a.round}`],
      ripple: k === st.attempts.length - 1,
    }));
  const vadList = Array.from({ length: Math.min(st.vadTries, 4) }, (_, k) => ({ y: jit(k + 5, 60, 50) }));

  const prompt =
    st.attempts.length < 2
      ? TEXT.tn2_near.replace("%s", word)
      : st.attempts.length < 3
        ? TEXT.tn2_far
        : TEXT.tn2_other;

  // The peer-muting status, explicit by owner decision (September 23 2026): who is muted for this
  // session, who could not be, or that nobody could even be looked for. Silent only when the
  // roster answered and named no same-area peer - there is nothing to say about an empty room.
  const pmNote = pm && (
    <>
      {pm.muted.length > 0 && (
        <p class="dim sm">
          {(pm.muted.length === 1 ? TEXT.tn_pm_one : TEXT.tn_pm_many.replace("%s", String(pm.muted.length))) +
            ` (${pm.muted.join(", ")})`}
        </p>
      )}
      {pm.failed.length > 0 && <p class="t-warn sm">{TEXT.tn_pm_failed.replace("%s", pm.failed.join(", "))}</p>}
      {pm.unknown && <p class="t-warn sm">{TEXT.tn_pm_unknown}</p>}
    </>
  );

  // The live readout under the placement graph: safe names the margin, the edges push back. The
  // persisted voice stats feed it as data even in quick mode - they are never drawn as a band.
  const readout = () => {
    const floorC = st.floorV ? pctN(st.floorV) : 0;
    const roomC = (() => {
      let m = 0;
      for (const v of st.day || []) m = Math.max(m, pctN(v));
      return Math.max(m, st.roomReg ? pctN(st.roomReg) : 0);
    })();
    const c = Math.round(st.cutC);
    if (floorC && c >= floorC - 2) return { tone: "warn", text: TEXT.tn2_high.replace("%s", `${floorC}%`) };
    if (roomC && c <= roomC) return { tone: "err", text: TEXT.tn2_low };
    let text = TEXT.tn2_ok.replace("%s", `${c}%`);
    if (floorC) text += TEXT.tn2_under.replace("%s", String(floorC - c));
    return { tone: "dim", text: `${text}.` };
  };

  const graphSpec = (extra) => ({
    gid: `tn${i}`,
    h: 150,
    axis: true,
    legend: true,
    ...extra,
  });

  return (
    <div class="tn-flow">
      {st.phase === "ready" && (
        <>
          {/* The same height as every training phase: the canvas resizing between ready and the
              rounds read as a layout bug (owner's screenshots, September 22 2026). Two heights
              exist in the whole feature - the collapsed row strip and this one. */}
          <TouchGraph spec={graphSpec({ marks: smatter(38, 80) })} />
          <p class="dim sm">{(isStop ? TEXT.tn2_ready_stop : TEXT.tn2_ready).replace("%s", word)}</p>
          <div class="row-actions">
            <Btn solid onClick={start}>
              {TEXT.tn2_start}
            </Btn>
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>
      )}

      {st.phase === "voice" && (
        <>
          <TouchGraph spec={graphSpec({ marks: smatter(38, 80), dots: attemptsDots(42, 26), vads: vadList })} />
          <p class="dim sm">
            {prompt} ({Math.min(st.attempts.length, 4)} / 4)
          </p>
          {st.vadTries > 0 && <p class="t-warn sm">{TEXT.tn_vad}</p>}
          {pmNote}
          <div class="row-actions">
            {st.attempts.length >= 3 && <Btn onClick={skip}>{TEXT.tn2_skip}</Btn>}
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>
      )}

      {st.phase === "place" && (
        <>
          <TouchGraph
            spec={graphSpec({
              cutC: st.cutC,
              held: st.held,
              marks: [...smatter(38, 80), ...(quick ? rowMarks(track, 42, 84) : [])],
              dots: quick ? [] : attemptsDots(42, 26),
            })}
            onCut={(c) => setSt((s) => ({ ...s, cutC: c, held: true }))}
            onCutEnd={() => setSt((s) => ({ ...s, held: false }))}
          />
          <p class={`sm t-${readout().tone}`}>{readout().text}</p>
          {pmNote}
          {/* Four verbs, one row, this order (owner call): back into the rounds, wipe the 24h
              record, out, commit. Clear history wears the danger red - it erases a measurement -
              and Apply the accent, being the row's whole point. */}
          <div class="row-actions one4">
            <Btn onClick={start}>{TEXT.tn2_retune}</Btn>
            <Btn
              danger
              onClick={async () => {
                await post(`/api/sat1/wakewords/clearhist?i=${i}`);
                await wakeRead();
              }}
            >
              {TEXT.tn2_clear}
            </Btn>
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
            <Btn solid onClick={apply}>
              {TEXT.tn_apply}
            </Btn>
          </div>
        </>
      )}

      {st.phase === "nogap" && (
        <>
          <TouchGraph spec={graphSpec({ marks: smatter(38, 80), dots: attemptsDots(42, 26) })} />
          <p class="t-err sm">{st.nogapSide === "room" ? TEXT.tn_nogap_room : TEXT.tn_nogap_voice}</p>
          <div class="row-actions">
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>
      )}

      {st.phase === "nocap" && <p class="dim sm">{TEXT.tn_nocap}</p>}
      {st.phase === "gone" && <p class="t-warn sm">{TEXT.tn_gone}</p>}

      {["nocap", "gone"].includes(st.phase) && (
        <div class="row-actions">
          <Btn onClick={onClose}>{TEXT.cancel}</Btn>
        </div>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The Wake Word Sources card (unchanged logic)                        */
/* ------------------------------------------------------------------ */

function SourcesCard({ sources, setSources, cat }) {
  const [draft, setDraft] = useState("");
  const [bad, setBad] = useState(false);

  const add = () => {
    const url = draft.trim().replace(/\/+$/, "");
    const gh = /^https?:\/\/(?:www\.)?github\.com\/[^/]+\/[^/#?]+/.test(url);
    const json = /\.json($|\?)/.test(url);
    if (!gh && !json) {
      setBad(true);
      return;
    }
    setBad(false);
    if (sources.some((s) => s.url === url)) return;
    const label = gh ? url.split("/").slice(3, 5).join("/") : url.split("/").pop();
    setSources([...sources, { url, label }]);
    setDraft("");
  };

  const missingDefaults = DEFAULT_SOURCES.filter((d) => !sources.some((s) => s.url === d.url));

  return (
    <Card title={TEXT.ws_title} icon={N_WSRC} hint={HINTS.wake_sources} data-card="sources">
      {sources.map((s) => {
        const c = cat[s.url];
        return (
          <div key={s.url} class="ws-row">
            <div class="ws-name">
              <a href={s.url} target="_blank" rel="noopener">
                {s.label}
              </a>
              <span class="dim sm">
                {c?.entries ? `${c.entries.length} ${TEXT.ws_words}` : c?.error ? TEXT.ww_source_failed : TEXT.ww_source_loading}
              </span>
            </div>
            <Confirm
              label={"\u2715"}
              title={TEXT.ws_remove_t}
              body={TEXT.ws_remove_b}
              confirmLabel={TEXT.ws_remove_c}
              onConfirm={() => setSources(sources.filter((x) => x.url !== s.url))}
            />
          </div>
        );
      })}

      {missingDefaults.length > 0 && (
        <p class="dim sm">
          <button class="linkish" onClick={() => setSources([...missingDefaults, ...sources])}>
            {TEXT.ws_restore}
          </button>
        </p>
      )}

      <div class="ws-add">
        <input
          class="ww-search"
          type="url"
          placeholder={TEXT.ws_ph}
          value={draft}
          onInput={(e) => {
            setDraft(e.currentTarget.value);
            setBad(false);
          }}
          onKeyDown={(e) => e.key === "Enter" && add()}
        />
        <Btn onClick={add} disabled={!draft.trim()}>
          {TEXT.ws_add}
        </Btn>
      </div>
      {bad && <p class="t-err sm">{TEXT.ws_bad_url}</p>}

      <p class="dim sm ws-foot">
        {TEXT.ws_footer_q}
        <a href={REQUEST_WORD_URL} target="_blank" rel="noopener">
          {TEXT.ws_request}
        </a>
        {TEXT.ws_or}
        <a href={TRAIN_URL} target="_blank" rel="noopener">
          {TEXT.ws_train}
        </a>
        .
      </p>
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Route                                                               */
/* ------------------------------------------------------------------ */

export function WakeWords({ ctx }) {
  const { ha, haRefresh } = ctx;
  // The standing poll (2.5s): the graphs' dots, the live landings and the row-flash moments are
  // living facts. Swaps and tune sessions still run their faster loops on top.
  const { wake, wakeRead, setSlot } = useWakeSlots(2500);

  const [sources, setSourcesState] = useState(readSources);
  const [cat, setCat] = useState({});
  // Slot indexes with the inline picker expanded - a Set, because both words' pickers may stand
  // open at once (owner call: opening the second must not close the first).
  const [picking, setPicking] = useState(() => new Set());
  const [tuning, setTuning] = useState(null); // { i, word, isStop, quick } while the tuner card is up
  const [swaps, setSwaps] = useState({}); // i -> {phase, word, spec, err, dl, tot}
  const [pop, setPop] = useState(null); // { key, c, y, text } - the tapped dot's story
  const alive = useRef(true);
  useEffect(() => () => (alive.current = false), []);

  const togglePick = (i) =>
    setPicking((prev) => {
      const next = new Set(prev);
      next.has(i) ? next.delete(i) : next.add(i);
      return next;
    });
  const closePick = (i) =>
    setPicking((prev) => {
      const next = new Set(prev);
      next.delete(i);
      return next;
    });

  const setSources = (list) => {
    setSourcesState(list);
    writeSources(list);
  };

  useEffect(() => {
    let live = true;
    for (const s of sources) {
      if (cat[s.url]) continue;
      setCat((c) => ({ ...c, [s.url]: { loading: true } }));
      enumerateSource(s)
        .then((entries) => live && setCat((c) => ({ ...c, [s.url]: { entries } })))
        .catch(() => live && setCat((c) => ({ ...c, [s.url]: { error: true } })));
    }
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sources]);

  const slots = wake?.slots || [];
  const slotAt = (i) => slots.find((s) => s.i === i);
  const anyBusy = Object.values(swaps).some((s) => s?.phase === "busy");
  const activeWords = slots.filter((s) => s.m && s.w).map((s) => s.w);
  const assist = useAssist(ha, haRefresh, activeWords);

  useEffect(() => {
    haSyncOnce(haRefresh);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // The one-shot mount repair for historical staleness: a word that is listening on the device but
  // holds no Home Assistant slot - a browser closed mid-pairing, or a swap that predates the
  // device-side reload - gets slotted the moment anyone opens this page. One word per mount (a
  // second unslotted word would race the first for the same free slot on a stale view; the next
  // visit catches it), and syncSlot's own guards skip slotted words and stop when no slot is free.
  const repaired = useRef(false);
  useEffect(() => {
    if (repaired.current || !assist.ready) return;
    const orphan = activeWords.find((w) => assist.pipelineFor(w) === null);
    if (!orphan) return;
    repaired.current = true;
    assist.syncSlot(orphan, true).catch(() => {});
    // Also keyed on the device's word list: HA readiness and the slot read land in either order.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [assist.ready, activeWords.length]);

  // The row-flash moment: a fresh firing (det seq bump) lights its word's row while the dot lands
  // on its graph with the ripple - the same beat, no text.
  const [hot, setHot] = useState(null);
  const seqRef = useRef(null);
  useEffect(() => {
    const det = wake?.det;
    if (!det) return;
    if (seqRef.current === null) {
      seqRef.current = det[0]; // the history that predates this page load is not news
      return;
    }
    if (det[0] > seqRef.current) {
      seqRef.current = det[0];
      setHot({ seq: det[0], word: det[1] });
      const t = setTimeout(() => setHot(null), 3500);
      return () => clearTimeout(t);
    }
  }, [wake]);

  // The popover dismisses itself: dots keep arriving underneath it, and a story read is a story done.
  useEffect(() => {
    if (!pop) return undefined;
    const t = setTimeout(() => setPop(null), 4000);
    return () => clearTimeout(t);
  }, [pop]);

  /** Writes the swap into Home Assistant's wake word selects - unchanged in shape from the
   *  previous route (see git history for the war stories), hardened on two fronts after the
   *  stale-select bug: an empty payload is a transient miss to wait out rather than a verdict to
   *  stop on, and the deadline covers a config-entry reload rather than a bare reconnect. */
  const syncAssist = async (i, prevWord, nextWord) => {
    const named = (sel, w) => sel.findIndex((x) => w && (x[1] || "").toLowerCase() === w.toLowerCase());
    // 60s, not the old 30: after a download the device now asks Home Assistant to reload this
    // device's config entry, which tears down and rebuilds every entity - longer than the old
    // reconnect, and the pairing write only lands once the selects are back with the new option.
    const deadline = Date.now() + 60000;
    for (;;) {
      const fresh = await requestJson("/api/sat1/ha").catch(() => null);
      const sel = fresh?.d?.asst?.s;
      // A missing or empty asst block is EXPECTED right now - Home Assistant is reconnecting or
      // mid-reload, which is exactly when this loop runs - so it is a miss to retry, not a reason
      // to stop. Returning here permanently was how a download could strand the HA select on
      // "No Wake Word" forever (one bad poll and nothing ever retried).
      if (Array.isArray(sel) && sel.length) {
        let ent, option;
        if (!nextWord) {
          const off = named(sel, prevWord);
          if (off < 0) return;
          ent = sel[off][0];
          option = NO_WAKE_WORD;
        } else {
          if (named(sel, nextWord) >= 0) return;
          let at = named(sel, prevWord);
          if (at < 0) at = sel.findIndex((x) => x[1] === NO_WAKE_WORD);
          if (at < 0) at = Math.min(i, sel.length - 1);
          ent = sel[at][0];
          option = nextWord;
        }
        await post(`/api/sat1/ha/select?e=${encodeURIComponent(ent)}&o=${encodeURIComponent(option)}`).catch(() => {});
        await ctx.haRefresh();
      }
      if (!alive.current || Date.now() > deadline) return;
      // A breather between rounds: Home Assistant needs seconds to reconnect and re-read after
      // the reload, and re-posting flat out just burned the device's socket table while it did.
      await sleep(500);
    }
  };

  const choose = async (i, { spec, word }) => {
    const prevWord = slotAt(i)?.w || "";
    if (tuning && tuning.i === i) setTuning(null);
    setSwaps((s) => ({ ...s, [i]: { phase: "busy", word, spec } }));
    const r = await setSlot(i, spec).catch(() => ({ ok: false }));
    if (!r.ok) {
      if (alive.current) setSwaps((s) => ({ ...s, [i]: { phase: "error", word, spec, err: 0 } }));
      return;
    }
    const deadline = Date.now() + (isUrl(spec) ? 90000 : 10000);
    for (;;) {
      await sleep(isUrl(spec) ? 900 : 400);
      if (!alive.current) return;
      const d = await wakeRead();
      const s = d?.slots?.find((x) => x.i === i);
      if (s) {
        const arrived = spec === "none" ? s.m === "" : s.m === spec;
        if (arrived && s.st === 0) {
          setSwaps((prev) => ({ ...prev, [i]: null }));
          await syncAssist(i, prevWord, spec === "none" ? "" : s.w || word);
          return;
        }
        if (s.st === 2 && (arrived || s.err)) {
          setSwaps((prev) => ({ ...prev, [i]: { phase: "error", word, spec, err: s.err } }));
          return;
        }
        if (s.st === 1) setSwaps((prev) => ({ ...prev, [i]: { ...prev[i], dl: s.dl || 0, tot: s.tot || 0 } }));
      }
      if (Date.now() > deadline) {
        setSwaps((prev) => ({ ...prev, [i]: { phase: "error", word, spec, err: 7 } }));
        return;
      }
    }
  };

  if (!ctx.device) return <Missing what="the device" />;
  if (wake === null) return <Missing what="Wake word control" />;

  const groups = [
    {
      key: "inc",
      label: TEXT.ww_included,
      entries: (wake?.builtin || []).map(([id, w]) => ({ word: w, spec: id, langs: [] })),
    },
  ];
  for (const s of sources) {
    const c = cat[s.url] || {};
    groups.push({ key: s.url, label: s.label, entries: c.entries || [], loading: c.loading, error: c.error });
  }

  const stopSwitch = entity(ctx, "stop_word");
  const stopOn = stopSwitch ? stopSwitch.value === true || stopSwitch.state === "ON" : false;
  const stopw = wake?.stopw || null;

  const assistNote = !assist.ready && activeWords.length > 0 && (
    <p class="dim sm">
      {haBlocked(ctx.ha) ? TEXT.assistant_blocked : haTooOld(ctx.ha) ? TEXT.ha_too_old : TEXT.assistant_needs_ha}
      {haBlocked(ctx.ha) && ctx.onShowFix && (
        <>
          {" "}
          <button class="linkish" onClick={ctx.onShowFix}>
            {TEXT.show_fix}
          </button>
        </>
      )}
    </p>
  );

  const inlinePicker = (i) => (
    <InlinePicker
      groups={groups}
      current={swaps[i]?.phase === "busy" ? swaps[i].spec : slotAt(i)?.m || ""}
      otherWord={slotAt(1 - i)?.w || swaps[1 - i]?.word || ""}
      busy={anyBusy}
      onPick={(entry) => {
        closePick(i);
        choose(i, entry ? { spec: entry.spec || entry.url, word: entry.word } : { spec: "none", word: "" });
      }}
    />
  );

  /** A tuned track's collapsed graph: the standing 24h record, knob-tappable back into placement,
   *  dots tappable for their story. `key` scopes the popover to one row. */
  const rowGraph = (key, track, onKnob) => {
    const marks = rowMarks(track);
    return (
      <TouchGraph
        spec={{
          gid: `g${key}`,
          h: 56,
          cutC: pctN(track.cut),
          marks,
          pop: pop && pop.key === key ? pop : null,
        }}
        onKnobTap={onKnob}
        onMarkTap={(m) =>
          setPop(m ? { key, c: m.c, y: m.y, text: `${pct(m.sc)} \u00b7 ${ago(m.ms)}${m.kind === "near" ? ` \u00b7 ${TEXT.lg_ignored_short}` : ""}` } : null)
        }
      />
    );
  };

  /** One word slot's row, in every state the mockups enumerate. */
  const wordRow = (i, first) => {
    const slot = slotAt(i);
    const swap = swaps[i];
    const swapping = swap && swap.phase === "busy";
    const failed = swap && swap.phase === "error";
    const word = swapping ? swap.word : slot?.w || "";
    const waiting = slot?.st === 3 || (slot?.st === 2 && !slot?.ld && isUrl(slot?.m));
    if (!word && !swapping) return null;

    const tuned = !swapping && slot?.cut > 0;
    const isHot = hot && hot.word === word;

    // Mid-swap the card shows two truths at once (Journey 4): the old word's row dimmed but drawn
    // - it is still listening - and the new word's row downloading beneath it.
    if (swapping && slot?.w && slot.w !== swap.word) {
      return (
        <>
          <WordRow
            key={`old${i}`}
            first={first}
            word={showWord(slot.w)}
            right={TEXT.mb_still}
            dimmed
            graph={slot.cut > 0 ? rowGraph(`old${i}`, slot, null) : null}
          />
          <WordRow
            key={`new${i}`}
            word={showWord(swap.word)}
            right={swap.tot > 0 ? `${Math.round((swap.dl || 0) / 1024)} / ${Math.round(swap.tot / 1024)} KB` : isUrl(swap.spec) ? TEXT.ww_downloading : TEXT.ww_loading}
            sub={TEXT.mb_swap_note.replace("%1", showWord(slot.w)).replace("%2", showWord(swap.word))}
          />
        </>
      );
    }

    return (
      <WordRow
        key={i}
        first={first}
        word={showWord(word)}
        hot={isHot}
        live={tuned && !!slot?.ld}
        right={swapping ? (swap.tot > 0 ? `${Math.round((swap.dl || 0) / 1024)} / ${Math.round(swap.tot / 1024)} KB` : TEXT.ww_downloading) : ""}
        pillOpen={picking.has(i)}
        onPill={swapping ? null : () => togglePick(i)}
        tuneBtn={!tuned && !swapping && !failed && !!slot?.ld}
        onTune={() => setTuning({ i, word, isStop: false, quick: false })}
        btn={failed ? TEXT.ww_retry : null}
        onBtn={failed ? () => choose(i, { spec: swap.spec, word: swap.word }) : null}
        graph={tuned ? rowGraph(String(i), slot, () => setTuning({ i, word, isStop: false, quick: true })) : null}
        picker={inlinePicker(i)}
        sub={
          failed
            ? `${TEXT.ww_failed} ${WW_ERR[swap.err] || ""}`
            : waiting
              ? slot.st === 3
                ? TEXT.ww_waiting
                : `${WW_ERR[slot.err] || ""} ${TEXT.ww_retrying}`
              : null
        }
        subTone={failed ? "err" : waiting ? "warn" : undefined}
        pipeline={!swapping && word ? <PipelineRow assist={assist} word={word} disabled={swapping} /> : null}
      />
    );
  };

  /** The stop word's permanent third row, always last and behind its own clear break (owner call:
   *  the card reads word 1, word 2, break, Stop - nothing may render below Stop). Its pill IS the
   *  switch - green listening, red off, the graph gone entirely while off. */
  const stopRow = () => {
    if (!stopw || !stopSwitch) return null;
    const tuned = stopOn && stopw.cut > 0;
    const isHot = hot && hot.word === "stop";
    return (
      <WordRow
        brk
        word={showWord("stop")}
        isStop
        stopOn={stopOn}
        onStopToggle={() => post(pathFor(ctx, "stop_word", stopOn ? "turn_off" : "turn_on"))}
        hot={isHot}
        live={tuned}
        right={!stopOn ? TEXT.mb_off : ""}
        tuneBtn={stopOn && !tuned}
        onTune={() => setTuning({ i: STOP_SLOT, word: "stop", isStop: true, quick: false })}
        graph={tuned ? rowGraph("stop", stopw, () => setTuning({ i: STOP_SLOT, word: "stop", isStop: true, quick: true })) : null}
        sub={!stopOn ? TEXT.stop_off_note : null}
      />
    );
  };

  const freeSlot = [0, 1].find((i) => !slotAt(i)?.w && !swaps[i]);
  const noWords = activeWords.length === 0 && !anyBusy;

  /** The tuner's seed: everything the graph already knows about this track, so quick edit reopens
   *  placement with no re-recording. */
  const tuneSeed = (i) => {
    const t = i === STOP_SLOT ? stopw : slotAt(i);
    return {
      cut: t?.cut || 0,
      noise: (t?.tn || [])[0] || 0,
      floor: (t?.tn || [])[1] || 0,
      hi: (t?.tn || [])[2] || 0,
      day: t?.day || [],
    };
  };

  return (
    <>
      {tuning ? (
        <Card title={TEXT.tn_title.replace("%s", showWord(tuning.word))} icon={N_WAKE} hint={HINTS.living_graph}>
          <TunerFlow
            ctx={ctx}
            i={tuning.i}
            word={tuning.word}
            isStop={tuning.isStop}
            quick={tuning.quick}
            seed={tuneSeed(tuning.i)}
            track={tuning.i === STOP_SLOT ? stopw : slotAt(tuning.i)}
            wakeRead={wakeRead}
            onClose={() => setTuning(null)}
          />
        </Card>
      ) : (
        <Card title={TEXT.ww_card} icon={N_WAKE} hint={HINTS.wake_words} right={<ChimeBell ctx={ctx} />}>
          {noWords && (
            <>
              <p class="dim sm">{TEXT.ww_route_none}</p>
              {!picking.has(0) && (
                <div class="row-actions center">
                  <Btn solid onClick={() => togglePick(0)}>
                    {TEXT.ww_choose}
                  </Btn>
                </div>
              )}
            </>
          )}
          {/* The card's fixed order (owner call): word 1, word 2, the break, Stop - nothing ever
              renders below Stop. An empty slot's expanded add-picker holds that slot's own place
              in the order, and the + Add word button lives in the words section too. */}
          {[0, 1].map((i, k) => {
            const row = wordRow(i, k === 0 && !noWords);
            if (row) return row;
            if (picking.has(i) && !slotAt(i)?.w && !swaps[i]) {
              return (
                <div class={`mb${k === 0 && !noWords ? " first" : ""}`} key={`add${i}`}>
                  {inlinePicker(i)}
                </div>
              );
            }
            return null;
          })}
          {!noWords && freeSlot !== undefined && !picking.has(freeSlot) && (
            <div class="row-actions">
              <Btn onClick={() => togglePick(freeSlot)} disabled={anyBusy}>
                {TEXT.ww_add}
              </Btn>
            </div>
          )}
          {stopRow()}
          {assistNote}
        </Card>
      )}

      <SourcesCard sources={sources} setSources={setSources} cat={cat} />
    </>
  );
}
