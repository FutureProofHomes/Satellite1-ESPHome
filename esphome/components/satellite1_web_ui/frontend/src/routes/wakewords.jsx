/**
 * The Wake Words route, four cards in the owner's order (September 2026): Wake Word 1, Wake Word 2,
 * Wake Word Settings, Wake Word Sources. Two word cards because two is what the device runs at once
 * (plus the internal "stop") - each carries its picker, its Assistant pairing, and its own
 * sensitivity; the settings card holds what is not per-word (the chime and the stop toggle); the
 * sources card says where the massive list comes from.
 *
 * Each picker expands in place into the full list, grouped by source with "Included" first,
 * searchable and filterable by language; picking a word beyond the included two downloads it to the
 * device on the fly. The device is the validator and the source of truth - the browser only
 * enumerates sources (lib/wakesources.js) and polls the swap it asked for. A failed download leaves
 * the previous word listening, and the card says so.
 *
 * Which assistant answers each word stays Home Assistant's, exactly as before: the dropdown under
 * each picker writes select.select_option through the device, and the slot-sync keeps Home
 * Assistant's two wake-word pairings equal to the two slots (see useAssist in lib/device.js).
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { HINTS, TEXT, WW_ERR } from "../copy.js";
import {
  NO_WAKE_WORD,
  PIPELINE_PREFERRED,
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
import { Btn, Card, Chevron, Confirm, Missing, N_WAKE, Row, Select, Toggle, ni } from "../ui.jsx";

/* This route's own card glyphs (owner request, September 2026: every card here wears one), drawn in
   ui.jsx's 16-box stroke grammar. The two word cards share the route's waveform - they are twins
   and the numbers in their titles already tell them apart. */
/** Settings: an equalizer of three slider tracks with offset knobs. */
const N_WSET = ni(
  <>
    <path d="M3 5h10" />
    <circle cx="6" cy="5" r="1.6" fill="currentColor" stroke="none" />
    <path d="M3 8h10" />
    <circle cx="10.4" cy="8" r="1.6" fill="currentColor" stroke="none" />
    <path d="M3 11h10" />
    <circle cx="7.4" cy="11" r="1.6" fill="currentColor" stroke="none" />
  </>
);
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
const kb = (n) => (n > 0 ? `${Math.round(n / 1024)} KB` : "");

/* ------------------------------------------------------------------ */
/* The picker: one slot's expanding list                               */
/* ------------------------------------------------------------------ */

/** How many rows a search shows before asking for more letters. Rendering all ~800 would be the
 *  cost this cap exists to avoid, and past a screenful the answer is a narrower search anyway. */
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

/** One pickable word, on the /audio tree's row grammar: the drawn checkbox, the name in the grow
 *  slot, the fine print right-aligned where tree-why sits. Single-select semantics ride the same
 *  visuals - a slot holds one word, so ticking one is what unticks the rest. */
function PickRow({ entry, selected, disabled, onPick }) {
  const meta = [entry.langs?.length ? entry.langs.join(",") : null, kb(entry.size), entry.unverified ? TEXT.ww_unverified : null]
    .filter(Boolean)
    .join(" \u00b7 ");
  return (
    <div class={`tree-p${disabled ? " tree-off" : ""}`}>
      <Check state={selected ? "on" : "off"} disabled={disabled} onClick={() => onPick(entry)} label={entry.word} />
      <button class="ww-name grow" disabled={disabled} onClick={() => onPick(entry)}>
        &ldquo;{entry.word}&rdquo;
      </button>
      {meta && <span class="tree-why">{meta}</span>}
      {/* The training-generation tag, shown only when this phrase has twins in the list - it
          exists to tell three "Computer"s apart, not to decorate every row. The count pill's
          clothes, because it is the same kind of small fact. */}
      {entry.dup && entry.ver && <span class="tree-n">{entry.ver}</span>}
      <SpeakBtn word={entry.word} />
    </div>
  );
}

/**
 * The expanded list, wearing the same clothes as the /audio route's area/player tree (owner's rule,
 * September 2026: one selection-list pattern app-wide): the sunken scrolling .tree box, .tree-a
 * groups whose headers carry the caret, the label and the count pill, rows indented under a .tree-ps
 * guide line. What the tree does not have - the search box and the language filter - rides a sticky
 * bar at the top of the same box.
 *
 * `groups` is [{key, label, entries, error, loading, exp}], Included first. Included opens by
 * default; the big community groups start closed, because 778 rows is a scroll nobody asked for
 * until they ask for it. A search flattens across every group.
 */
function Picker({ groups, current, otherWord, busy, onPick }) {
  const [q, setQ] = useState("");
  const [lang, setLang] = useState("");
  const [open, setOpen] = useState(() => new Set([groups[0]?.key]));

  const langs = new Set();
  for (const g of groups) for (const e of g.entries || []) for (const l of e.langs || []) langs.add(l);

  const match = (e) =>
    (!q || e.word.toLowerCase().includes(q.toLowerCase())) && (!lang || (e.langs || []).includes(lang));
  const disabled = (e) => busy || (!!otherWord && e.word.toLowerCase() === otherWord.toLowerCase());

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
        // While searching, groups flatten: every match shows under its group header, and empty
        // groups vanish rather than standing as headers over nothing.
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
                  // Tapping the checked word unchecks it - the house checkbox gesture - which is
                  // how a slot empties now that the explicit Disabled row is gone (owner call,
                  // September 2026: a "Disabled" entry read as one of the words).
                  return (
                    <PickRow
                      key={e.url || e.spec}
                      entry={e}
                      selected={selected}
                      disabled={disabled(e)}
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

/* ------------------------------------------------------------------ */
/* The Wake Word Tuner                                                 */
/* ------------------------------------------------------------------ */

/** Quantized probability (0-255) as the percentage people read. */
const pct = (v) => `${Math.round((v / 255) * 100)}%`;

/** The probe floor the firmware drops to during a session (WL_TUNE_FLOOR). Room noise that never
 *  scored means "below this", which renders as a floor rather than a false zero. */
const TUNE_FLOOR = 107;

/** How long the room-listen phase runs. Long enough for a TV or a fan to show itself, short
 *  enough that nobody wonders whether it is stuck. */
const LISTEN_S = 15;

/**
 * The guided tuner: listen to the room, say the word three times, apply a measured threshold.
 *
 * The device does the measuring - a session floors this word's cutoff so nearly every attempt and
 * every room false-trigger produces a scored event, which the wake words payload carries in its
 * `tune` block while the session lives. This panel is choreography over those events: it decides
 * which belong to the quiet phase and which are attempts purely by when they arrive, computes the
 * recommendation, and holds the session open with a keepalive so an abandoned tab can never leave
 * the device on the probe floor (the firmware expires the session and restores the cutoff anyway).
 *
 * The recommendation splits the gap between the room's loudest false score and the quietest
 * attempt, biased toward the noise side (60% up the gap): false accepts annoy every hour of the
 * day, while a slightly-too-strict threshold costs one repeat. No usable gap is an honest failure,
 * not a shrug - the word cannot be told apart from this room.
 */
function TunerPanel({ i, word, cut, wakeRead, onClose }) {
  const [st, setSt] = useState({ phase: "listen", left: LISTEN_S, mark: 0, mark2: 0, noise: null, attempts: [], vad: 0, rec: 0 });
  const evRef = useRef([]);
  const endAt = useRef(Date.now() + LISTEN_S * 1000);

  // Session lifecycle: open on mount, keepalive while open, close on unmount - whatever phase the
  // panel died in. The capability answer decides whether there is any point proceeding.
  useEffect(() => {
    let live = true;
    (async () => {
      const r = await post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => null);
      if (!live) return;
      let cap = 1;
      try {
        cap = JSON.parse(r.text).cap ?? 1;
      } catch {
        /* an old firmware answers without cap; assume able and let the meter speak */
      }
      if (!r || !r.ok) setSt((s) => ({ ...s, phase: "gone" }));
      else if (cap === 0) setSt((s) => ({ ...s, phase: "nocap" }));
    })();
    const ka = setInterval(() => post(`/api/sat1/wakewords/tune?i=${i}&on=1`).catch(() => {}), 20000);
    return () => {
      live = false;
      clearInterval(ka);
      post(`/api/sat1/wakewords/tune?i=${i}&on=0`).catch(() => {});
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [i]);

  // The cosmetic countdown; the authoritative listen-to-speak transition happens in the poll below
  // against the same clock, so a slow poll cannot lose events into the wrong phase.
  useEffect(() => {
    const t = setInterval(() => setSt((s) => (s.phase === "listen" && s.left > 0 ? { ...s, left: s.left - 1 } : s)), 1000);
    return () => clearInterval(t);
  }, []);

  // The poll: the session's event ring drives every phase. Events are session-scoped (the firmware
  // clears the ring at open), so positions in the list are the phase boundary markers.
  useEffect(() => {
    let live = true;
    const tick = async () => {
      if (!live) return;
      const d = await wakeRead();
      if (!live) return;
      const ev = d?.tune && d.tune.i === i ? d.tune.ev || [] : null;
      if (ev) evRef.current = ev;
      setSt((s) => {
        if (s.phase === "done" || s.phase === "nogap" || s.phase === "nocap" || s.phase === "gone") return s;
        if (ev === null) return { ...s, phase: "gone" };
        const peaks = (list) => list.filter((e) => !e[2]).map((e) => e[0]);
        if (s.phase === "listen") {
          if (Date.now() < endAt.current) return s;
          const noisy = peaks(ev);
          return { ...s, phase: "speak", mark: ev.length, noise: noisy.length ? Math.max(...noisy) : null };
        }
        if (s.phase === "speak") {
          const slice = ev.slice(s.mark);
          const attempts = peaks(slice);
          const vad = slice.filter((e) => e[2]).length;
          if (attempts.length < 3) return { ...s, attempts, vad };
          const noise = s.noise ?? TUNE_FLOOR;
          const minAtt = Math.min(...attempts);
          const gap = minAtt - noise;
          if (gap < 26 || minAtt - 8 < 115) return { ...s, attempts, vad, phase: "nogap" };
          const rec = Math.min(minAtt - 8, Math.max(115, Math.round(noise + 0.6 * gap)));
          return { ...s, attempts, vad, rec, phase: "rec" };
        }
        if (s.phase === "confirm") {
          const heard = ev.slice(s.mark2).some((e) => !e[2] && e[0] >= s.rec);
          return heard ? { ...s, phase: "done" } : s;
        }
        return s;
      });
      if (live) setTimeout(tick, 700);
    };
    tick();
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [i]);

  // The confirmed ending closes itself: the threshold is already applied and heard, and a panel
  // that lingers invites re-reading a finished story.
  useEffect(() => {
    if (st.phase !== "done") return undefined;
    const t = setTimeout(onClose, 2500);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [st.phase]);

  const apply = async () => {
    await post(`/api/sat1/wakewords/cutoff?i=${i}&v=${st.rec}`);
    setSt((s) => ({ ...s, mark2: evRef.current.length, phase: "confirm" }));
  };

  const bar = (peak, k) => (
    <div class="tn-bar" key={k}>
      <span style={`width:${Math.round((peak / 255) * 100)}%`} />
      <em class="num">{pct(peak)}</em>
    </div>
  );

  return (
    <div class="tn-panel">
      {st.phase === "nocap" && <p class="dim sm">{TEXT.tn_nocap}</p>}
      {st.phase === "gone" && <p class="t-warn sm">{TEXT.tn_gone}</p>}

      {st.phase === "listen" && (
        <>
          <p class="sm">{TEXT.tn_listen.replace("%s", String(st.left))}</p>
          <p class="dim sm">{TEXT.tn_listen_sub}</p>
        </>
      )}

      {(st.phase === "speak" || st.phase === "rec" || st.phase === "nogap") && (
        <>
          <p class="dim sm">{st.noise != null ? TEXT.tn_noise.replace("%s", pct(st.noise)) : TEXT.tn_noise_quiet}</p>
          <p class="sm">{TEXT.tn_speak.replace("%s", word)}</p>
          {st.attempts.map(bar)}
          {st.phase === "speak" && <p class="dim sm">({st.attempts.length} / 3)</p>}
          {st.vad > 0 && <p class="t-warn sm">{TEXT.tn_vad}</p>}
        </>
      )}

      {st.phase === "rec" && (
        <>
          <p class="sm">
            {TEXT.tn_rec
              .replace("%1", pct(st.rec))
              .replace("%2", st.noise != null ? pct(st.noise) : TEXT.tn_noise_floor)
              .replace("%3", pct(Math.min(...st.attempts)))}
          </p>
          <div class="tn-btns">
            <Btn solid onClick={apply}>
              {TEXT.tn_apply}
            </Btn>
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>
      )}

      {st.phase === "nogap" && (
        <>
          <p class="t-err sm">{TEXT.tn_nogap}</p>
          <div class="tn-btns">
            <Btn onClick={onClose}>{TEXT.cancel}</Btn>
          </div>
        </>
      )}

      {st.phase === "confirm" && <p class="sm ww-try">{TEXT.tn_confirm.replace("%s", word)}</p>}
      {st.phase === "done" && <p class="sm ww-heard">{TEXT.tn_heard}</p>}

      {st.phase !== "rec" && st.phase !== "nogap" && st.phase !== "done" && (
        <p class="dim sm">
          <button class="linkish" onClick={onClose}>
            {TEXT.cancel}
          </button>
        </p>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* One slot: collapsed control, swap states, assistant, sensitivity   */
/* ------------------------------------------------------------------ */

/** One wake word card's whole body: the picker, the swap line, the Assistant pairing, and the
 *  tuner-backed sensitivity. The card title carries the slot's name, so there is no label line. */
function SlotSection({ i, slot, swap, tuning, groups, otherWord, busy, assist, wakeRead, onPick, onCutoff, onRetry, onTune, onTuneClose }) {
  const [open, setOpen] = useState(false);

  // What the collapsed control says: the swap's optimistic word while one runs, otherwise what the
  // device reports. An empty slot says None in the dim voice.
  const swapping = swap && swap.phase === "busy";
  const failed = swap && swap.phase === "error";
  const word = swapping ? swap.word : slot?.w || "";
  // `ld` is the payload's "a model is loaded and listening" fact. An errored slot with a loaded
  // model is a failed swap whose previous word kept working - not a slot the device is retrying.
  const waiting = slot?.st === 3 || (slot?.st === 2 && !slot?.ld && isUrl(slot?.m));

  const pick = (entry) => {
    setOpen(false);
    onPick(entry ? { spec: entry.spec || entry.url, word: entry.word } : { spec: "none", word: "" });
  };

  return (
    <div class="ww-slot">
      <button class="ww-cur" aria-expanded={open} disabled={busy && !open} onClick={() => setOpen((v) => !v)}>
        {word ? <span class="ww-word">&ldquo;{word}&rdquo;</span> : <span class="dim">{TEXT.ww_none}</span>}
        <Chevron down={open} />
      </button>

      {open && <Picker groups={groups} current={swapping ? swap.spec : slot?.m || ""} otherWord={otherWord} busy={busy} onPick={pick} />}

      {/* The swap's own line: progress while it runs, the reason plus Retry when it failed, the
          "say it now" invitation when it landed. One line, swapped by state, under the control it
          describes. */}
      {swapping && (
        <p class="dim sm ww-note">
          {swap.tot > 0 ? `${TEXT.ww_downloading} ${Math.round((swap.dl || 0) / 1024)} / ${Math.round(swap.tot / 1024)} KB` : isUrl(swap.spec) ? TEXT.ww_downloading : TEXT.ww_loading}
        </p>
      )}
      {failed && (
        <p class="t-err sm ww-note">
          {TEXT.ww_failed} {WW_ERR[swap.err] || ""}{" "}
          <button class="linkish" onClick={onRetry}>
            {TEXT.ww_retry}
          </button>
        </p>
      )}
      {!swapping && !failed && waiting && (
        <p class="t-warn sm ww-note">{slot.st === 3 ? TEXT.ww_waiting : `${WW_ERR[slot.err] || ""} ${TEXT.ww_retrying}`}</p>
      )}

      {/* The tuner first, directly under the word it measures (owner's order, September 2026):
          which word, how readily it fires, then who answers it. Two buttons rather than a state
          dropdown - Tune Now runs the guided session, Reset Default hands the model its own tuning
          back - and a tuned word shows its measured result in the grey box below, the transcript
          subcard's neutral shape. Keyed on `ld` (a model is loaded), not on a READY state: a slot
          whose last swap failed keeps its previous word listening, and that word stays tunable. */}
      {word && slot?.ld && !swapping && (
        <>
          <Row label={TEXT.tn_row} hint={HINTS.wake_advanced}>
            <span class="tn-btns-row">
              <Btn cls="tn-go" onClick={onTune} disabled={busy || tuning}>
                {TEXT.tn_tune_now}
              </Btn>
              <Btn danger onClick={() => onCutoff(0)} disabled={busy || tuning || !slot.cut}>
                {TEXT.tn_reset_default}
              </Btn>
            </span>
          </Row>
          {slot.cut > 0 && !tuning && <div class="tn-box">{TEXT.tn_box.replace("%s", pct(slot.cut))}</div>}
          {tuning && <TunerPanel i={i} word={word} cut={slot.cut} wakeRead={wakeRead} onClose={onTuneClose} />}
        </>
      )}

      {/* The voice pipeline this word wakes, renamed from Assistant at the owner's call: it is
          Home Assistant's word for the thing being picked, and the tooltip says where they come
          from - these are built in Home Assistant, not here. */}
      {word && assist.ready && (
        <Row
          label={TEXT.vp_label}
          hint={
            <>
              {HINTS.voice_pipeline}{" "}
              <a href={TEXT.vp_docs_url} target="_blank" rel="noopener">
                {TEXT.vp_docs}
              </a>
            </>
          }
        >
          <Select
            value={assist.pipelineFor(word) ?? assist.fallbackPipeline() ?? PIPELINE_PREFERRED}
            options={[[PIPELINE_PREFERRED, TEXT.pipeline_preferred], ...assist.pipelines.map((p) => [p, p])]}
            disabled={assist.busy || swapping}
            onChange={(v) => assist.setPipeline(word, v)}
          />
        </Row>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The wake word cards                                                 */
/* ------------------------------------------------------------------ */

/**
 * The route's first three cards - Wake Word 1, Wake Word 2, Wake Word Settings - from one component
 * because the two word cards share state the settings card reads (a swap in flight disables both
 * pickers; the both-slots-off note lives with the settings). The owner's four-card order
 * (September 2026): word 1, word 2, settings, then sources, which the route appends.
 */
function WakeCards({ ctx, wake, wakeRead, setSlot, setCutoff, assist, groups }) {
  const chime = entity(ctx, "wake_sound");
  const stopWord = entity(ctx, "stop_word");

  const [swaps, setSwaps] = useState({}); // i -> {phase, word, spec, err, dl, tot}
  const [tuner, setTuner] = useState(null); // the slot with an open tuner panel, one at a time
  const alive = useRef(true);
  useEffect(() => () => (alive.current = false), []);

  const slots = wake?.slots || [];
  const slotAt = (i) => slots.find((s) => s.i === i);
  const anyBusy = Object.values(swaps).some((s) => s?.phase === "busy");

  /**
   * Writes the swap into Home Assistant's wake word selects, directly and from a fresh payload.
   *
   * Not through useAssist's syncSlot, deliberately: its free-slot guard reads the closure this
   * function captured before the swap, where the old word still looks enabled and so its slot
   * still looks taken - and the write silently never happened (hardware-found: the device said
   * "Hey Nexus", Home Assistant's dropdown listed it, and the selection sat on the old word).
   * Home Assistant's select never self-selects either - reading its source settled that: it keeps
   * its stored option, only *validates* it against the device's active list, and only a
   * select_option call moves it. So this is that call, aimed by what the fresh payload actually
   * says: the select naming the word we replaced, else one holding no_wake_word, else this slot's
   * position.
   *
   * Write-and-verify with retries, not one blind write behind a fixed sleep: after a swap the
   * device drops its Home Assistant connection so the wake word list is re-read (nudge_ha_), and
   * a select write sent while Home Assistant is reconnecting - or naming an option it has not
   * re-read yet - is silently lost. Each pass reads the selects fresh, writes only what still
   * disagrees, refreshes, and checks again; the deadline covers a slow reconnect without hanging
   * on a Home Assistant that is genuinely gone. Repeating an already-landed write is harmless -
   * same entity, same option.
   */
  const syncAssist = async (i, prevWord, nextWord) => {
    const named = (sel, w) => sel.findIndex((x) => w && (x[1] || "").toLowerCase() === w.toLowerCase());
    const deadline = Date.now() + 30000;
    for (;;) {
      const fresh = await requestJson("/api/sat1/ha").catch(() => null);
      const sel = fresh?.d?.asst?.s;
      if (!Array.isArray(sel) || !sel.length) return;
      let entity, option;
      if (!nextWord) {
        // The slot emptied: clear whichever select named the removed word, and nothing else.
        const off = named(sel, prevWord);
        if (off < 0) return; // nothing names it (any more) - done
        entity = sel[off][0];
        option = NO_WAKE_WORD;
      } else {
        if (named(sel, nextWord) >= 0) return; // Home Assistant agrees - done
        let at = named(sel, prevWord);
        if (at < 0) at = sel.findIndex((x) => x[1] === NO_WAKE_WORD);
        if (at < 0) at = Math.min(i, sel.length - 1);
        entity = sel[at][0];
        option = nextWord;
      }
      await post(`/api/sat1/ha/select?e=${encodeURIComponent(entity)}&o=${encodeURIComponent(option)}`).catch(() => {});
      // The refresh both settles the round trip (~3s of spaced reads) and hands the Voice
      // Pipeline dropdowns the payload they render from; the next pass verifies against it.
      await ctx.haRefresh();
      if (!alive.current || Date.now() > deadline) return;
    }
  };

  const choose = async (i, { spec, word }) => {
    const prevWord = slotAt(i)?.w || "";
    if (tuner === i) setTuner(null);
    setSwaps((s) => ({ ...s, [i]: { phase: "busy", word, spec } }));
    const r = await setSlot(i, spec).catch(() => ({ ok: false }));
    if (!r.ok) {
      if (alive.current) setSwaps((s) => ({ ...s, [i]: { phase: "error", word, spec, err: 0 } }));
      return;
    }
    // The write is queued; this poll of the same GET is the only truthful confirmation. URLs get a
    // download's worth of patience, built-ins a couple of loop iterations.
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
          // A downloaded word is not in Home Assistant's cached options until the device drops the
          // API connection and Home Assistant reconnects onto the fresh list (nudge_ha_). No fixed
          // sleep here: syncAssist verifies each write and retries through that reconnect window.
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

  if (!slots.length) return <Missing what="Wake word control" />;

  const allOff = slots.every((s) => !s.m);

  // The no-assistants note, per word card: it explains the Assistant dropdown missing from *this*
  // card, and a reader on Wake Word 2 should not have to scroll to card 1 to learn why.
  const assistNote = !assist.ready && (
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

  const wordCard = (i, title, hint) => (
    <Card title={title} icon={N_WAKE} hint={hint}>
      <SlotSection
        i={i}
        slot={slotAt(i)}
        swap={swaps[i]}
        tuning={tuner === i}
        groups={groups}
        otherWord={slotAt(1 - i)?.w || swaps[1 - i]?.word || ""}
        busy={anyBusy}
        assist={assist}
        wakeRead={wakeRead}
        onPick={(p) => choose(i, p)}
        onCutoff={async (v) => {
          await setCutoff(i, v);
          await sleep(400);
          wakeRead();
        }}
        onRetry={() => choose(i, { spec: swaps[i].spec, word: swaps[i].word })}
        onTune={() => setTuner(i)}
        onTuneClose={() => {
          setTuner(null);
          wakeRead();
        }}
      />
      {slotAt(i)?.w && assistNote}
    </Card>
  );

  return (
    <>
      {wordCard(0, TEXT.ww_card1, HINTS.wake_words)}
      {wordCard(1, TEXT.ww_card2, null)}

      <Card title={TEXT.ww_settings} icon={N_WSET}>
        {chime && (
          <Row label="Wake chime" hint={HINTS.wake_sound}>
            <Toggle
              checked={chime.value === true || chime.state === "ON"}
              onChange={(v) => post(pathFor(ctx, "wake_sound", v ? "turn_on" : "turn_off"))}
            />
          </Row>
        )}

        {stopWord && (
          <Row label={'Say "stop" to interrupt'} hint={HINTS.stop_word}>
            <Toggle
              checked={stopWord.value === true || stopWord.state === "ON"}
              onChange={(v) => post(pathFor(ctx, "stop_word", v ? "turn_on" : "turn_off"))}
            />
          </Row>
        )}

        {/* With both cards above on None, the one consequence worth a sentence. It lives here so the
            word cards stay controls rather than carrying a shared warning twice. */}
        {allOff && <p class="dim sm">{TEXT.no_wake_words}</p>}
      </Card>
    </>
  );
}

/* ------------------------------------------------------------------ */
/* The Wake Word Sources card                                          */
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
    <Card title={TEXT.ws_title} icon={N_WSRC} hint={HINTS.wake_sources}>
      {sources.map((s) => {
        const c = cat[s.url];
        return (
          <div key={s.url} class="ws-row">
            <div class="ws-name">
              {/* The name is the link to the actual repository (owner request, September 2026):
                  which exact repo feeds the list should be one tap to verify, not a guess. */}
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

      {/* The owner's sentence, verbatim: "Don't see your wake word? Request one, or train your own
          microWakeWord." - each half a link into Tater's world, the request anchor landing on the
          README section that explains the free issue-driven trainer. */}
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
  const { wake, wakeRead, setSlot, setCutoff } = useWakeSlots();

  const [sources, setSourcesState] = useState(readSources);
  const [cat, setCat] = useState({});

  const setSources = (list) => {
    setSourcesState(list);
    writeSources(list);
  };

  // Enumerate every source once per page load; a source added later enumerates on arrival. The
  // promise cache in wakesources.js makes re-renders free.
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

  const activeWords = (wake?.slots || []).filter((s) => s.m && s.w).map((s) => s.w);
  const assist = useAssist(ha, haRefresh, activeWords);

  useEffect(() => {
    haSyncOnce(haRefresh);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  if (!ctx.device) return <Missing what="the device" />;

  // Included first (the device's own list, so it needs no network), then one group per source.
  // ESPHome's experiments folder is excluded at enumeration - "minimally trained and tested, not
  // supported in any way", their words, and hardware time proved it (owner, September 2026).
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

  return (
    <>
      <WakeCards ctx={ctx} wake={wake} wakeRead={wakeRead} setSlot={setSlot} setCutoff={setCutoff} assist={assist} groups={groups} />
      <SourcesCard sources={sources} setSources={setSources} cat={cat} />
    </>
  );
}
