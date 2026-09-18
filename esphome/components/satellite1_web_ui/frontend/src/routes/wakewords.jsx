/**
 * The Wake Words route (route id "wake-word"): which words the device answers to, how readily, and
 * what it does the moment it hears one.
 *
 * One card, and still a route of its own, per the owner's September 2026 information-architecture
 * pass: these five rows are the first thing a new owner goes looking for, and they were the third
 * card down a route named Audio, which is not where anyone would look for "can I turn off Hey
 * Jarvis". The card kept its shape in the move from the Audio route so nothing about how the
 * controls work changed, only where they are found.
 *
 * Each wake word's dropdown is the one place in this app where the two halves of a single control
 * come from opposite ends, and it is worth knowing which is which before changing either.
 *
 * Whether the device is listening for a wake word is the device's own, and is the only thing the app
 * controls that no entity carries: micro_wake_word makes neither a switch nor a select for its
 * models, so nothing about them reaches /events, and the list comes from /api/sat1/wakewords.
 *
 * Which assistant answers it is the opposite case, and the only value on any card that Home
 * Assistant owns outright. It arrives with the area and player tree and is written back through
 * select.select_option, because the pairing is kept on Home Assistant's side - see the `asst` notes
 * in common/web_ui_ha.yaml.
 */
import { useEffect } from "preact/hooks";

import { HINTS, TEXT } from "../copy.js";
import {
  ASSIST_SLOTS,
  NO_WAKE_WORD,
  PIPELINE_PREFERRED,
  entity,
  haBlocked,
  haSyncOnce,
  haTooOld,
  pathFor,
  post,
  useAssist,
  useWakeWords,
} from "../lib/device.js";
import { Card, Missing, N_WAKE, Row, Select, Toggle } from "../ui.jsx";

function WakeWordsCard({ ctx, wake, assist }) {
  const chime = entity(ctx, "wake_sound");
  const sens = entity(ctx, "wake_sensitivity");
  const stopWord = entity(ctx, "stop_word");
  const words = wake.words || [];

  // Off, then Preferred, then the customer's pipelines - already sorted case-insensitively by the payload.
  //
  // The first two carry labels rather than their own names, because both are values rather than pipelines:
  // `preferred` means whichever one Home Assistant has been told to prefer, and `no_wake_word` is what
  // Home Assistant stores in a slot that holds nothing. Reusing its constant for Off rather than inventing
  // a sentinel keeps the whole value space Home Assistant's, so nothing here can collide with a pipeline
  // name except by someone naming one `no_wake_word`.
  const choices = [
    [NO_WAKE_WORD, TEXT.pipeline_off],
    [PIPELINE_PREFERRED, TEXT.pipeline_preferred],
    ...assist.pipelines.map((p) => [p, p]),
  ];

  // Anything the list does not contain becomes Preferred, which is where Home Assistant itself lands a
  // pipeline that has been deleted - see _update_options in its assist_pipeline select.
  const pick = (v) => (choices.some(([value]) => value === v) ? v : PIPELINE_PREFERRED);

  if (!chime && !sens && !stopWord && !words.length) return <Missing what="Wake word control" />;

  return (
    <Card title="Wake Words" icon={N_WAKE}>
      {/* First, because which words the device answers to is a bigger question than how readily it
          answers, and someone scanning for "can I turn off Hey Jarvis" should not have to read past
          a dropdown to find it.

          One control per wake word, not a switch and a dropdown. Off is the first item in the list rather
          than a separate toggle, because there is only one decision here - whether this device answers to
          this word, and if so which assistant answers - and that is also the shape Home Assistant stores:
          a slot either names a wake word and a pipeline, or holds no_wake_word and means nothing at all.
          Two controls made it look like two settings, one of which could contradict the other.

          The label carries the wake word in quotes because it is a phrase someone says out loud, not a
          setting name. "Hey Jarvis" wake word reads as the thing it is; Hey Jarvis on its own beside a
          dropdown of assistant names reads like another one of them. */}
      {words.map((w) => (
        <Row key={w.i} label={`"${w.w}" wake word`} hint={w.i === 0 ? HINTS.wake_words : null}>
          {assist.ready ? (
            <Select
              // Off when the device is not listening for it. Otherwise the assistant its slot names, or
              // the first slot's, which is where Home Assistant sends anything it cannot match.
              //
              // Forced onto a value the list actually contains, because a <select> given a value no option
              // matches displays its first option instead - which here is Off, and would have read as "not
              // listening" for a wake word that is. Reachable without any bug of ours: a slot whose entity
              // is momentarily unavailable reports "unavailable" as its state, and that is not a pipeline.
              value={pick(w.on ? assist.pipelineFor(w.w) ?? assist.fallbackPipeline() : NO_WAKE_WORD)}
              options={choices}
              disabled={assist.busy}
              onChange={async (v) => {
                if (v === NO_WAKE_WORD) {
                  await wake.set(w.i, false);
                  // Clearing the slot is the second half of the same decision, not a separate feature.
                  // Home Assistant answers a slot write by pushing the union of its slots back as the
                  // complete set of active wake words, so a slot still naming a wake word that is off is
                  // one Home Assistant will switch on again.
                  await assist.syncSlot(w.w, false);
                  return;
                }
                // The device flag first, because it is local and instant, and because setPipeline may have
                // to claim a slot - which Home Assistant answers by rewriting the whole active set.
                if (!w.on) await wake.set(w.i, true);
                await assist.setPipeline(w.w, v);
              }}
            />
          ) : (
            // No Home Assistant, so there is no assistant to choose and a dropdown of one real option
            // would be a worse lie than a switch. Whether the device listens is its own to answer, so
            // this half keeps working; assistant_needs_ha below says what the other half costs.
            <Toggle checked={w.on} onChange={(v) => wake.set(w.i, v)} />
          )}
        </Row>
      ))}

      {/* Wake words to switch, but no assistants to point them at. Worth one line rather than silently
          falling back to switches, because the reader was told this could be set. Three variants for
          three causes, most specific first: the actions checkbox (with the fix one tap away), an old
          Home Assistant, and plain unreachable. */}
      {words.length > 0 && !assist.ready && (
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
      )}

      {/* More wake words listening than Home Assistant has slots to pair them with, which needs a build
          with three or more models - this product has two. Home Assistant sends anything it cannot match
          to the first slot's assistant, so the surplus words work but share, and the dropdowns will show
          that shared answer rather than what was picked for them. Said plainly because the alternative is
          a dropdown that quietly disagrees with itself. */}
      {words.filter((w) => w.on).length > ASSIST_SLOTS && <p class="dim sm">{TEXT.assistant_slots_full}</p>}

      {/* Only when the device offered a list and every one of them is off. Not a warning: it is a
          legitimate thing to want, and unlike most ways of silencing a device it undoes itself from
          this same card. It is worth saying out loud because the symptom - a device that ignores you -
          looks like a fault rather than like a setting. */}
      {words.length > 0 && words.every((w) => !w.on) && <p class="dim sm">{TEXT.no_wake_words}</p>}

      {/* "Wake words", plural, per the owner: it is one sensitivity shared by every word above, and the
          singular read as if it belonged to whichever word the eye had just left. */}
      {sens && (
        <Row label="Wake words sensitivity" hint={HINTS.wake_sensitivity}>
          <Select
            value={sens.value}
            options={sens.option}
            onChange={(v) => post(`${pathFor(ctx, "wake_sensitivity", "set")}?option=${encodeURIComponent(v)}`)}
          />
        </Row>
      )}

      {/* The chime is an output, but it lives here rather than on the Audio route on purpose: it exists
          only to answer a wake word, so it belongs with the words. Someone turning it off is thinking
          about being woken, not about the speaker. */}
      {chime && (
        <Row label="Wake chime" hint={HINTS.wake_sound}>
          <Toggle
            checked={chime.value === true || chime.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "wake_sound", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}

      {/* Last, at the owner's chosen order - it is the one word here the device listens for while it is
          talking rather than while it is idle. The label quotes the word for the same reason the wake
          word rows do: it is said, not set. */}
      {stopWord && (
        <Row label={'Say "stop" to interrupt'} hint={HINTS.stop_word}>
          <Toggle
            checked={stopWord.value === true || stopWord.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "stop_word", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}
    </Card>
  );
}

export function WakeWords({ ctx }) {
  const { ha, haRefresh } = ctx;
  const wake = useWakeWords();

  // The wake words that are listening, by the name Home Assistant knows them by, which is what decides
  // whether a slot is free: taking one held by another listening wake word would switch that one off.
  const assist = useAssist(
    ha,
    haRefresh,
    (wake.words || []).filter((w) => w.on).map((w) => w.w)
  );

  // Once per page load, shared with the Audio route - whichever mounts first asks.
  useEffect(() => {
    haSyncOnce(haRefresh);
    // haRefresh is stable for the life of the app and haSyncOnce guards itself.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  if (!ctx.device) return <Missing what="the device" />;

  return <WakeWordsCard ctx={ctx} wake={wake} assist={assist} />;
}
