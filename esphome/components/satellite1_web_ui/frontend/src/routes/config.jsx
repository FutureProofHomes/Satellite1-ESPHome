/**
 * Config: the settings that need Home Assistant to know what they are choosing between.
 *
 * Two kinds of control live here and they are stored differently, which is worth knowing before
 * reading further. The sliders and the wake chime toggle write to real ESPHome entities, exactly as
 * they always did. The two trees write the device's own selection at /api/sat1/sel, which is not an
 * entity at all - it used to be, and a text entity caps at 255 characters, which is about half of what
 * one area's worth of players needs.
 *
 * The two switches Home Assistant still shows for this - "Route TTS To All Area Players" and "Duck All
 * Area Players" - are projections of that selection rather than separate settings. Ticking this
 * device's own area in a tree turns the matching switch on; unticking any single player in it turns the
 * switch off. There is nothing to keep in step, because there is only one value.
 *
 * Every card degrades on its own. A missing data layer greys the trees and says which of the five
 * reasons applies and what to do about it, while the sliders keep working, because they are device
 * state that applies the moment Home Assistant comes back.
 */
import { useEffect } from "preact/hooks";

import { HINTS, TEXT } from "../copy.js";
import {
  ASSIST_SLOTS,
  HA_NEVER,
  NO_WAKE_WORD,
  PIPELINE_PREFERRED,
  entity,
  pathFor,
  post,
  useAssist,
  useWakeWords,
} from "../lib/device.js";
import { TargetTree } from "../tree.jsx";
import { Card, Missing, Row, Select, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* Reading and writing the entity-backed controls                      */
/* ------------------------------------------------------------------ */

function useEntity(ctx, key) {
  const e = entity(ctx, key);
  return {
    exists: !!ctx.device?.e?.[key],
    e,
    on: e ? e.value === true || e.state === "ON" || e.state === "on" : false,
    num: e ? Number(e.value ?? e.state) : 0,
  };
}

function writeSwitch(ctx, key, next) {
  const p = pathFor(ctx, key, next ? "turn_on" : "turn_off");
  if (p) post(p);
}

function writeNumber(ctx, key, value) {
  const p = pathFor(ctx, key, "set", { value });
  if (p) post(p);
}

/* ------------------------------------------------------------------ */
/* This device's own voice and audio settings                           */
/* ------------------------------------------------------------------ */

/**
 * Which wake words are armed, and how hard the device listens for them.
 *
 * These rows were a "Voice" card on Controls. They moved because they are set once and then left, which
 * is what this route is for, while Controls keeps what you watch. The name is "Voice Input" rather than
 * "Voice" so that it pairs with Audio Output below it: between them they are the two directions sound
 * travels through the device, and naming them that way means neither has to explain which it is.
 *
 * Each wake word's dropdown is the one place in this app where the two halves of a single control come from
 * opposite ends, and it is worth knowing which is which before changing either.
 *
 * Whether the device is listening for a wake word is the device's own, and is the only thing the app
 * controls that no entity carries: micro_wake_word makes neither a switch nor a select for its models, so
 * nothing about them reaches /events, and the list comes from /api/sat1/wakewords.
 *
 * Which assistant answers it is the opposite case, and the only value on any card that Home Assistant owns
 * outright. It arrives with the area and player tree and is written back through select.select_option,
 * because the pairing is kept on Home Assistant's side - see the `asst` notes in common/web_ui_ha.yaml.
 */
function VoiceInput({ ctx, wake, assist }) {
  const mute = entity(ctx, "mute_mics");
  const chime = entity(ctx, "wake_sound");
  const sens = entity(ctx, "wake_sensitivity");
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

  if (!mute && !chime && !sens && !words.length) return null;

  return (
    <Card title="Voice Input">
      {mute && (
        <Row label="Mute microphones" hint={HINTS.mute}>
          <Toggle
            checked={mute.value === true || mute.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "mute_mics", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}

      {/* Above sensitivity, because which words the device answers to is a bigger question than how
          readily it answers, and someone scanning for "can I turn off Hey Jarvis" should not have to
          read past a dropdown to find it.

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
          falling back to switches, because the reader was told this could be set. */}
      {words.length > 0 && !assist.ready && <p class="dim sm">{TEXT.assistant_needs_ha}</p>}

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

      {sens && (
        <Row label="Wake word sensitivity" hint={HINTS.wake_sensitivity}>
          <Select
            value={sens.value}
            options={sens.option}
            onChange={(v) => post(`${pathFor(ctx, "wake_sensitivity", "set")}?option=${encodeURIComponent(v)}`)}
          />
        </Row>
      )}

      {/* The chime is an output, and it is here rather than in Audio Output on purpose: it exists only
          to answer a wake word, so it belongs with the words. Someone turning it off is thinking about
          being woken, not about the speaker. */}
      {chime && (
        <Row label="Wake chime" hint={HINTS.wake_sound}>
          <Toggle
            checked={chime.value === true || chime.state === "ON"}
            onChange={(v) => post(pathFor(ctx, "wake_sound", v ? "turn_on" : "turn_off"))}
          />
        </Row>
      )}
    </Card>
  );
}

/**
 * This device's own speaker: how it is wired, and how loud it speaks.
 *
 * Assistant volume came here with the card rather than staying on Controls, which puts it one card away
 * from Remote routing's "Remote TTS volume" - the pair the plan says must never be confused for each
 * other. They are not separated by distance any more, so each says which speakers it moves in its own
 * hint, which is the more reliable of the two protections anyway: a reader who has both on screen can
 * compare them, where a reader who has to remember the other one cannot.
 */
function AudioOutput({ ctx }) {
  const chan = entity(ctx, "speaker_channel");
  const lineOut = entity(ctx, "line_out");
  const vol = entity(ctx, "voice_override");
  if (!chan && !lineOut && !vol) return null;

  return (
    <Card title="Audio Output">
      {vol && (
        <Row label="Assistant volume" hint={HINTS.voice_override}>
          <Slider
            value={Number(vol.value)}
            min={Number(vol.min_value ?? 0)}
            max={Number(vol.max_value ?? 100)}
            step={Number(vol.step ?? 1)}
            format={(v) => (v === 0 ? "follow media" : `${v}%`)}
            onCommit={(v) => post(`${pathFor(ctx, "voice_override", "set")}?value=${v}`)}
          />
        </Row>
      )}
      {chan && (
        <Row label="Channel" hint={HINTS.speaker_channel}>
          <Select
            value={chan.value}
            options={chan.option}
            onChange={(v) => post(`${pathFor(ctx, "speaker_channel", "set")}?option=${encodeURIComponent(v)}`)}
          />
        </Row>
      )}
      {lineOut && (
        <Row label="Line out">
          <span class="dim">{lineOut.value ? "Connected" : "Nothing plugged in"}</span>
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Why the trees are not here                                          */
/* ------------------------------------------------------------------ */

/**
 * One of five states, never a generic failure. `rung` is what separates them: 0 means the device has
 * not asked yet, which on a fresh boot is a five-second wait rather than a problem, and -1 means both
 * rungs of the ladder were refused, which has a specific fix.
 *
 * "No area" is still reported, and still matters, but it no longer stops the trees being useful: with
 * no area of its own a device can still route to and duck any other room. What it cannot do is answer
 * the two whole-my-area switches, which is why the message stays.
 */
function haProblem(ctx, ha) {
  // No response yet, which is a wait rather than a fault, and ha_pending is exactly that sentence. This
  // used to return null, which said "no problem, go ahead and read ha.d" about a payload that was not
  // there - so whether the route crashed came down to whether /api/sat1/sel answered before
  // /api/sat1/ha, since the selection is what gates the first render.
  if (!ha) return TEXT.ha_pending;
  if (ha.rung === -1) return TEXT.ha_refused;
  if (ha.age === HA_NEVER) return ctx.device?.ha ? TEXT.ha_pending : TEXT.ha_never;
  if (!ha.d) return TEXT.ha_never;
  const empty = (!ha.d.areas || ha.d.areas.length === 0) && (!ha.d.loose || ha.d.loose.length === 0);
  if (empty) return TEXT.ha_no_players;
  if (!ha.d.area) return TEXT.ha_no_area;
  return null;
}

/**
 * Only the problems now. The age line and the Refresh button that used to sit under them are gone: the
 * sync happens once when the app loads (see Config), so there is nothing here for anyone to operate, and
 * a timestamp on a list of speakers is an answer to a question nobody was asking.
 *
 * Renders nothing at all rather than an empty box, which would otherwise leave its own margins behind
 * above every tree.
 */
function HaState({ problem, ha }) {
  const truncated = !problem && ha?.d?.t === 1;
  if (!problem && !truncated) return null;
  return (
    <div class="ctl-sub habox">
      {problem && <div class="banner warn">{problem}</div>}
      {truncated && <div class="banner warn">{TEXT.ha_truncated}</div>}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Remote routing                                                      */
/* ------------------------------------------------------------------ */

/*
 * The three mechanisms behind the Remote TTS volume slider - Sonos reading the level off the announcement,
 * another Satellite1 having its Voice Override set and put back, everything else having its media volume
 * set and restored - are now in HINTS.remote_tts_volume rather than a paragraph under the slider. Still
 * said somewhere, because one slider with three behaviours behind it gets reported as a bug, but said
 * where it explains a control instead of occupying the page permanently.
 *
 * Deliberately still not a per-target table, which is what the plan drew. Which of the three applies
 * depends on Sonos membership and on the target's own override value, and neither is in the payload, so
 * a table would be a confident guess per row.
 */

function RemoteRouting({ ctx, ha, sel, write }) {
  const vol = useEntity(ctx, "remote_tts_volume");
  const chime = useEntity(ctx, "remote_wake_chime");

  const problem = haProblem(ctx, ha);
  // Anything chosen means a response is going somewhere other than this speaker, which is the same
  // question ${tts_routing_active} asks on the device. The old master switch is gone; there is nothing
  // left that can be on with an empty list.
  const active = sel.routing.areas.size > 0 || sel.routing.extra.size > 0;

  return (
    <Card title="Remote routing" hint={HINTS.remote_routing}>
      {/* "Play responses on" left it open which responses - the device also plays media, and a bare
          "responses" next to a list of speakers reads as either. */}
      <div class="ctl-label">
        <span>Play assistant responses on selected players</span>
      </div>
      <HaState problem={problem} ha={ha} />
      <TargetTree
        payload={problem ? null : ha?.d}
        sel={sel.routing}
        onSel={(next) => write({ ...sel, routing: next })}
        local={sel.local}
        onLocal={(v) => write({ ...sel, local: v })}
      />

      {vol.exists && (
        <Row label="Remote TTS volume" hint={HINTS.remote_tts_volume}>
          <Slider
            value={vol.num}
            min={vol.e?.min_value ?? 0}
            max={vol.e?.max_value ?? 100}
            step={vol.e?.step ?? 1}
            disabled={!active}
            format={(v) => `${Math.round(v)}%`}
            onCommit={(v) => writeNumber(ctx, "remote_tts_volume", v)}
          />
        </Row>
      )}

      {chime.exists && (
        <Row label="Remote wake chime" hint={HINTS.remote_wake_chime}>
          <Toggle
            checked={chime.on}
            disabled={!active}
            onChange={(v) => writeSwitch(ctx, "remote_wake_chime", v)}
          />
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Area ducking                                                        */
/* ------------------------------------------------------------------ */

function AreaDucking({ ctx, ha, sel, write }) {
  const vol = useEntity(ctx, "duck_volume");

  const problem = haProblem(ctx, ha);
  const active = sel.duck.areas.size > 0 || sel.duck.extra.size > 0;

  return (
    <Card title="Area ducking" hint={HINTS.area_ducking}>
      {/* Was "Quieten while talking", which was wrong as well as vague: the duck starts at the wake word
          and holds until the answer ends, so it is also quiet while the device listens. */}
      <div class="ctl-label">
        <span>Lower the volume on selected players upon wake word detection</span>
      </div>
      <HaState problem={problem} ha={ha} />
      {/* No Local Speaker row: this device's own volume while it is talking is the voice level, which
          lives on Controls, and its own player is filtered out of the payload anyway. Every area in
          the house is offered, not just this device's own - ducking a room this device is not in is a
          deliberate capability of the redesign rather than a side effect. */}
      <TargetTree
        payload={problem ? null : ha?.d}
        sel={sel.duck}
        onSel={(next) => write({ ...sel, duck: next })}
        local={null}
      />

      {vol.exists && (
        <Row label="Duck volume" hint={HINTS.duck_volume}>
          <Slider
            value={vol.num}
            min={vol.e?.min_value ?? 0}
            max={vol.e?.max_value ?? 100}
            step={vol.e?.step ?? 1}
            disabled={!active}
            format={(v) => `${Math.round(v)}%`}
            onCommit={(v) => writeNumber(ctx, "duck_volume", v)}
          />
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */

/**
 * Whether this page load has already asked Home Assistant for a fresh list.
 *
 * Module scope, not component state, and deliberately not reset on unmount: the sync is once per load of
 * the app, so switching to Diagnostics and back does not ask again. Reloading the page is the gesture that
 * means "look again", and it clears this by definition.
 *
 * The sync matters because GET /api/sat1/ha returns the device's cached copy. The device only re-asks
 * Home Assistant 5s after the native API connects, or when something POSTs /api/sat1/ha/refresh - there is
 * no interval. Without this, a speaker added in Home Assistant would not appear here until Home Assistant
 * reconnected or the device rebooted, no matter how many times the page was reloaded.
 */
let askedThisLoad = false;

export function Config({ ctx }) {
  const { ha, haRefresh, sel, selError, selWrite } = ctx;
  const wake = useWakeWords();

  // The wake words that are listening, by the name Home Assistant knows them by, which is what decides
  // whether a slot is free: taking one held by another listening wake word would switch that one off.
  const assist = useAssist(
    ha,
    haRefresh,
    (wake.words || []).filter((w) => w.on).map((w) => w.w)
  );

  useEffect(() => {
    if (askedThisLoad) return;
    askedThisLoad = true;
    haRefresh();
    // haRefresh is stable for the life of the app and the guard above makes this run once regardless.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  if (!ctx.device) return <Missing what="the device" />;

  return (
    <>
      <VoiceInput ctx={ctx} wake={wake} assist={assist} />
      <AudioOutput ctx={ctx} />

      {/* The selection endpoint gates the two trees and nothing else. It used to return early for the
          whole route, which was true when every card here was about the selection; the two above are
          this device's own settings and have no reason to disappear because a different endpoint
          failed. Still distinct from a device that is not there: the shell is up, so this is the
          selection specifically. */}
      {sel ? (
        <>
          {selError && <div class="banner warn">{TEXT.sel_failed}</div>}
          <RemoteRouting ctx={ctx} ha={ha} sel={sel} write={selWrite} />
          <AreaDucking ctx={ctx} ha={ha} sel={sel} write={selWrite} />
        </>
      ) : (
        <Missing what={selError ? "the saved selection" : "settings"} />
      )}
    </>
  );
}
