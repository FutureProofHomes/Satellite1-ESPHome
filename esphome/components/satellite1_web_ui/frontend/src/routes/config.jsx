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
import { HA_NEVER, entity, pathFor, post } from "../lib/device.js";
import { TargetTree } from "../tree.jsx";
import { Card, Missing, Row, Slider, Toggle } from "../ui.jsx";

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

  useEffect(() => {
    if (askedThisLoad) return;
    askedThisLoad = true;
    haRefresh();
    // haRefresh is stable for the life of the app and the guard above makes this run once regardless.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  if (!ctx.device) return <Missing what="the device" />;
  // Distinct from a device that is not there: the shell is up, so this is the selection endpoint
  // specifically, and every control on this route is about the selection or gated on it.
  if (!sel) return <Missing what={selError ? "the saved selection" : "settings"} />;

  return (
    <>
      {selError && <div class="banner warn">{TEXT.sel_failed}</div>}
      <RemoteRouting
        ctx={ctx}
        ha={ha}
        sel={sel}
        write={selWrite}
      />
      <AreaDucking ctx={ctx} ha={ha} sel={sel} write={selWrite} />
    </>
  );
}
