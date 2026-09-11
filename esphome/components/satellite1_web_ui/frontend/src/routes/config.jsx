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
import { HINTS, TEXT } from "../copy.js";
import { HA_NEVER, entity, pathFor, post } from "../lib/device.js";
import { TargetTree } from "../tree.jsx";
import { Btn, Card, Missing, Row, Slider, Toggle } from "../ui.jsx";

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
  if (!ha) return null;
  if (ha.rung === -1) return TEXT.ha_refused;
  if (ha.age === HA_NEVER) return ctx.device?.ha ? TEXT.ha_pending : TEXT.ha_never;
  if (!ha.d) return TEXT.ha_never;
  const empty = (!ha.d.areas || ha.d.areas.length === 0) && (!ha.d.loose || ha.d.loose.length === 0);
  if (empty) return TEXT.ha_no_players;
  if (!ha.d.area) return TEXT.ha_no_area;
  return null;
}

function HaState({ problem, ha, refresh, refreshing }) {
  return (
    <div class="ctl-sub habox">
      {problem && <div class="banner warn">{problem}</div>}
      {!problem && ha?.d?.t === 1 && <div class="banner warn">{TEXT.ha_truncated}</div>}
      <div class="harow">
        <span class="dim xs">
          {ha && ha.age !== HA_NEVER ? `From Home Assistant ${ha.age}s ago` : ""}
        </span>
        <Btn onClick={refresh} disabled={refreshing}>
          {refreshing ? TEXT.ha_refreshing : TEXT.ha_refresh}
        </Btn>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Remote routing                                                      */
/* ------------------------------------------------------------------ */

/**
 * What the Remote TTS volume slider actually does, which is three different things.
 *
 * Sonos reads the level off the announcement itself. Another Satellite1 has its Voice Override set to
 * this for the duration of the answer and put back afterwards. Everything else has its media volume
 * set before the announcement and restored after. One slider with three behaviours behind it is the
 * kind of thing that gets reported as a bug, so it says so.
 *
 * Deliberately not a per-target table, which is what the plan drew. Which of the three applies depends
 * on Sonos membership and on the target's own override value, and neither is in the payload - so a
 * table would be a confident guess per row. Saying it once and accurately is better than saying it per
 * target and wrongly.
 */
function Mechanisms() {
  return (
    <p class="ctl-sub">
      How this reaches each speaker depends on what it is. Sonos takes the level from the announcement
      itself. Another Satellite1 has its Voice Override set to this while the answer plays, then put
      back. Everything else has its media volume set and restored the same way.
    </p>
  );
}

function RemoteRouting({ ctx, ha, sel, write, refresh, refreshing }) {
  const vol = useEntity(ctx, "remote_tts_volume");
  const chime = useEntity(ctx, "remote_wake_chime");

  const problem = haProblem(ctx, ha);
  // Anything chosen means a response is going somewhere other than this speaker, which is the same
  // question ${tts_routing_active} asks on the device. The old master switch is gone; there is nothing
  // left that can be on with an empty list.
  const active = sel.routing.areas.size > 0 || sel.routing.extra.size > 0;

  return (
    <Card title="Remote routing">
      <div class="ctl-label">
        <span>Play responses on</span>
      </div>
      <HaState problem={problem} ha={ha} refresh={refresh} refreshing={refreshing} />
      <TargetTree
        payload={problem ? null : ha.d}
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
      {active && <Mechanisms />}

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

function AreaDucking({ ctx, ha, sel, write, refresh, refreshing }) {
  const vol = useEntity(ctx, "duck_volume");

  const problem = haProblem(ctx, ha);
  const active = sel.duck.areas.size > 0 || sel.duck.extra.size > 0;

  return (
    <Card title="Area ducking">
      <div class="ctl-label">
        <span>Quieten while talking</span>
      </div>
      <HaState problem={problem} ha={ha} refresh={refresh} refreshing={refreshing} />
      {/* No Local Speaker row: this device's own volume while it is talking is the voice level, which
          lives on Controls, and its own player is filtered out of the payload anyway. Every area in
          the house is offered, not just this device's own - ducking a room this device is not in is a
          deliberate capability of the redesign rather than a side effect. */}
      <TargetTree
        payload={problem ? null : ha.d}
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

export function Config({ ctx }) {
  const { ha, haRefresh, haRefreshing, sel, selError, selWrite } = ctx;

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
        refresh={haRefresh}
        refreshing={haRefreshing}
      />
      <AreaDucking
        ctx={ctx}
        ha={ha}
        sel={sel}
        write={selWrite}
        refresh={haRefresh}
        refreshing={haRefreshing}
      />
    </>
  );
}
