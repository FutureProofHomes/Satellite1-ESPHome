/**
 * Config: the settings that need Home Assistant to know what they are choosing between.
 *
 * Everything here writes to an entity that already exists in tts_routing.yaml or area_ducking.yaml, so
 * this is a second face on the same controls Home Assistant shows rather than a parallel set of
 * settings that could disagree with them. What the device cannot know unaided - which areas exist and
 * which speakers are in them - comes from GET /api/sat1/ha.
 *
 * Every card degrades on its own. A missing data layer greys the lists and says which of the five
 * reasons applies and what to do about it, while the toggles and sliders keep working, because they
 * are device state that applies the moment Home Assistant comes back.
 */
import { HINTS, TEXT } from "../copy.js";
import { HA_NEVER, entity, pathFor, post } from "../lib/device.js";
import { ManualIds, TargetTree } from "../tree.jsx";
import { Btn, Card, Missing, Row, Slider, Toggle } from "../ui.jsx";

/* ------------------------------------------------------------------ */
/* Reading and writing the shared entity shapes                        */
/* ------------------------------------------------------------------ */

function useEntity(ctx, key) {
  const e = entity(ctx, key);
  return {
    exists: !!ctx.device?.e?.[key],
    e,
    on: e ? e.value === true || e.state === "ON" || e.state === "on" : false,
    num: e ? Number(e.value ?? e.state) : 0,
    text: e ? String(e.value ?? e.state ?? "") : "",
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

function writeText(ctx, key, value) {
  const p = pathFor(ctx, key, "set", { value });
  if (p) post(p);
}

/**
 * The hint text tts_target_media_player carries when it is unset.
 *
 * It is a real state rather than a placeholder - the native API has no placeholder field, and Home
 * Assistant renders an empty one as "(empty value)" - so tts_routing.yaml seeds this string and every
 * reader treats it as empty. This has to match ${tts_targets_hint} exactly.
 */
const TARGETS_HINT = "Enter media_player IDs, comma separated";

function parseTargets(text) {
  if (!text || text === TARGETS_HINT) return [];
  return text
    .split(",")
    .map((s) => s.trim().toLowerCase())
    .filter(Boolean);
}

/* ------------------------------------------------------------------ */
/* Why the lists are not here                                          */
/* ------------------------------------------------------------------ */

/**
 * One of five states, never a generic failure. `rung` is what separates them: 0 means the device has
 * not asked yet, which on a fresh boot is a five-second wait rather than a problem, and -1 means both
 * rungs of the ladder were refused, which has a specific fix.
 */
function haProblem(ctx, ha) {
  if (!ha) return null;
  if (ha.rung === -1) return TEXT.ha_refused;
  if (ha.age === HA_NEVER) return ctx.device?.ha ? TEXT.ha_pending : TEXT.ha_never;
  if (!ha.d) return TEXT.ha_never;
  if (!ha.d.area) return TEXT.ha_no_area;
  if (!ha.d.areas || ha.d.areas.length === 0) return TEXT.ha_no_players;
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
 * Sonos reads the level off the announcement itself. A Satellite1 whose own Voice Override is above
 * zero ignores whatever we send and uses that instead. Everything else has its media volume set
 * before the announcement and restored afterwards. One slider with three behaviours behind it is the
 * kind of thing that gets reported as a bug, so it says so.
 *
 * Deliberately not a per-target table, which is what the plan drew. Which of the three applies
 * depends on Sonos membership and on the target's own override value, and neither is in the payload -
 * so a table would be a confident guess per row. Saying it once and accurately is better than saying
 * it per target and wrongly. Making it per-target means adding a flag per player to the Jinja.
 */
function Mechanisms({ targets }) {
  if (targets.length === 0) return null;
  return (
    <p class="ctl-sub">
      How this reaches each target depends on what it is. Sonos speakers take the level from the
      announcement itself. Another Satellite1 with its own Voice Override set above zero uses that
      instead and ignores this. Everything else has its media volume set before the answer plays and
      put back afterwards.
    </p>
  );
}

function RemoteRouting({ ctx, ha, refresh, refreshing }) {
  const routing = useEntity(ctx, "tts_routing");
  const muteLocal = useEntity(ctx, "tts_mute_local");
  const vol = useEntity(ctx, "remote_tts_volume");
  const chime = useEntity(ctx, "remote_wake_chime");
  const targetsText = useEntity(ctx, "tts_targets");

  if (!routing.exists) return null;

  const problem = haProblem(ctx, ha);
  const all = parseTargets(targetsText.text);
  // The tree only knows area-assigned players, so anything else in the list belongs to the manual
  // field. Splitting them here is what stops the tree from silently dropping ids it cannot render.
  const known = new Set();
  (ha?.d?.areas || []).forEach((a) => a.p.forEach(([id]) => known.add(id)));
  const inTree = new Set(all.filter((id) => known.has(id)));
  const manual = all.filter((id) => !known.has(id));

  const write = (treeSet, manualList) => {
    const next = [...treeSet, ...manualList];
    // Empty means unset, and unset is the hint string - the entity's own on_value re-seeds it, but
    // writing the hint keeps Home Assistant from showing "(empty value)" in the interim.
    writeText(ctx, "tts_targets", next.length ? next.join(",") : TARGETS_HINT);
  };

  return (
    <Card title="Remote routing">
      <Row label="Send responses elsewhere" hint={HINTS.tts_routing}>
        <Toggle checked={routing.on} onChange={(v) => writeSwitch(ctx, "tts_routing", v)} />
      </Row>

      {muteLocal.exists && (
        <Row label="Also speak here" hint={HINTS.tts_local_speaker}>
          {/* Inverted deliberately: the entity is "mute the local voice", and a customer reading a
              row called "Mute local voice" next to a list of places to send audio has to work out
              which way round it is. The device keeps the negative; the page does not. */}
          <Toggle
            checked={!muteLocal.on}
            disabled={!routing.on}
            onChange={(v) => writeSwitch(ctx, "tts_mute_local", !v)}
          />
        </Row>
      )}

      <div class="ctl-label">
        <span>Send to</span>
      </div>
      <HaState problem={problem} ha={ha} refresh={refresh} refreshing={refreshing} />
      {!problem && (
        <TargetTree
          areas={ha.d.areas}
          selected={inTree}
          disabled={!routing.on}
          onSelected={(s) => write(s, manual)}
        />
      )}

      <ManualIds
        value={manual.join(", ")}
        disabled={!routing.on}
        hint={HINTS.tts_manual_ids}
        onCommit={(v) => write(inTree, parseTargets(v))}
      />

      {vol.exists && (
        <Row label="Remote TTS volume" hint={HINTS.remote_tts_volume}>
          <Slider
            value={vol.num}
            min={vol.e?.min_value ?? 0}
            max={vol.e?.max_value ?? 100}
            step={vol.e?.step ?? 1}
            disabled={!routing.on}
            format={(v) => `${Math.round(v)}%`}
            onCommit={(v) => writeNumber(ctx, "remote_tts_volume", v)}
          />
        </Row>
      )}
      <Mechanisms targets={all} />

      {chime.exists && (
        <Row label="Remote wake chime" hint={HINTS.remote_wake_chime}>
          <Toggle checked={chime.on} disabled={!routing.on} onChange={(v) => writeSwitch(ctx, "remote_wake_chime", v)} />
        </Row>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */
/* Area ducking                                                        */
/* ------------------------------------------------------------------ */

/**
 * The exclusions duck_players_jinja already applies, shown rather than hidden.
 *
 * Only the two the payload can actually justify are listed. Music Assistant duplicates are known from
 * the walk having rejected them, and a routing target is known from the target list. The "Satellite1
 * Do Not Duck" label and the already-quieter-than-the-floor test are decided on Home Assistant's side
 * at call time and are not in the payload, so they are not claimed here - an exclusion shown without
 * a real reason is worse than one that is simply absent.
 */
function duckExclusions(targets) {
  const out = {};
  targets.forEach((id) => (out[id] = "a routing target"));
  return out;
}

function AreaDucking({ ctx, ha, refresh, refreshing }) {
  const duck = useEntity(ctx, "duck_area");
  const vol = useEntity(ctx, "duck_volume");
  const duckTargets = useEntity(ctx, "duck_tts_targets");
  const targetsText = useEntity(ctx, "tts_targets");

  if (!duck.exists) return null;

  const problem = haProblem(ctx, ha);
  // Ducking is scoped to this device's own area, so the tree is filtered to it rather than showing the
  // house. Routing can target any room; ducking cannot duck one it is not in.
  const mine = (ha?.d?.areas || []).filter((a) => a.n === ha?.d?.area);
  const excluded = duckTargets.on ? {} : duckExclusions(parseTargets(targetsText.text));

  return (
    <Card title="Area ducking">
      <Row label="Quieten this room while talking" hint={HINTS.duck_area}>
        <Toggle checked={duck.on} onChange={(v) => writeSwitch(ctx, "duck_area", v)} />
      </Row>

      {vol.exists && (
        <Row label="Duck volume" hint={HINTS.duck_volume}>
          <Slider
            value={vol.num}
            min={vol.e?.min_value ?? 0}
            max={vol.e?.max_value ?? 100}
            step={vol.e?.step ?? 1}
            disabled={!duck.on}
            format={(v) => `${Math.round(v)}%`}
            onCommit={(v) => writeNumber(ctx, "duck_volume", v)}
          />
        </Row>
      )}

      {duckTargets.exists && (
        <Row label="Duck routing targets too" hint={HINTS.duck_tts_targets}>
          <Toggle checked={duckTargets.on} disabled={!duck.on} onChange={(v) => writeSwitch(ctx, "duck_tts_targets", v)} />
        </Row>
      )}

      <div class="ctl-label">
        <span>{ha?.d?.area ? `In ${ha.d.area}` : "In this device's area"}</span>
      </div>
      <HaState problem={problem} ha={ha} refresh={refresh} refreshing={refreshing} />

      {/* Read-only for now. Ducking resolves its own membership at call time from the whole area, so
          there is nothing on the device to write a narrower selection to until duck_players_list
          exists - and a tree that looked writable but was not would be a lie. */}
      {!problem && mine.length > 0 && (
        <>
          <TargetTree areas={mine} selected={new Set()} onSelected={() => {}} excluded={excluded} disabled />
          <p class="ctl-sub">
            Everything in {ha.d.area} is ducked, including speakers added later. Choosing individual
            players is not available yet.
          </p>
        </>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ */

export function Config({ ctx }) {
  const { ha, haRefresh, haRefreshing } = ctx;

  if (!ctx.device) return <Missing what="the device" />;

  return (
    <>
      <RemoteRouting ctx={ctx} ha={ha} refresh={haRefresh} refreshing={haRefreshing} />
      <AreaDucking ctx={ctx} ha={ha} refresh={haRefresh} refreshing={haRefreshing} />
    </>
  );
}
