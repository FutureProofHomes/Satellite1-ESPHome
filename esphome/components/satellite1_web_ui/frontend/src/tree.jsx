/**
 * The area/player tree, used twice: once for remote routing and once for area ducking.
 *
 * One component rather than two because the two lists must never disagree about what is in an area -
 * they are rendered from the same `areas` payload, which Home Assistant built from the same walk
 * ducking itself uses. The difference between the two uses is now only whether a "Local Speaker" row
 * is offered, which is a prop.
 *
 * The selection it edits is the same three fields the device stores, and it edits them directly rather
 * than flattening to a list of ids and expanding again on save:
 *
 *   areas     area ids taken wholesale, so a speaker added to that room later is included
 *   extra     individually chosen entity ids, including players in no area at all
 *   excluded  `areaId:entityId`, one player carved back out of a wholesale area
 *
 * Editing the stored shape directly is what keeps the whole thing small enough to store. Unticking one
 * speaker in a twelve-player area adds one exclusion rather than replacing the area with eleven ids -
 * about forty characters against about four hundred and sixty, and the old text entity held 255.
 *
 * The area prefix on an exclusion is not needed to resolve the selection, since an entity belongs to
 * at most one area. It is there so the firmware can answer "is my own area selected whole" without
 * knowing area membership, which is what the two derived switches read.
 */
import { useState } from "preact/hooks";
import { Chevron } from "./ui.jsx";
import { TEXT } from "./copy.js";

/**
 * A tri-state box, still a button rather than an input: an indeterminate checkbox needs a ref to set the
 * property, and there is no attribute for it.
 *
 * Drawn rather than typed, for the reason ui.jsx's Chevron is. This used to be U+2610, U+2611 and U+25EA,
 * and with no webfont on the device those came from whatever the browser had - U+25EA in particular, the
 * half-filled square that carried the whole "some of this area is selected" meaning, is obscure enough to
 * be missing from most system fonts and arrives as an empty rectangle. A tri-state control whose third
 * state renders as a blank box is worse than no third state.
 *
 * Empty outline, filled with a tick, filled with a dash: the shapes people already read as off, all and
 * some. The mark is coloured from CSS rather than a fill attribute so it follows the theme.
 */
function Check({ state, disabled, onClick, label }) {
  const on = state === "on";
  const mixed = state === "mixed";
  return (
    <button
      class={`cb ${state}${disabled ? " dim" : ""}`}
      role="checkbox"
      aria-checked={on ? "true" : mixed ? "mixed" : "false"}
      aria-label={label}
      disabled={disabled}
      onClick={onClick}
    >
      <svg class="cb-i" viewBox="0 0 16 16" aria-hidden="true">
        <rect class="cb-box" x="1.6" y="1.6" width="12.8" height="12.8" rx="3.4" />
        {on && <path class="cb-mark" d="M4.5 8.2 6.9 10.6l4.6-5.2" />}
        {mixed && <path class="cb-mark" d="M4.8 8h6.4" />}
      </svg>
    </button>
  );
}

/** A row of players under a heading, with its own bulk checkbox. Used for a real area and for the
 *  "No Area Assigned" group, which behave differently enough to be worth one shared shell and two
 *  callers rather than one component with a mode flag. */
function Group({ label, count, state, expanded, onExpand, onBulk, disabled, children }) {
  return (
    <div class="tree-a">
      <div class="tree-h">
        <button
          class="caret tree-x"
          aria-label={expanded ? "Collapse" : "Expand"}
          aria-expanded={expanded ? "true" : "false"}
          onClick={onExpand}
        >
          <Chevron down={expanded} />
        </button>
        <Check state={state} disabled={disabled} onClick={onBulk} label={label} />
        <span class="grow">{label}</span>
        <span class="tree-n">{count}</span>
      </div>
      {expanded && <div class="tree-ps">{children}</div>}
    </div>
  );
}

/**
 * @param payload  the `/api/sat1/ha` body:
 *                 {areas: [{i, n, p: [[id, name, caps, avail], ...]}], loose: [[id, name, caps, avail]]}
 * @param sel      {areas, extra, excluded} as Sets
 * @param onSel    receives the next {areas, extra, excluded}
 * @param local    when not null, a "Local Speaker" row is shown first, holding this boolean
 * @param onLocal  receives the next boolean
 * @param need     which capability bit this tree's call requires of a player - 1 play_media
 *                 (routing), 2 volume_set (ducking). Rows without it render greyed and uncheckable
 *                 with a one-line reason, rather than being omitted: a player silently missing from
 *                 the list reads as "the app is broken", a greyed row explains itself.
 */
export function TargetTree({ payload, sel, onSel, local, onLocal, disabled, need = 3 }) {
  const [open, setOpen] = useState({});

  const areas = (payload && payload.areas) || [];
  const loose = (payload && payload.loose) || [];

  /** Whether a player can answer the call this tree configures. `?? 3` keeps rows from a payload the
   *  device cached before the caps field existed live in both trees rather than greying everything. */
  const capOk = (row) => ((row[2] ?? 3) & need) !== 0;
  const reason = need === 2 ? TEXT.cap_no_volume : TEXT.cap_no_media;

  /** Caps bit 4 marks this device's own media player. Shown greyed as "This device" rather than
   *  omitted - routing to yourself is the Local Speaker row's job, and ducking your own volume while
   *  you talk is never right, but a hole where a row should be reads as a bug. */
  const isSelf = (row) => ((row[2] ?? 0) & 4) !== 0;

  /** Whether a row can be added to the selection at all: capable, and not this device itself. */
  const elig = (row) => capOk(row) && !isSelf(row);

  /** Whether Home Assistant could reach the player when the payload was rendered. `?? 1` keeps rows
   *  from a payload cached before the availability field existed reading as online, which is the
   *  behaviour those payloads always had. Offline changes styling and adds a label, nothing else:
   *  unlike a missing capability it is transient, so the checkbox keeps working and the stored
   *  selection is untouched - the call-time walks (tts_routing.yaml) are what skip the player until
   *  it comes back. */
  const isLive = (row) => (row[3] ?? 1) !== 0;

  const edit = (fn) => {
    const next = {
      areas: new Set(sel.areas),
      extra: new Set(sel.extra),
      excluded: new Set(sel.excluded),
    };
    fn(next);
    onSel(next);
  };

  /** Whether a player counts as chosen: covered by its whole area and not carved out of it, or picked
   *  individually. Both routes are live at once, which is what lets an area be taken wholesale while a
   *  player in a different room is added on its own. */
  const isOn = (areaId, id) =>
    (areaId !== null && sel.areas.has(areaId) && !sel.excluded.has(`${areaId}:${id}`)) ||
    sel.extra.has(id);

  const clickPlayer = (areaId, id) => {
    edit((next) => {
      if (areaId !== null && next.areas.has(areaId)) {
        // Inside a wholesale area, unticking means carving out rather than rewriting the area as a
        // list of the players that are left.
        const key = `${areaId}:${id}`;
        if (next.excluded.has(key)) {
          next.excluded.delete(key);
        } else {
          next.excluded.add(key);
          next.extra.delete(id);
        }
        return;
      }
      if (next.extra.has(id)) {
        next.extra.delete(id);
      } else {
        next.extra.add(id);
      }
    });
  };

  // The area-level maths run over eligible rows only. With ineligible rows counted, an area whose
  // eligible players are all ticked would read "mixed" forever and "select whole area" would look
  // broken - the greyed rows are visible but they are not part of what the box is deciding.
  const areaState = (area) => {
    const ids = area.p.filter(elig).map(([id]) => id);
    if (sel.areas.has(area.i)) {
      // Whole, unless something has been carved out of it - which is exactly what the two derived
      // switches in the firmware report, so the box here and the switch in Home Assistant agree.
      return ids.some((id) => sel.excluded.has(`${area.i}:${id}`)) ? "mixed" : "on";
    }
    const on = ids.filter((id) => sel.extra.has(id)).length;
    return on === 0 ? "off" : on === ids.length ? "on" : "mixed";
  };

  const clickArea = (area) => {
    const state = areaState(area);
    edit((next) => {
      next.excluded.forEach((k) => {
        if (k.startsWith(`${area.i}:`)) next.excluded.delete(k);
      });
      area.p.forEach(([id]) => next.extra.delete(id));
      if (state === "on") {
        next.areas.delete(area.i);
      } else {
        // Both from off and from mixed: the second click on a partly-ticked area takes the whole
        // thing, which is the behaviour a tri-state box trains people to expect.
        next.areas.add(area.i);
      }
    });
  };

  const looseState = () => {
    const rows = loose.filter(elig);
    const on = rows.filter(([id]) => sel.extra.has(id)).length;
    return on === 0 ? "off" : on === rows.length ? "on" : "mixed";
  };

  const clickLoose = () => {
    const state = looseState();
    edit((next) => {
      // Bulk-ticking adds eligible players only; bulk-unticking is likewise scoped, so a stale
      // ineligible extra is removed by its own row (always allowed) rather than as a side effect.
      loose
        .filter(elig)
        .forEach(([id]) => (state === "on" ? next.extra.delete(id) : next.extra.add(id)));
    });
  };

  const areaCount = (area) => {
    const rows = area.p.filter(elig);
    if (sel.areas.has(area.i)) {
      const cut = rows.filter(([id]) => sel.excluded.has(`${area.i}:${id}`)).length;
      return cut === 0 ? "whole area" : `${rows.length - cut}/${rows.length}`;
    }
    return `${rows.filter(([id]) => sel.extra.has(id)).length}/${rows.length}`;
  };

  return (
    <div class="tree">
      {local !== null && local !== undefined && (
        // Always first, and ticked by default. It is in the same list as everywhere else the answer
        // could go because that is the honest way to present it: one choice about which speakers
        // speak, not a routing list plus a separate switch that silences this one.
        <div class="tree-p tree-self">
          <Check
            state={local ? "on" : "off"}
            disabled={disabled}
            onClick={() => onLocal(!local)}
            label="Local Speaker"
          />
          <span class="grow">Local Speaker</span>
        </div>
      )}

      {areas.map((area) => {
        const state = areaState(area);
        return (
          <Group
            key={area.i}
            label={area.n}
            count={areaCount(area)}
            state={state}
            expanded={open[area.i]}
            onExpand={() => setOpen({ ...open, [area.i]: !open[area.i] })}
            onBulk={() => clickArea(area)}
            // An area with nothing eligible in it has nothing for the bulk box to add - unless a
            // stale selection still covers it, in which case unticking must stay possible.
            disabled={disabled || (state === "off" && !area.p.some(elig))}
          >
            {area.p.map((row) => {
              const [id, name] = row;
              const capable = capOk(row);
              const self = isSelf(row);
              const eligible = capable && !self;
              const live = isLive(row);
              // A permanently ineligible row never reads as selected and is never operable, even
              // when an old saved selection explicitly names it. The call-time walks skip it, so a
              // tick would be a false promise. Offline is different: it is transient, so a capable
              // remote player's saved tick remains visible and editable until the player returns.
              const on = eligible && isOn(area.i, id);
              // Permanent facts outrank the transient one: "This device" and the capability reason
              // never change, while offline fixes itself. A row that is both incompatible and
              // offline therefore says why it will remain grey after the player comes back.
              return (
                <div class={`tree-p${eligible && live ? "" : " tree-off"}`} key={id}>
                  <Check
                    state={on ? "on" : "off"}
                    disabled={disabled || !eligible}
                    onClick={() => clickPlayer(area.i, id)}
                    label={name}
                  />
                  <span class="grow">{name}</span>
                  {(!eligible || !live) && (
                    <span class="tree-why">
                      {self ? TEXT.cap_self : !capable ? reason : TEXT.player_offline}
                    </span>
                  )}
                </div>
              );
            })}
          </Group>
        );
      })}

      {loose.length > 0 && (
        // Not an edge case, and not a fallback: on the test installation 40 of the 104 media players
        // are in no area at all, against 37 that are in one. Cast and AirPlay shadow entities, group
        // helpers like all_sonos, laptops. This group is what replaced a free-text entity id field,
        // which asked someone to know an id the page could simply have shown them.
        <Group
          label="No Area Assigned"
          count={`${loose.filter(elig).filter(([id]) => sel.extra.has(id)).length}/${loose.filter(elig).length}`}
          state={looseState()}
          expanded={open.__loose}
          onExpand={() => setOpen({ ...open, __loose: !open.__loose })}
          onBulk={clickLoose}
          disabled={disabled || (looseState() === "off" && !loose.some(elig))}
        >
          {loose.map((row) => {
            const [id, name] = row;
            const capable = capOk(row);
            const self = isSelf(row);
            const eligible = capable && !self;
            const on = eligible && sel.extra.has(id);
            const live = isLive(row);
            return (
              <div class={`tree-p${eligible && live ? "" : " tree-off"}`} key={id}>
                <Check
                  state={on ? "on" : "off"}
                  disabled={disabled || !eligible}
                  onClick={() => clickPlayer(null, id)}
                  label={name}
                />
                <span class="grow">{name}</span>
                {(!eligible || !live) && (
                  <span class="tree-why">
                    {self ? TEXT.cap_self : !capable ? reason : TEXT.player_offline}
                  </span>
                )}
              </div>
            );
          })}
        </Group>
      )}
    </div>
  );
}
