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

/** ☐ ☑ ◪ as a control rather than an input: tri-state needs a ref on a real checkbox, and three
 *  glyphs cost nothing next to the CSS a custom box would. */
function Check({ state, disabled, onClick, label }) {
  const glyph = state === "on" ? "\u2611" : state === "mixed" ? "\u25EA" : "\u2610";
  return (
    <button
      class={`cb${disabled ? " dim" : ""}`}
      role="checkbox"
      aria-checked={state === "on" ? "true" : state === "mixed" ? "mixed" : "false"}
      aria-label={label}
      disabled={disabled}
      onClick={onClick}
    >
      {glyph}
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
          {expanded ? "\u25BE" : "\u25B8"}
        </button>
        <Check state={state} disabled={disabled} onClick={onBulk} label={label} />
        <span class="grow">{label}</span>
        <span class="tree-n">{count}</span>
      </div>
      {expanded && <div class="tree-ps">{children}</div>}
    </div>
  );
}

/** Empty sets rather than undefined, so a caller that has not loaded a selection yet renders an empty
 *  tree instead of throwing. */
export function emptySel() {
  return { areas: new Set(), extra: new Set(), excluded: new Set() };
}

/**
 * @param payload  the `/api/sat1/ha` body: {areas: [{i, n, p: [[id, name], ...]}], loose: [[id, name]]}
 * @param sel      {areas, extra, excluded} as Sets
 * @param onSel    receives the next {areas, extra, excluded}
 * @param local    when not null, a "Local Speaker" row is shown first, holding this boolean
 * @param onLocal  receives the next boolean
 */
export function TargetTree({ payload, sel, onSel, local, onLocal, disabled }) {
  const [open, setOpen] = useState({});

  const areas = (payload && payload.areas) || [];
  const loose = (payload && payload.loose) || [];

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

  const areaState = (area) => {
    const ids = area.p.map(([id]) => id);
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
    const on = loose.filter(([id]) => sel.extra.has(id)).length;
    return on === 0 ? "off" : on === loose.length ? "on" : "mixed";
  };

  const clickLoose = () => {
    const state = looseState();
    edit((next) => {
      loose.forEach(([id]) => (state === "on" ? next.extra.delete(id) : next.extra.add(id)));
    });
  };

  const areaCount = (area) => {
    if (sel.areas.has(area.i)) {
      const cut = area.p.filter(([id]) => sel.excluded.has(`${area.i}:${id}`)).length;
      return cut === 0 ? "whole area" : `${area.p.length - cut}/${area.p.length}`;
    }
    return `${area.p.filter(([id]) => sel.extra.has(id)).length}/${area.p.length}`;
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

      {areas.map((area) => (
        <Group
          key={area.i}
          label={area.n}
          count={areaCount(area)}
          state={areaState(area)}
          expanded={open[area.i]}
          onExpand={() => setOpen({ ...open, [area.i]: !open[area.i] })}
          onBulk={() => clickArea(area)}
          disabled={disabled}
        >
          {area.p.map(([id, name]) => (
            <div class="tree-p" key={id}>
              <Check
                state={isOn(area.i, id) ? "on" : "off"}
                disabled={disabled}
                onClick={() => clickPlayer(area.i, id)}
                label={name}
              />
              <span class="grow">{name}</span>
            </div>
          ))}
        </Group>
      ))}

      {loose.length > 0 && (
        // Not an edge case, and not a fallback: on the test installation 40 of the 104 media players
        // are in no area at all, against 37 that are in one. Cast and AirPlay shadow entities, group
        // helpers like all_sonos, laptops. This group is what replaced a free-text entity id field,
        // which asked someone to know an id the page could simply have shown them.
        <Group
          label="No Area Assigned"
          count={`${loose.filter(([id]) => sel.extra.has(id)).length}/${loose.length}`}
          state={looseState()}
          expanded={open.__loose}
          onExpand={() => setOpen({ ...open, __loose: !open.__loose })}
          onBulk={clickLoose}
          disabled={disabled}
        >
          {loose.map(([id, name]) => (
            <div class="tree-p" key={id}>
              <Check
                state={sel.extra.has(id) ? "on" : "off"}
                disabled={disabled}
                onClick={() => clickPlayer(null, id)}
                label={name}
              />
              <span class="grow">{name}</span>
            </div>
          ))}
        </Group>
      )}
    </div>
  );
}
