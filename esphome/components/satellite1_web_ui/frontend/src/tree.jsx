/**
 * The area/player tree, used twice: once for Remote routing and once for Area ducking.
 *
 * One component rather than two because the two lists must never disagree about what is in an area -
 * they are rendered from the same `areas` payload, which Home Assistant built from the same walk
 * ducking itself uses. The difference between the two uses is entirely in the props: ducking passes
 * exclusions, routing does not.
 *
 * Selection is by entity id and crosses areas freely, because routing to another room is a normal
 * thing to want. An area's own checkbox is a bulk operation on its children, not a value of its own -
 * except in `whole` mode, where an area can be selected as an area and picks up speakers added to it
 * later. That distinction is the whole reason Area ducking needs it and routing does not.
 */
import { useState } from "preact/hooks";

import { Hint } from "./ui.jsx";

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

/**
 * @param areas      [{n, p: [[entity_id, friendly_name], ...]}] as served in /api/sat1/ha
 * @param selected   Set of selected entity ids
 * @param onSelected receives the next Set
 * @param excluded   optional entity_id -> reason. Rendered as a non-selectable row carrying why.
 * @param whole      optional Set of area names selected wholesale, plus onWhole. Enables `whole` mode.
 */
export function TargetTree({ areas, selected, onSelected, excluded, whole, onWhole, disabled }) {
  const [open, setOpen] = useState({});

  if (!areas || areas.length === 0) return null;

  const toggle = (id) => {
    const next = new Set(selected);
    if (next.has(id)) {
      next.delete(id);
    } else {
      next.add(id);
    }
    onSelected(next);
  };

  return (
    <div class="tree">
      {areas.map((area) => {
        // Excluded players are listed but never counted: an area showing 2/3 when the third can
        // never be ticked reads as a bug in the page rather than a property of the speaker.
        const rows = area.p;
        const pickable = rows.filter(([id]) => !excluded || !excluded[id]);
        const on = pickable.filter(([id]) => selected.has(id));
        const isWhole = whole ? whole.has(area.n) : false;
        const state = isWhole || (pickable.length > 0 && on.length === pickable.length)
          ? "on"
          : on.length > 0
            ? "mixed"
            : "off";
        const expanded = open[area.n];

        const clickArea = () => {
          if (whole) {
            // In whole mode the area checkbox owns the area itself, so ticking it clears the
            // individual picks underneath - they would be redundant and would survive a speaker
            // being removed from the area, which is exactly what whole mode exists to avoid.
            const nextWhole = new Set(whole);
            const next = new Set(selected);
            if (isWhole) {
              nextWhole.delete(area.n);
            } else {
              nextWhole.add(area.n);
              pickable.forEach(([id]) => next.delete(id));
            }
            onWhole(nextWhole);
            onSelected(next);
            return;
          }
          const next = new Set(selected);
          if (state === "on") {
            pickable.forEach(([id]) => next.delete(id));
          } else {
            pickable.forEach(([id]) => next.add(id));
          }
          onSelected(next);
        };

        return (
          <div class="tree-a" key={area.n}>
            <div class="tree-h">
              <button
                class="caret tree-x"
                aria-label={expanded ? "Collapse" : "Expand"}
                aria-expanded={expanded ? "true" : "false"}
                onClick={() => setOpen({ ...open, [area.n]: !expanded })}
              >
                {expanded ? "\u25BE" : "\u25B8"}
              </button>
              <Check state={state} disabled={disabled} onClick={clickArea} label={area.n} />
              <span class="grow">{area.n}</span>
              <span class="tree-n">
                {isWhole ? "whole area" : `${on.length}/${pickable.length}`}
              </span>
            </div>

            {expanded && (
              <div class="tree-ps">
                {rows.map(([id, name]) => {
                  const why = excluded && excluded[id];
                  if (why) {
                    return (
                      <div class="tree-p tree-off" key={id}>
                        <span class="cb dim">{"\u2298"}</span>
                        <span class="grow">
                          {name}
                          <span class="tree-why">{why}</span>
                        </span>
                      </div>
                    );
                  }
                  return (
                    <div class="tree-p" key={id}>
                      <Check
                        state={isWhole || selected.has(id) ? "on" : "off"}
                        disabled={disabled || isWhole}
                        onClick={() => toggle(id)}
                        label={name}
                      />
                      <span class="grow">{name}</span>
                    </div>
                  );
                })}
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}

/**
 * The manual entity-id field, which is not a fallback so much as the only way to reach two thirds of
 * a real installation's media players.
 *
 * Measured on a 104-player house: 52 were Music Assistant duplicates the walk rejects on purpose, and
 * 40 more had no area at all - Cast and AirPlay shadow entities, universal remotes, `all_sonos`-style
 * group helpers, laptops. An area-keyed tree cannot show any of those, so the text field stays.
 */
export function ManualIds({ value, onCommit, disabled, hint }) {
  const [draft, setDraft] = useState(null);
  const shown = draft === null ? value : draft;

  return (
    <div class="ids">
      <label class="ctl-label">
        Other entity ids
        {hint && <Hint text={hint} />}
      </label>
      <input
        type="text"
        class="in"
        value={shown}
        disabled={disabled}
        placeholder="media_player.all_sonos, media_player.shield"
        onInput={(e) => setDraft(e.currentTarget.value)}
        onBlur={() => {
          if (draft !== null && draft !== value) onCommit(draft);
          setDraft(null);
        }}
        onKeyDown={(e) => {
          if (e.key === "Enter") e.currentTarget.blur();
        }}
      />
    </div>
  );
}
