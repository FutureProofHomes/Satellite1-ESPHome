/**
 * The Audio route: where this device's sound goes - Audio routing and Area ducking,
 * each with the shared area/player tree. Ported from routes/config.jsx + tree.jsx.
 */
import React, { useState } from 'react';
import { Card, Check, Chevron, Row, Select, Slider, Toggle } from './ui';
import { HA_AREAS, HA_LOOSE } from './mock';
const HINTS = {
  remote_routing: "Send the assistant's answers, sign-in prompts, timer rings and the wake chime to other speakers instead of (or as well as) this one.",
  remote_tts_volume: 'The level remote players answer at. Zero leaves every target volume alone.',
  remote_wake_chime: 'Play the wake chime on the selected players too, so a far room hears that the device is listening.',
  remote_timer_ring: 'Ring finished timers on the selected players as well.',
  remote_sync_guard: 'What happens to music the routing interrupted on the remote players once the answer ends.',
  area_ducking: 'Lower other speakers while you talk to this device, from the wake word until the answer ends.',
  duck_volume: 'The level ducked players drop to. Zero silences them for the length of the interaction.'
};
type Sel = {
  areas: Set<string>;
  extra: Set<string>;
  excluded: Set<string>;
};

/* ------------------------------------------------------------------ */
/* The area/player tree                                                */
/* ------------------------------------------------------------------ */

function Group({
  label,
  count,
  state,
  expanded,
  onExpand,
  onBulk,
  disabled,
  children
}: {
  label: string;
  count: string;
  state: 'on' | 'off' | 'mixed';
  expanded: boolean;
  onExpand: () => void;
  onBulk: () => void;
  disabled?: boolean;
  children?: React.ReactNode;
}) {
  return <div className="tree-a">
      <div className="tree-h">
        <button className="caret tree-x" aria-label={expanded ? 'Collapse' : 'Expand'} aria-expanded={expanded} onClick={onExpand}>
          <Chevron down={expanded} />
        </button>
        <Check state={state} disabled={disabled} onClick={onBulk} label={label} />
        <span className="grow">{label}</span>
        <span className="tree-n">{count}</span>
      </div>
      {expanded && <div className="tree-ps">{children}</div>}
    </div>;
}
function TargetTree({
  sel,
  onSel,
  local,
  onLocal,
  need
}: {
  sel: Sel;
  onSel: (next: Sel) => void;
  local: boolean | null;
  onLocal?: (v: boolean) => void;
  need: number;
}) {
  const [open, setOpen] = useState<Record<string, boolean>>({
    living_room: true
  });
  const capOk = (row: [string, string, number, number]) => ((row[2] ?? 3) & need) !== 0;
  const isSelf = (row: [string, string, number, number]) => ((row[2] ?? 0) & 4) !== 0;
  const elig = (row: [string, string, number, number]) => capOk(row) && !isSelf(row);
  const isLive = (row: [string, string, number, number]) => (row[3] ?? 1) !== 0;
  const reason = need === 2 ? 'no volume control' : 'cannot play media';
  const edit = (fn: (next: Sel) => void) => {
    const next: Sel = {
      areas: new Set(sel.areas),
      extra: new Set(sel.extra),
      excluded: new Set(sel.excluded)
    };
    fn(next);
    onSel(next);
  };
  const isOn = (areaId: string | null, id: string) => areaId !== null && sel.areas.has(areaId) && !sel.excluded.has(`${areaId}:${id}`) || sel.extra.has(id);
  const clickPlayer = (areaId: string | null, id: string) => {
    edit(next => {
      if (areaId !== null && next.areas.has(areaId)) {
        const key = `${areaId}:${id}`;
        if (next.excluded.has(key)) next.excluded.delete(key);else {
          next.excluded.add(key);
          next.extra.delete(id);
        }
        return;
      }
      if (next.extra.has(id)) next.extra.delete(id);else next.extra.add(id);
    });
  };
  const areaState = (area: (typeof HA_AREAS)[0]): 'on' | 'off' | 'mixed' => {
    const ids = area.p.filter(elig).map(([id]) => id);
    if (sel.areas.has(area.i)) return ids.some(id => sel.excluded.has(`${area.i}:${id}`)) ? 'mixed' : 'on';
    const on = ids.filter(id => sel.extra.has(id)).length;
    return on === 0 ? 'off' : on === ids.length ? 'on' : 'mixed';
  };
  const clickArea = (area: (typeof HA_AREAS)[0]) => {
    const state = areaState(area);
    edit(next => {
      next.excluded.forEach(k => {
        if (k.startsWith(`${area.i}:`)) next.excluded.delete(k);
      });
      area.p.forEach(([id]) => next.extra.delete(id));
      if (state === 'on') next.areas.delete(area.i);else next.areas.add(area.i);
    });
  };
  const looseState = (): 'on' | 'off' | 'mixed' => {
    const rows = HA_LOOSE.filter(elig);
    const on = rows.filter(([id]) => sel.extra.has(id)).length;
    return on === 0 ? 'off' : on === rows.length ? 'on' : 'mixed';
  };
  const areaCount = (area: (typeof HA_AREAS)[0]) => {
    const rows = area.p.filter(elig);
    if (sel.areas.has(area.i)) {
      const cut = rows.filter(([id]) => sel.excluded.has(`${area.i}:${id}`)).length;
      return cut === 0 ? 'whole area' : `${rows.length - cut}/${rows.length}`;
    }
    return `${rows.filter(([id]) => sel.extra.has(id)).length}/${rows.length}`;
  };
  const playerRow = (areaId: string | null) => (row: [string, string, number, number]) => {
    const [id, name] = row;
    const capable = capOk(row);
    const self = isSelf(row);
    const eligible = capable && !self;
    const live = isLive(row);
    const on = eligible && isOn(areaId, id);
    return <div className={`tree-p${eligible && live ? '' : ' tree-off'}`} key={id}>
        <Check state={on ? 'on' : 'off'} disabled={!eligible} onClick={() => clickPlayer(areaId, id)} label={name} />
        <span className="grow">{name}</span>
        {(!eligible || !live) && <span className="tree-why">{self ? 'This device' : !capable ? reason : 'offline'}</span>}
      </div>;
  };
  return <div className="tree">
      {local !== null && local !== undefined && <div className="tree-p tree-self">
          <Check state={local ? 'on' : 'off'} onClick={() => onLocal && onLocal(!local)} label="Local Speaker" />
          <span className="grow">Local Speaker</span>
        </div>}

      {HA_AREAS.map(area => <Group key={area.i} label={area.n} count={areaCount(area)} state={areaState(area)} expanded={!!open[area.i]} onExpand={() => setOpen({
      ...open,
      [area.i]: !open[area.i]
    })} onBulk={() => clickArea(area)}>
          {area.p.map(playerRow(area.i))}
        </Group>)}

      <Group label="No Area Assigned" count={`${HA_LOOSE.filter(elig).filter(([id]) => sel.extra.has(id)).length}/${HA_LOOSE.filter(elig).length}`} state={looseState()} expanded={!!open.__loose} onExpand={() => setOpen({
      ...open,
      __loose: !open.__loose
    })} onBulk={() => edit(next => {
      const state = looseState();
      HA_LOOSE.filter(elig).forEach(([id]) => state === 'on' ? next.extra.delete(id) : next.extra.add(id));
    })}>
        {HA_LOOSE.map(playerRow(null))}
      </Group>
    </div>;
}

/* ------------------------------------------------------------------ */
/* The two cards                                                       */
/* ------------------------------------------------------------------ */

function RemoteRouting() {
  const [sel, setSel] = useState<Sel>({
    areas: new Set(['living_room']),
    extra: new Set(),
    excluded: new Set()
  });
  const [local, setLocal] = useState(true);
  const [vol, setVol] = useState(0);
  const [chime, setChime] = useState(true);
  const [timer, setTimer] = useState(true);
  const [guard, setGuard] = useState('0.2s');
  const active = sel.areas.size > 0 || sel.extra.size > 0;
  return <Card title="Audio routing" hint={HINTS.remote_routing}>
      <p className="tree-title">Play assistant audio on selected players</p>
      <TargetTree sel={sel} onSel={setSel} local={local} onLocal={setLocal} need={1} />

      <Row label="Remote TTS volume" hint={HINTS.remote_tts_volume}>
        <Slider value={vol} min={0} max={100} step={1} disabled={!active} format={v => v === 0 ? 'follow device' : `${Math.round(v)}%`} onCommit={setVol} />
      </Row>

      <Row label="Remote wake chime" hint={HINTS.remote_wake_chime}>
        <Toggle checked={chime} disabled={!active} onChange={setChime} />
      </Row>

      <Row label="Remote timer ring" hint={HINTS.remote_timer_ring}>
        <Toggle checked={timer} disabled={!active} onChange={setTimer} />
      </Row>

      <Row label="Remote sync guard" hint={HINTS.remote_sync_guard}>
        <Select value={guard} options={['0.2s', '0.4s', '0.6s', '0.8s', '1s', '1.2s', '1.4s', '1.6s', '1.8s']} disabled={!active} onChange={setGuard} />
      </Row>
    </Card>;
}
function AreaDucking() {
  const [sel, setSel] = useState<Sel>({
    areas: new Set(),
    extra: new Set(['media_player.kitchen_display']),
    excluded: new Set()
  });
  const [vol, setVol] = useState(20);
  const active = sel.areas.size > 0 || sel.extra.size > 0;
  return <Card title="Area ducking" hint={HINTS.area_ducking}>
      <p className="tree-title">Lower the volume on selected players upon wake word detection</p>
      <TargetTree sel={sel} onSel={setSel} local={null} need={2} />

      <Row label="Duck volume" hint={HINTS.duck_volume}>
        <Slider value={vol} min={0} max={100} step={1} disabled={!active} format={v => v === 0 ? 'mute' : `${Math.round(v)}%`} onCommit={setVol} />
      </Row>
    </Card>;
}
export function AudioRoute() {
  return <>
      <RemoteRouting />
      <AreaDucking />
    </>;
}