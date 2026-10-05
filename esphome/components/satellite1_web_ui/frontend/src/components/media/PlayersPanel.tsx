import { TEXT } from '../../copy.js';
import { Drawer } from '../Drawer';
import { Elsewhere } from './Elsewhere';
import type { Model, Other, Tiers } from './model';
import { I_MINUS, I_PLUS, I_SEARCH, I_VOL, Svg, Vol } from './parts';

/**
 * The group behind the bar's speaker button: each member with its volume and a remove, then the
 * speakers that could join, then what Music Assistant is playing elsewhere, from whichever tier
 * answers (useTiers). With no tier at all - Home Assistant delivered no Music Assistant player for
 * this device (Home Assistant absent, its actions off, or no Music Assistant) and no direct
 * connection is set up - it says so once the payload has arrived, and says nothing while it is
 * still loading.
 *
 * The whole-group slider shows only while there is a group (owner's request, September 2026): with
 * one speaker it duplicated that speaker's own row and the bar's slider both, and "Volume" over a
 * list of speakers read as nobody's in particular. A tapped add row rings and locks until the join
 * is confirmed, per row, so two quick adds spin independently. An edit that is never confirmed
 * leaves a note at the top saying so, rather than a ring that spins and vanishes.
 *
 * `row` opens it on another speaker's group instead, from that speaker's card on the bar (owner's
 * request, October 2026): the same members, volumes, removes and adds, aimed at that group's
 * leader. "Playing elsewhere" is left out there - its Join and Take over are this device's.
 */
export function PlayersPanel({
  model,
  tiers,
  haReady,
  row,
  onClose,
  onSearch
}: {
  model: Model;
  tiers: Tiers;
  haReady: boolean;
  row?: Other | null;
  onClose: () => void;
  onSearch: () => void;
}) {
  const { media, mediaCmd, srcParam, groupHeld } = model;
  const { wsOn, me, live, optVol, pendingGroup, gVol, gUnjoin, gJoin, groupNote, rVolume } = tiers;
  const group = row ? tiers.groupOf(row) : null;
  const members = group ? group.members : tiers.members;
  const addables = group ? group.addables : tiers.addables;
  const lead = row ? row.id : undefined;
  const any = !!row || wsOn || !!me;
  const label = row ? `${TEXT.media_players_title} \u00b7 ${row.name}` : TEXT.media_players_title;
  const groupVol = members.length < 2 ? null
    : row ? row.vctl && <Vol value={optVol[row.id] ?? row.volume} label={TEXT.media_group_volume} onCommit={v => rVolume(row, v)} />
    : media?.volume != null && <Vol value={groupHeld ? media.ss_volume : media.volume} label={TEXT.media_group_volume} onCommit={v => mediaCmd('volume', { v, src: srcParam })} />;

  return <Drawer label={label} onClose={onClose} className="mpanel">
    <button className="mbtn msheet-search" aria-label={TEXT.search_open} onClick={onSearch}><Svg size={18}>{I_SEARCH}</Svg></button>
    <span className="eyebrow">{label.toUpperCase()}</span>
    {groupNote && <p className="mnote warn" role="status">{groupNote}</p>}
    {groupVol && <div className="mvol-row group"><span>{TEXT.media_group_volume}</span>{groupVol}</div>}
    {!any && haReady && <p className="mnote">{TEXT.media_no_tiers}</p>}
    {any && <div className="mpanel-scroll">
      {!row && !wsOn && !live && <p className="mnote">{TEXT.media_group_loading}</p>}
      <ul className="mlist">{members.map(([id, name, vol]) => <li key={id}><span>{name}</span><div className="mvol-row">{vol >= 0 && <><Svg size={14}>{I_VOL}</Svg><Vol value={optVol[id] ?? vol} label={`${name} volume`} onCommit={v => gVol(id, v)} /></>}{members.length > 1 && <button className="mbtn" aria-label={`Remove ${name} from the group`} title={`Remove ${name} from the group`} onClick={() => gUnjoin(id, lead)}><Svg>{I_MINUS}</Svg></button>}</div></li>)}</ul>
      {(wsOn || !!live) && addables.length > 0 && <div className="madd"><span className="eyebrow">{TEXT.media_add_speaker.toUpperCase()}</span>{addables.map(([id, name]) => <button key={id} className={pendingGroup[id] ? 'busy' : ''} disabled={!!pendingGroup[id]} onClick={() => gJoin(id, lead)}><span className="madd-glyph"><Svg>{I_PLUS}</Svg></span><span>{name}</span></button>)}</div>}
      {!row && (wsOn || !!live) && <Elsewhere tiers={tiers} title={model.title} />}
    </div>}
  </Drawer>;
}
