import { TEXT } from '../../copy.js';
import { Drawer } from '../Drawer';
import type { Model, Tiers } from './model';
import { I_MINUS, I_PLUS, I_VOL, Svg, Vol } from './parts';

/**
 * The group behind the bar's speaker button: each member with its volume and a remove, then the
 * speakers that could join, from whichever tier answers (useTiers). With no tier at all - Home
 * Assistant delivered no Music Assistant player for this device (Home Assistant absent, its actions
 * off, or no Music Assistant) and no direct connection is set up - it says so once the payload has
 * arrived, and says nothing while it is still loading.
 *
 * The whole-group slider shows only while there is a group (owner's request, September 2026): with
 * one speaker it duplicated that speaker's own row and the bar's slider both, and "Volume" over a
 * list of speakers read as nobody's in particular. A tapped add row rings and locks until the join
 * is confirmed, per row, so two quick adds spin independently.
 */
export function PlayersPanel({
  model,
  tiers,
  haReady,
  onClose
}: {
  model: Model;
  tiers: Tiers;
  haReady: boolean;
  onClose: () => void;
}) {
  const { media, mediaCmd, srcParam, groupHeld } = model;
  const { wsOn, me, live, members, addables, optVol, pendingGroup, gVol, gUnjoin, gJoin } = tiers;
  const any = wsOn || !!me;

  return <Drawer label={TEXT.media_players_title} onClose={onClose} className="mpanel">
    <span className="eyebrow">{TEXT.media_players_title.toUpperCase()}</span>
    {members.length > 1 && media?.volume != null && <div className="mvol-row group"><span>{TEXT.media_group_volume}</span><Vol value={groupHeld ? media.ss_volume : media.volume} label={TEXT.media_group_volume} onCommit={v => mediaCmd('volume', { v, src: srcParam })} /></div>}
    {!any && haReady && <p className="mnote">{TEXT.media_no_tiers}</p>}
    {any && <div className="mpanel-scroll">
      {!wsOn && !live && <p className="mnote">{TEXT.media_group_loading}</p>}
      <ul className="mlist">{members.map(([id, name, vol]) => <li key={id}><span>{name}</span><div className="mvol-row">{vol >= 0 && <><Svg size={14}>{I_VOL}</Svg><Vol value={optVol[id] ?? vol} label={`${name} volume`} onCommit={v => gVol(id, v)} /></>}{members.length > 1 && <button className="mbtn" aria-label={`Remove ${name} from the group`} title={`Remove ${name} from the group`} onClick={() => gUnjoin(id)}><Svg>{I_MINUS}</Svg></button>}</div></li>)}</ul>
      {(wsOn || !!live) && addables.length > 0 && <div className="madd"><span className="eyebrow">{TEXT.media_add_speaker.toUpperCase()}</span>{addables.map(([id, name]) => <button key={id} className={pendingGroup[id] ? 'busy' : ''} disabled={!!pendingGroup[id]} onClick={() => gJoin(id)}><span className="madd-glyph"><Svg>{I_PLUS}</Svg></span><span>{name}</span></button>)}</div>}
    </div>}
  </Drawer>;
}
