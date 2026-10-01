import { useRef } from 'react';
import { createPortal } from 'react-dom';
import { TEXT } from '../../../src/copy.js';
import type { Model, Tiers } from './model';
import { I_MINUS, I_PLUS, I_VOL, Svg, Vol, dragHandle } from './parts';

/**
 * The group behind the bar's speaker button: each member with its volume and a remove, then the
 * speakers that could join, from whichever tier answers (useTiers). With no tier at all - Home
 * Assistant delivered no Music Assistant player for this device and no direct connection is set
 * up - it says so once the payload has arrived.
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
  const drag = useRef<number | null>(null);
  const any = wsOn || !!me;

  return createPortal([<div key="scrim" className="mscrim top" onClick={onClose} />, <section key="mpanel" className="mpanel" role="dialog" aria-label={TEXT.media_players_title} onClick={e => e.stopPropagation()}>
    <div className="handle" {...dragHandle(drag, '.mpanel', onClose)} />
    <span className="eyebrow">{TEXT.media_players_title.toUpperCase()}</span>
    {members.length > 1 && media?.volume != null && <div className="mvol-row group"><span>{TEXT.media_group_volume}</span><Vol value={groupHeld ? media.ss_volume : media.volume} label={TEXT.media_group_volume} onCommit={v => mediaCmd('volume', { v, src: srcParam })} /></div>}
    {!any && haReady && <p className="mnote">{TEXT.media_no_tiers}</p>}
    {any && <div className="mpanel-scroll">
      {!wsOn && !live && <p className="mnote">{TEXT.media_group_loading}</p>}
      <ul className="mlist">{members.map(([id, name, vol]) => <li key={id}><span>{name}</span><div className="mvol-row">{vol >= 0 && <><Svg size={14}>{I_VOL}</Svg><Vol value={optVol[id] ?? vol} label={`${name} volume`} onCommit={v => gVol(id, v)} /></>}{members.length > 1 && <button className="mbtn" aria-label={`Remove ${name} from the group`} title={`Remove ${name} from the group`} onClick={() => gUnjoin(id)}><Svg>{I_MINUS}</Svg></button>}</div></li>)}</ul>
      {(wsOn || !!live) && addables.length > 0 && <div className="madd"><span className="eyebrow">{TEXT.media_add_speaker.toUpperCase()}</span>{addables.map(([id, name]) => <button key={id} className={pendingGroup[id] ? 'busy' : ''} disabled={!!pendingGroup[id]} onClick={() => gJoin(id)}><span className="madd-glyph"><Svg>{I_PLUS}</Svg></span><span>{name}</span></button>)}</div>}
    </div>}
  </section>], document.body);
}
