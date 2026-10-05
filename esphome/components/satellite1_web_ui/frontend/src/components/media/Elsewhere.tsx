import { useEffect, useRef } from 'react';
import { TEXT } from '../../copy.js';
import type { Other, Tiers } from './model';
import { Art } from './NowPlaying';

const fill = (s: string, ...a: string[]) => a.reduce((t, x) => t.replace('%s', x), s);

/** "Living Room Sonos" with its group's size, "+1", in the muted weight. */
export const WhoName = ({ row }: { row: Other }) => <>{row.name}{row.members.length > 0 && <i>{` +${row.members.length}`}</i>}</>;

/**
 * Join and Take over for a row that can play in sync with this group (or might: the relay cannot
 * tell, and neither can a server without can_group_with), Move here for one that cannot. A row's
 * command in flight rings its button and locks the rest, as does a row asking to confirm.
 */
export function ElseActs({ row, tiers }: { row: Other; tiers: Tiers }) {
  const { rowOps, asking, rJoin, rTakeover, rMove } = tiers;
  const op = rowOps[row.id];
  const locked = !!op || !!asking;
  const btn = (kind: string, go: boolean) => 'melse-btn' + (go ? ' go' : '') + (op?.kind === kind ? ' busy' : '');
  if (row.sync === false) return <div className="melse-acts"><button className={btn('move', true)} disabled={locked} onClick={() => rMove(row)}>{TEXT.media_move_here}</button></div>;
  return <div className="melse-acts">
    <button className={btn('join', false)} disabled={locked} onClick={() => rJoin(row)}>{TEXT.media_join}</button>
    <button className={btn('takeover', true)} disabled={locked} onClick={() => rTakeover(row)}>{TEXT.media_takeover}</button>
  </div>;
}

/**
 * The question Take over and Move here ask when this group has music of its own (mockup 16): what
 * goes, by name, and the action again to confirm it. Focus lands on the action, as in any alert.
 */
export function ElseAsk({ row, tiers, title }: { row: Other; tiers: Tiers; title: string }) {
  const go = useRef<HTMLButtonElement>(null);
  useEffect(() => {
    go.current?.focus();
  }, []);
  const move = tiers.asking?.kind === 'move';
  const body = move
    ? title ? fill(TEXT.media_replace_move, title, row.name) : fill(TEXT.media_replace_move_any, row.name)
    : title ? fill(TEXT.media_replace_takeover, title, row.name) : fill(TEXT.media_replace_takeover_any, row.name);
  return <div className="melse-ask" role="alertdialog" aria-label={TEXT.media_replace_q} aria-describedby={`ask-${row.id}`}>
    <p><b>{TEXT.media_replace_q}</b><small id={`ask-${row.id}`}>{body}</small></p>
    <div className="melse-acts">
      <button className="melse-btn" onClick={() => tiers.setAsking(null)}>{TEXT.cancel}</button>
      <button ref={go} className="melse-btn go" onClick={tiers.confirmAsk}>{move ? TEXT.media_move_here : TEXT.media_takeover}</button>
    </div>
  </div>;
}

/** The players panel's "Playing elsewhere": every other speaker or group Music Assistant is
 *  playing from a queue, with what it plays and the ways to share or take it. */
export function Elsewhere({ tiers, title }: { tiers: Tiers; title: string }) {
  const { elsewhere, asking } = tiers;
  if (!elsewhere.length) return null;
  return <div className="melse">
    <span className="eyebrow">{TEXT.media_elsewhere.toUpperCase()}</span>
    {elsewhere.map(row => <div key={row.id}>
      <div className="melse-row">
        <Art art={row.art} title={row.title} cls="melse-art" />
        <div className="melse-meta">
          <b><WhoName row={row} /></b>
          <small>{[row.title, row.artist].filter(Boolean).join(' \u00b7 ') || (row.state === 'paused' ? TEXT.media_paused : '')}</small>
          {row.sync === false && <small className="melse-why">{TEXT.media_cant_sync}</small>}
        </div>
        <ElseActs row={row} tiers={tiers} />
      </div>
      {asking?.id === row.id && <ElseAsk row={row} tiers={tiers} title={title} />}
    </div>)}
  </div>;
}
