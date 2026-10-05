import { useEffect, useState } from 'react';
import { TEXT } from '../../copy.js';
import { rowClock } from '../../lib/media.js';
import type { PeerLinkProps } from '../Satellite1Now';
import { Drawer } from '../Drawer';
import { ElseActs, ElseAsk, WhoName } from './Elsewhere';
import type { Other, Tiers, Transport } from './model';
import { Art, Seek } from './NowPlaying';
import { I_NEXT, I_PAUSE, I_PLAY, I_PREV, I_SPK_BOX, Svg, Vol } from './parts';

export type PeerFor = (mac: string, name: string) => { name: string; props: PeerLinkProps } | null;

/** A row's state as the bar and the drawer say it. */
export const rowState = (row: Other) => row.state === 'playing' ? TEXT.media_playing : row.state === 'paused' ? TEXT.media_paused : TEXT.media_stopped;

/**
 * Another speaker's Now Playing: what it plays, its transport and volume (its whole group's), and
 * Join and Take over (or Move here) when it plays from a Music Assistant queue. A Satellite1 adds
 * the way to open its own page, the device switcher's jump. The progress bar needs the socket's
 * playhead; through the relay the rest still works.
 */
export function RemoteSheet({
  row,
  tiers,
  title,
  peerFor,
  onClose
}: {
  row: Other;
  tiers: Tiers;
  title: string;
  peerFor?: PeerFor;
  onClose: () => void;
}) {
  const { wsOn, ws, remote, optVol, groupNote, asking, rTransport, rVolume, rSeek } = tiers;
  const playing = row.state === 'playing';
  const [, tick] = useState(0);
  useEffect(() => {
    if (!playing || !row.clock) return undefined;
    const t = setInterval(() => tick(n => n + 1), 500);
    return () => clearInterval(t);
  }, [playing, !!row.clock]);
  const clock = wsOn ? rowClock(row, Date.now(), ws.skew) : null;
  const peer = peerFor?.(row.mac, row.name) || null;
  const busy = (c: Transport) => !!remote[`${row.id}:${c}`];
  const btn = (c: Transport) => 'mbtn' + (busy(c) ? ' busy' : '');

  return <Drawer label={row.name} onClose={onClose} className="msheet">
    <span className="rs-who"><Svg size={12}>{I_SPK_BOX}</Svg><span><WhoName row={row} /></span></span>
    <Art art={row.art} title={row.title} cls="mart" />
    <div className="mmeta">
      <h2>{row.title || rowState(row)}</h2>
      {row.artist && <p>{row.artist}</p>}
      <small><span className={'dot' + (playing ? '' : ' off')} /> {rowState(row)}</small>
    </div>
    {clock && clock.dur > 0 && <Seek pos={clock.pos} dur={clock.dur} canSeek={row.transferable} onSeek={ms => rSeek(row, ms)} />}
    <div className="mrow">
      <button className={btn('previous')} aria-label="Previous track" disabled={busy('previous')} onClick={() => rTransport(row, 'previous')}><Svg>{I_PREV}</Svg></button>
      <button className={'mplay' + (busy('play_pause') ? ' busy' : '')} aria-label={playing ? 'Pause' : 'Play'} disabled={busy('play_pause')} onClick={() => rTransport(row, 'play_pause')}><Svg size={22}>{playing ? I_PAUSE : I_PLAY}</Svg></button>
      <button className={btn('next')} aria-label="Next track" disabled={busy('next')} onClick={() => rTransport(row, 'next')}><Svg>{I_NEXT}</Svg></button>
    </div>
    {row.vctl && <div className="mvol-row group"><span>{row.members.length ? TEXT.media_group_volume : TEXT.media_volume}</span><Vol value={optVol[row.id] ?? row.volume} label={`${row.name} volume`} onCommit={v => rVolume(row, v)} /></div>}
    {groupNote && <p className="mnote warn" role="status">{groupNote}</p>}
    {(row.transferable || peer) && <div className="rs-acts">
      {row.transferable && <ElseActs row={row} tiers={tiers} />}
      {peer && <a className="melse-btn rs-open" {...peer.props}>{TEXT.media_open_peer.replace('%s', peer.name)}</a>}
    </div>}
    {asking?.id === row.id && <ElseAsk row={row} tiers={tiers} title={title} />}
  </Drawer>;
}
