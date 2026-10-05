import type { CSSProperties } from 'react';
import { useEffect, useState } from 'react';
import { TEXT } from '../../copy.js';
import { REPEAT_MODE, SUP, fmtTime, groupLabel, nextRepeat, pct, queueClock } from '../../lib/media.js';
import { Drawer } from '../Drawer';
import { MaPanel } from './MaPanel';
import { usePlayhead } from './model';
import type { Model, Tiers } from './model';
import { I_NEXT, I_NOTE, I_PREV, I_REPEAT, I_SEARCH, I_SHUFFLE, I_SPK_BOX, PlayButton, Svg, useRange } from './parts';

/** The artwork, or the note glyph while there is none or it will not load. Keyed by URL so a track
 *  change swaps the element instead of the old picture lingering while the new one loads. */
export function Art({
  art,
  title,
  cls
}: {
  art: string;
  title: string;
  cls: string;
}) {
  const [bad, setBad] = useState('');
  return <div className={cls}>{art && bad !== art ? <img key={art} src={art} alt={title ? `${title} album artwork` : ''} onError={() => setBad(art)} /> : <span className="mart-ph"><Svg size={cls === 'mart' ? 48 : 18}>{I_NOTE}</Svg></span>}</div>;
}

/**
 * The scrubber, read-only until something can seek: the device's own protocol has no seek, so a
 * drag lands on the Music Assistant socket or as a relayed media_seek.
 */
export function Seek({
  pos,
  dur,
  canSeek,
  onSeek
}: {
  pos: number;
  dur: number;
  canSeek: boolean;
  onSeek: (ms: number) => void;
}) {
  // The relay's echo lands near the seek, not on it, hence the wide tolerance.
  const [shown, props] = useRange(pos, 2500, onSeek);
  return <>
    <input className="mseek" type="range" aria-label="Seek" min={0} max={dur} step={1000} disabled={!canSeek} aria-valuetext={`${fmtTime(shown)} of ${fmtTime(dur)}`} style={{
      '--pct': pct(shown, dur)
    } as CSSProperties} {...props} />
    <div className="time"><span>{fmtTime(shown)}</span><span>{fmtTime(dur)}</span></div>
  </>;
}

/**
 * The expanded view. It carries no volume slider (owner's request, September 2026): the bar and the
 * players panel both have one, and a third copy here was clutter, not control.
 */
export function NowPlaying({
  model,
  tiers,
  ip,
  onClose,
  onPlayers,
  onSearch
}: {
  model: Model;
  tiers: Tiers;
  ip?: string;
  onClose: () => void;
  onPlayers: () => void;
  onSearch: () => void;
}) {
  const { media, mediaCmd, sendspin, playing, active, announcing, srcParam, pending, startPending } = model;
  const { ws, wsOn, me, maCmd, members } = tiers;
  const { pos, dur } = usePlayhead(media, playing);

  // With the socket up the queue's own clock, ticked by every queue_time_updated, drives the
  // scrubber, so the position is real rather than extrapolated from a poll.
  const clock = wsOn ? queueClock(ws.queue, Date.now()) : null;
  const sDur = clock ? clock.dur : dur;
  const sPos = clock ? clock.pos : pos;
  const seekTo = (v: number) => {
    const t = Math.round(v / 1000);
    if (clock) ws.cmd('player_queues/seek', { queue_id: clock.id, position: t }).catch(() => {});
    else maCmd('seek', { e: me, t });
  };

  // Shown at once, because a toggle that sits unmoved until the next poll reads as refused; the
  // pending ring says "working" alongside it. The override lasts as long as the ring: it settles on
  // the echo, or gives up at the deadline, and a refused toggle falls back to what the player says.
  const [optShuffle, setOptShuffle] = useState<boolean | null>(null);
  const [optRepeat, setOptRepeat] = useState<number | null>(null);
  const shufflePending = !!pending.shuffle;
  const repeatPending = !!pending.repeat;
  useEffect(() => {
    if (!shufflePending) setOptShuffle(null);
  }, [media?.shuffle, shufflePending]);
  useEffect(() => {
    if (!repeatPending) setOptRepeat(null);
  }, [media?.repeat, repeatPending]);
  const ctrl = media?.shuffle != null;
  const shuffle = optShuffle ?? media?.shuffle === 1;
  const repeat = optRepeat ?? media?.repeat ?? 0;
  const supHas = (bit: number) => media?.sup == null || (media.sup & bit) !== 0;
  const toggleShuffle = () => {
    const next = shuffle ? 0 : 1;
    setOptShuffle(!shuffle);
    startPending('shuffle', m => m?.shuffle === next);
    mediaCmd('shuffle', { v: next, src: 'sendspin' });
  };
  const cycleRepeat = () => {
    const next = nextRepeat(repeat);
    setOptRepeat(next);
    startPending('repeat', m => m?.repeat === next);
    mediaCmd('repeat', { m: REPEAT_MODE[next], src: 'sendspin' });
  };
  // No queue index to watch: a skip is done when the title moves or the position falls back (prev
  // mid-track restarts the same song). Two identical consecutive tracks ride to the deadline - a
  // slightly long spin, never a wrong state.
  const skip = (key: 'prev' | 'next') => {
    const t0 = media?.title;
    const p0 = media?.pos ?? 0;
    startPending(key, m => m?.title !== t0 || (m?.pos ?? 0) < p0);
    mediaCmd(key, { src: srcParam });
  };
  // Each control disables while its own command is pending, as Music Assistant's do; that is also
  // the guard against a double-send the device would replay.
  const btn = (key: string, on?: boolean) => 'mbtn' + (on ? ' on' : '') + (pending[key] ? ' busy' : '');

  return <Drawer label="Now playing" onClose={onClose} className="msheet">
    <button className="mbtn msheet-search" aria-label={TEXT.search_open} onClick={onSearch}><Svg size={18}>{I_SEARCH}</Svg></button>
    <Art art={model.art} title={model.title} cls="mart" />
    <div className="mmeta">
      <h2>{model.title || (active ? '' : TEXT.media_idle_bar)}</h2>
      {model.artist && <p>{model.artist}</p>}
      {model.album && <p>{model.album}</p>}
      <small>{model.stateText ? <><span className={'dot' + (playing ? '' : ' off')} /> {`${model.stateText} \u00b7 ${model.srcText}`}</> : TEXT.media_idle}</small>
    </div>
    {sDur > 0 && sendspin && active && <Seek pos={sPos} dur={sDur} canSeek={!!clock || !!me} onSeek={seekTo} />}
    {active && !announcing && <div className="mrow">
      {sendspin && ctrl && supHas(SUP.SHUFFLE) && <button className={btn('shuffle', shuffle)} aria-label="Shuffle" aria-pressed={shuffle} disabled={!!pending.shuffle} onClick={toggleShuffle}><Svg>{I_SHUFFLE}</Svg></button>}
      {sendspin && supHas(SUP.PREV) && <button className={btn('prev')} aria-label="Previous track" disabled={!!pending.prev} onClick={() => skip('prev')}><Svg>{I_PREV}</Svg></button>}
      <PlayButton model={model} />
      {sendspin && supHas(SUP.NEXT) && <button className={btn('next')} aria-label="Next track" disabled={!!pending.next} onClick={() => skip('next')}><Svg>{I_NEXT}</Svg></button>}
      {sendspin && ctrl && <button className={btn('repeat', repeat !== 0)} aria-label={repeat === 1 ? 'Repeat one' : 'Repeat'} aria-pressed={repeat !== 0} disabled={!!pending.repeat} onClick={cycleRepeat}><Svg>{I_REPEAT}</Svg>{repeat === 1 && <i>1</i>}</button>}
    </div>}
    <button className="mchip" onClick={onPlayers}><Svg>{I_SPK_BOX}</Svg><span>{groupLabel(members, TEXT.media_players_title)}</span></button>
    <MaPanel tiers={tiers} ip={ip} />
  </Drawer>;
}
