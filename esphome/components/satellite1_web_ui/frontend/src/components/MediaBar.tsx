import type { CSSProperties } from 'react';
import { useEffect, useRef, useState } from 'react';
import { TEXT } from '../copy.js';
import type { Ctx } from '../ctx';
import { relayPausedOf, tintOf, wsPausedOf } from '../lib/media.js';
import { tipDone, tipsDone } from '../lib/tips.js';
import { useTheme } from './bits';
import { Presence } from './Drawer';
import { Art, NowPlaying } from './media/NowPlaying';
import { PlayersPanel } from './media/PlayersPanel';
import { RemoteSheet, rowState } from './media/RemoteSheet';
import type { PeerFor } from './media/RemoteSheet';
import { SearchDrawer } from './media/SearchDrawer';
import { useArtColor, useMediaModel, useTiers } from './media/model';
import type { Other, Tiers } from './media/model';
import { I_PAUSE, I_PLAY, I_SEARCH, I_SPK_BOX, I_VOL, PlayButton, Svg, Vol } from './media/parts';
import { Pager } from './Pager';

/** How often the relay is asked about the rest of the house while nothing of it is showing: roughly
 *  one action call a minute, and only on a visible page with no socket. It is what brings another
 *  speaker's card to the bar - the device hears nothing of other speakers from Home Assistant - and
 *  re-earns a relay-reported pause's trust window, the same moment Music Assistant's own bar would
 *  go dark on a queue cleared elsewhere. Anything open runs useMaData's own faster cycle. */
const MA_POLL_MS = 45000;

/** The speaker button's glyph, with the group's size as its badge - a dot alone, a number past one. */
const SpeakerBadge = ({ n }: { n: number }) => <span className="mspk-ico"><Svg size={24}>{I_SPK_BOX}</Svg>{n > 0 && <span aria-hidden="true" className={n > 1 ? 'mspk-badge n' : 'mspk-badge'}>{n > 1 ? n : ''}</span>}</span>;

/**
 * Another speaker's card on the bar: its art, title, and its name leading the subtitle, with the
 * same three buttons as this device's card (owner's request, October 2026) - search that plays on
 * this speaker, its own players panel, and Play/Pause. Next and Previous are in its drawer: a
 * fourth button left the title too little room on a phone. The volume row is its own, or its
 * group's; a speaker whose volume cannot be set keeps the row's space, so swiping never changes
 * the bar's height. The tap opens its drawer. Its tint is its own art's, reported up so the page's
 * --tint can follow the card in front.
 */
function RemoteCard({
  row,
  tiers,
  onOpen,
  onSearch,
  onPlayers,
  onTint
}: {
  row: Other;
  tiers: Tiers;
  onOpen: () => void;
  onSearch: () => void;
  onPlayers: () => void;
  onTint: (id: string, tint: string) => void;
}) {
  const { theme } = useTheme();
  const tint = tintOf(useArtColor(row.art), theme);
  useEffect(() => onTint(row.id, tint), [tint]);
  const { remote, optVol, rTransport, rVolume } = tiers;
  const busy = (c: string) => !!remote[`${row.id}:${c}`];
  const playing = row.state === 'playing';
  return <div className="mcard" style={{
    '--tint': tint
  } as CSSProperties} onClick={onOpen}>
    <div className="mbar-row">
      <Art art={row.art} title={row.title} cls="mbar-art" />
      <button className="mbar-meta" aria-label={`${TEXT.media_open_remote} ${row.name}`}><b>{row.title || rowState(row)}</b><small><span className="mwho"><Svg size={12}>{I_SPK_BOX}</Svg>{row.name}{row.members.length > 0 && ` +${row.members.length}`}</span>{row.artist && ` \u00b7 ${row.artist}`}</small></button>
      <button className="mbtn msearch" aria-label={`${TEXT.search_open} for ${row.name}`} onClick={e => {
        e.stopPropagation();
        onSearch();
      }}><Svg size={22}>{I_SEARCH}</Svg></button>
      <button className="mbtn mspk" aria-label={`${TEXT.media_players_title} \u00b7 ${row.name}`} onClick={e => {
        e.stopPropagation();
        onPlayers();
      }}><SpeakerBadge n={tiers.groupOf(row).members.length} /></button>
      <button className={'mplay sm' + (busy('play_pause') ? ' busy' : '')} aria-label={`${playing ? 'Pause' : 'Play'} ${row.name}`} disabled={busy('play_pause')} onClick={e => {
        e.stopPropagation();
        rTransport(row, 'play_pause');
      }}><Svg size={16}>{playing ? I_PAUSE : I_PLAY}</Svg></button>
    </div>
    {row.vctl ? <div className="mbar-vol" onClick={e => e.stopPropagation()}><span className="mbar-vol-ico"><Svg size={24}>{I_VOL}</Svg></span><Vol value={optVol[row.id] ?? row.volume} label={`${row.name} volume`} onCommit={v => rVolume(row, v)} /></div> : <div className="mbar-vol blank" aria-hidden="true"><span className="mbar-vol-ico"><Svg size={24}>{I_VOL}</Svg></span><input type="range" disabled tabIndex={-1} /></div>}
  </div>;
}

/**
 * The media bar, its Now Playing sheet, the players panel and search, and a card for every other
 * speaker Music Assistant is playing, swiped to sideways. Hidden until the first /api/sat1/media
 * answer, which never comes on a device with no media player (404). What each tier adds is in
 * docs/web-ui.md, "The media footer and its three tiers".
 */
export function MediaBar({
  ctx,
  peerFor
}: {
  ctx: Ctx;
  peerFor?: PeerFor;
}) {
  const [open, setOpen] = useState(false);
  // The players panel and search, each for this device (`row` null) or the speaker of the card
  // they were opened from. Search keeps the target it opened with, so a speaker that stops while
  // someone browses can still be played on; the panel needs its speaker's live group, and closes
  // with its card.
  const [panel, setPanel] = useState<{ row: string | null } | null>(null);
  const [search, setSearch] = useState<{ target: { queue: string; name: string } | null } | null>(null);
  // The card in front, by its speaker's id (null is this device's own), and the speaker whose
  // drawer is open. By id rather than index so a card leaving ahead of it does not shift the view.
  // One leaving from the front - stopped, or joined into this group by Take over - hands the front
  // to the card before it (owner's request, October 2026), found in the cards as last shown.
  const [front, setFront] = useState<string | null>(null);
  const [sheet, setSheet] = useState<string | null>(null);
  const tiers = useTiers(ctx.ha, ctx.device?.mac, open || !!panel || !!sheet || !!front);
  const { cards } = tiers;
  const ids = cards.map(c => c.id);
  const shown = useRef<string[]>([]);
  let lead = front;
  if (front && !ids.includes(front)) {
    const was = shown.current.indexOf(front);
    lead = shown.current.slice(0, Math.max(0, was)).reverse().find(id => ids.includes(id)) ?? null;
  }
  const at = lead ? ids.indexOf(lead) + 1 : 0;
  const panelRow = panel?.row ? cards.find(c => c.id === panel.row) || null : null;
  const sheetRow = sheet ? cards.find(c => c.id === sheet) || null : null;
  useEffect(() => {
    shown.current = ids;
    if (lead !== front) setFront(lead);
    if (panel?.row && !panelRow) setPanel(null);
    if (sheet && !sheetRow) setSheet(null);
  });

  // Neither tier says "paused" for a Sendspin player - the pause stops the stream and the MA player
  // goes idle - so both read "not playing, but the queue still holds the resume point" (wsPausedOf
  // in src/lib/media.js has the detail).
  const wsPaused = wsPausedOf(tiers.wsOn, tiers.ws.me, tiers.wsOn ? tiers.ws.queue : null);
  const relayPaused = relayPausedOf(tiers.wsOn, tiers.ma, Date.now());
  const model = useMediaModel(wsPaused || relayPaused);
  const { theme } = useTheme();
  const ownTint = tintOf(useArtColor(model.art), theme);
  const [tints, setTints] = useState<Record<string, string>>({});
  const onTint = (id: string, t: string) => setTints(m => m[id] === t ? m : { ...m, [id]: t });
  const tint = lead ? tints[lead] || 'transparent' : ownTint;
  // On the root rather than the bar, so the drawers portaled to <body> wash in it too - the card in
  // front's, so another speaker's drawer opens in its colour.
  useEffect(() => {
    const root = document.documentElement;
    root.style.setProperty('--tint', tint);
    return () => root.style.removeProperty('--tint');
  }, [tint]);

  // The group stream stopping means paused somewhere or ended; the transition is the one moment
  // Home Assistant has a fresh answer, so it is asked exactly then.
  const ssState = model.media?.ss_state;
  const prevSs = useRef(ssState);
  useEffect(() => {
    const was = prevSs.current;
    prevSs.current = ssState;
    if (was === 2 && ssState != null && ssState !== 2) tiers.maAsk();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ssState]);
  const relayOnly = !tiers.wsOn && !!tiers.me;
  const idle = !open && !panel && !sheet && !front;
  useEffect(() => {
    if (!relayOnly || !idle) return undefined;
    const t = setInterval(() => {
      if (!document.hidden) tiers.maAsk();
    }, MA_POLL_MS);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [relayOnly, idle]);

  // The one-time hint that there is more to swipe to: the strip slides a little left and springs
  // back the first time a second card appears, until a swipe or dot tap retires the tip.
  const n = cards.length + 1;
  const [nudge, setNudge] = useState(false);
  const nudged = useRef(false);
  useEffect(() => {
    if (n < 2 || nudged.current || tipsDone().includes('bar_swipe')) return;
    nudged.current = true;
    setNudge(true);
  }, [n]);
  useEffect(() => {
    if (!nudge) return undefined;
    const t = setTimeout(() => setNudge(false), 1300);
    return () => clearTimeout(t);
  }, [nudge]);

  if (!model.media) return null;
  const { media, mediaCmd, srcParam, active, announcing } = model;
  const title = model.title || model.stateText || TEXT.media_idle_bar;
  const sub = [model.artist, active || announcing ? model.srcText : ''].filter(Boolean).join(' \u00b7 ');
  const gcount = tiers.members.length;
  const searchFor = (row: Other) => setSearch({ target: { queue: row.id, name: row.name + (row.members.length ? ` +${row.members.length}` : '') } });

  // A card is a div with a click because its controls are buttons, and buttons do not nest. The
  // volume row stops the click on the row rather than the input, so a miss beside the slider does
  // not open Now Playing mid-drag.
  const own = <div className="mcard" style={{
    '--tint': ownTint
  } as CSSProperties} onClick={() => {
    tipDone('music');
    setOpen(true);
  }}>
    <div className="mbar-row">
      <Art art={model.art} title={model.title} cls="mbar-art" />
      <button className="mbar-meta" aria-label="Open media view" aria-expanded={open}><b>{title}</b>{sub && <small>{sub}</small>}</button>
      <button className="mbtn msearch" aria-label={TEXT.search_open} onClick={e => {
        e.stopPropagation();
        setSearch({ target: null });
      }}><Svg size={22}>{I_SEARCH}</Svg></button>
      <button className="mbtn mspk" aria-label={TEXT.media_players_title} onClick={e => {
        e.stopPropagation();
        tipDone('group');
        setPanel({ row: null });
      }}><SpeakerBadge n={gcount} /></button>
      {active && !announcing && <PlayButton model={model} sm />}
    </div>
    {media.volume != null && <div className="mbar-vol" onClick={e => e.stopPropagation()}><span className="mbar-vol-ico"><Svg size={24}>{I_VOL}</Svg></span><Vol value={model.groupHeld ? media.ss_volume : media.volume} label="Volume" onCommit={v => mediaCmd('volume', { v, src: srcParam })} /></div>}
  </div>;

  return <div className="mroot">
    <div className="mbar">
      <Pager prefix="mc" label={TEXT.media_cards_label} view={at} onView={i => {
        tipDone('bar_swipe');
        setFront(i === 0 ? null : cards[i - 1]?.id ?? null);
      }} labels={[tiers.selfName || TEXT.media_this_speaker, ...cards.map(c => c.name)]} keys={['own', ...cards.map(c => c.id)]} nudge={nudge} cards={[own, ...cards.map(c => <RemoteCard key={c.id} row={c} tiers={tiers} onTint={onTint} onOpen={() => setSheet(c.id)} onSearch={() => searchFor(c)} onPlayers={() => setPanel({ row: c.id })} />)]} />
    </div>
    <Presence>{open && <NowPlaying model={model} tiers={tiers} ip={ctx.device?.ip} onClose={() => setOpen(false)} onPlayers={() => setPanel({ row: null })} onSearch={() => {
      setOpen(false);
      setSearch({ target: null });
    }} />}</Presence>
    <Presence>{panel && (!panel.row || panelRow) && <PlayersPanel model={model} tiers={tiers} haReady={!!ctx.ha?.d} row={panelRow} onClose={() => {
      setPanel(null);
      tiers.setGroupNote('');
      tiers.setAsking(null);
    }} onSearch={() => {
      setPanel(null);
      if (panelRow) searchFor(panelRow);
      else setSearch({ target: null });
    }} />}</Presence>
    <Presence>{sheetRow && <RemoteSheet row={sheetRow} tiers={tiers} title={model.title} peerFor={peerFor} onClose={() => {
      setSheet(null);
      tiers.setGroupNote('');
      tiers.setAsking(null);
    }} />}</Presence>
    <Presence>{search && <SearchDrawer tiers={tiers} ip={ctx.device?.ip} target={search.target} onClose={() => setSearch(null)} />}</Presence>
  </div>;
}
