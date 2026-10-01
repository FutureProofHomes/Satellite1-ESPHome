import type { CSSProperties } from 'react';
import { useEffect, useRef, useState } from 'react';
import { TEXT } from '../copy.js';
import type { Ctx } from '../ctx';
import { relayPausedOf, tintOf, wsPausedOf } from '../lib/media.js';
import { Art, NowPlaying } from './media/NowPlaying';
import { PlayersPanel } from './media/PlayersPanel';
import { SearchDrawer } from './media/SearchDrawer';
import { useArtColor, useMediaModel, useTiers } from './media/model';
import { I_SPK_BOX, I_VOL, PlayButton, Svg, Vol } from './media/parts';

/** How often a relay-reported pause, alone holding the bar up, is re-asked while nothing is open:
 *  roughly one action call a minute, and only while a paused group is showing - the price of a
 *  queue cleared elsewhere going dark here within a minute rather than never, the same moment Music
 *  Assistant's own bar goes dark. */
const MA_PAUSED_RECHECK_MS = 45000;

/**
 * The media bar, its Now Playing sheet, the players panel and search. Hidden until the first
 * /api/sat1/media answer, which never comes on a device with no media player (404). What each tier
 * adds is in docs/web-ui.md, "The media footer and its three tiers".
 */
export function MediaBar({
  ctx
}: {
  ctx: Ctx;
}) {
  const [open, setOpen] = useState(false);
  const [panel, setPanel] = useState(false);
  const [search, setSearch] = useState(false);
  const tiers = useTiers(ctx.ha, ctx.device?.mac, open || panel);

  // Neither tier says "paused" for a Sendspin player - the pause stops the stream and the MA player
  // goes idle - so both read "not playing, but the queue still holds the resume point" (wsPausedOf
  // in src/lib/media.js has the detail).
  const wsPaused = wsPausedOf(tiers.wsOn, tiers.ws.me, tiers.wsOn ? tiers.ws.queue : null);
  const relayPaused = relayPausedOf(tiers.wsOn, tiers.ma, Date.now());
  const model = useMediaModel(wsPaused || relayPaused);
  const tint = tintOf(useArtColor(model.art));

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
  // While a relay-reported pause alone holds the bar up, re-ask slowly so its trust window keeps
  // being re-earned for as long as the pause is real. The open drawers run useMaData's own faster
  // cycle and the socket streams updates unasked, so neither needs this.
  useEffect(() => {
    if (!relayPaused || open || panel) return undefined;
    const t = setInterval(() => tiers.maAsk(), MA_PAUSED_RECHECK_MS);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [relayPaused, open, panel]);

  // The three drawers share one blur and one Escape, which closes the topmost.
  const drawer = open || panel || search;
  useEffect(() => {
    if (!drawer) return undefined;
    document.body.classList.add('has-drawer');
    const esc = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return;
      if (search) setSearch(false);else if (panel) setPanel(false);else setOpen(false);
    };
    document.addEventListener('keydown', esc);
    return () => {
      document.body.classList.remove('has-drawer');
      document.removeEventListener('keydown', esc);
    };
  }, [drawer, search, panel]);

  if (!model.media) return null;
  const { media, mediaCmd, srcParam, active, announcing } = model;
  const title = model.title || model.stateText || TEXT.media_idle_bar;
  const sub = [model.artist, active || announcing ? model.srcText : ''].filter(Boolean).join(' \u00b7 ');
  const gcount = tiers.members.length;

  // The bar is a div with a click because its controls are buttons, and buttons do not nest. The
  // volume row stops the click on the row rather than the input, so a miss beside the slider does
  // not open Now Playing mid-drag.
  return <div className="mroot" style={{
    '--tint': tint
  } as CSSProperties}>
    <div className="mbar" onClick={() => setOpen(true)}>
      <div className="mbar-row">
        <Art art={model.art} title={model.title} cls="mbar-art" />
        <button className="mbar-meta" aria-label="Open media view" aria-expanded={open}><b>{title}</b>{sub && <small>{sub}</small>}</button>
        <button className="mbtn mspk" aria-label={TEXT.media_players_title} onClick={e => {
          e.stopPropagation();
          setPanel(true);
        }}><span style={{
            position: 'relative',
            display: 'inline-flex',
            color: '#fff'
          }}><Svg size={24}>{I_SPK_BOX}</Svg>{gcount > 0 && <span aria-hidden="true" style={{
              position: 'absolute',
              top: -3,
              right: -4,
              minWidth: gcount > 1 ? 12 : 8,
              height: gcount > 1 ? 12 : 8,
              padding: gcount > 1 ? '0 2px' : 0,
              borderRadius: 999,
              background: 'var(--orb-a, var(--accent))',
              color: '#fff',
              fontSize: 8,
              fontWeight: 700,
              lineHeight: '12px',
              textAlign: 'center',
              fontStyle: 'normal'
            }}>{gcount > 1 ? gcount : ''}</span>}</span></button>
        {active && !announcing && <PlayButton model={model} sm />}
      </div>
      {media.volume != null && <div className="mbar-vol" onClick={e => e.stopPropagation()}><span style={{
          display: 'inline-flex',
          color: '#fff'
        }}><Svg size={24}>{I_VOL}</Svg></span><Vol value={model.groupHeld ? media.ss_volume : media.volume} label="Volume" onCommit={v => mediaCmd('volume', { v, src: srcParam })} /></div>}
    </div>
    {open && <NowPlaying model={model} tiers={tiers} ip={ctx.device?.ip} onClose={() => setOpen(false)} onPlayers={() => setPanel(true)} onSearch={() => {
      setOpen(false);
      setSearch(true);
    }} />}
    {panel && <PlayersPanel model={model} tiers={tiers} haReady={!!ctx.ha?.d} onClose={() => setPanel(false)} />}
    {search && <SearchDrawer tiers={tiers} ip={ctx.device?.ip} onClose={() => setSearch(false)} />}
  </div>;
}
