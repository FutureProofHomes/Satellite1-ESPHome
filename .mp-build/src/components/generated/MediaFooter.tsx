/**
 * The floating media bar on every route, the expanded media view behind a tap on it, the
 * players panel behind its speaker button, and the Music Assistant search drawer behind the
 * top bar's magnifying glass. Ported from media.jsx + search.jsx. The bar, sheet and panel
 * wear the album's colour the way Music Assistant's own player does.
 *
 * One Music Assistant connection state feeds all of it: Disconnect in the connection panel
 * flips the search drawer into its set-up-first flow, exactly like pulling the token does on
 * the device.
 */
import React, { useEffect, useRef, useState } from 'react';
import { Chevron, I_MINUS, I_NEXT, I_NOTE, I_PAUSE, I_PLAY, I_PLUS, I_PREV, I_REPEAT, I_SHUFFLE, I_SPK, I_VOL, mi, rangeFill, useDrawer, useSheetDrag } from './ui';
import { DEVICE } from './mock';
const ART = 'https://storage.googleapis.com/storage.magicpath.ai/component-assets/454802455327313920/454809714925146112/ecc89c59771f06b4c2fa7960733c67a3eb86e4e4fb31599723751ffb6e85f432.png';
const TRACKS = [{
  title: 'Amber Skies',
  artist: 'Fieldlight',
  album: 'Harvest',
  dur: 254000
}, {
  title: 'Low September Sun',
  artist: 'Fieldlight',
  album: 'Harvest',
  dur: 221000
}];

/** The artwork's colour as the tint custom properties media.jsx computes from the album pixels. */
const TINT = {
  ['--tint' as string]: 'hsl(28,42%,36%)',
  ['--tint2' as string]: 'hsl(28,42%,26%)',
  ['--tfg' as string]: '#fff',
  ['--tdim' as string]: 'rgba(255,255,255,.72)',
  ['--tbtn' as string]: 'hsl(28,42%,85%)'
} as React.CSSProperties;
const fmtTime = (ms: number) => {
  const s = Math.max(0, Math.round(ms / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
};

/* The drawer's own magnifier, on the media glyphs' 16-box: .search-field and .search-empty size
   their `.mi` children, so the ni-class top bar glyph would render unsized here. */
const I_SEARCH = mi(<>
    <circle cx="7" cy="7" r="4.4" />
    <path d="M10.4 10.4 14 14" />
  </>);
const I_X = <svg className="mi" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" aria-hidden="true">
    <path d="M2.8 2.8l6.4 6.4M9.2 2.8l-6.4 6.4" />
  </svg>;
const I_CHECK = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
    <path d="M3.2 8.6 6.4 11.8 12.8 4.8" />
  </svg>;
function Vol({
  value,
  onCommit,
  label
}: {
  value: number;
  onCommit: (v: number) => void;
  label?: string;
}) {
  const [drag, setDrag] = useState<number | null>(null);
  const shown = drag ?? value;
  return <div className="mvol">
      <input type="range" min={0} max={100} step={1} value={shown} aria-label={label || 'Volume'} style={rangeFill(shown, 0, 100)} onInput={e => setDrag(Number((e.target as HTMLInputElement).value))} onChange={e => {
      onCommit(Number((e.target as HTMLInputElement).value));
      setDrag(null);
    }} />
    </div>;
}
function Artwork({
  big
}: {
  big?: boolean;
}) {
  return <div className={big ? 'mart' : 'mbar-art'}>
      <img src={ART} alt="" loading="lazy" />
    </div>;
}
function SeekBar({
  pos,
  dur,
  onSeek
}: {
  pos: number;
  dur: number;
  onSeek: (v: number) => void;
}) {
  const [drag, setDrag] = useState<number | null>(null);
  const shown = drag ?? pos;
  return <div className="mseek">
      <span className="num">{fmtTime(shown)}</span>
      <input type="range" min={0} max={dur} step={1000} value={shown} aria-label="Seek" style={rangeFill(shown, 0, dur)} onInput={e => setDrag(Number((e.target as HTMLInputElement).value))} onChange={e => {
      onSeek(Number((e.target as HTMLInputElement).value));
      setDrag(null);
    }} />
      <span className="num">{fmtTime(dur)}</span>
    </div>;
}

/* ------------------------------------------------------------------ */
/* The Music Assistant connection panel                                */
/* ------------------------------------------------------------------ */

function MaPanel({
  connected,
  onConnect,
  onDisconnect,
  defaultOpen
}: {
  connected: boolean;
  onConnect: () => void;
  onDisconnect: () => void;
  defaultOpen?: boolean;
}) {
  const [open, setOpen] = useState(!!defaultOpen);
  const [url, setUrl] = useState('http://192.168.4.10:8095');
  const [token, setToken] = useState(connected ? 'mass_9f27c1e4' : '');
  return <div className="mapanel">
      <button className="offhead" aria-expanded={open} onClick={() => setOpen(!open)}>
        <span className={`dot${connected ? ' ok' : ''}`} />
        <span className="grow">Music Assistant</span>
        <Chevron down={open} cls="caret-s" />
      </button>
      {open && <div className="mapanel-body">
          <p className="dim xs">
            Connect straight to Music Assistant for search, queues and instant updates - the address and a long-lived token from MA's
            profile settings.
          </p>
          <input className="in" type="text" value={url} placeholder="http://homeassistant.local:8095" aria-label="Music Assistant address" onInput={e => setUrl((e.target as HTMLInputElement).value)} />
          {!connected && <button className="btn sm mascan-btn" onClick={() => {}}>
              Find my server
            </button>}
          <input className="in" type="password" value={token} placeholder="Long-lived token" aria-label="Music Assistant token" onInput={e => setToken((e.target as HTMLInputElement).value)} />
          <div className="mapanel-row">
            <button className="btn" onClick={() => {
          if (url.trim() && token.trim()) onConnect();
        }}>
              Connect
            </button>
            {connected && <button className="btn" onClick={() => {
          setToken('');
          onDisconnect();
        }}>
                Disconnect
              </button>}
          </div>
          {connected && <p className="dim xs">Connected.</p>}
        </div>}
    </div>;
}

/* ------------------------------------------------------------------ */
/* The search drawer                                                   */
/* ------------------------------------------------------------------ */

type Hit = {
  uri: string;
  name: string;
  kind: 'track' | 'artist' | 'album' | 'playlist';
  sub: string;
  art: boolean;
};
const LIBRARY: Hit[] = [{
  uri: 't1',
  name: 'Amber Skies',
  kind: 'track',
  sub: 'Track \u00b7 Fieldlight',
  art: true
}, {
  uri: 't2',
  name: 'Low September Sun',
  kind: 'track',
  sub: 'Track \u00b7 Fieldlight',
  art: true
}, {
  uri: 't3',
  name: 'Harvest Wind',
  kind: 'track',
  sub: 'Track \u00b7 The Analog Hearts',
  art: true
}, {
  uri: 'a1',
  name: 'Fieldlight',
  kind: 'artist',
  sub: 'Artist',
  art: false
}, {
  uri: 'a2',
  name: 'The Analog Hearts',
  kind: 'artist',
  sub: 'Artist',
  art: false
}, {
  uri: 'al1',
  name: 'Harvest',
  kind: 'album',
  sub: 'Album \u00b7 Fieldlight',
  art: true
}, {
  uri: 'al2',
  name: 'Night Signals',
  kind: 'album',
  sub: 'Album \u00b7 The Analog Hearts',
  art: false
}, {
  uri: 'p1',
  name: 'Evening Warmth',
  kind: 'playlist',
  sub: 'Playlist \u00b7 24 tracks',
  art: false
}];
const TYPE_LABEL: Record<string, string> = {
  track: 'Tracks',
  artist: 'Artists',
  album: 'Albums',
  playlist: 'Playlists'
};
const FILTERS: [string, string][] = [['all', 'All'], ['track', 'Tracks'], ['artist', 'Artists'], ['album', 'Albums'], ['playlist', 'Playlists'], ['radio', 'Radio']];
function SearchDrawer({
  connected,
  onConnect,
  onDisconnect,
  members,
  onClose
}: {
  connected: boolean;
  onConnect: () => void;
  onDisconnect: () => void;
  members: number;
  onClose: () => void;
}) {
  useDrawer('search', true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);
  const [q, setQ] = useState('');
  const [filter, setFilter] = useState('all');
  const [sel, setSel] = useState<string | null>(null);
  const [done, setDone] = useState<{
    uri: string;
    label: string;
  } | null>(null);
  const [recents, setRecents] = useState(['fieldlight', 'morning jazz']);
  const [setupOpen, setSetupOpen] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);
  const play = (item: Hit, option: string) => {
    setSel(null);
    setDone({
      uri: item.uri,
      label: option === 'replace' ? 'Playing' : 'Queued'
    });
    setTimeout(() => setDone(d => d && d.uri === item.uri ? null : d), 2600);
    const t = q.trim().toLowerCase();
    if (t && !recents.includes(t)) setRecents(r => [t, ...r].slice(0, 6));
  };
  const actBtn = (item: Hit, label: string, sub: string | null, glyph: React.ReactNode, option: string, solid?: boolean) => <button className={`search-act${solid ? ' solid' : ''}`} onClick={() => play(item, option)}>
      {glyph}
      <span>{label}</span>
      {sub && <span className="search-act-sub">{sub}</span>}
    </button>;
  const row = (item: Hit) => {
    const on = sel === item.uri;
    const ok = done && done.uri === item.uri;
    return <div key={item.uri} className={`search-hit${on ? ' on' : ''}`}>
        <button className="search-row" aria-expanded={on} onClick={() => setSel(on ? null : item.uri)}>
          <span className={`search-art${item.kind === 'artist' ? ' round' : ''}`}>
            {I_NOTE}
            {item.art && <img src={ART} alt="" loading="lazy" />}
          </span>
          <span className="search-meta">
            <span className="search-t">{item.name}</span>
            <span className="search-s dim">{item.sub}</span>
          </span>
          {ok ? <span className="search-ok">
              {I_CHECK}
              {done!.label}
            </span> : <span className="search-go">{I_PLAY}</span>}
        </button>
        {on && <div className="search-acts">
            {actBtn(item, 'Play now', 'replaces queue', I_PLAY, 'replace', true)}
            {actBtn(item, 'Play next', null, I_NEXT, 'next')}
            {actBtn(item, 'Add to queue', null, I_PLUS, 'add')}
          </div>}
      </div>;
  };

  /* The body, by connection state and then by query state - the real drawer's exact ladder. */
  let body: React.ReactNode;
  if (!connected) {
    body = <div className="search-empty">
        {I_SEARCH}
        <div className="search-empty-t">Search needs Music Assistant</div>
        <p className="dim sm">
          Search rides a direct connection to your Music Assistant server. Set it up once and every browser signed into this device
          shares it.
        </p>
        {setupOpen ? <MaPanel connected={false} onConnect={onConnect} onDisconnect={onDisconnect} defaultOpen /> : <button className="btn solid" onClick={() => setSetupOpen(true)}>
            Set up connection
          </button>}
      </div>;
  } else if (!q.trim()) {
    body = <>
        {recents.length > 0 && <div className="search-sec dim">Recent searches</div>}
        {recents.map(r => <div key={r} className="search-rec">
            <button className="search-rec-hit" onClick={() => setQ(r)}>
              {r}
            </button>
            <button className="icon search-rec-x" aria-label="Forget this search" onClick={() => setRecents(recents.filter(x => x !== r))}>
              {I_X}
            </button>
          </div>)}
      </>;
  } else {
    const ql = q.trim().toLowerCase();
    const hits = LIBRARY.filter(h => (filter === 'all' || h.kind === filter) && (h.name.toLowerCase().includes(ql) || h.sub.toLowerCase().includes(ql) || ql.length > 1));
    if (!hits.length) {
      body = <p className="dim sm search-status">{`Nothing found for \u201c${q.trim()}\u201d.`}</p>;
    } else {
      const types = filter === 'all' ? ['track', 'artist', 'album', 'playlist'] : [filter];
      body = types.map(ty => {
        const list = hits.filter(h => h.kind === ty);
        if (!list.length) return null;
        return <React.Fragment key={ty}>
            <div className="search-sec dim">{TYPE_LABEL[ty]}</div>
            {list.slice(0, 3).map(row)}
            {filter === 'all' && list.length > 3 && <button className="search-more" onClick={() => setFilter(ty)}>
                Show more
              </button>}
          </React.Fragment>;
      });
    }
  }
  return <>
      <div className="scrim search-scrim" onClick={onClose} />
      <div className={`search-drawer${connected ? ' tall' : ''}`} style={dragStyle || undefined} {...drag} role="dialog" aria-label="Search Music Assistant">
        <button className="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
        {connected && <>
            <div className="search-field">
              {I_SEARCH}
              <input ref={inputRef} className="search-in" type="text" enterKeyHint="search" value={q} placeholder={'Search Music Assistant\u2026'} aria-label="Search Music Assistant" onInput={e => setQ((e.target as HTMLInputElement).value)} />
              {q && <button className="search-x" aria-label="Clear search" onClick={() => {
            setQ('');
            inputRef.current?.focus();
          }}>
                  {I_X}
                </button>}
            </div>
            <div className="search-pills" role="tablist">
              {FILTERS.map(([id, label]) => <button key={id} className={`npill${filter === id ? ' on' : ''}`} role="tab" aria-selected={filter === id} onClick={() => setFilter(id)}>
                  {label}
                </button>)}
            </div>
          </>}
        <div className="search-body">{body}</div>
        {connected && <div className="search-foot dim">
            <span className="dot ok" />
            <span>
              Plays on <strong>{`${DEVICE.label}${members > 1 ? ` +${members - 1}` : ''}`}</strong> {'\u00b7'} via Music Assistant
            </span>
          </div>}
      </div>
    </>;
}

/* ------------------------------------------------------------------ */
/* The players panel                                                   */
/* ------------------------------------------------------------------ */

type Member = {
  id: string;
  name: string;
  vol: number;
};
function PlayersPanel({
  members,
  setMembers,
  groupVol,
  setGroupVol,
  onClose
}: {
  members: Member[];
  setMembers: React.Dispatch<React.SetStateAction<Member[]>>;
  groupVol: number;
  setGroupVol: (v: number) => void;
  onClose: () => void;
}) {
  useDrawer('players', true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);
  const addables = [{
    id: 'office',
    name: 'Office Satellite'
  }, {
    id: 'bedroom',
    name: 'Bedroom Satellite'
  }].filter(a => !members.some(m => m.id === a.id));
  return <>
      <div className="scrim mpanel-scrim" onClick={onClose} />
      <div className="mpanel" style={{
      ...TINT,
      ...(dragStyle || {})
    }} {...drag} role="dialog" aria-label="Players">
        <button className="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />

        {members.length > 1 && <div className="mvol-row">
            <span className="dim sm">Group volume</span>
            <Vol value={groupVol} label="Group volume" onCommit={setGroupVol} />
          </div>}

        <div className="mgroup">
          {members.map(m => <div className="mgroup-row" key={m.id}>
              <span className="mgroup-name">{m.name}</span>
              <Vol value={m.vol} label={`${m.name} volume`} onCommit={v => setMembers(ms => ms.map(x => x.id === m.id ? {
            ...x,
            vol: v
          } : x))} />
              {members.length > 1 && <button className="icon mgroup-x" aria-label={`Remove ${m.name} from the group`} title={`Remove ${m.name} from the group`} onClick={() => setMembers(ms => ms.filter(x => x.id !== m.id))}>
                  {I_MINUS}
                </button>}
            </div>)}
          {addables.length > 0 && <div className="mgroup-adds">
              <div className="dim sm">Add a speaker</div>
              {addables.map(a => <button key={a.id} className="mgroup-addrow" onClick={() => setMembers(ms => [...ms, {
            id: a.id,
            name: a.name,
            vol: 40
          }])}>
                  <span className="mgroup-glyph">{I_PLUS}</span>
                  <span>{a.name}</span>
                </button>)}
            </div>}
        </div>
      </div>
    </>;
}

/* ------------------------------------------------------------------ */
/* The expanded view                                                   */
/* ------------------------------------------------------------------ */

function MediaSheet({
  track,
  playing,
  pos,
  onSeek,
  onPlayPause,
  onSkip,
  members,
  onPlayers,
  maConnected,
  onMaConnect,
  onMaDisconnect,
  onClose
}: {
  track: (typeof TRACKS)[0];
  playing: boolean;
  pos: number;
  onSeek: (v: number) => void;
  onPlayPause: () => void;
  onSkip: (dir: 1 | -1) => void;
  members: Member[];
  onPlayers: () => void;
  maConnected: boolean;
  onMaConnect: () => void;
  onMaDisconnect: () => void;
  onClose: () => void;
}) {
  useDrawer('media', true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);
  const [shuffle, setShuffle] = useState(false);
  const [repeat, setRepeat] = useState(0); // 0 off, 2 all, 1 one - cycled in that order

  const chip = members.length ? members[0].name + (members.length > 1 ? ` +${members.length - 1}` : '') : 'Players';
  return <>
      <div className="scrim msheet-scrim" onClick={onClose} />
      <div className="msheet" style={{
      ...TINT,
      ...(dragStyle || {})
    }} {...drag}>
        <button className="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />

        <div className="msheet-body">
          <Artwork big />

          <div className="mmeta">
            <div className="mmeta-t">{track.title}</div>
            <div className="mmeta-a dim">{track.artist}</div>
            <div className="mmeta-al dim">{track.album}</div>
            <div className="dim xs">{playing ? 'Playing' : 'Paused'} &middot; Music Assistant</div>
          </div>

          <SeekBar pos={pos} dur={track.dur} onSeek={onSeek} />

          <div className="mrow">
            <button className={`mbtn${shuffle ? ' on' : ''}`} aria-label="Shuffle" aria-pressed={shuffle} onClick={() => setShuffle(!shuffle)}>
              {I_SHUFFLE}
            </button>
            <button className="mbtn" aria-label="Previous track" onClick={() => onSkip(-1)}>
              {I_PREV}
            </button>
            <button className="mplay" aria-label={playing ? 'Pause' : 'Play'} onClick={onPlayPause}>
              {playing ? I_PAUSE : I_PLAY}
            </button>
            <button className="mbtn" aria-label="Next track" onClick={() => onSkip(1)}>
              {I_NEXT}
            </button>
            <button className={`mbtn${repeat !== 0 ? ' on' : ''}`} aria-label="Repeat" onClick={() => setRepeat(repeat === 0 ? 2 : repeat === 2 ? 1 : 0)}>
              {I_REPEAT}
              {repeat === 1 && <span className="mrpt1 num">1</span>}
            </button>
          </div>

          <button className="mchip" onClick={onPlayers}>
            {I_SPK}
            <span>{chip}</span>
          </button>

          <MaPanel connected={maConnected} onConnect={onMaConnect} onDisconnect={onMaDisconnect} />
        </div>
      </div>
    </>;
}

/* ------------------------------------------------------------------ */
/* The bar                                                             */
/* ------------------------------------------------------------------ */

export function MediaFooter({
  search,
  onSearchClose
}: {
  search?: boolean;
  onSearchClose?: () => void;
}) {
  const [playing, setPlaying] = useState(true);
  const [vol, setVol] = useState(38);
  const [open, setOpen] = useState(false);
  const [panel, setPanel] = useState(false);
  const [trackIdx, setTrackIdx] = useState(0);
  const [pos, setPos] = useState(74000);
  const [maConnected, setMaConnected] = useState(true);
  const [members, setMembers] = useState<Member[]>([{
    id: 'living',
    name: 'Living Room Satellite',
    vol: 38
  }, {
    id: 'kitchen',
    name: 'Kitchen Satellite',
    vol: 52
  }]);
  const track = TRACKS[trackIdx];

  // The playhead: a second per second while playing, wrapping into the next track.
  useEffect(() => {
    if (!playing) return;
    const t = setInterval(() => setPos(p => {
      if (p + 1000 >= track.dur) {
        setTrackIdx(i => (i + 1) % TRACKS.length);
        return 0;
      }
      return p + 1000;
    }), 1000);
    return () => clearInterval(t);
  }, [playing, track.dur]);
  const skip = (dir: 1 | -1) => {
    setTrackIdx(i => (i + dir + TRACKS.length) % TRACKS.length);
    setPos(0);
  };
  const searchEl = search ? <SearchDrawer connected={maConnected} onConnect={() => setMaConnected(true)} onDisconnect={() => setMaConnected(false)} members={members.length} onClose={() => onSearchClose && onSearchClose()} /> : null;
  return <>
      <div className="mbar" style={TINT} onClick={() => setOpen(true)}>
        <div className="mbar-row">
          <Artwork />
          <button className="mbar-meta" aria-label="Open media view" aria-expanded={open}>
            <div className="mbar-t">
              <span>{track.title}</span>
            </div>
            <div className="mbar-s dim">{track.artist} &middot; Music Assistant</div>
          </button>
          <button className="mbtn mspk" aria-label="Players" onClick={e => {
          e.stopPropagation();
          setPanel(true);
        }}>
            {I_SPK}
            {members.length > 1 && <span className="mbdg num">{members.length}</span>}
          </button>
          <button className="mplay sm" aria-label={playing ? 'Pause' : 'Play'} onClick={e => {
          e.stopPropagation();
          setPlaying(!playing);
        }}>
            {playing ? I_PAUSE : I_PLAY}
          </button>
        </div>
        <div className="mbar-vol" onClick={e => e.stopPropagation()}>
          {I_VOL}
          <Vol value={vol} onCommit={setVol} />
        </div>
      </div>

      {open && <MediaSheet track={track} playing={playing} pos={pos} onSeek={setPos} onPlayPause={() => setPlaying(!playing)} onSkip={skip} members={members} onPlayers={() => setPanel(true)} maConnected={maConnected} onMaConnect={() => setMaConnected(true)} onMaDisconnect={() => setMaConnected(false)} onClose={() => setOpen(false)} />}
      {panel && <PlayersPanel members={members} setMembers={setMembers} groupVol={vol} setGroupVol={setVol} onClose={() => setPanel(false)} />}
      {searchEl}
    </>;
}