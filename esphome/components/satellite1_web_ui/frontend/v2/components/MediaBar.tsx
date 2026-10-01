import React, { useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import type { Ctx } from '../ctx';
const ART = 'https://storage.googleapis.com/storage.magicpath.ai/component-assets/454802455327313920/454809714925146112/ecc89c59771f06b4c2fa7960733c67a3eb86e4e4fb31599723751ffb6e85f432.png';
const TRACKS = [{
  id: 't1',
  title: 'Amber Skies',
  artist: 'Fieldlight',
  album: 'Harvest',
  dur: 254000
}, {
  id: 't2',
  title: 'Low September Sun',
  artist: 'Fieldlight',
  album: 'Harvest',
  dur: 228000
}, {
  id: 't3',
  title: 'Porchlight',
  artist: 'Fieldlight',
  album: 'Harvest',
  dur: 201000
}];
const SATELLITES = [{
  id: 'living',
  name: 'Living Room Satellite'
}, {
  id: 'kitchen',
  name: 'Kitchen Satellite'
}, {
  id: 'office',
  name: 'Office Satellite'
}, {
  id: 'bedroom',
  name: 'Bedroom Satellite'
}];
const INITIAL_MEMBERS = [{
  id: 'living',
  name: 'Living Room Satellite',
  vol: 38
}, {
  id: 'kitchen',
  name: 'Kitchen Satellite',
  vol: 52
}];
interface Member {
  id: string;
  name: string;
  vol: number;
}
const Svg = ({
  children,
  size = 16
}: {
  children: React.ReactNode;
  size?: number;
}) => <svg width={size} height={size} viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{children}</svg>;
const I_PREV = <path d="M3.5 4v8M5 8l7-4v8L5 8z" />;
const I_NEXT = <path d="M12.5 4v8M11 8L4 4v8l7-4z" />;
const I_SHUFFLE = <g><path d="M1.5 4.5h2.6c3.6 0 4.2 7 7.8 7h2.1M1.5 11.5h2.6c1.3 0 2.2-.9 2.9-2M14 4.5h-2.1c-1.4 0-2.3 1-3 2.1" /><path d="M12.2 2.7l1.8 1.8-1.8 1.8M12.2 9.7l1.8 1.8-1.8 1.8" /></g>;
const I_REPEAT = <g><path d="M2.5 6.5v-.4A2.6 2.6 0 0 1 5.1 3.5h7.4M13.5 9.5v.4a2.6 2.6 0 0 1-2.6 2.6H3.5" /><path d="M10.8 1.8 12.6 3.5l-1.8 1.7M5.2 14.2 3.4 12.5l1.8-1.7" /></g>;
const I_SPK = <g><path d="M9 4.5 5.5 7H3a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h2.5L9 13.5V4.5z" /><path d="M12 6.5a3.5 3.5 0 0 1 0 5" /></g>;
const I_VOL = <g><path d="M9 4.5 5.5 7H3a.5.5 0 0 0-.5.5v3a.5.5 0 0 0 .5.5h2.5L9 13.5V4.5z" /><path d="M11.5 7.5a2 2 0 0 1 0 3" /></g>;
const I_SPK_BOX = <g><rect x="3.5" y="1.75" width="9" height="12.5" rx="2" /><circle cx="8" cy="4.6" r=".6" fill="currentColor" stroke="none" /><circle cx="8" cy="9.6" r="2.4" /></g>;
const I_PLUS = <path d="M8 4v8M4 8h8" />;
const I_MINUS = <path d="M4 8h8" />;
const I_PAUSE = <path d="M5.5 4v8M10.5 4v8" />;
const I_PLAY = <path d="M5 4l8 4-8 4V4z" />;
const fmt = (ms: number) => {
  const s = Math.floor(ms / 1000);
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
};
const I_SEARCH_SM = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" aria-hidden="true"><circle cx="7" cy="7" r="4.4" /><path d="M10.4 10.4 14 14" /></svg>;
const I_X_SM = <svg className="mi" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" aria-hidden="true"><path d="M2.8 2.8l6.4 6.4M9.2 2.8l-6.4 6.4" /></svg>;
const I_CHECK_SM = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3.2 8.6 6.4 11.8 12.8 4.8" /></svg>;
const mi = (p: React.ReactNode) => <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{p}</svg>;
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
  sub: 'Track · Fieldlight',
  art: true
}, {
  uri: 't2',
  name: 'Low September Sun',
  kind: 'track',
  sub: 'Track · Fieldlight',
  art: true
}, {
  uri: 't3',
  name: 'Harvest Wind',
  kind: 'track',
  sub: 'Track · The Analog Hearts',
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
  sub: 'Album · Fieldlight',
  art: true
}, {
  uri: 'al2',
  name: 'Night Signals',
  kind: 'album',
  sub: 'Album · The Analog Hearts',
  art: false
}, {
  uri: 'p1',
  name: 'Evening Warmth',
  kind: 'playlist',
  sub: 'Playlist · 24 tracks',
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
  const [q, setQ] = useState('');
  const [filter, setFilter] = useState('all');
  const [sel, setSel] = useState<string | null>(null);
  const [done, setDone] = useState<{
    uri: string;
    label: string;
  } | null>(null);
  const [recents, setRecents] = useState(['fieldlight', 'morning jazz']);
  const inputRef = useRef<HTMLInputElement>(null);
  const searchPanelRef = useRef<HTMLDivElement>(null);
  const searchDragStartY = useRef<number | null>(null);
  useEffect(() => {
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, []);
  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', handler);
    return () => window.removeEventListener('keydown', handler);
  }, [onClose]);
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
  const actBtn = (item: Hit, label: string, sub: string | null, glyph: React.ReactNode, option: string, solid?: boolean) => <button className={`search-act${solid ? ' solid' : ''}`} onClick={() => play(item, option)}>{glyph}<span>{label}</span>{sub && <span className="search-act-sub">{sub}</span>}</button>;
  const row = (item: Hit) => {
    const on = sel === item.uri;
    const ok = done && done.uri === item.uri;
    return <div key={item.uri} className={`search-hit${on ? ' on' : ''}`}>
      <button className="search-row" aria-expanded={on} onClick={() => setSel(on ? null : item.uri)}>
        <span className={`search-art${item.kind === 'artist' ? ' round' : ''}`}>
          <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" aria-hidden="true"><path d="M6 13V4l7-1.5V11" /><circle cx="4.5" cy="13" r="1.5" /><circle cx="11.5" cy="11" r="1.5" /></svg>
          {item.art && <img src={ART} alt="" loading="lazy" />}
        </span>
        <span className="search-meta"><span className="search-t">{item.name}</span><span className="search-s dim">{item.sub}</span></span>
        {ok ? <span className="search-ok">{I_CHECK_SM}<span>{done!.label}</span></span> : <span className="search-go">{mi(I_PLAY)}</span>}
      </button>
      {on && <div className="search-acts">
        {actBtn(item, 'Play now', 'replaces queue', mi(I_PLAY), 'replace', true)}
        {actBtn(item, 'Play next', null, mi(I_NEXT), 'next')}
        {actBtn(item, 'Add to queue', null, mi(I_PLUS), 'add')}
      </div>}
    </div>;
  };
  let body: React.ReactNode;
  if (!connected) {
    body = <div className="search-empty">{I_SEARCH_SM}<div className="search-empty-t">Search needs Music Assistant</div><p className="dim sm">Search rides a direct connection to your Music Assistant server. Set it up once and every browser signed into this device shares it.</p><button className="btn solid primary" onClick={onConnect}>Set up connection</button></div>;
  } else if (!q.trim()) {
    body = <div style={{
      display: 'contents'
    }}>
      {recents.length > 0 && <div className="search-sec dim">Recent searches</div>}
      {recents.map(r => <div key={r} className="search-rec"><button className="search-rec-hit" onClick={() => setQ(r)}>{r}</button><button className="icon search-rec-x" aria-label="Forget this search" onClick={() => setRecents(recents.filter(x => x !== r))}>{I_X_SM}</button></div>)}
    </div>;
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
          {filter === 'all' && list.length > 3 && <button className="search-more" onClick={() => setFilter(ty)}>Show more</button>}
        </React.Fragment>;
      });
    }
  }
  return <div>
    <div className="scrim search-scrim" onClick={onClose} />
    <div ref={searchPanelRef} className={`search-drawer${connected ? ' tall' : ''}`} role="dialog" aria-label="Search Music Assistant">
      <div className="mpanel-handle" role="button" aria-label="Close" style={{
        touchAction: 'none',
        cursor: 'grab'
      }} onPointerDown={e => {
        if (window.innerWidth >= 1024) return;
        searchDragStartY.current = e.clientY;
        e.currentTarget.setPointerCapture(e.pointerId);
        if (searchPanelRef.current) searchPanelRef.current.style.transition = 'none';
      }} onPointerMove={e => {
        if (searchDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - searchDragStartY.current);
        if (searchPanelRef.current) {
          searchPanelRef.current.style.transform = `translateY(${dy}px)`;
          searchPanelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
        }
      }} onPointerUp={e => {
        if (searchDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - searchDragStartY.current);
        if (dy > 80) {
          if (searchPanelRef.current) {
            searchPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
            searchPanelRef.current.style.transform = 'translateY(120%)';
            searchPanelRef.current.style.opacity = '0';
            setTimeout(onClose, 210);
          } else onClose();
        } else if (searchPanelRef.current) {
          searchPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          searchPanelRef.current.style.transform = 'translateY(0)';
          searchPanelRef.current.style.opacity = '1';
        }
        searchDragStartY.current = null;
        setTimeout(() => {
          if (searchPanelRef.current) {
            searchPanelRef.current.style.transition = '';
            searchPanelRef.current.style.transform = '';
            searchPanelRef.current.style.opacity = '';
          }
        }, 250);
      }} />
      
      {connected && <div className="search-field">
        {I_SEARCH_SM}
        <input ref={inputRef} className="search-in" type="text" enterKeyHint="search" value={q} placeholder={'Search Music Assistant\u2026'} aria-label="Search Music Assistant" onChange={e => setQ(e.target.value)} />
        {q && <button className="search-x" aria-label="Clear search" onClick={() => {
          setQ('');
          inputRef.current?.focus();
        }}>{I_X_SM}</button>}
      </div>}
      {connected && <div className="search-pills" role="tablist">
        {FILTERS.map(([id, label]) => <button key={id} className={`npill${filter === id ? ' on' : ''}`} role="tab" aria-selected={filter === id} onClick={() => setFilter(id)}>{label}</button>)}
      </div>}
      <div className="search-body">{body}</div>
      {connected && <div className="search-foot dim"><span className="dot ok" /><span>Plays on <strong>{`Living Room Satellite${members > 1 ? ` +${members - 1}` : ''}`}</strong> {'\u00b7'} via Music Assistant</span></div>}
    </div>
  </div>;
}
export function MediaBar({
  ctx,
  playing,
  setPlaying
}: {
  ctx: Ctx;
  playing: boolean;
  setPlaying: (v: boolean) => void;
}) {
  const [members, setMembers] = useState<Member[]>(INITIAL_MEMBERS);
  const [groupVol, setGroupVol] = useState(45);
  const [maConnected, setMaConnected] = useState(true);
  const [maOpen, setMaOpen] = useState(false);
  const [maUrl, setMaUrl] = useState('http://homeassistant.local:8095');
  const [maToken, setMaToken] = useState('');
  const [trackIdx, setTrackIdx] = useState(0);
  const [pos, setPos] = useState(74000);
  const [panel, setPanel] = useState(false);
  const [open, setOpen] = useState(false);
  useEffect(() => {
    if (!(open || panel)) return;
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, [open, panel]);
  const sheetDragStartY = useRef<number | null>(null);
  const panelDragStartY = useRef<number | null>(null);
  const dragHandle = (ref: React.MutableRefObject<number | null>, sel: string, dismiss: () => void) => ({
    role: 'button',
    'aria-label': 'Close',
    style: {
      touchAction: 'none' as const,
      cursor: 'grab'
    },
    onPointerDown: (e: React.PointerEvent<HTMLDivElement>) => {
      if (window.innerWidth >= 1024) return;
      ref.current = e.clientY;
      e.currentTarget.setPointerCapture(e.pointerId);
      const s = e.currentTarget.closest(sel) as HTMLElement | null;
      if (s) s.style.transition = 'none';
    },
    onPointerMove: (e: React.PointerEvent<HTMLDivElement>) => {
      if (ref.current === null) return;
      const dy = Math.max(0, e.clientY - ref.current);
      const s = e.currentTarget.closest(sel) as HTMLElement | null;
      if (s) {
        s.style.transform = `translateX(-50%) translateY(${dy}px)`;
        s.style.opacity = String(Math.max(0, 1 - dy / 240));
      }
    },
    onPointerUp: (e: React.PointerEvent<HTMLDivElement>) => {
      if (ref.current === null) return;
      const dy = Math.max(0, e.clientY - ref.current);
      const s = e.currentTarget.closest(sel) as HTMLElement | null;
      if (dy > 90) {
        if (s) {
          s.style.transition = 'transform .22s ease,opacity .22s ease';
          s.style.transform = 'translateX(-50%) translateY(120%)';
          s.style.opacity = '0';
          setTimeout(dismiss, 210);
        } else dismiss();
      } else {
        if (s) {
          s.style.transition = 'transform .22s ease,opacity .22s ease';
          s.style.transform = 'translateX(-50%)';
          s.style.opacity = '1';
        }
        if (dy < 6) dismiss();
      }
      ref.current = null;
      setTimeout(() => {
        if (s) {
          s.style.transition = '';
          s.style.transform = '';
          s.style.opacity = '';
        }
      }, 250);
    }
  });
  const [search, setSearch] = useState(false);
  const [shuffle, setShuffle] = useState(false);
  const [repeat, setRepeat] = useState<0 | 1 | 2>(0);
  const track = TRACKS[trackIdx];
  useEffect(() => {
    if (!playing) return;
    const id = setInterval(() => {
      setPos(p => {
        const n = p + 1000;
        if (n > TRACKS[trackIdx].dur) {
          setTrackIdx(i => (i + 1) % TRACKS.length);
          return 0;
        }
        return n;
      });
    }, 1000);
    return () => clearInterval(id);
  }, [playing, trackIdx]);
  const skip = (d: number) => {
    setTrackIdx(i => (i + d + TRACKS.length) % TRACKS.length);
    setPos(0);
  };
  const addable = SATELLITES.filter(s => !members.some(m => m.id === s.id));
  const playBtn = (sm: boolean) => <button className={'mplay' + (sm ? ' sm' : '')} aria-label={playing ? 'Pause' : 'Play'} onClick={e => {
    e.stopPropagation();
    setPlaying(!playing);
  }}><Svg size={sm ? 16 : 22}>{playing ? I_PAUSE : I_PLAY}</Svg></button>;
  return <div className="mroot">
    <div className="mbar" onClick={() => setOpen(true)}>
      <div className="mbar-row">
        <div className="mbar-art"><img src={ART} alt={`${track.title} album artwork`} /></div>
        <button className="mbar-meta" aria-label="Open media view"><b>{track.title}</b><small>{track.artist} · Music Assistant</small></button>
        <button className="mbtn mspk" aria-label="Speakers" onClick={e => {
          e.stopPropagation();
          setPanel(true);
        }}><span style={{
            position: 'relative',
            display: 'inline-flex',
            color: '#fff'
          }}><Svg size={24}>{I_SPK_BOX}</Svg><span aria-hidden="true" style={{
              position: 'absolute',
              top: -3,
              right: -4,
              minWidth: members.length > 1 ? 12 : 8,
              height: members.length > 1 ? 12 : 8,
              padding: members.length > 1 ? '0 2px' : 0,
              borderRadius: 999,
              background: 'var(--orb-a, var(--accent))',
              color: '#fff',
              fontSize: 8,
              fontWeight: 700,
              lineHeight: '12px',
              textAlign: 'center',
              fontStyle: 'normal'
            }}>{members.length > 1 ? members.length : ''}</span></span></button>
        {playBtn(true)}
      </div>
      <div className="mbar-vol" onClick={e => e.stopPropagation()}><span style={{
          display: 'inline-flex',
          color: '#fff'
        }}><Svg size={24}>{I_VOL}</Svg></span><input type="range" aria-label="Volume" value={groupVol} style={{
          '--pct': `${groupVol}%`
        } as React.CSSProperties} onChange={e => setGroupVol(+e.target.value)} /></div>
    </div>

    {open && createPortal([<div key="scrim" className="mscrim" onClick={() => setOpen(false)} />, <section key="msheet" className="msheet" onClick={e => e.stopPropagation()} aria-label="Now playing">
      <div className="msheet-top">
        <div className="handle" {...dragHandle(sheetDragStartY, '.msheet', () => setOpen(false))} />
        <button className="mbtn msheet-search" aria-label="Search music" onClick={() => {
          setOpen(false);
          setSearch(true);
        }}><Svg size={18}><path d="m11 11 3 3M6.8 11a4.2 4.2 0 1 1 0-8.4 4.2 4.2 0 0 1 0 8.4Z" /></Svg></button>
      </div>
      <div className="mart"><img src={ART} alt={`${track.title} album artwork`} /></div>
      <div className="mmeta"><h2>{track.title}</h2><p>{track.artist}</p><p>{track.album}</p><small><span className="dot" /> {playing ? 'Playing' : 'Paused'} · Music Assistant</small></div>
      <input className="mseek" type="range" aria-label="Seek" min={0} max={track.dur} step={1000} value={pos} style={{
        '--pct': `${(pos / track.dur * 100).toFixed(1)}%`
      } as React.CSSProperties} onChange={e => setPos(+e.target.value)} />
      <div className="time"><span>{fmt(pos)}</span><span>{fmt(track.dur)}</span></div>
      <div className="mrow">
        <button className={'mbtn' + (shuffle ? ' on' : '')} aria-label="Shuffle" onClick={() => setShuffle(!shuffle)}><Svg>{I_SHUFFLE}</Svg></button>
        <button className="mbtn" aria-label="Previous" onClick={() => skip(-1)}><Svg>{I_PREV}</Svg></button>
        {playBtn(false)}
        <button className="mbtn" aria-label="Next" onClick={() => skip(1)}><Svg>{I_NEXT}</Svg></button>
        <button className={'mbtn' + (repeat ? ' on' : '')} aria-label="Repeat" onClick={() => setRepeat(r => (r + 1) % 3 as 0 | 1 | 2)}><Svg>{I_REPEAT}</Svg>{repeat === 2 && <i>1</i>}</button>
      </div>
      <button className="mchip" onClick={() => setPanel(true)}><Svg>{I_SPK_BOX}</Svg><span>{members[0]?.name ?? 'No speaker'}{members.length > 1 ? ` +${members.length - 1}` : ''}</span></button>
      <div className="mapanel">
        <button className="mapanel-head" onClick={() => setMaOpen(!maOpen)} aria-expanded={maOpen}><span className={'madot' + (maConnected ? ' on' : '')} /><span>Music Assistant</span><svg width="16" height="16" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" style={{
            transform: maOpen ? 'rotate(180deg)' : 'none'
          }} aria-hidden="true"><path d="m5 7 3 3 3-3" /></svg></button>
        {maOpen && <div className="mapanel-body">
          <input aria-label="Music Assistant URL" value={maUrl} onChange={e => setMaUrl(e.target.value)} placeholder="Server URL" />
          <input aria-label="Token" type="password" value={maToken} onChange={e => setMaToken(e.target.value)} placeholder="Token" />
          <div className="mapanel-actions"><button className="primary" onClick={() => setMaConnected(true)}>Connect</button><button className="secondary" onClick={() => setMaConnected(false)}>Disconnect</button></div>
          <small>{maConnected ? 'Connected · 3 players available' : 'Not connected'}</small>
        </div>}
      </div>
    </section>], document.body)}

    {panel && createPortal([<div key="scrim" className="mscrim top" onClick={() => setPanel(false)} />, <section key="mpanel" className="mpanel" onClick={e => e.stopPropagation()} aria-label="Players">
      <div className="handle" {...dragHandle(panelDragStartY, '.mpanel', () => setPanel(false))} />
      <span className="eyebrow">PLAYERS</span>
      {members.length > 1 && <div className="mvol-row group"><span>Group volume</span><input type="range" aria-label="Group volume" value={groupVol} style={{
          '--pct': `${groupVol}%`
        } as React.CSSProperties} onChange={e => setGroupVol(+e.target.value)} /></div>}
      <ul className="mlist">{members.map(m => <li key={m.id}><span>{m.name}</span><div className="mvol-row"><Svg size={14}>{I_VOL}</Svg><input type="range" aria-label={`${m.name} volume`} value={m.vol} style={{
              '--pct': `${m.vol}%`
            } as React.CSSProperties} onChange={e => setMembers(ms => ms.map(x => x.id === m.id ? {
              ...x,
              vol: +e.target.value
            } : x))} />{members.length > 1 && <button className="mbtn" aria-label={`Remove ${m.name}`} onClick={() => setMembers(ms => ms.filter(x => x.id !== m.id))}><Svg>{I_MINUS}</Svg></button>}</div></li>)}</ul>
      {addable.length > 0 && <div className="madd"><span className="eyebrow">ADD A SPEAKER</span>{addable.map(s => <button key={s.id} onClick={() => setMembers(ms => [...ms, {
          id: s.id,
          name: s.name,
          vol: 40
        }])}><Svg>{I_PLUS}</Svg><span>{s.name}</span></button>)}</div>}
    </section>], document.body)}

    {search && <SearchDrawer connected={maConnected} onConnect={() => setMaConnected(true)} onDisconnect={() => setMaConnected(false)} members={members.length} onClose={() => setSearch(false)} />}
  </div>;
}