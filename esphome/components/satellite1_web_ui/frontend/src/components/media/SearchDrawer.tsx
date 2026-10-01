/**
 * Search, over this browser's Music Assistant socket only, by design: the Home Assistant relay
 * speaks the fixed verbs the firmware bakes in, and teaching it search would cost firmware bytes
 * plus a device round trip per keystroke, against the socket tier's whole point of costing the
 * firmware nothing. Without a configured connection the drawer offers the same MaPanel as Now
 * Playing. A pick plays on the device's active queue, which Music Assistant redirects to the group
 * leader's whenever this speaker is grouped, so playing on the group costs nothing extra.
 *
 * Picks go out as player_queues/play_media { queue_id, media: uri, option }, with option replace,
 * next or add (verified against the MA frontend @ 367878c, September 2026). "Play now" sends
 * replace rather than play on purpose: its sub-label promises "replaces queue", and that is the
 * common intent of picking an album by name - predictable beats clever.
 */
import type { ReactNode } from 'react';
import { Fragment, useEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { TEXT } from '../../copy.js';
import { maHttpBase } from '../../lib/ma.js';
import { toast } from '../../lib/toast.js';
import { ALL_SHOWN, SEARCH_TYPES, addRecent, artThumb, groupLabel, searchArgs, searchGroups, subOf } from '../../lib/media.js';
import { MaPanel } from './MaPanel';
import type { Tiers } from './model';
import { I_NEXT, I_PLAY, I_PLUS, dragHandle, mi } from './parts';

/** The pause between the last keystroke and the search, the same order of magnitude as MA's own
 *  modal; Enter skips it. */
const DEBOUNCE_MS = 400;
/** How long a row's confirmation check stands before the row returns to normal. */
const DONE_MS = 2500;
/** The ceiling on a play command's spinner: a socket that silently swallows the answer must not
 *  spin the button forever. */
const ACT_TIMEOUT_MS = 10000;
/** Recent searches stay in this browser's localStorage only, guarded like every localStorage touch
 *  in the app: losing the memory is survivable. */
const KEY_RECENT = 'sat1.ma.recent';
const RECENT_MAX = 8;

const TYPE_LABEL: Record<string, string> = {
  track: TEXT.search_track,
  artist: TEXT.search_artist,
  album: TEXT.search_album,
  playlist: TEXT.search_playlist,
  radio: TEXT.search_radio,
  podcast: TEXT.search_podcast,
  audiobook: TEXT.search_audiobook
};
const FILTERS: [string, string][] = [['all', TEXT.search_all], ...SEARCH_TYPES.map((t: string) => [t, TYPE_LABEL[t]] as [string, string])];

const I_SEARCH_SM = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" aria-hidden="true"><circle cx="7" cy="7" r="4.4" /><path d="M10.4 10.4 14 14" /></svg>;
const I_X_SM = <svg className="mi" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" aria-hidden="true"><path d="M2.8 2.8l6.4 6.4M9.2 2.8l-6.4 6.4" /></svg>;
const I_CHECK_SM = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M3.2 8.6 6.4 11.8 12.8 4.8" /></svg>;
const I_NOTE_SM = <svg className="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" aria-hidden="true"><path d="M6 13V4l7-1.5V11" /><circle cx="4.5" cy="13" r="1.5" /><circle cx="11.5" cy="11" r="1.5" /></svg>;

function readRecents(): string[] {
  try {
    const v = JSON.parse(localStorage.getItem(KEY_RECENT) || '[]');
    return Array.isArray(v) ? v.filter(s => typeof s === 'string') : [];
  } catch {
    return [];
  }
}
function writeRecents(list: string[]) {
  try {
    localStorage.setItem(KEY_RECENT, JSON.stringify(list));
  } catch {
    /* not remembered; searching still works */
  }
}

/**
 * Debounced and sequenced: only the newest request's answer lands, so results never step back to
 * an older keystroke's. A failed search keeps the previous results and says so in a toast.
 */
function useMaSearch(cmd: (c: string, a: object) => Promise<any>, ready: boolean, q: string, filter: string) {
  const [res, setRes] = useState<{ q: string; r: any } | null>(null);
  const [busy, setBusy] = useState(false);
  const seq = useRef(0);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const run = (query: string, f: string) => {
    const my = ++seq.current;
    setBusy(true);
    cmd('music/search', searchArgs(query, f)).then(r => {
      if (seq.current !== my) return;
      setRes({ q: query, r: r || {} });
      setBusy(false);
    }).catch(() => {
      if (seq.current !== my) return;
      setBusy(false);
      toast({ kind: 'err', key: 'search', ttl: 6000, title: TEXT.ma_error });
    });
  };
  useEffect(() => {
    if (timer.current) clearTimeout(timer.current);
    const query = q.trim();
    if (!ready || query.length < 2) {
      // Invalidate anything in flight, so its late answer cannot paint over the recents view.
      seq.current++;
      setRes(null);
      setBusy(false);
      return undefined;
    }
    setBusy(true);
    timer.current = setTimeout(() => run(query, filter), DEBOUNCE_MS);
    return () => clearTimeout(timer.current!);
    // `cmd` is a new closure each render on the same socket; the query and filter are the inputs.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [q, filter, ready]);
  return {
    res,
    busy,
    runNow: () => {
      if (timer.current) clearTimeout(timer.current);
      const query = q.trim();
      if (ready && query.length >= 2) run(query, filter);
    }
  };
}

export function SearchDrawer({
  tiers,
  ip,
  onClose
}: {
  tiers: Tiers;
  ip?: string;
  onClose: () => void;
}) {
  const { ws, wsOn, maCfg, members } = tiers;
  const [q, setQ] = useState('');
  const [filter, setFilter] = useState('all');
  const [recents, setRecents] = useState(readRecents);
  const [sel, setSel] = useState<string | null>(null);
  const [acting, setActing] = useState<{ uri: string; option: string } | null>(null);
  const [done, setDone] = useState<{ uri: string; label: string } | null>(null);
  const [setupOpen, setSetupOpen] = useState(false);
  const configured = !!(maCfg.url && maCfg.token);
  const httpBase = maHttpBase(maCfg.url);
  const { res, busy, runNow } = useMaSearch(ws.cmd, wsOn, q, filter);
  const inputRef = useRef<HTMLInputElement>(null);
  const drag = useRef<number | null>(null);
  useEffect(() => {
    if (wsOn) inputRef.current?.focus();
  }, [wsOn]);

  // The player id stands in until the queue answer lands.
  const queueId = ws.queue?.queue_id || ws.me?.player_id || '';
  const saveRecents = (list: string[]) => {
    setRecents(list);
    writeRecents(list);
  };

  const play = (item: any, option: string) => {
    if (!queueId || acting) return;
    setActing({ uri: item.uri, option });
    const ceiling = setTimeout(() => setActing(a => a && a.uri === item.uri ? null : a), ACT_TIMEOUT_MS);
    ws.cmd('player_queues/play_media', { queue_id: queueId, media: item.uri, option }).then(() => {
      clearTimeout(ceiling);
      setActing(null);
      setSel(null);
      setDone({ uri: item.uri, label: option === 'replace' ? TEXT.search_playing : TEXT.search_queued });
      setTimeout(() => setDone(d => d && d.uri === item.uri ? null : d), DONE_MS);
      // A search someone played from is a search worth remembering.
      saveRecents(addRecent(recents, q, RECENT_MAX));
    }).catch(() => {
      clearTimeout(ceiling);
      setActing(null);
      toast({ kind: 'err', key: 'search-play', ttl: 6000, title: TEXT.search_play_failed, sub: TEXT.search_play_failed_sub });
    });
  };

  const actBtn = (item: any, label: string, sub: string | null, glyph: ReactNode, option: string, solid?: boolean) => <button className={`search-act${solid ? ' solid' : ''}`} disabled={!queueId || !!acting} onClick={() => play(item, option)}>{acting && acting.uri === item.uri && acting.option === option ? <span className="search-spin" /> : glyph}<span>{label}</span>{sub && <span className="search-act-sub">{sub}</span>}</button>;
  const row = (item: any) => {
    const on = sel === item.uri;
    const ok = done && done.uri === item.uri;
    const art = artThumb(item, httpBase);
    return <div key={item.uri} className={`search-hit${on ? ' on' : ''}`}>
      <button className="search-row" aria-expanded={on} onClick={() => setSel(on ? null : item.uri)}>
        <span className={`search-art${item.media_type === 'artist' ? ' round' : ''}`}>
          {I_NOTE_SM}
          {art && <img key={art} src={art} alt="" loading="lazy" onError={e => {
            e.currentTarget.style.display = 'none';
          }} />}
        </span>
        <span className="search-meta"><span className="search-t">{item.name}</span><span className="search-s dim">{subOf(item, TEXT.search_kind)}</span></span>
        {ok ? <span className="search-ok">{I_CHECK_SM}<span>{done!.label}</span></span> : <span className="search-go">{mi(I_PLAY)}</span>}
      </button>
      {on && <div className="search-acts">
        {actBtn(item, TEXT.search_play_now, TEXT.search_play_now_sub, mi(I_PLAY), 'replace', true)}
        {actBtn(item, TEXT.search_play_next, null, mi(I_NEXT), 'next')}
        {actBtn(item, TEXT.search_add, null, mi(I_PLUS), 'add')}
      </div>}
    </div>;
  };

  const setup = <MaPanel tiers={tiers} ip={ip} defaultOpen />;
  const status = (text: string) => <div className="search-status"><span className="search-spin" /><div className="dim sm">{text}</div></div>;
  let body: ReactNode = null;
  if (!configured) {
    // The drawer says what it needs and offers the setup where it stands; the panel unfolds on the
    // button rather than greeting everyone with a token field.
    body = <div className="search-empty">{I_SEARCH_SM}<div className="search-empty-t">{TEXT.search_need_ma_t}</div><p className="dim sm">{TEXT.search_need_ma_b}</p>{setupOpen ? setup : <button className="btn solid primary" onClick={() => setSetupOpen(true)}>{TEXT.search_setup_btn}</button>}</div>;
  } else if (!wsOn) {
    // A refused token gets the panel's own error line and the panel itself, so the fix is where the
    // failure is.
    body = ws.status === 'error' ? <div className="search-empty"><p className="dim sm">{TEXT.ma_error}</p>{setup}</div> : status(TEXT.search_connecting);
  } else if (q.trim().length < 2) {
    body = <>
      {recents.length > 0 && <div className="search-sec dim">{TEXT.search_recent}</div>}
      {recents.map(r => <div key={r} className="search-rec"><button className="search-rec-hit" onClick={() => setQ(r)}>{r}</button><button className="icon search-rec-x" aria-label={TEXT.search_forget} onClick={() => saveRecents(recents.filter(x => x !== r))}>{I_X_SM}</button></div>)}
    </>;
  } else if (busy && !res) {
    body = status(TEXT.search_searching);
  } else if (res) {
    const groups = searchGroups(res.r, filter);
    body = groups.length === 0 && !busy ? <p className="dim sm search-status">{TEXT.search_none.replace('%s', res.q)}</p> : groups.map(([ty, items]: [string, any[]]) => <Fragment key={ty}>
        <div className="search-sec dim">{TYPE_LABEL[ty]}</div>
        {(filter === 'all' ? items.slice(0, ALL_SHOWN) : items).map(row)}
        {filter === 'all' && items.length > ALL_SHOWN && <button className="search-more" onClick={() => setFilter(ty)}>{TEXT.search_more}</button>}
      </Fragment>);
  }
  const target = groupLabel(members, ws.me?.name || '');

  return createPortal([<div key="scrim" className="scrim search-scrim" onClick={onClose} />, <div key="drawer" className={`search-drawer${wsOn ? ' tall' : ''}`} role="dialog" aria-label={TEXT.search_title}>
    <div className="mpanel-handle" {...dragHandle(drag, '.search-drawer', onClose, '')} />
    {wsOn && <div className="search-field">
      {I_SEARCH_SM}
      <input ref={inputRef} className="search-in" type="text" enterKeyHint="search" value={q} placeholder={TEXT.search_ph} aria-label={TEXT.search_title} onInput={e => setQ(e.currentTarget.value)} onKeyDown={e => {
        if (e.key === 'Enter') runNow();
      }} />
      {q && <button className="search-x" aria-label={TEXT.search_clear} onClick={() => {
        setQ('');
        inputRef.current?.focus();
      }}>{I_X_SM}</button>}
    </div>}
    {wsOn && <div className="search-pills" role="tablist">
      {FILTERS.map(([id, label]) => <button key={id} className={`npill${filter === id ? ' on' : ''}`} role="tab" aria-selected={filter === id} onClick={() => setFilter(id)}>{label}</button>)}
    </div>}
    <div className="search-body">{body}</div>
    {wsOn && target && <div className="search-foot dim"><span className="dot ok" /><span>{TEXT.search_target} <strong>{target}</strong> {'\u00b7'} {TEXT.search_via}</span></div>}
  </div>], document.body);
}
