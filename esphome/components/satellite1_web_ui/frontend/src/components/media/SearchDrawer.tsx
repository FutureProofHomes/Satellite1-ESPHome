/**
 * Search, over this browser's Music Assistant socket only, by design: the Home Assistant relay
 * speaks the fixed verbs the firmware bakes in, and teaching it search would cost firmware bytes
 * plus a device round trip per keystroke, against the socket tier's whole point of costing the
 * firmware nothing. Without a configured connection the drawer offers the same MaPanel as Now
 * Playing. A pick plays on the device's active queue, which Music Assistant redirects to the group
 * leader's whenever this speaker is grouped, so playing on the group costs nothing extra.
 *
 * Picks go out as player_queues/play_media { queue_id, media: uri, option, start_item? }, with
 * option replace, next or add (verified against the MA frontend @ 367878c, September 2026).
 *
 * One set of row rules (owner's request, October 2026). The round play button on any row plays it
 * now, replacing the queue - on a song inside an album or playlist, that album or playlist from
 * that song. A chevron means the row opens: artists to their albums, albums and playlists to their
 * songs, podcasts to their episodes, with Back returning to the results exactly where they were.
 * Rows that do not open show Play next and Add to queue when tapped, and in results lists keep the
 * chevron's space so every play button lines up.
 */
import type { ReactNode } from 'react';
import { Fragment, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { TEXT } from '../../copy.js';
import { maHttpBase } from '../../lib/ma.js';
import { toast } from '../../lib/toast.js';
import { ALL_SHOWN, SEARCH_TYPES, addRecent, albumSections, albumSub, artThumb, canOpen, episodeSub, fmtTime, groupLabel, heroSub, openArgs, openedOrder, searchArgs, searchGroups, subOf } from '../../lib/media.js';
import { Drawer } from '../Drawer';
import { MaPanel } from './MaPanel';
import type { Tiers } from './model';
import { I_NEXT, I_PLAY, I_PLUS, mi } from './parts';

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
/** An opened list's rows per "Show more", so a 2,000-song playlist does not build 2,000 rows. */
const PAGE = 100;

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
const I_CHEV = <path d="M6 3.5 10.5 8 6 12.5" />;
const I_BACK = <path d="M10 3.5 5.5 8 10 12.5" />;

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

/** One opened item: what it holds once loaded (null while loading), where its list was scrolled
 *  when something inside it was opened, and how many rows of it are built. */
type Level = { item: any; items: any[] | null; scroll: number; shown: number };

/** `target` aims the picks at another speaker - opened from its card on the bar - by its queue and
 *  the name the footer says they play on; without it they play here. */
export function SearchDrawer({
  tiers,
  ip,
  target,
  onClose
}: {
  tiers: Tiers;
  ip?: string;
  target?: { queue: string; name: string } | null;
  onClose: () => void;
}) {
  const { ws, wsOn, maCfg, members } = tiers;
  const [q, setQ] = useState('');
  const [filter, setFilter] = useState('all');
  const [recents, setRecents] = useState(readRecents);
  const [sel, setSel] = useState<string | null>(null);
  const [acting, setActing] = useState<{ key: string; option: string } | null>(null);
  const [done, setDone] = useState<{ key: string; label: string } | null>(null);
  const [setupOpen, setSetupOpen] = useState(false);
  const [stack, setStack] = useState<Level[]>([]);
  const configured = !!(maCfg.url && maCfg.token);
  const httpBase = maHttpBase(maCfg.url);
  const { res, busy, runNow } = useMaSearch(ws.cmd, wsOn, q, filter);
  const inputRef = useRef<HTMLInputElement>(null);
  const bodyRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (wsOn) inputRef.current?.focus();
  }, [wsOn]);

  // Back lands where the list was: the results' scroll is kept on the way in, each opened level's
  // on the way further in, and put back once the shorter stack has rendered.
  const rootScroll = useRef(0);
  const restore = useRef<number | null>(null);
  useLayoutEffect(() => {
    if (restore.current != null && bodyRef.current) bodyRef.current.scrollTop = restore.current;
    restore.current = null;
  }, [stack.length]);

  // The player id stands in until the queue answer lands.
  const queueId = target ? target.queue : ws.queue?.queue_id || ws.me?.player_id || '';
  const saveRecents = (list: string[]) => {
    setRecents(list);
    writeRecents(list);
  };

  /** Plays or queues `uri`; `key` is the row whose button rings and then shows the check. */
  const play = (key: string, uri: string, option: string, startItem?: string) => {
    if (!queueId || acting) return;
    setActing({ key, option });
    const ceiling = setTimeout(() => setActing(a => a && a.key === key ? null : a), ACT_TIMEOUT_MS);
    const args: Record<string, string> = { queue_id: queueId, media: uri, option };
    if (startItem) args.start_item = startItem;
    ws.cmd('player_queues/play_media', args).then(() => {
      clearTimeout(ceiling);
      setActing(null);
      setSel(null);
      setDone({ key, label: option === 'replace' ? TEXT.search_playing : TEXT.search_queued });
      setTimeout(() => setDone(d => d && d.key === key ? null : d), DONE_MS);
      // A search someone played from is a search worth remembering.
      if (q.trim()) saveRecents(addRecent(recents, q, RECENT_MAX));
    }).catch(() => {
      clearTimeout(ceiling);
      setActing(null);
      toast({ kind: 'err', key: 'search-play', ttl: 6000, title: TEXT.search_play_failed, sub: TEXT.search_play_failed_sub });
    });
  };

  const open = (item: any) => {
    const call = openArgs(item);
    if (!call) return;
    const y = bodyRef.current?.scrollTop || 0;
    if (!stack.length) rootScroll.current = y;
    restore.current = 0;
    setSel(null);
    setStack(s => [...s.map((l, i) => i === s.length - 1 ? { ...l, scroll: y } : l), { item, items: null, scroll: 0, shown: PAGE }]);
    const uri = item.uri;
    ws.cmd(call[0], call[1]).then(r => {
      setStack(s => s.map(l => l.item.uri === uri && l.items == null ? { ...l, items: openedOrder(item.media_type, r) } : l));
    }).catch(() => {
      setStack(s => s.map(l => l.item.uri === uri && l.items == null ? { ...l, items: [] } : l));
      toast({ kind: 'err', key: 'search-open', ttl: 6000, title: TEXT.ma_error });
    });
  };
  const back = () => {
    setSel(null);
    setStack(s => {
      const next = s.slice(0, -1);
      restore.current = next.length ? next[next.length - 1].scroll : rootScroll.current;
      return next;
    });
  };
  const more = () => setStack(s => s.map((l, i) => i === s.length - 1 ? { ...l, shown: l.shown + PAGE } : l));

  const actBtn = (key: string, uri: string, label: string, glyph: ReactNode, option: string, solid?: boolean, startItem?: string) => <button className={`search-act${solid ? ' solid' : ''}`} disabled={!queueId || !!acting} onClick={() => play(key, uri, option, startItem)}>{acting && acting.key === key && acting.option === option ? <span className="search-spin" /> : glyph}<span>{label}</span></button>;
  const queueActs = (key: string, uri: string) => <div className="search-acts">
    {actBtn(key, uri, TEXT.search_play_next, mi(I_NEXT), 'next')}
    {actBtn(key, uri, TEXT.search_add, mi(I_PLUS), 'add')}
  </div>;
  const thumb = (item: any, round?: boolean) => {
    const art = artThumb(item, httpBase);
    return <span className={`search-art${round ? ' round' : ''}`}>
      {I_NOTE_SM}
      {art && <img key={art} src={art} alt="" loading="lazy" onError={e => {
        e.currentTarget.style.display = 'none';
      }} />}
    </span>;
  };

  /**
   * One row. `lead` is what sits left of the text (art, or a track number), `sub` its second line,
   * `right` anything after the text (a duration). The play button plays `playUri` from
   * `startItem`; openable rows add the chevron, and in results (`gap`) the rest keep its space.
   */
  const row = ({ item, lead, sub, right, playUri, startItem, gap, ep }: { item: any; lead: ReactNode; sub: ReactNode; right?: ReactNode; playUri?: string; startItem?: string; gap?: boolean; ep?: boolean }) => {
    const key = item.uri;
    const opens = canOpen(item);
    const on = sel === key;
    const ok = done && done.key === key;
    const ringing = acting && acting.key === key && acting.option === 'replace';
    const label = startItem ? `${TEXT.search_play_from_here} \u00b7 ${item.name}` : `${TEXT.search_play_now} ${item.name} \u00b7 ${TEXT.search_play_now_sub}`;
    return <div key={key} className={`search-hit${on ? ' on' : ''}${ep ? ' ep' : ''}`}>
      <div className="search-line">
        <button className="search-main" aria-expanded={opens ? undefined : on} aria-label={opens ? `${TEXT.search_open_item} ${item.name}` : undefined} onClick={() => opens ? open(item) : setSel(on ? null : key)}>
          {lead}
          <span className="search-meta"><span className="search-t">{item.name}</span><span className="search-s dim">{sub}</span></span>
          {right}
        </button>
        {ok ? <span className="search-ok">{I_CHECK_SM}<span>{done!.label}</span></span> : <button className="search-play" aria-label={label} title={label} disabled={!queueId || !!acting} onClick={() => play(key, playUri || item.uri, 'replace', startItem)}>{ringing ? <span className="search-spin" /> : mi(I_PLAY)}</button>}
        {opens ? <button className="search-chev" aria-label={`${TEXT.search_open_item} ${item.name}`} onClick={() => open(item)}>{mi(I_CHEV)}</button> : gap && <span className="search-chev gap" aria-hidden="true" />}
      </div>
      {on && !opens && queueActs(key, item.uri)}
    </div>;
  };
  const resultRow = (item: any) => row({ item, lead: thumb(item, item.media_type === 'artist'), sub: [subOf(item, TEXT.search_kind), item.media_type === 'album' && item.year].filter(Boolean).join(' \u00b7 '), gap: true });

  const top = stack.length ? stack[stack.length - 1] : null;
  const setup = <MaPanel tiers={tiers} ip={ip} defaultOpen />;
  const status = (text: string) => <div className="search-status"><span className="search-spin" /><div className="dim sm">{text}</div></div>;

  /** An opened item: its header with the whole-item verbs, then what it holds. */
  const opened = (level: Level) => {
    const { item, items, shown } = level;
    const kind = item.media_type;
    const heroArt = artThumb(item, httpBase, 256);
    const key = item.uri;
    const hero = <div className="search-hero">
      <span className={`search-hero-art${kind === 'artist' ? ' round' : ''}`}>{I_NOTE_SM}{heroArt && <img key={heroArt} src={heroArt} alt="" onError={e => {
          e.currentTarget.style.display = 'none';
        }} />}</span>
      <h3>{item.name}</h3>
      <div className="dim">{heroSub(item, items ? items.length : null, TEXT)}</div>
      {kind === 'podcast' ? <div className="search-acts">{actBtn(key, item.uri, TEXT.search_play_latest, mi(I_PLAY), 'replace', true, 'latest')}</div> : <div className="search-acts">
          {actBtn(key, item.uri, TEXT.search_play, mi(I_PLAY), 'replace', true)}
          {actBtn(key, item.uri, TEXT.search_play_next, mi(I_NEXT), 'next')}
          {actBtn(key, item.uri, TEXT.search_add, mi(I_PLUS), 'add')}
        </div>}
    </div>;
    if (!items) return <>{hero}{status(TEXT.search_searching)}</>;
    const page = items.slice(0, shown);
    const rest = items.length > shown && <button className="search-more" onClick={more}>{TEXT.search_more}</button>;
    if (kind === 'artist') return <>{hero}{albumSections(items).map(([sec, list]: [string, any[]]) => <Fragment key={sec}>
        <div className="search-sec dim">{sec === 'singles' ? TEXT.search_singles : TEXT.search_album}</div>
        {list.map(a => row({ item: a, lead: thumb(a), sub: albumSub(a, TEXT.search_kind) }))}
      </Fragment>)}</>;
    if (kind === 'album') return <>{hero}{page.map((t, i) => row({ item: t, lead: <span className="search-num dim">{t.track_number || i + 1}</span>, sub: t.artists?.map((a: any) => a.name).join(' | ') || '', right: t.duration ? <span className="search-dur">{fmtTime(t.duration * 1000)}</span> : null, playUri: item.uri, startItem: t.uri }))}{rest}</>;
    if (kind === 'playlist') return <>{hero}{page.map(t => row({ item: t, lead: thumb(t), sub: t.artists?.map((a: any) => a.name).join(' | ') || subOf(t, TEXT.search_kind), right: t.duration ? <span className="search-dur">{fmtTime(t.duration * 1000)}</span> : null, playUri: item.uri, startItem: t.uri }))}{rest}</>;
    return <>{hero}{page.map(ep => {
        const s = episodeSub(ep);
        const sub = <>{[s.date, s.mins ? TEXT.search_minutes.replace('%s', String(s.mins)) : ''].filter(Boolean).join(' \u00b7 ')}{s.left > 0 && <>{' \u00b7 '}<span className="search-left">{TEXT.search_min_left.replace('%s', String(s.left))}</span></>}{s.played && <>{' \u00b7 '}{I_CHECK_SM}{TEXT.search_played}</>}</>;
        return row({ item: ep, lead: null, sub, ep: true });
      })}{rest}</>;
  };

  let body: ReactNode = null;
  if (!configured) {
    // The drawer says what it needs and offers the setup where it stands; the panel unfolds on the
    // button rather than greeting everyone with a token field.
    body = <div className="search-empty">{I_SEARCH_SM}<div className="search-empty-t">{TEXT.search_need_ma_t}</div><p className="dim sm">{TEXT.search_need_ma_b}</p>{setupOpen ? setup : <button className="btn solid primary" onClick={() => setSetupOpen(true)}>{TEXT.search_setup_btn}</button>}</div>;
  } else if (!wsOn) {
    // A refused token gets the panel's own error line and the panel itself, so the fix is where the
    // failure is.
    body = ws.status === 'error' ? <div className="search-empty"><p className="dim sm">{TEXT.ma_error}</p>{setup}</div> : status(TEXT.search_connecting);
  } else if (top) {
    body = opened(top);
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
        {(filter === 'all' ? items.slice(0, ALL_SHOWN) : items).map(resultRow)}
        {filter === 'all' && items.length > ALL_SHOWN && <button className="search-more" onClick={() => setFilter(ty)}>{TEXT.search_more}</button>}
      </Fragment>);
  }
  const where = target ? target.name : groupLabel(members, ws.me?.name || '');
  const prev = stack.length > 1 ? stack[stack.length - 2].item.name : TEXT.search_back;

  return <Drawer label={TEXT.search_title} onClose={onClose} className={`dw-fixed search-drawer${wsOn ? ' tall' : ''}`}>
    {wsOn && top && <div className="search-nav">
      <button className="search-back" onClick={back}>{mi(I_BACK)}<span>{prev}</span></button>
      <span className="search-nav-t">{top.item.name}</span>
    </div>}
    {wsOn && !top && <div className="search-field">
      {I_SEARCH_SM}
      <input ref={inputRef} className="search-in" type="text" enterKeyHint="search" value={q} placeholder={TEXT.search_ph} aria-label={TEXT.search_title} onInput={e => setQ(e.currentTarget.value)} onKeyDown={e => {
        if (e.key === 'Enter') runNow();
      }} />
      {q && <button className="search-x" aria-label={TEXT.search_clear} onClick={() => {
        setQ('');
        inputRef.current?.focus();
      }}>{I_X_SM}</button>}
    </div>}
    {wsOn && !top && <div className="search-pills" role="tablist">
      {FILTERS.map(([id, label]) => <button key={id} className={`npill${filter === id ? ' on' : ''}`} role="tab" aria-selected={filter === id} onClick={() => setFilter(id)}>{label}</button>)}
    </div>}
    <div ref={bodyRef} className="search-body">{body}</div>
    {wsOn && where && <div className="search-foot dim"><span className="dot ok" /><span>{TEXT.search_target} <strong>{where}</strong> {'\u00b7'} {TEXT.search_via}</span></div>}
  </Drawer>;
}
