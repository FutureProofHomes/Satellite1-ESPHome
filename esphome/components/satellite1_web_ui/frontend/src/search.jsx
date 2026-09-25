/**
 * The search drawer (Plan 20): the top bar's magnifying glass, opening a bottom sheet that
 * searches Music Assistant and plays the pick on this speaker - or on the group it is part of,
 * which costs nothing extra: the commands aim at the device's *active queue*, and Music Assistant
 * redirects that to the group leader's queue whenever the player is grouped.
 *
 * Owner-approved mockups are the spec (canvas ma-search-drawer-mockups, September 2026): the
 * pinned field, the pill row, grouped results with 40px art (round for artists), the inline
 * three-verb action row, the "Queued" check, the footer naming the play target, and the
 * not-connected empty state. Deviations need a comment at the spot, not silence.
 *
 * Socket tier only, by design. The Home Assistant relay speaks the fixed verbs the firmware
 * bakes in, and teaching it search would cost firmware bytes plus a device round trip per
 * keystroke - against the MA tier's whole reason to exist (lib/ma.js: "costs the firmware zero
 * bytes"). So the drawer is honest about its requirement: without a configured connection it
 * offers the same MaPanel the expanded media view carries, and works the moment that does.
 *
 * The MA contract (verified against the MA frontend @ 367878c, September 2026):
 *   music/search { search_query, media_types, limit } -> { tracks, artists, albums, playlists,
 *     radio, podcasts, audiobooks } (limit applies per type);
 *   player_queues/play_media { queue_id, media: uri, option } with option replace|next|add.
 * "Play now" sends replace rather than play on purpose: its sub-label promises "replaces queue",
 * and that is the common intent of picking an album by name - predictable beats clever here.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { TEXT } from "./copy.js";
import { maHttpBase } from "./lib/ma.js";
import { toast } from "./lib/toast.js";
import { useDrawer, useSheetDrag } from "./ui.jsx";

/* The searchable types, in the order the sections render - MA's own modal's order. Each is one
   media_types string, so the last two cost pills, not code. */
const TYPES = ["track", "artist", "album", "playlist", "radio", "podcast", "audiobook"];
/** SearchResults keys by type - the one place the API's plural spellings live. */
const RESULT_KEY = {
  track: "tracks",
  artist: "artists",
  album: "albums",
  playlist: "playlists",
  radio: "radio",
  podcast: "podcasts",
  audiobook: "audiobooks",
};
const TYPE_LABEL = {
  track: TEXT.search_track,
  artist: TEXT.search_artist,
  album: TEXT.search_album,
  playlist: TEXT.search_playlist,
  radio: TEXT.search_radio,
  podcast: TEXT.search_podcast,
  audiobook: TEXT.search_audiobook,
};

/** How many rows an "All" group shows. One more is fetched so "Show more" only appears when
 *  there genuinely is more, and a filtered pill fetches a real page. */
const ALL_SHOWN = 3;
const FILTERED_LIMIT = 25;
/** The debounce between the last keystroke and the search - MA's own modal uses the same order
 *  of magnitude; Enter skips it. */
const DEBOUNCE_MS = 400;
/** How long a row's confirmation check stands before the row returns to normal. */
const DONE_MS = 2500;
/** The give-up ceiling on a play command's spinner (plan 12's rule: never spin forever). */
const ACT_TIMEOUT_MS = 10000;

/* ------------------------------------------------------------------ */
/* Glyphs - media.jsx's mi() dialect, drawn locally because importing them from media.jsx would
   be a cycle (media.jsx renders this drawer). Same 16-box, same stroke. */
/* ------------------------------------------------------------------ */

const si = (path) => (
  <svg class="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    {path}
  </svg>
);

const I_SEARCH = si(
  <>
    <circle cx="7" cy="7" r="4.4" />
    <path d="M10.4 10.4 14 14" />
  </>,
);
const I_CLOCK = si(
  <>
    <circle cx="8" cy="8" r="5.6" />
    <path d="M8 4.8V8l2.3 1.6" />
  </>,
);
const I_X = si(<path d="M4 4l8 8M12 4l-8 8" />);
const I_CHECK = si(<path d="M3.5 8.5 6.6 11.6 12.5 4.9" />);
/** Play-next: a queue with an entry pushing in at the top - the insertion is the meaning. */
const I_NEXT = si(
  <>
    <path d="M3 3.5h6M3 8h10M3 12.5h10" />
    <path d="M11.5 1.2v4.6M13.8 3.5 11.5 5.8 9.2 3.5" fill="none" />
  </>,
);
/** Add-to-queue: the same queue with a + beside its tail. */
const I_ADD = si(<path d="M3 3.5h10M3 8h10M3 12.5h5M11.5 10v5M9 12.5h5" />);
/** The filled play triangle, media.jsx's exact geometry. */
const I_PLAY = (
  <svg class="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M5.4 3c0-.5.6-.9 1-.6l7 4.5c.4.3.4.9 0 1.2l-7 4.5c-.4.3-1 0-1-.6V3Z" />
  </svg>
);
/** The note in a rounded square - the artwork fallback, same as the footer's. */
const I_NOTE = (
  <svg class="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M11.8 1.6 6.4 3v7.2a2.3 2.3 0 1 0 1.2 2V6.2l4.2-1.1v3.8a2.3 2.3 0 1 0 1.2 2V1.9a.3.3 0 0 0-.4-.3H11.8Z" />
  </svg>
);

/* ------------------------------------------------------------------ */
/* Recent searches - localStorage, this browser only, the MA token's own storage policy. Guarded
   like every localStorage touch in this app: losing the memory is survivable.              */
/* ------------------------------------------------------------------ */

const KEY_RECENT = "sat1.ma.recent";
const RECENT_MAX = 8;

function readRecents() {
  try {
    const v = JSON.parse(localStorage.getItem(KEY_RECENT) || "[]");
    return Array.isArray(v) ? v.filter((s) => typeof s === "string") : [];
  } catch {
    return [];
  }
}

function writeRecents(list) {
  try {
    localStorage.setItem(KEY_RECENT, JSON.stringify(list));
  } catch {
    /* Not remembered; the session still searches. */
  }
}

/* ------------------------------------------------------------------ */
/* Artwork                                                             */
/* ------------------------------------------------------------------ */

/**
 * A result's thumbnail image record, in Music Assistant's own preference order (their
 * getMediaItemImage): the ItemMapping's own `image`, then the album's (tracks wear their album),
 * then the item's metadata images, then the first artist's.
 */
function findImage(item) {
  if (!item) return null;
  if (item.image?.path && item.image.type === "thumb") return item.image;
  if (item.album) {
    const a = findImage(item.album);
    if (a) return a;
  }
  for (const im of item.metadata?.images || []) if (im.type === "thumb" && im.path) return im;
  if (item.artists?.length) return findImage(item.artists[0]);
  return null;
}

/**
 * The image record as a URL this browser can load. Three forms, told apart by the record itself:
 * an opaque proxy id (server schema >= 31) rides /imageproxy/<id>, a remotely-accessible http(s)
 * path is used as-is, and anything else takes the legacy /imageproxy?provider=&path= form with
 * MA's own double-encoding. 80px is the proxy's smallest allowed size - exactly the 40px row at
 * 2x. No CORS concern here: these are plain <img> loads, never canvas readbacks.
 */
function artThumb(item, base) {
  const img = findImage(item);
  if (!img?.path) return "";
  if (img.path.startsWith("data:image")) return img.path;
  if (img.proxy_id) return base ? `${base}/imageproxy/${img.proxy_id}?size=80` : "";
  if (img.remotely_accessible && /^https?:/i.test(img.path)) return img.path;
  if (!base) return "";
  const enc = encodeURIComponent(encodeURIComponent(img.path));
  return `${base}/imageproxy?path=${enc}&provider=${encodeURIComponent(img.provider || "")}&size=80`;
}

/** The row's sub-line: the kind by name, then whoever made it - artists joined the way MA's own
 *  modal joins them, a playlist's owner, an audiobook's authors. */
function subOf(item) {
  const names =
    (item.artists?.length && item.artists.map((a) => a.name).join(" | ")) ||
    item.owner ||
    (Array.isArray(item.authors) && item.authors.join(" | ")) ||
    item.publisher ||
    "";
  const kind = TEXT.search_kind[item.media_type] || "";
  return names ? `${kind} \u00b7 ${names}` : kind;
}

/* ------------------------------------------------------------------ */
/* The search itself                                                   */
/* ------------------------------------------------------------------ */

/**
 * Debounced, sequenced search over the socket. One request in flight is the newest one; answers
 * to anything older are dropped by sequence number, so results never flicker backwards to an
 * earlier keystroke's answer. A failed search keeps the previous results standing and reports
 * through the toast - blanking a list someone is reading over one dropped frame helps nobody.
 */
function useMaSearch(cmd, ready, q, filter) {
  const [res, setRes] = useState(null);
  const [busy, setBusy] = useState(false);
  const seq = useRef(0);
  const timer = useRef(null);

  const run = (query, f) => {
    const my = ++seq.current;
    setBusy(true);
    cmd("music/search", {
      search_query: query,
      media_types: f === "all" ? TYPES : [f],
      limit: f === "all" ? ALL_SHOWN + 1 : FILTERED_LIMIT,
    })
      .then((r) => {
        if (seq.current !== my) return;
        setRes({ q: query, r: r || {} });
        setBusy(false);
      })
      .catch(() => {
        if (seq.current !== my) return;
        setBusy(false);
        toast({ kind: "err", key: "search", ttl: 6000, title: TEXT.ma_error });
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
    return () => clearTimeout(timer.current);
    // `cmd` is a fresh closure every render but always speaks to the same socket, so re-running
    // on its identity would re-fire the search on every paint; the query and filter are the inputs.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [q, filter, ready]);

  return {
    res,
    busy,
    // Enter's path: the same search, minus the wait.
    runNow: () => {
      if (timer.current) clearTimeout(timer.current);
      const query = q.trim();
      if (ready && query.length >= 2) run(query, filter);
    },
  };
}

/* ------------------------------------------------------------------ */
/* The drawer                                                          */
/* ------------------------------------------------------------------ */

/**
 * `tiers` is MediaFooter's useTiers result - the socket, its status, and the group rows the
 * footer chip is built from. `setup` is a ready-made MaPanel element (constructed by the footer,
 * passed in rather than imported, because importing it from media.jsx here would be a cycle).
 */
export function SearchDrawer({ tiers, setup, onClose }) {
  const { ws, wsOn, maCfg, members } = tiers;

  useDrawer("search", true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);

  const [q, setQ] = useState("");
  const [filter, setFilter] = useState("all");
  const [recents, setRecents] = useState(readRecents);
  // The tapped row (by uri), holding the action row open under it.
  const [sel, setSel] = useState(null);
  // One command in flight at a time: { uri, option }, cleared by the answer or the ceiling.
  const [acting, setActing] = useState(null);
  // The last confirmed command's row: { uri, label }, standing for DONE_MS.
  const [done, setDone] = useState(null);
  // The not-connected state's fold: the setup panel appears on the button, not before.
  const [setupOpen, setSetupOpen] = useState(false);

  const configured = !!(maCfg.url && maCfg.token);
  const httpBase = maHttpBase(maCfg.url);
  const { res, busy, runNow } = useMaSearch(ws.cmd, wsOn, q, filter);

  // The field owns the keyboard from the moment the drawer opens - the mockups' contract.
  const inputRef = useRef(null);
  useEffect(() => {
    if (wsOn) inputRef.current?.focus();
  }, [wsOn]);

  // Where a pick plays: the active queue, which is the group leader's whenever this player is
  // grouped - the redirect Music Assistant does for us. The player id is the fallback for the
  // moment before the queue answer lands.
  const queueId = ws.queue?.queue_id || ws.me?.player_id || "";

  const remember = (query) => {
    const t = query.trim();
    if (!t) return;
    const list = [t, ...recents.filter((r) => r.toLowerCase() !== t.toLowerCase())].slice(0, RECENT_MAX);
    setRecents(list);
    writeRecents(list);
  };
  const forget = (r) => {
    const list = recents.filter((x) => x !== r);
    setRecents(list);
    writeRecents(list);
  };

  const play = (item, option) => {
    if (!queueId || acting) return;
    setActing({ uri: item.uri, option });
    // The ceiling: a socket that silently swallows the answer must not spin the button forever.
    const ceiling = setTimeout(() => setActing((a) => (a && a.uri === item.uri ? null : a)), ACT_TIMEOUT_MS);
    ws.cmd("player_queues/play_media", { queue_id: queueId, media: item.uri, option })
      .then(() => {
        clearTimeout(ceiling);
        setActing(null);
        setSel(null);
        setDone({ uri: item.uri, label: option === "replace" ? TEXT.search_playing : TEXT.search_queued });
        setTimeout(() => setDone((d) => (d && d.uri === item.uri ? null : d)), DONE_MS);
        // A search someone played from is a search worth remembering.
        remember(q);
      })
      .catch(() => {
        clearTimeout(ceiling);
        setActing(null);
        toast({ kind: "err", key: "search-play", ttl: 6000, title: TEXT.search_play_failed, sub: TEXT.search_play_failed_sub });
      });
  };

  /* One result row, and the action row a tap opens under it. */
  const actBtn = (item, label, sub, glyph, option, solid) => {
    const spinning = acting && acting.uri === item.uri && acting.option === option;
    return (
      <button
        class={`search-act${solid ? " solid" : ""}`}
        disabled={!queueId || !!acting}
        onClick={() => play(item, option)}
      >
        {spinning ? <span class="search-spin" /> : glyph}
        <span>{label}</span>
        {sub && <span class="search-act-sub">{sub}</span>}
      </button>
    );
  };

  const row = (item) => {
    const on = sel === item.uri;
    const ok = done && done.uri === item.uri;
    const art = artThumb(item, httpBase);
    // The active modifier is "on", not "sel": .sel is the app's <select> dress app-wide, and a
    // div wearing it inherits a form control's box.
    return (
      <div key={item.uri} class={`search-hit${on ? " on" : ""}`}>
        <button class="search-row" aria-expanded={on} onClick={() => setSel(on ? null : item.uri)}>
          <span class={`search-art${item.media_type === "artist" ? " round" : ""}`}>
            {I_NOTE}
            {/* Keyed by URL so a filter change swaps the element rather than letting the old
                image linger; a host that refuses the load hides the img and the note shows. */}
            {art && <img key={art} src={art} alt="" loading="lazy" onError={(e) => (e.currentTarget.style.display = "none")} />}
          </span>
          <span class="search-meta">
            <span class="search-t">{item.name}</span>
            <span class="search-s dim">{subOf(item)}</span>
          </span>
          {ok ? (
            <span class="search-ok">
              {I_CHECK}
              {done.label}
            </span>
          ) : (
            <span class="search-go">{I_PLAY}</span>
          )}
        </button>
        {on && (
          <div class="search-acts">
            {actBtn(item, TEXT.search_play_now, TEXT.search_play_now_sub, I_PLAY, "replace", true)}
            {actBtn(item, TEXT.search_play_next, null, I_NEXT, "next")}
            {actBtn(item, TEXT.search_add, null, I_ADD, "add")}
          </div>
        )}
      </div>
    );
  };

  /* The body, by connection state and then by query state. */
  let body;
  if (!configured) {
    // Mockup frame 6: the drawer says what it needs and offers the setup where it stands. The
    // panel unfolds on the button rather than greeting everyone with a token field.
    body = (
      <div class="search-empty">
        {I_SEARCH}
        <div class="search-empty-t">{TEXT.search_need_ma_t}</div>
        <p class="dim sm">{TEXT.search_need_ma_b}</p>
        {setupOpen ? setup : (
          <button class="btn solid" onClick={() => setSetupOpen(true)}>
            {TEXT.search_setup_btn}
          </button>
        )}
      </div>
    );
  } else if (!wsOn) {
    // Configured but not up: connecting gets the spinner, a refused token gets the panel's own
    // error vocabulary plus the panel, so the fix is where the failure is.
    body =
      ws.status === "error" ? (
        <div class="search-empty">
          <p class="dim sm">{TEXT.ma_error}</p>
          {setup}
        </div>
      ) : (
        <div class="search-status">
          <span class="search-spin" />
          <div class="dim sm">{TEXT.search_connecting}</div>
        </div>
      );
  } else if (q.trim().length < 2) {
    body = (
      <>
        {recents.length > 0 && <div class="search-sec dim">{TEXT.search_recent}</div>}
        {recents.map((r) => (
          <div key={r} class="search-rec">
            <button class="search-rec-hit" onClick={() => setQ(r)}>
              {I_CLOCK}
              <span>{r}</span>
            </button>
            <button class="icon search-rec-x" aria-label={TEXT.search_forget} onClick={() => forget(r)}>
              {I_X}
            </button>
          </div>
        ))}
      </>
    );
  } else if (busy && !res) {
    body = (
      <div class="search-status">
        <span class="search-spin" />
        <div class="dim sm">{TEXT.search_searching}</div>
      </div>
    );
  } else if (res) {
    const groups = (filter === "all" ? TYPES : [filter])
      .map((ty) => [ty, res.r[RESULT_KEY[ty]] || []])
      .filter(([, items]) => items.length > 0);
    body =
      groups.length === 0 && !busy ? (
        <p class="dim sm search-status">{TEXT.search_none.replace("%s", res.q)}</p>
      ) : (
        groups.map(([ty, items]) => (
          <div key={ty}>
            <div class="search-sec dim">{TYPE_LABEL[ty]}</div>
            {(filter === "all" ? items.slice(0, ALL_SHOWN) : items).map(row)}
            {filter === "all" && items.length > ALL_SHOWN && (
              <button class="search-more" onClick={() => setFilter(ty)}>
                {TEXT.search_more}
              </button>
            )}
          </div>
        ))
      );
  }

  // The footer's target: the first group member plus a count - the expanded view's chip, reused
  // as a sentence - or the player's own name before any group exists.
  const target = members.length
    ? members[0][1] + (members.length > 1 ? ` +${members.length - 1}` : "")
    : ws.me?.name || "";

  return (
    <>
      <div class="scrim search-scrim" onClick={onClose} />
      <div
        class={`search-drawer${wsOn ? " tall" : ""}`}
        style={dragStyle || undefined}
        {...drag}
        role="dialog"
        aria-label={TEXT.search_title}
      >
        <button class="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
        {wsOn && (
          <>
            <div class="search-field">
              {I_SEARCH}
              <input
                ref={inputRef}
                class="search-in"
                type="text"
                enterkeyhint="search"
                value={q}
                placeholder={TEXT.search_ph}
                aria-label={TEXT.search_title}
                onInput={(e) => setQ(e.currentTarget.value)}
                onKeyDown={(e) => e.key === "Enter" && runNow()}
              />
              {q && (
                <button
                  class="search-x"
                  aria-label={TEXT.search_clear}
                  onClick={() => {
                    setQ("");
                    inputRef.current?.focus();
                  }}
                >
                  {I_X}
                </button>
              )}
            </div>
            <div class="search-pills" role="tablist">
              {[["all", TEXT.search_all], ...TYPES.map((t) => [t, TYPE_LABEL[t]])].map(([id, label]) => (
                <button
                  key={id}
                  class={`npill${filter === id ? " on" : ""}`}
                  role="tab"
                  aria-selected={filter === id}
                  onClick={() => setFilter(id)}
                >
                  {label}
                </button>
              ))}
            </div>
          </>
        )}
        <div class="search-body">{body}</div>
        {wsOn && target && (
          <div class="search-foot dim">
            <span class="dot ok" />
            <span>
              {TEXT.search_target} <strong>{target}</strong> {"\u00b7"} {TEXT.search_via}
            </span>
          </div>
        )}
      </div>
    </>
  );
}
