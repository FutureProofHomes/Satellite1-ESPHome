/**
 * The media bar's logic: which player owns the bar and where its commands land, the two pause
 * signals, the group's rows, the queue's clock, the album tint, and Music Assistant's search
 * results and artwork.
 */

/** ESPHome's MediaPlayerState numbers, as GET /api/sat1/media carries them. */
export const PLAYING = 2;
export const PAUSED = 3;
export const ANNOUNCING = 4;
export const MEDIA_STATE = { [PLAYING]: "Playing", [PAUSED]: "Paused", [ANNOUNCING]: "Announcing" };

/** Bits of the Sendspin controller-command enum `sup` carries - only those that decide whether a
 *  control renders. */
export const SUP = { NEXT: 1 << 3, PREV: 1 << 4, SHUFFLE: 1 << 10 };

/** How long a transport command's pending ring may spin. */
export const PENDING_MAX_MS = 5000;
/** A group edit's: a relayed join is confirmed by a resync that takes 2-7s. */
export const GROUP_PENDING_MAX_MS = 12000;
/** How stale (s) the relay payload may grow while its "paused" claim still holds the card up. */
export const MA_PAUSED_TRUST_S = 120;

export const fmtTime = (ms) => {
  const s = Math.max(0, Math.floor((ms || 0) / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
};

/** A slider's filled share as the design's --pct. */
export const pct = (v, max) => `${(max > 0 ? Math.max(0, Math.min(100, (v / max) * 100)) : 0).toFixed(1)}%`;

/** The `{key: entry}` map without the entries `keep` rejects - the same object when nothing goes,
 *  so a setState with it is a no-op. */
export function prune(map, keep) {
  const all = Object.entries(map);
  const kept = all.filter(keep);
  return kept.length === all.length ? map : Object.fromEntries(kept);
}

/**
 * What the bar shows and where its commands land, from one /api/sat1/media payload.
 *
 * `held` is this browser's own pause of the group stream and `maPaused` the upper tiers' word that
 * the group is paused. Either keeps a stopped group stream on the bar as paused - the Sendspin
 * protocol has no paused state, so the device alone sees a paused group as idle. Track metadata
 * shows only while the group stream owns the bar: a stopped stream keeps its last track cached.
 */
export function mediaView(media, held, maPaused) {
  const deviceActive = !!media && (media.state === PLAYING || media.state === PAUSED);
  const groupHeld = !!(held || maPaused) && !deviceActive && media?.ss_state != null;
  const sendspin = groupHeld || media?.src === "sendspin";
  const active = deviceActive || groupHeld;
  const meta = sendspin && active;
  return {
    groupHeld,
    sendspin,
    active,
    playing: !groupHeld && media?.state === PLAYING,
    announcing: media?.state === ANNOUNCING,
    srcParam: sendspin ? "sendspin" : "local",
    state: groupHeld ? PAUSED : media?.state,
    title: (meta && media.title) || "",
    artist: (meta && media.artist) || "",
    album: (meta && media.album) || "",
    art: (meta && media.art) || "",
  };
}

/** Whether a held pause is over: the group resumed, or the local player started a stream of its own
 *  (so the next idle is that stream ending, not our pause). */
export const holdEnds = (media) =>
  media?.ss_state === PLAYING || (media?.src === "local" && (media.state === PLAYING || media.state === PAUSED));

/** The socket's pause signal: not playing, but the queue still holds the resume point. */
export function wsPausedOf(wsOn, me, queue) {
  return !!wsOn && (me?.state === "paused" || (queue != null && queue.state !== "playing" && queue.current_item != null));
}

/** The relay's: the Home Assistant entity idle with a title (or literally paused), believed only
 *  while fresh - its age when served plus how long this browser has held it - and only with no
 *  socket up to contradict it. */
export function relayPausedOf(wsOn, ma, now) {
  if (wsOn || ma?.at == null || !(ma.age >= 0)) return false;
  if (ma.age + (now - ma.at) / 1000 >= MA_PAUSED_TRUST_S) return false;
  const st = ma.d?.st;
  return st === "paused" || (st === "idle" && !!ma.d.ti);
}

/** The next repeat mode, off -> all -> one -> off, in the device's numbering (0 off, 1 one, 2 all). */
export const nextRepeat = (r) => (r === 0 ? 2 : r === 2 ? 1 : 0);
export const REPEAT_MODE = ["off", "one", "all"];

const byName = (a, b) => String(a[1]).localeCompare(String(b[1]));

/**
 * The group as rows, from whichever tier answers: the socket's players (`me` is this device's) or
 * the relay's `live.g`. `raw` is every member as reported; `members` hides the ones a pending unjoin
 * already took out, sorted by name; `addables` are the speakers that could join - the socket's
 * available players, or the relay's candidates `cands` from the Home Assistant payload.
 * Rows are [id, name, volume 0-100 or -1]; addables [id, name].
 */
export function groupRows({ wsOn, me, players, live, cands, pending }) {
  const ids = wsOn ? (me.group_members?.length ? me.group_members : [me.player_id]) : [];
  const raw = wsOn
    ? ids.map((id) => {
        const p = players[id];
        return [id, p?.name || id, p?.volume_level ?? -1];
      })
    : live?.g || [];
  const members = raw.filter(([id]) => pending[id]?.kind !== "unjoin").sort(byName);
  const addables = (
    wsOn
      ? Object.values(players)
          .filter((p) => p.available && p.player_id !== me.player_id && !ids.includes(p.player_id))
          .map((p) => [p.player_id, p.name])
      : (cands || []).filter(([id]) => !members.some((m) => m[0] === id))
  ).sort(byName);
  return { raw, members, addables };
}

/** Pending group edits that the fresh rows have not confirmed yet: a join until its id appears, an
 *  unjoin until it is gone, either until the deadline. */
export function settleGroup(pending, raw, now) {
  const has = (id) => raw.some((m) => m[0] === id);
  return prune(pending, ([id, e]) => (e.kind === "join" ? !has(id) : has(id)) && now - e.at < GROUP_PENDING_MAX_MS);
}

/** "Kitchen +2": the first member and how many more, or `fallback` with no group to name. */
export const groupLabel = (members, fallback) =>
  members.length ? members[0][1] + (members.length > 1 ? ` +${members.length - 1}` : "") : fallback;

/** The socket queue's playhead in ms, or null when its current item has no duration. MA reports
 *  elapsed_time at elapsed_time_last_updated (s, re-anchored to this browser's clock by lib/ma.js). */
export function queueClock(q, now) {
  const d = q?.current_item?.duration;
  if (!d) return null;
  const dur = d * 1000;
  const pos = q.elapsed_time * 1000 + (q.state === "playing" ? now - q.elapsed_time_last_updated * 1000 : 0);
  return { id: q.queue_id, dur, pos: Math.max(0, Math.min(dur, pos)) };
}

/* ------------------------------------------------------------------ */
/* The album tint                                                      */
/* ------------------------------------------------------------------ */

/** RGB in 0-1 to [h, s, l], with saturation lifted a step (averaging muddies it) and lightness
 *  clamped to where light and dark text both still read on it. */
export function toHsl(r, g, b) {
  const mx = Math.max(r, g, b);
  const mn = Math.min(r, g, b);
  const l = (mx + mn) / 2;
  let h = 0;
  let s = 0;
  if (mx !== mn) {
    const d = mx - mn;
    s = d / (1 - Math.abs(2 * l - 1));
    h = mx === r ? (g - b) / d + (g < b ? 6 : 0) : mx === g ? (b - r) / d + 2 : (r - g) / d + 4;
    h *= 60;
  }
  return [h, Math.min(1, s * 1.3 + 0.04), Math.min(0.72, Math.max(0.34, l))];
}

/** The average colour of RGBA pixel data (a canvas readback) as toHsl's triple. */
export function averageHsl(data) {
  let r = 0;
  let g = 0;
  let b = 0;
  const n = data.length / 4;
  for (let i = 0; i < data.length; i += 4) {
    r += data[i];
    g += data[i + 1];
    b += data[i + 2];
  }
  return toHsl(r / n / 255, g / n / 255, b / n / 255);
}

/** The colour as the design's --tint wash, at the design's .22 alpha. */
export function tintOf(col) {
  if (!col) return "transparent";
  const [h, s, l] = col;
  return `hsla(${h | 0},${(s * 100) | 0}%,${(l * 100) | 0}%,.22)`;
}

/* ------------------------------------------------------------------ */
/* Search                                                              */
/* ------------------------------------------------------------------ */

/** The searchable types in the order their sections render, and the result keys they come back
 *  under (music/search answers plurals, except radio). */
export const SEARCH_TYPES = ["track", "artist", "album", "playlist", "radio", "podcast", "audiobook"];
const RESULT_KEY = {
  track: "tracks",
  artist: "artists",
  album: "albums",
  playlist: "playlists",
  radio: "radio",
  podcast: "podcasts",
  audiobook: "audiobooks",
};
/** How many rows an "All" section shows; one more is fetched so "Show more" is honest. */
export const ALL_SHOWN = 3;
const FILTERED_LIMIT = 25;

/** music/search's args for a query under a filter pill. */
export const searchArgs = (query, filter) => ({
  search_query: query,
  media_types: filter === "all" ? SEARCH_TYPES : [filter],
  limit: filter === "all" ? ALL_SHOWN + 1 : FILTERED_LIMIT,
});

/** [type, items] per section with anything in it, in render order. */
export const searchGroups = (r, filter) =>
  (filter === "all" ? SEARCH_TYPES : [filter]).map((ty) => [ty, r?.[RESULT_KEY[ty]] || []]).filter(([, items]) => items.length > 0);

/** A result's thumbnail record in Music Assistant's own preference order: its own thumb, its
 *  album's, its metadata's, its first artist's. */
export function findImage(item) {
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

/** The thumbnail as a loadable URL against MA's HTTP origin `base`: a data URI as-is, a proxy id
 *  (schema >= 31) through /imageproxy/<id>, a remotely-accessible http(s) path as-is, anything else
 *  through the legacy /imageproxy with MA's double encoding. 80px is the proxy's smallest size. */
export function artThumb(item, base) {
  const img = findImage(item);
  if (!img?.path) return "";
  if (img.path.startsWith("data:image")) return img.path;
  if (img.proxy_id) return base ? `${base}/imageproxy/${img.proxy_id}?size=80` : "";
  if (img.remotely_accessible && /^https?:/i.test(img.path)) return img.path;
  if (!base) return "";
  const enc = encodeURIComponent(encodeURIComponent(img.path));
  return `${base}/imageproxy?path=${enc}&provider=${encodeURIComponent(img.provider || "")}&size=80`;
}

/** A row's sub-line: the kind by name (`kinds` maps media_type to a label), then its makers. */
export function subOf(item, kinds) {
  const names =
    (item.artists?.length && item.artists.map((a) => a.name).join(" | ")) ||
    item.owner ||
    (Array.isArray(item.authors) && item.authors.join(" | ")) ||
    item.publisher ||
    "";
  const kind = kinds[item.media_type] || "";
  return names ? `${kind} \u00b7 ${names}` : kind;
}

/** The recent-searches list with `query` on top, deduped without regard to case. */
export function addRecent(list, query, max) {
  const t = String(query || "").trim();
  if (!t) return list;
  return [t, ...list.filter((r) => r.toLowerCase() !== t.toLowerCase())].slice(0, max);
}
