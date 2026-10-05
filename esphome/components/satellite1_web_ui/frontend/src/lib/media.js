/**
 * The media bar's logic: which player owns the bar and where its commands land, the two pause
 * signals, the group's rows, the queue's clock, the album tint, and Music Assistant's search
 * results and artwork. What each tier adds is in docs/web-ui.md, "The media footer and its three
 * tiers".
 */

/** ESPHome's MediaPlayerState numbers, as GET /api/sat1/media carries them. */
export const PLAYING = 2;
export const PAUSED = 3;
export const ANNOUNCING = 4;
export const MEDIA_STATE = { [PLAYING]: "Playing", [PAUSED]: "Paused", [ANNOUNCING]: "Announcing" };

/** Bits of the Sendspin controller-command enum `sup` carries - only those that decide whether a
 *  control renders. */
export const SUP = { NEXT: 1 << 3, PREV: 1 << 4, SHUFFLE: 1 << 10 };

/** How long a transport command's pending ring may spin: useHeld's five seconds, the app-wide
 *  escape for a write the device refused or dropped. Confirmation arrives on the 1s playing-cadence
 *  poll, so a healthy command settles in one or two ticks; the deadline only keeps a dead device
 *  from leaving a ring spinning forever. */
export const PENDING_MAX_MS = 5000;
/** A group edit's, deliberately longer: a relayed join or unjoin is confirmed by a resync that
 *  legitimately takes 2-7s (the 2.2s early read, the ~6s panel cycle), so five seconds would give up
 *  on healthy edits. The socket tier confirms in well under a second; twelve is only how long a
 *  failure takes to unwind. */
export const GROUP_PENDING_MAX_MS = 12000;
/** Take over and Move here, which are two steps on the relay (the queue moves, then the speakers
 *  join) and so get a little longer than a join before they are called failed. */
export const TAKEOVER_PENDING_MAX_MS = 15000;
/** Another speaker's Next or Play/Pause, confirmed by its row changing. Longer than the bar's own
 *  five seconds because the relay confirms on its ~6s cycle rather than the device's 1s poll. */
export const REMOTE_PENDING_MAX_MS = 10000;
/** How stale (s) the relay payload may grow while its "paused" claim still holds the card up,
 *  counted as its `age` when Home Assistant rendered it plus how long this browser has held it
 *  (`at`), because the two go stale independently. Two minutes spans the bar's 45s recheck
 *  (MA_PAUSED_RECHECK_MS) with room for a slow round trip, and is short enough that a relay that
 *  stopped answering takes the card down rather than captioning silence indefinitely. */
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
 * the group is paused. Either keeps a stopped group stream on the bar as paused, with every command
 * aimed at the group player (`src=sendspin`). The Sendspin protocol has no paused state - playing or
 * stopped is its whole vocabulary - so the moment a pause lands the device sees an idle player, and
 * without the hold the bar would go idle and strand the resume it just offered. Track metadata
 * shows only while the group stream owns the bar, because a stopped stream keeps its last track in
 * the device's cache and showing it would caption silence; a held or reported pause keeps it, since
 * that is the track resume continues.
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

/**
 * The socket's pause signal, for a pause made anywhere else - Music Assistant's own UI, another
 * Satellite - which the device alone cannot tell from a stream that ended (owner's screenshots,
 * September 2026: MA's bar holding the paused track while this one said nothing was playing).
 * Pausing a Sendspin player stops its stream, so the MA player - and with it the Home Assistant
 * entity - reports idle, never paused (verified against the integration source and a live pause).
 * What MA's own bar shows is the queue's current item, which survives the pause as the resume
 * point, so both tiers test "not playing, but the queue still holds a current item". The literal
 * "paused" state is honoured too, for the day a Sendspin pause stops meaning "stop": it costs
 * nothing and can only agree. The socket reads the queue directly and is believed outright.
 */
export function wsPausedOf(wsOn, me, queue) {
  return !!wsOn && (me?.state === "paused" || (queue != null && queue.state !== "playing" && queue.current_item != null));
}

/** The relay's: the Home Assistant entity idle with a title (`st`/`ti` - Home Assistant mirrors the
 *  queue's current item as media_title in every playback scenario), or literally paused. Believed
 *  only while fresh - its age when served plus how long this browser has held it, under
 *  MA_PAUSED_TRUST_S - and only with no socket up to contradict it. */
export function relayPausedOf(wsOn, ma, now) {
  if (wsOn || ma?.at == null || !(ma.age >= 0)) return false;
  if (ma.age + (now - ma.at) / 1000 >= MA_PAUSED_TRUST_S) return false;
  const st = ma.d?.st;
  return st === "paused" || (st === "idle" && !!ma.d.ti);
}

/** The next repeat mode, off -> all -> one -> off (the order the three are reached for), in the
 *  device's numbering (0 off, 1 one, 2 all). */
export const nextRepeat = (r) => (r === 0 ? 2 : r === 2 ? 1 : 0);
export const REPEAT_MODE = ["off", "one", "all"];

const byName = (a, b) => String(a[1]).localeCompare(String(b[1]));

/** Music Assistant's player types that are not speakers anyone would join or take music from: a
 *  speaker's hidden per-protocol twin (a Sonos's AirPlay side, which would list the Sonos twice),
 *  and the screens, visualizers, lights and capture-only inputs MA also models as players. Excluded
 *  rather than the speaker types included, so a server older than the field lists everything. */
const NOT_SPEAKERS = new Set(["protocol", "display", "visualizer", "light", "source"]);

/** A socket player's state. `playback_state` since MA 2.5; `state` is its alias for older clients. */
export const playerState = (p) => p?.playback_state || p?.state || "";

const isSpeaker = (p) =>
  !!p && p.available !== false && p.enabled !== false && !p.hide_in_ui && !p.private && !NOT_SPEAKERS.has(p.type);

/**
 * The player whose group `me` plays in: the group player it is a member of, the sync leader it
 * follows, or itself. Every group view and every join is built from this, because a member's own
 * group_members is empty - read from the member, the group was a group of one (owner's report,
 * October 2026: Dev3 leading showed a badge "2" while its member Dev12 showed none), and an add
 * aimed at the member split the group instead of growing it.
 */
export function leaderOf(me, players) {
  if (!me) return null;
  const by = (id) => (id && id !== me.player_id ? players?.[id] : null);
  return by(me.active_group) || by(me.synced_to) || me;
}

/** Whether `a`'s can_group_with names `b`, by player id or by the provider instance MA uses for
 *  "every player of this provider". Null when `a` gives no list to judge by. */
function namesIn(a, b) {
  const list = a?.can_group_with;
  if (!Array.isArray(list) || list.length === 0) return null;
  return list.includes(b.player_id) || (!!b.provider && list.includes(b.provider));
}

/**
 * Whether two socket players can play in sync: true when either side's list names the other,
 * false only when both give a list and neither does, null (unknown) otherwise. Either direction
 * counts because Music Assistant leaves a playing leader out of everyone else's list; unknown
 * offers Join and Take over and lets the attempt decide, as the relay always does.
 */
export function canSync(a, b) {
  const x = namesIn(a, b);
  const y = namesIn(b, a);
  if (x || y) return true;
  return x === false && y === false ? false : null;
}

/**
 * The group as rows, from whichever tier answers: the socket's players (`me` is this device's) or
 * the relay's `live.g`. `raw` is every member as reported; `members` hides the ones a pending unjoin
 * already took out, sorted by name; `addables` are the speakers that could join - on the socket the
 * leader's can_group_with (every available speaker on a server without it), on the relay the
 * candidates `cands` from the Home Assistant payload. `leader` is the id every join targets.
 * Rows are [id, name, volume 0-100 or -1]; addables [id, name]. The ids are MA player ids on the
 * socket and Home Assistant entity ids through the relay, which is fine because every command goes
 * to the tier that produced its row. Both lists sort by name (owner's request, September 2026): the
 * socket reports members in join order and the relay in payload order, and neither order means
 * anything to someone scanning the list for a room.
 *
 * A speaker that is playing, or is in another group, is not an addable: one tap there used to stop
 * someone else's music without a word. Neither is anything `others` (elsewhereRows) lists, members
 * included - a paused speaker is someone's music too, and it has its own row under Playing
 * elsewhere - which is also what keeps the relay's stateless candidates honest. A row kept only for
 * its bar card (idle now) blocks nothing.
 */
export function groupRows({ wsOn, me, players, live, cands, pending, others }) {
  const busy = new Set();
  for (const r of others || []) {
    if (r.state === "idle") continue;
    busy.add(r.id);
    for (const m of r.members) busy.add(m[0]);
  }
  if (wsOn) {
    const lead = leaderOf(me, players);
    const ids = lead.group_members?.length ? [...lead.group_members] : [lead.player_id];
    // A group player is its members, never one of them (another speaker's panel opens on one).
    if (!ids.includes(me.player_id) && me.type !== "group") ids.push(me.player_id);
    const raw = ids.map((id) => {
      const p = players[id];
      return [id, p?.name || id, p?.volume_level ?? -1];
    });
    const members = raw.filter(([id]) => pending[id]?.kind !== "unjoin").sort(byName);
    const cg = Array.isArray(lead.can_group_with) && lead.can_group_with.length ? lead.can_group_with : null;
    const addables = Object.values(players)
      .filter(
        (p) =>
          isSpeaker(p) &&
          p.available &&
          p.type !== "group" &&
          p.player_id !== me.player_id &&
          p.player_id !== lead.player_id &&
          !ids.includes(p.player_id) &&
          !p.synced_to &&
          !(p.active_group && p.active_group !== p.player_id) &&
          !busy.has(p.player_id) &&
          playerState(p) !== "playing" &&
          (!cg || cg.includes(p.player_id) || (!!p.provider && cg.includes(p.provider))),
      )
      .map((p) => [p.player_id, p.name])
      .sort(byName);
    return { raw, members, addables, leader: lead.player_id };
  }
  const raw = live?.g || [];
  const members = raw.filter(([id]) => pending[id]?.kind !== "unjoin").sort(byName);
  const addables = (cands || []).filter(([id]) => !members.some((m) => m[0] === id) && !busy.has(id)).sort(byName);
  return { raw, members, addables, leader: live?.l || raw[0]?.[0] || "" };
}

const normId = (s) => String(s || "").toLowerCase().replace(/[:-]/g, "");

/**
 * The other speakers and groups Music Assistant is playing, one row per group, from whichever tier
 * answers. Each is `{id, name, members, group, title, artist, art, state, volume, vctl,
 * transferable, queue, sync, mac, clock}`: `id` is what commands target (the group's leader or the
 * group player), `members` the rest of its group as rows, `vctl` whether its volume can be set at
 * all, `transferable` whether it plays from a Music Assistant queue (Take over and Move here need
 * one; Spotify Connect and other native sources have none), `queue` the queue to take, `sync` the
 * canSync verdict against this group's leader (always null through the relay, which cannot know),
 * `mac` the device's MAC where the socket knows it, and `clock` the socket's playhead.
 *
 * A row is a leader or an ungrouped speaker that is playing or paused, outside this device's
 * group. A member of a group player is left to the group player's row. `keep` lists ids to include
 * even while idle - the bar's cards for speakers paused during this visit, which a Sendspin
 * player reports as idle, never paused.
 */
export function elsewhereRows({ wsOn, me, players, live, keep }) {
  if (wsOn) {
    if (!me || !players) return [];
    const lead = leaderOf(me, players);
    const mine = new Set([me.player_id, lead.player_id, ...(lead.group_members || [])]);
    const out = [];
    for (const p of Object.values(players)) {
      if (mine.has(p.player_id) || !isSpeaker(p)) continue;
      const st = playerState(p);
      if (st !== "playing" && st !== "paused" && !keep?.has(p.player_id)) continue;
      if (p.synced_to && players[p.synced_to]) continue;
      if (p.active_group && p.active_group !== p.player_id && players[p.active_group]) continue;
      const cm = p.current_media || null;
      const group = p.type === "group";
      const memberIds = (p.group_members || []).filter((id) => id !== p.player_id);
      const vol = group || memberIds.length ? (p.group_volume ?? p.volume_level) : p.volume_level;
      out.push({
        id: p.player_id,
        name: p.name || p.player_id,
        members: memberIds.map((id) => [id, players[id]?.name || id, players[id]?.volume_level ?? -1]),
        group,
        title: cm?.title || "",
        artist: cm?.artist || "",
        art: cm?.image_url || "",
        state: st || "idle",
        volume: vol ?? -1,
        vctl: vol != null && (group || !p.volume_control || p.volume_control !== "none"),
        transferable: !!cm?.queue_item_id,
        queue: p.active_source || p.player_id,
        sync: canSync(lead, p),
        mac: normId(p.device_info?.mac_address),
        clock:
          p.elapsed_time != null && cm?.duration
            ? { pos: p.elapsed_time, at: p.elapsed_time_last_updated || 0, dur: cm.duration }
            : null,
      });
    }
    return out.sort((a, b) => a.name.localeCompare(b.name));
  }
  // The relay's `o`: [entity, name, [[member, volume], ...], title, artist, art, state, volume,
  // supported_features, has a queue, is a group player] - web_ui_media.yaml has the selection.
  // Members carry no names (payload size); their entity id stands in, and only their count shows.
  return (live?.o || []).map((r) => ({
    id: r[0],
    name: r[1] || r[0],
    members: (r[2] || []).map(([m, v]) => [m, m, v]),
    group: r[10] === 1,
    title: r[3] || "",
    artist: r[4] || "",
    art: r[5] || "",
    state: r[6] || "playing",
    volume: r[7] ?? -1,
    // VOLUME_SET is Home Assistant's media player feature bit 4.
    vctl: ((r[8] || 0) & 4) !== 0 && (r[7] ?? -1) >= 0,
    transferable: r[9] === 1,
    queue: r[0],
    sync: null,
    mac: "",
    clock: null,
  }));
}

/**
 * The bar's cards after this device's own: one per other speaker playing now, and one per speaker
 * seen playing during this page visit (`seen`) that is paused now. A speaker whose music stopped
 * has no card (owner's request, October 2026: a "Stopped" card read as clutter). `order` is
 * first-seen order and holds, so cards do not reorder as speakers come and go; `gone` are the ones
 * that joined this device's group, which makes them part of its own card.
 *
 * Paused is the reported state, or "idle with a title": Music Assistant pauses a Sendspin player by
 * stopping it, so a paused Satellite1 reports idle with its track still loaded - the signal MA's own
 * bar goes by, and this device's own card too (wsPausedOf). The relay lists no idle speakers at all,
 * so `held` covers the case that matters there: a speaker paused from its own card, here, keeps it,
 * rebuilt from `last` (its last row) once its row is gone. Kept cards say "paused" whatever the tier
 * reported, so the card offers Play.
 */
export function barCards({ rows, seen, order, last, gone, held }) {
  const by = new Map(rows.map((r) => [r.id, r]));
  const out = [];
  for (const id of order) {
    if (gone?.has(id)) continue;
    const r = by.get(id);
    if (r?.state === "playing") out.push(r);
    else if (r && seen.has(id) && (r.state === "paused" || held?.has(id) || (r.state === "idle" && !!r.title)))
      out.push(r.state === "paused" ? r : { ...r, state: "paused" });
    else if (!r && held?.has(id) && last?.[id]) out.push({ ...last[id], state: "paused" });
  }
  return out;
}

/**
 * Whether this device's group has music of its own that Take over or Move here would replace: on
 * the socket, a queue holding a current item (playing, paused, or a Sendspin pause, which reads as
 * idle); on the relay, playing, or not playing with a title (the payload's paused signal, as in
 * relayPausedOf). Those two ask first; with nothing to lose they act on one tap.
 */
export function hasOwnMusic({ wsOn, queue, live }) {
  if (wsOn) return queue?.state === "playing" || queue?.current_item != null;
  const st = live?.st;
  return st === "playing" || (!!live?.ti && (st === "paused" || st === "idle"));
}

/**
 * The relay's group-volume step. Home Assistant has no group volume command, so the whole group
 * moves by the step between its current average and `v`: each member's volume plus that step,
 * clamped to 0-100, as [id, volume] writes. Members without a volume are left alone.
 */
export function groupSteps(rows, v) {
  const known = rows.filter((r) => r[2] >= 0);
  if (!known.length) return [];
  const avg = known.reduce((n, r) => n + r[2], 0) / known.length;
  const step = v - avg;
  return known.map((r) => [r[0], Math.max(0, Math.min(100, Math.round(r[2] + step)))]);
}

/** Another speaker's playhead in ms, from the socket's elapsed_time at elapsed_time_last_updated,
 *  `skew` being how far this browser's clock runs ahead of the server's (seconds). */
export function rowClock(row, now, skew = 0) {
  const c = row?.clock;
  if (!c) return null;
  const dur = c.dur * 1000;
  const run = row.state === "playing" ? now - (c.at + skew) * 1000 : 0;
  return { dur, pos: Math.max(0, Math.min(dur, c.pos * 1000 + Math.max(0, run))) };
}

/** Pending group edits that the fresh rows have not confirmed yet: a join until its id appears, an
 *  unjoin until it is gone, either until the deadline. Settling on membership rather than on any
 *  fresh payload matters: a poll that landed before the unjoin would otherwise briefly resurrect the
 *  removed row. An edit to another speaker's group carries that group's leader as `lead`, and
 *  settles on `groups[lead]`, its rows. */
export function settleGroup(pending, raw, now, groups = {}) {
  return prune(pending, ([id, e]) => {
    const has = (e.lead ? groups[e.lead] || [] : raw).some((m) => m[0] === id);
    return (e.kind === "join" ? !has : has) && now - e.at < GROUP_PENDING_MAX_MS;
  });
}

/** "Kitchen +2": the first member and how many more, or `fallback` with no group to name. */
export const groupLabel = (members, fallback) =>
  members.length ? members[0][1] + (members.length > 1 ? ` +${members.length - 1}` : "") : fallback;

/** The socket queue's playhead in ms, or null when its current item has no duration. MA reports
 *  elapsed_time at elapsed_time_last_updated (s, re-anchored to this browser's clock by
 *  src/lib/ma.js). */
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

/** RGB in 0-1 to [h (degrees), s, l]. */
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
  return [h, s, l];
}

const HUE_BUCKETS = 12;

/**
 * The artwork's most vivid colour family, as [h, s, l], from RGBA pixel data (a canvas readback).
 * Averaging every pixel turns a red cover on black into brown, so pixels vote by chroma into hue
 * buckets and the strongest bucket, with its neighbours so a family split across a boundary still
 * counts as one, gives the colour. Greys, blacks and whites carry no chroma and so no vote; null
 * when the artwork has nothing else.
 */
export function vibrantHsl(data) {
  const b = Array.from({ length: HUE_BUCKETS }, () => ({ w: 0, x: 0, y: 0, s: 0, l: 0 }));
  for (let i = 0; i < data.length; i += 4) {
    if (data[i + 3] < 128) continue;
    const r = data[i] / 255;
    const g = data[i + 1] / 255;
    const bl = data[i + 2] / 255;
    const w = Math.max(r, g, bl) - Math.min(r, g, bl);
    if (w < 0.08) continue;
    const [h, s, l] = toHsl(r, g, bl);
    const k = b[Math.floor(h / (360 / HUE_BUCKETS)) % HUE_BUCKETS];
    k.w += w;
    k.x += Math.cos((h * Math.PI) / 180) * w;
    k.y += Math.sin((h * Math.PI) / 180) * w;
    k.s += s * w;
    k.l += l * w;
  }
  const at = (i) => b[(i + HUE_BUCKETS) % HUE_BUCKETS];
  let best = -1;
  let score = 0;
  for (let i = 0; i < HUE_BUCKETS; i++) {
    const sc = at(i - 1).w / 2 + at(i).w + at(i + 1).w / 2;
    if (sc > score) [best, score] = [i, sc];
  }
  if (best < 0) return null;
  const fam = [at(best - 1), at(best), at(best + 1)];
  const sum = (key) => fam.reduce((n, k) => n + k[key], 0);
  const w = sum("w");
  return [Math.round(((Math.atan2(sum("y"), sum("x")) * 180) / Math.PI + 360) % 360), sum("s") / w, sum("l") / w];
}

/**
 * The colour as the --tint wash behind the media bar and sheets. Saturation is floored so a muted
 * cover still gives the bar a colour, and lightness is held where the bar's text reads in each
 * theme - mid-dark under dark mode's light text, mid-light under light mode's dark text.
 */
export function tintOf(col, theme = "dark") {
  if (!col) return "transparent";
  const [h, s, l] = col;
  const light = theme === "light";
  const sat = Math.min(0.9, Math.max(0.45, s));
  const lit = light ? Math.min(0.62, Math.max(0.5, l)) : Math.min(0.55, Math.max(0.4, l));
  return `hsla(${Math.round(h)},${Math.round(sat * 100)}%,${Math.round(lit * 100)}%,${light ? 0.42 : 0.5})`;
}

/* ------------------------------------------------------------------ */
/* Search                                                              */
/* ------------------------------------------------------------------ */

/** The searchable types in the order their sections render (Music Assistant's own search modal's
 *  order), and the result keys they come back under (music/search answers plurals, except radio). */
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
/** How many rows an "All" section shows; one more is fetched so "Show more" appears only when there
 *  genuinely is more, and a filtered pill fetches a real page. */
export const ALL_SHOWN = 3;
const FILTERED_LIMIT = 25;

/** music/search's args for a query under a filter pill. The contract (verified against the MA
 *  frontend @ 367878c, September 2026): { search_query, media_types, limit } answers { tracks,
 *  artists, albums, playlists, radio, podcasts, audiobooks }, with `limit` applied per type. */
export const searchArgs = (query, filter) => ({
  search_query: query,
  media_types: filter === "all" ? SEARCH_TYPES : [filter],
  limit: filter === "all" ? ALL_SHOWN + 1 : FILTERED_LIMIT,
});

/** [type, items] per section with anything in it, in render order. */
export const searchGroups = (r, filter) =>
  (filter === "all" ? SEARCH_TYPES : [filter]).map((ty) => [ty, r?.[RESULT_KEY[ty]] || []]).filter(([, items]) => items.length > 0);

/** A result's thumbnail record in Music Assistant's own preference order (their
 *  getMediaItemImage): its own thumb, its album's (tracks wear their album), its metadata's, its
 *  first artist's. */
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
 *  through the legacy /imageproxy with MA's double encoding. 80px is the proxy's smallest size,
 *  exactly the 40px row at 2x; an opened item's header asks for more. No CORS concern: these are
 *  plain <img> loads, never canvas readbacks. */
export function artThumb(item, base, size = 80) {
  const img = findImage(item);
  if (!img?.path) return "";
  if (img.path.startsWith("data:image")) return img.path;
  if (img.proxy_id) return base ? `${base}/imageproxy/${img.proxy_id}?size=${size}` : "";
  if (img.remotely_accessible && /^https?:/i.test(img.path)) return img.path;
  if (!base) return "";
  const enc = encodeURIComponent(encodeURIComponent(img.path));
  return `${base}/imageproxy?path=${enc}&provider=${encodeURIComponent(img.provider || "")}&size=${size}`;
}

/** A row's sub-line: the kind by name (`kinds` maps media_type to a label), then its makers -
 *  artists joined the way MA's own modal joins them, a playlist's owner, an audiobook's authors. */
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

/** The kinds of result that open into a view of their own: an artist to its albums, an album or a
 *  playlist to its songs, a podcast to its episodes. Everything else plays or queues in place. */
const OPENS = new Set(["artist", "album", "playlist", "podcast"]);
export const canOpen = (item) => OPENS.has(item?.media_type);

/** How to list what an opened item holds, against Music Assistant's commands (verified against the
 *  server's controllers, October 2026). An album asks beyond the library so a provider album lists
 *  every track, not only the ones saved. */
export function openArgs(item) {
  const args = { item_id: item.item_id, provider_instance_id_or_domain: item.provider };
  switch (item.media_type) {
    case "artist":
      return ["music/artists/artist_albums", args];
    case "album":
      return ["music/albums/album_tracks", { ...args, in_library_only: false }];
    case "playlist":
      return ["music/playlists/playlist_tracks", args];
    case "podcast":
      return ["music/podcasts/podcast_episodes", args];
    default:
      return null;
  }
}

const SINGLES = new Set(["single", "ep"]);
const newestFirst = (a, b) => (b.year || 0) - (a.year || 0) || String(a.name).localeCompare(String(b.name));

/** An artist's albums as [key, albums] sections, newest first in each: "albums" (compilations and
 *  anything untyped included) and "singles" (singles and EPs), each only when it has any. */
export function albumSections(albums) {
  const main = (albums || []).filter((a) => !SINGLES.has(a.album_type)).sort(newestFirst);
  const singles = (albums || []).filter((a) => SINGLES.has(a.album_type)).sort(newestFirst);
  return [
    ["albums", main],
    ["singles", singles],
  ].filter(([, list]) => list.length > 0);
}

/** An album's line inside its artist's view, where the artist goes without saying: the year, or
 *  the kind when there is none. */
export const albumSub = (album, kinds) => (album.year ? String(album.year) : kinds.album || "");

/** An opened item's list in its natural order: an album by disc and track, a podcast newest
 *  episode first (by position, else by release date), a playlist as its owner ordered it. */
export function openedOrder(kind, items) {
  const list = Array.isArray(items) ? [...items] : [];
  if (kind === "album") list.sort((a, b) => (a.disc_number || 0) - (b.disc_number || 0) || (a.track_number || 0) - (b.track_number || 0));
  if (kind === "podcast") {
    const date = (e) => Date.parse(e.metadata?.release_date || "") || 0;
    list.sort((a, b) => (b.position || 0) - (a.position || 0) || date(b) - date(a));
  }
  return list;
}

/** "Fleetwood Mac · 1977 · 11 songs": an opened item's header line from its kind, makers, year and
 *  how many it holds (`n`, null while loading). `t` is the copy: search_kind and the count forms. */
export function heroSub(item, n, t) {
  const count = (one, many) => (n == null ? "" : n === 1 ? one : many.replace("%s", String(n)));
  const artists = item.artists?.length ? item.artists.map((a) => a.name).join(" | ") : "";
  const parts = {
    artist: [t.search_kind.artist, count(t.search_album_1, t.search_albums_n)],
    album: [artists || t.search_kind.album, item.year ? String(item.year) : "", count(t.search_song_1, t.search_songs_n)],
    playlist: [t.search_kind.playlist, item.owner || "", count(t.search_song_1, t.search_songs_n)],
    podcast: [t.search_kind.podcast, item.publisher || "", count(t.search_episode_1, t.search_episodes_n)],
  }[item.media_type] || [t.search_kind[item.media_type] || ""];
  return parts.filter(Boolean).join(" \u00b7 ");
}

/**
 * A podcast episode's facts for its row: the release date ("Oct 2", with the year when it is not
 * this one), its length in whole minutes, and how far through it is - `left` minutes when Music
 * Assistant holds a resume point, `played` once finished. Empty fields are 0 or "".
 */
export function episodeSub(ep, now = Date.now()) {
  const d = ep?.metadata?.release_date ? new Date(ep.metadata.release_date) : null;
  const ok = d && !Number.isNaN(d.getTime());
  const opts = { month: "short", day: "numeric" };
  if (ok && d.getFullYear() !== new Date(now).getFullYear()) opts.year = "numeric";
  const date = ok ? d.toLocaleDateString(undefined, opts) : "";
  const mins = ep?.duration > 0 ? Math.max(1, Math.round(ep.duration / 60)) : 0;
  const played = ep?.fully_played === true;
  const left =
    !played && ep?.resume_position_ms > 0 && ep?.duration > 0
      ? Math.max(1, Math.ceil((ep.duration * 1000 - ep.resume_position_ms) / 60000))
      : 0;
  return { date, mins, left, played };
}

/** The recent-searches list with `query` on top, deduped without regard to case. */
export function addRecent(list, query, max) {
  const t = String(query || "").trim();
  if (!t) return list;
  return [t, ...list.filter((r) => r.toLowerCase() !== t.toLowerCase())].slice(0, max);
}
