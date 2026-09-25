/**
 * The media footer: a floating bar under every route, in the album's colour, expanding to a
 * full-screen view, with a slide-up players panel for the group.
 *
 * This replaced the Media card on the home page (owner, Sept 2026) and was reshaped after Music
 * Assistant's own mobile player (owner's screenshots, later that month). One media surface instead
 * of a card that only home had: the bar rides the shell so a track playing in the kitchen is
 * visible and pausable from Diagnostics - artwork and title open the expanded view, the speaker
 * button opens the players panel, and the bar carries its own volume row. The expanded view is
 * where the controls that would crowd a bar live - artwork, the scrubber, shuffle and repeat -
 * and the players panel is where the group's speakers are added, removed and mixed.
 *
 * What appears is layered by what can answer, and each layer only ever adds:
 *
 *  - The device alone (Sendspin): title, artist, album and artwork URL ride GET /api/sat1/media
 *    from the hub's metadata role, as do position, duration, shuffle and repeat state and the
 *    server's supported-command list. Transport, volume, shuffle and repeat all write locally.
 *  - Home Assistant reachable: the grouped-speakers section with per-speaker volumes and a
 *    draggable scrubber - each a relayed action call. (A like button rode this tier until
 *    September 2026, when the owner cut favorites app-wide.)
 *  - Music Assistant connected directly (a token this browser holds): the same controls answered in
 *    real time over its WebSocket, with no device polling behind them.
 *
 * The held-pause rule moved here intact from the card: the Sendspin protocol has no paused state -
 * playing or stopped is its whole vocabulary - so the moment pause lands, the device sees an idle
 * player and would report the footer should go idle, stranding the resume it just promised. The
 * footer is the only witness that the group is resumable, so pausing sets `held`, which keeps the
 * paused-group controls up and aims every command at the group player by name (`src=`). Cleared
 * when anything else starts playing, or when the group resumes.
 *
 * A pause made anywhere else - Music Assistant's own UI, another Satellite's footer - used to show
 * idle here, because nothing on this device can tell it apart from a stream that ended. The upper
 * tiers can (owner's screenshots, September 2026: MA's bar holding the paused track over this
 * footer's "Nothing playing"), though not the way one would hope: pausing a Sendspin player stops
 * the stream, so the MA *player* - and with it the Home Assistant entity - reports idle, never
 * paused (verified against the integration source and a live pause). What MA's own bar actually
 * shows is the *queue's* current item, which survives the pause as the resume point. So that is
 * the signal both tiers now read: the relay payload carries the entity's state and media_title
 * (`st`/`ti` - Home Assistant mirrors the queue's current item as media_title for every playback
 * scenario), and the socket reads the queue's current_item directly. "Not playing, but still
 * holding a track" props the paused-group card up exactly as `held` does - same controls, same
 * src= aiming, same resume - and the moment the group stream stops, one relayed sync asks the
 * question the device cannot answer alone: paused or ended? While a relay-reported pause is the
 * only thing holding the card up, the ask repeats slowly, so a queue cleared from elsewhere goes
 * dark here within a minute rather than never - the same moment MA's own bar goes dark, which is
 * the parity this exists for. With no tier to answer, everything degrades to the held-pause
 * behaviour above, honestly.
 *
 * Track metadata renders only while the group stream owns the footer, because a stopped stream
 * keeps its last track in the device's cache and showing it would caption silence. The held or
 * reported pause keeps it: that track is exactly what resume will continue.
 */
import { useEffect, useMemo, useRef, useState } from "preact/hooks";

import { HINTS, TEXT } from "./copy.js";
import { useMaData, useMedia } from "./lib/device.js";
import { maCfgFetch, maCfgSave, maSettings, scanForMa, useMaSocket } from "./lib/ma.js";
import { SearchDrawer } from "./search.jsx";
import { Chevron, rangeFill, useDrawer, useHeld, useSheetDrag } from "./ui.jsx";

const MEDIA_STATE = { 2: "Playing", 3: "Paused", 4: "Announcing" };

/** How stale, in seconds, the relay payload may grow while its "paused" claim still props the
 *  paused-group card up. Counted from when Home Assistant rendered it (`age`) plus how long this
 *  browser has held it (`at`), because both halves go stale independently. Two minutes spans the
 *  recheck cadence below with room for a slow round trip, and is short enough that a relay that
 *  stopped answering takes the card down rather than captioning silence indefinitely. */
const MA_PAUSED_TRUST_S = 120;

/** How often the relay is re-asked while its pause report is the only thing holding the card up.
 *  One action call a minute, roughly, and only for as long as a paused group is actually showing -
 *  the price of a queue cleared from elsewhere going dark here within a minute rather than never. */
const MA_PAUSED_RECHECK_MS = 45000;

/** Bit numbers of the Sendspin controller-command enum, exactly as `sup` carries them. Only the
 *  bits that decide whether a control renders; the rest are commands the footer always offers. */
const SUP = { NEXT: 1 << 3, PREV: 1 << 4, SHUFFLE: 1 << 10 };

/** How long a transport command's pending ring may ride before giving up - useHeld's five seconds,
 *  the app-wide escape for a write the device refused or dropped. Transport confirmation arrives on
 *  the 1s playing-cadence poll, so a healthy command settles in one or two ticks; the deadline only
 *  exists so a dead device cannot leave a ring spinning forever. */
const PENDING_MAX_MS = 5000;

/** The group edits' own deadline, deliberately longer than PENDING_MAX_MS: a relayed join/unjoin is
 *  confirmed by a resync that legitimately takes 2-7s (the 2.2s poke, the ~6s panel cycle), so five
 *  seconds would routinely give up on healthy edits. The socket tier clears in well under a second
 *  regardless; twelve is only how long a failure takes to unwind. */
const GROUP_PENDING_MAX_MS = 12000;

const fmtTime = (ms) => {
  const s = Math.max(0, Math.floor(ms / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
};

/* ------------------------------------------------------------------ */
/* Icons - geometry, not codepoints, for the reason ui.jsx's Chevron gives: nothing here ships a
   webfont, and emoji shuffle/repeat/heart arrive coloured and inconsistent. */
/* ------------------------------------------------------------------ */

const mi = (path, extra) => (
  <svg class="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    {path}
    {extra}
  </svg>
);

const I_SHUFFLE = mi(
  <path d="M1.5 4.5h2.6c3.6 0 4.2 7 7.8 7h2.1M1.5 11.5h2.6c1.3 0 2.2-.9 2.9-2M14 4.5h-2.1c-1.4 0-2.3 1-3 2.1" />,
  <path d="M12.2 2.7l1.8 1.8-1.8 1.8M12.2 9.7l1.8 1.8-1.8 1.8" />,
);
const I_REPEAT = mi(
  <path d="M2.5 6.5v-.4A2.6 2.6 0 0 1 5.1 3.5h7.4M13.5 9.5v.4a2.6 2.6 0 0 1-2.6 2.6H3.5" />,
  <path d="M10.8 1.8 12.6 3.5l-1.8 1.7M5.2 14.2 3.4 12.5l1.8-1.7" />,
);
/** A note in a rounded square: the artwork placeholder, and the bar's stand-in thumbnail. */
const I_NOTE = (
  <svg class="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M11.8 1.6 6.4 3v7.2a2.3 2.3 0 1 0 1.2 2V6.2l4.2-1.1v3.8a2.3 2.3 0 1 0 1.2 2V1.9a.3.3 0 0 0-.4-.3H11.8Z" />
  </svg>
);

/* Transport glyphs, drawn to match Music Assistant's own player bar (owner's reference screenshot,
   September 2026): filled triangles for play and the skip arrows, rounded bars for pause, the skip
   bar as a stroke. Replaces the ⏮ ▶ ⏸ ⏭ codepoints, which rendered as coloured emoji on iOS. */
const I_PLAY = (
  <svg class="mi" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true">
    <path d="M5.4 3c0-.5.6-.9 1-.6l7 4.5c.4.3.4.9 0 1.2l-7 4.5c-.4.3-1 0-1-.6V3Z" />
  </svg>
);
const I_PAUSE = (
  <svg class="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" aria-hidden="true">
    <path d="M5.4 3.6v8.8M10.6 3.6v8.8" />
  </svg>
);
const I_PREV = (
  <svg class="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true">
    <path d="M3.7 3.6v8.8" />
    <path fill="currentColor" stroke="none" d="M12.9 3.9v8.2c0 .5-.6.8-1 .5L6.3 8.5a.6.6 0 0 1 0-1l5.6-4.1c.4-.3 1 0 1 .5Z" />
  </svg>
);
const I_NEXT = (
  <svg class="mi" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true">
    <path d="M12.3 3.6v8.8" />
    <path fill="currentColor" stroke="none" d="M3.1 3.9v8.2c0 .5.6.8 1 .5l5.6-4.1a.6.6 0 0 0 0-1L4.1 3.4c-.4-.3-1 0-1 .5Z" />
  </svg>
);
/** A speaker cabinet, for the bar button that opens the players panel - the glyph MA's bar uses. */
const I_SPK = mi(
  <rect x="4" y="1.8" width="8" height="12.4" rx="1.6" />,
  <>
    <circle cx="8" cy="10.3" r="2.1" />
    <circle cx="8" cy="4.9" r="0.4" />
  </>,
);
/** A plus, for the add-speaker rows that replaced the native select in the players panel. */
const I_PLUS = mi(<path d="M8 3.5v9M3.5 8h9" />);
/** A minus, for the remove control on each member row - the visual inverse of the + above, and
 *  drawn in the same hairline circle, so the pair reads as add/remove without a legend. Replaced
 *  the bare ✕ (owner's request, September 2026), which read as "close" rather than "remove". */
const I_MINUS = mi(<path d="M3.5 8h9" />);
/** A speaker cone with a wave, marking the bar's second row as a volume (owner's report, September
    2026: without a label the slider read as a scrubber). Same geometry as ui.jsx's Audio glyph. */
const I_VOL = mi(
  <path d="M2.8 6.4h2.4L8.6 3.8v8.4L5.2 9.6H2.8z" />,
  <path d="M11.2 6a3.1 3.1 0 0 1 0 4" />,
);

/* ------------------------------------------------------------------ */
/* The artwork's colour                                                */
/* ------------------------------------------------------------------ */

/**
 * The bar and the expanded view wear the album's colour, the way Music Assistant's own player does
 * (owner's screenshots, September 2026). The colour is the artwork averaged through a 6x6 canvas -
 * cheap, no library - then nudged in HSL: saturation up a step because averaging mudddies it,
 * lightness clamped to where both a light and a dark glyph set can sit on it.
 *
 * This only works if the artwork host lets this origin read pixels back (crossOrigin plus a CORS
 * header on MA's /imageproxy). When it does not, getImageData throws on the tainted canvas, the
 * colour stays null, and every tinted surface falls back to theme colours through the CSS
 * `var(--tint, ...)` fallbacks - the whole feature degrades to exactly the old look.
 */
function useArtColor(art) {
  const [col, setCol] = useState(null);
  useEffect(() => {
    setCol(null);
    if (!art) return undefined;
    let live = true;
    const img = new Image();
    img.crossOrigin = "anonymous";
    img.onload = () => {
      if (!live) return;
      try {
        const c = document.createElement("canvas");
        c.width = c.height = 6;
        const g = c.getContext("2d");
        g.drawImage(img, 0, 0, 6, 6);
        const d = g.getImageData(0, 0, 6, 6).data;
        let r = 0;
        let gr = 0;
        let b = 0;
        for (let i = 0; i < d.length; i += 4) {
          r += d[i];
          gr += d[i + 1];
          b += d[i + 2];
        }
        const n = d.length / 4;
        setCol(toHsl(r / n / 255, gr / n / 255, b / n / 255));
      } catch {
        /* tainted canvas - artwork host without CORS; the tint just stays off */
      }
    };
    img.src = art;
    return () => {
      live = false;
    };
  }, [art]);
  return col;
}

function toHsl(r, g, b) {
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

/**
 * The colour as inline custom properties, or null for "no tint". --tint is the surface, --tfg and
 * --tdim the text on it, --tbtn/--tbfg the filled play circle - a dark block on a light tint and a
 * light one on a dark tint, which is how MA's own bar keeps the pause readable on any album.
 */
function tintStyle(col) {
  if (!col) return null;
  const [h, s, l] = col;
  const hs = `${h | 0},${(s * 100) | 0}%`;
  const dark = l < 0.5;
  return (
    `--tint:hsl(${hs},${(l * 100) | 0}%);` +
    // The same colour ~10 points darker, the bottom of the expanded view's gradient - depth the flat
    // wash did not have, clamped so a near-black album cannot fall off the bottom.
    `--tint2:hsl(${hs},${Math.max(8, ((l * 100) | 0) - 10)}%);` +
    `--tfg:${dark ? "#fff" : `hsl(${hs},13%)`};` +
    `--tdim:${dark ? "rgba(255,255,255,.72)" : `hsla(${hs},16%,.72)`};` +
    `--tbtn:hsl(${hs},${dark ? 85 : 16}%);` +
    `--tbfg:${dark ? `hsl(${hs},18%)` : "#fff"}`
  );
}

/* ------------------------------------------------------------------ */
/* Shared model                                                       */
/* ------------------------------------------------------------------ */

/**
 * Commands awaiting their echo in the polled payload: { key: { done, at } }.
 *
 * Music Assistant's own player rings the acting button while the *server's* in-progress flag is up
 * (PlayBtn.vue's `play_action_in_progress`), but /api/sat1/media carries no such flag and the
 * command POST's answer carries nothing at all - the next poll is the only confirmation that
 * exists. So the lifecycle is reconstructed client-side: an entry starts on click (the honest
 * "processing began" moment - the POST is fire-and-forget behind it), and settles on the first
 * payload whose per-command `done` predicate passes, or at the deadline - whichever is first, so a
 * ring can never spin forever even against a dead device.
 */
function usePendingCmds(media) {
  const [pending, setPending] = useState({});

  // Every fresh payload settles whatever it can; the deadline settles the rest.
  useEffect(() => {
    setPending((p) => {
      const now = Date.now();
      const kept = Object.entries(p).filter(([, e]) => !e.done(media) && now - e.at < PENDING_MAX_MS);
      return kept.length === Object.keys(p).length ? p : Object.fromEntries(kept);
    });
  }, [media]);

  // The deadline must also fire between payloads: polls run 5s apart while idle, and a device that
  // stopped answering sends no payload at all - the effect above never runs on silence.
  useEffect(() => {
    const ats = Object.values(pending).map((e) => e.at);
    if (!ats.length) return undefined;
    const wait = Math.max(50, PENDING_MAX_MS - (Date.now() - Math.min(...ats)) + 30);
    const t = setTimeout(() => {
      setPending((p) => {
        const now = Date.now();
        const kept = Object.entries(p).filter(([, e]) => now - e.at < PENDING_MAX_MS);
        return kept.length === Object.keys(p).length ? p : Object.fromEntries(kept);
      });
    }, wait);
    return () => clearTimeout(t);
  }, [pending]);

  const start = (key, done) => setPending((p) => ({ ...p, [key]: { done, at: Date.now() } }));
  return [pending, start];
}

/** The card's logic, verbatim where it could be: which source owns the footer, the held pause, and
 *  where each command must land. `maPaused` is the upper tiers' word that the group is paused (the
 *  file comment's second pause path); it joins `held` rather than replacing it, because each covers
 *  the other's blind spot - `held` works with no tier answering, the tiers work across browsers. */
function useMediaModel(maPaused) {
  const { media, mediaCmd, mediaPoke } = useMedia(true);
  const [held, setHeld] = useState(false);

  // One pending map for both surfaces: the bar's play button and the sheet's send the same
  // command, so they must ring together. Starting an entry also pulls the next poll forward -
  // an idle-cadence poll is 5s away, the ring's whole deadline.
  const [pending, startCmd] = usePendingCmds(media);
  const startPending = (key, done) => {
    startCmd(key, done);
    mediaPoke();
  };

  const deviceActive = media ? media.state === 2 || media.state === 3 : false;
  const ssPlaying = media?.ss_state === 2;

  // The hold ends when the world moves on: the group resumed (from here or anywhere), or the local
  // player started making sound, which means the next idle is that stream ending rather than our pause.
  useEffect(() => {
    if (ssPlaying || (media?.src === "local" && deviceActive)) setHeld(false);
  }, [ssPlaying, media?.src, deviceActive]);

  // maPaused needs no clearing of its own: it is derived from the tiers' live view, so the group
  // resuming or the queue being cleared is what makes it false, wherever that happens.
  const groupHeld = (held || maPaused) && !deviceActive && media?.ss_state != null;
  const sendspin = groupHeld || media?.src === "sendspin";
  const playing = !groupHeld && media?.state === 2;
  const active = deviceActive || groupHeld;
  const announcing = media?.state === 4;
  const srcParam = sendspin ? "sendspin" : "local";

  // Metadata is the group stream's; see the file comment for why it hides otherwise.
  const showMeta = sendspin && active;
  const title = showMeta ? media?.title : "";
  const artist = showMeta ? media?.artist : "";
  const album = showMeta ? media?.album : "";
  const art = showMeta ? media?.art : "";

  // The pause predicate on the sendspin path reads ss_state, not the derived state: pausing a
  // group sets `held` before the POST, which flips the derived view to "Paused" instantly - the
  // raw stream state is what actually confirms the command landed.
  const playPause = () => {
    if (playing) {
      if (sendspin) setHeld(true);
      startPending("play", sendspin ? (m) => m?.ss_state !== 2 : (m) => m?.state !== 2);
      mediaCmd("pause", { src: srcParam });
    } else {
      startPending("play", sendspin ? (m) => m?.ss_state === 2 : (m) => m?.state === 2);
      mediaCmd("play", { src: srcParam });
    }
  };

  const stateText = groupHeld ? MEDIA_STATE[3] : MEDIA_STATE[media?.state];
  const srcText = sendspin ? TEXT.media_src_group : TEXT.media_src_local;

  return {
    media,
    mediaCmd,
    groupHeld,
    sendspin,
    playing,
    active,
    announcing,
    srcParam,
    title,
    artist,
    album,
    art,
    playPause,
    pending,
    startPending,
    stateText,
    srcText,
  };
}

/**
 * The playhead between polls. The device reports position once per poll; while playing, the gap
 * since that report is added on a half-second tick, so the scrubber moves like a clock rather than
 * lurching once a second. Re-anchored by every poll, so drift is bounded by one poll interval.
 */
function usePlayhead(media, playing, live) {
  const anchor = useMemo(() => ({ pos: media?.pos ?? 0, at: Date.now() }), [media]);
  const [, tick] = useState(0);

  useEffect(() => {
    if (!live || !playing) return;
    const t = setInterval(() => tick((n) => n + 1), 500);
    return () => clearInterval(t);
  }, [live, playing]);

  const dur = media?.dur ?? 0;
  let pos = anchor.pos + (playing ? Date.now() - anchor.at : 0);
  if (dur > 0 && pos > dur) pos = dur;
  return { pos, dur };
}

/**
 * The two upper tiers, hoisted out of the expanded view so the bar (its badge), the expanded view
 * (seek, like) and the players panel (the group) share one copy of the state.
 *
 * The Music Assistant socket: a connection this browser holds straight to the MA server, configured
 * in the panel at the bottom of the expanded view. It now lives for the whole page rather than only
 * while a sheet is open - the badge and the bar want real-time membership too. When it is up it
 * answers everything the relay does; when not, everything falls back without a seam.
 *
 * The Home Assistant tier: `disc` is the discovery block riding the big payload (this device's own
 * MA player, its favorite button, the join candidates); `ma`/`live` is the fast-moving view the
 * device relays. `wake` gates the relay's polling to while something showing it is open - each poll
 * is an action call on the device - with the single mount-time read inside useMaData covering the
 * badge.
 */
function useTiers(ha, mac, wake) {
  const [maCfg, setMaCfg] = useState(maSettings.get);
  const ws = useMaSocket(mac, maCfg.url && maCfg.token ? maCfg : null);
  const wsOn = ws.status === "on" && !!ws.me;

  // A browser with no stored connection asks the device for its copy, once per mount (AppInner
  // remounts per device, which is the right cadence - the copy is per-device). This is what makes
  // "set up once, works on every phone" true: the first browser saved here through MaPanel, and
  // every later one seeds itself. localStorage keeps winning when it has an answer, so a browser
  // that deliberately points at a different server is not overwritten.
  useEffect(() => {
    if (maCfg.url && maCfg.token) return undefined;
    let live = true;
    maCfgFetch().then((cfg) => {
      if (!live || !cfg) return;
      maSettings.set(cfg.url, cfg.token);
      setMaCfg(cfg);
    });
    return () => {
      live = false;
    };
    // Mount-time seeding only: maCfg gaining a value is this effect's own doing.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const disc = ha?.d?.ma;
  const me = disc?.e || "";
  const { ma, maCmd, maRead, maAsk } = useMaData(!!me && !wsOn && wake);
  const live = ma?.d?.e ? ma.d : null;

  // A group edit shows up only after the device's own resync lands (~1s after the action call), so
  // schedule one early read rather than waiting out the poll cycle.
  const poke = () => setTimeout(maRead, 2200);

  // Optimistic per-member volume, cleared whenever a fresh payload arrives, keyed because the group
  // is a list. On the socket tier the clearing event is a player update, arriving within a moment.
  const [optVol, setOptVol] = useState({});
  useEffect(() => {
    setOptVol({});
  }, [ma, ws.players]);

  // Group edits in flight: { id: { kind: "join" | "unjoin", at } }. A pending join keeps its
  // addable row visible with a spinning ring (Plan 12's mechanism, a payload-confirmed pending
  // entry); a pending unjoin keeps the member row *hidden* - the instant optimistic hide the old
  // optGone list did, which the owner kept (September 2026) over a lingering ringed row.
  const [pendingGroup, setPendingGroup] = useState({});

  // The group as rows of [id, name, volume], from whichever tier is answering. The ids differ in
  // kind - MA player ids on the socket, Home Assistant entity ids through the relay - which is
  // fine, because the commands below come from the same tier as the rows they act on.
  const wsMemberIds = wsOn ? (ws.me.group_members?.length ? ws.me.group_members : [ws.me.player_id]) : null;
  const rawMembers = wsOn
    ? wsMemberIds.map((id) => {
        const p = ws.players[id];
        return [id, p?.name || id, p?.volume_level ?? -1];
      })
    : live?.g || [];

  // Clearing is membership-confirmation, not any-payload (which is what the old optGone did, and
  // what let a poll that landed before the unjoin briefly resurrect the removed row): a join
  // settles once the id appears in the fresh members list, an unjoin once it no longer does, and
  // anything older than the deadline settles regardless.
  useEffect(() => {
    setPendingGroup((p) => {
      if (!Object.keys(p).length) return p;
      const now = Date.now();
      const has = (id) => rawMembers.some((m) => m[0] === id);
      const kept = Object.entries(p).filter(
        ([id, e]) => (e.kind === "join" ? !has(id) : has(id)) && now - e.at < GROUP_PENDING_MAX_MS,
      );
      return kept.length === Object.keys(p).length ? p : Object.fromEntries(kept);
    });
    // rawMembers is derived from exactly these two payload sources; the payloads are the events.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ma, ws.players]);

  // The deadline must also fire without a payload - a socket that went quiet sends nothing for the
  // effect above to run on, and a ring (or a hidden row) must never outlive a failed edit.
  useEffect(() => {
    const ats = Object.values(pendingGroup).map((e) => e.at);
    if (!ats.length) return undefined;
    const wait = Math.max(50, GROUP_PENDING_MAX_MS - (Date.now() - Math.min(...ats)) + 30);
    const t = setTimeout(() => {
      setPendingGroup((p) => {
        const now = Date.now();
        const kept = Object.entries(p).filter(([, e]) => now - e.at < GROUP_PENDING_MAX_MS);
        return kept.length === Object.keys(p).length ? p : Object.fromEntries(kept);
      });
    }, wait);
    return () => clearTimeout(t);
  }, [pendingGroup]);

  // Alphabetical on both tiers (owner's request, September 2026): the socket reports members in
  // join order and the relay in payload order, and neither order means anything to the person
  // scanning the list for a room's name.
  const members = rawMembers
    .filter(([id]) => pendingGroup[id]?.kind !== "unjoin")
    .sort((a, b) => a[1].localeCompare(b[1]));
  const addables = (
    wsOn
      ? Object.values(ws.players)
          .filter((p) => p.available && p.player_id !== ws.me.player_id && !wsMemberIds.includes(p.player_id))
          .map((p) => [p.player_id, p.name])
      : (disc?.c || []).filter(([id]) => !members.some((m) => m[0] === id))
  ).sort((a, b) => a[1].localeCompare(b[1]));

  // Group commands, aimed at whichever tier produced the rows. The socket's promise failures are
  // swallowed: the optimistic row has already moved, the next player update is the truth, and the
  // relay path surfaces its own failures through the write toast.
  const gVol = (id, v) => {
    setOptVol((prev) => ({ ...prev, [id]: v }));
    if (wsOn) ws.cmd("players/cmd/volume_set", { player_id: id, volume_level: v }).catch(() => {});
    else {
      maCmd("vol", { e: id, v });
      poke();
    }
  };
  const gUnjoin = (id) => {
    setPendingGroup((p) => ({ ...p, [id]: { kind: "unjoin", at: Date.now() } }));
    if (wsOn) ws.cmd("players/cmd/ungroup", { player_id: id }).catch(() => {});
    else {
      maCmd("unjoin", { e: id });
      poke();
    }
  };
  const gJoin = (id) => {
    setPendingGroup((p) => ({ ...p, [id]: { kind: "join", at: Date.now() } }));
    if (wsOn) ws.cmd("players/cmd/group", { player_id: id, target_player: ws.me.player_id }).catch(() => {});
    else {
      maCmd("join", { e: me, m: id });
      poke();
    }
  };

  // The favorites feature lived here until September 2026, when the owner cut it app-wide: a press
  // had no readable echo in any payload, so the heart could only ever claim "sent", never "stored".

  return {
    maCfg,
    setMaCfg,
    ws,
    wsOn,
    disc,
    me,
    maCmd,
    // The raw relay payload rides out beside `live` because the paused-group logic needs the parts
    // `live` drops: `age` and `at`, which decide whether a "paused" claim is fresh enough to trust.
    ma,
    maAsk,
    live,
    members,
    addables,
    optVol,
    pendingGroup,
    gVol,
    gUnjoin,
    gJoin,
  };
}

/* ------------------------------------------------------------------ */
/* Pieces                                                             */
/* ------------------------------------------------------------------ */

/**
 * The scrubber. Local state while dragging so the extrapolated playhead does not fight the thumb,
 * one commit on release - the Slider discipline, but with the time labels the generic component
 * has no room for. Read-only until something can actually seek: the device's own protocol offers
 * no seek passthrough, so the drag needs Home Assistant or Music Assistant behind it.
 */
function SeekBar({ pos, dur, canSeek, onSeek }) {
  const [drag, setDrag] = useState(null);
  // The generous tolerance is for the relay tier: the position that comes back lands near the seek,
  // not on it, and the hold should end on "close enough" rather than riding out the timeout.
  const [base, hold] = useHeld(pos, 2500);
  const shown = drag ?? base;

  return (
    <div class="mseek">
      <span class="num">{fmtTime(shown)}</span>
      <input
        type="range"
        min={0}
        max={dur}
        step={1000}
        value={shown}
        disabled={!canSeek}
        aria-label="Seek"
        style={rangeFill(shown, 0, dur)}
        onInput={(e) => setDrag(Number(e.currentTarget.value))}
        onChange={(e) => {
          const v = Number(e.currentTarget.value);
          if (canSeek) {
            hold(v);
            onSeek(v);
          }
          setDrag(null);
        }}
      />
      <span class="num">{fmtTime(dur)}</span>
    </div>
  );
}

/** A volume slider that previews locally and writes once on release, as every slider here does. */
function Vol({ value, onCommit, label }) {
  const [drag, setDrag] = useState(null);
  const [base, hold] = useHeld(value, 1);
  const shown = drag ?? base;
  return (
    <div class="mvol">
      <input
        type="range"
        min={0}
        max={100}
        step={1}
        value={shown}
        aria-label={label || "Volume"}
        style={rangeFill(shown, 0, 100)}
        onInput={(e) => setDrag(Number(e.currentTarget.value))}
        onChange={(e) => {
          const v = Number(e.currentTarget.value);
          hold(v);
          setDrag(null);
          onCommit(v);
        }}
      />
      <span class="num mvol-val">{`${shown}%`}</span>
    </div>
  );
}

function Artwork({ art, big }) {
  // Keyed by URL so a track change swaps the element rather than letting the old image linger
  // while the new one loads over a slow LAN.
  return (
    <div class={big ? "mart" : "mbar-art"}>
      {art ? <img key={art} src={art} alt="" loading="lazy" /> : I_NOTE}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The expanded view                                                  */
/* ------------------------------------------------------------------ */

function MediaSheet({ model, tiers, tint, ip, onClose, onPlayers }) {
  const { media, mediaCmd, sendspin, playing, active, announcing, srcParam, pending, startPending } = model;
  const { ws, wsOn, me, maCmd, maCfg, setMaCfg, members } = tiers;

  // A drawer like the players panel now (owner's request, September 2026): Escape and the
  // one-drawer rule via useDrawer, the finger-following swipe-down via useSheetDrag.
  useDrawer("media", true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);

  const { pos, dur } = usePlayhead(media, playing, true);

  // The scrubber and the playhead, upgraded by the socket: the queue's clock arrives with every
  // time tick, so the position is real rather than extrapolated from a poll.
  const wsQueue = wsOn && ws.queue?.current_item?.duration ? ws.queue : null;
  const sDur = wsQueue ? wsQueue.current_item.duration * 1000 : dur;
  let sPos = pos;
  if (wsQueue) {
    sPos = wsQueue.elapsed_time * 1000 + (wsQueue.state === "playing" ? Date.now() - wsQueue.elapsed_time_last_updated * 1000 : 0);
    sPos = Math.max(0, Math.min(sDur, sPos));
  }
  const canSeek = wsQueue ? true : !!me;
  const seekTo = (v) => {
    const t = Math.round(v / 1000);
    if (wsQueue) ws.cmd("player_queues/seek", { queue_id: wsQueue.queue_id, position: t }).catch(() => {});
    else maCmd("seek", { e: me, t });
  };

  // Optimistic shuffle and repeat: the next poll is up to a second away and a toggle that sits
  // unmoved for that second reads as refused. The override clears the moment the device reports
  // any value again, which is also what unwinds a write the server refused.
  const [optShuffle, setOptShuffle] = useState(null);
  const [optRepeat, setOptRepeat] = useState(null);
  useEffect(() => setOptShuffle(null), [media?.shuffle]);
  useEffect(() => setOptRepeat(null), [media?.repeat]);

  const ctrl = media?.shuffle != null;
  const shuffle = optShuffle ?? media?.shuffle === 1;
  const repeat = optRepeat ?? media?.repeat ?? 0;
  const sup = media?.sup;
  const supHas = (bit) => sup == null || (sup & bit) !== 0;

  // Both feedback mechanisms run together: the optimistic value moves the icon ("to here"), the
  // pending ring says "working" until the poll echoes the value - and both settle on the same
  // payload, so they cannot disagree.
  const toggleShuffle = () => {
    const next = shuffle ? 0 : 1;
    setOptShuffle(!shuffle);
    startPending("shuffle", (m) => m?.shuffle === next);
    mediaCmd("shuffle", { v: next, src: "sendspin" });
  };
  // One button cycling off -> all -> one -> off, which is the order the three are reached for.
  const cycleRepeat = () => {
    const next = repeat === 0 ? 2 : repeat === 2 ? 1 : 0;
    setOptRepeat(next);
    startPending("repeat", (m) => m?.repeat === next);
    mediaCmd("repeat", { m: ["off", "one", "all"][next], src: "sendspin" });
  };

  // Prev/next have no queue index to watch, so "done" is a heuristic: the track identity moved
  // (title changed), or the position fell below where it stood at the click - which is also what
  // prev does mid-track, restarting the same song. Two identical consecutive tracks ride to the
  // deadline: a slightly long spin, never a wrong state.
  const skip = (key) => {
    const t0 = media?.title;
    const p0 = media?.pos ?? 0;
    startPending(key, (m) => m?.title !== t0 || (m?.pos ?? 0) < p0);
    mediaCmd(key, { src: srcParam });
  };

  const showTransport = active && !announcing;
  const showSkips = sendspin;

  // The chip at the bottom naming who is playing, MA's own furniture: the first member plus a
  // count, opening the players panel. Before any tier has answered it just says "Players".
  const chip = members.length ? members[0][1] + (members.length > 1 ? ` +${members.length - 1}` : "") : TEXT.media_players_title;

  return (
    <>
      {/* Its own scrim now that the sheet no longer reaches the top of the screen: the sliver of
          page above the drawer dims rather than staying live, and tapping it closes - the same
          contract as the players panel's. */}
      <div class="scrim msheet-scrim" onClick={onClose} />
      <div class="msheet" style={(tint || "") + dragStyle} {...drag}>
        {/* Just the handle on top (owner's request, September 2026, twice): first the ✕/title/caret
            header went - a drawer with a handle, a scrim and a swipe does not also need three close
            buttons in a row under a rule - and then the "Media" label with its ⓘ went too. The
            artwork announces what this drawer is; a caption above it was furniture. */}
        <button class="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />

      <div class="msheet-body">
        <Artwork art={model.art} big />

        <div class="mmeta">
          <div class="mmeta-t">{model.title || (active ? "" : TEXT.media_idle_bar)}</div>
          {model.artist && <div class="mmeta-a dim">{model.artist}</div>}
          {model.album && <div class="mmeta-al dim">{model.album}</div>}
          <div class="dim xs">{model.stateText ? `${model.stateText} \u00b7 ${model.srcText}` : TEXT.media_idle}</div>
        </div>

        {sDur > 0 && sendspin && active && (
          // Draggable once anything can seek: the device's own protocol offers no seek
          // passthrough, so the drag lands on the socket or as a relayed media_seek.
          <SeekBar pos={sPos} dur={sDur} canSeek={canSeek} onSeek={seekTo} />
        )}

        {/* One row in Music Assistant's own order - heart, shuffle, prev, play, next, repeat - with
            everything drawn as a bare glyph and only the play/pause carrying a filled circle
            (owner's reference screenshot, September 2026). Buttons a tier can't serve still simply
            don't render; the row centres whatever remains. */}
        {showTransport && (
          <div class="mrow">
            {/* The heart went with the whole favorites feature (owner's request, September 2026) -
                a press had no readable echo in any payload, so the button could only ever say
                "sent", and the owner judged it not worth the row space. */}
            {/* Each control disables while its own command is pending - Music Assistant's behaviour,
                and the guard against a double-send the device would replay. The ring explains the
                inertness; the disabled state needs no extra styling (nothing greys these classes). */}
            {showSkips && ctrl && supHas(SUP.SHUFFLE) && (
              <button
                class={`mbtn${shuffle ? " on" : ""}${pending.shuffle ? " busy" : ""}`}
                aria-label="Shuffle"
                aria-pressed={shuffle}
                disabled={!!pending.shuffle}
                onClick={toggleShuffle}
              >
                {I_SHUFFLE}
              </button>
            )}
            {showSkips && supHas(SUP.PREV) && (
              <button class={`mbtn${pending.prev ? " busy" : ""}`} aria-label="Previous track" disabled={!!pending.prev} onClick={() => skip("prev")}>
                {I_PREV}
              </button>
            )}
            <button class={`mplay${pending.play ? " busy" : ""}`} aria-label={playing ? "Pause" : "Play"} disabled={!!pending.play} onClick={model.playPause}>
              {playing ? I_PAUSE : I_PLAY}
            </button>
            {showSkips && supHas(SUP.NEXT) && (
              <button class={`mbtn${pending.next ? " busy" : ""}`} aria-label="Next track" disabled={!!pending.next} onClick={() => skip("next")}>
                {I_NEXT}
              </button>
            )}
            {showSkips && ctrl && (
              <button class={`mbtn${repeat !== 0 ? " on" : ""}${pending.repeat ? " busy" : ""}`} aria-label="Repeat" disabled={!!pending.repeat} onClick={cycleRepeat}>
                {I_REPEAT}
                {repeat === 1 && <span class="mrpt1 num">1</span>}
              </button>
            )}
          </div>
        )}

        {/* No volume slider here (owner's request, September 2026): the bar underneath and the
            players panel both carry one, and a third copy on this screen was clutter, not control. */}

        {/* The group moved to the players panel the bar's speaker button opens; this chip is the
            expanded view's way in, named for who is playing (owner's screenshot 4). */}
        <button class="mchip" onClick={onPlayers}>
          {I_SPK}
          <span>{chip}</span>
        </button>

        <MaPanel cfg={maCfg} setCfg={setMaCfg} status={maCfg.url && maCfg.token ? ws.status : "off"} ip={ip} />
        </div>
      </div>
    </>
  );
}

/**
 * The Music Assistant connection panel, folded shut at the bottom of the sheet.
 *
 * The one place in the app that asks for a credential, and deliberately buried: the footer is
 * complete without it, and this exists for the person who wants the real-time tier. The token is a
 * long-lived one from MA's own profile settings; it lands in localStorage and - since September
 * 2026, at the owner's request - on the device behind the session gate, so the second browser in a
 * household inherits the connection instead of asking for the token again (see lib/ma.js's header
 * for the trust reasoning). The dot on the fold is the connection, so the panel can stay shut once
 * it works.
 *
 * The address no longer has to be known or typed: "Find my server" sweeps the device's own /24 on
 * Music Assistant's signature port and verifies every listener over the WebSocket hello (see
 * scanForMa), so the common path is one tap, one row, one pasted token.
 *
 * `defaultOpen` starts the fold open - the search drawer's not-connected state passes it, because
 * there the person has already asked for the panel by pressing "Set up connection". `ip` seeds the
 * scan's subnet: the device's address is the one address guaranteed to be on the right network.
 */
function MaPanel({ cfg, setCfg, status, ip, defaultOpen }) {
  const [open, setOpen] = useState(!!defaultOpen);
  const [url, setUrl] = useState(cfg.url);
  const [token, setToken] = useState(cfg.token);
  const configured = !!(cfg.url && cfg.token);

  // The scan: idle -> {done, total} while running -> a result list (or "https"/"none"), with a
  // liveness flag so a panel closed mid-sweep drops the answer instead of setting dead state.
  const [scan, setScan] = useState(null);
  const scanLive = useRef(false);
  useEffect(
    () => () => {
      scanLive.current = false;
    },
    [],
  );
  const startScan = () => {
    scanLive.current = true;
    setScan({ done: 0, total: 0 });
    scanForMa(ip, (done, total) => scanLive.current && setScan((s) => (s && s.list ? s : { done, total })))
      .then((list) => scanLive.current && setScan(list.length ? { list } : { err: "none" }))
      .catch(() => scanLive.current && setScan({ err: "https" }));
  };

  const connect = () => {
    const u = url.trim();
    const t = token.trim();
    if (!u || !t) return;
    maSettings.set(u, t);
    setCfg({ url: u, token: t });
    // The device's copy, for the next browser. Fire-and-forget: this browser is already connected.
    maCfgSave(u, t);
  };
  const disconnect = () => {
    maSettings.clear();
    setCfg({ url: "", token: "" });
    maCfgSave("", "");
  };

  return (
    <div class="mapanel">
      <button class="offhead" aria-expanded={open} onClick={() => setOpen(!open)}>
        <span class={`dot${status === "on" ? " ok" : ""}`} />
        <span class="grow">{TEXT.ma_title}</span>
        <Chevron down={open} cls="caret-s" />
      </button>
      {open && (
        <div class="mapanel-body">
          <p class="dim xs">{HINTS.ma_connect}</p>
          <input
            class="in"
            type="text"
            value={url}
            placeholder={TEXT.ma_url_ph}
            aria-label="Music Assistant address"
            onInput={(e) => setUrl(e.currentTarget.value)}
          />
          {/* The scan, under the field it fills. One button while idle; a counter while the sweep
              runs; then one tappable row per verified server, named by the server itself with its
              version and address, choosing which fills the field above. */}
          {!scan && (
            <button class="btn sm mascan-btn" onClick={startScan}>
              {TEXT.ma_scan_btn}
            </button>
          )}
          {scan && !scan.list && !scan.err && (
            <p class="dim xs mascan-note">
              {TEXT.ma_scanning.replace("%s", scan.total ? `${scan.done}/${scan.total}` : "")}
            </p>
          )}
          {scan?.err && (
            <p class="dim xs mascan-note">{scan.err === "https" ? TEXT.ma_scan_https : TEXT.ma_scan_none}</p>
          )}
          {scan?.list &&
            scan.list.map((s) => (
              <button
                key={s.id}
                class={`mascan-row${url.trim() === s.url ? " on" : ""}`}
                onClick={() => setUrl(s.url)}
              >
                <span class="mascan-name">{s.name}</span>
                <span class="mascan-sub">
                  {s.version ? `${s.version} \u00b7 ` : ""}
                  {s.url.replace(/^https?:\/\//, "")}
                </span>
              </button>
            ))}
          <input
            class="in"
            type="password"
            value={token}
            placeholder={TEXT.ma_token_ph}
            aria-label="Music Assistant token"
            onInput={(e) => setToken(e.currentTarget.value)}
          />
          <div class="mapanel-row">
            <button class="btn" onClick={connect}>
              {TEXT.ma_connect_btn}
            </button>
            {configured && (
              <button class="btn" onClick={disconnect}>
                {TEXT.ma_disconnect_btn}
              </button>
            )}
          </div>
          {configured && status === "on" && <p class="dim xs">{TEXT.ma_connected}</p>}
          {configured && status === "error" && <p class="dim xs">{TEXT.ma_error}</p>}
        </div>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* The players panel                                                  */
/* ------------------------------------------------------------------ */

/**
 * The slide-up behind the bar's speaker button and the expanded view's chip, shaped after MA's own
 * Players sheet (owner's screenshots 2 and 3): the group's members each with a volume and a remove,
 * the join picker, and the group volume on top. Partial-height over a scrim rather than
 * full-screen, because it is a side errand from whatever is under it.
 */
function PlayersPanel({ model, tiers, tint, haReady, onClose }) {
  const { media, mediaCmd, srcParam, groupHeld } = model;
  const { wsOn, me, live, members, addables, optVol, pendingGroup, gVol, gUnjoin, gJoin } = tiers;

  // Escape and the one-drawer rule, plus the finger-following swipe-down. Opening this panel is
  // what closes the expanded view under it - the chip in that view opens this one.
  useDrawer("players", true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, 1);

  const any = wsOn || me;
  return (
    <>
      <div class="scrim mpanel-scrim" onClick={onClose} />
      <div class="mpanel" style={(tint || "") + dragStyle} {...drag} role="dialog" aria-label={TEXT.media_players_title}>
        {/* Just the handle, like the media sheet (owner's request, September 2026): the "Players"
            label with its ⓘ and the rule under the top block went - the speaker rows say what this
            drawer is. The name survives in the dialog's aria-label above. */}
        <button class="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />

        {/* The whole-group slider, present only while there is a group to speak of (owner's request,
            September 2026): with one speaker it duplicated that speaker's own row below and the
            bar's slider both, and "Volume" over a list of speakers read as nobody's in particular. */}
        {members.length > 1 && (
          <div class="mvol-row">
            <span class="dim sm">{TEXT.media_group_volume}</span>
            <Vol
              value={groupHeld ? media.ss_volume : media.volume}
              label={TEXT.media_group_volume}
              onCommit={(v) => mediaCmd("volume", { v, src: srcParam })}
            />
          </div>
        )}

        {/* No tier at all: the payload arrived and named no player (Home Assistant absent, actions
            off, or no Music Assistant), and no direct connection is configured. Said plainly; the
            connection panel it points to sits at the bottom of the expanded view. Nothing shows
            while the payload is still loading. */}
        {!any && haReady && <div class="dim sm">{TEXT.media_no_tiers}</div>}

        {any && (
          <div class="mgroup">
            {!wsOn && !live && <div class="dim sm">{TEXT.media_group_loading}</div>}
            {members.map(([id, name, vol]) => (
              <div class="mgroup-row" key={id}>
                <span class="mgroup-name">{name}</span>
                {vol >= 0 && <Vol value={optVol[id] ?? vol} label={`${name} volume`} onCommit={(v) => gVol(id, v)} />}
                {members.length > 1 && (
                  <button
                    class="icon mgroup-x"
                    aria-label={`Remove ${name} from the group`}
                    title={`Remove ${name} from the group`}
                    onClick={() => gUnjoin(id)}
                  >
                    {I_MINUS}
                  </button>
                )}
              </div>
            ))}
            {/* One row per joinable speaker with a drawn +, replacing a native <select> - the select
                rendered as a grey form control on the tinted panel, and it hid the list MA's own
                sheet shows outright behind an extra tap. A tapped row stays put with a ring spinning
                around its + (and locks) until the join is confirmed - per-row, so two quick adds
                spin independently; it replaced a single "Asking Music Assistant..." line that named
                no row. The + rides in .mgroup-glyph so the ring can wrap the drawn circle rather
                than the full-width row. */}
            {(wsOn || live) && addables.length > 0 && (
              <div class="mgroup-adds">
                <div class="dim sm">{TEXT.media_add_speaker}</div>
                {addables.map(([id, name]) => (
                  <button
                    key={id}
                    class={`mgroup-addrow${pendingGroup[id] ? " busy" : ""}`}
                    disabled={!!pendingGroup[id]}
                    onClick={() => gJoin(id)}
                  >
                    <span class="mgroup-glyph">{I_PLUS}</span>
                    <span>{name}</span>
                  </button>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </>
  );
}

/* ------------------------------------------------------------------ */
/* The bar                                                            */
/* ------------------------------------------------------------------ */

/**
 * The bar's title line, marqueeing when it overflows. CSS cannot know a line is clipped, so the
 * overflow is measured after paint and the slide distance handed back as a custom property; a title
 * that fits stays a plain ellipsis-less line and costs nothing. Re-measured per title, not per
 * resize - the next track corrects a rotated phone, which is the cheap kind of eventually-right.
 */
function BarTitle({ text }) {
  const ref = useRef(null);
  const [mq, setMq] = useState(0);
  useEffect(() => {
    const el = ref.current;
    setMq(el && el.scrollWidth > el.clientWidth ? el.clientWidth - el.scrollWidth : 0);
  }, [text]);
  return (
    <div class={`mbar-t${mq ? " scroll" : ""}`} ref={ref}>
      <span style={mq ? `--mq:${mq}px` : null}>{text}</span>
    </div>
  );
}

/** `search`/`onSearchClose`: the search drawer's open state, owned by the shell (its opener is
 *  the top bar's magnifying glass) but rendered here, because this component owns the tiers the
 *  drawer runs on - hoisting the socket instead would remount it around every open. `ip` is the
 *  device's own address, seeding the MaPanel scan's subnet. */
export function MediaFooter({ ha, mac, ip, search, onSearchClose }) {
  const [open, setOpen] = useState(false);
  const [panel, setPanel] = useState(false);
  const tiers = useTiers(ha, mac, open || panel);

  // The upper tiers' word that the group is paused - the file comment's second pause path. Neither
  // tier ever literally says "paused" for a Sendspin player (the pause stops the stream and the MA
  // player goes idle), so the test on both is the one MA's own bar uses: not playing, but the
  // queue still holds a current item - the resume point. The socket reads the queue directly and
  // is believed outright; the relay reads the Home Assistant entity (whose media_title mirrors the
  // queue's current item), is believed only while fresh (its `age` when served plus how long this
  // render has held it), and only when no socket is up to contradict it. Either way the claim
  // rides into the model as one boolean, where it props the paused-group card up exactly as a held
  // pause does. The literal "paused" states are still honoured for the day a Sendspin pause stops
  // meaning "stop": they cost nothing and can only ever agree.
  const wsQ = tiers.wsOn ? tiers.ws.queue : null;
  const wsPaused =
    tiers.wsOn &&
    (tiers.ws.me?.state === "paused" || (wsQ != null && wsQ.state !== "playing" && wsQ.current_item != null));
  const maFresh =
    tiers.ma?.at != null && tiers.ma.age >= 0 && tiers.ma.age + (Date.now() - tiers.ma.at) / 1000 < MA_PAUSED_TRUST_S;
  const relayPaused =
    !tiers.wsOn &&
    maFresh &&
    (tiers.ma?.d?.st === "paused" || (tiers.ma?.d?.st === "idle" && !!tiers.ma?.d?.ti));

  const model = useMediaModel(wsPaused || relayPaused);
  const tint = tintStyle(useArtColor(model.art));

  // The group stream just stopped, which on this device is one fact wearing two meanings: paused
  // somewhere, or ended. One relayed sync asks Home Assistant which - the transition is the only
  // moment the question has a fresh answer, so it is asked exactly then rather than on a poll.
  const ssState = model.media?.ss_state;
  const prevSs = useRef(ssState);
  useEffect(() => {
    const was = prevSs.current;
    prevSs.current = ssState;
    if (was === 2 && ssState != null && ssState !== 2) tiers.maAsk();
    // maAsk is a stable question, not a dependency; only the transition should re-run this.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ssState]);

  // While a relay-reported pause is the only thing holding the card up, re-ask slowly, so the trust
  // window above keeps being re-earned for as long as the pause is real. The open panels run their
  // own faster cycle and the socket streams updates unasked, so both cases are excused here.
  useEffect(() => {
    if (!relayPaused || open || panel) return undefined;
    const t = setInterval(() => tiers.maAsk(), MA_PAUSED_RECHECK_MS);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [relayPaused, open, panel]);

  // The search drawer, built before the early return below: searching must not wait on the
  // media poll - the drawer's own states cover every tier the poll would have vouched for. The
  // MaPanel rides in as a prop rather than an import on search.jsx's side, because media.jsx
  // renders SearchDrawer and importing back the other way would be a cycle.
  const searchEl = search ? (
    <SearchDrawer
      tiers={tiers}
      setup={
        <MaPanel
          cfg={tiers.maCfg}
          setCfg={tiers.setMaCfg}
          status={tiers.maCfg.url && tiers.maCfg.token ? tiers.ws.status : "off"}
          ip={ip}
          defaultOpen
        />
      }
      onClose={onSearchClose}
    />
  ) : null;

  // Waits for the first poll rather than painting an empty shell, exactly as the card did.
  if (!model.media) return searchEl;

  const { media, mediaCmd, srcParam, active, announcing, playing } = model;
  const barTitle = model.title || model.stateText || TEXT.media_idle_bar;
  const barSub = model.artist || (active || announcing ? model.srcText : "");
  const gcount = tiers.members.length;

  return (
    <>
      {/* A div with a click rather than a button, because the controls inside are buttons of their
          own and buttons do not nest; the meta block is the accessible way in. Floating and tinted
          after MA's own bar (owner's screenshot 1): tap the artwork or the title for the full view,
          the speaker for the players panel, and the second row is the volume, riding along. */}
      <div class="mbar" style={tint} onClick={() => setOpen(true)}>
        <div class="mbar-row">
          <Artwork art={model.art} />
          <button class="mbar-meta" aria-label="Open media view" aria-expanded={open}>
            <BarTitle text={barTitle} />
            {barSub && <div class="mbar-s dim">{barSub}</div>}
          </button>
          <button
            class="mbtn mspk"
            aria-label={TEXT.media_players_title}
            onClick={(e) => {
              e.stopPropagation();
              setPanel(true);
            }}
          >
            {I_SPK}
            {gcount > 1 && <span class="mbdg num">{gcount}</span>}
          </button>
          {active && !announcing && (
            <button
              class={`mplay sm${model.pending.play ? " busy" : ""}`}
              aria-label={playing ? "Pause" : "Play"}
              disabled={!!model.pending.play}
              onClick={(e) => {
                e.stopPropagation();
                model.playPause();
              }}
            >
              {playing ? I_PAUSE : I_PLAY}
            </button>
          )}
        </div>
        {media.volume != null && (
          // stopPropagation on the row, not the input: a miss around the slider should not flip
          // the whole page into the expanded view mid-drag.
          <div class="mbar-vol" onClick={(e) => e.stopPropagation()}>
            {I_VOL}
            <Vol
              value={model.groupHeld ? media.ss_volume : media.volume}
              onCommit={(v) => mediaCmd("volume", { v, src: srcParam })}
            />
          </div>
        )}
      </div>
      {open && (
        <MediaSheet
          model={model}
          tiers={tiers}
          tint={tint}
          ip={ip}
          onClose={() => setOpen(false)}
          onPlayers={() => setPanel(true)}
        />
      )}
      {panel && <PlayersPanel model={model} tiers={tiers} tint={tint} haReady={!!ha?.d} onClose={() => setPanel(false)} />}
      {searchEl}
    </>
  );
}
