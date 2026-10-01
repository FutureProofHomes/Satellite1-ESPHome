/**
 * The media bar's state: the device poll and the held pause (useMediaModel), the commands awaiting
 * their echo (usePendingCmds), the playhead between polls (usePlayhead), and the two upper tiers -
 * the Home Assistant relay and this browser's own Music Assistant socket - that add grouping,
 * seeking and search (useTiers).
 */
import { useEffect, useMemo, useRef, useState } from 'react';
import { TEXT } from '../../../src/copy.js';
import { useMaData, useMedia } from '../../../src/lib/device.js';
import { maCfgFetch, maSettings, useMaSocket } from '../../../src/lib/ma.js';
import { GROUP_PENDING_MAX_MS, MEDIA_STATE, PENDING_MAX_MS, averageHsl, groupRows, holdEnds, mediaView, prune, settleGroup } from '../../lib/media.js';

/** A row as the tiers report it: [id, name, volume 0-100 or -1]. */
export type Row = [string, string, number];
type Done = (media: any) => boolean;
type Pending = Record<string, { done: Done; at: number }>;

/**
 * Commands awaiting their echo in the polled payload. Music Assistant's own player rings the acting
 * button while the server's in-progress flag is up (PlayBtn.vue's `play_action_in_progress`), but
 * /api/sat1/media carries no such flag and the command POST answers nothing, so the lifecycle is
 * rebuilt here: an entry starts on click and settles on the first payload its `done` passes, or at
 * the deadline - which has to fire between payloads too, since idle polls run 5s apart and a device
 * that stopped answering sends none.
 */
function usePendingCmds(media: any): [Pending, (key: string, done: Done) => void] {
  const [pending, setPending] = useState<Pending>({});
  useEffect(() => {
    const now = Date.now();
    setPending(p => prune(p, ([, e]: [string, Pending[string]]) => !e.done(media) && now - e.at < PENDING_MAX_MS));
  }, [media]);
  useEffect(() => {
    const ats = Object.values(pending).map(e => e.at);
    if (!ats.length) return undefined;
    const t = setTimeout(() => {
      const now = Date.now();
      setPending(p => prune(p, ([, e]: [string, Pending[string]]) => now - e.at < PENDING_MAX_MS));
    }, Math.max(50, PENDING_MAX_MS - (Date.now() - Math.min(...ats)) + 30));
    return () => clearTimeout(t);
  }, [pending]);
  return [pending, (key, done) => setPending(p => ({ ...p, [key]: { done, at: Date.now() } }))];
}

/**
 * GET /api/sat1/media and what the bar makes of it. `maPaused` is the upper tiers' word that the
 * group is paused; it joins this browser's own `held` pause rather than replacing it, because
 * `held` works with no tier answering and the tiers work across browsers. It needs no clearing of
 * its own: it is derived from the tiers' live view, so the group resuming or the queue being
 * cleared, wherever that happens, is what makes it false.
 */
export function useMediaModel(maPaused: boolean) {
  const { media, mediaCmd, mediaPoke } = useMedia(true);
  const [held, setHeld] = useState(false);
  // One pending map for the bar and Now Playing: their play buttons send the same command, so they
  // must ring together.
  const [pending, startCmd] = usePendingCmds(media);
  // Starting an entry pulls the next poll forward: an idle poll is 5s away, the ring's whole deadline.
  const startPending = (key: string, done: Done) => {
    startCmd(key, done);
    mediaPoke();
  };
  const view = mediaView(media, held, maPaused);
  const ends = holdEnds(media);
  useEffect(() => {
    if (ends) setHeld(false);
  }, [ends]);

  // On the group path the pause is confirmed by ss_state, not the derived state: `held` flips the
  // view to paused before the POST even leaves.
  const playPause = () => {
    const ss = view.sendspin;
    if (view.playing) {
      if (ss) setHeld(true);
      startPending('play', ss ? m => m?.ss_state !== 2 : m => m?.state !== 2);
      mediaCmd('pause', { src: view.srcParam });
    } else {
      startPending('play', ss ? m => m?.ss_state === 2 : m => m?.state === 2);
      mediaCmd('play', { src: view.srcParam });
    }
  };

  return {
    ...view,
    media,
    mediaCmd,
    playPause,
    pending,
    startPending,
    stateText: (MEDIA_STATE as Record<number, string>)[view.state] || '',
    srcText: view.sendspin ? TEXT.media_src_group : TEXT.media_src_local
  };
}
export type Model = ReturnType<typeof useMediaModel>;

/** The playhead between polls: the last reported position plus the time since, on a half-second
 *  tick while playing, re-anchored by every poll. */
export function usePlayhead(media: any, playing: boolean) {
  const anchor = useMemo(() => ({ pos: media?.pos ?? 0, at: Date.now() }), [media]);
  const [, tick] = useState(0);
  useEffect(() => {
    if (!playing) return undefined;
    const t = setInterval(() => tick(n => n + 1), 500);
    return () => clearInterval(t);
  }, [playing]);
  const dur = media?.dur ?? 0;
  let pos = anchor.pos + (playing ? Date.now() - anchor.at : 0);
  if (dur > 0 && pos > dur) pos = dur;
  return { pos, dur };
}

/**
 * The two upper tiers. The Music Assistant socket is this browser's own connection to the MA
 * server, open for the life of the page while a connection is configured, because the bar's badge
 * wants real-time membership too (one socket at most - it is the MA server's, not the device's).
 * The Home Assistant relay is `disc` (this device's MA player and its join candidates, riding the
 * big payload) plus /api/sat1/ma's live view, polled only while `wake` - something showing it is
 * open, since each poll is an action call on the device - and only when no socket answers instead.
 * useMaData's single mount-time read covers the badge.
 */
export function useTiers(ha: any, mac: string | undefined, wake: boolean) {
  const [maCfg, setMaCfg] = useState<{ url: string; token: string }>(maSettings.get);
  const ws = useMaSocket(mac, maCfg.url && maCfg.token ? maCfg : null);
  const wsOn = ws.status === 'on' && !!ws.me;

  // A browser with no stored connection seeds itself from the device's copy, once per mount - the
  // app remounts per device, which is the right cadence, since the copy is per device. That is what
  // lets one setup serve every phone. A browser that holds its own keeps it, so one deliberately
  // pointed at a different server is not overwritten.
  useEffect(() => {
    if (maCfg.url && maCfg.token) return undefined;
    let live = true;
    maCfgFetch().then(cfg => {
      if (!live || !cfg) return;
      maSettings.set(cfg.url, cfg.token);
      setMaCfg(cfg);
    });
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const disc = ha?.d?.ma;
  const me: string = disc?.e || '';
  const { ma, maCmd, maRead, maAsk } = useMaData(!!me && !wsOn && wake);
  const live = ma?.d?.e ? ma.d : null;
  // A group edit lands ~1s after the action call; one early read beats waiting out the cycle.
  const poke = () => setTimeout(maRead, 2200);

  // Optimistic member volumes, cleared by any fresh payload.
  const [optVol, setOptVol] = useState<Record<string, number>>({});
  useEffect(() => {
    setOptVol({});
  }, [ma, ws.players]);

  // Group edits in flight: a pending join rings its add row, a pending unjoin hides its member row -
  // the instant optimistic hide the owner kept (September 2026) over a lingering ringed row.
  const [pendingGroup, setPendingGroup] = useState<Record<string, { kind: 'join' | 'unjoin'; at: number }>>({});
  const { raw, members, addables } = groupRows({ wsOn, me: ws.me, players: ws.players, live, cands: disc?.c, pending: pendingGroup });
  useEffect(() => {
    setPendingGroup(p => Object.keys(p).length ? settleGroup(p, raw, Date.now()) : p);
    // The payloads are the events; `raw` is derived from exactly them.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ma, ws.players]);
  // The deadline fires without a payload too: a socket that went quiet sends nothing to settle on,
  // and neither a ring nor a hidden row may outlive a failed edit.
  useEffect(() => {
    const ats = Object.values(pendingGroup).map(e => e.at);
    if (!ats.length) return undefined;
    const t = setTimeout(() => {
      const now = Date.now();
      setPendingGroup(p => prune(p, ([, e]: [string, { at: number }]) => now - e.at < GROUP_PENDING_MAX_MS));
    }, Math.max(50, GROUP_PENDING_MAX_MS - (Date.now() - Math.min(...ats)) + 30));
    return () => clearTimeout(t);
  }, [pendingGroup]);

  // Commands go to the tier that produced the rows. The socket's failures are swallowed - the next
  // player update is the truth - and the relay's surface through the write toast.
  const gVol = (id: string, v: number) => {
    setOptVol(o => ({ ...o, [id]: v }));
    if (wsOn) ws.cmd('players/cmd/volume_set', { player_id: id, volume_level: v }).catch(() => {});
    else {
      maCmd('vol', { e: id, v });
      poke();
    }
  };
  const gUnjoin = (id: string) => {
    setPendingGroup(p => ({ ...p, [id]: { kind: 'unjoin', at: Date.now() } }));
    if (wsOn) ws.cmd('players/cmd/ungroup', { player_id: id }).catch(() => {});
    else {
      maCmd('unjoin', { e: id });
      poke();
    }
  };
  const gJoin = (id: string) => {
    setPendingGroup(p => ({ ...p, [id]: { kind: 'join', at: Date.now() } }));
    if (wsOn) ws.cmd('players/cmd/group', { player_id: id, target_player: ws.me.player_id }).catch(() => {});
    else {
      maCmd('join', { e: me, m: id });
      poke();
    }
  };

  return {
    maCfg,
    setMaCfg,
    ws,
    wsOn,
    me,
    maCmd,
    // The raw relay payload rides beside `live` for its `age` and `at`, which decide whether a
    // "paused" claim is fresh enough to trust (relayPausedOf).
    ma,
    maAsk,
    live,
    members: members as Row[],
    addables: addables as [string, string][],
    optVol,
    pendingGroup,
    gVol,
    gUnjoin,
    gJoin
  };
}
export type Tiers = ReturnType<typeof useTiers>;

/**
 * The artwork's average colour through a 6x6 canvas, for the bar's --tint. Only works when the art
 * host allows this origin to read pixels back; a tainted canvas throws and the tint stays off.
 */
export function useArtColor(art: string) {
  const [col, setCol] = useState<number[] | null>(null);
  useEffect(() => {
    setCol(null);
    if (!art) return undefined;
    let live = true;
    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => {
      if (!live) return;
      try {
        const c = document.createElement('canvas');
        c.width = c.height = 6;
        const g = c.getContext('2d')!;
        g.drawImage(img, 0, 0, 6, 6);
        setCol(averageHsl(g.getImageData(0, 0, 6, 6).data));
      } catch {
        /* tainted canvas: an art host without CORS */
      }
    };
    img.src = art;
    return () => {
      live = false;
    };
  }, [art]);
  return col;
}

/**
 * Holds a just-committed slider value over the stale reads that follow it, until the incoming
 * value comes within `tol` of it or five seconds pass (a refused write, where the old value is true).
 */
export function useHeld(value: number, tol: number): [number, (v: number) => void] {
  const h = useRef<{ v: number; at: number } | null>(null);
  if (h.current !== null && (Math.abs(value - h.current.v) <= tol || Date.now() - h.current.at > 5000)) h.current = null;
  return [h.current !== null ? h.current.v : value, v => {
    h.current = { v, at: Date.now() };
  }];
}
