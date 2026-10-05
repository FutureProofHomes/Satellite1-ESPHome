/**
 * The media bar's state: the device poll and the held pause (useMediaModel), the commands awaiting
 * their echo (usePendingCmds), the playhead between polls (usePlayhead), and the two upper tiers -
 * the Home Assistant relay and this browser's own Music Assistant socket - that add grouping,
 * seeking and search (useTiers).
 */
import { useEffect, useMemo, useRef, useState } from 'react';
import { TEXT } from '../../copy.js';
import { useMaData, useMedia } from '../../lib/device.js';
import { maCfgFetch, maSettings, useMaSocket } from '../../lib/ma.js';
import { GROUP_PENDING_MAX_MS, MEDIA_STATE, PENDING_MAX_MS, REMOTE_PENDING_MAX_MS, TAKEOVER_PENDING_MAX_MS, barCards, elsewhereRows, groupRows, groupSteps, hasOwnMusic, holdEnds, mediaView, prune, settleGroup, vibrantHsl } from '../../lib/media.js';

/** A row as the tiers report it: [id, name, volume 0-100 or -1]. */
export type Row = [string, string, number];
/** Another speaker or group Music Assistant is playing (elsewhereRows in src/lib/media.js). */
export type Other = {
  id: string;
  name: string;
  members: Row[];
  group: boolean;
  title: string;
  artist: string;
  art: string;
  state: string;
  volume: number;
  vctl: boolean;
  transferable: boolean;
  queue: string;
  sync: boolean | null;
  mac: string;
  clock: { pos: number; at: number; dur: number } | null;
};
type RowOp = { kind: 'join' | 'takeover' | 'move'; at: number; name: string };
/** `lead` is set for an edit to another speaker's group: the leader whose rows confirm it. */
type GroupOp = { kind: 'join' | 'unjoin'; at: number; name?: string; lead?: string };
/** A group as the players panel shows it (groupRows). */
export type Group = { members: Row[]; addables: [string, string][]; leader: string };
export type Transport = 'play_pause' | 'next' | 'previous';
const ROW_OP_MAX_MS = { join: GROUP_PENDING_MAX_MS, takeover: TAKEOVER_PENDING_MAX_MS, move: TAKEOVER_PENDING_MAX_MS };
const fill = (s: string, ...a: string[]) => a.reduce((t, x) => t.replace('%s', x), s);
/** Resolves true once `ok` holds, polled every quarter second, or false at `ms`. */
const until = (ok: () => boolean, ms: number) => new Promise<boolean>(res => {
  const t0 = Date.now();
  const tick = () => {
    if (ok()) res(true);
    else if (Date.now() - t0 > ms) res(false);
    else setTimeout(tick, 250);
  };
  tick();
});
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
 * open, since each poll is an action call on the device - or a command awaits its echo, and only
 * when no socket answers instead. useMaData's single mount-time read covers the badge.
 *
 * Beyond this device's own group it reads the other speakers Music Assistant is playing: the
 * players panel's "Playing elsewhere" (`elsewhere`) with Join, Take over and Move here, and the
 * bar's cards (`cards`) with their transport and volume. Everything is built from the group's
 * leader (leaderOf), so a member shows and grows the same group its leader does.
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

  // Group edits in flight: a pending join rings its add row, a pending unjoin hides its member row -
  // the instant optimistic hide the owner kept (September 2026) over a lingering ringed row.
  const [pendingGroup, setPendingGroup] = useState<Record<string, GroupOp>>({});
  // Join, Take over and Move here in flight, by the other row's id; Next and Play/Pause on another
  // speaker, by `${id}:${cmd}`, with the row's state and title when tapped (a change is the echo).
  const [rowOps, setRowOps] = useState<Record<string, RowOp>>({});
  const [remote, setRemote] = useState<Record<string, { at: number; sig: string }>>({});
  const busy = Object.keys(pendingGroup).length + Object.keys(rowOps).length + Object.keys(remote).length > 0;

  // Polled while something showing it is open, and while a command waits on its echo - Take over
  // started from another speaker's card has nothing open to wake it otherwise.
  const { ma, maCmd, maRead, maAsk } = useMaData(!!me && !wsOn && (wake || busy));
  const live = ma?.d?.e ? ma.d : null;
  // A group edit lands ~1s after the action call; one early read beats waiting out the cycle.
  const poke = () => setTimeout(maRead, 2200);

  // Optimistic volumes - group members' and other speakers' - cleared by any fresh payload.
  const [optVol, setOptVol] = useState<Record<string, number>>({});
  useEffect(() => {
    setOptVol({});
  }, [ma, ws.players]);

  // The bar's cards: every other speaker playing, and any seen playing during this page visit that
  // is paused now, in first-seen order (barCards). Refs, updated as the rows are read, because
  // which rows exist is itself derived from them (the socket keeps a seen speaker's row while it
  // idles). `held` is the speakers paused from their own card here, by when: the relay drops a
  // paused Satellite1's row, since it reports idle, and the card must not vanish under the finger
  // that paused it. Playing again a while after the press releases it.
  const seen = useRef<Set<string>>(new Set());
  const order = useRef<string[]>([]);
  const last = useRef<Record<string, Other>>({});
  const held = useRef<Map<string, number>>(new Map());
  const others: Other[] = elsewhereRows({ wsOn, me: ws.me, players: ws.players, live, keep: seen.current });
  for (const r of others) {
    if (r.state === 'playing' && !seen.current.has(r.id)) seen.current.add(r.id);
    if (seen.current.has(r.id) && !order.current.includes(r.id)) order.current.push(r.id);
    if (r.state === 'playing' && Date.now() - (held.current.get(r.id) ?? Infinity) > REMOTE_PENDING_MAX_MS) held.current.delete(r.id);
    last.current[r.id] = r;
  }
  const { raw, members, addables, leader: leaderId } = groupRows({ wsOn, me: ws.me, players: ws.players, live, cands: disc?.c, pending: pendingGroup, others });
  const leader: string = leaderId || (wsOn ? ws.me?.player_id : me) || '';
  const myId: string = wsOn ? ws.me?.player_id || '' : me;
  const selfName: string = (wsOn ? ws.me?.name : raw.find((r: Row) => r[0] === me)?.[1]) || '';
  const rawIds = new Set(raw.map((r: Row) => r[0]));
  const cards: Other[] = barCards({ rows: others, seen: seen.current, order: order.current, last: last.current, gone: rawIds, held: held.current });
  // Every other group's rows by its leader, which an edit made from that speaker's panel settles on.
  const groups: Record<string, Row[]> = {};
  for (const r of others) groups[r.id] = r.group ? r.members : [[r.id, r.name, r.volume], ...r.members];
  // The panel lists what Take over and Move here can act on: a Music Assistant queue, playing or
  // paused. The bar's cards list them all.
  const elsewhere = others.filter(r => r.transferable && (r.state === 'playing' || r.state === 'paused'));

  // The latest view for the multi-step commands below, which wait on later payloads.
  const view = useRef({ raw, leader, others });
  view.current = { raw, leader, others };

  // What a transport press waits to see change: the row as the tier reports it (a card may show it
  // paused where the tier says idle), or its absence - a held card's row the relay dropped.
  const rowSig = (id: string) => {
    const r = others.find(o => o.id === id);
    return r ? `${r.state}|${r.title}` : '';
  };

  // A failed group edit's explanation, shown at the top of the panel until the next edit.
  const [groupNote, setGroupNote] = useState('');
  const failJoin = (name: string) => setGroupNote(fill(TEXT.media_join_failed, name));

  const settle = (now: number) => {
    setPendingGroup(p => {
      if (!Object.keys(p).length) return p;
      for (const [id, e] of Object.entries(p)) {
        const ids = e.lead ? (groups[e.lead] || []).map(m => m[0]) : [...rawIds];
        if (e.kind === 'join' && !ids.includes(id) && now - e.at >= GROUP_PENDING_MAX_MS) failJoin(e.name || id);
      }
      return settleGroup(p, raw, now, groups);
    });
    setRowOps(p => {
      if (!Object.keys(p).length) return p;
      const row = (id: string) => others.find(r => r.id === id);
      return prune(p, ([id, e]: [string, RowOp]) => {
        const r = row(id);
        const done = e.kind === 'join' ? leader === id || rawIds.has(id) || !wsOn && !r
          : e.kind === 'takeover' ? rawIds.has(id) || (r?.members || []).some(m => rawIds.has(m[0])) || !!last.current[id]?.members.some(m => rawIds.has(m[0]))
          : !r || r.state !== 'playing';
        if (done) return false;
        if (now - e.at < ROW_OP_MAX_MS[e.kind]) return true;
        if (e.kind === 'move') setGroupNote(fill(TEXT.media_move_failed, e.name));
        else if (e.kind === 'takeover' && (!r || r.state !== 'playing')) setGroupNote(fill(TEXT.media_moved_alone, e.name, selfName || TEXT.media_this_speaker));
        else failJoin(e.name);
        return false;
      });
    });
    setRemote(p => {
      if (!Object.keys(p).length) return p;
      return prune(p, ([key, e]: [string, { at: number; sig: string }]) => rowSig(key.slice(0, key.lastIndexOf(':'))) === e.sig && now - e.at < REMOTE_PENDING_MAX_MS);
    });
  };
  useEffect(() => {
    settle(Date.now());
    // The payloads are the events; every row is derived from exactly them.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ma, ws.players]);
  // The deadlines fire without a payload too: a socket that went quiet sends nothing to settle on,
  // and neither a ring nor a hidden row may outlive a failed edit.
  useEffect(() => {
    const ends = [
      ...Object.values(pendingGroup).map(e => e.at + GROUP_PENDING_MAX_MS),
      ...Object.values(rowOps).map(e => e.at + ROW_OP_MAX_MS[e.kind]),
      ...Object.values(remote).map(e => e.at + REMOTE_PENDING_MAX_MS)
    ];
    if (!ends.length) return undefined;
    const t = setTimeout(() => settle(Date.now()), Math.max(50, Math.min(...ends) - Date.now() + 30));
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pendingGroup, rowOps, remote]);

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
  // `lead` names another speaker's group, edited from its own panel; without it, this device's.
  const gUnjoin = (id: string, lead?: string) => {
    setGroupNote('');
    setPendingGroup(p => ({ ...p, [id]: { kind: 'unjoin', at: Date.now(), lead } }));
    if (wsOn) ws.cmd('players/cmd/ungroup', { player_id: id }).catch(() => {});
    else {
      maCmd('unjoin', { e: id });
      poke();
    }
  };
  // Adds go to the group's leader, never this device: aimed at a member, MA made the member the
  // leader of a new group and the old one split.
  const gJoin = (id: string, lead?: string) => {
    setGroupNote('');
    setPendingGroup(p => ({ ...p, [id]: { kind: 'join', at: Date.now(), name: nameOf(id), lead } }));
    const to = lead || leader;
    if (wsOn) ws.cmd('players/cmd/group', { player_id: id, target_player: to }).catch(() => {});
    else {
      maCmd('join', { e: to, m: id });
      poke();
    }
  };

  // Names for the relay's member rows, which carry entity ids only: the join candidates, this
  // group, and the other rows between them name every speaker the relay can show.
  const names = new Map<string, string>();
  for (const p of Object.values(wsOn ? ws.players || {} : {}) as any[]) names.set(p.player_id, p.name);
  for (const [id, n] of (disc?.c || []) as [string, string][]) names.set(id, n);
  for (const r of raw as Row[]) names.set(r[0], r[1]);
  for (const r of others) names.set(r.id, r.name);
  const nameOf = (id: string) => names.get(id) || id;

  /**
   * Another speaker's group, for its own players panel from its card: its members with their
   * volumes, and the speakers that could join it - groupRows from that speaker's side. Nothing
   * playing anywhere else is offered, this device's group included while it has music of its own.
   */
  const groupOf = (row: Other): Group => {
    const mine = { id: leader, state: own ? 'playing' : 'idle', members: raw };
    const rest = [...others.filter(r => r.id !== row.id), mine];
    if (wsOn) {
      const p = ws.players?.[row.id];
      if (!p) return { members: [], addables: [], leader: row.id };
      return groupRows({ wsOn, me: p, players: ws.players, pending: pendingGroup, others: rest });
    }
    const g = (groups[row.id] || []).map(([id, , v]) => [id, nameOf(id), v]);
    return groupRows({ wsOn, live: { g, l: row.id }, cands: disc?.c, pending: pendingGroup, others: rest });
  };

  const startRow = (row: Other, kind: RowOp['kind']) => {
    setGroupNote('');
    setRowOps(p => ({ ...p, [row.id]: { kind, at: Date.now(), name: row.name } }));
  };
  const dropRow = (id: string) => setRowOps(p => prune(p, ([k]: [string, RowOp]) => k !== id));

  /** This speaker alone into their group. A leader with others first steps out, which hands its
   *  group to a remaining member (MA's ungroup on a sync leader) - joining straight away would
   *  dissolve it and silence them. A member's join needs no first step: MA releases it itself. */
  const rJoin = async (row: Other) => {
    startRow(row, 'join');
    const leadsOthers = leader === myId && raw.length > 1;
    if (wsOn) {
      if (leadsOthers) await ws.cmd('players/cmd/ungroup', { player_id: myId }).catch(() => {});
      ws.cmd('players/cmd/group', { player_id: myId, target_player: row.id }).catch(() => {});
      return;
    }
    if (leadsOthers) {
      maCmd('unjoin', { e: me });
      poke();
      await until(() => view.current.raw.length <= 1 || view.current.leader !== me, GROUP_PENDING_MAX_MS - 3000);
    }
    maCmd('join', { e: row.id, m: me });
    poke();
  };

  /** Every speaker of theirs, to name in the join: MA breaks up a group whose leader joins another,
   *  so naming the leader alone would leave the rest silent. A group player is named by its
   *  members, never itself. */
  const theirs = (row: Other) => [...(row.group ? [] : [row.id]), ...row.members.map(m => m[0])];

  /** Their music to this whole group at the same track and position, then their speakers in. */
  const takeover = async (row: Other) => {
    startRow(row, 'takeover');
    if (wsOn) {
      try {
        await ws.cmd('player_queues/transfer', { source_queue_id: row.queue, target_queue_id: leader, auto_play: true });
      } catch {
        dropRow(row.id);
        failJoin(row.name);
        return;
      }
      ws.cmd('players/cmd/set_members', { target_player: leader, player_ids_to_add: theirs(row) }).catch(() => {});
      return;
    }
    maCmd('takeover', { e: leader, s: row.id, m: theirs(row).join(',') });
    poke();
  };

  /** Their music to this group; their speakers stop. For speakers that cannot play in sync. */
  const move = (row: Other) => {
    startRow(row, 'move');
    if (wsOn) {
      ws.cmd('player_queues/transfer', { source_queue_id: row.queue, target_queue_id: leader, auto_play: true }).catch(() => {
        dropRow(row.id);
        setGroupNote(fill(TEXT.media_move_failed, row.name));
      });
      return;
    }
    maCmd('move', { e: leader, s: row.id });
    poke();
  };

  // Take over and Move here replace this group's queue, with no undo, so they ask first when there
  // is something here to lose (hasOwnMusic). One row asks at a time; it clears if its row goes.
  const [asking, setAsking] = useState<{ id: string; kind: 'takeover' | 'move' } | null>(null);
  const own = hasOwnMusic({ wsOn, queue: wsOn ? ws.queue : null, live });
  const rTakeover = (row: Other) => own ? setAsking({ id: row.id, kind: 'takeover' }) : takeover(row);
  const rMove = (row: Other) => own ? setAsking({ id: row.id, kind: 'move' }) : move(row);
  const confirmAsk = () => {
    const row = asking && others.find(r => r.id === asking.id);
    setAsking(null);
    if (!row) return;
    if (asking.kind === 'takeover') takeover(row);
    else move(row);
  };
  useEffect(() => {
    if (asking && !others.some(r => r.id === asking.id)) setAsking(null);
  });

  /** Next, Previous or Play/Pause on another speaker or group. */
  const rTransport = (row: Other, cmd: Transport) => {
    setRemote(p => ({ ...p, [`${row.id}:${cmd}`]: { at: Date.now(), sig: rowSig(row.id) } }));
    if (cmd === 'play_pause') {
      if (row.state === 'playing') held.current.set(row.id, Date.now());
      else held.current.delete(row.id);
    }
    if (wsOn) ws.cmd(`players/cmd/${cmd}`, { player_id: row.id }).catch(() => {});
    else {
      maCmd('transport', { e: row.id, c: cmd });
      poke();
    }
  };
  /** Another speaker's volume, or its whole group's. Home Assistant has no group volume, so on the
   *  relay a sync group moves member by member, each by the same step (groupSteps); a group player
   *  takes one write, which Music Assistant spreads itself. */
  const rVolume = (row: Other, v: number) => {
    setOptVol(o => ({ ...o, [row.id]: v }));
    if (wsOn) {
      ws.cmd('players/cmd/group_volume', { player_id: row.id, volume_level: v }).catch(() => {});
      return;
    }
    const writes: [string, number][] = row.group || !row.members.length ? [[row.id, v]] : groupSteps([[row.id, row.name, row.volume], ...row.members], v);
    for (const [e, vol] of writes) maCmd('vol', { e, v: vol });
    poke();
  };
  /** Socket only: the relay's rows carry no playhead to seek from. */
  const rSeek = (row: Other, ms: number) => {
    if (wsOn) ws.cmd('player_queues/seek', { queue_id: row.queue, position: Math.round(ms / 1000) }).catch(() => {});
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
    gJoin,
    groupOf,
    elsewhere,
    cards,
    rowOps,
    remote,
    groupNote,
    setGroupNote,
    asking,
    setAsking,
    confirmAsk,
    selfName,
    rJoin,
    rTakeover,
    rMove,
    rTransport,
    rVolume,
    rSeek
  };
}
export type Tiers = ReturnType<typeof useTiers>;

/**
 * The artwork's most vivid colour family, read from a 16x16 canvas, for the bar's --tint. Only works
 * when the art host allows this origin to read pixels back; a tainted canvas throws and the tint
 * stays off. Art with no colour in it at all (vibrantHsl's null) leaves it off as well.
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
        c.width = c.height = 16;
        const g = c.getContext('2d')!;
        g.drawImage(img, 0, 0, 16, 16);
        setCol(vibrantHsl(g.getImageData(0, 0, 16, 16).data));
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
