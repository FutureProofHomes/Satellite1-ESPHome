/**
 * The media bar's logic: which player owns the bar, the held pause that keeps a stopped group
 * stream resumable, the tiers' pause signals, the group rows, and Music Assistant's results.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { gatherResult } from "../src/lib/ma.js";
import {
  addRecent,
  albumSections,
  albumSub,
  artThumb,
  barCards,
  canOpen,
  canSync,
  elsewhereRows,
  episodeSub,
  fmtTime,
  groupLabel,
  groupRows,
  groupSteps,
  hasOwnMusic,
  heroSub,
  holdEnds,
  leaderOf,
  mediaView,
  nextRepeat,
  openArgs,
  openedOrder,
  pct,
  prune,
  queueClock,
  relayPausedOf,
  rowClock,
  searchArgs,
  searchGroups,
  settleGroup,
  subOf,
  tintOf,
  toHsl,
  vibrantHsl,
  wsPausedOf,
} from "../src/lib/media.js";

const track = { title: "Amber Skies", artist: "Fieldlight", album: "Harvest", art: "http://ma/a.jpg" };

test("times format as m:ss and never go negative", () => {
  assert.equal(fmtTime(0), "0:00");
  assert.equal(fmtTime(254000), "4:14");
  assert.equal(fmtTime(59999), "0:59");
  assert.equal(fmtTime(-5), "0:00");
  assert.equal(fmtTime(undefined), "0:00");
});

test("the slider fill is clamped and safe on a zero range", () => {
  assert.equal(pct(50, 100), "50.0%");
  assert.equal(pct(120, 100), "100.0%");
  assert.equal(pct(5, 0), "0.0%");
});

test("prune keeps the same object when nothing goes", () => {
  const m = { a: 1, b: 2 };
  assert.equal(prune(m, () => true), m);
  assert.deepEqual(prune(m, ([k]) => k === "a"), { a: 1 });
});

test("a playing group stream owns the bar with its metadata", () => {
  const v = mediaView({ src: "sendspin", state: 2, ss_state: 2, ...track }, false, false);
  assert.equal(v.playing, true);
  assert.equal(v.active, true);
  assert.equal(v.sendspin, true);
  assert.equal(v.srcParam, "sendspin");
  assert.equal(v.title, "Amber Skies");
  assert.equal(v.art, "http://ma/a.jpg");
});

test("a stopped group stream's cached track is not shown", () => {
  const v = mediaView({ src: "local", state: 1, ss_state: 1, ...track }, false, false);
  assert.equal(v.active, false);
  assert.equal(v.playing, false);
  assert.equal(v.title, "");
  assert.equal(v.srcParam, "local");
});

test("a held or tier-reported pause keeps the group on the bar, paused, aimed at the group", () => {
  for (const [held, maPaused] of [[true, false], [false, true]]) {
    const v = mediaView({ src: "local", state: 1, ss_state: 1, ...track }, held, maPaused);
    assert.equal(v.groupHeld, true);
    assert.equal(v.active, true);
    assert.equal(v.playing, false);
    assert.equal(v.state, 3);
    assert.equal(v.srcParam, "sendspin");
    assert.equal(v.title, "Amber Skies");
  }
});

test("the hold needs a group player and yields to a local stream", () => {
  assert.equal(mediaView({ src: "local", state: 1, ...track }, true, false).groupHeld, false);
  const local = mediaView({ src: "local", state: 2, ss_state: 1, ...track }, true, false);
  assert.equal(local.groupHeld, false);
  assert.equal(local.playing, true);
  assert.equal(local.title, "");
});

test("an announcement is neither playing nor active", () => {
  const v = mediaView({ src: "local", state: 4 }, false, false);
  assert.equal(v.announcing, true);
  assert.equal(v.active, false);
});

test("no payload yet is an empty view", () => {
  const v = mediaView(null, true, true);
  assert.equal(v.active, false);
  assert.equal(v.groupHeld, false);
  assert.equal(v.title, "");
});

test("the hold ends when the group resumes or the local player starts", () => {
  assert.equal(holdEnds({ src: "local", state: 1, ss_state: 2 }), true);
  assert.equal(holdEnds({ src: "local", state: 2, ss_state: 1 }), true);
  assert.equal(holdEnds({ src: "local", state: 1, ss_state: 1 }), false);
  assert.equal(holdEnds({ src: "sendspin", state: 3, ss_state: 1 }), false);
  assert.equal(holdEnds(null), false);
});

test("the socket reports a pause as a queue that is not playing but holds an item", () => {
  assert.equal(wsPausedOf(true, { state: "idle" }, { state: "idle", current_item: { name: "x" } }), true);
  assert.equal(wsPausedOf(true, { state: "paused" }, null), true);
  assert.equal(wsPausedOf(true, { state: "idle" }, { state: "idle", current_item: null }), false);
  assert.equal(wsPausedOf(true, { state: "playing" }, { state: "playing", current_item: {} }), false);
  assert.equal(wsPausedOf(false, { state: "paused" }, null), false);
});

test("the relay's pause is believed only while fresh and without a socket", () => {
  const now = 1_000_000;
  const ma = (d, age = 5, at = now) => ({ age, at, d });
  assert.equal(relayPausedOf(false, ma({ st: "idle", ti: "Amber Skies" }), now), true);
  assert.equal(relayPausedOf(false, ma({ st: "paused", ti: "" }), now), true);
  assert.equal(relayPausedOf(false, ma({ st: "idle", ti: "" }), now), false);
  assert.equal(relayPausedOf(true, ma({ st: "paused" }), now), false);
  // Stale either way: served old, or held here too long.
  assert.equal(relayPausedOf(false, ma({ st: "paused" }, 119), now + 2000), false);
  assert.equal(relayPausedOf(false, ma({ st: "paused" }, 0, now - 121000), now), false);
  // Nothing landed since boot.
  assert.equal(relayPausedOf(false, { age: -1, at: now, d: null }, now), false);
  assert.equal(relayPausedOf(false, null, now), false);
});

test("repeat cycles off, all, one", () => {
  assert.equal(nextRepeat(0), 2);
  assert.equal(nextRepeat(2), 1);
  assert.equal(nextRepeat(1), 0);
});

test("the relay's rows sort by name and hide pending unjoins", () => {
  const live = {
    g: [
      ["media_player.living", "Living Room", 38],
      ["media_player.kitchen", "Kitchen", 52],
      ["media_player.office", "Office", -1],
    ],
  };
  const cands = [
    ["media_player.kitchen", "Kitchen"],
    ["media_player.bedroom", "Bedroom"],
    ["media_player.attic", "Attic"],
  ];
  const { raw, members, addables } = groupRows({
    wsOn: false,
    live,
    cands,
    pending: { "media_player.office": { kind: "unjoin", at: 0 } },
  });
  assert.equal(raw.length, 3);
  assert.deepEqual(members.map((m) => m[1]), ["Kitchen", "Living Room"]);
  assert.deepEqual(addables.map((a) => a[1]), ["Attic", "Bedroom"]);
});

test("the socket's rows come from this player's group, and only available players can join", () => {
  const players = {
    me: { player_id: "me", name: "Satellite", volume_level: 30, available: true, group_members: [] },
    k: { player_id: "k", name: "Kitchen", volume_level: 40, available: true },
    o: { player_id: "o", name: "Office", available: false },
    b: { player_id: "b", name: "Bedroom", available: true },
  };
  const alone = groupRows({ wsOn: true, me: players.me, players, pending: {} });
  assert.deepEqual(alone.members, [["me", "Satellite", 30]]);
  assert.deepEqual(alone.addables.map((a) => a[0]), ["b", "k"]);
  const me = { ...players.me, group_members: ["me", "k", "gone"] };
  const grouped = groupRows({ wsOn: true, me, players: { ...players, me }, pending: {} });
  assert.deepEqual(grouped.members, [
    ["gone", "gone", -1],
    ["k", "Kitchen", 40],
    ["me", "Satellite", 30],
  ]);
  assert.deepEqual(grouped.addables.map((a) => a[0]), ["b"]);
});

// Dev3 leads a sync group with Dev12 in it; a Sonos plays alone; a group player plays through two
// speakers of its own; the rest are idle or not speakers at all.
const house = () => ({
  dev3: {
    player_id: "dev3",
    name: "Dev3",
    available: true,
    volume_level: 55,
    playback_state: "playing",
    group_members: ["dev3", "dev12"],
    can_group_with: ["sendspin", "sonos"],
    provider: "sendspin",
  },
  dev12: { player_id: "dev12", name: "Dev12", available: true, volume_level: 40, synced_to: "dev3", group_members: [], provider: "sendspin" },
  sonos: {
    player_id: "sonos",
    name: "Living Room",
    available: true,
    volume_level: 30,
    playback_state: "playing",
    provider: "sonos",
    current_media: { title: "Harvest Moon", artist: "Neil Young", image_url: "http://x/a.jpg", duration: 300, queue_item_id: "q1" },
    elapsed_time: 12,
    elapsed_time_last_updated: 1000,
    device_info: { mac_address: "AA:BB:CC:00:11:22" },
  },
  down: {
    player_id: "down",
    name: "Downstairs",
    type: "group",
    available: true,
    group_volume: 60,
    playback_state: "paused",
    group_members: ["kit", "den"],
    current_media: { title: "Dreams", queue_item_id: "q2" },
  },
  kit: { player_id: "kit", name: "Kitchen", available: true, playback_state: "paused", active_group: "down", provider: "sonos" },
  den: { player_id: "den", name: "Den", available: true, playback_state: "paused", active_group: "down", provider: "sonos" },
  air: { player_id: "air", name: "Living Room (AirPlay)", type: "protocol", available: true, playback_state: "playing" },
  bed: { player_id: "bed", name: "Bedroom", available: true, provider: "sonos" },
  spot: { player_id: "spot", name: "Spotify Box", available: true, playback_state: "playing", provider: "other", current_media: { title: "Native" } },
});

test("a member's group is its leader's: the same rows and badge, and joins aim at the leader", () => {
  const players = house();
  assert.equal(leaderOf(players.dev12, players).player_id, "dev3");
  assert.equal(leaderOf(players.dev3, players).player_id, "dev3");
  assert.equal(leaderOf(players.kit, players).player_id, "down");
  assert.equal(leaderOf(null, players), null);
  const fromMember = groupRows({ wsOn: true, me: players.dev12, players, pending: {} });
  const fromLeader = groupRows({ wsOn: true, me: players.dev3, players, pending: {} });
  assert.equal(fromMember.leader, "dev3");
  assert.deepEqual(fromMember.members, fromLeader.members);
  assert.deepEqual(fromMember.members.map((m) => m[0]), ["dev12", "dev3"]);
  // Not the leader (already in), not the playing Sonos or Spotify box, not the group player or
  // its members, not a protocol twin; and only what the leader's can_group_with names.
  assert.deepEqual(fromMember.addables.map((a) => a[0]), ["bed"]);
  const others = elsewhereRows({ wsOn: true, me: players.dev12, players });
  const paused = { ...players, bed: { ...players.bed, playback_state: "paused" } };
  const pausedOthers = elsewhereRows({ wsOn: true, me: players.dev12, players: paused });
  assert.deepEqual(groupRows({ wsOn: true, me: players.dev12, players: paused, pending: {}, others: pausedOthers }).addables, []);
  assert.deepEqual(groupRows({ wsOn: true, me: players.dev12, players, pending: {}, others }).addables.map((a) => a[0]), ["bed"]);
  const narrow = { ...players, dev3: { ...players.dev3, can_group_with: ["nobody"] } };
  assert.deepEqual(groupRows({ wsOn: true, me: players.dev12, players: narrow, pending: {} }).addables, []);
});

test("the relay's leader is its `l`, else the first member; other speakers' groups are not addable", () => {
  const live = {
    l: "media_player.dev3",
    g: [
      ["media_player.dev3", "Dev3", 55],
      ["media_player.dev12", "Dev12", 40],
    ],
  };
  const cands = [
    ["media_player.bed", "Bedroom"],
    ["media_player.sonos", "Living Room"],
    ["media_player.sub", "Sub"],
  ];
  const others = [{ id: "media_player.sonos", members: [["media_player.sub", "media_player.sub", 50]] }];
  const r = groupRows({ wsOn: false, live, cands, pending: {}, others });
  assert.equal(r.leader, "media_player.dev3");
  assert.deepEqual(r.addables.map((a) => a[0]), ["media_player.bed"]);
  assert.equal(groupRows({ wsOn: false, live: { g: live.g }, pending: {} }).leader, "media_player.dev3");
  assert.equal(groupRows({ wsOn: false, live: null, pending: {} }).leader, "");
});

test("sync is known when either side names the other, refused only when both lists say no", () => {
  const a = { player_id: "a", provider: "p1", can_group_with: ["b"] };
  const b = { player_id: "b", provider: "p2", can_group_with: [] };
  const c = { player_id: "c", provider: "p3", can_group_with: ["x"] };
  const d = { player_id: "d", provider: "p1", can_group_with: ["y"] };
  assert.equal(canSync(a, b), true);
  assert.equal(canSync(b, a), true);
  assert.equal(canSync(a, c), false);
  assert.equal(canSync(b, c), null);
  assert.equal(canSync(c, { ...d, can_group_with: ["p3"] }), true);
});

test("playing elsewhere lists one row per other group, never this one or a member of another", () => {
  const players = house();
  const rows = elsewhereRows({ wsOn: true, me: players.dev12, players });
  assert.deepEqual(rows.map((r) => r.id), ["down", "sonos", "spot"]);
  const [down, sonos, spot] = rows;
  assert.equal(down.group, true);
  assert.equal(down.state, "paused");
  assert.equal(down.volume, 60);
  assert.deepEqual(down.members.map((m) => m[0]), ["kit", "den"]);
  assert.equal(sonos.title, "Harvest Moon");
  assert.equal(sonos.transferable, true);
  assert.equal(sonos.sync, true);
  assert.equal(sonos.mac, "aabbcc001122");
  assert.deepEqual(sonos.clock, { pos: 12, at: 1000, dur: 300 });
  assert.equal(spot.transferable, false);
  assert.equal(spot.sync, null);
  // An idle speaker shows only while kept (a card paused during this visit).
  const kept = elsewhereRows({ wsOn: true, me: players.dev12, players, keep: new Set(["bed"]) });
  assert.ok(kept.some((r) => r.id === "bed" && r.state === "idle"));
  assert.deepEqual(elsewhereRows({ wsOn: true, me: null, players }), []);
});

test("the relay's rows map from `o`, members unnamed, volume control from the feature bits", () => {
  const live = {
    o: [
      ["media_player.down", "Downstairs", [["media_player.kit", 20], ["media_player.den", 25]], "Dreams", "", "https://i/a.jpg", "paused", 60, 4, 1, 1],
      ["media_player.spot", "Spotify Box", [], "Native", "", "", "playing", -1, 0, 0, 0],
    ],
  };
  const [down, spot] = elsewhereRows({ wsOn: false, live });
  assert.deepEqual(down.members, [
    ["media_player.kit", "media_player.kit", 20],
    ["media_player.den", "media_player.den", 25],
  ]);
  assert.equal(down.group, true);
  assert.equal(down.vctl, true);
  assert.equal(down.transferable, true);
  assert.equal(down.sync, null);
  assert.equal(spot.vctl, false);
  assert.equal(spot.transferable, false);
  assert.deepEqual(elsewhereRows({ wsOn: false, live: {} }), []);
});

test("bar cards keep first-seen order and outlive a pause, but not a stop or a join here", () => {
  const row = (id, state, title = "") => ({ id, state, name: id, title });
  const rows = [row("a", "playing"), row("b", "paused"), row("c", "playing"), row("e", "idle"), row("f", "idle", "Dreams")];
  const order = ["c", "b", "a", "d", "e", "f"];
  const all = new Set(order);
  // A paused speaker keeps its card only once seen playing this visit.
  assert.deepEqual(barCards({ rows, seen: new Set(["c", "a"]), order }).map((r) => r.id), ["c", "a"]);
  // Stopped (idle, nothing loaded) has no card; idle with a track loaded is a Sendspin pause.
  const cards = barCards({ rows, seen: all, order });
  assert.deepEqual(cards.map((r) => [r.id, r.state]), [["c", "playing"], ["b", "paused"], ["a", "playing"], ["f", "paused"]]);
  // A speaker paused from its card here keeps it, rebuilt from its last row once the relay drops it.
  const last = { d: row("d", "playing", "Song"), e: row("e", "playing") };
  const held = new Map([["d", 0], ["e", 0]]);
  assert.deepEqual(barCards({ rows, seen: all, order, last, held }).map((r) => [r.id, r.state]), [
    ["c", "playing"],
    ["b", "paused"],
    ["a", "playing"],
    ["d", "paused"],
    ["e", "paused"],
    ["f", "paused"],
  ]);
  assert.deepEqual(barCards({ rows, seen: all, order, last }).map((r) => r.id), ["c", "b", "a", "f"]);
  assert.deepEqual(barCards({ rows, seen: all, order, last, held, gone: new Set(["d", "c"]) }).map((r) => r.id), ["b", "a", "e", "f"]);
});

test("another speaker's group reads from its side: a group player is only its members", () => {
  const players = house();
  const down = groupRows({ wsOn: true, me: players.down, players, pending: {} });
  assert.equal(down.leader, "down");
  assert.deepEqual(down.members.map((m) => m[0]), ["den", "kit"]);
  const sonos = groupRows({ wsOn: true, me: players.sonos, players, pending: {}, others: [{ id: "dev3", state: "playing", members: [["dev12"]] }] });
  assert.deepEqual(sonos.members.map((m) => m[0]), ["sonos"]);
  assert.deepEqual(sonos.addables.map((a) => a[0]), ["bed"]);
});

test("an edit to another group settles on that group's rows", () => {
  const groups = { sonos: [["sonos", "Sonos", 30], ["bed", "Bedroom", 20]] };
  const pending = {
    bed: { kind: "join", at: 0, lead: "sonos" },
    kit: { kind: "join", at: 0, lead: "sonos" },
    den: { kind: "unjoin", at: 0, lead: "gone" },
  };
  assert.deepEqual(Object.keys(settleGroup(pending, [["bed", "Bedroom", 20]], 1000, groups)), ["kit"]);
  assert.deepEqual(Object.keys(settleGroup({ bed: { kind: "join", at: 0 } }, [], 1000, groups)), ["bed"]);
});

test("this group has music to lose when it plays, or holds a paused item", () => {
  assert.equal(hasOwnMusic({ wsOn: true, queue: { state: "playing" } }), true);
  assert.equal(hasOwnMusic({ wsOn: true, queue: { state: "idle", current_item: { name: "x" } } }), true);
  assert.equal(hasOwnMusic({ wsOn: true, queue: { state: "idle", current_item: null } }), false);
  assert.equal(hasOwnMusic({ wsOn: true, queue: null }), false);
  assert.equal(hasOwnMusic({ wsOn: false, live: { st: "playing" } }), true);
  assert.equal(hasOwnMusic({ wsOn: false, live: { st: "idle", ti: "Dreams" } }), true);
  assert.equal(hasOwnMusic({ wsOn: false, live: { st: "idle", ti: "" } }), false);
  assert.equal(hasOwnMusic({ wsOn: false, live: { st: "off", ti: "Dreams" } }), false);
});

test("the relay's group volume moves every member by the same step, clamped", () => {
  const rows = [
    ["a", "A", 20],
    ["b", "B", 90],
    ["c", "C", -1],
  ];
  assert.deepEqual(groupSteps(rows, 75), [
    ["a", 40],
    ["b", 100],
  ]);
  assert.deepEqual(groupSteps(rows, 0), [
    ["a", 0],
    ["b", 35],
  ]);
  assert.deepEqual(groupSteps([["c", "C", -1]], 50), []);
});

test("another speaker's playhead runs from the server's stamp, corrected for clock skew", () => {
  const row = { state: "playing", clock: { pos: 10, at: 100, dur: 200 } };
  assert.deepEqual(rowClock(row, 105_000), { dur: 200_000, pos: 15_000 });
  assert.deepEqual(rowClock(row, 107_000, 2), { dur: 200_000, pos: 15_000 });
  assert.deepEqual(rowClock({ ...row, state: "paused" }, 900_000), { dur: 200_000, pos: 10_000 });
  assert.deepEqual(rowClock(row, 900_000), { dur: 200_000, pos: 200_000 });
  assert.equal(rowClock({ state: "playing", clock: null }, 0), null);
});

test("artists, albums, playlists and podcasts open; each asks its own listing", () => {
  for (const t of ["artist", "album", "playlist", "podcast"]) assert.equal(canOpen({ media_type: t }), true);
  for (const t of ["track", "radio", "audiobook"]) assert.equal(canOpen({ media_type: t }), false);
  const item = (media_type) => ({ media_type, item_id: "7", provider: "spotify" });
  assert.deepEqual(openArgs(item("artist")), ["music/artists/artist_albums", { item_id: "7", provider_instance_id_or_domain: "spotify" }]);
  assert.deepEqual(openArgs(item("album")), [
    "music/albums/album_tracks",
    { item_id: "7", provider_instance_id_or_domain: "spotify", in_library_only: false },
  ]);
  assert.equal(openArgs(item("playlist"))[0], "music/playlists/playlist_tracks");
  assert.equal(openArgs(item("podcast"))[0], "music/podcasts/podcast_episodes");
  assert.equal(openArgs(item("track")), null);
});

test("an artist's albums split into albums and singles, newest first", () => {
  const albums = [
    { name: "Rumours", year: 1977, album_type: "album" },
    { name: "Tusk", year: 1979, album_type: "album" },
    { name: "Dreams", year: 1977, album_type: "single" },
    { name: "Live", album_type: "compilation" },
    { name: "EP One", year: 1980, album_type: "ep" },
  ];
  const s = albumSections(albums);
  assert.deepEqual(s.map(([k]) => k), ["albums", "singles"]);
  assert.deepEqual(s[0][1].map((a) => a.name), ["Tusk", "Rumours", "Live"]);
  assert.deepEqual(s[1][1].map((a) => a.name), ["EP One", "Dreams"]);
  assert.deepEqual(albumSections([{ name: "Only", year: 1 }]).map(([k]) => k), ["albums"]);
  assert.deepEqual(albumSections(null), []);
  assert.equal(albumSub({ year: 1977 }, { album: "Album" }), "1977");
  assert.equal(albumSub({}, { album: "Album" }), "Album");
});

test("opened lists keep their natural order", () => {
  const tracks = [
    { name: "b2", disc_number: 2, track_number: 1 },
    { name: "a3", disc_number: 1, track_number: 3 },
    { name: "a1", disc_number: 1, track_number: 1 },
  ];
  assert.deepEqual(openedOrder("album", tracks).map((t) => t.name), ["a1", "a3", "b2"]);
  assert.equal(tracks[0].name, "b2");
  const eps = [
    { name: "old", position: 1 },
    { name: "new", position: 3 },
    { name: "undated", position: 0 },
    { name: "dated", position: 0, metadata: { release_date: "2026-09-01" } },
  ];
  assert.deepEqual(openedOrder("podcast", eps).map((e) => e.name), ["new", "old", "dated", "undated"]);
  assert.deepEqual(openedOrder("playlist", tracks).map((t) => t.name), ["b2", "a3", "a1"]);
  assert.deepEqual(openedOrder("album", null), []);
});

test("an opened item's header line names its kind, makers, year and count", () => {
  const t = {
    search_kind: { artist: "Artist", album: "Album", playlist: "Playlist", podcast: "Podcast", track: "Track" },
    search_album_1: "1 album",
    search_albums_n: "%s albums",
    search_song_1: "1 song",
    search_songs_n: "%s songs",
    search_episode_1: "1 episode",
    search_episodes_n: "%s episodes",
  };
  assert.equal(heroSub({ media_type: "artist" }, 12, t), "Artist \u00b7 12 albums");
  assert.equal(heroSub({ media_type: "album", artists: [{ name: "Fleetwood Mac" }], year: 1977 }, 11, t), "Fleetwood Mac \u00b7 1977 \u00b7 11 songs");
  assert.equal(heroSub({ media_type: "album" }, null, t), "Album");
  assert.equal(heroSub({ media_type: "playlist", owner: "Me" }, 1, t), "Playlist \u00b7 Me \u00b7 1 song");
  assert.equal(heroSub({ media_type: "podcast", publisher: "NPR" }, 2, t), "Podcast \u00b7 NPR \u00b7 2 episodes");
  assert.equal(heroSub({ media_type: "track" }, 3, t), "Track");
});

test("an episode's row says when, how long, and how far through", () => {
  const now = Date.parse("2026-10-05T12:00:00Z");
  const fresh = episodeSub({ duration: 1800, metadata: { release_date: "2026-10-02T12:00:00Z" } }, now);
  assert.equal(fresh.mins, 30);
  assert.equal(fresh.left, 0);
  assert.equal(fresh.played, false);
  assert.ok(fresh.date && !/2026/.test(fresh.date));
  assert.ok(/2025/.test(episodeSub({ metadata: { release_date: "2025-06-01T12:00:00Z" } }, now).date));
  assert.equal(episodeSub({ duration: 1800, resume_position_ms: 600_000 }, now).left, 20);
  const done = episodeSub({ duration: 1800, resume_position_ms: 600_000, fully_played: true }, now);
  assert.equal(done.played, true);
  assert.equal(done.left, 0);
  assert.deepEqual(episodeSub({}, now), { date: "", mins: 0, left: 0, played: false });
});

test("a long listing's partial chunks gather into one result", () => {
  const parts = new Map();
  assert.equal(gatherResult(parts, { message_id: 4, partial: true, result: [1, 2] }), null);
  assert.equal(gatherResult(parts, { message_id: 4, partial: true, result: [3] }), null);
  assert.deepEqual(gatherResult(parts, { message_id: 4, result: [4] }), { result: [1, 2, 3, 4] });
  assert.equal(parts.size, 0);
  assert.deepEqual(gatherResult(parts, { message_id: 5, result: { ok: 1 } }), { result: { ok: 1 } });
});

test("group edits settle on membership, or at the deadline", () => {
  const raw = [["a", "A", 1]];
  const pending = { a: { kind: "join", at: 0 }, b: { kind: "join", at: 0 }, c: { kind: "unjoin", at: 0 }, d: { kind: "unjoin", at: 0 } };
  const rawWithD = [...raw, ["d", "D", 1]];
  assert.deepEqual(Object.keys(settleGroup(pending, rawWithD, 1000)), ["b", "d"]);
  assert.deepEqual(settleGroup(pending, rawWithD, 13000), {});
});

test("the group label names the first member and counts the rest", () => {
  assert.equal(groupLabel([], "Players"), "Players");
  assert.equal(groupLabel([["a", "Kitchen", 1]], "Players"), "Kitchen");
  assert.equal(groupLabel([["a", "Kitchen", 1], ["b", "Office", 1], ["c", "Den", 1]], ""), "Kitchen +2");
});

test("the queue clock runs while playing and is clamped to the track", () => {
  const q = { queue_id: "q1", state: "playing", elapsed_time: 10, elapsed_time_last_updated: 100, current_item: { duration: 200 } };
  assert.deepEqual(queueClock(q, 105000), { id: "q1", dur: 200000, pos: 15000 });
  assert.equal(queueClock({ ...q, state: "paused" }, 105000).pos, 10000);
  assert.equal(queueClock(q, 400000).pos, 200000);
  assert.equal(queueClock({ ...q, current_item: { duration: 0 } }, 0), null);
  assert.equal(queueClock(null, 0), null);
});

test("hsl conversion", () => {
  assert.deepEqual(toHsl(1, 0, 0), [0, 1, 0.5]);
  assert.deepEqual(toHsl(0, 0, 0), [0, 0, 0]);
  assert.deepEqual(toHsl(1, 1, 1), [0, 0, 1]);
  assert.equal(toHsl(0, 0, 1)[0], 240);
});

const px = (...rgbs) => rgbs.flatMap(([r, g, b]) => [r, g, b, 255]);

test("the tint is the artwork's most vivid colour family, not its average", () => {
  // A red cover on a mostly black background: the average is a dark brown, the tint is red.
  const cover = px(...Array(12).fill([8, 8, 8]), [220, 30, 40], [200, 20, 30], [235, 50, 60]);
  const [h, s] = vibrantHsl(cover);
  assert.ok(h >= 350 || h <= 5, `hue ${h}`);
  assert.ok(s > 0.6);
  // Greys, blacks and whites carry no vote.
  assert.equal(vibrantHsl(px([0, 0, 0], [128, 128, 128], [255, 255, 255])), null);
  // Transparent pixels are skipped.
  assert.equal(vibrantHsl([255, 0, 0, 0]), null);
  // A family split across a bucket edge still wins over a bigger single bucket's neighbour.
  const split = px([255, 120, 0], [255, 120, 0], [255, 135, 0], [255, 135, 0], [40, 40, 245], [40, 40, 245], [40, 40, 245]);
  assert.ok(vibrantHsl(split)[0] < 60);
});

test("the tint wash is floored and clamped per theme", () => {
  assert.equal(tintOf(null), "transparent");
  assert.equal(tintOf([30, 0.6, 0.45]), "hsla(30,60%,45%,0.5)");
  assert.equal(tintOf([30, 0.1, 0.1]), "hsla(30,45%,40%,0.5)");
  assert.equal(tintOf([30, 1, 0.9], "light"), "hsla(30,90%,62%,0.42)");
  assert.equal(tintOf([30, 0.6, 0.45], "light"), "hsla(30,60%,50%,0.42)");
});

test("search asks for every type under All and a real page under a pill", () => {
  assert.deepEqual(searchArgs("jazz", "all"), {
    search_query: "jazz",
    media_types: ["track", "artist", "album", "playlist", "radio", "podcast", "audiobook"],
    limit: 4,
  });
  assert.deepEqual(searchArgs("jazz", "album"), { search_query: "jazz", media_types: ["album"], limit: 25 });
});

test("search sections come back in render order, empty ones dropped", () => {
  const r = { albums: [{ uri: "a" }], tracks: [{ uri: "t" }], artists: [], radio: [{ uri: "r" }] };
  assert.deepEqual(searchGroups(r, "all").map(([ty]) => ty), ["track", "album", "radio"]);
  assert.deepEqual(searchGroups(r, "artist"), []);
  assert.deepEqual(searchGroups(null, "all"), []);
});

test("artwork follows Music Assistant's preference order and URL forms", () => {
  const base = "http://ma.local:8095";
  assert.equal(artThumb({ image: { type: "thumb", path: "x", proxy_id: "abc" } }, base), `${base}/imageproxy/abc?size=80`);
  assert.equal(artThumb({ image: { type: "thumb", path: "x", proxy_id: "abc" } }, ""), "");
  assert.equal(
    artThumb({ album: { image: { type: "thumb", path: "https://cdn/a.jpg", remotely_accessible: true } } }, ""),
    "https://cdn/a.jpg",
  );
  assert.equal(
    artThumb({ metadata: { images: [{ type: "fanart", path: "f" }, { type: "thumb", path: "/a b.jpg", provider: "fs" }] } }, base),
    `${base}/imageproxy?path=%252Fa%2520b.jpg&provider=fs&size=80`,
  );
  assert.equal(artThumb({ artists: [{ image: { type: "thumb", path: "data:image/png;base64,AA" } }] }, ""), "data:image/png;base64,AA");
  assert.equal(artThumb({ name: "none" }, base), "");
});

test("a row's sub-line is its kind, then its makers", () => {
  const kinds = { track: "Track", playlist: "Playlist", audiobook: "Audiobook", radio: "Radio" };
  assert.equal(subOf({ media_type: "track", artists: [{ name: "A" }, { name: "B" }] }, kinds), "Track \u00b7 A | B");
  assert.equal(subOf({ media_type: "playlist", owner: "Me" }, kinds), "Playlist \u00b7 Me");
  assert.equal(subOf({ media_type: "audiobook", authors: ["X", "Y"] }, kinds), "Audiobook \u00b7 X | Y");
  assert.equal(subOf({ media_type: "radio" }, kinds), "Radio");
});

test("recent searches dedupe case-insensitively, newest first, capped", () => {
  assert.deepEqual(addRecent(["jazz", "Fieldlight"], "fieldlight ", 8), ["fieldlight", "jazz"]);
  assert.deepEqual(addRecent(["a", "b", "c"], "d", 3), ["d", "a", "b"]);
  const list = ["a"];
  assert.equal(addRecent(list, "  ", 8), list);
});
