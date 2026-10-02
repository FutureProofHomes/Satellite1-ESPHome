/**
 * The media bar's logic: which player owns the bar, the held pause that keeps a stopped group
 * stream resumable, the tiers' pause signals, the group rows, and Music Assistant's results.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  addRecent,
  artThumb,
  fmtTime,
  groupLabel,
  groupRows,
  holdEnds,
  mediaView,
  nextRepeat,
  pct,
  prune,
  queueClock,
  relayPausedOf,
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
