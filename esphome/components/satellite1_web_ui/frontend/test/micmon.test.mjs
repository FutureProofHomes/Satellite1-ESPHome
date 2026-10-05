/**
 * The mic monitor's wire format and recorder (src/lib/micframes.js). The device side is
 * mic_monitor.cpp; these pin the byte layout both sides agree on, and the recorder's promise that
 * the file's timeline matches the room's.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { FLAG_IDLE, FLAG_MUTED, FLAG_XMOS, HEADER, createParser, createRecorder, encodeFrame, headerEvents, recordingStem } from "../src/lib/micframes.js";

const frame = (over = {}) => ({ seq: 0, count: 0, dropped: 0, flags: 0, phase: 0, wakeSeq: 0, wakeSlot: 255, scores: [0, 0, 0], stt: new Int16Array(0), ww: new Int16Array(0), ...over });
const audio = (seq, n, wall) => [frame({ seq, count: n, stt: new Int16Array(n).fill(1), ww: new Int16Array(n).fill(2) }), wall];

test("a message round-trips through the parser with every field intact", () => {
  const msg = encodeFrame({ seq: 123456, dropped: 7, flags: FLAG_MUTED | FLAG_XMOS, phase: 3, wakeSeq: 9, wakeSlot: 1, scores: [10, 200, 255], stt: [-32768, 0, 32767], ww: [1, -1, 5] });
  assert.equal(msg.length, HEADER + 3 * 4);
  const [f] = createParser().push(msg);
  assert.equal(f.seq, 123456);
  assert.equal(f.count, 3);
  assert.equal(f.dropped, 7);
  assert.equal(f.flags, FLAG_MUTED | FLAG_XMOS);
  assert.equal(f.phase, 3);
  assert.equal(f.wakeSeq, 9);
  assert.equal(f.wakeSlot, 1);
  assert.deepEqual(f.scores, [10, 200, 255]);
  assert.deepEqual([...f.stt], [-32768, 0, 32767]);
  assert.deepEqual([...f.ww], [1, -1, 5]);
});

test("messages split at any byte still come out whole and in order", () => {
  const a = encodeFrame({ seq: 0, stt: [1, 2], ww: [3, 4] });
  const b = encodeFrame({ seq: 2 });
  const c = encodeFrame({ seq: 2, stt: [5], ww: [6] });
  const all = new Uint8Array([...a, ...b, ...c]);
  for (let cut = 1; cut < all.length; cut++) {
    const p = createParser();
    const got = [...p.push(all.subarray(0, cut)), ...p.push(all.subarray(cut))];
    assert.deepEqual(got.map((f) => [f.seq, f.count]), [[0, 2], [2, 0], [2, 1]], `cut at ${cut}`);
  }
});

test("garbage before a message is skipped by resynchronising on the magic", () => {
  const msg = encodeFrame({ seq: 42, stt: [7], ww: [8] });
  const got = createParser().push(new Uint8Array([1, 2, 3, ...msg]));
  assert.equal(got.length, 1);
  assert.equal(got[0].seq, 42);
});

test("header changes become markers, and the first header has nothing to compare against", () => {
  const a = frame();
  assert.deepEqual(headerEvents(null, a), []);
  assert.deepEqual(headerEvents(a, frame({ wakeSeq: 1, wakeSlot: 0 })), [{ kind: "wake", text: "Wake word (Primary)" }]);
  assert.deepEqual(headerEvents(a, frame({ wakeSeq: 1, wakeSlot: 2 })), [{ kind: "wake", text: "Wake word (Stop word)" }]);
  assert.deepEqual(headerEvents(a, frame({ wakeSeq: 1, wakeSlot: 255 })), [{ kind: "wake", text: "Wake word" }]);
  assert.deepEqual(headerEvents(a, frame({ phase: 2 }), (p) => `P${p}`), [{ kind: "phase", text: "P2" }]);
  assert.deepEqual(headerEvents(frame({ phase: 2 }), a), [], "going idle is not a marker");
  assert.deepEqual(headerEvents(a, frame({ flags: FLAG_XMOS | FLAG_MUTED })).map((e) => e.text), ["XMOS not ready", "Muted"]);
  assert.deepEqual(headerEvents(frame({ flags: FLAG_XMOS | FLAG_MUTED }), a).map((e) => e.text), ["XMOS ready", "Unmuted"]);
});

test("the recorder keeps both channels in step and counts frames", () => {
  const r = createRecorder({ rate: 100 });
  r.add(...audio(0, 10, 0));
  r.add(...audio(10, 5, 50));
  assert.equal(r.frames, 15);
  assert.equal(r.stt.reduce((n, c) => n + c.length, 0), 15);
  assert.equal(r.ww.reduce((n, c) => n + c.length, 0), 15);
  assert.deepEqual(r.markers, []);
});

test("a jump in the sample index becomes that much silence and a marker", () => {
  const r = createRecorder({ rate: 100 });
  r.add(...audio(0, 10, 0));
  r.add(...audio(30, 10, 300));
  assert.equal(r.frames, 40);
  assert.deepEqual(r.markers, [{ at: 10, kind: "gap", text: "Audio dropped" }]);
  assert.deepEqual([...r.stt[1]], new Array(20).fill(0));
});

test("an idle microphone is filled by the browser's clock, since the device index stands still", () => {
  const r = createRecorder({ rate: 100 });
  r.add(...audio(0, 10, 1000));
  r.add(frame({ flags: FLAG_IDLE }), 1250);
  r.add(frame({ flags: FLAG_IDLE }), 2000);
  // Two seconds after the last audio, ten frames arrive: 200 frames of wall time, 10 of them audio.
  r.add(...audio(10, 10, 3000));
  assert.equal(r.frames, 10 + 190 + 10);
  assert.deepEqual(r.markers, [{ at: 10, kind: "idle", text: "Microphone idle" }]);
});

test("a reconnect inserts the outage and forgets the old sample index", () => {
  const r = createRecorder({ rate: 100 });
  r.add(...audio(0, 10, 0));
  r.restart(500);
  r.add(...audio(0, 10, 600));
  assert.equal(r.frames, 10 + 50 + 10);
  assert.deepEqual(r.markers.map((m) => [m.at, m.kind]), [[10, "reconnect"]]);
});

test("the take stops at its limit, cutting the last chunk to fit", () => {
  const r = createRecorder({ rate: 10, maxSeconds: 2 });
  assert.equal(r.add(...audio(0, 15, 0)), true);
  assert.equal(r.add(...audio(15, 15, 100)), false);
  assert.equal(r.full, true);
  assert.equal(r.frames, 20);
  assert.equal(r.stt[1].length, 5);
  assert.equal(r.add(...audio(30, 15, 200)), false);
  assert.equal(r.frames, 20);
});

test("manual marks land at the current frame", () => {
  const r = createRecorder({ rate: 100 });
  r.add(...audio(0, 25, 0));
  r.mark("xmos", "XMOS firmware 1.2.3");
  assert.deepEqual(r.markers, [{ at: 25, kind: "xmos", text: "XMOS firmware 1.2.3" }]);
});

test("the file stem names the device, the XMOS version and the minute", () => {
  const d = new Date(2026, 9, 5, 14, 3, 59);
  assert.equal(recordingStem("Kitchen Sat", "v1.2.3", d), "Kitchen-Sat_xmos-1.2.3_2026-10-05T14-03");
  assert.equal(recordingStem("satellite1-c5ac00", "1.3.0-beta.2", d), "satellite1-c5ac00_xmos-1.3.0-beta.2_2026-10-05T14-03");
  assert.equal(recordingStem("", "XMOS not responding", d), "satellite1_xmos-unknown_2026-10-05T14-03");
});
