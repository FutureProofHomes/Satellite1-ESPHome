/**
 * The Home tab's pure logic: the orb's state from the assistant's phase, the sensor readings and
 * their calibration steppers, the per-word transcript tabs and the transcript's layout, timers and
 * the LED ring's colours.
 */
import assert from "node:assert/strict";
import test from "node:test";

import {
  agentName,
  clock,
  DEFAULT_AGENT,
  fitBytes,
  pipelineAgent,
  readAgentMap,
  rememberAgent,
  savedAgent,
  setupStamp,
  hexToRgb,
  hsvToRgb,
  isOn,
  lineId,
  offsetSpec,
  offsetText,
  orbState,
  orbTips,
  orbView,
  pctTo255,
  reading,
  rgbHue,
  ringPct,
  STAMP_GAP,
  stampLabel,
  stepOffset,
  timerLabel,
  timerLeft,
  transcriptRows,
  transcriptWindows,
} from "../src/lib/orb.js";

test("each voice assistant phase maps to its orb state", () => {
  assert.equal(orbState(1, true), "idle");
  assert.equal(orbState(2, true), "listening");
  assert.equal(orbState(3, true), "listening");
  assert.equal(orbState(4, true), "thinking");
  assert.equal(orbState(5, true), "speaking");
  assert.equal(orbState(10, true), "connecting");
  assert.equal(orbState(11, true), "error");
});

test("unknown and missing phases are idle; a lost stream is disabled whatever the phase", () => {
  for (const p of [0, 6, 99, undefined, null, "4"]) {
    assert.equal(orbState(p, true), p === "4" ? "thinking" : "idle", String(p));
  }
  for (const p of [1, 4, 11, undefined]) assert.equal(orbState(p, false), "disabled");
});

test("muted mics wear the paused idle look, except when the stream is gone", () => {
  assert.deepEqual(orbView(4, true, true), { state: "idle", label: "Mic muted", muted: true });
  assert.deepEqual(orbView(4, false, true), { state: "disabled", label: "Offline", muted: false });
  assert.deepEqual(orbView(5, true, false), { state: "speaking", label: "Speaking\u2026", muted: false });
  assert.deepEqual(orbView(10, true, false), { state: "connecting", label: "Not ready", muted: false });
});

const HOME = { word: "Okay Nabu", room: "Living Room", voice: true, tap: true, mute: true };

test("the orb's tip leads, then page tips and the word's examples alternate, filled in", () => {
  const tips = orbTips({ ...HOME, ai: true, music: true });
  assert.equal(tips[0], "Tap the orb to start a voice conversation.");
  assert.equal(tips[1], "Say, \u201cOkay Nabu, what devices can you control in the Living Room?\u201d");
  assert.ok(tips.includes("Say, \u201cOkay Nabu, play that one song about a yellow submarine.\u201d"));
  assert.ok(tips.includes("Say, \u201cOkay Nabu, make the Living Room cozy for movie night.\u201d"));
  assert.ok(tips.every(t => !/\{(word|other|room)\}/.test(t)), "no placeholder left");
  // More page tips than examples: every second line is an example, and all four come round.
  const said = tips.filter((_, i) => i % 2);
  assert.ok(tips.length > 8);
  assert.ok(said.every(t => t.startsWith("Say, \u201cOkay Nabu,") && !t.includes("pizza")));
  assert.equal(new Set(said).size, 4);
});

test("Home Assistant's own agent gets set phrases, and the room only when the device has one", () => {
  const roomed = orbTips({ ...HOME, ai: false });
  assert.ok(roomed.includes("Say, \u201cOkay Nabu, turn off the lights.\u201d It knows you're in the Living Room."));
  assert.ok(!roomed.some(t => t.includes("yellow submarine")));
  const roomless = orbTips({ ...HOME, room: "" });
  assert.ok(roomless.includes("Say, \u201cOkay Nabu, what time is it?\u201d"));
  assert.ok(!roomless.some(t => t.includes("the lights")));
  assert.ok(orbTips({ ...HOME, ai: true, room: "" }).includes("Say, \u201cOkay Nabu, what devices can you control in my home?\u201d"));
  assert.ok(orbTips({ ...HOME, ai: true, room: "Bob's Office" }).some(t => t.includes("in Bob's Office?")));
});

test("a tip retires once acted on, except tuning, which goes when the word is tuned", () => {
  const f = { ...HOME, other: "Hey Jarvis", untuned: true };
  const all = orbTips(f);
  assert.ok(all.includes("Swipe the conversation to talk to \u201cHey Jarvis\u201d instead."));
  const after = orbTips(f, ["orb", "swipe", "mute", "tune"]);
  assert.ok(!after.some(t => t.startsWith("Tap the orb") || t.startsWith("Swipe") || t.startsWith("Tap the mic")));
  assert.ok(after.includes("Tap \u201cTune it\u201d below to make \u201cOkay Nabu\u201d more accurate."));
  assert.ok(!orbTips({ ...f, untuned: false }).some(t => t.includes("Tune it")));
});

test("without Home Assistant nothing invites speaking; the page's own tips still show", () => {
  const tips = orbTips({ ...HOME, voice: false, stop: true, route: true, ring: true });
  assert.ok(!tips.some(t => t.startsWith("Say") || t.startsWith("Tap the orb") || t.startsWith("Route")));
  assert.deepEqual(tips, ["Tap the mic icon to mute the microphones.", "Tap \u201cCustomize\u201d to set your LED ring and theme colors."]);
  assert.deepEqual(orbTips({ word: "" }), []);
});

test("a switch reads on from either field /events uses", () => {
  assert.equal(isOn({ value: true }), true);
  assert.equal(isOn({ state: "ON" }), true);
  assert.equal(isOn({ value: false, state: "OFF" }), false);
  assert.equal(isOn(undefined), false);
});

test("readings format in the shown unit, and a sensor without one is a dash", () => {
  assert.equal(reading("21.44", 1, "\u00B0C"), "21.4\u00B0C");
  assert.equal(reading(21.44, 0, "\u00B0C"), "21\u00B0C");
  assert.equal(reading(20, 1, "\u00B0C", true), "68.0\u00B0F");
  assert.equal(reading(46.4, 0, "%"), "46%");
  assert.equal(reading(182, 0, " lx"), "182 lx");
  for (const v of [null, undefined, "", "NaN", NaN]) assert.equal(reading(v, 1, "\u00B0C"), "\u2014", String(v));
});

test("an offset is a delta: °F scales it without adding 32, and zero carries no sign", () => {
  assert.equal(offsetText(0.5, 1, "\u00B0"), "+0.5\u00B0");
  assert.equal(offsetText(-1.2, 1, "\u00B0"), "-1.2\u00B0");
  assert.equal(offsetText(1, 1, "\u00B0", true), "+1.8\u00B0");
  assert.equal(offsetText(0, 1, "\u00B0"), "0.0\u00B0");
  assert.equal(offsetText(-0.04, 1, "\u00B0"), "0.0\u00B0");
  assert.equal(offsetText(10, 0, " lx"), "+10 lx");
});

test("the stepper uses the entity's own step and range, never finer than the display step", () => {
  const temp = { step: 0.1, min: -20, max: 20 };
  assert.deepEqual(offsetSpec({ step: "0.1", min_value: "-20", max_value: "20" }, { step: 0.1, min: -5, max: 5 }), temp);
  assert.deepEqual(offsetSpec({ step: "0.5", min_value: "-10", max_value: "10" }, { step: 0.1, min: -20, max: 20 }), { step: 0.5, min: -10, max: 10 });
  assert.deepEqual(offsetSpec({ step: "0.1", min_value: "-50", max_value: "50" }, { step: 1, min: -50, max: 50 }), { step: 1, min: -50, max: 50 });
  assert.deepEqual(offsetSpec({ value: "0" }, temp), temp);
  assert.deepEqual(offsetSpec(undefined, temp), temp);
});

test("stepping rounds to the step's precision and stops at the range ends", () => {
  assert.equal(stepOffset(0.2, 1, 0.1, -20, 20), 0.3);
  assert.equal(stepOffset(-0.1, 1, 0.1, -20, 20), 0);
  assert.equal(stepOffset(19.95, 1, 0.1, -20, 20), 20);
  assert.equal(stepOffset(-20, -1, 0.1, -20, 20), -20);
  assert.equal(stepOffset(495, 1, 5, -500, 500), 500);
  assert.equal(stepOffset(3, -1, 1, -50, 50), 2);
});

const SLOTS = ["okay nabu", "hey jarvis"];
const texts = (w) => w.lines.map((l) => l.text);

test("each slot's window holds its word's lines, and the newest exchange is cued", () => {
  const lines = [
    { heard: true, w: "okay nabu", text: "a", at: 1 },
    { heard: false, w: "okay nabu", text: "b", at: 2 },
    { heard: true, w: "hey jarvis", text: "c", at: 3 },
    { heard: false, w: "hey jarvis", text: "d", at: 4 },
  ];
  const t = transcriptWindows(lines, SLOTS);
  assert.deepEqual(t.windows.map((w) => w.word), SLOTS);
  assert.deepEqual(t.windows.map(texts), [["a", "b"], ["c", "d"]]);
  assert.equal(t.newest, 1);
  // The cue is the newest thing said, not the answer after it, so an answer does not move the page.
  assert.deepEqual(t.cue, { index: 1, key: lineId(lines[2]) });
  // Nothing said yet: both windows, neither newest.
  assert.deepEqual(transcriptWindows([], SLOTS), { newest: null, cue: null, windows: [{ word: "okay nabu", lines: [] }, { word: "hey jarvis", lines: [] }] });
});

test("a word no slot holds shows nowhere, untagged lines show in every word's window, an empty slot in none", () => {
  const lines = [
    { text: "old", w: "", heard: true, at: 1 },
    { text: "gone", w: "alexa", heard: true, at: 2 },
    { text: "x", w: "hey jarvis", heard: false, at: 3 },
    { text: "gone too", w: "alexa", heard: true, at: 4 },
  ];
  const t = transcriptWindows(lines, SLOTS);
  assert.deepEqual(t.windows.map(texts), [["old"], ["old", "x"]]);
  assert.deepEqual([t.newest, t.cue], [1, null]);
  assert.deepEqual(transcriptWindows(lines, ["", "hey jarvis"]).windows.map(texts), [[], ["old", "x"]]);
});

test("until the slots are known there is one window with every line", () => {
  const lines = [{ text: "x", w: "hey jarvis" }, { text: "y", w: "okay nabu" }];
  assert.deepEqual(transcriptWindows(lines), { newest: null, cue: null, windows: [{ word: "", lines }] });
});

test("a stop word goes with the exchange it stopped", () => {
  const lines = [
    { heard: true, w: "hey jarvis", text: "a", at: 1 },
    { heard: true, w: "okay nabu", text: "b", at: 2 },
    { heard: true, w: "stop", text: "stop", at: 3 },
  ];
  const t = transcriptWindows(lines, SLOTS);
  assert.deepEqual(t.windows.map(texts), [["b", "stop"], ["a"]]);
  assert.deepEqual([t.newest, t.cue.index], [0, 0]);
  // With nothing before it to stop, it belongs to no slot.
  const lone = transcriptWindows([{ w: "stop", text: "s", heard: true }, { w: "hey jarvis", text: "j", heard: true }], SLOTS);
  assert.deepEqual(lone.windows.map(texts), [[], ["j"]]);
});

// Local dates, so the day boundaries are the test machine's own midnight, as they are the browser's.
const at = (y, mo, d, h, mi) => new Date(y, mo - 1, d, h, mi).getTime();
const NOW = at(2026, 10, 1, 15, 30);
const plain = (s) => ({ day: s.day, time: s.time.replace(/\s/g, " ") });

test("time headers read today, yesterday, the weekday within the week, then the date", () => {
  assert.deepEqual(plain(stampLabel(at(2026, 10, 1, 3, 12), NOW, "en-US")), { day: "Today", time: "3:12 AM" });
  assert.deepEqual(plain(stampLabel(at(2026, 10, 1, 0, 0), NOW, "en-US")), { day: "Today", time: "12:00 AM" });
  assert.deepEqual(plain(stampLabel(at(2026, 9, 30, 23, 59), NOW, "en-US")), { day: "Yesterday", time: "11:59 PM" });
  assert.deepEqual(plain(stampLabel(at(2026, 9, 25, 9, 40), NOW, "en-US")), { day: "Friday", time: "9:40 AM" });
  assert.deepEqual(plain(stampLabel(at(2026, 9, 24, 21, 40), NOW, "en-US")), { day: "Sep 24", time: "at 9:40 PM" });
  assert.deepEqual(plain(stampLabel(at(2025, 12, 31, 21, 40), NOW, "en-US")), { day: "Dec 31, 2025", time: "at 9:40 PM" });
  // A device clock a little ahead of the browser's is still today.
  assert.equal(stampLabel(NOW + 5000, NOW, "en-US").day, "Today");
});

const line = (heard, atS, text) => ({ heard, at: atS, w: "", text });

test("transcript rows group each side's run, tail its last bubble, and stamp long gaps", () => {
  const boot = at(2026, 10, 1, 3, 0);
  const lines = [
    line(true, 60, "Turn on the"),
    line(true, 64, "Turn on the porch light."),
    line(false, 66, "The porch light is on."),
    line(true, 66 + STAMP_GAP, "Is the garage closed?"),
    line(false, 68 + STAMP_GAP, "Yes."),
  ];
  const rows = transcriptRows(lines, boot, false, NOW, "en-US");
  const shape = rows.map((r) => (r.stamp ? `[${r.stamp.day} ${r.stamp.time.replace(/\s/g, " ")}]` : `${r.line.text}${r.run ? " run" : ""}${r.tail ? " tail" : ""}`));
  assert.deepEqual(shape, [
    "[Today 3:01 AM]",
    "Turn on the",
    "Turn on the porch light. run tail",
    "The porch light is on. tail",
    "[Today 3:16 AM]",
    "Is the garage closed? tail",
    "Yes. tail",
  ]);
  assert.equal(new Set(rows.map((r) => r.key)).size, rows.length);
});

test("a gap just short of the threshold earns no header, and no boot time means no headers", () => {
  const lines = [line(true, 0, "a"), line(false, STAMP_GAP - 1, "b"), line(true, 2 * STAMP_GAP - 1, "c")];
  assert.equal(transcriptRows(lines, 0, false, NOW).filter((r) => r.stamp).length, 2);
  assert.equal(transcriptRows(lines, null, false, NOW).some((r) => r.stamp), false);
});

test("the typing bubble joins an answer run it follows, and stands alone after a question", () => {
  const afterQuestion = transcriptRows([line(true, 1, "q")], null, true, NOW);
  assert.deepEqual(afterQuestion.map((r) => [r.key, r.run, r.tail]), [[lineId(line(true, 1, "q")), false, true], ["typing", false, undefined]]);
  const afterAnswer = transcriptRows([line(true, 1, "q"), line(false, 2, "a")], null, true, NOW);
  assert.equal(afterAnswer[1].tail, false);
  assert.equal(afterAnswer[2].run, true);
  assert.deepEqual(transcriptRows([], null, true, NOW).map((r) => r.key), ["typing"]);
  assert.deepEqual(transcriptRows([], null, false, NOW), []);
});

test("identical lines still get distinct keys", () => {
  const twice = [line(false, 5, "Sorry."), line(false, 5, "Sorry.")];
  const keys = transcriptRows(twice, null, false, NOW).map((r) => r.key);
  assert.notEqual(keys[0], keys[1]);
});

test("timers count down between polls, hold while paused, and never go negative", () => {
  const t = { left: 90, active: true };
  assert.equal(timerLeft(t, 1000, 1000), 90);
  assert.equal(timerLeft(t, 1000, 2999), 89);
  assert.equal(timerLeft(t, 1000, 200000), 0);
  assert.equal(timerLeft({ left: 90, active: false }, 1000, 60000), 90);
  assert.equal(clock(581), "09:41");
  assert.equal(clock(0), "00:00");
  assert.equal(clock(3900), "1:05:00");
  assert.equal(clock(-3), "00:00");
});

test("an unnamed timer is labelled by its set duration", () => {
  assert.equal(timerLabel({ name: "Pizza", total: 600 }), "Pizza");
  assert.equal(timerLabel({ name: "", total: 600 }), "10 min timer");
  assert.equal(timerLabel({ name: "", total: 5400 }), "1 h 30 min timer");
  assert.equal(timerLabel({ name: "", total: 45 }), "1 min timer");
  assert.equal(timerLabel({ name: "", total: 20 }), "20 s timer");
});

test("ring colours convert between hex, hue and RGB", () => {
  assert.deepEqual(hexToRgb("#a78bfa"), [167, 139, 250]);
  assert.deepEqual(hexToRgb("#fff"), [255, 255, 255]);
  assert.deepEqual(hsvToRgb(0, 1), [255, 0, 0]);
  assert.deepEqual(hsvToRgb(120, 1), [0, 255, 0]);
  assert.deepEqual(hsvToRgb(240, 0), [255, 255, 255]);
  for (const h of [0, 45, 120, 200, 290, 359]) assert.equal(rgbHue(...hsvToRgb(h, 1)), h, String(h));
  assert.equal(rgbHue(128, 128, 128), 0);
});

test("a typed message is cut to the text entity's byte limit, never mid-character", () => {
  const bytes = (s) => new TextEncoder().encode(s).length;
  assert.equal(fitBytes("hello", 255), "hello");
  assert.equal(fitBytes("a".repeat(300), 255).length, 255);
  // Two-byte é and a four-byte emoji: the cut lands before the character that would overflow.
  assert.equal(fitBytes("caf\u00e9", 4), "caf");
  assert.equal(fitBytes("ab\u{1F600}", 5), "ab");
  const mixed = fitBytes("\u00e9\u{1F600}x".repeat(60), 255);
  assert.ok(bytes(mixed) <= 255 && bytes(mixed) > 248, String(bytes(mixed)));
  assert.equal(fitBytes("", 255), "");
});

test("the typed-message agent is named from Home Assistant's list", () => {
  const cv = [[DEFAULT_AGENT, "Home Assistant"], ["conversation.ollama", "Ollama Studio"]];
  assert.equal(agentName(cv, ""), "Home Assistant");
  assert.equal(agentName(cv, "conversation.ollama"), "Ollama Studio");
  // Before the payload lands, and for an agent Home Assistant has since removed.
  assert.equal(agentName(undefined, ""), "Home Assistant");
  assert.equal(agentName(cv, "conversation.gone"), "conversation.gone");
});

test("a pipeline's agent is the person's answer, else the one agent its name matches", () => {
  const cv = [[DEFAULT_AGENT, "Home Assistant"], ["conversation.local_llama", "Local Llama"], ["conversation.openai_conversation", "OpenAI Conversation"]];
  const st = setupStamp(["Home Assistant", "Local Llama"], cv);
  assert.equal(pipelineAgent("Local Llama", cv, {}, st), "conversation.local_llama");
  assert.equal(pipelineAgent("local-llama", cv, {}, st), "conversation.local_llama");
  assert.equal(pipelineAgent("Home Assistant", cv, {}, st), DEFAULT_AGENT);
  // The built-in agent before Home Assistant's list has it, and names that say nothing.
  assert.equal(pipelineAgent("Home Assistant", [], {}, st), DEFAULT_AGENT);
  assert.equal(pipelineAgent("ChatGPT", cv, {}, st), null);
  assert.equal(pipelineAgent("preferred", cv, {}, st), null);
  // An answer wins over the name, is stored without its prefix, and lapses with its agent.
  assert.equal(pipelineAgent("ChatGPT", cv, { ChatGPT: `openai_conversation~${st}` }, st), "conversation.openai_conversation");
  assert.equal(pipelineAgent("Local Llama", cv, { "Local Llama": `openai_conversation~${st}` }, st), "conversation.openai_conversation");
  assert.equal(pipelineAgent("preferred", cv, { preferred: `home_assistant~${st}` }, st), DEFAULT_AGENT);
  assert.equal(pipelineAgent("ChatGPT", cv, { ChatGPT: `gone~${st}` }, st), null);
  assert.equal(pipelineAgent("Local Llama", cv, { "Local Llama": `gone~${st}` }, st), "conversation.local_llama");
  // Two agents with the pipeline's name is a guess, so it asks.
  assert.equal(pipelineAgent("Twin", [["conversation.a", "Twin"], ["conversation.twin", "Other"]], {}, st), null);
});

test("an answer is asked again once Home Assistant's pipelines or agents change", () => {
  const cv = [[DEFAULT_AGENT, "Home Assistant"], ["conversation.openai_conversation", "OpenAI Conversation"]];
  const pipes = ["Home Assistant", "ChatGPT"];
  const st = setupStamp(pipes, cv);
  assert.match(st, /^[0-9a-z]{4}$/);
  // Order and agents' display names change nothing; a pipeline or an agent coming or going does.
  assert.equal(setupStamp([...pipes].reverse(), [...cv].reverse()), st);
  assert.equal(setupStamp(pipes, [[DEFAULT_AGENT, "Assist"], cv[1]]), st);
  const moved = [setupStamp([...pipes, "Kitchen"], cv), setupStamp(["Home Assistant", "GPT"], cv), setupStamp(pipes, [...cv, ["conversation.claude", "Claude"]])];
  for (const m of moved) assert.notEqual(m, st);
  const known = { preferred: `openai_conversation~${st}` };
  assert.equal(pipelineAgent("preferred", cv, known, st), "conversation.openai_conversation");
  assert.equal(pipelineAgent("preferred", cv, known, moved[2]), null);
  // A stale answer for a pipeline whose name matches is still asked, not quietly replaced.
  assert.equal(pipelineAgent("Home Assistant", cv, { "Home Assistant": `openai_conversation~${st}` }, moved[0]), null);
  // Answers from before stamps count as stale, once.
  assert.equal(pipelineAgent("preferred", cv, { preferred: "openai_conversation" }, st), null);
  assert.deepEqual(savedAgent(known, "preferred"), { agent: "conversation.openai_conversation", stamp: st });
  assert.deepEqual(savedAgent({ preferred: "openai_conversation" }, "preferred"), { agent: "conversation.openai_conversation", stamp: "" });
  assert.equal(savedAgent(known, "other"), null);
});

test("pipeline answers fit the text entity, oldest unused first", () => {
  const bytes = (s) => new TextEncoder().encode(s).length;
  assert.deepEqual(readAgentMap(""), {});
  assert.deepEqual(readAgentMap("not json"), {});
  assert.deepEqual(readAgentMap("[1]"), {});
  const one = rememberAgent({}, "ChatGPT", "conversation.openai_conversation", "ab12");
  assert.deepEqual(JSON.parse(one), { ChatGPT: "openai_conversation~ab12" });
  // Answering again moves the pipeline to the newest end.
  assert.deepEqual(Object.keys(JSON.parse(rememberAgent({ a: "x", b: "y" }, "a", "z", "ab12"))), ["b", "a"]);
  const long = (n) => `pipeline ${n} ${"x".repeat(40)}`;
  let map = {};
  for (let n = 0; n < 8; n++) map = JSON.parse(rememberAgent(map, long(n), "conversation.agent", "ab12", [long(0)]));
  const kept = Object.keys(map);
  assert.ok(bytes(JSON.stringify(map)) <= 255);
  assert.ok(kept.includes(long(0)) && kept.includes(long(7)), kept.join());
  assert.ok(!kept.includes(long(1)));
  assert.equal(rememberAgent({}, "y".repeat(300), "conversation.a", "ab12"), null);
});

test("ring brightness is a percent, and an off ring reads zero", () => {
  assert.equal(ringPct({ state: "ON", brightness: 255 }), 100);
  assert.equal(ringPct({ state: "ON", brightness: 168 }), 66);
  assert.equal(ringPct({ state: "ON" }), 100);
  assert.equal(ringPct({ state: "OFF", brightness: 255 }), 0);
  assert.equal(ringPct(undefined), 0);
  assert.equal(pctTo255(100), 255);
  assert.equal(pctTo255(66), 168);
});
