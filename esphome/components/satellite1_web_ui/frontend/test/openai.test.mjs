import test from "node:test";
import assert from "node:assert/strict";
import {
  canConnect,
  canSave,
  deriveEndpoints,
  discovery,
  isLocalHost,
  keyWillBeDropped,
  modelChoices,
  normalizeBaseUrl,
  originOf,
  protocolLabel,
  reconcile,
  stateLabel,
  validKey,
  validModel,
  validVoice,
  voiceChoices,
} from "../src/lib/openai.js";

// These cases mirror tests/openai_realtime/test_rt_util.cpp: the page and the device must derive
// the same URLs from the same base URL.
test("base URL to Realtime, models and voices URLs", () => {
  assert.deepEqual(deriveEndpoints("https://api.openai.com/v1", "gpt-realtime-2.1"), {
    realtimeUrl: "wss://api.openai.com/v1/realtime?model=gpt-realtime-2.1",
    modelsUrl: "https://api.openai.com/v1/models",
    voicesUrl: "https://api.openai.com/v1/audio/voices",
    origin: "https://api.openai.com",
    secure: true,
  });
  assert.equal(deriveEndpoints("  https://API.openai.com/v1/// ", "m").realtimeUrl, "wss://api.openai.com/v1/realtime?model=m");
  const pasted = deriveEndpoints("wss://api.openai.com/v1/realtime", "gpt-realtime");
  assert.equal(pasted.realtimeUrl, "wss://api.openai.com/v1/realtime?model=gpt-realtime");
  assert.equal(pasted.modelsUrl, "https://api.openai.com/v1/models");
  const relay = deriveEndpoints("http://relay.lan:8080/v1?api-version=2025-08-28", "m x");
  assert.equal(relay.realtimeUrl, "ws://relay.lan:8080/v1/realtime?api-version=2025-08-28&model=m%20x");
  assert.equal(relay.voicesUrl, "http://relay.lan:8080/v1/audio/voices");
  assert.equal(relay.origin, "http://relay.lan:8080");
  assert.equal(deriveEndpoints("wss://x.example/openai/v1/realtime?model=fixed", "other").realtimeUrl, "wss://x.example/openai/v1/realtime?model=fixed");
});

test("invalid base URLs are refused like the device refuses them", () => {
  for (const bad of ["", "ftp://x", "https://", "https://user:pw@host/v1", "https://ho st/v1", "api.openai.com/v1", "https://h/é"]) {
    assert.equal(deriveEndpoints(bad, "m"), null, bad);
  }
  assert.equal(normalizeBaseUrl(" https://a/v1/ "), "https://a/v1");
  assert.equal(normalizeBaseUrl("https://a/v1/?x=1"), "https://a/v1?x=1");
});

test("local servers are recognised", () => {
  for (const u of ["http://192.168.1.10:8000/v1", "http://10.0.0.2/v1", "http://172.20.1.1/v1", "ws://homeassistant.local:8099/v1", "http://localhost:8000/v1", "http://[fd00::1]:8000/v1"]) {
    assert.ok(isLocalHost(u), u);
  }
  for (const u of ["https://api.openai.com/v1", "http://172.40.0.1/v1", "nope"]) assert.ok(!isLocalHost(u), u);
});

test("a stored key is dropped when the host changes and no key is typed", () => {
  const saved = { key_set: true, base_url: "https://api.openai.com/v1" };
  assert.equal(keyWillBeDropped(saved, "https://api.openai.com/v1/", ""), false);
  assert.equal(keyWillBeDropped(saved, "wss://api.openai.com/v1/realtime", ""), false);
  assert.equal(keyWillBeDropped(saved, "http://relay.lan/v1", ""), true);
  assert.equal(keyWillBeDropped(saved, "http://relay.lan/v1", "sk-new"), false);
  assert.equal(keyWillBeDropped({ key_set: false, base_url: "x" }, "http://relay.lan/v1", ""), false);
  assert.equal(originOf("https://api.openai.com/v1"), "https://api.openai.com");
});

test("validation and readiness", () => {
  assert.ok(validModel("gpt-realtime-2.1"));
  assert.ok(validModel("org/model:v1"));
  assert.ok(!validModel(""));
  assert.ok(!validModel("has space"));
  assert.ok(validVoice("marin"));
  assert.ok(validVoice("voice_123abc"));
  assert.ok(validVoice("af_bella+af_sky"));
  assert.ok(!validVoice("a b"));
  assert.ok(!validVoice("x".repeat(64)));
  assert.ok(validKey(""));
  assert.ok(validKey("sk-proj-abc_DEF-123"));
  assert.ok(!validKey("sk with space"));
  assert.ok(!validKey('sk"quote'));
  assert.ok(canConnect("https://api.openai.com/v1", "gpt-realtime", true));
  assert.ok(!canConnect("https://api.openai.com/v1", "gpt-realtime", false));
  assert.ok(canConnect("http://relay.lan/v1", "gpt-realtime", false));
  assert.ok(!canConnect("nope", "gpt-realtime", true));
});

const okResult = (over = {}) => ({
  gen: 3,
  st: "ok",
  base: "https://api.openai.com/v1",
  err: "",
  list: true,
  ids: ["gpt-realtime", "gpt-realtime-2.1"],
  voices: [["marin", "", 0], ["cedar", "", 0], ["voice_1", "Narrator", 1]],
  ...over,
});

test("discovery only counts the latest request for the address on screen", () => {
  assert.deepEqual(discovery(null, "x", null), { state: "idle", ready: false });
  assert.equal(discovery(okResult(), "https://api.openai.com/v1", 2).state, "loading"); // older request
  assert.equal(discovery(okResult(), "http://relay/v1", 3).state, "loading"); // other address
  const d = discovery(okResult(), "https://api.openai.com/v1/", 3);
  assert.equal(d.state, "ok");
  assert.ok(d.ready && d.hasModelList);
  assert.equal(d.serverVoices, 1);
  assert.equal(discovery(okResult({ st: "loading" }), "https://api.openai.com/v1", 3).state, "loading");
  const err = discovery(okResult({ st: "error", err: "HTTP 401" }), "https://api.openai.com/v1", 3);
  assert.equal(err.state, "error");
  assert.ok(!err.ready);
  // Reachable without a model list: ready, but the model is typed.
  const bare = discovery(okResult({ list: false, ids: [], voices: [] }), "https://api.openai.com/v1", 3);
  assert.ok(bare.ready && !bare.hasModelList);
  assert.equal(bare.voices.length, 0);
});

test("pickers keep a saved choice the server does not offer, and label custom voices", () => {
  assert.deepEqual(modelChoices(["a", "b"], "b"), [["a", "a"], ["b", "b"]]);
  assert.deepEqual(modelChoices(["a", "b"], "pinned")[0], ["pinned", "pinned (not offered by this server)"]);
  assert.deepEqual(voiceChoices([["marin", "", 0], ["voice_1", "Narrator", 1]], "marin"), [["marin", "marin"], ["voice_1", "Narrator (voice_1)"]]);
  assert.equal(voiceChoices([["marin", "", 0]], "gone")[0][0], "gone");
  assert.equal(reconcile("gpt-realtime", ["a", "b"], true), "gpt-realtime");
  assert.equal(reconcile("gpt-realtime", ["a", "b"], false), "a");
  assert.equal(reconcile("b", ["a", "b"], false), "b");
  assert.equal(reconcile("x", [], false), "x");
});

test("Save needs a successful connection, except for the on/off switch alone", () => {
  const saved = { enabled: false, base_url: "https://api.openai.com/v1", model: "gpt-realtime", voice: "marin", key_set: true };
  const draft = { enabled: false, base: "https://api.openai.com/v1", model: "gpt-realtime", voice: "marin", key: "", clearKey: false };
  const ready = { ready: true };
  assert.ok(!canSave({ saved, draft, disc: ready })); // nothing changed
  assert.ok(canSave({ saved, draft: { ...draft, enabled: true }, disc: null })); // switch alone
  assert.ok(!canSave({ saved, draft: { ...draft, model: "gpt-realtime-2.1" }, disc: null }));
  assert.ok(canSave({ saved, draft: { ...draft, model: "gpt-realtime-2.1" }, disc: ready }));
  assert.ok(!canSave({ saved, draft: { ...draft, key: "sk new" }, disc: ready })); // invalid key
  assert.ok(!canSave({ saved, draft: { ...draft, voice: "" }, disc: ready }));
  assert.ok(!canSave({ saved: null, draft, disc: ready }));
});

test("labels", () => {
  assert.equal(stateLabel("idle"), "Idle");
  assert.equal(stateLabel("active", "speaking"), "In a conversation - speaking");
  assert.equal(stateLabel("connecting"), "Connecting…");
  assert.equal(protocolLabel("beta"), "Realtime beta (compatible servers)");
  assert.equal(protocolLabel(""), "Not connected yet");
});
