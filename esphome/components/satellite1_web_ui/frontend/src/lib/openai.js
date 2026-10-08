/**
 * Settings > OpenAI: the Realtime connection the device's openai_realtime component uses - OpenAI
 * itself, or any compatible server, including one on the local network.
 *
 * The device owns the settings (GET/POST /api/sat1/openai) and never returns the API key - only
 * whether one is stored and its last four characters. Models and voices come from the server
 * itself: POST /api/sat1/openai/models makes the *device* fetch <base>/models and
 * <base>/audio/voices (the result rides the next GET). The device does it, not the browser,
 * because it can reach a LAN server the browser may not, the stored key never has to come back to
 * the browser, and api.openai.com refuses browser-origin requests anyway. A successful fetch is
 * also the page's proof that the address and key work: the model and voice pickers unlock on it.
 *
 * Everything below the API calls is pure and covered by test/openai.test.mjs.
 */
import { request, requestJson } from "./device.js";

export const DEFAULT_BASE_URL = "https://api.openai.com/v1";
export const OPENAI_ORIGIN = "https://api.openai.com";
/** The device's stored field sizes (openai_realtime.h); longer values are refused there. */
export const LIMITS = { base_url: 159, model: 63, voice: 63, api_key: 255 };

export function readOpenAI() {
  return requestJson("/api/sat1/openai");
}

/** The device's 400 body names the field it refused; anything else reads as a transport failure. */
export async function saveOpenAI(cfg) {
  const r = await request("/api/sat1/openai", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(cfg),
    quiet: true,
  });
  let body = null;
  try {
    body = JSON.parse(r.text);
  } catch {
    body = null;
  }
  return { ok: r.ok && !!body?.ok, err: body?.err || (r.ok ? null : `HTTP ${r.status}`) };
}

/** Starts the device's discovery (models + voices) for this base URL; `apiKey` empty = the stored key. */
export async function discover(baseUrl, apiKey) {
  const body = { base_url: normalizeBaseUrl(baseUrl) };
  if (apiKey) body.api_key = apiKey;
  return requestJson("/api/sat1/openai/models", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    quiet: true,
  });
}

export function testConnection() {
  return requestJson("/api/sat1/openai/test", { method: "POST", quiet: true });
}

// ------------------------------------------------------------------------------------------------
// Pure helpers
// ------------------------------------------------------------------------------------------------

/** Trims whitespace and trailing slashes on the path, as the device does before storing. */
export function normalizeBaseUrl(url) {
  const s = String(url || "").trim();
  const q = s.indexOf("?");
  const path = (q < 0 ? s : s.slice(0, q)).replace(/\/+$/, "");
  return path + (q < 0 ? "" : s.slice(q));
}

/**
 * Mirrors the device's derive_endpoints (rt_util.cpp): null when the URL would be refused,
 * otherwise the URLs it stands for. Kept in step so the page can say "not a valid address" before
 * a round trip, and show the WebSocket URL a session will open.
 */
export function deriveEndpoints(baseUrl, model) {
  const s = normalizeBaseUrl(baseUrl);
  if (!s || s.length > LIMITS.base_url) return null;
  if (/[\s"\\<>\u0000-\u001f\u007f-\uffff]/.test(s)) return null;
  const m = /^(https?|wss?):\/\/(.*)$/i.exec(s);
  if (!m) return null;
  const secure = /^(https|wss)$/i.test(m[1]);
  let rest = m[2];
  let query = "";
  const q = rest.indexOf("?");
  if (q >= 0) {
    query = rest.slice(q + 1).split("#")[0];
    rest = rest.slice(0, q);
  }
  const slash = rest.indexOf("/");
  const host = (slash < 0 ? rest : rest.slice(0, slash)).toLowerCase();
  const path = slash < 0 ? "" : rest.slice(slash);
  if (!host || host.includes("@") || host.endsWith(":")) return null;
  const isRt = path.endsWith("/realtime");
  const basePath = isRt ? path.slice(0, -"/realtime".length) : path;
  const rtPath = isRt ? path : `${path}/realtime`;
  let rtQuery = query;
  if (!`&${query}`.includes("&model=") && model) rtQuery += `${rtQuery ? "&" : ""}model=${encodeURIComponent(model)}`;
  const http = `${secure ? "https" : "http"}://${host}`;
  return {
    realtimeUrl: `${secure ? "wss" : "ws"}://${host}${rtPath}${rtQuery ? `?${rtQuery}` : ""}`,
    modelsUrl: `${http}${basePath}/models`,
    voicesUrl: `${http}${basePath}/audio/voices`,
    origin: http,
    secure,
  };
}

export const originOf = (url) => deriveEndpoints(url, "")?.origin ?? null;

/** A host on the local network: private IPv4 ranges, localhost, *.local / *.lan / *.home.arpa. */
export function isLocalHost(url) {
  const ep = deriveEndpoints(url, "");
  if (!ep) return false;
  const host = ep.origin.replace(/^https?:\/\//, "").replace(/:\d+$/, "").replace(/^\[|\]$/g, "");
  if (/^(localhost|.*\.local|.*\.lan|.*\.home\.arpa|.*\.internal)$/.test(host)) return true;
  if (/^(10\.|127\.|192\.168\.|169\.254\.)/.test(host)) return true;
  if (/^172\.(1[6-9]|2\d|3[01])\./.test(host)) return true;
  return /^(fc|fd|fe80)/i.test(host);
}

/** Model names the device accepts: letters, digits and - _ . : / (1-63). */
export const validModel = (m) => /^[A-Za-z0-9._:/-]{1,63}$/.test(String(m || ""));

/** Voice ids the device accepts: letters, digits and - _ . : / + (1-63). */
export const validVoice = (v) => /^[A-Za-z0-9._:/+-]{1,63}$/.test(String(v || ""));

/** API keys: printable ASCII, no spaces or quotes, at most 255. Empty means "keep the stored one". */
export const validKey = (k) => k === "" || /^[\x21\x23-\x5b\x5d-\x7e]{1,255}$/.test(k);

/**
 * Whether the stored key will survive a save with this base URL. The device drops a stored key
 * when the base URL moves to another host and no new key is typed - a key is never carried to a
 * server it was not entered for - so the page must say so before the person presses Save.
 */
export function keyWillBeDropped(saved, draftBase, draftKey) {
  if (!saved?.key_set || draftKey) return false;
  return originOf(draftBase) !== originOf(saved.base_url);
}

/** Whether a session can start with these values: api.openai.com always needs a key; a local server may not. */
export function canConnect(draftBase, model, hasKey) {
  const ep = deriveEndpoints(draftBase, model);
  if (!ep || !validModel(model)) return false;
  return hasKey || ep.origin !== OPENAI_ORIGIN;
}

/**
 * The page's view of the device's discovery result, for the base URL it shows now and the request
 * it made last (`gen`). A result for another address or an older request is "stale" - never shown,
 * so a slow answer for a URL the person has since edited away from cannot unlock the pickers.
 */
export function discovery(models, base, gen) {
  const mine = !!models && gen != null && models.gen === gen && normalizeBaseUrl(models.base) === normalizeBaseUrl(base);
  if (!mine) return { state: gen == null ? "idle" : "loading", ready: false };
  if (models.st === "loading" || models.st === "idle") return { state: "loading", ready: false };
  if (models.st === "error") return { state: "error", ready: false, error: models.err || "Could not connect" };
  const voices = Array.isArray(models.voices) ? models.voices.filter((v) => Array.isArray(v) && typeof v[0] === "string" && v[0]) : [];
  return {
    state: "ok",
    ready: true,
    hasModelList: !!models.list && Array.isArray(models.ids) && models.ids.length > 0,
    ids: Array.isArray(models.ids) ? models.ids.filter((x) => typeof x === "string" && x) : [],
    voices,
    serverVoices: voices.filter((v) => v[2]).length,
  };
}

/**
 * Picker options as [value, label]. The current value is kept (marked) when the server does not
 * offer it, so opening the page never silently changes a saved choice.
 */
export function modelChoices(ids, current, notOffered = "not offered by this server") {
  const list = (ids || []).map((id) => [id, id]);
  if (current && !(ids || []).includes(current)) list.unshift([current, `${current} (${notOffered})`]);
  return list;
}

export function voiceChoices(voices, current, notOffered = "not offered by this server") {
  const list = (voices || []).map(([id, name]) => [id, name ? `${name} (${id})` : id]);
  if (current && !list.some(([id]) => id === current)) list.unshift([current, `${current} (${notOffered})`]);
  return list;
}

/**
 * After a successful discovery for a *different* server than the saved one, a model or voice the
 * new server does not offer is replaced by its first offer - saving a name the server lacks would
 * only fail at the next wake word. For the saved server, the person's choice is kept as is.
 */
export function reconcile(value, offered, sameServer) {
  if (sameServer || !offered.length || offered.includes(value)) return value;
  return offered[0];
}

/**
 * Whether Save is allowed. Changing the connection (address, key, model, voice) needs a successful
 * discovery for the address on screen; turning the feature on or off alone does not - so the
 * switch still works while the server happens to be down.
 */
export function canSave({ saved, draft, disc }) {
  if (!saved) return false;
  const base = normalizeBaseUrl(draft.base);
  const connChanged = base !== saved.base_url || draft.model !== saved.model || draft.voice !== saved.voice || !!draft.key || !!draft.clearKey;
  const dirty = connChanged || draft.enabled !== !!saved.enabled;
  if (!dirty) return false;
  if (!connChanged) return true;
  if (!deriveEndpoints(base, draft.model) || !validModel(draft.model) || !validVoice(draft.voice) || !validKey(draft.key || "")) return false;
  return !!disc?.ready;
}

/** The device's 400 field names, in the page's words. */
export const SAVE_ERRORS = {
  base_url: "That base URL is not a valid http(s):// or ws(s):// address.",
  model: "Model names use letters, digits and - _ . : / only.",
  voice: "Voice names use letters, digits and - _ . : / + only.",
  api_key: "That API key contains characters a key cannot have.",
  fields: "The device did not accept the form - reload the page and try again.",
  body: "The device did not accept the form - reload the page and try again.",
};

/** A session's live state, as the page says it. */
export function stateLabel(st, phase) {
  if (st === "connecting") return "Connecting…";
  if (st === "closing") return "Hanging up…";
  if (st === "active") {
    return (
      { listening: "In a conversation - listening", user_speaking: "In a conversation - hearing you", thinking: "In a conversation - thinking", speaking: "In a conversation - speaking" }[phase] ||
      "In a conversation"
    );
  }
  return "Idle";
}

/** The wire format a session used, as the page says it. */
export function protocolLabel(proto) {
  return { ga: "OpenAI Realtime (GA)", beta: "Realtime beta (compatible servers)" }[proto] || "Not connected yet";
}
