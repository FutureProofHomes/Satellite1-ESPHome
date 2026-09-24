/**
 * The device layer: one SSE stream, one serialised request queue, and the key -> entity-id table.
 *
 * Everything here is shaped by two limits of the server it talks to. esp_http_server is configured
 * with max_open_sockets = 7 and lru_purge_enable, so a fourth browser tab evicts the oldest SSE
 * connection rather than failing - which is why there is exactly one EventSource per tab and why
 * every request in this file is single-flight instead of fired in parallel. And web_server
 * addresses entities by display name, so nothing in the UI may contain one: ids come from the table
 * the device serves.
 */
import { useEffect, useReducer, useRef, useState } from "preact/hooks";

/* ------------------------------------------------------------------ */
/* The remote-control target                                           */
/* ------------------------------------------------------------------ */

/**
 * Which device this app is talking to: its own (null) or a peer being remote-controlled - the
 * single-origin device switch that keeps the iOS home-screen app inside its install origin, where
 * a navigation to the peer would push it into Safari's in-app sheet.
 *
 * Module scope rather than context, because request() below is module scope and every hook in this
 * file rides it. The shell owns the value's lifecycle: it calls setRemoteTarget and then remounts
 * the whole app keyed on it, so no hook ever holds one device's state while the URLs point at
 * another - entity ids collide across devices by construction ("switch/Wake Chime" is every
 * device's id), and a reducer left standing across a switch would blend two devices.
 *
 * The key rides the query string on every remote request because it has to ride it on one of them:
 * EventSource can set no headers, and Safari refuses cross-site cookies outright. Same secret the
 * peer's own cookie carries, held in memory only - a reload lands back on the local device.
 */
let remoteTarget = null; // { base, key } | null

/**
 * Where this document is mounted: "" served from the device root, "/api/ingress/<name>" when a
 * Home Assistant ingress proxy (hass_ingress) serves it under HA's own origin. Computed once at
 * module load and never again - the router is hash-based, so the pathname cannot change under us.
 * Every local URL the app builds rides this prefix; at "" the strings are byte-identical to what
 * they always were, which is the whole no-regression argument for direct access.
 *
 * The typeof guard is for the node test runner, which imports this module (through auth.js) with
 * no `location` in scope - under node the prefix is simply "", the direct-access shape.
 */
export const BASE = (typeof location === "undefined" ? "/" : location.pathname).replace(/\/+$/, "");

/**
 * True when the page is served through an ingress proxy rather than by the device itself. The
 * origin games (the IP<->.local redirect, the dual-origin cookie priming) and the peer switcher's
 * in-place jumps are all wrong in that context - the page's origin is Home Assistant's, possibly
 * https, where a navigation or fetch to a plain-http device origin is blocked as mixed content -
 * so their owners check here and stand down.
 */
export const proxied = BASE !== "";

export function setRemoteTarget(target) {
  remoteTarget = target ? { base: String(target.base).replace(/\/+$/, ""), key: target.key } : null;
  // The once-per-load HA sync guard is per-device state in disguise: a fresh target deserves the
  // same one fresh sync a page load gets.
  haAskedThisLoad = false;
}

export const isRemote = () => remoteTarget != null;

/** The peer regenerated its sessions with us among the callers ("sign out everywhere" on its
 *  Diagnostics) and handed back the fresh key; adopting it is what keeps this session being the
 *  one caller that survives, same as the cookie path does for a local session. */
export function updateRemoteKey(key) {
  if (remoteTarget && key) remoteTarget = { ...remoteTarget, key };
}

/** A path like "/api/sat1/state" as this app should actually fetch it: the ingress prefix (empty
 *  when served directly) for the local device, prefixed and key-carrying for a remote one. */
export function apiUrl(path) {
  if (!remoteTarget) return BASE + path;
  return `${remoteTarget.base}${path}${path.includes("?") ? "&" : "?"}key=${remoteTarget.key}`;
}

/* ------------------------------------------------------------------ */
/* The request queue                                                   */
/* ------------------------------------------------------------------ */

/** Tail of the single-flight chain. Private to request(), which is the only way to join it. */
let chain = Promise.resolve();

/**
 * Failed writes, announced to whoever is listening - in practice the shell's toast, but this file
 * cannot import from there without a cycle, so it is a plain subscription.
 *
 * Writes only, and that is the whole design. Every optimistic control in the app already handles its
 * own failure by putting the old value back, which is correct and completely silent - a checkbox
 * that quietly un-ticks itself looks like a page that ignores clicks. Reads stay out of it: the
 * stream-lost banner covers a device that has gone away, the degraded-mode copy covers Home
 * Assistant, and the radar probe *expects* a 404 from the module that is not fitted - a toast on
 * any of those would cry wolf on every mount or poll.
 */
const writeErrorListeners = new Set();
export function onWriteError(fn) {
  writeErrorListeners.add(fn);
  return () => writeErrorListeners.delete(fn);
}
function reportWriteError(path, why) {
  for (const fn of writeErrorListeners) fn({ path, why });
}

/**
 * Warning and error log lines, announced as they arrive on the stream - the toast wiring in the
 * shell is the listener, and the same no-cycle reasoning as onWriteError puts the registry here.
 * The stream is live-only, so these fire within milliseconds of the device emitting the line and
 * never for history: a warning from before the page loaded is in the ring, not on this channel.
 */
const logAlertListeners = new Set();
export function onLogAlert(fn) {
  logAlertListeners.add(fn);
  return () => logAlertListeners.delete(fn);
}

/** The component tag out of "[W][wifi:123]: ...": the coalescing key and the toast's subject. */
const LOG_TAG = /^\[(?:VV|V|D|I|W|E|C)\]\[([^\]:]+)/;

/**
 * Ask Home Assistant for a fresh area/player/pipeline list, once per page load, no matter which
 * route asks first or how many do.
 *
 * Module scope, not component state, and deliberately not reset on unmount: GET /api/sat1/ha returns
 * the device's cached copy, and the device only re-asks Home Assistant 5s after the native API
 * connects or when something POSTs /api/sat1/ha/refresh - there is no interval. One sync per load is
 * the contract, and reloading the page is the gesture that means "look again". This lived as a
 * module global in the Config route until the wake words card moved to its own route and both needed
 * the same guard.
 */
let haAskedThisLoad = false;
export function haSyncOnce(haRefresh) {
  if (haAskedThisLoad) return;
  haAskedThisLoad = true;
  haRefresh();
}

/**
 * Every request this app makes, one at a time.
 *
 * Reads are queued as well as writes, because each request the browser has in flight is a separate
 * TCP connection and every connection costs the device about 2.1KB of internal RAM for its lwIP
 * receive mailbox. The Presence route alone was good for three or four at once - the 4Hz radar
 * poll, the state poll, the voice poll and SSE.
 *
 * The body is read inside the slot, not after it. `fetch` settles when the headers arrive while the
 * body is still coming down the socket, so returning the Response would release the queue with the
 * connection still busy and let the next request open a second one.
 *
 * What it does not serialise is time: callers waiting on Home Assistant sleep between requests
 * rather than inside one, so a resync never holds a slider's writes behind a slow round trip.
 * And every request carries a deadline (below), because a serialised queue is only as alive as its
 * slowest member.
 */
export function request(path, init) {
  // `quiet` is this file's, not fetch's: a write whose failure is already surfaced by its own
  // banner opts out of the toast here, so one failure cannot show up twice in different words.
  const { quiet, ...opts } = init || {};
  const run = async () => {
    try {
      // The queue's own deadline. A fetch that never settles - a connection half-opened against a
      // rebooting device is enough - would otherwise hold the chain for the life of the tab, with
      // every later read queued behind it forever (found live: Presence stuck on "Looking for a
      // radar module" and the media bar absent, while the same endpoints answered a bare fetch
      // instantly). Twenty seconds is several times anything the device legitimately takes; a
      // caller's own signal in `init` still wins the spread.
      const r = await fetch(apiUrl(path), { signal: AbortSignal.timeout(20000), ...opts });
      const out = { ok: r.ok, status: r.status, text: await r.text() };
      if (!out.ok && opts.method === "POST" && !quiet) reportWriteError(path, `HTTP ${out.status}`);
      return out;
    } catch (e) {
      if (opts.method === "POST" && !quiet) reportWriteError(path, String(e?.message || e));
      throw e;
    }
  };
  // Queued on both settlements, so one failed request does not wedge the queue for the session, and
  // advanced by a swallowing continuation rather than by `next` - a caller that handles its own
  // rejection should not also have to keep the queue alive.
  const next = chain.then(run, run);
  chain = next.then(
    () => undefined,
    () => undefined,
  );
  return next;
}

/** The parsed body, or null for a failed request or one that did not answer with JSON. */
export async function requestJson(path, init) {
  const r = await request(path, init);
  if (!r.ok) return null;
  try {
    return JSON.parse(r.text);
  } catch {
    return null;
  }
}

export function post(path) {
  return request(path, { method: "POST" });
}

/**
 * "<domain>/<name>" or "<domain>/<device>/<name>" into a REST path. Every segment after the domain
 * is encoded, because entity names contain spaces and parentheses as a matter of course -
 * "Button Right (Action)" and "Radar Factory Reset" are real ids on a stock device.
 */
export function entityPath(id, action, query) {
  const parts = id.split("/");
  const path = [parts[0], ...parts.slice(1).map(encodeURIComponent)].join("/");
  const q = query ? `?${new URLSearchParams(query)}` : "";
  return `/${path}${action ? `/${action}` : ""}${q}`;
}

/* ------------------------------------------------------------------ */
/* Static device facts and the entity table                            */
/* ------------------------------------------------------------------ */

/**
 * GET /api/sat1/state. Carries the facts that deliberately have no entity behind them - heap,
 * PSRAM, loop time, reset reason, MAC, IP - plus `e`, the key -> entity-id table.
 *
 * Polled while something is watching, once otherwise. Diagnostics passes an interval; every other
 * route reads it once for the header and the entity table.
 */
export function useDeviceState(intervalMs) {
  const [state, setState] = useState(null);
  const [error, setError] = useState(null);

  useEffect(() => {
    let live = true;
    let timer = null;

    const tick = async () => {
      try {
        const r = await request("/api/sat1/state");
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const json = JSON.parse(r.text);
        if (!live) return;
        setState(json);
        setError(null);
      } catch (e) {
        if (live) setError(String(e.message || e));
      }
      if (live && intervalMs) timer = setTimeout(tick, intervalMs);
    };

    tick();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
  }, [intervalMs]);

  return { device: state, deviceError: error };
}

/* ------------------------------------------------------------------ */
/* The Home Assistant data layer                                       */
/* ------------------------------------------------------------------ */

/** Nothing has ever arrived, so the age is not a duration. Matches handle_ha_ on the device. */
export const HA_NEVER = -1;

/** Older than this and opening a card that needs it asks again. */
const HA_STALE_S = 60;

/**
 * What to call this device and which room it is in, preferring what its owner set in Home Assistant.
 *
 * The firmware cannot know the name. `App.get_friendly_name()` is the `friendly_name` substitution with
 * the MAC suffix that `name_add_mac_suffix: true` appends, so it reads "Satellite1 c5ac00" however the
 * device is labelled in the app people actually use. Renaming it in Home Assistant writes `name_by_user`
 * in the device registry and never reaches the firmware.
 *
 * So it arrives through the payload web_ui_ha.yaml was already sending: every FutureProofHomes device in
 * every area, as [model, name, area, mac, version]. Matching on the MAC and not on the name is the whole
 * point, since the name is the thing that differs - and it is the same way the device finds itself in
 * `self_area_jinja`. Compared lowercased because the two sides format it differently: the device prints it
 * uppercase through get_mac_address_pretty_into_buffer and Home Assistant stores its `connections`
 * lowercase, so a plain equality test is a bug that only appears on hardware.
 *
 * Reading the area off the matched row rather than off the payload's own `area` field, because that field
 * is this device's area specifically, and this wants to keep working unchanged when the sheet grows a list
 * of peers with a row each. Falls back to it, since the two agree for this device.
 *
 * Both fall back to what the firmware knows, which is what shows for the paint before the payload lands
 * and for as long as Home Assistant is not connected. The name is never concatenated with the firmware's:
 * one or the other is returned, so a device called "Satellite1 Ceiling Mount" cannot come out doubled.
 *
 * A device Home Assistant has in no area at all is not in `dev` - it is built by walking areas - so it
 * keeps the firmware's name. That is the same blind spot `self_area_jinja` has and the reason `ha_no_area`
 * exists; worth knowing, not worth a second lookup.
 */
export function deviceIdentity(device, ha) {
  const own = device?.friendly_name || device?.name || "";
  const mac = device?.mac?.toLowerCase();
  const hit = mac ? (ha?.d?.dev || []).find((d) => (d?.[3] || "").toLowerCase() === mac) : null;
  // `named`: the name is Home Assistant's own for this device, not the firmware fallback. What lets
  // the blocked card say "tap the cog next to Satellite1 Loft Refurb" with confidence when a stale
  // payload holds the real name, and hedge with "unless you renamed it" only when first onboarding
  // leaves it guessing (owner hit the unhedged firmware name on a renamed fleet, September 2026).
  return { name: hit?.[1] || own, area: hit?.[2] || ha?.d?.area || "", named: !!hit };
}

/**
 * A roster row's reachable origin, for the switcher's jump and the tuner's peer-mute.
 *
 * The row carries two addresses: configuration_url (d[5], the host Home Assistant connects on) and
 * the peer's current IP (d[11], parsed off its Network Status sensor; empty on older peer
 * firmware). configuration_url is preferred exactly as long as its host is an IP literal - which
 * it normally is. But a peer added to Home Assistant by its .local hostname carries a .local URL,
 * and on a network that does not resolve mDNS every request against it dies; when the row also
 * carries the IP, the IP replaces the hostname (same scheme, same port). A row with an IP but no
 * URL at all still gets an origin, on this page's port - peers run the same firmware.
 */
const IP_HOST_RE = /^\d+\.\d+\.\d+\.\d+$/;
export function peerOrigin(d) {
  const url = d?.[5] ? String(d[5]).replace(/\/+$/, "") : "";
  const ip = d?.[11] && IP_HOST_RE.test(String(d[11])) ? String(d[11]) : "";
  if (!url) return ip ? `http://${ip}${location.port ? `:${location.port}` : ""}` : "";
  if (!ip) return url;
  try {
    const u = new URL(url);
    if (IP_HOST_RE.test(u.hostname) || u.hostname.includes(":")) return url;
    u.hostname = ip;
    return u.toString().replace(/\/+$/, "");
  } catch {
    return url;
  }
}

/**
 * GET /api/sat1/ha: the area, player and device tree Home Assistant rendered.
 *
 * The browser cannot ask Home Assistant itself - it has no token, and requiring one to open a
 * settings page is not a setup step this product can have - so the device asks over the native API
 * and caches the answer. What comes back is `{rung, actions, age, d}`: which rung of the
 * responding-action ladder worked, what the device's own probe concluded about the actions
 * checkbox, how many seconds ago, and the payload.
 *
 * `rung` says what the last sync managed. 1 or 2 means the channel works; 0 means nothing has been
 * asked yet, which on a fresh boot is simply "not for another five seconds"; -1 means both rungs
 * were refused.
 *
 * `actions` says why a refused one was refused, which the rung alone cannot: 0 nothing concluded
 * yet, 1 allowed, 2 blocked (the "Allow the device to perform Home Assistant actions" checkbox is
 * off), 3 unverifiable (Home Assistant predates 2025.12 and answers no action call, so a silence
 * must not accuse the checkbox). It is tts_routing's ha_actions_allowed verdict, pushed into the
 * component at every transition - see tts_routing_status_publish.
 *
 * A stale payload is still served with its real age rather than withheld, because a list of speakers
 * from a minute ago is more use than an empty one - so `stale` is advice to the UI, not an error.
 */

/** The checkbox is off. Also inferred from rung -1 alone: the ladder only runs at all when the
 *  connected Home Assistant supports action replies, so a refusal there cannot mean "too old" - it
 *  is the same evidence the probe's watchdog reads, arriving a few seconds sooner. */
export const haBlocked = (ha) => !!ha && (ha.actions === 2 || ha.rung === -1);

/** Home Assistant predates 2025.12. Never true alongside a working payload: a rung that answered
 *  proves the calls work, whatever an earlier probe concluded. */
export const haTooOld = (ha) => !!ha && ha.actions === 3 && ha.rung !== 1 && ha.rung !== 2;

export function useHaData() {
  const [ha, setHa] = useState(null);
  const [refreshing, setRefreshing] = useState(false);

  const read = async () => {
    // The stream-lost banner in the shell already covers a device that has gone away, and the
    // cached payload we may already be holding is still worth showing, so a failure is reported to
    // the caller rather than surfaced.
    const json = await requestJson("/api/sat1/ha").catch(() => null);
    if (!json) return false;
    setHa(json);
    return true;
  };

  useEffect(() => {
    let live = true;
    let timer = null;

    const tick = async () => {
      const json = await requestJson("/api/sat1/ha").catch(() => null);
      if (!live || !json) return;
      setHa(json);
      // Nothing has arrived yet and the device is still inside the 5s it waits after Home
      // Assistant connects. Ask again shortly rather than showing "unavailable" for a sync that
      // has not been attempted.
      if (json.age === HA_NEVER && json.rung === 0) timer = setTimeout(tick, 3000);
    };

    tick();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
  }, []);

  /**
   * Asks the device to sync again. The endpoint only records the request - the action call has to
   * start from the main loop - so this waits for the round trip rather than expecting the POST's
   * response to carry anything.
   */
  const refresh = async () => {
    setRefreshing(true);
    try {
      // Safe to queue: the endpoint only records the request and returns, so it holds the slot for
      // one round trip. The waiting below happens between requests, not inside one.
      await post("/api/sat1/ha/refresh");
      // The device waits on Home Assistant, which took about a second on a real installation. Two
      // reads a second apart, so a slow answer still lands without polling for minutes.
      await new Promise((r) => setTimeout(r, 1200));
      if (!(await read())) return;
      await new Promise((r) => setTimeout(r, 1500));
      await read();
    } finally {
      setRefreshing(false);
    }
  };

  return {
    ha,
    haRefresh: refresh,
    haRefreshing: refreshing,
    // One plain re-read of the device's cache, no refresh POST: what the splash and the fix drawer
    // poll while they are watching for the checkbox verdict to change. The device re-syncs on its
    // own when Home Assistant reconnects (ticking the checkbox reloads the config entry, which is a
    // reconnect), so reading the cache is enough to notice recovery - no action call is spent on it.
    haRead: read,
    haStale: ha ? ha.age === HA_NEVER || ha.age > HA_STALE_S : false,
  };
}

/* ------------------------------------------------------------------ */
/* The selection: which players routing and ducking are aimed at       */
/* ------------------------------------------------------------------ */

/**
 * Reads and writes the selection the device owns at /api/sat1/sel.
 *
 * Not an entity, which is the point of the whole redesign: it used to be a text entity and could not
 * hold a real selection, because ESPHome caps a text entity at 255 characters and one area's worth of
 * players needs roughly twice that.
 *
 * Held as Sets in the app and as comma-separated strings on the device. The conversion happens here so
 * neither the tree nor the route has to think about it.
 */
export function useSelection() {
  const [sel, setSel] = useState(null);
  const [error, setError] = useState(false);

  const parse = (json) => ({
    local: json.local === 1 || json.local === true,
    area: json.area || "",
    routing: {
      areas: new Set(json.routing?.areas || []),
      extra: new Set(json.routing?.extra || []),
      excluded: new Set(json.routing?.excluded || []),
    },
    duck: {
      areas: new Set(json.duck?.areas || []),
      extra: new Set(json.duck?.extra || []),
      excluded: new Set(json.duck?.excluded || []),
    },
  });

  useEffect(() => {
    let live = true;
    requestJson("/api/sat1/sel")
      .then((json) => {
        if (!live) return;
        if (json) setSel(parse(json));
        else setError(true);
      })
      .catch(() => live && setError(true));
    return () => {
      live = false;
    };
  }, []);

  /**
   * Writes the whole selection, not a delta.
   *
   * Optimistic, and deliberately so: ticking a checkbox has to move it now, and the device's own copy
   * is what the next read returns anyway. A rejected write puts the previous selection back, because
   * a checkbox that stays ticked after the device refused it is the worst of the three outcomes.
   *
   * JSON rather than form fields, which is what routes it to handleBody on the device side. The form
   * branch in web_server_idf rejects a body over CONFIG_HTTPD_MAX_REQ_HDR_LEN, 1024 here, and a
   * selection can legitimately exceed that.
   */
  const write = async (next) => {
    const previous = sel;
    setSel(next);
    const body = JSON.stringify({
      local: next.local ? 1 : 0,
      routing: {
        areas: [...next.routing.areas],
        extra: [...next.routing.extra],
        excluded: [...next.routing.excluded],
      },
      duck: {
        areas: [...next.duck.areas],
        extra: [...next.duck.extra],
        excluded: [...next.duck.excluded],
      },
    });
    try {
      const r = await request("/api/sat1/sel", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body,
        // Not quiet any more: this used to opt out of the toast in favour of a sel_failed banner by
        // the trees, which went with the amber banners (owner decision, September 2026). The
        // write-failed toast is now this write's surface like every other write's - one failure,
        // one report - and the revert below stays.
      });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
    } catch {
      // The revert only; selError stays what the *read* concluded. It used to flip here too, which
      // fed a banner that no longer exists - and a route deciding it cannot render because one write
      // bounced would be the wrong lesson to draw anyway.
      setSel(previous);
    }
  };

  return { sel, selError: error, selWrite: write };
}

/* ------------------------------------------------------------------ */
/* Radar: the presence route's live feed and settings                  */
/* ------------------------------------------------------------------ */

/** How often the live feed is polled while the Presence route is mounted. The LD2450 reports at 10 Hz
 *  and the plot is a room, not an oscilloscope; 250ms tracks a walking person smoothly and leaves the
 *  single serialised fetch slot free most of the time. */
export const RADAR_LIVE_MS = 250;

/** How long after the last settings write the automatic flash save fires. Long enough to span a whole
 *  train of slider tweaks, short enough that walking away means it saved. */
const RADAR_SAVE_MS = 10000;

/**
 * Reads the radar tuner's own JSON API, which predates this app and needs no firmware work.
 *
 * `satellite1_radar` already serves `/api/v1/ld2450/config`, `/live`, `/api/v1/save` and
 * `/api/v1/reboot`, and answers 404 with a JSON error for a module that is not fitted. Which module
 * that is, is discovered by asking: the entity map only reports "Radar Detected" as display text, and
 * probing the two config endpoints is the same question asked of the component that actually knows.
 *
 * Engineering mode needs no handling here. The LD2410 requires it for gate energies, and the handler
 * arms it from the live poll itself and lets it lapse after the polling stops - so mounting this route
 * is the whole protocol, and there is no enter/exit pair to leak if a tab closes mid-session.
 */
export function useRadar(enabled) {
  const [kind, setKind] = useState(null); // "ld2450" | "ld2410" | "none" | null while probing
  const [config, setConfig] = useState(null);
  const [live, setLive] = useState(null);
  const [busy, setBusy] = useState(false);

  const read = async (which) => requestJson(`/api/v1/${which}/config`).catch(() => null);

  // Probe once per mount rather than once per tab: a radar swap needs the device opened anyway, and a
  // wrong answer cached for the life of the tab would be untraceable.
  useEffect(() => {
    if (!enabled) return undefined;
    let live_ = true;
    (async () => {
      for (const which of ["ld2450", "ld2410"]) {
        const cfg = await read(which);
        if (!live_) return;
        if (cfg) {
          setKind(which);
          setConfig(cfg);
          return;
        }
      }
      if (live_) setKind("none");
    })();
    return () => {
      live_ = false;
    };
  }, [enabled]);

  useEffect(() => {
    if (!enabled || !kind || kind === "none") return undefined;
    let live_ = true;
    let timer = null;

    const tick = async () => {
      const json = await requestJson(`/api/v1/${kind}/live`).catch(() => null);
      if (!live_) return;
      if (json) setLive(json);
      // Chained rather than an interval, so a slow device stretches the gap instead of queueing
      // requests behind each other.
      timer = setTimeout(tick, RADAR_LIVE_MS);
    };

    tick();
    return () => {
      live_ = false;
      if (timer) clearTimeout(timer);
    };
  }, [enabled, kind]);

  /* Settings persist on their own: /api/v1/save (a preferences flush to NVS) fires ten seconds after
     the last successful write, so a tuning session ends saved without anyone thinking about it. It
     replaced a "Save to flash" button, at the owner's call. Debounced rather than per-write because a
     session is dozens of writes and NVS wear is real, if small; flushed on unmount because "left the
     page eight seconds after the last tweak" must not mean "lost it". */
  const saveTimer = useRef(null);
  const savePending = useRef(false);

  const scheduleSave = () => {
    savePending.current = true;
    if (saveTimer.current) clearTimeout(saveTimer.current);
    saveTimer.current = setTimeout(() => {
      savePending.current = false;
      post("/api/v1/save").catch(() => {});
    }, RADAR_SAVE_MS);
  };

  useEffect(
    () => () => {
      if (saveTimer.current) clearTimeout(saveTimer.current);
      if (savePending.current) post("/api/v1/save").catch(() => {});
    },
    []
  );

  /** Writes one or more config fields. Every field on the device side is optional, so this sends only
   *  what changed and lets the rest stand. Optimistic, like the selection, with the previous value put
   *  back if the device refuses it. */
  const writeConfig = async (patch) => {
    const previous = config;
    setConfig({ ...config, ...patch });
    setBusy(true);
    try {
      const r = await request(`/api/v1/${kind}/config`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(patch),
      });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      // The LD2410 config POST only stores; /apply is what pushes the stored config onto the module's
      // command queue from the main loop. The standalone tuner page always paired the two, and this
      // hook not doing so was a latent bug - every LD2410 write from the app looked accepted and then
      // never reached the radar. The LD2450 path applies inside its own set_backend_config, so it has
      // no such second step.
      if (kind === "ld2410") await post("/api/v1/ld2410/apply");
      // Re-read rather than trusting the patch: `reboot_required` is the device's opinion, not ours,
      // and it is the whole reason the restart row appears.
      const fresh = await read(kind);
      if (fresh) setConfig(fresh);
      scheduleSave();
    } catch {
      setConfig(previous);
    } finally {
      setBusy(false);
    }
  };

  const reboot = () => post("/api/v1/reboot").catch(() => {});

  /** Local-only patch, no POST: what a slider calls while it is still moving, so the plot can follow
   *  the drag in real time. The commit on release goes through writeConfig, whose re-read squares
   *  whatever this previewed against what the device actually accepted. */
  const preview = (patch) => setConfig((c) => (c ? { ...c, ...patch } : c));

  return { radarKind: kind, radarConfig: config, radarLive: live, radarBusy: busy, radarWrite: writeConfig, radarPreview: preview, radarReboot: reboot };
}

/* ------------------------------------------------------------------ */
/* Voice: timers and the assistant's phase                             */
/* ------------------------------------------------------------------ */

/** Matches the `voice_assist_*_phase_id` substitutions in common/voice_assistant.yaml. */
export const PHASE = {
  1: "Idle",
  2: "Waiting for a command",
  3: "Listening",
  4: "Thinking",
  5: "Replying",
  10: "Not ready",
  11: "Error",
};

/**
 * GET /api/sat1/voice, polled only while the home page is on screen.
 *
 * A second while a timer is counting or the assistant is mid-exchange, five seconds otherwise. The
 * fast rate is there so a timer's remaining seconds move; polling that hard when nothing is
 * happening would be a request every second, forever, on a device that also has to do audio.
 */
export function useVoice(enabled) {
  const [voice, setVoice] = useState(null);
  const busy = voice ? voice.running || voice.timers.some((t) => t.active) : false;

  useEffect(() => {
    if (!enabled) return;
    let live = true;
    let timer = null;

    const tick = async () => {
      try {
        const json = await requestJson("/api/sat1/voice");
        if (json && live) setVoice(json);
      } catch {
        // A dropped poll is not worth surfacing: the next one is a second away, and the stream
        // banner already covers the case where the device has actually gone.
      }
      if (live) timer = setTimeout(tick, busy ? 1000 : 5000);
    };

    tick();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
  }, [enabled, busy]);

  return voice;
}

/**
 * GET /api/sat1/media, polled on every route now that the footer lives in the shell, at the voice
 * endpoint's cadence and for its reasons: a second while something is playing so the state tracks
 * the room, five when idle.
 *
 * A poll because there is nothing else: web_server has no media_player handler, so the players ride
 * neither /events nor the entity REST API. The track metadata rides this poll too - title, artist,
 * album and artwork URL from the device's Sendspin hub cache, plus position, duration, shuffle and
 * repeat state and the server's supported-command bitmask, none of which have an entity anywhere.
 *
 * `mediaCmd` posts a command with its query parameters as given - `v` for a volume or a shuffle
 * flag, `m` for a repeat mode, `src` to name the player the footer is showing, so a command lands
 * on it and not on whatever the device resolves as active when its queue drains (load-bearing for
 * a paused group stream, which the device sees as an idle player). Fire-and-forget on purpose: the
 * device queues the command for its main loop, so the POST's answer never carries the state that
 * resulted - the next poll does. A failed post already surfaces through the toast.
 *
 * `mediaPoke` pulls the next poll forward. It exists for the pending rings on the transport
 * controls (media.jsx's usePendingCmds): a play sent from idle rides the 5s cadence, and a ring
 * that spins out most of its deadline waiting for a poll that would have confirmed at once reads
 * as a slow device. One nudged re-poll shortly after the command is queued closes that gap.
 */
/** djb2 in base36: not cryptographic, just enough to tell one artwork URL from the next, so the
 *  relay path below changes exactly when the art does and browser caching handles the rest. */
const artKey = (s) => {
  let h = 5381;
  for (let i = 0; i < s.length; i++) h = ((h * 33) ^ s.charCodeAt(i)) >>> 0;
  return h.toString(36);
};

/** Behind the ingress proxy, a plain-http artwork URL is mixed content on an https Home Assistant
 *  - the browser refuses it before any request is made. The device relays the current track's art
 *  at /api/sat1/media/art (web_ui_handler.cpp), which rides the panel's own origin, so this swap
 *  is what makes covers appear inside HA at all. Direct visits and remote control keep the URL
 *  as-is: there the direct fetch works and costs the device nothing. An https artwork URL is
 *  never swapped - it embeds fine anywhere, relay or no relay. */
const mapArt = (json) =>
  proxied && !isRemote() && json?.art && /^http:\/\//i.test(json.art)
    ? { ...json, art: `${BASE}/api/sat1/media/art?v=${artKey(json.art)}` }
    : json;

export function useMedia(enabled) {
  const [media, setMedia] = useState(null);
  const busy = media ? media.state === 2 || media.state === 3 : false;
  const pokeRef = useRef(null);

  useEffect(() => {
    if (!enabled) return;
    let live = true;
    let timer = null;

    const tick = async () => {
      if (timer) {
        clearTimeout(timer);
        timer = null;
      }
      try {
        const json = await requestJson("/api/sat1/media");
        if (json && live) setMedia(mapArt(json));
      } catch {
        // Same as the voice poll: the stream banner covers a device that has gone.
      }
      if (live) timer = setTimeout(tick, busy ? 1000 : 5000);
    };

    // 250ms, not immediate: the command POST this follows has to reach the device's queue first,
    // or the nudged poll reads back the state the command is about to change.
    pokeRef.current = () => {
      if (!live) return;
      if (timer) clearTimeout(timer);
      timer = setTimeout(tick, 250);
    };

    tick();
    return () => {
      live = false;
      pokeRef.current = null;
      if (timer) clearTimeout(timer);
    };
  }, [enabled, busy]);

  const mediaCmd = (cmd, params = {}) => {
    const q = new URLSearchParams();
    for (const [k, v] of Object.entries(params)) if (v != null) q.set(k, v);
    const qs = q.toString();
    return post(`/api/sat1/media/${cmd}${qs ? `?${qs}` : ""}`);
  };

  const mediaPoke = () => pokeRef.current?.();

  return { media, mediaCmd, mediaPoke };
}

/**
 * GET /api/sat1/ma: the live Music Assistant view Home Assistant relays - who is grouped with this
 * device's player and at what volume, plus shuffle and repeat as Home Assistant sees them.
 *
 * Enabled only while the expanded view or the players panel is open, which is where it renders live
 * - plus a single read on mount, because the bar's group-count badge needs whatever the device
 * already holds even before anything opens. Each enabled cycle asks the device to sync (`/refresh`
 * records the request; the action call runs from its main loop, floored there at one per two
 * seconds however many tabs ask) and reads the payload back after the round trip has had time to
 * land. The first read skips the ask, so the cached payload paints immediately with its honest age.
 *
 * `maCmd` posts one relayed command - like, join, unjoin, vol, seek - and is fire-and-forget for
 * the reason mediaCmd is: the device queues it, the action call captures no response, and the
 * resync it schedules is the only confirmation that exists. `maRead` is the early re-read a group
 * edit schedules so the change shows before the next full cycle. `maAsk` is one refresh-and-read
 * cycle on demand - the question the footer asks when the group stream stops ("paused or ended?")
 * and repeats slowly while a relay-reported pause is what keeps the paused card up.
 *
 * The payload lands with `at`, this browser's clock at the read. The device's `age` says how old
 * the payload was when served; `at` is what lets a consumer keep counting after that, because a
 * copy held in state for ten minutes is ten minutes staler than its `age` claims.
 */
/** How old (seconds) the device's cached Music Assistant payload may be before the badge read
 *  asks for a fresh one. Thirty seconds spans a route change or reload comfortably while staying
 *  well inside how long a playback session lasts - the badge's subject. */
const MA_BADGE_STALE_S = 30;

export function useMaData(enabled) {
  const [ma, setMa] = useState(null);

  const read = async () => {
    const json = await requestJson("/api/sat1/ma").catch(() => null);
    if (json) setMa({ ...json, at: Date.now() });
  };

  /** One refresh-and-read cycle, the shape the standing cycle below uses: ask the device to sync,
   *  give the action call's round trip time to land, read the result back. Independent of
   *  `enabled` on purpose - its callers are exactly the moments nothing has the panel open. */
  const ask = async () => {
    await post("/api/sat1/ma/refresh").catch(() => {});
    await new Promise((r) => setTimeout(r, 1400));
    await read();
  };

  // The badge read. It starts from the payload the device last landed - but "last landed" can be
  // whenever a browser last had the panel open, and the group-count badge painting from a stale
  // payload (or not painting at all, on a device whose cache is empty) read as "not streaming" on
  // a device that audibly was (owner's report, September 2026: the badge only appeared after
  // opening the panel, whose cycle below runs the refresh this read deliberately skipped). So the
  // cached copy still paints first - immediately, with its honest age - and then one refresh cycle
  // runs when that copy is missing or older than MA_BADGE_STALE_S. One, not a poll: the badge only
  // needs to be right about now, and the panel's own cycle owns liveness from the moment it opens.
  // Harmless when the panel is already open at mount - the device floors the relayed sync at one
  // per two seconds however many callers ask.
  useEffect(() => {
    let live = true;
    (async () => {
      const json = await requestJson("/api/sat1/ma").catch(() => null);
      if (!live) return;
      if (json) setMa({ ...json, at: Date.now() });
      // HA_NEVER (-1) means the device has landed nothing since boot - the emptiest cache there
      // is, not the freshest, so it must not pass the <= staleness test below. It did until
      // September 2026, which left a page loaded right after a reboot never asking at all - and
      // once the payload carried the MA player's state, that silence hid a resumable pause.
      if (json && json.age != null && json.age !== HA_NEVER && json.age <= MA_BADGE_STALE_S) return;
      await post("/api/sat1/ma/refresh").catch(() => {});
      await new Promise((r) => setTimeout(r, 1400));
      if (live) await read();
    })();
    return () => {
      live = false;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => {
    if (!enabled) return undefined;
    let live = true;
    let timer = null;

    const cycle = async () => {
      await post("/api/sat1/ma/refresh").catch(() => {});
      // The sync is an action call from the device's main loop; ~1.4s covers the round trip that
      // took about a second on a live installation, without stretching the first paint.
      await new Promise((r) => setTimeout(r, 1400));
      if (!live) return;
      await read();
      if (live) timer = setTimeout(cycle, 4600);
    };

    read();
    cycle();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [enabled]);

  const maCmd = (cmd, params = {}) => {
    const q = new URLSearchParams();
    for (const [k, v] of Object.entries(params)) if (v != null) q.set(k, v);
    const qs = q.toString();
    return post(`/api/sat1/ma/${cmd}${qs ? `?${qs}` : ""}`);
  };

  return { ma, maCmd, maRead: read, maAsk: ask };
}

/* ------------------------------------------------------------------ */
/* Wake words, which are not entities                                  */
/* ------------------------------------------------------------------ */

/** The stop word's pseudo-slot index for the tune and cutoff endpoints - WL_STOP on the device.
 *  Not a real slot: it never appears in `slots`, cannot be swapped, and its on/off is the
 *  stop_word switch entity. Its tuned state rides the payload's `stopw` block. */
export const STOP_SLOT = 2;

/**
 * GET /api/sat1/wakewords: the two wake word slots, the built-in models, the stop word's tuned
 * state (`stopw`), the live room-pressure high-waters (`hw`), the close-call ring (`near`) and the
 * detection ring.
 *
 * Read once on mount, and - when `pollMs` is given - on a gentle standing poll while the caller is
 * mounted. The poll earns its keep now that the payload carries *living* facts (the margin bars'
 * room fill, the detections lane, ripicked close calls), where the old payload only changed when a
 * person changed it; the route's transient fast loops (a swap in flight, a tune session) still run
 * on top through `wakeRead`.
 *
 * Writes are fire-and-queue like every device write: the endpoint records the request and the
 * loader applies it from the main loop, so the poll of this same GET is the only truthful
 * confirmation - `st` moves through downloading to ready or error, and a failed swap leaves the
 * previous word in `m`/`w`, which is exactly what the card should fall back to showing.
 */
export function useWakeSlots(pollMs) {
  const [wake, setWake] = useState(null);

  const read = async () => {
    const d = await requestJson("/api/sat1/wakewords").catch(() => null);
    if (d && Array.isArray(d.slots)) setWake(d);
    return d && Array.isArray(d.slots) ? d : null;
  };

  useEffect(() => {
    let live = true;
    let timer = null;
    const tick = async () => {
      const d = await requestJson("/api/sat1/wakewords").catch(() => null);
      if (!live) return;
      if (d && Array.isArray(d.slots)) setWake(d);
      // Anything else is left as null, which the card reads as "this build has no wake word
      // slots" - a build without the loader answers 404 here and means exactly that.
      if (pollMs) timer = setTimeout(tick, pollMs);
    };
    tick();
    return () => {
      live = false;
      if (timer) clearTimeout(timer);
    };
  }, [pollMs]);

  /** Points slot `i` at `spec`: a built-in id, a manifest URL, or "none". */
  const setSlot = (i, spec) => post(`/api/sat1/wakewords/slot?i=${i}&m=${encodeURIComponent(spec)}`);

  // Cutoff writes live where their one caller is: the tuner's apply() builds the POST itself,
  // because it alone knows the full stat set (`n`/`f`/`h`). A helper here once dropped `h` on the
  // floor for whoever called it next - dead code with a wrong signature is worse than none.

  return { wake, wakeRead: read, setSlot };
}

/* ------------------------------------------------------------------ */
/* Which assistant answers which wake word                             */
/* ------------------------------------------------------------------ */

/**
 * Home Assistant's value for an empty wake word slot, and for "use whichever pipeline is preferred".
 *
 * Internal constants of Home Assistant's, which it translates only for display - which is why the
 * payload finds the four selects by these and not by the names on the device page. `preferred` is
 * exported because it needs a label of its own, and that is the view's business rather than this file's.
 */
export const NO_WAKE_WORD = "no_wake_word";
export const PIPELINE_PREFERRED = "preferred";

/**
 * How many wake-word-and-assistant pairs exist. Two, and not our two: Home Assistant's ESPHome
 * integration creates exactly this many for every voice satellite, in its select platform, without
 * asking the device how many it could use. So this is a fact to be reported, not a limit to be raised.
 */
export const ASSIST_SLOTS = 2;

/**
 * Reads and writes which assistant answers which wake word.
 *
 * This is the only control in the app whose value does not live on the device. Home Assistant pairs each
 * of its two slots with a wake word and a pipeline, and when a wake word fires it walks those pairs
 * looking for the name the device reported - so the mapping is Home Assistant's, and there is no local
 * flag that could hold it. The four selects arrive in the Home Assistant payload as `asst`; writes go
 * out through the device, which turns them into select.select_option calls.
 *
 * `enabled` is the wake words currently listening, by display name, and it is what makes a slot free or
 * taken. It matters because a slot write has a side effect worth knowing about: Home Assistant answers
 * one by pushing the union of both slots back to the device as the complete set of active wake words,
 * disabling anything not in it. Keeping the slots equal to the enabled set is therefore not tidiness -
 * it is what stops Home Assistant from silently switching a wake word off later.
 */
export function useAssist(ha, haRefresh, enabled) {
  // What we have asked for and not yet seen confirmed, keyed by entity id. Home Assistant is two round
  // trips away - the write, then the resync that reads it back - so without this the dropdown would sit
  // on its old value for the better part of three seconds and read as though the choice was refused.
  const [local, setLocal] = useState({});
  const [busy, setBusy] = useState(false);

  const raw = ha?.d?.asst;
  const ready = Array.isArray(raw?.s) && raw.s.length === ASSIST_SLOTS;

  // The customer's own pipelines, by name, exactly as Home Assistant rebuilds them from its pipeline
  // store whenever one is added or renamed. `preferred` is not in here: the payload leaves it out
  // because it needs a label rather than a name, and the card puts it back at the front of the list.
  const pipelines = raw?.o || [];

  const slots = ready
    ? raw.s.map(([we, ws, pe, ps]) => ({ we, word: local[we] ?? ws, pe, pipeline: local[pe] ?? ps }))
    : [];

  /**
   * Sends one select to Home Assistant and waits for the payload to be read back.
   *
   * The optimistic entries are dropped once the resync lands rather than kept until contradicted. If
   * Home Assistant took the option the fresh payload already says so and dropping them changes nothing
   * on screen; if it refused - an option that no longer exists, actions not permitted - the control
   * snaps back, which is the truth and the only signal available. The device cannot tell us: this call
   * captures no response.
   */
  const send = async (writes) => {
    setBusy(true);
    setLocal((prev) => ({ ...prev, ...Object.fromEntries(writes) }));
    try {
      for (const [entity, option] of writes) {
        // The endpoint queues the write and returns; the action call runs from the device's main
        // loop, so this holds the slot for one round trip and no longer.
        await post(`/api/sat1/ha/select?e=${encodeURIComponent(entity)}&o=${encodeURIComponent(option)}`);
      }
      await haRefresh();
    } finally {
      setLocal((prev) => {
        const next = { ...prev };
        for (const [entity] of writes) delete next[entity];
        return next;
      });
      setBusy(false);
    }
  };

  /** Which slot holds `word`, or -1. */
  const slotOf = (word) => slots.findIndex((s) => s.word === word);

  /**
   * A slot `word` could move into: an empty one first, then one held by a wake word that is not
   * listening any more. A slot held by another enabled wake word is not available, because taking it
   * would turn that wake word off.
   */
  const freeSlot = (word) => {
    const at = slots.findIndex((s) => s.word === NO_WAKE_WORD);
    if (at >= 0) return at;
    return slots.findIndex((s) => s.word !== word && !(enabled || []).includes(s.word));
  };

  return {
    ready,
    busy,
    pipelines,

    /** Which assistant answers `word`, or null if it holds no slot and so falls back to the first. */
    pipelineFor: (word) => {
      const at = slotOf(word);
      return at < 0 ? null : slots[at].pipeline;
    },

    /** The assistant an unslotted wake word ends up at, which is whatever the first slot points to. */
    fallbackPipeline: () => (ready ? slots[0].pipeline : null),

    /** Points `word` at `option`, moving it into a slot first if it does not already hold one. */
    setPipeline: async (word, option) => {
      if (!ready) return;
      let at = slotOf(word);
      const writes = [];
      if (at < 0) {
        at = freeSlot(word);
        if (at < 0) return;
        writes.push([slots[at].we, word]);
      }
      writes.push([slots[at].pe, option]);
      await send(writes);
    },

    /**
     * Puts `word` into a slot, or takes it out, so the slots keep matching the wake words that are
     * listening. Called after a wake word is switched on or off, and the reason it has to be: a slot
     * still naming a wake word that is off would have Home Assistant switch it back on, and a wake word
     * in no slot at all gets switched off the next time Home Assistant writes one.
     *
     * Silent when Home Assistant is not there. The device has already stored the change either way, so
     * the wake word does what the switch said; what is lost is only the agreement, and the card says so.
     */
    syncSlot: async (word, on) => {
      if (!ready) return;
      const at = slotOf(word);
      if (on) {
        if (at >= 0) return;
        const free = freeSlot(word);
        if (free < 0) return;
        await send([[slots[free].we, word]]);
      } else {
        if (at < 0) return;
        await send([[slots[at].we, NO_WAKE_WORD]]);
      }
    },
  };
}

/* ------------------------------------------------------------------ */
/* Live state and the log, over one /events stream                     */
/* ------------------------------------------------------------------ */

/** The logger writes ANSI colour runs into every message before the "[D][tag:line]" header. */
const ANSI = /\u001b\[[0-9;]*m/g;
const LEVEL = /^\[(VV|V|D|I|W|E|C)\]/;

export const LOG_RING = 1000;

/**
 * Merge, never replace.
 *
 * web_server serialises an entity twice over, with two different shapes. A new /events session gets
 * every entity at JsonDetail::DETAIL_ALL; every subsequent publish is DETAIL_STATE, which on the
 * wire is as short as {"id":"text_sensor/Radar Target","value":"Approaching","state":"Approaching"}.
 * The fields that only appear in the DETAIL_ALL pass are exactly the ones the controls are built
 * from - a number's min_value/max_value/step, a select's option list, a light's effects - so
 * overwriting the entry would leave every slider without a range and every select without options
 * the moment its value first changed. Live values are safe: light_json_ calls dump_json on both
 * paths, so brightness and colour keep arriving.
 */
function reduceStates(states, msg) {
  const prev = states[msg.id];
  if (!prev) return { ...states, [msg.id]: msg };
  // Skip the render when nothing moved. Several sensors here publish on a timer regardless of
  // whether the value changed, and holding the old object keeps memo'd cards still.
  let changed = false;
  for (const k in msg) {
    if (prev[k] !== msg[k]) {
      changed = true;
      break;
    }
  }
  if (!changed) return states;
  return { ...states, [msg.id]: { ...prev, ...msg } };
}

/**
 * One EventSource for the tab. `state` events hydrate on connect - web_server walks every entity
 * for a new session - so there is no separate initial fetch, and `log` events feed a ring buffer.
 *
 * EventSource cannot set an Authorization header, so this depends on the browser answering the
 * digest challenge on its behalf. That works because check_digest_auth is stateless: the nonce the
 * device issues is never recorded, so nothing goes stale under a stream that stays open for hours.
 */
export function useEvents() {
  const [states, dispatch] = useReducer(reduceStates, {});
  // Starts true, not false: the stream has not failed yet, and a banner that flashes on every cold
  // load while the first connection is still dialing would be the bug this hook exists to fix.
  const [connected, setConnected] = useState(true);
  const logRef = useRef([]);
  const [logSeq, setLogSeq] = useState(0);
  const pausedRef = useRef(false);
  // How many components are currently reading the log - in practice the Logs card, or nobody. The
  // ring fills regardless (a push into a ref costs no render), but the seq bump below only fires
  // while someone is looking. Without this, every log line re-rendered whatever route was open,
  // around the clock: on a debug-level device that is tens of vdom diffs a second bought by a tab
  // sitting on the home page, which is exactly the tab a wall-mounted tablet is.
  const logWatchRef = useRef(0);

  useEffect(() => {
    let es = null;
    let closed = false;
    let graceTimer = null;
    // When the stream last proved itself alive, for the staleness test on foreground below.
    let lastSeen = 0;

    /* iOS kills the SSE socket the moment Safari is backgrounded, and that is routine, not failure.
       So the banner runs on a grace timer: a disconnect only shows after ~4 seconds of being down
       while the tab is visible, and disconnects that happen while hidden are ignored outright -
       the pageshow/visibility handler below reconnects before the grace ever elapses on a healthy
       network, so returning from another app never flashes "lost the connection". */
    const markDown = () => {
      if (document.hidden || graceTimer || closed) return;
      graceTimer = setTimeout(() => {
        graceTimer = null;
        setConnected(false);
      }, 4000);
    };
    const markUp = () => {
      lastSeen = Date.now();
      if (graceTimer) {
        clearTimeout(graceTimer);
        graceTimer = null;
      }
      setConnected(true);
    };

    const onLog = (e) => {
      lastSeen = Date.now();
      const text = e.data.replace(ANSI, "");
      const m = LEVEL.exec(text);
      const lvl = m ? m[1] : "?";
      // `at` is the browser's clock at arrival, because the line itself carries no wall time - the
      // device's logger stamps uptime, not time of day, and the /events payload is just the text.
      // Arrival is honest enough: the stream is live-only, so a line is read within milliseconds of
      // being produced or not at all. Stamped once, here, so the alert payload below and the ring
      // entry carry the same clock - it is the identity a toast's reveal uses to find its line.
      const at = Date.now();
      // The alert channel, deliberately ahead of the pause check below: pausing the Logs card to
      // read something must hold the ring, not silence error toasts app-wide. And deliberately
      // quiet while anyone is reading the log (the same watch count that gates the render): a
      // toast announcing the log to the person looking at it is noise.
      if ((lvl === "E" || lvl === "C" || lvl === "W") && logWatchRef.current === 0) {
        const tag = LOG_TAG.exec(text)?.[1] || "";
        for (const fn of logAlertListeners) fn({ lvl, tag, text, at });
      }
      if (pausedRef.current) return;
      logRef.current.push({ lvl, text, at });
      if (logRef.current.length > LOG_RING) logRef.current.splice(0, logRef.current.length - LOG_RING);
      // The ring is a ref and the counter is the state, so a burst of log lines costs one render
      // rather than one render per line (a noisy boot is several hundred lines in a second) - and
      // no render at all while nothing displays the log. The card catches up on mount: it reads the
      // ring directly, and mounting is itself the render.
      if (logWatchRef.current > 0) setLogSeq((n) => n + 1);
    };

    const connect = () => {
      if (closed) return;
      es?.close();
      // apiUrl carries the remote base and key when a peer is being controlled; EventSource is the
      // reason the key rides query strings at all (it can set no headers), so this line is the one
      // the whole cross-origin auth design is shaped around.
      es = new EventSource(apiUrl("/events"));
      es.onopen = markUp;
      es.onerror = markDown;
      es.addEventListener("state", (e) => {
        lastSeen = Date.now();
        dispatch(JSON.parse(e.data));
      });
      es.addEventListener("ping", markUp);
      es.addEventListener("log", onLog);
    };

    /* The foreground reconnect. EventSource retries on its own, but with a backoff that can leave
       the page dead for many seconds after returning from the background - the fourth screenshot
       this work started from. Recreate immediately instead, unless the stream is demonstrably fine:
       open and heard from within the server's ping interval. The "demonstrably" matters on iOS,
       where a suspended socket can report OPEN while carrying nothing. */
    const onVisible = () => {
      if (document.hidden || closed) return;
      const fresh = es && es.readyState === EventSource.OPEN && Date.now() - lastSeen < 35000;
      if (!fresh) connect();
    };
    addEventListener("visibilitychange", onVisible);
    addEventListener("pageshow", onVisible);

    connect();
    return () => {
      closed = true;
      removeEventListener("visibilitychange", onVisible);
      removeEventListener("pageshow", onVisible);
      if (graceTimer) clearTimeout(graceTimer);
      es?.close();
    };
  }, []);

  // Empties the ring in place rather than swapping the array, because consumers hold the same
  // reference; the sequence bump is what makes anyone re-render.
  const clearLog = () => {
    logRef.current.length = 0;
    setLogSeq((n) => n + 1);
  };

  // Called from an effect by whatever shows the log; returns the cleanup that stops watching. A
  // count rather than a flag, so a second reader some day cannot switch the first one off.
  const logWatch = () => {
    logWatchRef.current += 1;
    return () => {
      logWatchRef.current -= 1;
    };
  };

  return { states, connected, log: logRef.current, logSeq, pausedRef, clearLog, logWatch };
}

/* ------------------------------------------------------------------ */
/* Reading one entity                                                  */
/* ------------------------------------------------------------------ */

/** The /events payload for a logical key, or undefined if the key is not wired up on this build. */
export function entity(ctx, key) {
  const id = ctx.device?.e?.[key];
  return id ? ctx.states[id] : undefined;
}

/** The REST path for a logical key, or null when the entity does not exist on this build. */
export function pathFor(ctx, key, action, query) {
  const id = ctx.device?.e?.[key];
  return id ? entityPath(id, action, query) : null;
}
