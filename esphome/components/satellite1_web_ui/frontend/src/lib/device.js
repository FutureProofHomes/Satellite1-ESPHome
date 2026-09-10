/**
 * The device layer: one SSE stream, one serialised write queue, and the key -> entity-id table.
 *
 * Everything here is shaped by two limits of the server it talks to. esp_http_server is configured
 * with max_open_sockets = 7 and lru_purge_enable, so a fourth browser tab evicts the oldest SSE
 * connection rather than failing - which is why there is exactly one EventSource per tab and why
 * writes are single-flight instead of fired in parallel. And web_server addresses entities by
 * display name, so nothing in the UI may contain one: ids come from the table the device serves.
 */
import { useEffect, useReducer, useRef, useState } from "preact/hooks";

/* ------------------------------------------------------------------ */
/* Writes                                                              */
/* ------------------------------------------------------------------ */

/**
 * Single-flight write chain. Dragging a slider produces a request per input event, and firing
 * those in parallel is how you exhaust seven sockets in one gesture.
 */
let chain = Promise.resolve();

export function post(path) {
  // Queued on both settlements, so one failed write does not wedge the queue for the session.
  const run = () => fetch(path, { method: "POST" });
  chain = chain.then(run, run);
  return chain;
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
        const r = await fetch("/api/sat1/state");
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const json = await r.json();
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
 * GET /api/sat1/ha: the area, player and device tree Home Assistant rendered.
 *
 * The browser cannot ask Home Assistant itself - it has no token, and requiring one to open a
 * settings page is not a setup step this product can have - so the device asks over the native API
 * and caches the answer. What comes back is `{rung, age, d}`: which rung of the responding-action
 * ladder worked, how many seconds ago, and the payload.
 *
 * `rung` is what the degraded copy is written from. 1 or 2 means the channel works; 0 means nothing
 * has been asked yet, which on a fresh boot is simply "not for another five seconds"; -1 means both
 * rungs were refused, which is either an installation below 2025.12 or the actions checkbox off.
 *
 * A stale payload is still served with its real age rather than withheld, because a list of speakers
 * from a minute ago is more use than an empty one - so `stale` is advice to the UI, not an error.
 */
export function useHaData() {
  const [ha, setHa] = useState(null);
  const [refreshing, setRefreshing] = useState(false);

  const read = async () => {
    try {
      const r = await fetch("/api/sat1/ha");
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      setHa(await r.json());
      return true;
    } catch {
      // The stream-lost banner in the shell already covers a device that has gone away, and the
      // cached payload we may already be holding is still worth showing.
      return false;
    }
  };

  useEffect(() => {
    let live = true;
    let timer = null;

    const tick = async () => {
      const r = await fetch("/api/sat1/ha").catch(() => null);
      if (!live) return;
      if (r && r.ok) {
        const json = await r.json();
        if (!live) return;
        setHa(json);
        // Nothing has arrived yet and the device is still inside the 5s it waits after Home
        // Assistant connects. Ask again shortly rather than showing "unavailable" for a sync that
        // has not been attempted.
        if (json.age === HA_NEVER && json.rung === 0) timer = setTimeout(tick, 3000);
      }
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
      // Not queued through post(): that chain serialises entity writes, and a slider mid-drag should
      // not be held up behind a Home Assistant round trip.
      await fetch("/api/sat1/ha/refresh", { method: "POST" });
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
    haStale: ha ? ha.age === HA_NEVER || ha.age > HA_STALE_S : false,
  };
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
 * GET /api/sat1/voice, polled only while Controls is on screen.
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
        const r = await fetch("/api/sat1/voice");
        if (r.ok && live) setVoice(await r.json());
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
  const [connected, setConnected] = useState(false);
  const logRef = useRef([]);
  const [logSeq, setLogSeq] = useState(0);
  const pausedRef = useRef(false);

  useEffect(() => {
    const es = new EventSource("/events");

    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false);

    es.addEventListener("state", (e) => dispatch(JSON.parse(e.data)));
    es.addEventListener("ping", () => setConnected(true));

    es.addEventListener("log", (e) => {
      if (pausedRef.current) return;
      const text = e.data.replace(ANSI, "");
      const m = LEVEL.exec(text);
      logRef.current.push({ lvl: m ? m[1] : "?", text });
      if (logRef.current.length > LOG_RING) logRef.current.splice(0, logRef.current.length - LOG_RING);
      // The ring is a ref and the counter is the state, so a burst of log lines costs one render
      // rather than one render per line. A noisy boot is several hundred lines in a second.
      setLogSeq((n) => n + 1);
    });

    return () => es.close();
  }, []);

  return { states, connected, log: logRef.current, logSeq, pausedRef };
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
