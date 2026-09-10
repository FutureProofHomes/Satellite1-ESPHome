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
