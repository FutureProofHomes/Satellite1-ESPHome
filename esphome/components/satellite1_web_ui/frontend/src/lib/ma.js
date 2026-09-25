/**
 * The Music Assistant tier: this browser talking to the MA server directly, over its WebSocket API.
 *
 * Hand-rolled rather than the official client package, which is written for the MA frontend and
 * would cost more bundle than this whole app. The protocol (verified against the server source,
 * Sept 2026): the server sends its info object the moment the socket opens; the client must then
 * authenticate - `{"message_id","command":"auth","args":{"token"}}` with a long-lived token from
 * MA's profile settings, required since schema 28 - and from there every command is one
 * `{message_id, command, args}` answered by `{message_id, result}` or `{message_id, error_code,
 * details}`, while events arrive unasked as `{event, object_id, data}` for the life of the socket.
 *
 * The token and address live in localStorage, with a copy on the device since September 2026
 * (owner request: one setup per device, not one per browser). The device copy rides
 * /api/sat1/ma/cfg behind the same session gate as everything else - the trust class of the Web
 * UI Password the device already publishes to Home Assistant's roster - and is a convenience,
 * never a requirement: localStorage answers first, the device fills a browser that has nothing,
 * and a token too long for the device's fixed blob simply stays browser-local. Everything here is
 * additive - the footer works without it, and a socket that drops falls back to the Home
 * Assistant relay it was drawn over.
 *
 * Identity is the MAC again: the MA player_id of a Sendspin player is its client id, which the
 * ESPHome hub sets to this device's MAC - the same fact the Jinja discovery in web_ui_ha.yaml
 * leans on, compared the same way (case folded, separators stripped) because every layer formats a
 * MAC differently.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { requestJson } from "./device.js";

const KEY_URL = "sat1.ma.url";
const KEY_TOKEN = "sat1.ma.token";

/** The stored connection settings. Guarded like every localStorage touch in this app: with site
 *  data blocked it throws rather than no-ops, and losing the enhanced tier is survivable. */
export const maSettings = {
  get() {
    try {
      return { url: localStorage.getItem(KEY_URL) || "", token: localStorage.getItem(KEY_TOKEN) || "" };
    } catch {
      return { url: "", token: "" };
    }
  },
  set(url, token) {
    try {
      localStorage.setItem(KEY_URL, url);
      localStorage.setItem(KEY_TOKEN, token);
    } catch {
      /* Not remembered; the session still connects. */
    }
  },
  clear() {
    try {
      localStorage.removeItem(KEY_URL);
      localStorage.removeItem(KEY_TOKEN);
    } catch {
      /* Nothing to forget. */
    }
  },
};

/** The device's stored copy of the connection, or null when it holds none (or the read failed -
 *  the two are the same to the caller: nothing to seed from). */
export async function maCfgFetch() {
  const cfg = await requestJson("/api/sat1/ma/cfg").catch(() => null);
  return cfg && cfg.url && cfg.token ? { url: cfg.url, token: cfg.token } : null;
}

/** Writes the connection to the device - or forgets it there, when both fields are empty.
 *  Fire-and-forget by design: the device copy is a convenience for the *next* browser, and this
 *  one has already saved to localStorage and connected. A refusal (token too long for the
 *  device's fixed blob, old firmware without the endpoint) costs only that convenience. */
export function maCfgSave(url, token) {
  return requestJson("/api/sat1/ma/cfg", {
    method: "POST",
    body: JSON.stringify({ url: String(url || ""), token: String(token || "") }),
  }).catch(() => null);
}

/* ------------------------------------------------------------------ */
/* Finding the server                                                  */
/* ------------------------------------------------------------------ */

/** Music Assistant's signature port: the API, the frontend and /imageproxy all ride it. */
const MA_PORT = 8095;
/** The hostnames worth guessing before any sweep: the add-on and the docker image both default to
 *  names of this shape, and a .local answer is the best one - it survives DHCP. */
const MA_NAMES = ["music-assistant.local", "musicassistant.local", "homeassistant.local", "ma.local"];
/** One no-cors probe's budget. On a LAN, a live host refuses a closed port in milliseconds; the
 *  timeout is for addresses where nothing answers ARP at all, which is most of a /24. */
const SCAN_PROBE_MS = 900;
/** How many probes run at once. High enough to cover a /24 in a handful of waves, low enough that
 *  a phone's browser does not choke on the connection queue. */
const SCAN_LANES = 32;
/** A candidate's WebSocket-hello budget - generous, because the server is real and answering. */
const SCAN_HELLO_MS = 2500;

/**
 * Asks one host:port whether it is a Music Assistant server, by the only question that needs no
 * CORS and no credentials: open its WebSocket and read the hello the server volunteers to every
 * connection (the same message useMaSocket authenticates against below). A real server answers
 * with server_id and schema_version inside a moment; anything else - another service on 8095, a
 * proxy, silence - fails the parse or the clock. Resolves {url, name, version, id} or null.
 */
function maHello(hostport) {
  return new Promise((resolve) => {
    let ws;
    try {
      ws = new WebSocket(`ws://${hostport}/ws`);
    } catch {
      resolve(null);
      return;
    }
    const done = (hit) => {
      try {
        ws.close();
      } catch {
        /* already closed */
      }
      resolve(hit);
    };
    const timer = setTimeout(() => done(null), SCAN_HELLO_MS);
    ws.onmessage = (e) => {
      clearTimeout(timer);
      let msg;
      try {
        msg = JSON.parse(e.data);
      } catch {
        done(null);
        return;
      }
      if (msg.server_id && msg.schema_version != null) {
        done({
          url: `http://${hostport}`,
          name: msg.name || "Music Assistant",
          version: msg.server_version || "",
          id: msg.server_id,
        });
      } else {
        done(null);
      }
    };
    ws.onerror = () => {
      clearTimeout(timer);
      done(null);
    };
  });
}

/** Whether anything is listening on hostport at all: a no-cors fetch resolves (opaquely) for any
 *  HTTP answer and rejects for silence - the cheap filter that keeps the WebSocket phase small. */
async function portOpen(hostport) {
  try {
    await fetch(`http://${hostport}/`, { mode: "no-cors", signal: AbortSignal.timeout(SCAN_PROBE_MS) });
    return true;
  } catch {
    return false;
  }
}

/**
 * Scans the local network for Music Assistant servers: the guessable .local names first, then
 * every address on the device's own /24 probed on the signature port, with each listener asked to
 * prove itself over the WebSocket hello. Browsers cannot browse mDNS, so this is the honest
 * substitute: the page already knows one address on the right subnet - the device's.
 *
 * `seedIp` is that address (falling back to the page's own host when it is a bare IPv4).
 * `onProgress(done, total)` ticks once per finished probe, for the panel's counter. Resolves a
 * list of verified servers, deduped by server_id with a .local presentation winning over a bare
 * IP - the .local form survives the router reassigning addresses.
 *
 * Throws on an https page before probing anything: the browser will not open http fetches or ws://
 * sockets from a secure origin, so the scan cannot work there and should say so rather than
 * report an empty network.
 */
export async function scanForMa(seedIp, onProgress) {
  if (location.protocol === "https:") throw new Error("https");

  const found = new Map(); // server_id -> hit, .local presentation preferred
  const keep = (hit, mdns) => {
    if (!hit) return;
    const prev = found.get(hit.id);
    if (!prev || (mdns && !prev.mdns)) found.set(hit.id, { ...hit, mdns: !!mdns });
  };

  const own = /^(\d+\.\d+\.\d+)\.\d+$/.exec(String(seedIp || "").trim());
  const page = /^(\d+\.\d+\.\d+)\.\d+$/.exec(location.hostname);
  const prefix = (own || page)?.[1];
  const sweep = prefix ? Array.from({ length: 254 }, (_, i) => `${prefix}.${i + 1}`) : [];

  const total = MA_NAMES.length + sweep.length;
  let done = 0;
  const tick = () => onProgress && onProgress(++done, total);

  // The name guesses go straight to the hello - four sockets, no pre-filter needed.
  const names = MA_NAMES.map((h) =>
    maHello(`${h}:${MA_PORT}`).then((hit) => {
      keep(hit, true);
      tick();
    }),
  );

  // The sweep: probe in lanes, and only a listener earns a WebSocket.
  let next = 0;
  const lane = async () => {
    while (next < sweep.length) {
      const ip = sweep[next++];
      const hostport = `${ip}:${MA_PORT}`;
      if (await portOpen(hostport)) keep(await maHello(hostport), false);
      tick();
    }
  };
  await Promise.all([...names, ...Array.from({ length: SCAN_LANES }, lane)]);

  return [...found.values()];
}

/** "http://ma.local:8095", "ma.local:8095" or "wss://…" into the /ws endpoint's address. */
export function maWsUrl(raw) {
  let u = String(raw || "").trim().replace(/\/+$/, "");
  if (!u) return "";
  if (/^https?:/i.test(u)) u = u.replace(/^http/i, "ws");
  else if (!/^wss?:/i.test(u)) u = `ws://${u}`;
  return `${u}/ws`;
}

/** The same address as the HTTP origin MA's /imageproxy rides - maWsUrl's sibling, accepting the
 *  same spellings, so the search drawer's artwork can be fetched from the server the socket talks
 *  to. Plain http when the stored form names no scheme, mirroring maWsUrl's plain-ws default. */
export function maHttpBase(raw) {
  let u = String(raw || "").trim().replace(/\/+$/, "");
  if (!u) return "";
  if (/^wss?:/i.test(u)) u = u.replace(/^ws/i, "http");
  else if (!/^https?:/i.test(u)) u = `http://${u}`;
  return u;
}

const normMac = (s) => String(s || "").toLowerCase().replace(/[:-]/g, "");

/** Whether a player is this device. The player_id of a Sendspin player was the client id - this
 *  device's MAC - until MA 2.11 wrapped every player in its universal_player provider and made the
 *  ids opaque (`up` + hash); since then the MAC lives in device_info.mac_address. Checked both
 *  ways, so either server generation matches. */
const isMe = (p, mac) =>
  normMac(p?.player_id) === normMac(mac) || normMac(p?.device_info?.mac_address) === normMac(mac);

/**
 * One live connection, for as long as `cfg` names a server and the caller stays mounted.
 *
 * Returns `{status, me, players, queue, cmd}`. `status` walks off -> connecting -> on, with "error"
 * for a socket that could not be opened or a token the server refused - the settings panel's whole
 * vocabulary. `players` is every player the server knows, keyed by id and updated live from
 * player_updated events; `me` is this device's, found by MAC; `queue` is its active queue, with
 * elapsed_time re-anchored by every queue_time_updated tick. `cmd` sends one command and resolves
 * with its result - callers mostly ignore the promise, because the event that follows is the truth.
 *
 * Reconnects with a capped backoff while configured. A token the server *refused* does not retry:
 * that answer will not change, and hammering a server with a bad token is how tokens get revoked.
 */
export function useMaSocket(mac, cfg) {
  const [status, setStatus] = useState("off");
  const [players, setPlayers] = useState(null);
  const [queue, setQueue] = useState(null);
  const io = useRef(null);

  const url = cfg ? maWsUrl(cfg.url) : "";
  const token = cfg?.token || "";

  useEffect(() => {
    if (!url || !token) {
      setStatus("off");
      setPlayers(null);
      setQueue(null);
      return undefined;
    }

    let live = true;
    let ws = null;
    let timer = null;
    let tries = 0;
    let denied = false;
    // Pending commands by message id; results resolve them, teardown drops them.
    let nextId = 1;
    const pending = new Map();
    let myId = "";

    const send = (command, args) => {
      const message_id = String(nextId++);
      return new Promise((resolve, reject) => {
        pending.set(message_id, { resolve, reject });
        try {
          ws.send(JSON.stringify({ message_id, command, args }));
        } catch (e) {
          pending.delete(message_id);
          reject(e);
        }
      });
    };

    const refreshQueue = () => {
      if (!myId) return;
      send("player_queues/get_active_queue", { player_id: myId })
        .then((q) => live && setQueue(q ? { ...q, elapsed_time_last_updated: Date.now() / 1000 } : null))
        .catch(() => {});
    };

    const onEvent = (msg) => {
      if (msg.event === "player_updated" && msg.data?.player_id) {
        const p = msg.data;
        setPlayers((prev) => ({ ...prev, [p.player_id]: p }));
        // Our own player changing source or group is what moves the active queue; asking again is
        // one message to a server on the same LAN, and cheaper than modelling MA's redirects here.
        if (p.player_id === myId) refreshQueue();
      } else if (msg.event === "queue_updated" || msg.event === "queue_items_updated") {
        // elapsed_time_last_updated re-anchored to this browser's clock, here and below: the
        // server stamps its own epoch, and extrapolating across two clocks makes the scrubber
        // jump by whatever they disagree by.
        setQueue((prev) =>
          prev && msg.data?.queue_id === prev.queue_id
            ? { ...msg.data, elapsed_time_last_updated: Date.now() / 1000 }
            : prev,
        );
      } else if (msg.event === "queue_time_updated") {
        setQueue((prev) =>
          prev && msg.object_id === prev.queue_id
            ? { ...prev, elapsed_time: msg.data, elapsed_time_last_updated: Date.now() / 1000 }
            : prev,
        );
      }
    };

    const open = () => {
      if (!live) return;
      setStatus("connecting");
      try {
        ws = new WebSocket(url);
      } catch {
        setStatus("error");
        return;
      }

      ws.onmessage = async (e) => {
        let msg;
        try {
          msg = JSON.parse(e.data);
        } catch {
          return;
        }

        // The server's hello, first on every socket. Answering it with auth is the handshake.
        if (msg.server_id && msg.schema_version != null) {
          try {
            await send("auth", { token });
            if (!live) return;
            const all = await send("players/all");
            if (!live) return;
            const map = {};
            for (const p of all || []) map[p.player_id] = p;
            myId = Object.keys(map).find((id) => isMe(map[id], mac)) || "";
            setPlayers(map);
            setStatus("on");
            tries = 0;
            refreshQueue();
          } catch {
            // The one non-retrying failure: the server answered and said no. See the hook comment.
            denied = true;
            setStatus("error");
            ws.close();
          }
          return;
        }

        if (msg.message_id != null && pending.has(msg.message_id)) {
          const p = pending.get(msg.message_id);
          pending.delete(msg.message_id);
          if (msg.error_code != null) p.reject(new Error(msg.details || String(msg.error_code)));
          else p.resolve(msg.result);
          return;
        }

        if (msg.event) onEvent(msg);
      };

      ws.onclose = () => {
        for (const p of pending.values()) p.reject(new Error("closed"));
        pending.clear();
        if (!live || denied) return;
        setStatus((s) => (s === "on" ? "connecting" : "error"));
        // 2s doubling to 30s: fast enough that a restarted MA comes back before anyone reaches for
        // a reload, slow enough that a server that is simply gone is not being shouted at.
        timer = setTimeout(open, Math.min(30000, 2000 * 2 ** tries++));
      };
      ws.onerror = () => {};
    };

    io.current = { send: (c, a) => send(c, a) };
    open();

    return () => {
      live = false;
      if (timer) clearTimeout(timer);
      io.current = null;
      if (ws) ws.close();
    };
  }, [url, token, mac]);

  return {
    status,
    players,
    queue,
    me: players ? Object.values(players).find((p) => isMe(p, mac)) || null : null,
    cmd: (command, args) => (io.current ? io.current.send(command, args) : Promise.reject(new Error("off"))),
  };
}
