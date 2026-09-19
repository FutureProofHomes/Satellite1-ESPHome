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
 * The token and address live in localStorage, never on the device: the whole point of this tier is
 * that it costs the firmware zero bytes, and a token the device held would be one it had to store,
 * serve and protect. Everything here is additive - the footer works without it, and a socket that
 * drops falls back to the Home Assistant relay it was drawn over.
 *
 * Identity is the MAC again: the MA player_id of a Sendspin player is its client id, which the
 * ESPHome hub sets to this device's MAC - the same fact the Jinja discovery in web_ui_ha.yaml
 * leans on, compared the same way (case folded, separators stripped) because every layer formats a
 * MAC differently.
 */
import { useEffect, useRef, useState } from "preact/hooks";

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

/** "http://ma.local:8095", "ma.local:8095" or "wss://…" into the /ws endpoint's address. */
export function maWsUrl(raw) {
  let u = String(raw || "").trim().replace(/\/+$/, "");
  if (!u) return "";
  if (/^https?:/i.test(u)) u = u.replace(/^http/i, "ws");
  else if (!/^wss?:/i.test(u)) u = `ws://${u}`;
  return `${u}/ws`;
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
