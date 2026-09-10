/**
 * App shell: top bar, nav pane, device switcher, and the router.
 *
 * Routing is on the hash. ESPHome's httpd has a single wildcard handler per method and no notion of
 * client-side routes, so a path-based router would 404 on reload for every route but "/". The hash
 * never reaches the server, which also means a bookmarked route survives a firmware update.
 */
import { useEffect, useState } from "preact/hooks";

import { TEXT } from "./copy.js";
import { useDeviceState, useEvents, useHaData } from "./lib/device.js";
import { Config } from "./routes/config.jsx";
import { Controls } from "./routes/controls.jsx";
import { Diagnostics } from "./routes/diagnostics.jsx";

/**
 * Three of the canvas's four. Presence arrives with the tuner integration; a nav entry that opens an
 * empty page is worse than one that is not there yet.
 */
const ROUTES = [
  { id: "controls", label: "Controls", view: Controls },
  { id: "config", label: "Config", view: Config },
  { id: "diagnostics", label: "Diagnostics", view: Diagnostics },
];

function useHashRoute() {
  const read = () => (location.hash.replace(/^#\/?/, "").split("?")[0] || "controls").toLowerCase();
  const [route, setRoute] = useState(read);

  useEffect(() => {
    const on = () => setRoute(read());
    addEventListener("hashchange", on);
    return () => removeEventListener("hashchange", on);
  }, []);

  return [ROUTES.some((r) => r.id === route) ? route : "controls", (id) => (location.hash = `#/${id}`)];
}

/* ------------------------------------------------------------------ */
/* Overlays                                                            */
/* ------------------------------------------------------------------ */

function NavPane({ route, go, name, onClose }) {
  return (
    <div class="scrim" onClick={onClose}>
      <nav class="navpane" onClick={(e) => e.stopPropagation()}>
        <div class="navpane-head">{name}</div>
        {ROUTES.map((r) => (
          <button
            key={r.id}
            class={`navpane-item${route === r.id ? " on" : ""}`}
            onClick={() => {
              go(r.id);
              onClose();
            }}
          >
            {r.label}
          </button>
        ))}
      </nav>
    </div>
  );
}

/**
 * The device switcher, showing only this device.
 *
 * The canvas mocks a list of peers, which needs discovery this firmware does not do: mDNS browsing
 * from the browser is not a thing, and having every Satellite1 poll for its neighbours is a cost
 * paid on every device to populate a menu. So the sheet is honest about being a list of one, and
 * says what would make others appear.
 */
function SwitcherSheet({ device, routeLabel, onClose }) {
  return (
    <div class="scrim" onClick={onClose}>
      <div class="sheet" onClick={(e) => e.stopPropagation()}>
        <div class="sheet-head">
          <span>Your Satellite1s</span>
          <button class="x" aria-label="Close" onClick={onClose}>
            &#10005;
          </button>
        </div>
        <div class="peer here">
          <div class="row">
            <span class="dot accent" />
            <span class="grow">{device?.friendly_name || device?.name || "This device"}</span>
            <span class="dim xs">you are here</span>
          </div>
          <div class="peer-sub">{device?.ip || ""}</div>
        </div>
        <p class="sheet-foot">{TEXT.no_devices}</p>
        <p class="sheet-foot">You will stay on {routeLabel} when you switch.</p>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Shell                                                               */
/* ------------------------------------------------------------------ */

export function App() {
  const [route, go] = useHashRoute();
  const [nav, setNav] = useState(false);
  const [switcher, setSwitcher] = useState(false);

  // Diagnostics wants fresh heap and loop figures; everywhere else the only thing that goes stale is
  // the Home Assistant dot, which is worth one request every ten seconds so that it starts telling
  // the truth again on its own after the connection comes back.
  const { device, deviceError } = useDeviceState(route === "diagnostics" ? 2000 : 10000);
  const events = useEvents();
  // Read once for the tab rather than per route, so switching to Config and back does not re-ask the
  // device - and therefore does not re-ask Home Assistant - for a list that changes hourly at most.
  const ha = useHaData();

  const active = ROUTES.find((r) => r.id === route) || ROUTES[0];
  const View = active.view;
  const ctx = { device, deviceError, ...events, ...ha };

  return (
    <div class="app">
      <header class="topbar">
        <button class="icon" aria-label="Menu" onClick={() => setNav(true)}>
          <span class="burger" />
        </button>
        <button class="title" onClick={() => setSwitcher(true)}>
          <span class="grow">{device?.friendly_name || device?.name || "Satellite1"}</span>
          <span class="caret">&#9662;</span>
        </button>
        {/* Home Assistant, not the device stream. Whether the device itself is reachable is already
            obvious from the page working at all, whereas a missing smart home is the thing that
            explains why Media is not here and why the assistant does not answer. */}
        <span
          class={`dot${device?.ha ? " ok" : ""}`}
          title={device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected}
          aria-label={device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected}
        />
      </header>

      {!events.connected && <div class="banner warn">{TEXT.stream_lost}</div>}

      <main class="wrap">
        <View ctx={ctx} />
      </main>

      {nav && <NavPane route={route} go={go} name={device?.friendly_name || device?.name || "Satellite1"} onClose={() => setNav(false)} />}
      {switcher && <SwitcherSheet device={device} routeLabel={active.label} onClose={() => setSwitcher(false)} />}
    </div>
  );
}
