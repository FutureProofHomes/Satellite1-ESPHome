/**
 * App shell: top bar, nav pane, device switcher, and the router.
 *
 * Routing is on the hash. ESPHome's httpd has a single wildcard handler per method and no notion of
 * client-side routes, so a path-based router would 404 on reload for every route but "/". The hash
 * never reaches the server, which also means a bookmarked route survives a firmware update.
 */
import { useEffect, useState } from "preact/hooks";

import { HINTS, TEXT } from "./copy.js";
import { deviceIdentity, useDeviceState, useEvents, useHaData, useSelection } from "./lib/device.js";
import { Config } from "./routes/config.jsx";
import { Controls } from "./routes/controls.jsx";
import { Diagnostics } from "./routes/diagnostics.jsx";
import { Presence } from "./routes/presence.jsx";
import { Chevron, Hint } from "./ui.jsx";

/**
 * All four of the canvas's routes.
 *
 * Presence sits second, next to the sensor pills that link into it. It is the only route that keeps
 * polling while it is open, and the only one whose data does not come from entities at all - the radar
 * settings live in the module's own config, behind `satellite1_radar`'s /api/v1 endpoints.
 */
const ROUTES = [
  { id: "controls", label: "Controls", view: Controls },
  { id: "presence", label: "Presence", view: Presence },
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

/**
 * Light or dark, chosen by hand and remembered per browser.
 *
 * One icon that toggles, showing the theme it will switch to rather than the one in use - a moon while
 * the room is light. Both readings of a lone sun-or-moon icon are common enough that neither is obvious,
 * so the accessible name states the action outright instead of naming a state.
 *
 * The attribute is already on the root element by the time this mounts - index.html sets it from
 * localStorage in <head> so there is no flash - so this reads the element rather than storage, which
 * keeps the two from disagreeing if the write below ever fails.
 *
 * Guarded for the same reason the collapse state in ui.jsx is: localStorage throws rather than no-ops
 * with site data blocked, and forgetting a preference is survivable where a white screen is not.
 */
function ThemeSwitch() {
  const [theme, setTheme] = useState(() => (document.documentElement.dataset.theme === "dark" ? "dark" : "light"));
  const dark = theme === "dark";
  const label = dark ? TEXT.theme_to_light : TEXT.theme_to_dark;

  const toggle = () => {
    const next = dark ? "light" : "dark";
    document.documentElement.dataset.theme = next;
    setTheme(next);
    try {
      localStorage.setItem("sat1.theme", next);
    } catch {
      /* Not remembering it is survivable; the page is already in the right theme. */
    }
  };

  return (
    <button class="icon theme" aria-label={label} title={label} onClick={toggle}>
      {dark ? (
        <svg class="theme-i" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round">
          <circle cx="8" cy="8" r="3.1" />
          <path d="M8 1v1.5M8 13.5V15M1 8h1.5M13.5 8H15M3.1 3.1l1.1 1.1M11.8 11.8l1.1 1.1M12.9 3.1l-1.1 1.1M4.2 11.8l-1.1 1.1" />
        </svg>
      ) : (
        <svg class="theme-i" viewBox="0 0 16 16" fill="currentColor">
          <path d="M13.9 10.4A6.1 6.1 0 0 1 5.6 2.1 6.5 6.5 0 1 0 13.9 10.4Z" />
        </svg>
      )}
    </button>
  );
}

/**
 * The route menu, a drawer that slides in under the top bar.
 *
 * Always mounted and moved with a transform, rather than added and removed. An element unmounted the moment
 * it closes cannot animate on the way out - it is gone from the DOM before the transition can run - so the
 * usual fix is either to keep it and translate it, or to hold the unmount behind a timer. This is the
 * cheaper of the two and has no state to get out of step.
 *
 * Hidden with visibility as well as opacity, so its buttons leave the tab order while it is shut. A menu
 * that cannot be seen but can still be tabbed into is a worse bug than no animation.
 *
 * It no longer carries a heading. It used to repeat the device name that is already in the bar directly
 * above it, which is now only inches away since the drawer starts below the bar rather than over it.
 */
function NavPane({ route, go, open, onClose }) {
  return (
    <div class={`scrim navscrim${open ? " on" : ""}`} onClick={onClose} aria-hidden={!open}>
      <nav class="navpane" onClick={(e) => e.stopPropagation()}>
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
function SwitcherSheet({ device, label, area, onClose }) {
  const haOn = !!device?.ha;
  const haText = haOn ? TEXT.ha_connected : TEXT.ha_disconnected;
  return (
    <div class="scrim" onClick={onClose}>
      <div class="sheet" onClick={(e) => e.stopPropagation()}>
        <div class="sheet-head">
          <span>Satellite1 Device Switcher</span>
          <Hint text={HINTS.switcher} />
          <button class="x" aria-label="Close" onClick={onClose}>
            &#10005;
          </button>
        </div>
        <div class="peer here">
          <div class="row">
            {/* Home Assistant, which is the meaning this dot inherited when it left the top bar. Not the
                device's own reachability: this row is the device serving the page, so that is always true
                and would be a green light that could never go out. Green when connected and neutral when
                not, so the colour is the signal rather than its presence. */}
            <span class={`dot${haOn ? " ok" : ""}`} title={haText} aria-label={haText} />
            <span class="grow">{label || "This device"}</span>
          </div>
          {/* Address and room, which are the two things that tell one Satellite1 from another once the
              sheet lists more than this one. The separator is dropped rather than left dangling when Home
              Assistant has not placed the device in an area. */}
          <div class="peer-sub">{[device?.ip, area].filter(Boolean).join(" \u2502 ")}</div>
        </div>
        <p class="sheet-foot">{TEXT.no_devices}</p>
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
  // Same reasoning, and one read per tab: the selection is small but it is what both trees on Config
  // are editing, so re-fetching it on every route change would be a chance for a stale copy to
  // overwrite an edit the customer just made.
  const selection = useSelection();

  const active = ROUTES.find((r) => r.id === route) || ROUTES[0];
  const View = active.view;
  const ctx = { device, deviceError, ...events, ...ha, ...selection };
  // Resolved once here rather than in the three places that show it, so the bar, the nav pane and the
  // switcher sheet cannot end up disagreeing about what this device is called.
  const { name: label, area } = deviceIdentity(device, ha.ha);

  return (
    <div class="app">
      <header class="topbar">
        <button class="icon" aria-label="Menu" onClick={() => setNav(true)}>
          <span class="burger" />
        </button>
        {/* The caret has to sit against the name for the two to read as one control. It used to be a
            .grow span here, which pushed the caret to the far edge of the bar next to the status dot,
            where it looked like a stray mark rather than "there is a menu on this". */}
        <button class="title" onClick={() => setSwitcher(true)}>
          {/* A placeholder for the one paint before /api/sat1/state answers, not a prefix - the name is
              whatever deviceLabel returns, on its own, so nothing here can double it up. */}
          <span class="tname">{label || "Satellite1"}</span>
          {/* Always down: this opens a sheet, and a dropdown that points sideways reads as a link. */}
          <Chevron down cls="caret" />
        </button>
        {/* The Home Assistant dot used to sit here. It has moved into the switcher sheet, onto the row for
            the device it describes, which is where it can say the same thing about a peer later without
            the bar growing a dot per device. Diagnostics still carries the sentence explaining what a
            missing connection costs. */}
        <ThemeSwitch />
      </header>

      {!events.connected && <div class="banner warn">{TEXT.stream_lost}</div>}

      <main class="wrap">
        <View ctx={ctx} />
      </main>

      {/* Mounted whether or not it is open - see NavPane. */}
      <NavPane route={route} go={go} open={nav} onClose={() => setNav(false)} />
      {switcher && (
        <SwitcherSheet device={device} label={label} area={area} onClose={() => setSwitcher(false)} />
      )}
    </div>
  );
}
