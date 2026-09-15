/**
 * App shell: top bar, nav pane, device switcher, and the router.
 *
 * Routing is on the hash. ESPHome's httpd has a single wildcard handler per method and no notion of
 * client-side routes, so a path-based router would 404 on reload for every route but "/". The hash
 * never reaches the server, which also means a bookmarked route survives a firmware update.
 */
import { useEffect, useState } from "preact/hooks";

import { HINTS, TEXT } from "./copy.js";
import { deviceIdentity, onWriteError, useDeviceState, useEvents, useHaData, useSelection } from "./lib/device.js";
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
 * Where a peer row jumps to: the peer's address with the route that is open now on the end, so
 * moving to another device keeps the page. The hash never reaches either server, so this works
 * against old firmware too - it just falls off to whatever that firmware serves at "/".
 */
const peerHref = (base, route) => `${String(base).replace(/\/+$/, "")}/#/${route}`;

/** "192.168.4.31" out of "http://192.168.4.31:80/", for the row's subtitle and for de-duplication. */
const hostOf = (u) => String(u || "").replace(/^https?:\/\//, "").replace(/[/:].*$/, "");

/** A typed address may or may not carry a scheme; a link needs one. */
const withScheme = (a) => (/^https?:\/\//.test(a) ? a : `http://${a}`);

/**
 * The addresses someone typed in by hand - the fallback for everything the payload cannot list:
 * Home Assistant away, or a peer it has in no area. Guarded like the theme, because localStorage
 * throws rather than no-ops with site data blocked, and forgetting the list is survivable.
 */
function readManualPeers() {
  try {
    const v = JSON.parse(localStorage.getItem("sat1.peers") || "[]");
    return Array.isArray(v) ? v.filter((a) => typeof a === "string") : [];
  } catch {
    return [];
  }
}
function writeManualPeers(list) {
  try {
    localStorage.setItem("sat1.peers", JSON.stringify(list));
  } catch {
    /* Not remembering it is survivable; the rows already on screen keep working. */
  }
}

/**
 * The device switcher: this device on top, then every other Satellite1 the Home Assistant payload
 * lists, then whatever was added by address.
 *
 * The roster is the `dev` block that already rides GET /api/sat1/ha - no discovery happens here.
 * mDNS browsing from a browser is not a thing, and probing peers directly is not either: Digest
 * credentials are scoped per origin and a cross-origin probe dies on the preflight. So each row's
 * link is the peer's `configuration_url` - the same address behind Home Assistant's "Visit device" -
 * and each row's dot is Home Assistant's availability view, which is better data anyway: it knows a
 * device is off the moment it disconnects, where a probe would take a timeout to notice.
 *
 * Satellite1 models only. A Nexus is in `dev` too, and a row that jumps to a device with no page to
 * serve is a trap. This device is dropped by MAC rather than by name, because the name is exactly
 * the field owners change. A row with no URL renders unlinked rather than being hidden - a device
 * that exists but cannot be jumped to is still worth seeing. An unavailable peer dims but keeps its
 * link, since Home Assistant's view can lag a reboot by a few seconds.
 */
function SwitcherSheet({ device, label, area, route, ha, onClose }) {
  const haOn = !!device?.ha;
  const haText = haOn ? TEXT.ha_connected : TEXT.ha_disconnected;
  const mac = (device?.mac || "").toLowerCase();

  // Sorted by area then name, so a house full of these groups by room, matching the tree on Config.
  const peers = (ha?.d?.dev || [])
    .filter((d) => /satellite1/i.test(d?.[0] || "") && (d?.[3] || "").toLowerCase() !== mac)
    .sort((x, y) => `${x[2]}\u0000${x[1]}`.localeCompare(`${y[2]}\u0000${y[1]}`));

  const [manual, setManual] = useState(readManualPeers);
  const [draft, setDraft] = useState("");

  // A manual entry the roster also lists is hidden rather than deleted, so an address added while
  // Home Assistant was away does not become a duplicate row when it comes back - and comes back as
  // a row again if Home Assistant goes away again.
  const rosterHosts = new Set(peers.map((d) => hostOf(d[5])).filter(Boolean));
  const extras = manual.filter((a) => !rosterHosts.has(hostOf(withScheme(a))));

  const add = () => {
    const a = draft.trim();
    if (!a) return;
    const next = manual.includes(a) ? manual : [...manual, a];
    setManual(next);
    writeManualPeers(next);
    setDraft("");
  };
  const drop = (a) => {
    const next = manual.filter((x) => x !== a);
    setManual(next);
    writeManualPeers(next);
  };

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
          {/* Address and room, which are the two things that tell one Satellite1 from another now that
              the sheet lists more than this one. The separator is dropped rather than left dangling when
              Home Assistant has not placed the device in an area. */}
          <div class="peer-sub">{[device?.ip, area].filter(Boolean).join(" \u2502 ")}</div>
        </div>
        {peers.map((d) => {
          const url = d[5] ? String(d[5]) : "";
          const up = d[6] === 1 || d[6] === "1";
          const dotText = up ? TEXT.peer_up : TEXT.peer_down;
          const body = (
            <>
              <div class="row">
                <span class={`dot${up ? " ok" : ""}`} title={dotText} aria-label={dotText} />
                <span class="grow">{d[1]}</span>
              </div>
              <div class="peer-sub">{[hostOf(url), d[2]].filter(Boolean).join(" \u2502 ")}</div>
            </>
          );
          return url ? (
            <a key={d[3]} class={`peer go${up ? "" : " off"}`} href={peerHref(url, route)}>
              {body}
            </a>
          ) : (
            <div key={d[3]} class={`peer${up ? "" : " off"}`}>
              {body}
            </div>
          );
        })}
        {extras.map((a) => (
          <div key={a} class="peer">
            <div class="row">
              {/* Neutral on purpose: nothing here can check an address someone typed, and a green dot
                  that means "assumed fine" beside one that means "Home Assistant saw it" is a lie. */}
              <span class="dot" title={TEXT.peer_manual} aria-label={TEXT.peer_manual} />
              <a class="grow peer-a" href={peerHref(withScheme(a), route)}>
                {a}
              </a>
              <button class="x" aria-label={`Remove ${a}`} onClick={() => drop(a)}>
                &#10005;
              </button>
            </div>
          </div>
        ))}
        {peers.length + extras.length === 0 && <p class="sheet-foot">{TEXT.no_devices}</p>}
        <div class="peer-add">
          <input
            class="inp sm grow"
            placeholder={TEXT.peer_add_ph}
            value={draft}
            onInput={(e) => setDraft(e.currentTarget.value)}
            onKeyDown={(e) => e.key === "Enter" && add()}
          />
          <button class="btn sm" onClick={add} disabled={!draft.trim()}>
            {TEXT.peer_add}
          </button>
        </div>
      </div>
    </div>
  );
}

/**
 * One toast for every write that did not land, on whatever route it happened.
 *
 * The controls already handle failure correctly and silently - each optimistic one puts its old
 * value back - and the silence is the problem: a switch that un-flips itself a second after being
 * tapped looks like a page that ignores clicks. This says the one thing all those cases share, and
 * tapping it goes to Diagnostics, whose log panel is where the specific reason is.
 *
 * A button rather than a div with a handler, so it is focusable and announced. New failures reset
 * the timer rather than stacking: a slider mid-drag against a dead device can fail a dozen writes a
 * second, and a dozen identical toasts is a haranguing, not a notification. Six seconds, matching
 * nothing in particular - long enough to read on a phone at arm's length, short enough that the
 * page does not wear a permanent error for one dropped packet.
 */
function ErrorToast() {
  const [shown, setShown] = useState(false);

  useEffect(() => {
    let timer;
    const off = onWriteError(() => {
      setShown(true);
      clearTimeout(timer);
      timer = setTimeout(() => setShown(false), 6000);
    });
    return () => {
      off();
      clearTimeout(timer);
    };
  }, []);

  if (!shown) return null;
  return (
    <button
      class="toast"
      onClick={() => {
        setShown(false);
        location.hash = "#/diagnostics";
      }}
    >
      <span class="toast-t">{TEXT.write_failed}</span>
      <span class="toast-s">{TEXT.write_failed_go}</span>
    </button>
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

      <ErrorToast />

      {/* Mounted whether or not it is open - see NavPane. */}
      <NavPane route={route} go={go} open={nav} onClose={() => setNav(false)} />
      {switcher && (
        <SwitcherSheet
          device={device}
          label={label}
          area={area}
          route={route}
          ha={ha.ha}
          onClose={() => setSwitcher(false)}
        />
      )}
    </div>
  );
}
