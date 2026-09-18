/**
 * App shell: top bar, nav pane, device switcher, and the router.
 *
 * Routing is on the hash. ESPHome's httpd has a single wildcard handler per method and no notion of
 * client-side routes, so a path-based router would 404 on reload for every route but "/". The hash
 * never reaches the server, which also means a bookmarked route survives a firmware update.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { HINTS, TEXT } from "./copy.js";
import { loginKey, logout, maybeRedirectLocal, peerLogin, primeOtherOrigin, takeUrlKey } from "./lib/auth.js";
import { deviceIdentity, haBlocked, onWriteError, useDeviceState, useEvents, useHaData, useSelection } from "./lib/device.js";
import { LoginScreen } from "./login.jsx";
import { MediaFooter } from "./media.jsx";
import { FixDrawer, Splash } from "./splash.jsx";
import { Config } from "./routes/config.jsx";
import { Controls } from "./routes/controls.jsx";
import { Diagnostics } from "./routes/diagnostics.jsx";
import { Presence } from "./routes/presence.jsx";
import { WakeWords } from "./routes/wakewords.jsx";
import { Chevron, Hint, N_AUDIO, N_DIAG, N_HOME, N_PRES, N_WAKE, ni, useDrawer, useSheetDrag } from "./ui.jsx";

/**
 * Five routes, in the order the owner set in the September 2026 information-architecture pass:
 * Controls became Home (the page you land on and glance at), the wake words card became a route of
 * its own, and Config became Audio (what remains there is the speaker, routing and ducking - all
 * sound). Presence is the only route that keeps polling while it is open, and the only one whose
 * data does not come from entities at all - the radar settings live in the module's own config,
 * behind `satellite1_radar`'s /api/v1 endpoints.
 */
/* The drawer glyphs live in ui.jsx now (the routes' top cards wear them too, and importing them from
   here would be a cycle). Only the logout mark is drawn locally, because only this drawer uses it:
   a door frame with the arrow leaving it, on the same 16-box as its siblings. */
const N_OUT = ni(
  <>
    <path d="M6.5 3H3.5v10h3" />
    <path d="M6.8 8h6" />
    <path d="M10.6 5.8 12.8 8l-2.2 2.2" />
  </>,
);

const ROUTES = [
  { id: "home", label: "Home", view: Controls, icon: N_HOME },
  { id: "wake-word", label: "Wake Word", view: WakeWords, icon: N_WAKE },
  { id: "audio", label: "Audio", view: Config, icon: N_AUDIO },
  { id: "presence", label: "Presence", view: Presence, icon: N_PRES },
  { id: "diagnostics", label: "Diagnostics", view: Diagnostics, icon: N_DIAG },
];

/** The pre-rename hashes. Bookmarks survive firmware updates by design (the hash never reaches the
 *  server), so the old names must keep landing somewhere better than the default. */
const LEGACY_ROUTES = { controls: "home", config: "audio" };

function useHashRoute() {
  const read = () => {
    const h = (location.hash.replace(/^#\/?/, "").split("?")[0] || "home").toLowerCase();
    return LEGACY_ROUTES[h] || h;
  };
  const [route, setRoute] = useState(read);

  useEffect(() => {
    const on = () => setRoute(read());
    addEventListener("hashchange", on);
    return () => removeEventListener("hashchange", on);
  }, []);

  return [ROUTES.some((r) => r.id === route) ? route : "home", (id) => (location.hash = `#/${id}`)];
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
 * It no longer carries a heading - the device name in the bar is only inches away - but it did grow
 * a footer: the device's name and firmware version, pinned to the bottom the way most apps sign
 * their drawer. That is identity, not navigation, and the one place it cannot be wrong to repeat it
 * is the menu that lists everything this device can show.
 */
function NavPane({ route, go, open, onClose, label, fw }) {
  // Escape and the one-drawer rule: opening the menu closes whatever sheet is standing, and any
  // sheet opening closes the menu. Gated on `open` because this component stays mounted shut.
  useDrawer("nav", open, onClose);

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
            {r.icon}
            {r.label}
          </button>
        ))}
        <div class="navpane-foot">
          <div class="navpane-dev">{label || "Satellite1"}</div>
          {fw && <div class="navpane-fw">Firmware {fw}</div>}
          {/* This browser only - the everywhere version lives on Diagnostics, where its blast
              radius can be explained. The reload lands on the boot probe, which now finds no
              session and shows the login screen. */}
          <button
            class="btn ghost sm navpane-logout"
            onClick={async () => {
              await logout();
              location.reload();
            }}
          >
            {N_OUT}
            {TEXT.logout}
          </button>
        </div>
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

/** "192.168.4.31" out of "http://192.168.4.31:80/", for the row's subtitle. */
const hostOf = (u) => String(u || "").replace(/^https?:\/\//, "").replace(/[/:].*$/, "");

/**
 * The device switcher: this device on top, then every other Satellite1 the Home Assistant payload
 * lists as available, then the unavailable ones folded into a submenu at the bottom.
 *
 * The roster is the `dev` block that already rides GET /api/sat1/ha - no discovery happens here.
 * mDNS browsing from a browser is not a thing, and probing peers directly is not either: Digest
 * credentials are scoped per origin and a cross-origin probe dies on the preflight. So each row's
 * link is the peer's `configuration_url` - the same address behind Home Assistant's "Visit device" -
 * and each row's dot is Home Assistant's availability view, which is better data anyway: it knows a
 * device is off the moment it disconnects, where a probe would take a timeout to notice. The caveat
 * that comes with it: the payload is the device's cache, refreshed once per page load, so a peer
 * that was powered off since carries a stale green dot until the next sync.
 *
 * Offline peers sit behind an "Offline" disclosure, collapsed by default, per the owner: rows for
 * powered-off devices are noise on the way to the one being looked for, but deleting them would
 * read as devices that ceased to exist. Each keeps its link inside the fold, since Home Assistant's
 * view can lag a reboot by a few seconds.
 *
 * Satellite1 models only. A Nexus is in `dev` too, and a row that jumps to a device with no page to
 * serve is a trap. This device is dropped by MAC rather than by name, because the name is exactly
 * the field owners change. A row with no URL renders unlinked rather than being hidden - a device
 * that exists but cannot be jumped to is still worth seeing.
 *
 * The add-by-address field and the manually-kept rows that used to end the sheet are gone at the
 * owner's request ("I don't know what that is" is a fair review of a fallback that needed a
 * paragraph to explain). The roster covers the case that matters; a peer Home Assistant cannot
 * list is reachable the way it always was - by typing its address in the URL bar.
 */
function SwitcherSheet({ device, label, area, route, ha, haRefresh, onClose }) {
  const haOn = !!device?.ha;
  const haText = haOn ? TEXT.ha_connected : TEXT.ha_disconnected;
  const mac = (device?.mac || "").toLowerCase();
  const [showOff, setShowOff] = useState(false);

  // Re-sync the roster the moment the sheet opens. The dots ride the device's cached payload, and
  // before this the cache was only rebuilt when the Audio or Wake Word route asked (once per page
  // load) - so a sheet opened from Home showed availability from whenever that last was, and a peer
  // powered off in between kept a green dot for days. One action call per open is cheap, haRefresh
  // re-reads the payload after the round trip, and the rows correct themselves a second or two in.
  useEffect(() => {
    haRefresh();
    // haRefresh is stable for the life of the app; this is per-open by design.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // A drawer like the media surfaces (owner's request, September 2026): Escape and the one-drawer
  // rule via useDrawer, and a finger-following swipe - upward, because this one hangs from the top.
  useDrawer("switcher", true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, -1);

  // Sorted by area then name, so a house full of these groups by room, matching the tree on Audio.
  const peers = (ha?.d?.dev || [])
    .filter((d) => /satellite1/i.test(d?.[0] || "") && (d?.[3] || "").toLowerCase() !== mac)
    .sort((x, y) => `${x[2]}\u0000${x[1]}`.localeCompare(`${y[2]}\u0000${y[1]}`));

  const isUp = (d) => d[6] === 1 || d[6] === "1";
  const online = peers.filter(isUp);
  const offline = peers.filter((d) => !isUp(d));

  // Room and address, which are the two things that tell one Satellite1 from another. Room first
  // and bold, per the owner: it is the half a person actually scans for - the address is what you
  // fall back on when two devices share a room. The separator is dropped rather than left dangling
  // when Home Assistant has not placed the device in an area.
  const peerSub = (room, host) => (
    <div class="peer-sub">
      {room && <strong>{room}</strong>}
      {room && host ? " \u2502 " : ""}
      {host}
    </div>
  );

  // The seamless jump: sign in to the peer before leaving, so its page opens as the app rather
  // than as its login screen. The peer's password rides the roster (see web_ui_ha.yaml - it is the
  // Web UI Password sensor every device already publishes to Home Assistant), the sign-in is the
  // same challenge-response the login form uses, and the landing goes through ?key= so the peer
  // sets its cookie first-party, where third-party cookie blocking cannot eat it. Anything that
  // declines - a peer on older firmware, a missing password, mDNS trouble - falls back to plain
  // navigation and the peer's own login page. The href stays real underneath, so middle-click and
  // open-in-new-tab keep working (they skip the sign-in and land on the fallback).
  //
  // When the login body carries the peer's hostname (newer firmware) and this page is itself on a
  // .local origin - free proof this browser resolves mDNS - the landing goes straight to the peer's
  // .local origin, skipping its IP-then-redirect double load; peers run the same firmware, so this
  // page's port is the peer's port. The name is regex-checked before it becomes a URL: it arrives
  // in a CORS-readable body, and hostname characters are all a hostname needs. Known accepted risk:
  // this proves our mDNS works, not that the peer's name resolves - a peer with mdns: disabled
  // lands on a browser error page instead of its IP. The fleet ships mDNS on (the whole redirect
  // strategy assumes it), and the failure is recoverable: back, or middle-click the real href.
  const jump = async (e, url, pw) => {
    if (!pw || e.button !== 0 || e.metaKey || e.ctrlKey) return;
    e.preventDefault();
    const origin = String(url).replace(/\/+$/, "");
    const peer = await peerLogin(origin, pw);
    if (!peer) {
      location.href = peerHref(url, route);
      return;
    }
    const base =
      location.hostname.endsWith(".local") && peer.name && /^[a-z0-9-]+$/i.test(peer.name)
        ? `http://${peer.name}.local${location.port ? `:${location.port}` : ""}`
        : origin;
    location.href = `${base}/?key=${peer.key}#/${route}`;
  };

  const peerRow = (d) => {
    const url = d[5] ? String(d[5]) : "";
    const up = isUp(d);
    const dotText = up ? TEXT.peer_up : TEXT.peer_down;
    const body = (
      <>
        <div class="row">
          <span class={`dot${up ? " ok" : ""}`} title={dotText} aria-label={dotText} />
          <span class="grow">{d[1]}</span>
        </div>
        {peerSub(d[2], hostOf(url))}
      </>
    );
    return url ? (
      <a key={d[3]} class={`peer go${up ? "" : " off"}`} href={peerHref(url, route)} onClick={(e) => jump(e, url, d[7])}>
        {body}
      </a>
    ) : (
      <div key={d[3]} class={`peer${up ? "" : " off"}`}>
        {body}
      </div>
    );
  };

  return (
    <div class="scrim" onClick={onClose}>
      <div class="sheet" style={dragStyle || undefined} {...drag} onClick={(e) => e.stopPropagation()}>
        {/* The players panel's top, borrowed wholesale (owner's request, September 2026 - it
            replaced a ✕/title/caret header that mirrored the top bar and read as clutter): a small
            dim label with the ⓘ, doubling as the swipe zone. The handle sits at the bottom edge,
            because that is the edge this drawer dismisses toward. */}
        <div class="mgroup-head dim sm" data-grab>
          Device Switcher
          <Hint text={HINTS.switcher} />
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
          {peerSub(area, device?.ip)}
        </div>
        {online.map(peerRow)}
        {offline.length > 0 && (
          <>
            <button class="offhead" aria-expanded={showOff} onClick={() => setShowOff(!showOff)}>
              <span class="grow">{`Offline (${offline.length})`}</span>
              <Chevron down={showOff} cls="caret-s" />
            </button>
            {showOff && offline.map(peerRow)}
          </>
        )}
        {/* An empty roster has two honest readings: nothing else in the house, or a roster the device
            is not allowed to fetch. While actions are blocked the foot says so, instead of promising
            rows that cannot arrive. */}
        {peers.length === 0 && <p class="sheet-foot">{haBlocked(ha) ? TEXT.no_devices_blocked : TEXT.no_devices}</p>}
        {/* The grab handle, at the bottom edge because that is the edge this drawer hangs toward -
            the mirror of the bottom drawers' top handle. */}
        <button class="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
      </div>
    </div>
  );
}

/**
 * Every toast in the app, on one shared surface at the bottom edge. Toasts are the app's whole
 * out-of-band vocabulary since the amber banners were retired (owner decision, September 2026).
 *
 * Two kinds live here, stacked when they coexist. The one transient slot holds a moment: a write
 * that did not land (the controls already put their old values back, and that silence is the
 * problem - a switch that un-flips itself looks like a page that ignores clicks), or the blocked
 * nudge (Home Assistant actions are off; tapping opens the fix drawer). New transients replace the
 * standing one rather than stacking - a slider mid-drag against a dead device can fail a dozen
 * writes a second, and a dozen identical toasts is a haranguing, not a notification. Six seconds
 * for the write, eight for the nudge, which carries a sentence more.
 *
 * The sticky toast below them is the stream-lost state - an ongoing condition, not a moment, so it
 * stays until the SSE stream reconnects rather than timing out. It used to be a banner under the
 * top bar; it kept its wording and its meaning, only the surface changed.
 *
 * Buttons rather than divs with handlers, so each is focusable and announced. The write toast and
 * the stream toast go to Diagnostics, whose log panel is where the specific reason is.
 */
function ToastHost({ connected, blocked, onFix }) {
  const [t, setT] = useState(null);
  const timer = useRef(null);

  const show = (kind, ms) => {
    setT(kind);
    clearTimeout(timer.current);
    timer.current = setTimeout(() => setT(null), ms);
  };

  useEffect(() => {
    const off = onWriteError(() => show("write", 6000));
    return () => {
      off();
      clearTimeout(timer.current);
    };
    // show holds no state; the empty deps are the subscription's lifetime.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // The nudge, on the rising edge only: once when the app is entered while blocked (which includes
  // arriving through the splash's Continue), and again only if the state genuinely re-enters -
  // never on every re-render of a blocked session.
  useEffect(() => {
    if (blocked) show("blocked", 8000);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [blocked]);

  return (
    <div class="toasts">
      {t === "write" && (
        <button
          class="toast"
          onClick={() => {
            setT(null);
            location.hash = "#/diagnostics";
          }}
        >
          <span class="toast-t">{TEXT.write_failed}</span>
          <span class="toast-s">{TEXT.write_failed_go}</span>
        </button>
      )}
      {t === "blocked" && (
        <button
          class="toast warn"
          onClick={() => {
            setT(null);
            onFix();
          }}
        >
          <span class="toast-t">{TEXT.blocked_toast_t}</span>
          <span class="toast-s">{TEXT.blocked_toast_s}</span>
        </button>
      )}
      {!connected && (
        <button class="toast warn" onClick={() => (location.hash = "#/diagnostics")}>
          <span class="toast-t">{TEXT.stream_lost}</span>
          <span class="toast-s">{TEXT.write_failed_go}</span>
        </button>
      )}
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Shell                                                               */
/* ------------------------------------------------------------------ */

/**
 * The gatekeeper around the app: nothing below mounts until the session question is answered, so
 * an unauthenticated tab costs the device one probe rather than a storm of 401s from every hook.
 *
 * The boot sequence, in order and each for a reason. The smart redirect runs first, so everything
 * that follows - the cookie a login sets most of all - lands on the stable .local origin rather
 * than an IP that DHCP can reassign. Then a ?key= from a sign-in link is redeemed and scrubbed
 * from the URL. Then the session probe: one tiny gated GET whose 401 is the difference between
 * the login screen and the app.
 */
export function App() {
  // "boot" while the redirect probe and session check run, then "login" or "in".
  const [phase, setPhase] = useState("boot");
  // The sign-in key, held only long enough for the dual-origin priming in AppInner, then dropped.
  const primeKey = useRef(null);

  // The static splash from index.html, torn down the moment boot resolves into a real screen.
  // Removed rather than hidden: it exists only for boot, and boot happens once per page load. While
  // the smart redirect is navigating away, phase stays "boot" and the splash stays up - the right
  // cover for a page about to be replaced.
  useEffect(() => {
    if (phase !== "boot") document.getElementById("splash")?.remove();
  }, [phase]);

  useEffect(() => {
    (async () => {
      if (await maybeRedirectLocal()) return; // The page is navigating away; render nothing.
      const urlKey = takeUrlKey();
      if (urlKey) {
        try {
          const r = await loginKey(urlKey);
          if (r.ok) {
            primeKey.current = r.key;
            setPhase("in");
            return;
          }
        } catch {
          /* A dead link falls through to the probe; a live cookie may still exist. */
        }
      }
      try {
        const r = await fetch("/api/sat1/sel", { cache: "no-store", signal: AbortSignal.timeout(15000) });
        setPhase(r.status === 401 ? "login" : "in");
      } catch {
        setPhase("login");
      }
    })();
  }, []);

  if (phase === "boot") return null;
  if (phase === "login") {
    return (
      <LoginScreen
        onSignedIn={(key) => {
          primeKey.current = key || null;
          setPhase("in");
        }}
      />
    );
  }
  return <AppInner primeKey={primeKey} onAuthLost={() => setPhase("login")} />;
}

function AppInner({ primeKey, onAuthLost }) {
  const [route, go] = useHashRoute();
  const [nav, setNav] = useState(false);
  const [switcher, setSwitcher] = useState(false);
  // The verdict overlay (splash.jsx): up from the first authenticated render, gone for good once it
  // fades - state rather than anything remembered, so every page load gets the same honest check.
  const [splashDone, setSplashDone] = useState(false);
  // The fix drawer, reachable from the blocked toast and every in-place "Show fix" link.
  const [fixOpen, setFixOpen] = useState(false);

  // The drawer's own items close it as they navigate; this covers the routes nobody tapped - the
  // back button, a bookmark, a hash typed over the current one - so a navigation never leaves the
  // menu standing over the page it just changed.
  useEffect(() => setNav(false), [route]);

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

  // Dual-origin cookie priming: one CORS login against the device's other entrance (.local when we
  // are on the IP, the IP when we are on .local), fired once the state payload supplies the halves
  // we do not know. The key is dropped the moment it is used - it lives nowhere but the cookie
  // after this.
  useEffect(() => {
    if (!primeKey.current || !device) return;
    primeOtherOrigin(primeKey.current, device.name, device.ip);
    primeKey.current = null;
  }, [device, primeKey]);

  // A session dying under a running app - the password changed, or Sign out everywhere pressed
  // somewhere else. The state poll is the heartbeat that notices, and the login screen is the only
  // honest response.
  useEffect(() => {
    if (deviceError === "HTTP 401") onAuthLost();
  }, [deviceError, onAuthLost]);

  const active = ROUTES.find((r) => r.id === route) || ROUTES[0];
  const View = active.view;
  // onShowFix rides the ctx so any route's "Show fix" link can open the drawer without threading a
  // prop through every card between here and there.
  const ctx = { device, deviceError, ...events, ...ha, ...selection, onShowFix: () => setFixOpen(true) };
  // Resolved once here rather than in the three places that show it, so the bar, the nav pane and the
  // switcher sheet cannot end up disagreeing about what this device is called.
  const { name: label, area } = deviceIdentity(device, ha.ha);

  // The tab title follows the same identity: the friendly name once it is known, with the login
  // screen having already set the hostname as the fallback for the time before (and for browsers
  // that never sign in).
  useEffect(() => {
    if (label) document.title = label;
  }, [label]);

  return (
    <div class="app">
      <header class="topbar">
        {/* A toggle, not an opener: the drawer's scrim starts below this bar, so the burger stays
            visible while the drawer is out and a second press should put things back. */}
        <button class="icon" aria-label="Menu" aria-expanded={nav} onClick={() => setNav((v) => !v)}>
          <span class="burger" />
        </button>
        {/* The caret has to sit against the name for the two to read as one control. It used to be a
            .grow span here, which pushed the caret to the far edge of the bar next to the status dot,
            where it looked like a stray mark rather than "there is a menu on this". */}
        <button class="title" onClick={() => setSwitcher(true)}>
          {/* The Home Assistant dot, back beside the name it describes (September 2026 polish plan).
              It left once so the bar would not grow a dot per peer; this is one dot about one device -
              the one serving the page - and the switcher it opens still carries the per-device story.
              Same class and meaning as the switcher's own, so the two can never disagree. */}
          <span
            class={`dot${device?.ha ? " ok" : ""}`}
            title={device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected}
            aria-label={device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected}
          />
          {/* A placeholder for the one paint before /api/sat1/state answers, not a prefix - the name is
              whatever deviceLabel returns, on its own, so nothing here can double it up. */}
          <span class="tname">{label || "Satellite1"}</span>
          {/* Always down: this opens a sheet, and a dropdown that points sideways reads as a link. */}
          <Chevron down cls="caret" />
        </button>
        <ThemeSwitch />
      </header>

      {/* The stream-lost banner that sat here moved onto the toast surface below when the amber
          banners were retired - same wording, same meaning, one place for out-of-band news. */}

      <main class="wrap">
        <View ctx={ctx} />
      </main>

      {/* On every route, below the content - .wrap carries bottom padding so nothing hides under it.
          The footer owns the media poll now that it is the one media surface (the card it replaced
          lived on home and polled only there). */}
      <MediaFooter ha={ha.ha} mac={device?.mac} />

      {/* `blocked` waits for the splash to leave: while it is up, the verdict is its story to tell,
          and the toast's job is to keep the fix reachable afterwards. */}
      <ToastHost connected={events.connected} blocked={splashDone && haBlocked(ha.ha)} onFix={() => setFixOpen(true)} />

      {fixOpen && <FixDrawer ctx={ctx} onClose={() => setFixOpen(false)} />}

      {/* Last, so it paints over everything below while it stands. Mounted with AppInner: this is
          the first authenticated moment, whether the session came from the login screen a second
          ago or from a 90-day cookie. */}
      {!splashDone && <Splash ctx={ctx} onDone={() => setSplashDone(true)} />}

      {/* Mounted whether or not it is open - see NavPane. */}
      <NavPane route={route} go={go} open={nav} onClose={() => setNav(false)} label={label} fw={device?.fw} />
      {switcher && (
        <SwitcherSheet
          device={device}
          label={label}
          area={area}
          route={route}
          ha={ha.ha}
          haRefresh={ha.haRefresh}
          onClose={() => setSwitcher(false)}
        />
      )}
    </div>
  );
}
