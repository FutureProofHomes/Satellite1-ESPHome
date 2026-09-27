/**
 * The onboarding wizard: what a factory-fresh device serves instead of the login screen.
 *
 * The flow is browser-first (owner design, September 26 2026). The OS captive-portal sheet that
 * pops when a phone joins the setup AP gets exactly one page - a launcher whose one button opens
 * the customer's real browser at this same address - because the sheet closes itself the moment
 * the device leaves the setup network, and nothing served into it can outlive that. A real browser
 * tab survives the hop: the customer picks their WiFi there, and the joining step probes the
 * device's home-network address (http://<name>.local) until the phone can reach it, then redirects
 * - which is the instant the phone has fallen back to the home WiFi. Everything after the join
 * lives at that address: the Connect Mode chooser, then the Home Assistant connect step
 * (#/home-assistant-connect), which waits while the customer adds the device in Home Assistant.
 * The device notices the API attach and completes onboarding itself (no password step - the
 * generated password is published to Home Assistant, and VoiceTap needs none); this page's poll
 * sees it and hands over to the login screen.
 *
 * The shell mounts this whenever GET /api/sat1/setup/status says onboarding is pending; the device
 * side (session gate) keeps the endpoints this uses sessionless exactly as long as that is true.
 * Everything wears the login screen's clothes (.login, .login-card) because it is the login
 * screen's sibling: the first thing a browser sees, centered, one card of decisions.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { TEXT } from "./copy.js";
import { openHomeAssistant } from "./lib/openha.js";
import { portalPass, probeSetup, setupMode, setupStatus, wifiJoin, wifiScan, wifiStatus } from "./lib/setup.js";
import { cogStep, Logo } from "./ui.jsx";

// The real ESPHome mark (owner-supplied PNG, September 25 2026), inlined by esbuild's dataurl
// loader. It dresses the Discovered mock so the card is visually the one the customer must find in
// Home Assistant - an approximation would defeat the "looks exactly like this" instruction.
import esphomeLogo from "../assets/esphome-logo.png";

/**
 * Whether this page is inside an OS captive-portal sheet rather than a real browser - what decides
 * between the launcher and the network list. The iOS/macOS sheet is WebKit without the `Safari/`
 * token every real browser on Apple platforms carries (Chrome and Firefox on iOS carry it too);
 * Android's sheet is a bare WebView (the `; wv)` marker) or names its CaptivePortalLogin app
 * outright. A wrong "true" costs nothing - the launcher's "Continue here instead" link is one tap -
 * while a wrong "false" would strand a sheet user in a flow whose ending the sheet cannot show,
 * which is why the Apple test leans toward sheet.
 */
const inCaptiveSheet = () => {
  const ua = navigator.userAgent;
  if (/AppleWebKit/i.test(ua) && !/Safari\//i.test(ua)) return true;
  return /; wv\)/i.test(ua) || /CaptivePortalLogin/i.test(ua);
};

/**
 * How the launcher gets out of the captive sheet, in ONE tap - the machinery runs itself.
 *
 * The obvious escape does not exist: iOS opens other apps' URL schemes from the sheet but not the
 * browser's own (x-safari-* included; confirmed on hardware September 26 2026 and in Apple's
 * developer forums going back years). What DOES work is the mechanism production captive portals
 * ride (Cisco Spaces documents it as their iOS flow): once the portal answers the OS's
 * connectivity probes with the expected "online" responses, the sheet flips to its connected state
 * - Cancel becomes Done - and from THAT state, a tapped absolute link opens in the real browser,
 * possibly underneath the sheet until Done is tapped.
 *
 * Both preconditions run without the customer (hardware-proven order, September 26 2026): entering
 * the launcher opens the pass window (portalPass) and then NAVIGATES this very page to
 * /?setup=prime - a real navigation, which is what makes the sheet re-check connectivity;
 * background fetches do not count. The reloaded page holds a quiet "getting ready" beat while the
 * sheet acts on the re-check, then offers the one button: a plain absolute link the now-connected
 * sheet hands to the real browser. Android's sheet honours intent:// URLs directly, so its button
 * fires one at the default browser instead.
 *
 * ?setup=go rides the button so that the one failure mode - a tap the sheet kept for itself -
 * lands past the launcher on the network list instead of looping, and costs nothing when the link
 * opens in the real browser as intended.
 */
const androidBrowserIntent = (host) => {
  if (/android/i.test(navigator.userAgent)) {
    location.href = `intent://${host}/?setup=go#Intent;scheme=http;action=android.intent.action.VIEW;end`;
    return true;
  }
  return false;
};

/** How long the reloaded launcher holds its "getting ready" beat before offering the button: the
 *  reload's re-probe takes the sheet a few seconds to act on, and a tap inside that window opens
 *  in-sheet. */
const LAUNCH_HOLD_S = 5;

/** Signal strength as 0-4 bars out of dBm: the usual thresholds, advisory either way. */
const barsOf = (rssi) => (rssi >= -55 ? 4 : rssi >= -66 ? 3 : rssi >= -77 ? 2 : rssi >= -88 ? 1 : 0);

const Bars = ({ rssi }) => {
  const n = barsOf(rssi);
  return (
    <svg class="wifi-bars" viewBox="0 0 16 14" aria-hidden="true">
      {[0, 1, 2, 3].map((i) => (
        <rect key={i} x={i * 4} y={11 - i * 3} width="2.6" height={3 + i * 3} rx="1" opacity={i < n ? 1 : 0.25} />
      ))}
    </svg>
  );
};

const LockIcon = () => (
  <svg class="wifi-lock" viewBox="0 0 12 12" fill="none" stroke="currentColor" stroke-width="1.3" aria-hidden="true">
    <rect x="2.4" y="5.2" width="7.2" height="5" rx="1.2" />
    <path d="M4 5V3.6a2 2 0 0 1 4 0V5" />
  </svg>
);

/** The alternating plain/bold copy the HA connect instructions use - segments from copy.js, odd
 *  indices bolded, so the UI nouns (Settings path, Discovered, Add) stand out of the sentence. */
const BoldedCopy = ({ segments }) => (
  <p class="setup-copy">{segments.map((s, i) => (i % 2 ? <b key={i}>{s}</b> : s))}</p>
);

/** The Connect Mode roster: id is what POST /api/sat1/setup/mode receives (the device refuses all
 *  but "ha" with the coming-soon shape), live is the one choice that exists today. The chooser is
 *  no longer a wizard step - with one live option it was ceremony (owner decision, September 26
 *  2026; the cloud modes left the roadmap the same day) - so this renders only behind the HA
 *  connect step's Change link, where it waits for the day the Basestation ships. */
const MODES = [
  { id: "nexus_local", name: () => TEXT.setup_mode_nexus_local, sub: () => TEXT.setup_mode_nexus_local_sub, live: false },
  { id: "ha", name: () => TEXT.setup_mode_ha, sub: () => TEXT.setup_mode_ha_sub, live: true },
];

export function SetupWizard({ status, onDone }) {
  // "boot" resolves into the right entry step from the device's own facts, so a reload lands
  // where the customer actually is: mid-AP, mid-join, or back on the home network mid-wizard.
  const [step, setStep] = useState("boot");
  const [err, setErr] = useState(null);
  const [busy, setBusy] = useState(false);
  // The launcher's two taps: false shows the big button; true (restored from ?setup=prime after
  // the priming reload) shows the Continue stage. `hold` counts down the breath the sheet needs
  // to act on the reload's re-probe before Continue is worth tapping.
  const [launched, setLaunched] = useState(false);
  const [hold, setHold] = useState(LAUNCH_HOLD_S);

  // The network list and the selection on it.
  const [aps, setAps] = useState(null);
  const [picked, setPicked] = useState(null); // {ssid, sec} or null
  const [manual, setManual] = useState(false);
  const [ssidInput, setSsidInput] = useState("");
  const [wifiPw, setWifiPw] = useState("");

  // The joining step's facts: what we're joining, where the device lives afterwards, and how the
  // polls fare. `fn` is the friendly display name the Discovered mock wears.
  const [joinSsid, setJoinSsid] = useState("");
  const [host, setHost] = useState(status?.name || "");
  const [fn, setFn] = useState(status?.fn || "");
  const [joinState, setJoinState] = useState("trying"); // trying | slow
  const joinStart = useRef(0);
  // One navigation, however many polls confirm the way is clear.
  const redirected = useRef(false);
  // The station IP, snatched from wifi/status during the brief AP+STA overlap after the join
  // succeeds. It is the Android fix: Android browsers cannot resolve .local, so the redirect
  // probes this address too and navigates to whichever answers.
  const staIp = useRef(null);
  // Whether setup/mode=ha has been recorded - the chooser is no longer a step, so the HA connect
  // step records the mode itself on entry (the device needs it to self-complete). Seeded from the
  // mount-time status; the Change chooser's own pick sets it too.
  const [modeSet, setModeSet] = useState(status?.mode === 1);

  const live = useRef(true);
  useEffect(() => () => (live.current = false), []);

  /** The device's address on the home network. Whether this page is already served from it decides
   *  between redirecting there and simply advancing the step - checked inline where it matters. */
  const homeOrigin = host ? `http://${host}.local` : "";

  /* Entry: ask the device where the flow stands. Connected means the WiFi half is done (however it
     was done) and the Home Assistant connect step is next - directly, no chooser (it records the
     mode itself on entry). Not connected: a real browser goes straight to the network list (the
     wizard-flag case - a join that failed or is retrying - lands there too), and only the captive
     sheet sees the launcher, whose one job is to move the customer into that real browser. */
  useEffect(() => {
    (async () => {
      const st = await wifiStatus();
      if (!live.current) return;
      if (st?.host) setHost(st.host);
      if (st?.connected) {
        setStep("haconnect");
      } else if (/[?&]setup=prime\b/.test(location.search)) {
        // The launcher's own priming reload: land on its Continue stage, wherever this loaded.
        setLaunched(true);
        setStep("launcher");
      } else if (inCaptiveSheet() && !/[?&]setup=go\b/.test(location.search)) {
        // ?setup=go is the launcher's own Continue link: a tap the sheet kept for itself must not
        // loop back to the launcher - the network list works in-sheet, and the joining screen's
        // fallback address covers that path's ending.
        setStep("launcher");
      } else {
        setStep("network");
      }
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  /* The network list: read what the last scan saw, ask for a fresh one, and keep reading - rescan
     results land on a later poll by design (the device answers before it hops channels). */
  useEffect(() => {
    if (step !== "network") return;
    let asked = false;
    const load = async () => {
      const list = await wifiScan(!asked);
      asked = true;
      if (!live.current) return;
      if (list) setAps((prev) => (list.length || !prev ? list : prev));
    };
    load();
    const t = setInterval(load, 4000);
    return () => clearInterval(t);
  }, [step]);

  /* The joining step's heartbeat, and the redirect that carries the customer across the network
     gap. Each tick probes the device's home-network addresses cross-origin - the .local name AND
     the station IP once wifi/status has leaked it during the brief AP+STA overlap. The IP is the
     Android fix: Android browsers cannot resolve .local, so without it the redirect never fired
     there at all. The first tick runs immediately and the poll runs every 1.25s.
     Status polls still answering but never connecting past 45s read as a wrong password; a dead
     AP under this page needs no state of its own anymore - the probes are the plan, and the
     fallback address line is the guarantee.

     Two hard-won rules govern the navigation itself (hardware, September 26 2026 - a redirect
     landed on Safari's "not connected to the internet" page and stranded the customer, because we
     had already left our own page and nothing could retry):
     - Probes only start once the join is real - the status poll reported connected, or stopped
       answering (the AP died under us). Probing earlier risks false positives: the device answers
       its own .local name over the AP too.
     - Navigation demands TWO consecutive successes on the same address. The phone's hop off the
       dying AP is a dance (cellular, then home WiFi, then validation), and one probe can thread a
       gap the navigation a beat later cannot. Two answers 1.25s apart mean the network has
       settled, and the navigation rides the same settled path. */
  useEffect(() => {
    if (step !== "joining") return;
    joinStart.current = Date.now();
    const onHomeOrigin = () => host && location.hostname.toLowerCase() === `${host.toLowerCase()}.local`;
    // Per-origin consecutive-success counts, and whether probing is armed at all.
    const streak = {};
    let armed = false;
    const tick = async () => {
      const st = await wifiStatus();
      if (!live.current) return;
      if (st) {
        if (st.host) setHost(st.host);
        if (st.connected) {
          armed = true;
          if (st.ip) staIp.current = st.ip;
          if (onHomeOrigin()) {
            // Already on the home origin (a resumed wizard re-running the join): no gap to cross.
            setStep("haconnect");
            return;
          }
        } else if (Date.now() - joinStart.current > 45000) {
          setJoinState("slow");
        }
      } else {
        // The AP closed under this page: the join succeeded and the hop has begun.
        armed = true;
      }
      // Fired concurrently and not awaited: a hung .local lookup must not delay the IP probe that
      // will actually answer on Android.
      if (armed && !redirected.current && !onHomeOrigin()) {
        const origins = [];
        if (host) origins.push(`http://${host}.local`);
        if (staIp.current) origins.push(`http://${staIp.current}`);
        for (const origin of origins) {
          probeSetup(origin).then((there) => {
            if (!there) {
              streak[origin] = 0;
              return;
            }
            streak[origin] = (streak[origin] || 0) + 1;
            if (streak[origin] >= 2 && !redirected.current) {
              redirected.current = true;
              location.href = `${origin}/`;
            }
          });
        }
      }
    };
    tick();
    const t = setInterval(tick, 1250);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step, host]);

  /* The HA connect step records the mode itself: the chooser is no longer a wizard step (one live
     option was ceremony), but the device still needs mode=ha before it can complete onboarding on
     API attach. Idempotent - the endpoint answers ok to a re-record, and done:1 (already
     onboarded) just means the poll below is about to hand over anyway. */
  useEffect(() => {
    if (step !== "haconnect" || modeSet) return;
    setupMode("ha").then((r) => {
      if (live.current && (r.ok || r.done)) setModeSet(true);
    });
  }, [step, modeSet]);

  /* The wizard's hand-over to the login screen, hash scrubbed on the way out. */
  const finish = () => {
    try {
      history.replaceState(null, "", location.pathname + location.search);
    } catch {
      /* ignore */
    }
    onDone(null);
  };

  /* The Home Assistant connect step: wear the #/home-assistant-connect address (owner request -
     replaceState so the captive sheet's back button is not trapped), and poll setup/status until
     the device reports onboarding done - it completes itself the moment the HA API attaches. Then
     the ending forks on the actions verdict (owner request, September 26 2026): allowed (1), or an
     HA too old to answer the probe but whose calls still execute (3), hands straight over to the
     login screen; blocked or still-probing goes to the haactions step, so the checkbox gets
     ticked BEFORE the login's auto-VoiceTap - which is what lets that window speak the 4-digit
     code instead of falling back to the wake-word challenge. */
  useEffect(() => {
    if (step !== "haconnect") return;
    try {
      history.replaceState(null, "", `${location.pathname}${location.search}#/home-assistant-connect`);
    } catch {
      /* A webview that refuses replaceState keeps the old hash; the step works regardless. */
    }
    const t = setInterval(async () => {
      const st = await setupStatus();
      if (!live.current || !st) return;
      if (st.fn) setFn(st.fn);
      if (st.name) setHost(st.name);
      if (st.setup === 0) {
        clearInterval(t);
        if (st.actions === 1 || st.actions === 3) {
          finish();
        } else {
          setStep("haactions");
        }
      }
    }, 2000);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step]);

  /* The actions step's watcher: ticking the checkbox reloads the config entry, which reconnects
     the API and re-fires the device's probe - the verdict flips to allowed within about a second,
     and this poll advances on it with no button to press. */
  useEffect(() => {
    if (step !== "haactions") return;
    try {
      history.replaceState(null, "", `${location.pathname}${location.search}#/home-assistant-actions`);
    } catch {
      /* ignore */
    }
    const t = setInterval(async () => {
      const st = await setupStatus();
      if (!live.current || !st) return;
      if (st.fn) setFn(st.fn);
      if (st.actions === 1 || st.actions === 3) {
        clearInterval(t);
        finish();
      }
    }, 2000);
    return () => clearInterval(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step]);

  const join = async (ssid, sec) => {
    const password = wifiPw;
    if (sec && password.length > 0 && password.length < 8) {
      setErr(TEXT.setup_join_short);
      return;
    }
    setBusy(true);
    setErr(null);
    try {
      const r = await wifiJoin(ssid, password);
      if (r.ok) {
        if (r.host) setHost(r.host);
        setJoinSsid(ssid);
        setJoinState("trying");
        redirected.current = false;
        setWifiPw("");
        setStep("joining");
        return;
      }
      setErr(r.invalid ? TEXT.setup_join_short : TEXT.setup_join_failed);
    } catch {
      setErr(TEXT.setup_join_failed);
    } finally {
      setBusy(false);
    }
  };

  /* The launcher's automatic half: entering it (fresh, not via the priming reload) opens the pass
     window and navigates through ?setup=prime - the navigation is what makes the sheet re-check
     connectivity. replace() rather than href, so the sheet's back gesture cannot land on a page
     whose only job was to leave. */
  useEffect(() => {
    if (step !== "launcher" || launched) return;
    (async () => {
      await portalPass();
      if (!live.current) return;
      location.replace(`http://${location.host}/?setup=prime`);
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step, launched]);

  /* The reloaded launcher's "getting ready" beat: the re-probe takes the sheet a few seconds to
     act on (watch Cancel become Done), and a tap inside that window opens in-sheet. A couple of
     probe-shaped requests ride along as re-check nudges - cheap, even if only navigations are
     guaranteed to count. */
  useEffect(() => {
    if (step !== "launcher" || !launched) return;
    setHold(LAUNCH_HOLD_S);
    let n = LAUNCH_HOLD_S;
    const t = setInterval(() => {
      if (!live.current) {
        clearInterval(t);
        return;
      }
      fetch(`http://captive.apple.com/hotspot-detect.html?t=${Date.now()}`, { mode: "no-cors", cache: "no-store" }).catch(
        () => {},
      );
      n -= 1;
      setHold(n);
      if (n <= 0) clearInterval(t);
    }, 1000);
    return () => clearInterval(t);
  }, [step, launched]);

  /* The Change chooser's pick. Also how the wizard returns from the chooser: Home Assistant is
     the only live card, so choosing is confirming. */
  const chooseHa = async () => {
    setBusy(true);
    setErr(null);
    try {
      const r = await setupMode("ha");
      if (!r.ok && !r.done) {
        setErr(TEXT.setup_mode_failed);
        return;
      }
      setModeSet(true);
      setStep("haconnect");
    } catch {
      setErr(TEXT.setup_mode_failed);
    } finally {
      setBusy(false);
    }
  };

  const handoffUrl = homeOrigin;

  const networkRow = (ap) => {
    const on = picked?.ssid === ap.ssid && !manual;
    return (
      <div key={ap.ssid} class={`wifi-row${on ? " on" : ""}`}>
        <button
          class="wifi-hit"
          onClick={() => {
            setManual(false);
            setErr(null);
            setWifiPw("");
            setPicked(on ? null : { ssid: ap.ssid, sec: ap.sec === 1 });
          }}
        >
          <span class="wifi-name">{ap.ssid}</span>
          {ap.conn === 1 && <span class="vtag lit">{TEXT.setup_row_connected}</span>}
          {ap.sec === 1 && <LockIcon />}
          <Bars rssi={ap.rssi} />
        </button>
        {on && (
          <form
            class="wifi-join"
            onSubmit={(e) => {
              e.preventDefault();
              join(ap.ssid, ap.sec === 1);
            }}
          >
            {ap.sec === 1 ? (
              <div class="login-field">
                {/* type="text" masked by CSS, NOT type="password": a WiFi key is not an account
                    credential, and password managers pounced on the real thing with save-this and
                    strong-password sheets mid-onboarding (owner report, September 26 2026 -
                    Bitwarden's sheet covered the Join button). Managers key on the input type;
                    -webkit-text-security keeps the dots without it, and the vendor data-*
                    attributes tell the majors to stand down besides. */}
                <input
                  type="text"
                  class="wifi-mask"
                  name="wifi-key"
                  placeholder={TEXT.setup_wifi_pw_placeholder}
                  autocomplete="off"
                  autocorrect="off"
                  autocapitalize="none"
                  spellcheck={false}
                  data-1p-ignore
                  data-lpignore="true"
                  data-bwignore
                  data-form-type="other"
                  autofocus
                  value={wifiPw}
                  onInput={(e) => setWifiPw(e.currentTarget.value)}
                  aria-label={TEXT.setup_wifi_pw_placeholder}
                />
              </div>
            ) : (
              <div class="login-hint">{TEXT.setup_wifi_pw_open}</div>
            )}
            <button class="btn solid" type="submit" disabled={busy}>
              {busy ? <span class="login-spin" aria-hidden="true" /> : TEXT.setup_join}
            </button>
          </form>
        )}
      </div>
    );
  };

  /* The Discovered mock: Home Assistant's own card, recreated live so it carries this device's
     real name - the customer is being told "find exactly this", and exactly this is what renders.
     Decorative throughout (the buttons are what to look for in HA, not controls here). */
  const discoveredMock = (
    <div class="ha-wrap" aria-hidden="true">
      <p class="ha-disc">{TEXT.setup_hac_disc}</p>
      <div class="ha-card">
        <span class="ha-kebab">{"\u22ee"}</span>
        <img class="ha-logo" src={esphomeLogo} alt="" />
        <div class="ha-name">
          {fn || "Satellite1"} ({host || "satellite1"})
        </div>
        <div class="ha-sub">ESPHome</div>
        <div class="ha-btns">
          <span class="ha-ignore">{TEXT.setup_hac_ignore}</span>
          <span class="ha-add">{TEXT.setup_hac_add}</span>
        </div>
      </div>
    </div>
  );

  return (
    <div class="login setup">
      <div class="login-glow" aria-hidden="true" />
      <div class="login-hero">
        <Logo />
        <h1 class="login-name">{TEXT.setup_title}</h1>
      </div>

      <div class="card login-card">
        {step === "boot" && <div class="login-hint">{TEXT.setup_scanning}</div>}

        {step === "launcher" && (
          <div class="setup-launch">
            <p class="setup-copy">{TEXT.setup_launch_copy}</p>
            {!launched || hold > 0 ? (
              /* The automatic half at work: priming the way out, then the beat the sheet needs to
                 act on it. One quiet line - the machinery is not the customer's problem. */
              <div class="setup-wait-row" role="status">
                <span class="login-pulse" aria-hidden="true" />
                <span>{TEXT.setup_launch_prep}</span>
              </div>
            ) : (
              <>
                {/* A plain absolute link on purpose: from the sheet's connected state (which the
                    priming reload arranged), this is what the OS hands to the real browser. On
                    Android the sheet honours a browser intent directly instead. */}
                <a
                  class="btn solid setup-launch-btn"
                  href={`http://${location.host}/?setup=go`}
                  onClick={(e) => {
                    if (/android/i.test(navigator.userAgent)) {
                      e.preventDefault();
                      androidBrowserIntent(location.host);
                    }
                  }}
                >
                  {TEXT.setup_launch_btn}
                </a>
                <div class="login-hint">{TEXT.setup_launch_retry}</div>
              </>
            )}
            <div class="login-hint">
              {TEXT.setup_launch_fallback} <b>{`http://${location.host}`}</b>
            </div>
            {/* The sheet can finish the WiFi half itself; only the ending needs a real browser,
                and the joining screen's fallback address covers whoever takes this path. */}
            <button class="btn ghost sm" onClick={() => setStep("network")}>
              {TEXT.setup_launch_here}
            </button>
          </div>
        )}

        {step === "network" && (
          <>
            <div class="setup-head">{TEXT.setup_pick}</div>
            <div class="login-hint setup-pick-hint">{TEXT.setup_pick_hint}</div>
            <div class="wifi-list">
              {aps === null && <div class="login-hint">{TEXT.setup_scanning}</div>}
              {aps !== null && aps.length === 0 && <div class="login-hint">{TEXT.setup_no_networks}</div>}
              {(aps || []).map(networkRow)}
            </div>
            {/* The hidden-network path: a name field beside the password, same join underneath. */}
            <button
              class="btn ghost sm setup-other"
              onClick={() => {
                setPicked(null);
                setErr(null);
                setManual((v) => !v);
              }}
            >
              {TEXT.setup_other_network}
            </button>
            {manual && (
              <form
                class="wifi-join"
                onSubmit={(e) => {
                  e.preventDefault();
                  if (ssidInput) join(ssidInput, true);
                }}
              >
                <div class="login-field">
                  {/* The same manager-suppression dress as the row field below: a name+secret pair
                      of inputs is exactly the shape managers read as a login form. */}
                  <input
                    type="text"
                    name="ssid"
                    placeholder={TEXT.setup_ssid_placeholder}
                    autocomplete="off"
                    autocorrect="off"
                    autocapitalize="none"
                    spellcheck={false}
                    data-1p-ignore
                    data-lpignore="true"
                    data-bwignore
                    data-form-type="other"
                    autofocus
                    value={ssidInput}
                    onInput={(e) => setSsidInput(e.currentTarget.value)}
                    aria-label={TEXT.setup_ssid_placeholder}
                  />
                </div>
                <div class="login-field">
                  <input
                    type="text"
                    class="wifi-mask"
                    name="wifi-key"
                    placeholder={TEXT.setup_wifi_pw_placeholder}
                    autocomplete="off"
                    autocorrect="off"
                    autocapitalize="none"
                    spellcheck={false}
                    data-1p-ignore
                    data-lpignore="true"
                    data-bwignore
                    data-form-type="other"
                    value={wifiPw}
                    onInput={(e) => setWifiPw(e.currentTarget.value)}
                    aria-label={TEXT.setup_wifi_pw_placeholder}
                  />
                </div>
                <button class="btn solid" type="submit" disabled={busy || !ssidInput}>
                  {busy ? <span class="login-spin" aria-hidden="true" /> : TEXT.setup_join}
                </button>
              </form>
            )}
            <button class="btn ghost sm" onClick={() => wifiScan(true)}>
              {TEXT.setup_rescan}
            </button>
            {err && <div class="login-err">{err}</div>}
          </>
        )}

        {step === "joining" && (
          <div class="login-pending" role="status">
            <div class="login-mode">{TEXT.setup_joining.replace("%s", joinSsid)}</div>
            <div class="login-left-row">
              <span class="login-pulse" aria-hidden="true" />
            </div>
            <p class="setup-copy">{TEXT.setup_wait}</p>
            {/* The fallback that must already be on screen if this page's own network vanishes
                (the iOS captive sheet closes with it, and no script can follow). No network-gone
                warning box anymore (owner request, September 26 2026) - the probes carry the happy
                path and this address carries the rest. */}
            <div class="setup-handoff">
              <p class="setup-copy">{TEXT.setup_wait_fallback}</p>
              <div class="setup-url">{handoffUrl}</div>
              {joinState === "slow" && (
                <>
                  <p class="setup-copy warn">{TEXT.setup_slow}</p>
                  <button class="btn sm" onClick={() => setStep("network")}>
                    {TEXT.setup_back}
                  </button>
                </>
              )}
            </div>
          </div>
        )}

        {/* No longer a wizard step: reached only through the HA connect step's Change link, and
            kept for the day the Basestation ships a second live card. */}
        {step === "mode" && (
          <>
            <div class="setup-head">{TEXT.setup_mode_title}</div>
            <div class="login-hint">{TEXT.setup_mode_sub}</div>
            <div class="setup-modes">
              {MODES.map((m) => (
                <button
                  key={m.id}
                  class={`mode-card${m.live ? " sel" : ""}`}
                  disabled={!m.live || busy}
                  onClick={m.live ? chooseHa : undefined}
                >
                  <span class="mode-name">
                    {m.name()}
                    {m.live ? (
                      <span class="vtag lit">{TEXT.setup_mode_selected}</span>
                    ) : (
                      <span class="vtag">{TEXT.setup_mode_soon}</span>
                    )}
                  </span>
                  <span class="mode-sub">{m.sub()}</span>
                </button>
              ))}
            </div>
            {err && <div class="login-err">{err}</div>}
          </>
        )}

        {step === "haconnect" && (
          <>
            <div class="setup-head">{TEXT.setup_hac_title}</div>
            {/* The chooser's replacement: the mode as a stated fact with a way to change it, not a
                one-option question (owner decision, September 26 2026). */}
            <div class="setup-mode-line">
              {TEXT.setup_hac_mode_label} <b>{TEXT.setup_mode_ha}</b>
              <button class="linkish" onClick={() => setStep("mode")}>
                {TEXT.setup_hac_change}
              </button>
            </div>
            <BoldedCopy segments={TEXT.setup_hac_copy} />
            {discoveredMock}
            <a
              class="btn solid setup-hac-open"
              href={TEXT.setup_hac_web_url}
              target="_blank"
              rel="noreferrer"
              onClick={(e) => openHomeAssistant(e, TEXT.setup_hac_app_url)}
            >
              {TEXT.setup_hac_open}
            </a>
            {/* The web path as a visible choice, never an automatic one - see lib/openha.js for
                the dialog race that retired the auto-fallback. */}
            <div class="login-hint">
              <a class="setup-alt-link" href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer">
                {TEXT.setup_hac_web}
              </a>
            </div>
            <div class="setup-wait-row" role="status">
              <span class="login-pulse" aria-hidden="true" />
              <span>{TEXT.setup_hac_wait}</span>
            </div>
            {err && <div class="login-err">{err}</div>}
          </>
        )}

        {/* The actions checkbox, moved into the wizard (owner request, September 26 2026): meeting
            it AFTER sign-in read as one more thing past the finish line, and an unticked box is
            also why a first VoiceTap fell back to the wake-word challenge. The steps are the
            splash blocked card's own (cogStep keeps the surfaces identical); the wizard only knows
            the firmware name, so step 2 wears the "unless you renamed it" hedge. Skip never traps
            - the post-login splash card remains the fallback surface. */}
        {step === "haactions" && (
          <>
            <div class="setup-head">{TEXT.setup_act_title}</div>
            <p class="setup-copy">{TEXT.setup_act_intro}</p>
            <ol class="fix-steps">
              <li>{TEXT.blocked_step1}</li>
              <li>{cogStep(TEXT.blocked_step2_unnamed, fn || "Satellite1")}</li>
              <li>{TEXT.blocked_step3}</li>
            </ol>
            <a
              class="btn solid setup-hac-open"
              href={TEXT.setup_hac_web_url}
              target="_blank"
              rel="noreferrer"
              onClick={(e) => openHomeAssistant(e, TEXT.setup_hac_app_url)}
            >
              {TEXT.setup_hac_open}
            </a>
            <div class="login-hint">
              <a class="setup-alt-link" href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer">
                {TEXT.setup_hac_web}
              </a>
            </div>
            <div class="setup-wait-row" role="status">
              <span class="login-pulse" aria-hidden="true" />
              <span>{TEXT.setup_act_wait}</span>
            </div>
            <button class="btn ghost sm" onClick={finish}>
              {TEXT.setup_act_skip}
            </button>
          </>
        )}
      </div>

      {/* The same identity caption the login screen wears: which device this wizard belongs to. */}
      {host && <div class="login-dev">{host}</div>}
    </div>
  );
}
