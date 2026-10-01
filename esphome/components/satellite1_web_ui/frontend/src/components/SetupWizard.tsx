import { useEffect, useRef, useState } from 'react';
// The real ESPHome mark (owner-supplied PNG, September 25 2026) dresses the Discovered card, a live
// recreation of Home Assistant's own that wears this device's real name: the person is told to find
// exactly this card, and an approximation would defeat that.
import ESPHOME_LOGO from '../../assets/esphome-logo.png';
import { TEXT } from '../copy.js';
import { openHomeAssistant } from '../lib/openha.js';
import { portalPass, probeSetup, setupMode, setupStatus, wifiJoin, wifiScan, wifiStatus } from '../lib/setup.js';
import { actionsOk, afterAdd, entryStep, joinProblem, joinRead, mergeScan, onHomeOrigin, probeOrigins, probeStreak } from '../lib/setup-flow.js';
import { Logo, cogStep, useTheme } from './bits';

/**
 * The onboarding wizard a factory-fresh device serves instead of the sign-in screen, mounted while
 * GET /api/sat1/setup/status says onboarding is pending; the device's session gate keeps the
 * endpoints it uses sessionless exactly as long as that is true.
 *
 * The flow is browser-first (owner design, September 26 2026). The OS captive-portal sheet that pops
 * when a phone joins the setup AP gets only the launcher, whose one job is to open the person's real
 * browser at this same address, because the sheet closes itself the moment the device leaves the
 * setup network and nothing served into it can outlive that. A browser tab survives the hop: it
 * picks the WiFi and is redirected to the device's home-network address once both are there. The
 * Home Assistant steps wait there for the add - the device completes onboarding itself when the API
 * attaches, with no password step (the generated password is published to Home Assistant, and
 * VoiceTap needs none) - then for the actions checkbox, before handing over to sign-in. Open Home
 * Assistant tries the companion app and keeps the web address as a visible link, never an automatic
 * fallback (src/lib/openha.js has the dialog race behind that). docs/web-ui-copy.md ("The onboarding
 * wizard") walks the whole flow.
 */
type WizStep = 'boot' | 'launcher' | 'network' | 'joining' | 'mode' | 'haconnect' | 'haactions';
const WIZ_ORDER: WizStep[] = ['launcher', 'network', 'joining', 'mode', 'haconnect', 'haactions'];
/** GET /api/sat1/setup/status. */
type SetupStatus = {
  setup: number;
  mode: number;
  wizard: number;
  ha: number;
  actions: number;
  name: string;
  fn: string;
} | null;
/** One row of GET /api/sat1/wifi/scan. */
type Ap = {
  ssid: string;
  rssi: number;
  sec: number;
  conn: number;
};
/** How long the reloaded launcher holds before offering its button: the sheet takes a few seconds
 *  to act on the reload's re-probe, and a tap inside that window opens in the sheet. */
const LAUNCH_HOLD_S = 5;
const BAR_IDX = [{
  id: 'b0',
  i: 0
}, {
  id: 'b1',
  i: 1
}, {
  id: 'b2',
  i: 2
}, {
  id: 'b3',
  i: 3
}];
/** Signal strength as 0-4 bars from dBm, on the usual thresholds; advisory either way. */
const barsOf = (rssi: number) => rssi >= -55 ? 4 : rssi >= -66 ? 3 : rssi >= -77 ? 2 : rssi >= -88 ? 1 : 0;
const Bars = ({
  rssi
}: {
  rssi: number;
}) => {
  const n = barsOf(rssi);
  return <svg className="wifi-bars" viewBox="0 0 16 14" aria-hidden="true">
      {BAR_IDX.map(_mpRecord => {
      const {
        id,
        i
      } = _mpRecord;
      return <rect key={id} x={i * 4} y={11 - i * 3} width="2.6" height={3 + i * 3} rx="1" opacity={i < n ? 1 : 0.25} />;
    })}
    </svg>;
};
const LockIcon = () => <svg className="wifi-lock" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.3" aria-hidden="true">
    <rect x="2.4" y="5.2" width="7.2" height="5" rx="1.2" />
    <path d="M4 5V3.6a2 2 0 0 1 4 0V5" />
  </svg>;

/** The Home Assistant steps wear their own addresses (owner request), by replaceState rather than a
 *  push so the captive sheet's back gesture is not trapped. */
const wearHash = (hash: string) => {
  try {
    history.replaceState(null, '', location.pathname + location.search + hash);
  } catch {
    /* A webview that refuses replaceState keeps the old hash; the steps work regardless. */
  }
};

/** Android's captive sheet honours a browser intent. Everywhere else the plain absolute link is
 *  what the sheet, once satisfied, hands to the real browser. */
const launchAndroid = (e: MouseEvent) => {
  if (!/android/i.test(navigator.userAgent)) return;
  e.preventDefault();
  location.href = `intent://${location.host}/?setup=go#Intent;scheme=http;action=android.intent.action.VIEW;end`;
};

/** Typed into a type="text" field that CSS masks, not type="password": a WiFi key is not an account
 *  credential, and password managers key on the input type - their save-this and strong-password
 *  sheets pounced mid-onboarding (owner report, September 26 2026: Bitwarden's covered the Join
 *  button). The vendor data-* attributes tell the major managers to stand down besides. The
 *  network-name field wears them too, because a name-plus-secret pair is exactly the shape managers
 *  read as a login form. */
const NO_MANAGERS = {
  autoComplete: 'off',
  autoCorrect: 'off',
  autoCapitalize: 'none',
  spellcheck: false,
  'data-1p-ignore': true,
  'data-lpignore': 'true',
  'data-bwignore': true,
  'data-form-type': 'other'
};

export function SetupWizard({
  status,
  onDone
}: {
  /** GET /api/sat1/setup/status as App read it at boot. */
  status: SetupStatus;
  onDone: () => void;
}) {
  const [theme] = useTheme();
  const onDoneRef = useRef(onDone);
  useEffect(() => {
    onDoneRef.current = onDone;
  }, [onDone]);
  const live = useRef(true);
  useEffect(() => () => {
    live.current = false;
  }, []);
  const [step, setStep] = useState<WizStep>('boot');
  // The launcher's two halves: false while it primes its way out of the sheet, true once the
  // priming reload (?setup=prime) has landed and `hold` is counting down to the button.
  const [launched, setLaunched] = useState(false);
  const [hold, setHold] = useState(LAUNCH_HOLD_S);
  const [aps, setAps] = useState<Ap[] | null>(null);
  const [open, setOpen] = useState<string | null>(null);
  const [pw, setPw] = useState('');
  const [err, setErr] = useState('');
  const [busy, setBusy] = useState(false);
  const [manual, setManual] = useState(false);
  const [mSsid, setMSsid] = useState('');
  const [ssid, setSsid] = useState('');
  const [slow, setSlow] = useState(false);
  // The device's mDNS name and display name, refreshed by every read that carries them.
  const [host, setHost] = useState(status?.name || '');
  const [fn, setFn] = useState(status?.fn || '');
  const [modeSet, setModeSet] = useState(status?.mode === 1);
  const [checking, setChecking] = useState(false);
  const [notYet, setNotYet] = useState(false);
  // One navigation, however many probes confirm the way is clear.
  const redirected = useRef(false);
  // The station IP, caught during the brief AP+STA overlap after the join. Android browsers cannot
  // resolve .local, so the redirect probes this address too.
  const staIp = useRef<string | null>(null);
  const homeOrigin = host ? `http://${host}.local` : '';

  useEffect(() => {
    (async () => {
      const st: any = await wifiStatus();
      if (!live.current) return;
      if (st?.host) setHost(st.host);
      const entry = entryStep(st, location.search, navigator.userAgent);
      setLaunched(entry.launched);
      setStep(entry.step as WizStep);
    })();
  }, []);

  // The way out of the captive sheet. iOS opens other apps' URL schemes from the sheet but never the
  // browser's own (x-safari-* included; confirmed on hardware September 26 2026 and in Apple's
  // developer forums), so the launcher rides what production captive portals do (Cisco Spaces
  // documents it as their iOS flow), in an order proven on hardware the same day. Entering it fresh
  // opens the pass window (portalPass: the device answers the OS connectivity probes "online"),
  // then navigates through ?setup=prime - a real navigation is what makes the sheet re-check, and
  // background fetches do not count. The sheet flips to its connected state (Cancel becomes Done),
  // from which a tapped absolute link opens in the real browser, possibly underneath the sheet
  // until Done is tapped. replace() keeps the sheet's back gesture off a page whose only job was to
  // leave.
  useEffect(() => {
    if (step !== 'launcher' || launched) return;
    (async () => {
      await portalPass();
      if (!live.current) return;
      location.replace(`http://${location.host}/?setup=prime`);
    })();
  }, [step, launched]);
  useEffect(() => {
    if (step !== 'launcher' || !launched) return;
    setHold(LAUNCH_HOLD_S);
    let n = LAUNCH_HOLD_S;
    const t = setInterval(() => {
      // Probe-shaped nudges for the sheet's re-check; only navigations are guaranteed to count.
      fetch(`http://captive.apple.com/hotspot-detect.html?t=${Date.now()}`, {
        mode: 'no-cors',
        cache: 'no-store'
      }).catch(() => {});
      n -= 1;
      setHold(n);
      if (n <= 0) clearInterval(t);
    }, 1000);
    return () => clearInterval(t);
  }, [step, launched]);

  // The first read asks for a rescan, whose results land on a later read: the device answers
  // before it hops channels to scan.
  useEffect(() => {
    if (step !== 'network') return;
    let asked = false;
    const load = async () => {
      const list = await wifiScan(!asked);
      asked = true;
      if (live.current) setAps(prev => mergeScan(prev, list));
    };
    load();
    const t = setInterval(load, 4000);
    return () => clearInterval(t);
  }, [step]);

  // The joining heartbeat, and the redirect that carries the person across the network gap. Two
  // rules come from hardware (September 26 2026), where a redirect landed on Safari's "not connected
  // to the internet" page and stranded the person, because the page had already left itself and
  // nothing could retry: probing arms only once the join is real (joinRead), and the navigation
  // waits for two consecutive answers from one origin 1.25s apart (probeStreak), so it rides a path
  // that has settled. Probes fire unawaited, so a hung .local lookup cannot delay the IP probe that
  // answers on Android. A page whose network vanishes instead (the captive sheet closes with the AP,
  // and no script can follow) already has the fallback address on screen, which is why there is no
  // network-gone warning (owner request, September 26 2026).
  useEffect(() => {
    if (step !== 'joining') return;
    const start = Date.now();
    const hit = probeStreak();
    let armed = false;
    const tick = async () => {
      const st: any = await wifiStatus();
      if (!live.current) return;
      if (st?.host) setHost(st.host);
      const read = joinRead(st, Date.now() - start);
      if (read === 'slow') setSlow(true);
      if (read === 'connected' || read === 'gone') armed = true;
      if (read === 'connected' && st.ip) staIp.current = st.ip;
      if (onHomeOrigin(location.hostname, host)) {
        if (read === 'connected') setStep('haconnect');
        return;
      }
      if (!armed || redirected.current) return;
      for (const origin of probeOrigins(host, staIp.current)) {
        probeSetup(origin).then(there => {
          if (hit(origin, !!there) && !redirected.current) {
            redirected.current = true;
            location.href = `${origin}/`;
          }
        });
      }
    };
    tick();
    const t = setInterval(tick, 1250);
    return () => clearInterval(t);
  }, [step, host]);

  const finish = () => {
    wearHash('');
    onDoneRef.current();
  };
  // A setup/status read on the connect step; true once it has moved the wizard on.
  const settle = (st: any) => {
    if (!st) return false;
    if (st.fn) setFn(st.fn);
    if (st.name) setHost(st.name);
    const next = afterAdd(st);
    if (next === 'done') finish();
    if (next === 'haactions') setStep('haactions');
    return !!next;
  };

  // The connect step records the mode itself, because the chooser is not a step (owner decision,
  // September 26 2026: one live option was ceremony, so the mode shows as a stated fact with a
  // Change link) and the device needs mode=ha to complete onboarding when the API attaches.
  // Idempotent: a re-record answers ok, and done (already onboarded) means the poll is about to
  // hand over anyway.
  useEffect(() => {
    if (step !== 'haconnect' || modeSet) return;
    setupMode('ha').then((r: any) => {
      if (live.current && (r.ok || r.done)) setModeSet(true);
    }).catch(() => {});
  }, [step, modeSet]);
  useEffect(() => {
    if (step !== 'haconnect') return;
    wearHash('#/home-assistant-connect');
    setNotYet(false);
    const t = setInterval(async () => {
      const st = await setupStatus();
      if (live.current && settle(st)) clearInterval(t);
    }, 2000);
    return () => clearInterval(t);
  }, [step]);
  // The actions step (owner request, September 26 2026): meeting the checkbox after sign-in read as
  // one more thing past the finish line, and an unticked box is why a first VoiceTap fell back to
  // the wake-word challenge. Its steps are the post-login blocked card's own through the shared
  // cogStep; the wizard knows only the firmware name, so step 2 wears the "unless you renamed it"
  // hedge, and Skip never traps because that card remains the fallback. Ticking the checkbox
  // reloads the config entry, the device's probe re-fires, and the verdict flips within about a
  // second.
  useEffect(() => {
    if (step !== 'haactions') return;
    wearHash('#/home-assistant-actions');
    const t = setInterval(async () => {
      const st: any = await setupStatus();
      if (!live.current || !st) return;
      if (st.fn) setFn(st.fn);
      if (actionsOk(st.actions)) {
        clearInterval(t);
        finish();
      }
    }, 2000);
    return () => clearInterval(t);
  }, [step]);

  const join = async (name: string, secured: boolean) => {
    const problem = joinProblem(name, pw, secured);
    if (problem) {
      setErr(problem === 'ssid' ? 'Enter a network name.' : TEXT.setup_join_short);
      return;
    }
    setBusy(true);
    setErr('');
    try {
      const r: any = await wifiJoin(name, pw);
      if (r.ok) {
        if (r.host) setHost(r.host);
        setSsid(name);
        setSlow(false);
        redirected.current = false;
        setPw('');
        setStep('joining');
        return;
      }
      setErr(r.invalid ? TEXT.setup_join_short : TEXT.setup_join_failed);
    } catch {
      setErr(TEXT.setup_join_failed);
    } finally {
      setBusy(false);
    }
  };
  // The Change chooser's pick, and its way back: Home Assistant is the only live card, so choosing
  // is confirming.
  const chooseHa = async () => {
    setBusy(true);
    setErr('');
    try {
      const r: any = await setupMode('ha');
      if (!r.ok && !r.done) {
        setErr(TEXT.setup_mode_failed);
        return;
      }
      setModeSet(true);
      setStep('haconnect');
    } catch {
      setErr(TEXT.setup_mode_failed);
    } finally {
      setBusy(false);
    }
  };
  const checkAdded = async () => {
    setChecking(true);
    setNotYet(false);
    const st = await setupStatus();
    if (!live.current) return;
    setChecking(false);
    if (!settle(st)) setNotYet(true);
  };
  const toggleRow = (name: string) => {
    setOpen(open === name ? null : name);
    setPw('');
    setErr('');
    setManual(false);
  };
  const pwField = (focus: boolean) => <input type="text" className="wifi-mask" name="wifi-key" aria-label={TEXT.setup_wifi_pw_placeholder} placeholder={TEXT.setup_wifi_pw_placeholder} {...NO_MANAGERS} autoFocus={focus} value={pw} onInput={e => setPw((e.target as HTMLInputElement).value)} />;
  const idx = WIZ_ORDER.indexOf(step) + 1;
  return <main className="app setup-fullscreen" data-theme={theme}><section className="control setup wiz">
      {idx > 0 && <span className="eyebrow">SETUP · {String(idx).padStart(2, '0')} / 06</span>}
      <Logo cls="wiz-logo" />
      <h1 className="wiz-h">{TEXT.setup_title}</h1>
      <div className="wiz-glass">
        {step === 'boot' && <div className="wiz-wait" role="status"><span className="wiz-pulse" /><span>{TEXT.setup_scanning}</span></div>}
        {step === 'launcher' && <div className="wiz-center">
            <p className="wiz-p">{TEXT.setup_launch_copy}</p>
            {!launched ? <div className="wiz-wait" role="status"><span className="wiz-pulse" /><span>{TEXT.setup_launch_prep}</span></div> : hold > 0 ? <div className="wiz-wait" role="status"><span className="wiz-count" aria-hidden="true">{hold}</span><span>Almost there…</span></div> : <>
                <a className="primary wide wiz-btn" href={`http://${location.host}/?setup=go`} onClick={launchAndroid}>{TEXT.setup_launch_btn}</a>
                <p className="wiz-hint">{TEXT.setup_launch_retry}</p>
              </>}
            <p className="wiz-hint">{TEXT.setup_launch_fallback} <code>{`http://${location.host}`}</code></p>
            <button className="text-button" onClick={() => setStep('network')}>{TEXT.setup_launch_here}</button>
          </div>}
        {step === 'network' && <div>
            <h1 className="wiz-h">{TEXT.setup_pick}</h1>
            <p className="wiz-hint">{TEXT.setup_pick_hint}</p>
            {aps === null ? <div className="wiz-wait" role="status"><span className="wiz-pulse" /><span>{TEXT.setup_scanning}</span></div> : aps.length === 0 ? <p className="wiz-hint">{TEXT.setup_no_networks}</p> : <ul className="wiz-nets">
              {aps.map(n => <li key={n.ssid} className={open === n.ssid ? 'open' : ''}>
                  <button className="wiz-net" aria-expanded={open === n.ssid} onClick={() => toggleRow(n.ssid)}>
                    <Bars rssi={n.rssi} /><span className="wiz-ssid">{n.ssid}</span>{n.conn === 1 && <em className="wiz-badge on">{TEXT.setup_row_connected}</em>}{n.sec ? <LockIcon /> : null}
                  </button>
                  {open === n.ssid && <form className="wiz-form" onSubmit={e => {
              e.preventDefault();
              join(n.ssid, n.sec === 1);
            }}>
                      {n.sec ? pwField(true) : <p className="wiz-hint">{TEXT.setup_wifi_pw_open}</p>}
                      {err && <p className="error" role="alert">{err}</p>}
                      <button className="primary wide" type="submit" disabled={busy}>{TEXT.setup_join}</button>
                    </form>}
                </li>)}
            </ul>}
            <div className="wiz-row">
              <button className="text-button" aria-expanded={manual} onClick={() => {
            setManual(!manual);
            setOpen(null);
            setPw('');
            setErr('');
          }}>{TEXT.setup_other_network}</button>
              <button className="text-button" onClick={() => wifiScan(true)}>{TEXT.setup_rescan}</button>
            </div>
            {manual && <form className="wiz-form" onSubmit={e => {
          e.preventDefault();
          join(mSsid, true);
        }}>
                <input type="text" name="ssid" aria-label={TEXT.setup_ssid_placeholder} placeholder={TEXT.setup_ssid_placeholder} {...NO_MANAGERS} autoFocus value={mSsid} onInput={e => setMSsid((e.target as HTMLInputElement).value)} />
                {pwField(false)}
                {err && <p className="error" role="alert">{err}</p>}
                <button className="primary wide" type="submit" disabled={busy}>{TEXT.setup_join}</button>
              </form>}
          </div>}
        {step === 'joining' && <div className="wiz-center" role="status">
            <h1 className="wiz-h">{TEXT.setup_joining.replace('%s', ssid)}</h1>
            <span className="wiz-pulse lg" />
            <p className="wiz-p">{TEXT.setup_wait}</p>
            {homeOrigin && <p className="wiz-hint">{TEXT.setup_wait_fallback} <code>{homeOrigin}</code></p>}
            {slow && <div className="wiz-slow"><p>{TEXT.setup_slow}</p><button className="secondary" onClick={() => setStep('network')}>{TEXT.setup_back}</button></div>}
          </div>}
        {step === 'mode' && <div>
            <h1 className="wiz-h">{TEXT.setup_mode_title}</h1>
            <p className="wiz-hint">{TEXT.setup_mode_sub}</p>
            <button className="wiz-mode on" disabled={busy} onClick={chooseHa}><span><b>{TEXT.setup_mode_ha}</b><small>{TEXT.setup_mode_ha_sub}</small></span><em className="wiz-badge on">{TEXT.setup_mode_selected}</em></button>
            {err && <p className="error" role="alert">{err}</p>}
          </div>}
        {step === 'haconnect' && <div>
            <h1 className="wiz-h">{TEXT.setup_hac_title}</h1>
            <p className="wiz-hint wiz-instruction"><span>{TEXT.setup_hac_mode_label} <b>{TEXT.setup_mode_ha}</b> · </span><button className="text-button wiz-inline" onClick={() => setStep('mode')}>{TEXT.setup_hac_change}</button></p>
            <p className="wiz-p">{TEXT.setup_hac_copy.map((s: string, i: number) => i % 2 ? <strong key={i}>{s}</strong> : <span key={i}>{s}</span>)}</p>
            <div className="wiz-disc" aria-hidden="true">
              <div className="wiz-disc-header">
                <span className="wiz-disc-title">{TEXT.setup_hac_disc}</span>
              </div>
              <div className="wiz-disc-card">
                <button className="wiz-disc-dots" tabIndex={-1}>···</button>
                <img className="wiz-disc-logo" src={ESPHOME_LOGO} alt="" />
                <span className="wiz-disc-name">{fn || 'Satellite1'} ({host || 'satellite1'})</span>
                <span className="wiz-disc-int">ESPHome</span>
                <div className="wiz-disc-actions">
                  <button className="wiz-disc-ignore" tabIndex={-1}>{TEXT.setup_hac_ignore}</button>
                  <button className="wiz-disc-add" tabIndex={-1}>{TEXT.setup_hac_add}</button>
                </div>
              </div>
            </div>
            <a href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer" className="primary wide wiz-btn" onClick={e => openHomeAssistant(e, TEXT.setup_hac_app_url)}>{TEXT.setup_hac_open}</a>
            <button className="secondary wide" disabled={checking} onClick={checkAdded}>I've added it in Home Assistant</button>
            {notYet && <p className="wiz-hint wiz-not-yet" role="status">Your Satellite1 hasn't heard from Home Assistant yet. Finish adding it there - this page continues on its own.</p>}
            <a href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer" className="link wiz-center-link">{TEXT.setup_hac_web}</a>
            <div className="wiz-wait" role="status"><span className="wiz-pulse" /><span>{TEXT.setup_hac_wait}</span></div>
          </div>}
        {step === 'haactions' && <div>
            <h1 className="wiz-h">{TEXT.setup_act_title}</h1>
            <p className="wiz-p">{TEXT.setup_act_intro}</p>
            <ol className="wiz-steps">
              <li><span className="wiz-step-body">{TEXT.blocked_step1}</span></li>
              <li><span className="wiz-step-body">{cogStep(TEXT.blocked_step2_unnamed, fn || 'Satellite1')}</span></li>
              <li><span className="wiz-step-body">{TEXT.blocked_step3}</span></li>
            </ol>
            <a href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer" className="primary wide wiz-btn" onClick={e => openHomeAssistant(e, TEXT.setup_hac_app_url)}>{TEXT.setup_hac_open}</a>
            <a href={TEXT.setup_hac_web_url} target="_blank" rel="noreferrer" className="link wiz-center-link">{TEXT.setup_hac_web}</a>
            <div className="wiz-wait" role="status"><span className="wiz-pulse" /><span>{TEXT.setup_act_wait}</span></div>
            <button className="text-button wiz-center-link" onClick={finish}>{TEXT.setup_act_skip}</button>
          </div>}
      </div>
      {host && <div className="login-dev">{host}</div>}
    </section></main>;
}
