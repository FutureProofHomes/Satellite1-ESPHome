/**
 * The onboarding wizard: what a factory-fresh device serves instead of the login screen.
 * Ported from setup.jsx - the captive-sheet launcher, the WiFi network list, the joining
 * hand-over, the Home Assistant connect step with its live Discovered mock, and the actions
 * checkbox walk-through. The device polls are simulated: joining "connects" a few seconds in,
 * and Home Assistant "adds the device" while the connect step waits.
 */
import React, { useEffect, useRef, useState } from 'react';
import { cogStep, Logo } from './ui';
const TEXT = {
  title: 'Set up your Satellite1',
  launch_copy: 'Your Satellite1 is ready to meet your home.',
  launch_prep: 'Getting your setup ready\u2026',
  launch_btn: 'Setup Satellite1',
  launch_retry: 'If this page opens here again instead of your browser, wait for Done to appear in the corner and tap the button once more.',
  launch_fallback: 'You can also open your browser and go to',
  launch_here: 'Continue here instead',
  pick: 'Choose your WiFi network',
  pick_hint: '2.4 GHz networks only - if your WiFi has separate names for 2.4 and 5 GHz, pick the 2.4 GHz one.',
  rescan: 'Scan again',
  scanning: 'Looking for networks\u2026',
  other_network: 'Join another network\u2026',
  ssid_placeholder: 'Network name',
  wifi_pw_placeholder: 'WiFi password',
  wifi_pw_open: 'This network has no password.',
  join: 'Join',
  join_short: 'WiFi passwords are at least 8 characters.',
  joining: 'Connecting to %s\u2026',
  wait: "Please wait while your Satellite1 connects to your network. You'll be redirected to finish setting up.",
  wait_fallback: 'If nothing happens after it connects, join your home WiFi and open',
  slow: 'Still trying. If this takes much longer, the password may have been wrong - go back and re-enter it.',
  back: 'Back',
  mode_title: 'How will your Satellite1 connect?',
  mode_sub: 'More ways to connect are on the way.',
  mode_nexus: 'Nexus AI Basestation',
  mode_nexus_sub: 'Connect to your 100% private Nexus AI Basestation',
  mode_ha: 'Home Assistant',
  mode_ha_sub: 'Connect to your Home Assistant server',
  mode_soon: 'Coming soon',
  mode_selected: 'Selected',
  hac_title: 'Connect to Home Assistant',
  hac_mode_label: 'Connect mode:',
  hac_change: 'Change',
  hac_copy: ['In your Home Assistant, go to ', 'Settings \u2192 Devices & Services', '. Your Satellite1 is waiting under ', 'Discovered', ' \u2014 tap ', 'Add', ' and follow the steps.'],
  hac_disc: 'Discovered',
  hac_ignore: 'Ignore',
  hac_add: 'Add',
  hac_open: 'Open Home Assistant',
  hac_web: 'No Home Assistant app? Open it in your browser instead.',
  hac_wait: 'Waiting for Home Assistant\u2026 this page continues on its own once your Satellite1 is added.',
  act_title: 'One last Home Assistant setting',
  act_intro: 'This setting lets your Satellite1 speak announcements and route audio through Home Assistant.',
  act_wait: "Waiting for the setting\u2026 this page continues on its own once it's allowed.",
  act_skip: 'Skip for now',
  step1: 'In Home Assistant, open Settings \u203A Devices & services \u203A ESPHome.',
  step2: 'Tap the %c cog next to this device - %s, unless you renamed it.',
  step3: 'Tick \u201CAllow the device to perform Home Assistant actions\u201D, then Submit.',
  row_connected: 'Connected'
};
const HOST = 'satellite1-a4c2f8';
const FN = 'Satellite1 A4C2F8';
const NETWORKS = [{
  ssid: 'Davis Home',
  rssi: -48,
  sec: 1
}, {
  ssid: 'Davis Home Guest',
  rssi: -52,
  sec: 1
}, {
  ssid: 'HP-Print-A7',
  rssi: -71,
  sec: 0
}, {
  ssid: 'NETGEAR-2G',
  rssi: -84,
  sec: 1
}];
const barsOf = (rssi: number) => rssi >= -55 ? 4 : rssi >= -66 ? 3 : rssi >= -77 ? 2 : rssi >= -88 ? 1 : 0;
const Bars = ({
  rssi
}: {
  rssi: number;
}) => {
  const n = barsOf(rssi);
  return <svg className="wifi-bars" viewBox="0 0 16 14" aria-hidden="true">
      {[0, 1, 2, 3].map(i => <rect key={i} x={i * 4} y={11 - i * 3} width="2.6" height={3 + i * 3} rx="1" opacity={i < n ? 1 : 0.25} />)}
    </svg>;
};
const LockIcon = () => <svg className="wifi-lock" viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.3" aria-hidden="true">
    <rect x="2.4" y="5.2" width="7.2" height="5" rx="1.2" />
    <path d="M4 5V3.6a2 2 0 0 1 4 0V5" />
  </svg>;

/** The alternating plain/bold copy the HA connect instructions use. */
const BoldedCopy = ({
  segments
}: {
  segments: string[];
}) => <p className="setup-copy">{segments.map((s, i) => i % 2 ? <b key={i}>{s}</b> : s)}</p>;
type Step = 'launcher' | 'network' | 'joining' | 'mode' | 'haconnect' | 'haactions';
export function SetupWizard({
  onDone
}: {
  onDone: () => void;
}) {
  const [step, setStep] = useState<Step>('launcher');
  const [launched, setLaunched] = useState(false);
  const [hold, setHold] = useState(3);
  const [picked, setPicked] = useState<{
    ssid: string;
    sec: boolean;
  } | null>(null);
  const [manual, setManual] = useState(false);
  const [ssidInput, setSsidInput] = useState('');
  const [wifiPw, setWifiPw] = useState('');
  const [joinSsid, setJoinSsid] = useState('');
  // The join that never connects: typing the password "wrong" (literally) stalls the connect,
  // which is how the preview reaches the ~45s wrong-password state the real wizard shows.
  const [stalled, setStalled] = useState(false);
  const [joinState, setJoinState] = useState<'trying' | 'slow'>('trying');
  const [err, setErr] = useState<string | null>(null);
  const live = useRef(true);
  useEffect(() => () => {
    live.current = false;
  }, []);

  /* The launcher's automatic half: the priming beat the captive sheet needs before the button
     is worth tapping. */
  useEffect(() => {
    if (step !== 'launcher' || launched) return;
    const t = setTimeout(() => live.current && setLaunched(true), 1200);
    return () => clearTimeout(t);
  }, [step, launched]);
  useEffect(() => {
    if (step !== 'launcher' || !launched) return;
    setHold(3);
    let n = 3;
    const t = setInterval(() => {
      n -= 1;
      if (!live.current) return clearInterval(t);
      setHold(n);
      if (n <= 0) clearInterval(t);
    }, 1000);
    return () => clearInterval(t);
  }, [step, launched]);

  /* The joining step: the device "connects" and the redirect lands on the HA connect step - or,
     on a stalled join, the honest read after a while is a wrong password, and the wizard is the
     one to say it (the device itself never gives up). */
  useEffect(() => {
    if (step !== 'joining') return;
    const t = setTimeout(() => {
      if (!live.current) return;
      if (stalled) setJoinState('slow');else setStep('haconnect');
    }, stalled ? 7000 : 6000);
    return () => clearTimeout(t);
  }, [step, stalled]);

  /* The HA connect step: Home Assistant "adds the device" while this waits, then the actions
     walk-through - the blocked-checkbox path, so the whole flow shows. */
  useEffect(() => {
    if (step !== 'haconnect') return;
    const t = setTimeout(() => live.current && setStep('haactions'), 12000);
    return () => clearTimeout(t);
  }, [step]);

  /* The actions step advances on its own too - ticking the box in HA flips the verdict. */
  useEffect(() => {
    if (step !== 'haactions') return;
    const t = setTimeout(() => live.current && onDone(), 12000);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step]);
  const join = (ssid: string, sec: boolean) => {
    if (sec && wifiPw.length > 0 && wifiPw.length < 8) {
      setErr(TEXT.join_short);
      return;
    }
    setErr(null);
    setJoinSsid(ssid);
    // Any password containing "wrong" stalls (the literal word is under the 8-char floor).
    setStalled(sec && wifiPw.toLowerCase().includes('wrong'));
    setJoinState('trying');
    setWifiPw('');
    setStep('joining');
  };
  const managerOff = {
    autoComplete: 'off',
    autoCorrect: 'off',
    autoCapitalize: 'none',
    spellCheck: false
  } as const;
  const networkRow = (ap: (typeof NETWORKS)[0]) => {
    const on = picked?.ssid === ap.ssid && !manual;
    return <div key={ap.ssid} className={`wifi-row${on ? ' on' : ''}`}>
        <button className="wifi-hit" onClick={() => {
        setManual(false);
        setErr(null);
        setWifiPw('');
        setPicked(on ? null : {
          ssid: ap.ssid,
          sec: ap.sec === 1
        });
      }}>
          
          <span className="wifi-name">{ap.ssid}</span>
          {ap.sec === 1 && <LockIcon />}
          <Bars rssi={ap.rssi} />
        </button>
        {on && <form className="wifi-join" onSubmit={e => {
        e.preventDefault();
        join(ap.ssid, ap.sec === 1);
      }}>
          
            {ap.sec === 1 ? <div className="login-field">
                <input type="text" className="wifi-mask" name="wifi-key" placeholder={TEXT.wifi_pw_placeholder} {...managerOff} value={wifiPw} onInput={e => setWifiPw((e.target as HTMLInputElement).value)} aria-label={TEXT.wifi_pw_placeholder} />
            
              </div> : <div className="login-hint">{TEXT.wifi_pw_open}</div>}
            <button className="btn solid" type="submit">
              {TEXT.join}
            </button>
          </form>}
      </div>;
  };

  /* The Discovered mock: Home Assistant's own card, recreated live so it carries this device's
     real name. Decorative throughout. */
  const discoveredMock = <div className="ha-wrap" aria-hidden="true">
      <p className="ha-disc">{TEXT.hac_disc}</p>
      <div className="ha-card">
        <span className="ha-kebab">{'\u22ee'}</span>
        <img className="ha-logo" src="https://storage.googleapis.com/storage.magicpath.ai/component-assets/454802455327313920/454809714925146112/94468b329d143721ef9e4b9ca3fd46d52a305d180b732daf6ac832eab469af45.png" alt="" />
        <div className="ha-name">
          {FN} ({HOST})
        </div>
        <div className="ha-sub">ESPHome</div>
        <div className="ha-btns">
          <span className="ha-ignore">{TEXT.hac_ignore}</span>
          <span className="ha-add">{TEXT.hac_add}</span>
        </div>
      </div>
    </div>;
  return <div className="login setup">
      <div className="login-glow" aria-hidden="true" />
      <div className="login-hero">
        <Logo />
        <h1 className="login-name">{TEXT.title}</h1>
      </div>

      <div className="card login-card">
        {step === 'launcher' && <div className="setup-launch">
            <p className="setup-copy">{TEXT.launch_copy}</p>
            {!launched || hold > 0 ? <div className="setup-wait-row" role="status">
                <span className="login-pulse" aria-hidden="true" />
                <span>{TEXT.launch_prep}</span>
              </div> : <>
                <button className="btn solid setup-launch-btn" onClick={() => setStep('network')}>
                  {TEXT.launch_btn}
                </button>
                <div className="login-hint">{TEXT.launch_retry}</div>
              </>}
            <div className="login-hint">
              {TEXT.launch_fallback} <b>http://192.168.4.1</b>
            </div>
            <button className="btn ghost sm" onClick={() => setStep('network')}>
              {TEXT.launch_here}
            </button>
          </div>}

        {step === 'network' && <>
            <div className="setup-head">{TEXT.pick}</div>
            <div className="login-hint setup-pick-hint">{TEXT.pick_hint}</div>
            <div className="wifi-list">{NETWORKS.map(networkRow)}</div>
            <button className="btn ghost sm setup-other" onClick={() => {
          setPicked(null);
          setErr(null);
          setManual(v => !v);
        }}>
            
              {TEXT.other_network}
            </button>
            {manual && <form className="wifi-join" onSubmit={e => {
          e.preventDefault();
          if (ssidInput) join(ssidInput, true);
        }}>
            
                <div className="login-field">
                  <input type="text" name="ssid" placeholder={TEXT.ssid_placeholder} {...managerOff} value={ssidInput} onInput={e => setSsidInput((e.target as HTMLInputElement).value)} aria-label={TEXT.ssid_placeholder} />
              
                </div>
                <div className="login-field">
                  <input type="text" className="wifi-mask" name="wifi-key" placeholder={TEXT.wifi_pw_placeholder} {...managerOff} value={wifiPw} onInput={e => setWifiPw((e.target as HTMLInputElement).value)} aria-label={TEXT.wifi_pw_placeholder} />
              
                </div>
                <button className="btn solid" type="submit" disabled={!ssidInput}>
                  {TEXT.join}
                </button>
              </form>}
            <button className="btn ghost sm">{TEXT.rescan}</button>
            {err && <div className="login-err">{err}</div>}
          </>}

        {step === 'joining' && <div className="login-pending" role="status">
            <div className="login-mode">{TEXT.joining.replace('%s', joinSsid)}</div>
            <div className="login-left-row">
              <span className="login-pulse" aria-hidden="true" />
            </div>
            <p className="setup-copy">{TEXT.wait}</p>
            <div className="setup-handoff">
              <p className="setup-copy">{TEXT.wait_fallback}</p>
              <div className="setup-url">{`http://${HOST}.local`}</div>
              {joinState === 'slow' && <>
                  <p className="setup-copy warn">{TEXT.slow}</p>
                  <button className="btn sm" onClick={() => setStep('network')}>
                    {TEXT.back}
                  </button>
                </>}
            </div>
          </div>}

        {step === 'mode' && <>
            <div className="setup-head">{TEXT.mode_title}</div>
            <div className="login-hint">{TEXT.mode_sub}</div>
            <div className="setup-modes">
              <button className="mode-card" disabled>
                <span className="mode-name">
                  {TEXT.mode_nexus}
                  <span className="vtag">{TEXT.mode_soon}</span>
                </span>
                <span className="mode-sub">{TEXT.mode_nexus_sub}</span>
              </button>
              <button className="mode-card sel" onClick={() => setStep('haconnect')}>
                <span className="mode-name">
                  {TEXT.mode_ha}
                  <span className="vtag lit">{TEXT.mode_selected}</span>
                </span>
                <span className="mode-sub">{TEXT.mode_ha_sub}</span>
              </button>
            </div>
          </>}

        {step === 'haconnect' && <>
            <div className="setup-head">{TEXT.hac_title}</div>
            <div className="setup-mode-line">
              {TEXT.hac_mode_label} <b>{TEXT.mode_ha}</b>
              <button className="linkish" onClick={() => setStep('mode')}>
                {TEXT.hac_change}
              </button>
            </div>
            <BoldedCopy segments={TEXT.hac_copy} />
            {discoveredMock}
            <a className="btn solid setup-hac-open" href="https://my.home-assistant.io/redirect/integrations/" target="_blank" rel="noreferrer" onClick={e => e.preventDefault()}>
            
              {TEXT.hac_open}
            </a>
            <div className="login-hint">
              <a className="setup-alt-link" href="https://my.home-assistant.io/redirect/integrations/" onClick={e => e.preventDefault()}>
                {TEXT.hac_web}
              </a>
            </div>
            <div className="setup-wait-row" role="status">
              <span className="login-pulse" aria-hidden="true" />
              <span>{TEXT.hac_wait}</span>
            </div>
          </>}

        {step === 'haactions' && <>
            <div className="setup-head">{TEXT.act_title}</div>
            <p className="setup-copy">{TEXT.act_intro}</p>
            <ol className="fix-steps">
              <li>{TEXT.step1}</li>
              <li>{cogStep(TEXT.step2, FN)}</li>
              <li>{TEXT.step3}</li>
            </ol>
            <a className="btn solid setup-hac-open" href="https://my.home-assistant.io/redirect/integrations/" onClick={e => e.preventDefault()}>
            
              {TEXT.hac_open}
            </a>
            <div className="setup-wait-row" role="status">
              <span className="login-pulse" aria-hidden="true" />
              <span>{TEXT.act_wait}</span>
            </div>
            <button className="btn ghost sm" onClick={onDone}>
              {TEXT.act_skip}
            </button>
          </>}
      </div>

      <div className="login-dev">{HOST}</div>
    </div>;
}