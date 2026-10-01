import { useEffect, useRef, useState } from 'react';
import ESPHOME_LOGO from '../../assets/esphome-logo.png';
import { Logo } from './bits';
type WizStep = 'launcher' | 'network' | 'joining' | 'mode' | 'haconnect' | 'haactions';
const WIZ_ORDER: WizStep[] = ['launcher', 'network', 'joining', 'mode', 'haconnect', 'haactions'];
const WIZ_NETWORKS = [{
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
const HAC_COPY = [{
  id: 'c0',
  t: 'In your Home Assistant, go to ',
  b: false
}, {
  id: 'c1',
  t: 'Settings → Devices & Services',
  b: true
}, {
  id: 'c2',
  t: '. Your Satellite1 is waiting under ',
  b: false
}, {
  id: 'c3',
  t: 'Discovered',
  b: true
}, {
  id: 'c4',
  t: ' — tap ',
  b: false
}, {
  id: 'c5',
  t: 'Add',
  b: true
}, {
  id: 'c6',
  t: ' and follow the steps.',
  b: false
}];
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
export function SetupWizard({
  status,
  onDone
}: {
  /** GET /api/sat1/setup as App read it at boot: what the device already has. */
  status: any;
  onDone: () => void;
}) {
  void status;
  const onDoneRef = useRef(onDone);
  useEffect(() => {
    onDoneRef.current = onDone;
  }, [onDone]);
  const [step, setStep] = useState<WizStep>('launcher');
  const [prep, setPrep] = useState(true);
  const [count, setCount] = useState(3);
  const [open, setOpen] = useState<string | null>(null);
  const [pw, setPw] = useState('');
  const [err, setErr] = useState('');
  const [manual, setManual] = useState(false);
  const [mSsid, setMSsid] = useState('');
  const [ssid, setSsid] = useState('');
  const [joinPw, setJoinPw] = useState('');
  const [slow, setSlow] = useState(false);
  useEffect(() => {
    if (step !== 'launcher') return;
    setPrep(true);
    setCount(3);
    const t = setTimeout(() => setPrep(false), 1200);
    return () => clearTimeout(t);
  }, [step]);
  useEffect(() => {
    if (step !== 'launcher' || prep || count <= 0) return;
    const t = setTimeout(() => setCount(c => c - 1), 1000);
    return () => clearTimeout(t);
  }, [step, prep, count]);
  useEffect(() => {
    if (step !== 'joining') return;
    setSlow(false);
    if (/wrong/i.test(joinPw)) {
      const t = setTimeout(() => setSlow(true), 7000);
      return () => clearTimeout(t);
    }
    const t = setTimeout(() => setStep('haconnect'), 6000);
    return () => clearTimeout(t);
  }, [step, joinPw]);
  useEffect(() => {
    if (step === 'haconnect') {
      const t = setTimeout(() => setStep('haactions'), 12000);
      return () => clearTimeout(t);
    }
    if (step === 'haactions') {
      const t = setTimeout(() => onDoneRef.current(), 12000);
      return () => clearTimeout(t);
    }
  }, [step]);
  const join = (name: string, secured: boolean) => {
    if (!name.trim()) {
      setErr('Enter a network name.');
      return;
    }
    if (secured && pw.length < 8) {
      setErr('WiFi passwords are at least 8 characters.');
      return;
    }
    setErr('');
    setSsid(name);
    setJoinPw(pw);
    setStep('joining');
  };
  const idx = WIZ_ORDER.indexOf(step) + 1;
  const openHA = (e: React.MouseEvent) => e.preventDefault();
  return <section className="control setup wiz">
      <span className="eyebrow">SETUP · {String(idx).padStart(2, '0')} / 06</span>
      <Logo cls="wiz-logo" />
      <h1 className="wiz-h">Set up your Satellite1</h1>
      <div className="wiz-glass">
        {step === 'launcher' && <div className="wiz-center">
            <p className="wiz-p">Your Satellite1 is ready to meet your home.</p>
            {prep ? <div className="wiz-wait"><span className="wiz-pulse" /><span>Getting your setup ready…</span></div> : count > 0 ? <div className="wiz-wait"><span className="wiz-count">{count}</span><span>Almost there…</span></div> : <button className="primary wide" onClick={() => setStep('network')}>Setup Satellite1</button>}
          </div>}
        {step === 'network' && <div>
            <h1 className="wiz-h">Choose your WiFi network</h1>
            <p className="wiz-hint">2.4 GHz networks only - if your WiFi has separate names for 2.4 and 5 GHz, pick the 2.4 GHz one.</p>
            <ul className="wiz-nets">
              {WIZ_NETWORKS.map(n => <li key={n.ssid} className={open === n.ssid ? 'open' : ''}>
                  <button className="wiz-net" onClick={() => {
              setOpen(open === n.ssid ? null : n.ssid);
              setPw('');
              setErr('');
              setManual(false);
            }}>
                    <Bars rssi={n.rssi} /><span className="wiz-ssid">{n.ssid}</span>{n.sec ? <LockIcon /> : null}
                  </button>
                  {open === n.ssid && <form className="wiz-form" onSubmit={e => {
              e.preventDefault();
              join(n.ssid, !!n.sec);
            }}>
                      {n.sec ? <input type="password" aria-label="WiFi password" placeholder="WiFi password" value={pw} onChange={e => setPw(e.target.value)} autoFocus /> : <p className="wiz-hint">This network has no password.</p>}
                      {err && <p className="error">{err}</p>}
                      <button className="primary wide" type="submit">Join</button>
                    </form>}
                </li>)}
            </ul>
            <div className="wiz-row">
              <button className="text-button" onClick={() => {
            setManual(!manual);
            setOpen(null);
            setPw('');
            setErr('');
          }}>Join another network…</button>
              <button className="text-button">Scan again</button>
            </div>
            {manual && <form className="wiz-form" onSubmit={e => {
          e.preventDefault();
          join(mSsid, true);
        }}>
                <input aria-label="Network name" placeholder="Network name" value={mSsid} onChange={e => setMSsid(e.target.value)} />
                <input type="password" aria-label="WiFi password" placeholder="WiFi password" value={pw} onChange={e => setPw(e.target.value)} />
                {err && <p className="error">{err}</p>}
                <button className="primary wide" type="submit">Join</button>
              </form>}
          </div>}
        {step === 'joining' && <div className="wiz-center">
            <h1 className="wiz-h">Connecting to {ssid}…</h1>
            <span className="wiz-pulse lg" />
            <p className="wiz-p">Please wait while your Satellite1 connects to your network. You'll be redirected to finish setting up.</p>
            <p className="wiz-hint">If nothing happens after it connects, join your home WiFi and open <code>http://satellite1-a4c2f8.local</code></p>
            {slow && <div className="wiz-slow"><p>Still trying. If this takes much longer, the password may have been wrong - go back and re-enter it.</p><button className="secondary" onClick={() => setStep('network')}>Back</button></div>}
          </div>}
        {step === 'mode' && <div>
            <h1 className="wiz-h">How will your Satellite1 connect?</h1>
            <p className="wiz-hint">More ways to connect are on the way.</p>
            <button className="wiz-mode" disabled><span><b>Nexus AI Basestation</b><small>Connect to your 100% private Nexus AI Basestation</small></span><em className="wiz-badge">Coming soon</em></button>
            <button className="wiz-mode on" onClick={() => setStep('haconnect')}><span><b>Home Assistant</b><small>Connect to your Home Assistant server</small></span><em className="wiz-badge on">Selected</em></button>
          </div>}
        {step === 'haconnect' && <div>
            <h1 className="wiz-h">Connect to Home Assistant</h1>
            <p className="wiz-hint wiz-instruction"><span>Connect mode: <b>Home Assistant</b> · </span><button className="text-button wiz-inline" onClick={() => setStep('mode')}>Change</button></p>
            <p className="wiz-p">{HAC_COPY.map(s => s.b ? <strong key={s.id}>{s.t}</strong> : <span key={s.id}>{s.t}</span>)}</p>
            <div className="wiz-disc">
              <div className="wiz-disc-header">
                <span className="wiz-disc-title">Discovered</span>
              </div>
              <div className="wiz-disc-card">
                <button className="wiz-disc-dots" aria-label="More options">···</button>
                <img className="wiz-disc-logo" src={ESPHOME_LOGO} alt="ESPHome" />
                <span className="wiz-disc-name">Satellite1 A4C2F8 (satellite1-a4c2f8)</span>
                <span className="wiz-disc-int">ESPHome</span>
                <div className="wiz-disc-actions">
                  <button className="wiz-disc-ignore">Ignore</button>
                  <button className="wiz-disc-add">Add</button>
                </div>
              </div>
            </div>
            <a href="homeassistant://navigate/config/integrations" className="primary wide wiz-btn" onClick={openHA}>Open Home Assistant</a>
            <button className="secondary wide" onClick={() => setStep('haactions')}>I've added it in Home Assistant</button>
            <a href="http://homeassistant.local:8123/config/integrations" className="link wiz-center-link" onClick={openHA}>No Home Assistant app? Open it in your browser instead.</a>
            <div className="wiz-wait"><span className="wiz-pulse" /><span>Waiting for Home Assistant… this page continues on its own once your Satellite1 is added.</span></div>
          </div>}
        {step === 'haactions' && <div>
            <h1 className="wiz-h">One last Home Assistant setting</h1>
            <p className="wiz-p">This setting lets your Satellite1 speak announcements and route audio through Home Assistant.</p>
            <ol className="wiz-steps">
              <li><span className="wiz-step-body"><span>In Home Assistant, open </span><strong>Settings {'\u203A'} Devices &amp; services {'\u203A'} ESPHome</strong><span>.</span></span></li>
              <li><span className="wiz-step-body"><span>Tap the </span><span className="glyph" aria-label="cog">{'\u2699'}</span><span> cog next to this device — </span><strong>Satellite1 A4C2F8</strong><span>, unless you renamed it.</span></span></li>
              <li><span className="wiz-step-body"><span>Tick </span><strong>{'\u201C'}Allow the device to perform Home Assistant actions{'\u201D'}</strong><span>, then Submit.</span></span></li>
            </ol>
            <a href="homeassistant://navigate/config/integrations/integration/esphome" className="primary wide wiz-btn" onClick={openHA}>Open Home Assistant</a>
            <div className="wiz-wait"><span className="wiz-pulse" /><span>Waiting for the setting… this page continues on its own once it's allowed.</span></div>
            <button className="text-button wiz-center-link" onClick={onDone}>Skip for now</button>
          </div>}
      </div>
      <div className="login-dev">satellite1-a4c2f8</div>
    </section>;
}
