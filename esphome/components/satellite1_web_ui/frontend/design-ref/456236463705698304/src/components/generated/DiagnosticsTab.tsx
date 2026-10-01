import React, { useEffect, useLayoutEffect, useRef, useState } from 'react';
import ReactDOM from 'react-dom';
import { MSlider } from './MSlider';
const DEVICE = {
  name: 'satellite1-a4c2f8',
  label: 'Living Room Satellite',
  ip: '192.168.4.31',
  mac: '74:4d:bd:a4:c2:f8',
  fw: '25.9.3',
  esphome: '2025.9.1',
  xmos: 'v1.3.2',
  built: 'Sep 24 2026, 18:12',
  radarModule: 'LD2450',
  radarFw: 'V2.02.23'
};
type LogLine = {
  id: number;
  lvl: string;
  at: string;
  tag: string;
  text: string;
};
const LOG_LINES: LogLine[] = [{
  id: 1,
  lvl: 'I',
  at: '18:42:01.118',
  tag: 'app',
  text: 'ESPHome version 2025.9.1 compiled on Sep 24 2026'
}, {
  id: 2,
  lvl: 'D',
  at: '18:42:03.402',
  tag: 'sensor',
  text: "'Temperature': Sending state 21.43750 °C with 1 decimals of accuracy"
}, {
  id: 3,
  lvl: 'D',
  at: '18:42:03.921',
  tag: 'micro_wake_word',
  text: 'Streaming inference latency 41 ms'
}, {
  id: 4,
  lvl: 'I',
  at: '18:42:05.010',
  tag: 'voice_assistant',
  text: 'Waiting for wake word'
}, {
  id: 5,
  lvl: 'W',
  at: '18:42:11.223',
  tag: 'wifi',
  text: 'Rate limit hit, retrying in 640 ms'
}, {
  id: 6,
  lvl: 'D',
  at: '18:42:12.850',
  tag: 'ld2450',
  text: 'Target 1 moved to (-38, 214) cm, v=12 cm/s'
}, {
  id: 7,
  lvl: 'I',
  at: '18:42:14.771',
  tag: 'media_player',
  text: "State changed to 'playing'"
}, {
  id: 8,
  lvl: 'D',
  at: '18:42:16.204',
  tag: 'sensor',
  text: "'Ambient Light': Sending state 182.00000 lx"
}, {
  id: 9,
  lvl: 'V',
  at: '18:42:17.001',
  tag: 'api',
  text: 'Connection from Home Assistant established (encrypted)'
}, {
  id: 10,
  lvl: 'D',
  at: '18:42:18.532',
  tag: 'tas2780',
  text: 'DVC set to 74% (mode 2, high gain)'
}];
const HINTS = {
  heap: 'Internal RAM is what runs out first. Under 10% free is the red zone.',
  psram: 'The big external RAM: audio buffers, wake word models, the web app itself.',
  loop: 'The longest single pass through the main loop since the last read. Spikes over 500 ms starve audio.',
  reset: 'Why the device last restarted. "Brownout" points at the power supply.',
  usb_power: 'What the USB-C supply negotiated. High-gain speaker mode needs the PD contract.',
  esp_temp: 'The SoC\u2019s own junction reading - always well above room temperature.',
  speaker_amp: 'The TAS2780 driving the internal speaker: its computed level and its wiring.',
  amp_mode: 'Chosen from the negotiated USB-C contract: high gain on PD, low gain on plain 5 V.',
  amp_dvc: 'The level the firmware computed from the volume sliders - what the amp is actually fed.',
  amp_gain: 'The analog output stage. The notch is the factory default.',
  speaker_channel: 'Which side of a stereo stream this speaker plays.',
  crash: 'Stack traces caught by the panic handler, kept across reboots.',
  beta: 'Offer beta releases from the update channel as well as stable ones.',
  maintenance: 'Restarts and resets for the ESP32 itself.',
  safe_mode: 'Reboots with components disabled - the escape hatch when a bad config boot-loops.',
  factory_reset: 'Wipes settings and credentials. The device reboots into its setup hotspot.',
  radar_recovery: 'Restarts and resets for the radar module alone - the device keeps running.',
  xmos: 'The XMOS chip does echo cancellation and wake word audio. Reflash if voice goes deaf.',
  launch: 'Sign another device in without typing the password: scan the QR or share the link.',
  ha_ingress: 'Put this page (and every peer) in the Home Assistant sidebar through the hass_ingress integration\u2019s proxy mode.'
};
const HAI_YAML = ['ingress:', '  satellite1_a4c2f8:', '    work_mode: ingress', '    title: "Satellite1 Fleet"', '    icon: mdi:satellite-uplink', `    url: http://${DEVICE.ip}`, '    require_admin: true', '    expire_time: 604800', '    headers:', `      host: ${DEVICE.ip}`, '  satellite1_b1c302:', '    parent: satellite1_a4c2f8', '    work_mode: ingress', '    title: "Kitchen Satellite"', '    url: http://192.168.4.32', '    expire_time: 604800', '    headers:', '      host: 192.168.4.32', '  satellite1_c8d415:', '    parent: satellite1_a4c2f8', '    work_mode: ingress', '    title: "Office Satellite"', '    url: http://192.168.4.35', '    expire_time: 604800', '    headers:', '      host: 192.168.4.35'].join('\n');
const CHANNELS = ['Mono (Left + Right)', 'Left Channel Only', 'Right Channel Only'];
let closeActiveHint: (() => void) | null = null;
function HintBtn({
  text
}: {
  text: string;
}) {
  const [open, setOpen] = useState(false);
  const [pos, setPos] = useState<{
    top: number;
    left: number;
  } | null>(null);
  const btn = useRef<HTMLButtonElement>(null);
  const bubble = useRef<HTMLDivElement>(null);
  const close = useRef(() => setOpen(false));
  useLayoutEffect(() => {
    if (!open) {
      setPos(null);
      return;
    }
    const id = requestAnimationFrame(() => {
      const b = btn.current?.getBoundingClientRect();
      const w = bubble.current?.offsetWidth ?? 260;
      const h = bubble.current?.offsetHeight ?? 60;
      if (!b) return;
      let left = b.left + b.width / 2 - w / 2;
      left = Math.max(8, Math.min(left, window.innerWidth - w - 8));
      let top = b.bottom + 8;
      if (top + h > window.innerHeight - 8) top = b.top - h - 8;
      setPos({
        top,
        left
      });
    });
    return () => cancelAnimationFrame(id);
  }, [open]);
  useEffect(() => {
    if (!open) return;
    const onDown = (e: MouseEvent) => {
      const t = e.target as Node;
      if (btn.current?.contains(t) || bubble.current?.contains(t)) return;
      setOpen(false);
    };
    const onScroll = () => setOpen(false);
    document.addEventListener('mousedown', onDown);
    window.addEventListener('scroll', onScroll, true);
    window.addEventListener('resize', onScroll);
    return () => {
      document.removeEventListener('mousedown', onDown);
      window.removeEventListener('scroll', onScroll, true);
      window.removeEventListener('resize', onScroll);
      if (closeActiveHint === close.current) closeActiveHint = null;
    };
  }, [open]);
  const toggle = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (!open) {
      if (closeActiveHint && closeActiveHint !== close.current) closeActiveHint();
      closeActiveHint = close.current;
    }
    setOpen(v => !v);
  };
  return <span style={{
    display: 'inline-flex'
  }}>
      <button ref={btn} type="button" className="dx-hint-btn" aria-label="More info" aria-expanded={open} onClick={toggle}>i</button>
      {open && ReactDOM.createPortal(<div ref={bubble} role="tooltip" className="dx-hint-bubble" style={{
      top: pos?.top ?? -9999,
      left: pos?.left ?? -9999,
      visibility: pos ? 'visible' : 'hidden'
    }}>{text}</div>, document.body)}
    </span>;
}
function DxCard({
  title,
  children,
  collapsible = false,
  defaultOpen = true,
  hint
}: {
  title: string;
  children?: React.ReactNode;
  collapsible?: boolean;
  defaultOpen?: boolean;
  hint?: string;
}) {
  const [open, setOpen] = useState(defaultOpen);
  return <div className="dx-card">
      <div className={`dx-card-head${collapsible ? ' clickable' : ''}`} onClick={collapsible ? () => setOpen(v => !v) : undefined}>
        <h2 className="dx-card-title" style={{
        margin: 0
      }}>{title}</h2>
        {hint && <HintBtn text={hint} />}
        {collapsible && <svg className="dx-caret" width="14" height="14" viewBox="0 0 14 14" fill="none" style={{
        transform: open ? 'rotate(180deg)' : undefined,
        transition: 'transform .2s',
        marginLeft: 'auto'
      }}>
            <path d="M3 5l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
          </svg>}
      </div>
      {(!collapsible || open) && <div className="dx-card-body">{children}</div>}
    </div>;
}
function DxRow({
  label,
  hint,
  children
}: {
  label: string;
  hint?: string;
  children?: React.ReactNode;
}) {
  return <div className="dx-row">
      <div className="dx-row-label"><span>{label}</span>{hint && <HintBtn text={hint} />}</div>
      <div className="dx-row-val">{children}</div>
    </div>;
}
function DxFact({
  label,
  value,
  unit,
  sub,
  hint
}: {
  label: string;
  value: React.ReactNode;
  unit?: string;
  sub?: React.ReactNode;
  hint?: string;
}) {
  return <div className="dx-fact">
      <div className="dx-fact-label"><span>{label}</span>{hint && <HintBtn text={hint} />}</div>
      <div className="dx-fact-val">
        <span className="dx-fact-v">{value}</span>
        {unit && <span className="dx-fact-unit">{unit}</span>}
        {sub && <div className="dx-fact-sub">{sub}</div>}
      </div>
    </div>;
}
function DxFacts({
  children
}: {
  children: React.ReactNode;
}) {
  return <div className="dx-facts">{children}</div>;
}
function DxConfirm({
  label,
  title,
  body,
  confirmLabel,
  danger = false,
  disabled = false,
  onConfirm,
  solid = false
}: {
  label: string;
  title: string;
  body: string;
  confirmLabel: string;
  danger?: boolean;
  disabled?: boolean;
  onConfirm: () => void;
  solid?: boolean;
}) {
  const [open, setOpen] = useState(false);
  const modalPanelRef = useRef<HTMLDivElement>(null);
  const modalDragStartY = useRef<number | null>(null);
  useEffect(() => {
    if (!open) return;
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, [open]);
  return <span style={{
    display: 'contents'
  }}>
      <button className={`dx-btn${solid ? ' solid' : ''}${danger ? ' danger' : ''}`} disabled={disabled} onClick={() => setOpen(true)}>{label}</button>
      {open && ReactDOM.createPortal([<div key="scrim" className="dx-modal-over" onClick={() => setOpen(false)} />, <div key="modal" ref={modalPanelRef} className="dx-modal" role="dialog" aria-modal="true" onClick={e => e.stopPropagation()}>
            <div className="handle" role="button" aria-label="Close" style={{
        touchAction: 'none',
        cursor: 'grab'
      }} onPointerDown={e => {
        if (window.innerWidth >= 1024) return;
        modalDragStartY.current = e.clientY;
        e.currentTarget.setPointerCapture(e.pointerId);
        if (modalPanelRef.current) modalPanelRef.current.style.transition = 'none';
      }} onPointerMove={e => {
        if (modalDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - modalDragStartY.current);
        if (modalPanelRef.current) {
          modalPanelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
          modalPanelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
        }
      }} onPointerUp={e => {
        if (modalDragStartY.current === null) return;
        const dy = Math.max(0, e.clientY - modalDragStartY.current);
        const dismiss = () => setOpen(false);
        if (dy > 80) {
          if (modalPanelRef.current) {
            modalPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
            modalPanelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
            modalPanelRef.current.style.opacity = '0';
            setTimeout(dismiss, 210);
          } else dismiss();
        } else if (modalPanelRef.current) {
          modalPanelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          modalPanelRef.current.style.transform = 'translateX(-50%)';
          modalPanelRef.current.style.opacity = '1';
        }
        modalDragStartY.current = null;
        setTimeout(() => {
          if (modalPanelRef.current) {
            modalPanelRef.current.style.transition = '';
            modalPanelRef.current.style.transform = '';
            modalPanelRef.current.style.opacity = '';
          }
        }, 250);
      }} />
            <p className="dx-modal-title">{title}</p>
            <p className="dx-modal-body">{body}</p>
            <div className="dx-modal-actions">
              <button className="dx-btn" onClick={() => setOpen(false)}>Cancel</button>
              <button className={`dx-btn solid${danger ? ' danger' : ''}`} onClick={() => {
          onConfirm();
          setOpen(false);
        }}>{confirmLabel}</button>
            </div>
          </div>], document.body)}
    </span>;
}
function DxToggle({
  checked,
  onChange
}: {
  checked: boolean;
  onChange: (v: boolean) => void;
}) {
  return <button role="switch" aria-checked={checked} className={`dx-toggle${checked ? ' on' : ''}`} onClick={() => onChange(!checked)}>
      <span className="dx-toggle-thumb" />
    </button>;
}
function DxSelect({
  value,
  options,
  onChange
}: {
  value: string;
  options: string[];
  onChange: (v: string) => void;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!open) return;
    const close = (e: MouseEvent) => {
      if (!ref.current?.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener('mousedown', close);
    return () => document.removeEventListener('mousedown', close);
  }, [open]);
  return <div ref={ref} className="dx-sel" style={{
    position: 'relative',
    zIndex: 20
  }}>
      <button className={`dx-sel-btn${open ? ' open' : ''}`} onClick={() => setOpen(v => !v)}>
        <span>{value}</span>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" style={{
        transform: open ? 'rotate(180deg)' : undefined,
        transition: 'transform .2s'
      }}>
          <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
        </svg>
      </button>
      {open && <div className="dx-sel-pop">
          {options.map(o => <button key={o} className={`dx-sel-opt${o === value ? ' active' : ''}`} onClick={() => {
        onChange(o);
        setOpen(false);
      }}>
              {o === value && <svg width="12" height="12" viewBox="0 0 12 12" fill="none"><path d="M2 6l3 3 5-5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>}
              <span>{o}</span>
            </button>)}
        </div>}
    </div>;
}
function DeviceCard() {
  const [secs, setSecs] = useState(3 * 86400 + 7 * 3600 + 22 * 60);
  useEffect(() => {
    const t = setInterval(() => setSecs(s => s + 1), 1000);
    return () => clearInterval(t);
  }, []);
  const up = `${Math.floor(secs / 86400)}d ${Math.floor(secs % 86400 / 3600)}h ${Math.floor(secs % 3600 / 60)}m`;
  return <DxCard title="Device" collapsible defaultOpen={true}>
      <DxFacts>
        <DxFact label="Sat1 Firmware" value={<a href="https://github.com/FutureProofHomes/Satellite1-ESPHome/releases" target="_blank" rel="noopener noreferrer" className="dx-link">{DEVICE.fw}</a>} sub="Update available: 25.9.4" />
        <DxFact label="Internal RAM free" value="118 kB" unit=" of 268 kB" hint={HINTS.heap} />
        <DxFact label="PSRAM free" value="4.1 MB" unit=" of 7.9 MB" hint={HINTS.psram} />
        <DxFact label="Longest loop" value="38" unit=" ms" hint={HINTS.loop} />
        <DxFact label="ESP32 Temp" value="52.3" unit=" °C" hint={HINTS.esp_temp} />
        <DxFact label="Uptime" value={up} />
        <DxFact label="Last restart" value="Power on" hint={HINTS.reset} />
        <DxFact label="USB-C Power Supply" value="15V @ 3A~" sub="45 watts" hint={HINTS.usb_power} />
        <DxFact label="Network Type" value="Wi-Fi -52 dBm" sub={<span><span className="dx-mono">{DEVICE.ip}</span><span className="dx-mono">{DEVICE.mac}</span></span>} />
      </DxFacts>
    </DxCard>;
}
function SpeakerAmpCard() {
  const [gain, setGain] = useState(8);
  const [chan, setChan] = useState('Mono (Left + Right)');
  return <DxCard title="TAS2780 Amplifier Control" collapsible defaultOpen={true} hint={HINTS.speaker_amp}>
      <DxRow label="Power gain mode" hint={HINTS.amp_mode}>
        <span className="dx-dim">High gain</span>
      </DxRow>
      <DxRow label="Digital volume" hint={HINTS.amp_dvc}><span className="dx-dim">74%</span></DxRow>
      <DxRow label="Analog gain" hint={HINTS.amp_gain}>
        <MSlider value={gain} min={0} max={20} step={1} snap={8} ariaLabel="Analog gain" format={v => `${(11 + v / 2).toFixed(1)} dBV`} onCommit={setGain} />
      </DxRow>
      <DxRow label="Channel" hint={HINTS.speaker_channel}>
        <DxSelect value={chan} options={CHANNELS} onChange={setChan} />
      </DxRow>
      <DxRow label="Line out"><span className="dx-dim">Nothing plugged in</span></DxRow>
    </DxCard>;
}
function CrashCard() {
  return <DxCard title="Crash Reports" collapsible defaultOpen={true} hint={HINTS.crash}>
      <p className="dx-muted">No crashes recorded. The panic handler writes a report here if one ever happens.</p>
    </DxCard>;
}
function FirmwareCard() {
  const [beta, setBeta] = useState(false);
  const availableFirmware = '25.9.4';
  const releaseNotesUrl = 'https://github.com/FutureProofHomes/Satellite1-ESPHome/releases';
  return <DxCard title="Updates" collapsible defaultOpen={true}>
      <div className="dx-row" style={{
      alignItems: 'center'
    }}>
        <div style={{
        flex: 1,
        minWidth: 0,
        display: 'flex',
        flexDirection: 'column'
      }}>
          <div className="dx-fact-label"><span>Sat1 Firmware</span></div>
          <span className="dx-fact-v"><a href={releaseNotesUrl} target="_blank" rel="noopener noreferrer" className="dx-link">{DEVICE.fw}</a></span>
        </div>
        {availableFirmware !== DEVICE.fw && <a className="dx-btn solid" href={releaseNotesUrl} target="_blank" rel="noopener noreferrer" style={{
        marginLeft: 'auto',
        display: 'inline-flex',
        alignItems: 'center',
        textDecoration: 'none'
      }}>Update</a>}
      </div>
      <DxRow label="Beta updates" hint={HINTS.beta}>
        <DxToggle checked={beta} onChange={setBeta} />
      </DxRow>
    </DxCard>;
}
function FakeQr() {
  const size = 25;
  let d = '';
  let s = 41;
  const cell = (x: number, y: number) => {
    d += `M${x} ${y}h1v1h-1z`;
  };
  const finderAt = (fx: number, fy: number) => {
    for (let x = 0; x < 7; x++) for (let y = 0; y < 7; y++) {
      const ring = x === 0 || y === 0 || x === 6 || y === 6;
      const core = x >= 2 && x <= 4 && y >= 2 && y <= 4;
      if (ring || core) cell(fx + x, fy + y);
    }
  };
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      s = s * 1103515245 + 12345 & 0x7fffffff;
      const inFinder = x < 8 && y < 8 || x >= size - 8 && y < 8 || x < 8 && y >= size - 8;
      if (!inFinder && s % 5 < 2) cell(x, y);
    }
  }
  finderAt(0, 0);
  finderAt(size - 7, 0);
  finderAt(0, size - 7);
  return <svg className="dx-qr" viewBox="-2 -2 29 29" role="img" aria-label="Sign-in QR code">
      <path d={d} />
    </svg>;
}
const ROUTE_HEADLINES: Record<string, {
  prefix: string;
  em: string;
}> = {
  'device-info': {
    prefix: 'Know your ',
    em: 'hardware.'
  },
  'updates': {
    prefix: 'Stay ',
    em: 'current.'
  },
  'security': {
    prefix: 'Lock it ',
    em: 'down.'
  },
  'logs': {
    prefix: 'Read the ',
    em: 'tape.'
  },
  'integrations': {
    prefix: 'Connect ',
    em: 'everything.'
  },
  'recovery': {
    prefix: 'Back from the ',
    em: 'brink.'
  },
  'audio': {
    prefix: 'Shape the ',
    em: 'sound.'
  },
  'community': {
    prefix: "You're not ",
    em: 'alone.'
  }
};
const DEFAULT_HEADLINE = {
  prefix: 'Under the ',
  em: 'hood.'
};
function ChangePassword() {
  const [cur, setCur] = useState('');
  const [next, setNext] = useState('');
  const [again, setAgain] = useState('');
  const [err, setErr] = useState<string | null>(null);
  return <div className="dx-pwc">
      <p className="dx-pwc-title">Change password</p>
      <input className="dx-in" type="password" value={cur} placeholder="Current password" autoComplete="current-password" aria-label="Current password" onChange={e => setCur(e.target.value)} />
      <input className="dx-in" type="password" value={next} placeholder="New password" autoComplete="new-password" aria-label="New password" onChange={e => setNext(e.target.value)} />
      <input className="dx-in" type="password" value={again} placeholder="Confirm new password" autoComplete="new-password" aria-label="Confirm new password" onChange={e => setAgain(e.target.value)} />
      <div className="dx-pwc-actions">
        <DxConfirm label="Change password" title="Change the password?" body="Every other signed-in browser and sign-in link is signed out the moment it changes. This browser stays in." confirmLabel="Change password" danger disabled={!cur || !next || !again} onConfirm={() => {
        const bad = next.length < 8 || next.length > 31 ? 'The new password needs 8 to 31 characters.' : next !== again ? "The two copies don't match." : null;
        if (bad) {
          setErr(bad);
          return;
        }
        setErr(null);
        setCur('');
        setNext('');
        setAgain('');
      }} />
      </div>
      {err && <p className="dx-err">{err}</p>}
    </div>;
}
function AuthTokenCard() {
  const [copied, setCopied] = useState(false);
  const link = `http://${DEVICE.name}.local/?key=9f27c1e4a8b35d60`;
  return <DxCard title="Auth Token" collapsible defaultOpen={true} hint={HINTS.launch}>
      <div className="dx-launch">
        <FakeQr />
        <div className="dx-launch-side">
          <div className="dx-launch-link">{link}</div>
          <div className="dx-launch-actions">
            <button className="dx-btn" onClick={() => {
            setCopied(true);
            setTimeout(() => setCopied(false), 2000);
          }}>{copied ? 'Copied' : 'Copy link'}</button>
            <DxConfirm label="Sign out everywhere" danger title="Sign out everywhere?" body="Every signed-in browser, pasted link and QR stops working, this one included - you sign back in with the password." confirmLabel="Sign out everywhere" onConfirm={() => {}} />
          </div>
        </div>
      </div>
    </DxCard>;
}
function ChangePasswordCard() {
  return <DxCard title="Change Password" collapsible defaultOpen={true}>
      <ChangePassword />
    </DxCard>;
}
function HomeAssistantCard() {
  const [copied, setCopied] = useState(false);
  return <DxCard title="HA Side Panel" collapsible defaultOpen={true} hint={HINTS.ha_ingress}>
      <p className="dx-muted">
        <span>Put your Satellite1 fleet in the Home Assistant sidebar with the third-party </span>
        <a className="dx-link" href="https://github.com/lovelylain/hass_ingress" target="_blank" rel="noopener noreferrer">hass_ingress</a>
        <span> integration: paste this into its configuration and restart Home Assistant.</span>
      </p>
      <pre className="dx-yaml">{HAI_YAML}</pre>
      <div style={{
      padding: '0 16px'
    }}>
        <button className="dx-btn" onClick={() => {
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
      }}>{copied ? 'Copied' : 'Copy YAML'}</button>
      </div>
      <p className="dx-muted dx-sm">The proxy points at IP addresses, so give each device a DHCP reservation - a lease change breaks the panel quietly.</p>
    </DxCard>;
}
const LVL_CLASS: Record<string, string> = {
  E: 'dx-err',
  W: 'dx-warn',
  I: 'dx-ok',
  D: 'dx-dbg',
  V: 'dx-dim'
};
function LogsCard() {
  const [filter, setFilter] = useState('');
  const [level, setLevel] = useState<'everything' | 'debug' | 'info' | 'warning' | 'errors'>('everything');
  const [paused, setPaused] = useState(false);
  const [lines, setLines] = useState<LogLine[]>(LOG_LINES);
  const logRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (paused) return;
    const t = setInterval(() => {
      setLines(ls => {
        const n = ls.length + (ls[ls.length - 1]?.id ?? 0);
        const now = new Date();
        const at = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}.${String(now.getMilliseconds()).padStart(3, '0')}`;
        const next: LogLine = {
          id: (ls[ls.length - 1]?.id ?? 0) + 1,
          lvl: 'D',
          at,
          tag: n % 2 ? 'ld2450' : 'sensor',
          text: n % 2 ? `Target 1 moved to (${-30 - n % 40}, ${200 + n % 60}) cm` : `'Temperature': Sending state 21.4${n % 10} °C`
        };
        return [...ls, next].slice(-60);
      });
    }, 3000);
    return () => clearInterval(t);
  }, [paused]);
  useEffect(() => {
    if (!paused && logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight;
  }, [lines, paused]);
  const LEVEL_ALLOW: Record<string, string[]> = {
    everything: ['V', 'D', 'I', 'W', 'E'],
    debug: ['D', 'I', 'W', 'E'],
    info: ['I', 'W', 'E'],
    warning: ['W', 'E'],
    errors: ['E']
  };
  const shown = lines.filter(l => LEVEL_ALLOW[level].includes(l.lvl)).filter(l => !filter || l.text.toLowerCase().includes(filter.toLowerCase()) || l.tag.toLowerCase().includes(filter.toLowerCase()));
  return <DxCard title="Device Logs" collapsible defaultOpen={true}>
      <div className="dx-log-bar">
        <input className="dx-log-search" type="search" placeholder="Filter…" aria-label="Filter logs" value={filter} onChange={e => setFilter(e.target.value)} />
        <DxSelect value={level.charAt(0).toUpperCase() + level.slice(1)} options={['Everything', 'Debug', 'Info', 'Warning', 'Errors']} onChange={v => setLevel(v.toLowerCase() as typeof level)} />
      </div>
      <div className="dx-log" ref={logRef}>
        {shown.map(l => <div key={l.id} className={`dx-log-line ${LVL_CLASS[l.lvl] || ''}`}>
            <span className="dx-log-ts">{l.at}</span>
            <span className="dx-log-lvl">{l.lvl}</span>
            <span className="dx-log-tag">{l.tag}</span>
            <span className="dx-log-txt">{l.text}</span>
          </div>)}
      </div>
      <div className="dx-log-foot">
        <span className="dx-muted dx-xs" style={{
        padding: 0,
        flex: 1
      }}>{shown.length === lines.length ? `${lines.length} lines` : `${shown.length} of ${lines.length} lines`}</span>
        <button className={`dx-btn sm${!paused ? ' active' : ''}`} onClick={() => setPaused(v => !v)}>{paused ? 'Resume' : 'Following'}</button>
        <button className="dx-btn sm">Export</button>
      </div>
    </DxCard>;
}
function RecoveryCards() {
  return <div>
      <DxCard title="ESP32 System Control" collapsible defaultOpen={true} hint={HINTS.maintenance}>
        <DxFacts>
          <DxFact label="ESPHome Version" value={DEVICE.esphome} />
        </DxFacts>
        <DxRow label="Restart">
          <DxConfirm label="Restart" title="Restart the device?" body="The device is away for fifteen seconds or so. Music stops; timers keep counting." confirmLabel="Restart" onConfirm={() => {}} />
        </DxRow>
        <DxRow label="Safe mode" hint={HINTS.safe_mode}>
          <DxConfirm label="Safe mode" title="Reboot into safe mode?" body="Components stay disabled until the next normal boot - the escape hatch when something boot-loops." confirmLabel="Safe mode" onConfirm={() => {}} />
        </DxRow>
        <DxRow label="Factory reset" hint={HINTS.factory_reset}>
          <DxConfirm label="Factory reset" danger title="Factory reset the device?" body="Wi-Fi credentials, calibration and every setting are wiped. The device reboots into its setup hotspot." confirmLabel="Erase everything" onConfirm={() => {}} />
        </DxRow>
      </DxCard>
      <DxCard title="XMOS Audio Control" collapsible defaultOpen={true} hint={HINTS.xmos}>
        <DxFacts>
          <DxFact label="XMOS Firmware" value={DEVICE.xmos} hint={HINTS.xmos} />
        </DxFacts>
        <DxRow label="Restart XMOS">
          <DxConfirm label="Restart" title="Restart the XMOS?" body="Voice goes deaf for a few seconds while the audio processor reboots." confirmLabel="Restart XMOS" onConfirm={() => {}} />
        </DxRow>
        <DxRow label="Reflash XMOS firmware">
          <DxConfirm label="Reflash" danger title="Reflash the XMOS firmware?" body="Takes about a minute. Do not power the device off while the flash runs." confirmLabel="Reflash" onConfirm={() => {}} />
        </DxRow>
      </DxCard>
      <DxCard title="LD2450 Radar Control" collapsible defaultOpen={true} hint={HINTS.radar_recovery}>
        <DxFacts>
          <DxFact label="Radar Module" value={DEVICE.radarModule} />
          <DxFact label="Radar Firmware" value={DEVICE.radarFw} />
        </DxFacts>
        <DxRow label="Restart radar">
          <DxConfirm label="Restart" title="Restart the radar module?" body="Presence reads clear for a few seconds while it comes back. Nothing else on the device is touched." confirmLabel="Restart radar" onConfirm={() => {}} />
        </DxRow>
        <DxRow label="Factory reset radar">
          <DxConfirm label="Factory reset" danger title="Factory reset the radar?" body="Zones, thresholds and the detection range go back to the module defaults. This cannot be undone." confirmLabel="Reset radar" onConfirm={() => {}} />
        </DxRow>
      </DxCard>
      <DxCard title="LD2410 Radar Control" collapsible defaultOpen={true} hint={HINTS.radar_recovery}>
        <DxFacts>
          <DxFact label="Radar Module" value="LD2410" />
          <DxFact label="Radar Firmware" value={DEVICE.radarFw} />
        </DxFacts>
        <DxRow label="Restart radar">
          <DxConfirm label="Restart" title="Restart the LD2410?" body="Presence reads clear for a few seconds while it comes back. Nothing else on the device is touched." confirmLabel="Restart radar" onConfirm={() => {}} />
        </DxRow>
        <DxRow label="Factory reset radar">
          <DxConfirm label="Factory reset" danger title="Factory reset the LD2410?" body="All gate thresholds and detection settings go back to module defaults. This cannot be undone." confirmLabel="Reset radar" onConfirm={() => {}} />
        </DxRow>
      </DxCard>
    </div>;
}
function CommunityLinks() {
  return <nav className="dx-links" aria-label="FutureProofHomes community links">
      <a href="https://docs.futureproofhomes.net/" target="_blank" rel="noopener noreferrer">
        <svg viewBox="0 0 16 16" aria-hidden="true"><path d="M1.6 2.4h3.8a2.6 2.6 0 0 1 2.6 2.6v8.9a2 2 0 0 0-2-2H1.6z" /><path d="M14.4 2.4h-3.8A2.6 2.6 0 0 0 8 5v8.9a2 2 0 0 1 2-2h4.4z" /></svg>
        <span>Docs</span>
      </a>
      <a href="https://github.com/FutureProofHomes" target="_blank" rel="noopener noreferrer">
        <svg viewBox="0 0 16 16" aria-hidden="true"><path d="M10.67 14.67v-2.58a2.25 2.25 0 0 0-.63-1.74c2.09-.23 4.290-1.03 4.29-4.67a3.63 3.63 0 0 0-1-2.52 3.38 3.38 0 0 0-.06-2.51s-.79-.23-2.61.99a8.92 8.92 0 0 0-4.66 0c-1.82-1.22-2.61-.99-2.61-.99a3.38 3.38 0 0 0-.06 2.51 3.63 3.63 0 0 0-1 2.52c0 3.61 2.2 4.4 4.29 4.67a2.25 2.25 0 0 0-.62 1.73v2.59" /><path d="M6 12.67c-3.33 1-3.33-1.67-4.67-2" /></svg>
        <span>GitHub</span>
      </a>
      <a href="https://www.youtube.com/@futureproofhomes" target="_blank" rel="noopener noreferrer">
        <svg viewBox="0 0 16 16" aria-hidden="true"><rect x="1.8" y="4" width="12.4" height="8.4" rx="2.6" /><path d="M6.9 6.6v3.2l3-1.6z" fill="currentColor" /></svg>
        <span>YouTube</span>
      </a>
      <a href="https://discord.futureproofhomes.net/" target="_blank" rel="noopener noreferrer">
        <svg viewBox="0 0 16 16" aria-hidden="true"><path d="M10.33 11.67l.67 1.33s2.78-.89 3.67-2.33c0-.67.35-5.43-2-7-1-.67-2.67-1-2.67-1l-.67 1.33h-1.33" /><path d="M5.69 11.67l-.67 1.330s-2.78-.89-3.67-2.33c0-.67-.35-5.43 2-7 1-.67 2.670-1 2.67-1l.67 1.33h1.33" /><circle cx="5.67" cy="8.33" r="1" fill="currentColor" stroke="none" /><circle cx="10.33" cy="8.33" r="1" fill="currentColor" stroke="none" /></svg>
        <span>Discord</span>
      </a>
    </nav>;
}
export const SETTINGS_ROUTES = [{
  slug: 'device-info',
  label: 'Device Info'
}, {
  slug: 'updates',
  label: 'Updates'
}, {
  slug: 'security',
  label: 'Security'
}, {
  slug: 'logs',
  label: 'Logs'
}, {
  slug: 'integrations',
  label: 'Integrations'
}, {
  slug: 'recovery',
  label: 'Recovery'
}, {
  slug: 'audio',
  label: 'Audio'
}, {
  slug: 'community',
  label: 'Community'
}];
export function DiagnosticsTab({
  onSimulateHaBlock,
  subRoute = 'device-info',
  onSubRouteChange
}: {
  onSimulateHaBlock?: () => void;
  subRoute?: string;
  onSubRouteChange?: (v: string) => void;
} = {}) {
  void onSubRouteChange;
  const [visible, setVisible] = useState(true);
  const [displayedRoute, setDisplayedRoute] = useState(subRoute);
  useEffect(() => {
    if (subRoute === displayedRoute) return;
    setVisible(false);
    const t = setTimeout(() => {
      setDisplayedRoute(subRoute);
      setVisible(true);
    }, 160);
    return () => clearTimeout(t);
  }, [subRoute, displayedRoute]);
  const active = SETTINGS_ROUTES.find(r => r.slug === displayedRoute) ?? SETTINGS_ROUTES[0];
  return <section className="control dx-tab">
      <div style={{
      opacity: visible ? 1 : 0,
      transform: visible ? 'translateY(0)' : 'translateY(6px)',
      transition: 'opacity 0.16s ease, transform 0.16s ease'
    }}>
      <span className="eyebrow">SETTINGS · {active.label.toUpperCase()}</span>
      <h1><span>{(ROUTE_HEADLINES[active.slug] ?? DEFAULT_HEADLINE).prefix}</span><em>{(ROUTE_HEADLINES[active.slug] ?? DEFAULT_HEADLINE).em}</em></h1>
      {active.slug === 'device-info' && <DeviceCard />}
      {active.slug === 'updates' && <FirmwareCard />}
      {active.slug === 'security' && <AuthTokenCard />}
      {active.slug === 'security' && <ChangePasswordCard />}
      {active.slug === 'logs' && <LogsCard />}
      {active.slug === 'logs' && <CrashCard />}
      {active.slug === 'integrations' && <HomeAssistantCard />}
      {active.slug === 'recovery' && <RecoveryCards />}
      {active.slug === 'audio' && <SpeakerAmpCard />}
      {active.slug === 'community' && <CommunityLinks />}
      {onSimulateHaBlock && active.slug === 'integrations' && <button type="button" className="dx-sim-block" onClick={onSimulateHaBlock}>Simulate HA Block</button>}
      </div>
    </section>;
}