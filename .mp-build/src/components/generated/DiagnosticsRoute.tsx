/**
 * The Diagnostics route: device vitals, the speaker amplifier, crash reports, firmware, the
 * Launch card (sign-in link, QR, change password), the Home Assistant sidebar setup, the live
 * log viewer, the recovery cards and the community links. Ported from routes/diagnostics.jsx
 * in the real route's order.
 */
import React, { useEffect, useState } from 'react';
import { Btn, Card, Confirm, Fact, N_AUDIO, N_DIAG, ni, Row, Select, Slider, Toggle } from './ui';
import { DEVICE, LOG_LINES } from './mock';
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
const TEXT = {
  launch_copy: 'Copy link',
  launch_copied: 'Copied',
  launch_regen: 'Sign out everywhere',
  launch_regen_title: 'Sign out everywhere?',
  launch_regen_body: 'Every signed-in browser, pasted link and QR stops working, this one included - you sign back in with the password.',
  pw_title: 'Change password',
  pw_current: 'Current password',
  pw_new: 'New password',
  pw_again: 'Confirm new password',
  pw_change_t: 'Change the password?',
  pw_change_b: 'Every other signed-in browser and sign-in link is signed out the moment it changes. This browser stays in.',
  hai_pre: 'Put your Satellite1 fleet in the Home Assistant sidebar with the third-party ',
  hai_link: 'hass_ingress',
  hai_post: ' integration: paste this into its configuration and restart Home Assistant.',
  hai_copy: 'Copy YAML',
  hai_copied: 'Copied',
  hai_dhcp_hint: 'The proxy points at IP addresses, so give each device a DHCP reservation - a lease change breaks the panel quietly.'
};

/* ------------------------------------------------------------------ */

function DeviceCard() {
  const [uptime, setUptime] = useState(3 * 86400 + 7 * 3600 + 22 * 60);
  useEffect(() => {
    const t = setInterval(() => setUptime(v => v + 1), 1000);
    return () => clearInterval(t);
  }, []);
  const up = `${Math.floor(uptime / 86400)}d ${Math.floor(uptime % 86400 / 3600)}h ${Math.floor(uptime % 3600 / 60)}m`;
  return <Card title="Device" icon={N_DIAG}>
      <div className="facts">
        <Fact label="Internal RAM free" value="118 kB" unit=" of 268 kB" hint={HINTS.heap} />
        <Fact label="PSRAM free" value="4.1 MB" unit=" of 7.9 MB" hint={HINTS.psram} />
        <Fact label="Longest loop" value="38" unit=" ms" hint={HINTS.loop} />
        <Fact label="ESP32 Temp" value="52.3" unit=" °C" hint={HINTS.esp_temp} />
        <Fact label="Uptime" value={up} />
        <Fact label="Last restart" value="Power on" hint={HINTS.reset} />
        <Fact label="USB-C Power Supply" value="15V @ 3A~" sub="45 watts" hint={HINTS.usb_power} />
        <Fact label="Network Type" value="Wi-Fi -52 dBm" sub={<>
              <span className="num">{DEVICE.ip}</span>
              <span className="num">{DEVICE.mac}</span>
            </>} />
      </div>
    </Card>;
}
function SpeakerAmp() {
  const [gain, setGain] = useState(8);
  const [chan, setChan] = useState('Mono (Left + Right)');
  return <Card title="Speaker amplifier" icon={N_AUDIO} hint={HINTS.speaker_amp}>
      <Row label="Power gain mode" hint={HINTS.amp_mode}>
        <span className="dim">
          High gain<span className="xs"> &middot; Running from the USB-PD supply</span>
        </span>
      </Row>
      <Row label="Digital volume" hint={HINTS.amp_dvc}>
        <span className="dim">74%</span>
      </Row>
      <Row label="Analog gain" hint={HINTS.amp_gain}>
        <Slider value={gain} min={0} max={20} step={1} snap={8} format={v => `${(11 + v / 2).toFixed(1)} dBV`} onCommit={setGain} />
      </Row>
      <Row label="Channel" hint={HINTS.speaker_channel}>
        <Select value={chan} options={['Mono (Left + Right)', 'Left Channel Only', 'Right Channel Only']} onChange={setChan} />
      </Row>
      <Row label="Line out">
        <span className="dim">Nothing plugged in</span>
      </Row>
    </Card>;
}
function CrashCard() {
  return <Card title="Crash Reports" collapsible defaultOpen hint={HINTS.crash}>
      <p className="dim sm">No crashes recorded. The panic handler writes a report here if one ever happens.</p>
    </Card>;
}
function FirmwareCard() {
  const [beta, setBeta] = useState(false);
  return <Card title="Firmware">
      <div className="facts">
        <Fact label="Sat1 firmware" value={DEVICE.fw} sub="Update available: 25.9.4" />
        <Fact label="ESPHome Version" value={DEVICE.esphome} />
        <Fact label="XMOS firmware" value={DEVICE.xmos} hint={HINTS.xmos} />
        <Fact label="Built" value={DEVICE.built} />
        <Fact label="Radar module" value={DEVICE.radarModule} />
        <Fact label="Radar firmware" value={DEVICE.radarFw} />
      </div>
      <Row label="Install 25.9.4">
        <Confirm label="Install" solid title="Install firmware 25.9.4?" body="The device downloads the update, reboots, and is away for two to three minutes. Music stops; timers keep counting." confirmLabel="Install and reboot" onConfirm={() => {}} />
      </Row>
      <Row label="Beta updates" hint={HINTS.beta}>
        <Toggle checked={beta} onChange={setBeta} />
      </Row>
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Launch: the sign-in link, the QR, and the password change            */
/* ------------------------------------------------------------------ */

/** A drawn stand-in for the sign-in QR: deterministic noise on the QR grid, one path. */
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
  return <svg className="launch-qr" viewBox="-2 -2 29 29" role="img" aria-label="Sign-in QR code">
      <path d={d} />
    </svg>;
}

/** The authenticated password change, at the foot of the Launch card. */
function ChangePassword() {
  const [cur, setCur] = useState('');
  const [next, setNext] = useState('');
  const [again, setAgain] = useState('');
  const [err, setErr] = useState<string | null>(null);
  const check = () => {
    if (next.length < 8 || next.length > 31) return 'The new password needs 8 to 31 characters.';
    if (next !== again) return "The two copies of the new password don't match.";
    return null;
  };
  return <div className="pwc">
      <div className="row">
        <span className="grow strong">{TEXT.pw_title}</span>
      </div>
      <input className="in" type="password" value={cur} placeholder={TEXT.pw_current} autoComplete="current-password" aria-label={TEXT.pw_current} onInput={e => setCur((e.target as HTMLInputElement).value)} />
      <input className="in" type="password" value={next} placeholder={TEXT.pw_new} autoComplete="new-password" aria-label={TEXT.pw_new} onInput={e => setNext((e.target as HTMLInputElement).value)} />
      <input className="in" type="password" value={again} placeholder={TEXT.pw_again} autoComplete="new-password" aria-label={TEXT.pw_again} onInput={e => setAgain((e.target as HTMLInputElement).value)} />
      <div className="launch-actions">
        <Confirm label={TEXT.pw_title} title={TEXT.pw_change_t} body={TEXT.pw_change_b} confirmLabel={TEXT.pw_title} danger disabled={!cur || !next || !again} onConfirm={() => {
        const bad = check();
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
      {err && <p className="t-err sm">{err}</p>}
    </div>;
}
function LaunchCard() {
  const [copied, setCopied] = useState(false);
  const link = `http://${DEVICE.name}.local/?key=9f27c1e4a8b35d60`;
  return <Card title="Launch" collapsible hint={HINTS.launch}>
      <div className="launch">
        <FakeQr />
        <div className="launch-side">
          <div className="launch-link num">{link}</div>
          <div className="launch-actions">
            <Btn onClick={() => {
            setCopied(true);
            setTimeout(() => setCopied(false), 2000);
          }}>
              {copied ? TEXT.launch_copied : TEXT.launch_copy}
            </Btn>
            <Confirm label={TEXT.launch_regen} title={TEXT.launch_regen_title} body={TEXT.launch_regen_body} confirmLabel={TEXT.launch_regen} danger onConfirm={() => {}} />
          </div>
        </div>
      </div>
      <ChangePassword />
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Home Assistant sidebar setup                                        */
/* ------------------------------------------------------------------ */

const HAI_YAML = ['ingress:', '  satellite1_a4c2f8:', '    work_mode: ingress', '    title: "Satellite1 Fleet"', '    icon: mdi:satellite-uplink', `    url: http://${DEVICE.ip}`, '    require_admin: true   # only admins see the panel; remove to show everyone', '    expire_time: 604800   # keep long-lived tabs signed in (default is 1 hour)', '    headers:', `      host: ${DEVICE.ip}   # keeps push-button sign-in working through the proxy`, '  satellite1_b1c302:', '    parent: satellite1_a4c2f8   # hidden from the sidebar; the device switcher reaches it', '    work_mode: ingress', '    title: "Kitchen Satellite"', '    url: http://192.168.4.32', '    expire_time: 604800', '    headers:', '      host: 192.168.4.32', '  satellite1_c8d415:', '    parent: satellite1_a4c2f8   # hidden from the sidebar; the device switcher reaches it', '    work_mode: ingress', '    title: "Office Satellite"', '    url: http://192.168.4.35', '    expire_time: 604800', '    headers:', '      host: 192.168.4.35'].join('\n');
function HomeAssistantCard() {
  const [copied, setCopied] = useState(false);
  return <Card title="Home Assistant" collapsible hint={HINTS.ha_ingress}>
      <p className="dim sm hai-note">
        {TEXT.hai_pre}
        <a href="https://github.com/lovelylain/hass_ingress" target="_blank" rel="noopener noreferrer" onClick={e => e.preventDefault()}>
          {TEXT.hai_link}
        </a>
        {TEXT.hai_post}
      </p>
      <pre className="hai-yaml">{HAI_YAML}</pre>
      <div className="launch-actions">
        <Btn onClick={() => {
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
      }}>
          {copied ? TEXT.hai_copied : TEXT.hai_copy}
        </Btn>
      </div>
      <p className="dim sm">{TEXT.hai_dhcp_hint}</p>
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Logs                                                                */
/* ------------------------------------------------------------------ */

const LEVEL_TONE: Record<string, string> = {
  E: 't-err',
  W: 't-warn',
  I: 't-ok',
  D: 't-dbg',
  V: 't-dim'
};
function LogsCard() {
  const [filter, setFilter] = useState('');
  const [paused, setPaused] = useState(false);
  const [lines, setLines] = useState(LOG_LINES);
  useEffect(() => {
    if (paused) return;
    const t = setInterval(() => {
      setLines(ls => {
        const n = ls.length;
        const now = new Date();
        const at = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}.${String(now.getMilliseconds()).padStart(3, '0')}`;
        const next = {
          lvl: 'D',
          at,
          tag: n % 2 ? 'ld2450' : 'sensor',
          text: n % 2 ? `Target 1 moved to (${-30 - n % 40}, ${200 + n % 60}) cm` : `'Temperature': Sending state 21.4${n % 10} °C`
        };
        return [...ls.slice(-60), next];
      });
    }, 3000);
    return () => clearInterval(t);
  }, [paused]);
  const shown = filter ? lines.filter(l => `${l.tag} ${l.text}`.toLowerCase().includes(filter.toLowerCase())) : lines;
  return <Card title="Logs" collapsible defaultOpen>
      <div className="log-bar">
        <input className="inp sm grow" type="search" placeholder={'Filter\u2026'} value={filter} onInput={e => setFilter((e.target as HTMLInputElement).value)} />
      </div>

      <div className="log">
        {shown.map((l, i) => <div key={i} className={LEVEL_TONE[l.lvl] || ''}>
            {`[${l.at}][${l.lvl}][${l.tag}] ${l.text}`}
          </div>)}
      </div>

      <div className="log-foot">
        <p className="log-count dim xs">{shown.length === lines.length ? `${lines.length} lines` : `${shown.length} of ${lines.length} lines`}</p>
        <button className={`btn sm${paused ? '' : ' on'}`} onClick={() => setPaused(!paused)}>
          {paused ? 'Resume' : 'Following'}
        </button>
        <button className="btn sm">Export</button>
      </div>
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Recovery                                                            */
/* ------------------------------------------------------------------ */

function RecoveryCards() {
  return <>
      <Card title="LD2450 Recovery" collapsible hint={HINTS.radar_recovery}>
        <Row label="Restart radar">
          <Confirm label="Restart" title="Restart the radar module?" body="Presence reads clear for a few seconds while it comes back. Nothing else on the device is touched." confirmLabel="Restart radar" onConfirm={() => {}} />
        </Row>
        <Row label="Factory reset radar">
          <Confirm label="Factory reset" danger title="Factory reset the radar?" body="Zones, thresholds and the detection range go back to the module defaults. This cannot be undone." confirmLabel="Reset radar" onConfirm={() => {}} />
        </Row>
      </Card>

      <Card title="XMOS Recovery" collapsible hint={HINTS.xmos}>
        <Row label="Restart XMOS">
          <Confirm label="Restart" title="Restart the XMOS?" body="Voice goes deaf for a few seconds while the audio processor reboots." confirmLabel="Restart XMOS" onConfirm={() => {}} />
        </Row>
        <Row label="Reflash XMOS firmware">
          <Confirm label="Reflash" danger title="Reflash the XMOS firmware?" body="Takes about a minute. Do not power the device off while the flash runs." confirmLabel="Reflash" onConfirm={() => {}} />
        </Row>
      </Card>

      <Card title="ESP32 Recovery" collapsible defaultOpen hint={HINTS.maintenance}>
        <Row label="Restart">
          <Confirm label="Restart" title="Restart the device?" body="The device is away for fifteen seconds or so. Music stops; timers keep counting." confirmLabel="Restart" onConfirm={() => {}} />
        </Row>
        <Row label="Safe mode" hint={HINTS.safe_mode}>
          <Confirm label="Safe mode" title="Reboot into safe mode?" body="Components stay disabled until the next normal boot - the escape hatch when something boot-loops." confirmLabel="Safe mode" onConfirm={() => {}} />
        </Row>
        <Row label="Factory reset" hint={HINTS.factory_reset}>
          <Confirm label="Factory reset" danger title="Factory reset the device?" body="Wi-Fi credentials, calibration and every setting are wiped. The device reboots into its setup hotspot." confirmLabel="Erase everything" onConfirm={() => {}} />
        </Row>
      </Card>
    </>;
}

/* ------------------------------------------------------------------ */
/* Community links                                                     */
/* ------------------------------------------------------------------ */

const I_BOOK = ni(<>
    <path d="M1.6 2.4h3.8a2.6 2.6 0 0 1 2.6 2.6v8.9a2 2 0 0 0-2-2H1.6z" />
    <path d="M14.4 2.4h-3.8A2.6 2.6 0 0 0 8 5v8.9a2 2 0 0 1 2-2h4.4z" />
  </>);
const I_GITHUB = ni(<>
    <path d="M10.67 14.67v-2.58a2.25 2.25 0 0 0-.63-1.74c2.09-.23 4.29-1.03 4.29-4.67a3.63 3.63 0 0 0-1-2.52 3.38 3.38 0 0 0-.06-2.51s-.79-.23-2.61.99a8.92 8.92 0 0 0-4.66 0c-1.82-1.22-2.61-.99-2.61-.99a3.38 3.38 0 0 0-.06 2.51 3.63 3.63 0 0 0-1 2.52c0 3.61 2.2 4.4 4.29 4.67a2.25 2.25 0 0 0-.62 1.73v2.59" />
    <path d="M6 12.67c-3.33 1-3.33-1.67-4.67-2" />
  </>);
const I_YOUTUBE = ni(<>
    <rect x="1.8" y="4" width="12.4" height="8.4" rx="2.6" />
    <path d="M6.9 6.6v3.2l3-1.6z" fill="currentColor" />
  </>);
const I_DISCORD = ni(<>
    <path d="M10.33 11.67l.67 1.33s2.78-.89 3.67-2.33c0-.67.35-5.43-2-7-1-.67-2.67-1-2.67-1l-.67 1.33h-1.33" />
    <path d="M5.69 11.67l-.67 1.33s-2.78-.89-3.67-2.33c0-.67-.35-5.43 2-7 1-.67 2.67-1 2.67-1l.67 1.33h1.33" />
    <circle cx="5.67" cy="8.33" r="1" fill="currentColor" stroke="none" />
    <circle cx="10.33" cy="8.33" r="1" fill="currentColor" stroke="none" />
  </>);
const COMMUNITY = [{
  label: 'Docs',
  url: 'https://docs.futureproofhomes.net/',
  icon: I_BOOK
}, {
  label: 'GitHub',
  url: 'https://github.com/FutureProofHomes',
  icon: I_GITHUB
}, {
  label: 'YouTube',
  url: 'https://www.youtube.com/@futureproofhomes',
  icon: I_YOUTUBE
}, {
  label: 'Discord',
  url: 'https://discord.futureproofhomes.net/',
  icon: I_DISCORD
}];
function CommunityLinks() {
  return <nav className="dx-links" aria-label="FutureProofHomes community links">
      {COMMUNITY.map(c => <a key={c.label} href={c.url} target="_blank" rel="noopener noreferrer">
          {c.icon}
          {c.label}
        </a>)}
    </nav>;
}

/* ------------------------------------------------------------------ */

export function DiagnosticsRoute() {
  return <>
      <DeviceCard />
      {/* Right under Device, per the owner: its Power gain mode row is decided by the USB-C
          Power Supply reading a few rows up. */}
      <SpeakerAmp />
      <CrashCard />
      <FirmwareCard />
      <LaunchCard />
      {/* Right under Launch, whose subject it shares: ways to reach this UI. */}
      <HomeAssistantCard />
      <LogsCard />
      <RecoveryCards />
      <CommunityLinks />
    </>;
}