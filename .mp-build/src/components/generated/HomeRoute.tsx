/**
 * The Home route: sensor pills with in-place calibration, the Assistant card, a live timer,
 * the LED ring with its colour wheel, and the physical button states.
 * Ported from routes/controls.jsx with local mock state in place of the device APIs.
 */
import React, { useEffect, useRef, useState } from 'react';
import { Arrow, Card, Empty, Hint, N_CHAT, Pill, Row, Select, Slider, Toggle } from './ui';
import { SPARKS, TRANSCRIPT } from './mock';
const HINTS = {
  temp: 'Measured inside the enclosure and compensated for self-heating, so it may differ from a thermometer beside it.',
  humidity: 'Relative humidity at the device. Calibrate against a trusted reference if it reads high or low.',
  lux: 'Ambient light at the front face. Wall shadows and lamp angles matter more than absolute accuracy.',
  calibrate: 'The offset is added to the raw reading. Set it so the corrected value matches a reference you trust.',
  temp_unit: 'Display only - the sensor publishes °C and the stored calibration stays °C either way.',
  led_ring: 'The ring around the top edge. Voice feedback animations override this colour while the assistant is active.',
  timers: 'Timers are set by voice and live on the device - they keep counting with Home Assistant gone. To cancel one, say "cancel the timer".',
  voice_override: 'How loud the assistant answers. Zero follows the media volume instead of overriding it.',
  finished_speaking: "How much trailing silence ends your turn. Home Assistant's own pipeline setting, one per satellite.",
  mute: 'Cuts the microphones in hardware. The ring turns red while they are muted.'
};

/* ------------------------------------------------------------------ */
/* Sensor pills, with calibration on the pill itself                   */
/* ------------------------------------------------------------------ */

function Editor({
  title,
  hint,
  raw,
  unit,
  digits,
  step,
  offset,
  onBump,
  onClose,
  extra
}: {
  title: string;
  hint: string;
  raw: number;
  unit: string;
  digits: number;
  step: number;
  offset: number;
  onBump: (d: number) => void;
  onClose: () => void;
  extra?: React.ReactNode;
}) {
  const box = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const away = (e: PointerEvent) => {
      const t = e.target as HTMLElement;
      if (!box.current?.contains(t) && !t.closest?.('.pills')) onClose();
    };
    document.addEventListener('pointerdown', away, true);
    return () => document.removeEventListener('pointerdown', away, true);
  }, [onClose]);
  return <div className="editor" ref={box}>
      <div className="row">
        <span className="grow strong">{title}</span>
        <Hint text={hint} />
      </div>
      <div className="row sm">
        <span className="grow dim">sensor reads</span>
        <span className="num">
          {raw.toFixed(digits)}
          {unit}
        </span>
      </div>
      <div className="row sm">
        <span className="dim">offset</span>
        <Hint text={HINTS.calibrate} />
        <span className="grow" />
        <button className="btn sq" onClick={() => onBump(-step)}>
          &minus;
        </button>
        <span className="num w44">
          {offset > 0 ? '+' : ''}
          {offset.toFixed(digits)}
        </span>
        <button className="btn sq" onClick={() => onBump(step)}>
          +
        </button>
      </div>
      {extra}
    </div>;
}
const SENSORS = [{
  id: 'temp',
  label: 'Temperature',
  title: 'Temperature',
  unit: '\u00B0C',
  digits: 1,
  step: 0.1,
  hint: HINTS.temp,
  base: 21.4
}, {
  id: 'hum',
  key: 'humidity',
  label: 'Humidity',
  title: 'Humidity',
  unit: ' %',
  digits: 0,
  step: 1,
  hint: HINTS.humidity,
  base: 46
}, {
  id: 'lux',
  label: 'Light',
  title: 'Ambient Light',
  unit: ' lx',
  digits: 0,
  step: 5,
  hint: HINTS.lux,
  base: 182
}];
function SensorPills({
  go
}: {
  go?: (id: string) => void;
}) {
  const [open, setOpen] = useState<string | null>(null);
  const [offsets, setOffsets] = useState<Record<string, number>>({
    temp: 0,
    hum: 0,
    lux: 0
  });
  const [isF, setIsF] = useState(false);
  const c2f = (c: number) => c * 9 / 5 + 32;
  const value = (s: (typeof SENSORS)[0]) => s.base + (offsets[s.id] || 0);
  const show = (s: (typeof SENSORS)[0]) => s.id === 'temp' && isF ? `${c2f(value(s)).toFixed(s.digits)}\u00B0F` : `${value(s).toFixed(s.digits)}${s.unit}`;
  const openRow = SENSORS.find(s => s.id === open);
  const tempF = isF && openRow?.id === 'temp';
  return <div>
      <div className="pills">
        {SENSORS.map(s => <Pill key={s.id} id={s.id} open={open} setOpen={setOpen} label={s.label} value={show(s)} spark={{
        pts: SPARKS[s.key || s.id] || SPARKS.temp,
        seed: `local:${s.id}`
      }} />)}
        {/* Still a link rather than a button, so a long-press keeps offering "open in new tab"
            - and now it actually goes: the arrow means "elsewhere", and elsewhere is Presence. */}
        <a className="pill" href="#/presence" title="Still — LD2450 settings" onClick={e => {
        e.preventDefault();
        go?.('presence');
      }}>
          <span className="pill-v">Still</span>
          <span className="pill-l">Presence</span>
          <Arrow cls="pill-c" />
        </a>
      </div>

      {openRow && <Editor title={openRow.title} hint={openRow.hint} raw={tempF ? c2f(openRow.base) : openRow.base} unit={tempF ? '\u00B0F' : openRow.unit} digits={openRow.digits} step={openRow.step} offset={tempF ? (offsets[openRow.id] || 0) * 1.8 : offsets[openRow.id] || 0} onBump={d => setOffsets(o => ({
      ...o,
      [openRow.id]: Number(((o[openRow.id] || 0) + d).toFixed(2))
    }))} onClose={() => setOpen(null)} extra={openRow.id === 'temp' ? <div className="row sm">
                <span className="dim">Fahrenheit</span>
                <Hint text={HINTS.temp_unit} />
                <span className="grow" />
                <Toggle checked={isF} onChange={setIsF} />
              </div> : null} />}
    </div>;
}

/* ------------------------------------------------------------------ */
/* Colour wheel                                                        */
/* ------------------------------------------------------------------ */

function Wheel({
  hue,
  sat,
  onPick
}: {
  hue: number;
  sat: number;
  onPick: (h: number, s: number) => void;
}) {
  const R = 58;
  const rad = hue * Math.PI / 180;
  const x = R + Math.cos(rad) * sat * (R - 8);
  const y = R + Math.sin(rad) * sat * (R - 8);
  const pick = (e: React.PointerEvent<HTMLDivElement>) => {
    const b = e.currentTarget.getBoundingClientRect();
    const dx = e.clientX - b.left - R;
    const dy = e.clientY - b.top - R;
    const dist = Math.min(Math.hypot(dx, dy) / (R - 8), 1);
    let deg = Math.atan2(dy, dx) * 180 / Math.PI;
    if (deg < 0) deg += 360;
    onPick(Math.round(deg), dist);
  };
  return <div className="wheel-wrap">
      <div className="wheel" onPointerDown={pick}>
        <span className="wheel-dot" style={{
        left: `${x - 6}px`,
        top: `${y - 6}px`,
        background: `hsl(${hue} ${Math.round(sat * 100)}% 50%)`
      }} />
      </div>
    </div>;
}
function Leds() {
  const [on, setOn] = useState(true);
  const [bright, setBright] = useState(62);
  const [hue, setHue] = useState(196);
  const [sat, setSat] = useState(0.85);
  return <Card title="LED ring" hint={HINTS.led_ring}>
      <Row label="Power">
        <Toggle checked={on} onChange={setOn} />
      </Row>
      <Row label="Brightness">
        <Slider value={bright} min={1} max={100} step={1} disabled={!on} format={v => `${v}%`} onCommit={setBright} />
      </Row>
      <Wheel hue={hue} sat={sat} onPick={(h, s) => {
      setHue(h);
      setSat(s);
      setOn(true);
    }} />
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Timer and the Assistant card                                        */
/* ------------------------------------------------------------------ */

const mmss = (s: number) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
function Timers() {
  const [left, setLeft] = useState(9 * 60 + 41);
  useEffect(() => {
    const t = setInterval(() => setLeft(v => v > 0 ? v - 1 : 10 * 60), 1000);
    return () => clearInterval(t);
  }, []);
  return <Card title="Timer" hint={HINTS.timers}>
      <div className="ctl">
        <div className="ctl-label">
          <span>10 min timer</span>
        </div>
        <div className="ctl-body">
          <span className="num lg">{mmss(left)}</span>
        </div>
      </div>
    </Card>;
}
function VoiceStatus() {
  const [vol, setVol] = useState(0);
  const [fsd, setFsd] = useState('default');
  const [mute, setMute] = useState(false);
  const lines = TRANSCRIPT;
  return <Card title="Assistant" icon={N_CHAT} right={<span className="dim xs">Idle</span>}>
      <div className="transcript">
        {lines.length ? lines.slice().reverse().map((l, i) => <p key={i} className={`utt${l.heard ? ' heard' : ''}`}>
                <span className="utt-who">{l.heard ? 'User:' : 'Assist:'}</span>
                {l.text}
              </p>) : <Empty icon={<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinejoin="round" aria-hidden="true">
                <path d="M3 3h10a1.5 1.5 0 0 1 1.5 1.5V9A1.5 1.5 0 0 1 13 10.5H8.2L5 13.2v-2.7H3A1.5 1.5 0 0 1 1.5 9V4.5 A1.5 1.5 0 0 1 3 3Z" />
              </svg>} text="Nothing has been said since the device started." />}
      </div>

      <Row label="Voice Volume Override" hint={HINTS.voice_override}>
        <Slider value={vol} min={0} max={100} step={1} format={v => v === 0 ? 'follow media' : `${v}%`} onCommit={setVol} />
      </Row>

      <Row label="Finished speaking detection" hint={HINTS.finished_speaking}>
        <Select value={fsd} options={[['aggressive', 'Aggressive'], ['default', 'Default'], ['relaxed', 'Relaxed']]} onChange={setFsd} />
      </Row>

      <Row label="Mute microphones" hint={HINTS.mute}>
        <Toggle checked={mute} onChange={setMute} />
      </Row>
    </Card>;
}

/* ------------------------------------------------------------------ */
/* Physical buttons                                                    */
/* ------------------------------------------------------------------ */

const BUTTONS: [string, string?][] = [['Volume up'], ['Volume down'], ['Mute', 'warn'], ['Action']];
function Buttons() {
  const [pressed, setPressed] = useState<string | null>(null);

  // A little life: the Action button "presses itself" now and then, the way the real card lights
  // when a finger lands on the device.
  useEffect(() => {
    const t = setInterval(() => {
      setPressed('Action');
      setTimeout(() => setPressed(null), 900);
    }, 7000);
    return () => clearInterval(t);
  }, []);
  return <Card title="Buttons" right={pressed && <span className="dim xs">last: single press</span>}>
      <div className="btnstates">
        {BUTTONS.map(([label, tone]) => <span key={label} className={`bstate${tone ? ` ${tone}` : ''}${pressed === label ? ' on' : ''}`}>
            {label}
          </span>)}
      </div>
      <p className="dim xs">Press a button on the device; it lights up here.</p>
    </Card>;
}

/* ------------------------------------------------------------------ */

export function HomeRoute({
  go
}: {
  go?: (id: string) => void;
}) {
  return <>
      <Card>
        <SensorPills go={go} />
      </Card>
      <VoiceStatus />
      <Timers />
      <Leds />
      <Buttons />
    </>;
}