import { useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { HINTS, PRESENCE, TEXT } from '../copy.js';
import { hasTargets } from '../lib/audio.js';
import { ASSIST_SLOTS, deviceIdentity, entity, haBlocked, haTooOld, NO_WAKE_WORD, pathFor, PIPELINE_PREFERRED, post, request, useVoice } from '../lib/device.js';
import { onTips, tipDone, tipsDone } from '../lib/tips.js';
import { sparkPoints } from '../lib/sparkhist.js';
import { sparkPaths } from '../lib/sparkline.js';
import type { Ctx, Orb } from '../ctx';
import { Activity, ArrowUp, Check, ChevronDown, Clock, Plus, X } from '../icons';
import { agentName, clock, DEFAULT_AGENT, fitBytes, isOn, lineId, offsetSpec, offsetText, orbState, orbTips, pipelineAgent, readAgentMap, reading, rememberAgent, savedAgent, setupStamp, stepOffset, timerLabel, timerLeft, transcriptRows, transcriptWindows } from '../lib/orb.js';
import { HintBtn } from './bits';
import { Switch } from './controls';
import { useHeld, VoiceOrb } from './VoiceOrb';
import { useWakeWords } from './WakeTab';
import { Drawer, Presence } from './Drawer';
type WakeWords = ReturnType<typeof useWakeWords>;
type Sensor = {
  id: string;
  key: string;
  offsetKey: string;
  label: string;
  unit: string;
  digits: number;
  step: number;
  min: number;
  max: number;
  hint: string;
};
/** GET /api/sat1/voice's timer and transcript rows (web_ui_handler.cpp handle_voice_). */
type Timer = {
  id: string;
  name: string;
  total: number;
  left: number;
  active: boolean;
};
type Line = {
  heard: boolean;
  at: number;
  w: string;
  text: string;
};
/** transcriptWindows' answer (src/lib/orb.js). */
type Windows = {
  newest: number | null;
  cue: {
    index: number;
    key: string;
  } | null;
  windows: {
    word: string;
    lines: Line[];
  }[];
};

/**
 * The Home sensors, each with the number entity that calibrates it. `step` is the display step: the
 * offset stepper's floor and the sparkline's backfill amplitude. `min`/`max` stand in for a number
 * payload without its range.
 */
const SENSORS: Sensor[] = [{
  id: 'temp',
  key: 'temp',
  offsetKey: 'temp_offset',
  label: 'Temperature',
  unit: '\u00B0C',
  digits: 1,
  step: 0.1,
  min: -20,
  max: 20,
  hint: HINTS.temp
}, {
  id: 'humidity',
  key: 'humidity',
  offsetKey: 'humidity_offset',
  label: 'Humidity',
  unit: '%',
  digits: 0,
  step: 1,
  min: -50,
  max: 50,
  hint: HINTS.humidity
}, {
  id: 'light',
  key: 'lux',
  offsetKey: 'lux_offset',
  label: 'Light',
  unit: ' lx',
  digits: 0,
  step: 5,
  min: -500,
  max: 500,
  hint: HINTS.lux
}];
type SparkData = {
  pts: [number, number][];
  amp: number;
  seed: string;
};
/**
 * The area chart in a chip's bottom band: decoration under the reading, never a control; the maths
 * is in src/lib/sparkline.js. One look for all three chips (owner decision, September 2026). The
 * fill is the line's own colour faded through stop-opacity to transparent rather than a pale tint,
 * which sat invisible on the light theme's near-white chip and heavy on the dark theme's - one colour
 * at one low opacity reads the same against both. Both layers stay well under full opacity (user
 * request, September 2026: "somewhat faded behind the text"), so the reading stays the loudest thing
 * in the chip. Gradient ids are document-global, hence one per device and sensor; non-scaling-stroke
 * because preserveAspectRatio="none" would otherwise smear the line into different widths on the
 * two axes.
 */
function Spark({
  pts,
  amp,
  seed
}: SparkData) {
  const made = sparkPaths(pts, amp, seed);
  if (!made) return null;
  const gid = `spkg-${seed}`.replace(/[^a-zA-Z0-9-]/g, '-');
  return <svg className="spark" viewBox="0 0 100 100" preserveAspectRatio="none" pointerEvents="none" aria-hidden="true"><defs><linearGradient id={gid} x1="0" y1="0" x2="0" y2="1"><stop offset="0" stopColor="var(--orb-a)" stopOpacity="0.22" /><stop offset="1" stopColor="var(--orb-a)" stopOpacity="0" /></linearGradient></defs><path d={made.fill} fill={`url(#${gid})`} stroke="none" /><path d={made.line} fill="none" stroke="var(--orb-a)" strokeOpacity={0.5} strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke" /></svg>;
}
function DrawerSpark({
  pts,
  amp,
  seed
}: SparkData) {
  const made = sparkPaths(pts, amp, seed);
  if (!made) return null;
  return <div className="spark"><svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true"><path d={made.line} /></svg></div>;
}
function SensorDrawer({
  label,
  onClose,
  children
}: {
  label: string;
  onClose: () => void;
  children: React.ReactNode;
}) {
  return <Drawer label={label} onClose={onClose} className="sensor-sheet">{children}</Drawer>;
}

/**
 * The calibration offset: a number entity whose value the sensor's filter adds, so the reading the
 * sensor publishes is already corrected. It is stepped in the entity's native unit (°C for
 * temperature) whatever the display shows, so what is stored stays a clean multiple of the entity's
 * step. Every press writes; the pressed value holds on screen until the device echoes it, so quick
 * presses step on from each other rather than from a stale value.
 */
function useOffset(ctx: Ctx, s: Sensor) {
  const off = entity(ctx, s.offsetKey);
  const spec = offsetSpec(off, s);
  const [offset, hold] = useHeld(Number(off?.value) || 0, spec.step / 2);
  const bump = (dir: 1 | -1) => {
    const next = stepOffset(offset, dir, spec.step, spec.min, spec.max);
    if (next === offset) return;
    hold(next);
    post(`${pathFor(ctx, s.offsetKey, 'set')}?value=${next}`);
  };
  return {
    offset,
    bump,
    atMin: offset <= spec.min,
    atMax: offset >= spec.max
  };
}
function TempPopup({
  ctx,
  s,
  value,
  spark,
  close
}: {
  ctx: Ctx;
  s: Sensor;
  value: unknown;
  spark: SparkData;
  close: () => void;
}) {
  const {
    offset,
    bump,
    atMin,
    atMax
  } = useOffset(ctx, s);
  // The unit preference is an internal ESPHome switch rather than browser storage, so a wall tablet
  // and a phone agree. Display-only: the sensor publishes °C and the offset stores °C whatever it
  // says, so flipping it can never drift the calibration. Absent on older firmware, where no toggle
  // renders and everything stays °C.
  const unitF = entity(ctx, 'temp_unit_f');
  const isF = isOn(unitF);
  return <div className="sensor-drawer-body temp-pop" aria-label="Temperature settings"><div><span className="eyebrow">CALIBRATION · Temperature</span><strong>{reading(value, s.digits, s.unit, isF)}</strong></div><p className="muted cal-hint">{s.hint}</p><DrawerSpark {...spark} /><div className="temp-row"><span>Offset</span><div className="stepper"><button aria-label="Decrease offset" disabled={atMin} onClick={() => bump(-1)}>−</button><b>{offsetText(offset, s.digits, '°', isF)}</b><button aria-label="Increase offset" disabled={atMax} onClick={() => bump(1)}>+</button></div></div>{unitF && <div className="temp-row"><span>Fahrenheit <HintBtn text={HINTS.temp_unit} /></span><Switch on={isF} label="Use Fahrenheit" onChange={v => post(pathFor(ctx, 'temp_unit_f', v ? 'turn_on' : 'turn_off'))} /></div>}<div className="cal-actions"><button className="done" onClick={close}>Done</button></div></div>;
}
function Calibration({
  ctx,
  s,
  value,
  spark,
  close
}: {
  ctx: Ctx;
  s: Sensor;
  value: unknown;
  spark: SparkData;
  close: () => void;
}) {
  const {
    offset,
    bump,
    atMin,
    atMax
  } = useOffset(ctx, s);
  return <div className="sensor-drawer-body"><div><span className="eyebrow">CALIBRATION · {s.label}</span><strong>{reading(value, s.digits, s.unit)}</strong></div><p className="muted cal-hint">{s.hint}</p><DrawerSpark {...spark} /><div className="cal-actions"><button aria-label="Decrease offset" disabled={atMin} onClick={() => bump(-1)}>−</button><span>offset {offsetText(offset, s.digits, s.unit)}</span><button aria-label="Increase offset" disabled={atMax} onClick={() => bump(1)}>+</button><button className="done" onClick={close}>Done</button></div></div>;
}

/**
 * Temperature, humidity and light, each opening its calibration drawer when the build has the
 * offset entity behind it, and the radar's presence, which leaves for the Presence tab.
 */
function SensorPills({
  ctx
}: {
  ctx: Ctx;
}) {
  const [expanded, setExpanded] = useState<string | null>(null);
  // The one paint before /api/sat1/state answers: chip-shaped shimmers hold the row's geometry so
  // the readings land in place. Gated on the device payload, not on the rows - a device with no
  // sensors at all should show its truthful nothing, not shimmer forever.
  if (!ctx.device) return <div className="pills">{[0, 1, 2, 3].map(i => <div key={i}><div className="sensor skel" aria-hidden="true"><strong>&nbsp;</strong><small>&nbsp;</small><ChevronDown size={10} className="sensor-caret" /></div></div>)}</div>;
  const isF = isOn(entity(ctx, 'temp_unit_f'));
  const mac = String(ctx.device.mac || 'local').toLowerCase();
  // Referenced by id: satellite1_radar registers it at runtime from a C++ literal the LD2450 and
  // LD2410 handlers share, so its name is owned by code rather than anyone's YAML and it has no
  // config id for the entity map to point at.
  const presence = ctx.states['text_sensor/Radar Target'];
  const module = entity(ctx, 'radar_module');
  // The presence chip is a real link and only a plain left click is routed in-app, so a long-press
  // or modifier-click still opens a new tab - someone comparing the plot against what they can see
  // in the room wants both at once. Its title carries the firmware's full wording; the chip shows
  // the short form.
  const close = () => setExpanded(null);
  return <div className="pills">{SENSORS.map(s => {
      const sensor = entity(ctx, s.key);
      if (!sensor) return null;
      const editable = !!entity(ctx, s.offsetKey);
      const spark: SparkData = {
        pts: sparkPoints(s.key),
        amp: s.step,
        seed: `${mac}:${s.key}`
      };
      const val = reading(sensor.value, 0, s.unit, s.id === 'temp' && isF);
      const face = <><Spark {...spark} /><strong>{val}</strong><small>{s.label}</small></>;
      return <div key={s.id}>{editable ? <button className={'sensor ' + (expanded === s.id ? 'active' : '')} aria-haspopup="dialog" aria-expanded={expanded === s.id} onClick={() => {
          tipDone('sensors');
          setExpanded(expanded === s.id ? null : s.id);
        }}>{face}<ChevronDown size={10} className="sensor-caret" aria-hidden="true" /></button> : <div className="sensor">{face}</div>}
        <Presence>{editable && expanded === s.id && <SensorDrawer label={`${s.label} calibration`} onClose={close}>{s.id === 'temp' ? <TempPopup ctx={ctx} s={s} value={sensor.value} spark={spark} close={close} /> : <Calibration ctx={ctx} s={s} value={sensor.value} spark={spark} close={close} />}</SensorDrawer>}</Presence></div>;
    })}{presence && <div><a className="sensor" href="#/presence" title={[presence.value, module?.value ? `${module.value} settings` : 'Presence'].filter(Boolean).join(' \u2014 ')} onClick={e => {
        tipDone('sensors');
        if (e.button !== 0 || e.metaKey || e.ctrlKey || e.shiftKey) return;
        e.preventDefault();
        ctx.go('PRESENCE');
      }} style={{
        textDecoration: 'none'
      }}><strong>{PRESENCE[presence.value as keyof typeof PRESENCE] || presence.value || '\u2014'}</strong><small>Presence ›</small></a></div>}</div>;
}

/** How long after its arrival a bubble may still start its pop-in; longer than the animation runs. */
const BUBBLE_POP_MS = 1000;

/** How long the voice poll runs fast after an orb tap. */
const TAP_FAST_MS = 6000;

/**
 * When each line first reached the page, by lineId: 0 for those already there when the transcript
 * first loaded, so only a line that arrives while the page is open pops in. A tab switch remounts
 * its bubbles, and they stay still because their arrival is long past by then.
 */
function useArrivals(lines: Line[] | undefined) {
  const seen = useRef<Map<string, number> | null>(null);
  if (!lines) return seen.current;
  const now = Date.now();
  const prior = seen.current;
  const next = new Map<string, number>();
  for (const l of lines) {
    const id = lineId(l);
    next.set(id, prior ? prior.get(id) ?? now : 0);
  }
  seen.current = next;
  return next;
}

/**
 * One window's conversation, laid out as iMessage lays out a conversation (owner request, October
 * 2026): newest at the bottom, a short conversation sitting down there too, bubbles grouped by
 * speaker with the tail on each group's last, time headers, and new bubbles popping in. That way a
 * misheard command is visible without opening the log. The empty state sits inside the same box,
 * so it does not change shape the first time something is said. The box follows new lines unless
 * the person has scrolled up to read. While the assistant works on an answer, iMessage's typing
 * bubble holds its place in the conversation being answered (`own`) - for a typed message, once its
 * bubble is in.
 */
function Conversation({
  lines,
  polled,
  own,
  thinking,
  asking,
  boot,
  paged
}: {
  lines: Line[];
  /** Every line the device sent, which is what tells a bubble that just arrived. */
  polled: Line[] | undefined;
  own: boolean;
  thinking: boolean;
  asking: boolean;
  boot: number | null;
  /** There is another window to swipe to, which the empty state says. */
  paged: boolean;
}) {
  const arrived = useArrivals(polled);
  const box = useRef<HTMLElement>(null);
  const stuck = useRef(true);
  const last = lines[lines.length - 1];
  const typing = own && (thinking || asking && !!last?.heard);
  // Sending is reading the bottom of the conversation again, wherever the box was scrolled to.
  if (asking) stuck.current = true;
  const now = Date.now();
  const rows = transcriptRows(lines, boot, typing, now);
  useLayoutEffect(() => {
    const el = box.current;
    if (el && stuck.current) el.scrollTop = el.scrollHeight;
  }, [rows.length, last?.text, typing]);
  // A box that shrinks keeps its scrollTop and fires no scroll event, so the newest line would slide
  // out of view whenever the header, the pills or the window change the space the box gets.
  useEffect(() => {
    const el = box.current;
    if (!el) return;
    const ro = new ResizeObserver(() => {
      if (stuck.current) el.scrollTop = el.scrollHeight;
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);
  return <section ref={box} className="tt-scroll" onScroll={e => {
    const el = e.currentTarget;
    stuck.current = el.scrollHeight - el.scrollTop - el.clientHeight < 32;
  }}>{rows.length ? rows.map(r => {
      if ('stamp' in r) return <time key={r.key} className="tstamp" dateTime={new Date(r.ms).toISOString()}><b>{r.stamp.day}</b> {r.stamp.time}</time>;
      const run = r.run ? ' run' : '';
      if ('typing' in r) return <p key={r.key} className={'assistant typing in' + run} aria-hidden="true"><i /><i /><i /></p>;
      const at = arrived?.get(r.id) || 0;
      const popping = at && now - at < BUBBLE_POP_MS ? ' in' : '';
      return <p key={r.key} className={(r.line.heard ? 'user' : 'assistant') + run + (r.tail ? ' tail' : '') + popping}>{r.line.text}</p>;
    }) : <p className="transcript-empty">{paged ? `${TEXT.nothing_said} ${TEXT.tt_swipe}` : TEXT.nothing_said}</p>}</section>;
}

/** A word as a window's title shows it: in quotes. */
const tabLabel = (w: string) => `“${w}”`;

/**
 * A window's title bar, frosted over the conversation scrolling under it: the slot's word, which
 * opens the wake word picker, and the word's tuning - "Tuned", or "Tune it" in amber - which opens
 * the tuner. Those are the Wake tab's own drawers, through useWakeWords. Until the device's slots
 * are read, Home Assistant's word stands in, as a title only.
 */
function WakeHead({
  ww,
  i,
  word
}: {
  ww: WakeWords;
  i: number;
  word: string;
}) {
  if (!ww.wake) return <header className="tt-head"><span className="tt-word plain">{word ? tabLabel(word) : TEXT.tt_empty}</span></header>;
  const slot = ww.slotAt(i);
  const swap = ww.swaps[i];
  const swapping = swap?.phase === 'busy';
  const failed = swap?.phase === 'error';
  const tuned = !swapping && (slot?.cut || 0) > 0;
  const tuneText = (tuned ? TEXT.tt_tuned : TEXT.tt_untuned).replace('%s', word);
  return <header className="tt-head">
    <button type="button" className={'tt-word' + (word ? '' : ' empty') + (failed ? ' err' : '')} disabled={swapping} aria-haspopup="dialog" aria-label={word ? `${TEXT.tt_change}, ${tabLabel(word)}` : TEXT.tt_add} title={failed ? TEXT.ww_failed : undefined} onClick={() => {
      tipDone('pick');
      ww.setPicking(i);
    }}><span>{word ? tabLabel(word) : TEXT.tt_empty}</span>{swapping ? <i className="tt-busy" aria-label={TEXT.ww_loading} /> : <ChevronDown size={12} strokeWidth={2.4} aria-hidden="true" />}</button>
    {!!word && !swapping && !!slot?.ld && <button type="button" className={'tt-tune' + (tuned ? ' tuned' : '')} aria-haspopup="dialog" aria-label={tuneText} title={tuneText} onClick={() => ww.setTuning({
      i,
      word,
      isStop: false,
      quick: tuned
    })}><span className="tt-cap"><Activity size={11} strokeWidth={2.4} aria-hidden="true" />{tuned ? TEXT.tt_tuned_short : TEXT.tt_tune_short}</span></button>}
  </header>;
}

/**
 * The wake word slots' windows side by side in a strip the person swipes through, on the native
 * scroll's own momentum and snap (owner's design, October 2026: two windows rather than two tabs).
 * The window off to the side recedes - smaller, dimmer - in step with the finger, and peeks in at
 * the edge so there is plainly something there; the page dots under the strip stretch into a pill
 * as it moves. Tapping the peeking window or a dot, or tabbing into it, brings it forward, and so
 * does `view` changing from outside, on the browser's smooth scroll. The scroll drives the look
 * through CSS variables set straight on the elements, so a swipe never re-renders the page, and
 * `view` follows only once the strip comes to rest. `cue` lights a dot: something new was said in
 * that window while the person was writing in the other.
 */
function Pager({
  view,
  onView,
  labels,
  cue,
  cards
}: {
  view: number;
  onView: (i: number) => void;
  labels: string[];
  cue: number | null;
  cards: React.ReactNode[];
}) {
  const root = useRef<HTMLDivElement>(null);
  const track = useRef<HTMLDivElement>(null);
  const viewRef = useRef(view);
  viewRef.current = view;
  const onViewRef = useRef(onView);
  onViewRef.current = onView;
  // A finger on the strip: nothing from outside moves it, and where it lands is decided on release.
  const held = useRef(false);
  const rest = useRef(0);
  const placed = useRef(false);
  const n = cards.length;
  const span = () => {
    const el = track.current;
    return el ? el.scrollWidth - el.clientWidth : 0;
  };
  const progress = () => {
    const el = track.current;
    const max = span();
    return el && max > 0 ? el.scrollLeft / max * (n - 1) : 0;
  };
  const paint = () => {
    const el = track.current;
    if (!el) return;
    const at = progress();
    root.current?.style.setProperty('--p', at.toFixed(4));
    Array.from(el.children).forEach((page, i) => {
      const card = page.firstElementChild as HTMLElement | null;
      if (!card) return;
      card.style.setProperty('--k', Math.min(1, Math.abs(at - i)).toFixed(4));
      card.style.transformOrigin = i < at ? '100% 50%' : '0% 50%';
    });
  };
  const land = () => {
    const i = Math.round(progress());
    if (!held.current && i !== viewRef.current) onViewRef.current(i);
  };
  useLayoutEffect(() => {
    const el = track.current;
    if (!el || held.current) return;
    const left = n > 1 ? view / (n - 1) * span() : 0;
    if (Math.abs(el.scrollLeft - left) >= 2) {
      const still = !placed.current || window.matchMedia('(prefers-reduced-motion: reduce)').matches;
      el.scrollTo({
        left,
        behavior: still ? 'auto' : 'smooth'
      });
    }
    placed.current = true;
    paint();
  }, [view, n]);
  useEffect(() => {
    const el = track.current;
    if (!el) return;
    // A new width (a phone turned, the window resized) keeps the open window where it was.
    const ro = new ResizeObserver(() => {
      if (!held.current) el.scrollLeft = n > 1 ? viewRef.current / (n - 1) * span() : 0;
      paint();
    });
    ro.observe(el);
    el.addEventListener('scrollend', land);
    return () => {
      ro.disconnect();
      el.removeEventListener('scrollend', land);
      clearTimeout(rest.current);
    };
  }, [n]);
  const release = () => {
    held.current = false;
    clearTimeout(rest.current);
    rest.current = window.setTimeout(land, 140);
  };
  return <div ref={root} className="tt-pager">
    <div ref={track} className="tt-track" role="region" aria-roledescription="carousel" aria-label={TEXT.tt_label} onScroll={() => {
      requestAnimationFrame(paint);
      clearTimeout(rest.current);
      rest.current = window.setTimeout(land, 140);
    }} onTouchStart={() => {
      held.current = true;
    }} onTouchEnd={release} onTouchCancel={release} onKeyDown={e => {
      if ((e.target as HTMLElement).closest('input, select, textarea')) return;
      const to = e.key === 'ArrowRight' ? view + 1 : e.key === 'ArrowLeft' ? view - 1 : -1;
      if (to < 0 || to >= n) return;
      e.preventDefault();
      onView(to);
    }}>{cards.map((card, i) => <div key={i} className="tt-page"><article className={'transcript tt-card' + (i === view ? ' on' : '')} role="group" aria-roledescription="slide" aria-label={labels[i]} aria-current={i === view || undefined} onClick={() => i !== viewRef.current && onView(i)} onFocusCapture={() => i !== viewRef.current && onView(i)}>{card}</article></div>)}</div>
    {n > 1 && <div className="tt-dots">{labels.map((label, i) => <button key={i} type="button" className={'tt-dot' + (i === view ? ' on' : '') + (i === cue ? ' new' : '')} aria-label={label} aria-current={i === view || undefined} onClick={() => onView(i)} />)}</div>}
  </div>;
}

/**
 * The message field at the bottom of a wake word's window (owner request, October 2026), typing to
 * that word, where the window shows the exchange like a spoken one. Its menu is
 * the word's voice pipeline (owner's design, October 2026) - the one Home Assistant runs when the
 * word is said or the orb tapped, set here as on the Wake tab. A typed message cannot go through a
 * pipeline (common/web_ui_assist.yaml), so it goes to the pipeline's conversation agent, and since
 * Home Assistant will not say which that is, pipelineAgent works it out from the pipeline's name
 * or the person's answer. When it cannot, the field is read-only and tapping it asks, in a drawer
 * (AgentDrawer), before any keyboard comes up; the answer is kept on the device for that pipeline
 * (pipeline_agents) and the field takes focus with it. The menu's last entry asks again.
 *
 * `status` is the device's word on the last message (web_ui_ask_status). A failure is shown only
 * for a message this page sent, because the status outlives the page and an old failure is
 * nobody's news. The draft clears on send, so a failure says what happened rather than offering
 * the text back, and goes once the person starts the next message. The menus are native selects
 * laid over their chips, so a phone opens its own picker. `onEngage` reports focus and typing, so
 * the open tab - who this goes to - holds still while a message is being written.
 */
function Composer({
  ctx,
  status,
  word,
  pipeline,
  pipe,
  keep,
  onEngage
}: {
  ctx: Ctx;
  status: string;
  word: string;
  /** The pipeline the open tab's word runs, or null before Home Assistant's slots are known. */
  pipeline: string | null;
  /** The menu that changes it, for a word the device listens for. */
  pipe: {
    value: string;
    options: [string, string][];
    busy: boolean;
    onPick: (v: string) => void;
  } | null;
  /** The pipelines the slots use now, whose answers are the last to make room for new ones. */
  keep: string[];
  onEngage: (engaged: boolean) => void;
}) {
  const [draft, setDraft] = useState('');
  // The status this page's last send found, and whether it has moved since - until it does, what
  // the status says is about some earlier message.
  const sent = useRef<{
    from: string;
    moved: boolean;
  } | null>(null);
  // The device refused the message outright, so no status will ever say anything about it.
  const [refused, setRefused] = useState(false);
  const mapEnt = entity(ctx, 'pipeline_agents');
  const stored = String(mapEnt?.state ?? mapEnt?.value ?? '');
  // An answer just given, shown until the device's own value catches up with it.
  const [given, setGiven] = useState<string | null>(null);
  useEffect(() => {
    if (given !== null && given === stored) setGiven(null);
  }, [given, stored]);
  // Why the agent question is open: to start typing, or to change the answer from the menu.
  const [choosing, setChoosing] = useState<'type' | 'change' | null>(null);
  useEffect(() => setChoosing(null), [pipeline]);
  const field = useRef<HTMLInputElement>(null);
  const cv: [string, string][] = Array.isArray(ctx.ha?.d?.cv) ? ctx.ha.d.cv : [];
  const known = readAgentMap(given ?? stored);
  const stamp = setupStamp(ctx.ha?.d?.asst?.o, cv);
  const agent = pipeline === null ? null : pipelineAgent(pipeline, cv, known, stamp);
  // An answer Home Assistant's setup has changed under: asked again, with that answer ticked.
  const prior = pipeline === null ? null : savedAgent(known, pipeline);
  const changed = agent === null && !!prior && prior.stamp !== stamp;
  const pipeName = pipeline === PIPELINE_PREFERRED ? TEXT.pipeline_preferred : pipeline || '';
  const blocked = !ctx.device?.ha ? TEXT.ask_no_ha : haBlocked(ctx.ha) ? TEXT.ask_blocked : haTooOld(ctx.ha) ? TEXT.ask_too_old : null;
  const busy = status === 'busy';
  if (sent.current && (busy || status !== sent.current.from)) sent.current.moved = true;
  const failed = refused ? TEXT.ask_err : sent.current?.moved && !busy ? ASK_FAILED[status] : undefined;
  const name = agent ? agentName(cv, agent) : '';
  const unknown = !blocked && pipeline !== null && agent === null;
  const choose = (how: 'type' | 'change') => {
    onEngage(true);
    setChoosing(how);
  };
  const dismiss = () => {
    setChoosing(null);
    onEngage(draft !== '');
  };
  const answer = (id: string) => {
    const how = choosing;
    setChoosing(null);
    if (pipeline !== null && id !== agent) {
      const next = rememberAgent(known, pipeline, id, stamp, keep);
      if (next !== null) {
        setGiven(next);
        request(pathFor(ctx, 'pipeline_agents', 'set'), {
          method: 'POST',
          body: new URLSearchParams({
            value: next
          })
        }).catch(() => {});
      }
    }
    // A phone raises the keyboard only for focus given inside the tap itself, so the field is made
    // writable and focused here rather than after the next render.
    if (how === 'type' && field.current) {
      field.current.readOnly = false;
      field.current.focus();
    } else onEngage(draft !== '');
  };
  const send = (e: {
    preventDefault: () => void;
  }) => {
    e.preventDefault();
    const text = draft.trim();
    if (!text || busy || blocked || !agent) return;
    tipDone('type');
    sent.current = {
      from: status,
      moved: false
    };
    setRefused(false);
    request('/api/sat1/ask', {
      method: 'POST',
      quiet: true,
      body: new URLSearchParams({
        t: text,
        a: agent === DEFAULT_AGENT ? '' : agent,
        w: word
      })
    }).then((r: {
      ok: boolean;
    }) => setRefused(!r.ok), () => setRefused(true));
    setDraft('');
  };
  const label = TEXT.ask_label.replace('%s', name || pipeName || TEXT.ask_default_agent);
  const agents: [string, string][] = [...(cv.some(r => r[0] === DEFAULT_AGENT) ? [] : [[DEFAULT_AGENT, TEXT.ask_default_agent] as [string, string]]), ...cv.map(r => [r[0], r[1] || r[0]] as [string, string])];
  return <div className="composer">
    <Presence>{choosing !== null && pipeline !== null && <AgentDrawer pipelineName={pipeline === PIPELINE_PREFERRED ? TEXT.pipeline_preferred : `“${pipeline}”`} agents={agents} value={agent ?? (changed && prior ? prior.agent : null)} changed={changed} onPick={answer} onClose={dismiss} />}</Presence>
    <form className={'composer-row' + (blocked ? ' off' : '')} onSubmit={send}>
      {pipe && <label className="composer-agent"><select value={pipe.value} aria-label={TEXT.vp_label} disabled={!!blocked || pipe.busy} onChange={e => {
          const v = e.currentTarget.value;
          if (v === REASK) {
            e.currentTarget.value = pipe.value;
            choose('change');
          } else pipe.onPick(v);
        }}>{pipe.options.map(([id, label]) => <option key={id} value={id}>{label}</option>)}{agent && <option value={REASK}>{TEXT.ask_reask.replace('%s', name)}</option>}</select><span aria-hidden="true">{pipeName}</span><ChevronDown size={10} aria-hidden="true" /></label>}
      <input type="text" enterKeyHint="send" autoComplete="off" maxLength={ASK_MAX_BYTES} value={draft} disabled={!!blocked} readOnly={unknown} aria-haspopup={unknown ? 'dialog' : undefined} placeholder={TEXT.ask_placeholder} aria-label={label} ref={field} onClick={() => unknown && choose('type')} onKeyDown={e => {
        if (unknown && (e.key === 'Enter' || e.key.length === 1)) {
          e.preventDefault();
          choose('type');
        }
      }} onFocus={() => onEngage(true)} onBlur={() => choosing === null && onEngage(draft !== '')} onInput={e => {
        if (!busy) {
          sent.current = null;
          setRefused(false);
        }
        const next = fitBytes(e.currentTarget.value, ASK_MAX_BYTES);
        setDraft(next);
        onEngage(true);
      }} />
      <button type="submit" className="composer-send" disabled={!draft.trim() || busy || !!blocked || !agent} aria-label={TEXT.ask_send}><ArrowUp size={16} strokeWidth={2.6} /></button>
    </form>
    {(blocked || failed) && <div className="composer-note" role="status">{blocked || failed}</div>}
  </div>;
}

/** The pipeline menu's entry that reopens the question of which agent the pipeline uses. */
const REASK = ' reask';

/**
 * Which conversation agent a pipeline uses, asked in a drawer over the page (owner request,
 * October 2026) rather than in the transcript box, whose room is the conversation's. `agents` is
 * every agent Home Assistant lists, as [entity_id, name]; `value` is the one typing goes to now.
 */
function AgentDrawer({
  pipelineName,
  agents,
  value,
  changed,
  onPick,
  onClose
}: {
  pipelineName: string;
  agents: [string, string][];
  value: string | null;
  /** Asked again because Home Assistant's pipelines or agents changed since the last answer. */
  changed: boolean;
  onPick: (id: string) => void;
  onClose: () => void;
}) {
  const title = TEXT.ask_which.replace('%s', pipelineName);
  return <Drawer label={title} onClose={onClose} className="ww-panel agent-panel">
    <div className="dw-top"><h2>{title}</h2><button className="dw-x" aria-label="Close" onClick={onClose}><X size={18} /></button></div>
    {changed && <p className="ww-note">{TEXT.ask_changed}</p>}
    <div className="ww-opts" role="radiogroup" aria-label={title}>{agents.map(([id, label]) => <button key={id} type="button" role="radio" aria-checked={value === id} className={value === id ? 'ww-opt on' : 'ww-opt'} onClick={() => onPick(id)}><span>{label}</span><span className="ww-mark">{value === id && <Check size={13} strokeWidth={3} />}</span></button>)}</div>
    <p className="ww-foot">{TEXT.ask_why}</p>
  </Drawer>;
}

/** A typed message's ceiling, in bytes (WU_ASK_TEXT_MAX in web_ui_handler.h). */
const ASK_MAX_BYTES = 255;
const ASK_FAILED: Record<string, string> = {
  err: TEXT.ask_err,
  timeout: TEXT.ask_timeout,
  offline: TEXT.ask_offline
};

/**
 * The device's timers. They live on the device, not in Home Assistant, so they keep counting and
 * still ring with the connection gone - which is why they are worth showing on a tab that works
 * offline. Nothing shows when there are none: timers are created by voice, so an empty state would
 * invite a press that does nothing.
 *
 * Read-only by architecture, not by choice. The owner asked for a cancel button, and there is
 * nowhere to wire one: Home Assistant owns Assist timers, the native API only pushes their events
 * device-ward, and Home Assistant offers no action that cancels one (conversation.process carries no
 * device id, and timer intents are device-scoped). Voice is the interface - "cancel the timer" -
 * which HINTS.timers says. If the protocol ever grows a cancel message, this is where it lands.
 *
 * The poll runs every second while one counts; between answers the shown time counts down from the
 * last one so seconds never stall or skip.
 */
function Timers({
  timers
}: {
  timers: Timer[];
}) {
  const polledAt = useMemo(() => Date.now(), [timers]);
  const [now, setNow] = useState(Date.now);
  const counting = timers.some(t => t.active);
  useEffect(() => {
    if (!counting) return;
    const id = setInterval(() => setNow(Date.now()), 250);
    return () => clearInterval(id);
  }, [counting]);
  return <div className="timer-list">{timers.map(t => {
      const left = timerLeft(t, polledAt, Math.max(now, polledAt));
      return <div key={t.id} className={'timer-pill' + (left === 0 ? ' done' : '') + (t.active ? '' : ' paused')} title={HINTS.timers}><Clock size={15} aria-hidden="true" /><span className="timer-pill-label">{timerLabel(t)}{!t.active && <small> · paused</small>}</span><strong className="timer-pill-time">{clock(left)}</strong></div>;
    })}</div>;
}
/** The orb tips this browser has acted on, kept current as they retire. */
function useTipsDone() {
  const [done, setDone] = useState(tipsDone);
  useEffect(() => onTips(() => setDone(tipsDone())), []);
  return done;
}

/**
 * The things a person glances at and adjusts daily, all of which work with Home Assistant switched
 * off.
 */
export function HomeTab({
  ctx,
  orb,
  onOrbColor
}: {
  ctx: Ctx;
  orb: Orb;
  onOrbColor: (from: string, to: string) => void;
}) {
  // Its presence is also this build's sign that it has the message field and the orb's tap.
  const askEnt = entity(ctx, 'ask_status');
  const askStatus = String(askEnt?.state ?? askEnt?.value ?? '');
  const asking = askStatus === 'busy';
  // A tap reaches the device before its next poll would show the run starting, so the poll speeds
  // up for a few seconds after one - by then a run that started is fast-polling on its own.
  const [tapped, setTapped] = useState(0);
  useEffect(() => {
    if (!tapped) return;
    const id = setTimeout(() => setTapped(0), TAP_FAST_MS);
    return () => clearTimeout(id);
  }, [tapped]);
  const voice = useVoice(true, asking || !!tapped);
  // Each line is stamped with the device's uptime, so its time of day is the boot time plus that.
  // The shell re-reads the uptime every ten seconds, which keeps this within a few seconds of true.
  const uptime = ctx.device?.uptime;
  const boot = useMemo(() => typeof uptime === 'number' ? Date.now() - uptime * 1000 : null, [ctx.device]);

  // Each of the device's wake word slots is a window (Pager), and the window in front decides who
  // the orb's tap talks to: the tap passes its word on as the wake word phrase, so Home Assistant
  // runs that word's pipeline. Each window's message field goes to its own word's pipeline's agent.
  // A word that holds no Home Assistant slot gets the first slot's pipeline, which is where Home
  // Assistant sends it too. Until the device's slots are read, Home Assistant's own words stand in.
  const ww = useWakeWords(ctx, WAKE_POLL_MS, false);
  const asst = ctx.ha?.d?.asst;
  const haWords: string[] = Array.isArray(asst?.s) && asst.s.length === ASSIST_SLOTS ? asst.s.map((r: string[]) => r[1] && r[1] !== NO_WAKE_WORD ? r[1] : '') : [];
  const slotWords = ww.wake ? [0, 1].map(i => {
    const swap = ww.swaps[i];
    return swap?.phase === 'busy' ? swap.word : ww.slotAt(i)?.w || '';
  }) : haWords;
  const polled: Line[] | undefined = voice?.transcript;
  const win: Windows = transcriptWindows(polled || [], slotWords);
  const [view, setView] = useState(0);
  const at = Math.min(view, win.windows.length - 1);
  // A new exchange brings its window forward - except while a message is being written (onEngage),
  // when the window in front is who it goes to and must hold still; the other's dot lights instead.
  const engaged = useRef(false);
  const [unseen, setUnseen] = useState<number | null>(null);
  const cueRef = useRef<string | null | undefined>(undefined);
  const cue = win.cue;
  useEffect(() => {
    const prev = cueRef.current;
    cueRef.current = cue ? cue.key : null;
    if (!cue || cue.key === prev) return;
    if (prev === undefined || !engaged.current) setView(cue.index);else if (cue.index !== at) setUnseen(cue.index);
  }, [cue?.key]);
  useEffect(() => {
    if (unseen === at) setUnseen(null);
  }, [at, unseen]);
  const word = win.windows[at]?.word || '';
  const assist = ww.assist;
  const keep = ww.activeWords.map(w => ww.pipeValue(w));
  const onEngage = (on: boolean) => {
    engaged.current = on;
  };
  const thinking = orbState(voice?.phase, ctx.connected) === 'thinking';
  const own = win.newest ?? at;
  const known = slotWords.length > 0;
  const labels = known ? win.windows.map(w => w.word ? tabLabel(w.word) : TEXT.tt_empty) : [TEXT.tt_one];
  const cards = win.windows.map((w, i) => {
    const pipe = assist.ready && ww.activeWords.includes(w.word) ? {
      value: ww.pipeValue(w.word),
      options: ww.pipeOptions,
      busy: assist.busy,
      onPick: (v: string) => assist.setPipeline(w.word, v)
    } : null;
    return <>
      {known && <WakeHead ww={ww} i={i} word={w.word} />}
      {w.word || !known ? <>
        <Conversation lines={w.lines} polled={polled} own={i === own} thinking={thinking} asking={asking} boot={boot} paged={known} />
        {askEnt && <Composer ctx={ctx} status={askStatus} word={w.word} pipeline={assist.ready ? ww.pipeValue(w.word) : null} pipe={pipe} keep={keep} onEngage={onEngage} />}
      </> : <div className="tt-vacant"><p>{TEXT.nothing_said} {TEXT.tt_swipe}</p><button type="button" className="primary" disabled={!ww.wake} aria-haspopup="dialog" onClick={() => {
        tipDone('pick');
        ww.setPicking(i);
      }}><Plus size={14} strokeWidth={2.6} aria-hidden="true" />{TEXT.tt_add}</button></div>}
    </>;
  });
  const talk = askEnt ? () => {
    tipDone('orb');
    request('/api/sat1/talk', {
      method: 'POST',
      body: new URLSearchParams({
        w: word
      })
    }).catch(() => {});
    setTapped(Date.now());
  } : undefined;

  // The orb's tips, from what the page already holds. The spoken examples use the word the orb
  // answers to - the front window's, or with that slot empty the other's - and that word's agent,
  // worked out as its message field does.
  const done = useTipsDone();
  const timersOn = (voice?.timers || []).length > 0;
  const stopSaid = !!polled?.some(l => l.w === 'stop');
  useEffect(() => {
    if (timersOn) tipDone('timer');
  }, [timersOn]);
  useEffect(() => {
    if (stopSaid) tipDone('stop');
  }, [stopSaid]);
  const sayWord = word || slotWords.find(Boolean) || '';
  const cv: [string, string][] = Array.isArray(ctx.ha?.d?.cv) ? ctx.ha.d.cv : [];
  const mapEnt = entity(ctx, 'pipeline_agents');
  const sayPipe = assist.ready && sayWord ? ww.pipeValue(sayWord) : null;
  const sayAgent = sayPipe === null ? null : pipelineAgent(sayPipe, cv, readAgentMap(String(mapEnt?.state ?? mapEnt?.value ?? '')), setupStamp(asst?.o, cv));
  const front = ww.wake && word ? ww.slotAt(at) : null;
  const mac = String(ctx.device?.mac || '').toLowerCase();
  const tips = orbTips({
    word: sayWord,
    other: known && slotWords.every(Boolean) ? slotWords[1 - at] || '' : '',
    room: deviceIdentity(ctx.device, ctx.ha).area,
    ai: !!sayAgent && sayAgent !== DEFAULT_AGENT,
    voice: !!ctx.device?.ha,
    tap: !!askEnt,
    type: !!askEnt && !!ctx.device?.ha && !haBlocked(ctx.ha) && !haTooOld(ctx.ha),
    untuned: !!front?.ld && !((front?.cut || 0) > 0) && ww.swaps[at]?.phase !== 'busy',
    pick: !!ww.wake && !!word,
    sensors: SENSORS.some(s => entity(ctx, s.key) && entity(ctx, s.offsetKey)),
    radar: !!ctx.states['text_sensor/Radar Target'],
    noRadar: entity(ctx, 'radar_module')?.value === 'None',
    music: !!ctx.ha?.d?.ma?.e,
    peers: (ctx.ha?.d?.dev || []).some((d: string[]) => /satellite1/i.test(d?.[0] || '') && String(d?.[3] || '').toLowerCase() !== mac),
    mute: !!entity(ctx, 'mute_mics'),
    ring: !!entity(ctx, 'ring'),
    stop: isOn(entity(ctx, 'stop_word')),
    route: !!ctx.sel?.routing && !hasTargets(ctx.sel.routing)
  }, done);
  return <section className="now now-compact"><div className="now-left" style={{
      width: '100%'
    }}><SensorPills ctx={ctx} /><VoiceOrb ctx={ctx} phase={voice?.phase} orb={orb} onOrbColor={onOrbColor} onTalk={talk} tips={tips} /></div><div className="now-right"><Pager view={at} onView={i => {
        tipDone('swipe');
        setView(i);
      }} labels={labels} cue={unseen} cards={cards} /><Timers timers={voice?.timers || []} /></div>{ww.drawers}</section>;
}

/** The Home page's standing read of the wake word slots: words and tuning change rarely, and a
 *  swap or tune started here runs its own fast loop. */
const WAKE_POLL_MS = 20000;
