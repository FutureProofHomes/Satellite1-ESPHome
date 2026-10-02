import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { TEXT } from '../copy.js';
import { logout, peerLogin, primeOtherOrigin, probePeer, putPanelHandoff } from '../lib/auth.js';
import { deviceIdentity, entity, haBlocked, onLogAlert, onWriteError, peerOrigin, proxied, useDeviceState, useEvents, useHaData, useSelection } from '../lib/device.js';
import { tipDone } from '../lib/tips.js';
import { archiveAllNotifs, archiveNotif, listNotifs, notifCount, setNotifDevice, subscribeNotifs } from '../lib/notif.js';
import { setSparkDevice, sparkRecord } from '../lib/sparkhist.js';
import { dismissToast, subscribeToasts, tapToast, toast, toastIntent } from '../lib/toast.js';
import type { Ctx, Orb, Tab } from '../ctx';
import { AlertTriangle, AudioLines, ChevronLeft, ChevronRight, Clock, House, Info, LogOut, RadarIcon, Settings, Volume2, X, XCircle } from '../icons';
import { parseRoute, routeDir, routeHash } from '../lib/routes.js';
import { AudioTab } from './AudioTab';
import { Icon, ORB_KEY, ThemeMenu, paintOrb, readOrb, reduceMotion } from './bits';
import { DiagnosticsTab, SETTINGS_ROUTES } from './DiagnosticsTab';
import { Drawer, Presence } from './Drawer';
import { HaGate } from './HaGate';
import { HomeTab } from './HomeTab';
import { MediaBar } from './MediaBar';
import { PresenceTab } from './PresenceTab';
import { WakeTab } from './WakeTab';
type Sheet = 'media' | 'device' | 'notice' | null;
type ToastKind = 'info' | 'warn' | 'error' | 'timer';
/** src/lib/toast.js's visible toast, and src/lib/notif.js's history row: the same anatomy. */
type Toast = {
  id: number;
  kind: string;
  title: string;
  sub?: string;
  count?: number;
  go?: string;
  intent?: unknown;
  act?: string;
};
type Remote = { base: string; key: string } | null;
type Route = { tab: string; sub: string | null };
type VtDoc = Document & { startViewTransition?: (update: () => Promise<void>) => { finished: Promise<void> } };

let slides = 0;
/**
 * Swaps the page inside a view transition that slides it the way the nav reads (styles/motion.css)
 * while the header, nav and media bar hold still. The class scopes those rules to route changes, so
 * the theme's crossfade keeps its own; a newer slide supersedes an older one's cleanup.
 */
function slide(from: Route, to: Route, swap: () => void) {
  const dir = routeDir(from, to);
  const html = document.documentElement;
  if (dir) html.dataset.dir = dir;
  const doc = document as VtDoc;
  if (!dir || !doc.startViewTransition || reduceMotion() || document.hidden) return swap();
  const n = ++slides;
  html.classList.add('vt-route');
  // Preact renders on a microtask; the timeout lets the new page commit before the snapshot.
  doc.startViewTransition(() => {
    swap();
    return new Promise(res => setTimeout(res, 0));
  }).finished.catch(() => {}).then(() => {
    if (n === slides) html.classList.remove('vt-route');
  });
}
const TABS: Tab[] = ['NOW', 'WAKE', 'PRESENCE', 'AUDIO', 'SETTINGS'];
const TAB_ICON = { NOW: House, WAKE: AudioLines, PRESENCE: RadarIcon, AUDIO: Volume2, SETTINGS: Settings };
const TAB_LABEL = { NOW: TEXT.nav_home, WAKE: TEXT.nav_wake, PRESENCE: TEXT.nav_presence, AUDIO: TEXT.nav_audio, SETTINGS: TEXT.nav_settings };
const TOAST_ICONS = {
  info: Info,
  warn: AlertTriangle,
  error: XCircle,
  timer: Clock
};
/** The store's kinds in the design's palette. "ok" has no colour of its own there and wears info's. */
const pillKind = (k: string): ToastKind => k === 'err' ? 'error' : k === 'warn' || k === 'timer' ? k : 'info';

/**
 * The header's toast: a view of whatever src/lib/toast.js says is visible. The store owns the
 * timing, the queue and the ×N coalescing; the pill only draws, swipes and reports taps.
 */
function ToastPill({
  toast,
  onDismiss,
  onTap,
  leaving = false
}: {
  toast: Toast;
  onDismiss: (id: number) => void;
  onTap?: () => void;
  leaving?: boolean;
}) {
  const pillRef = useRef<HTMLDivElement>(null);
  const startX = useRef<number | null>(null);
  const [swipeDx, setSwipeDx] = useState(0);
  const [swiping, setSwiping] = useState(false);
  const [dismissed, setDismissed] = useState(false);
  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (leaving || dismissed || (e.target as HTMLElement).closest('.toast-x')) return;
    startX.current = e.clientX;
    setSwiping(true);
    pillRef.current?.setPointerCapture(e.pointerId);
  };
  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (!swiping || startX.current === null) return;
    setSwipeDx(e.clientX - startX.current);
  };
  const onPointerUp = (e: React.PointerEvent<HTMLDivElement>) => {
    if (startX.current === null) return;
    setSwiping(false);
    const dx = e.clientX - startX.current;
    startX.current = null;
    if (Math.abs(dx) >= 72) {
      setDismissed(true);
      setSwipeDx(dx > 0 ? 400 : -400);
      setTimeout(() => onDismiss(toast.id), 250);
    } else {
      setSwipeDx(0);
      if (Math.abs(dx) < 6 && !(e.target as HTMLElement).closest('.toast-x')) onTap?.();
    }
  };
  const kind = pillKind(toast.kind);
  const Ico = TOAST_ICONS[kind];
  const label = (toast.count ?? 0) > 1 ? `${toast.title} \u00d7${toast.count}` : toast.title;
  return <div ref={pillRef} className={`header-toast-pill toast-${kind}${leaving ? ' leaving' : ' entering'}`} role="status" title={toast.sub || undefined} style={{
    transform: `translateX(${swipeDx}px)`,
    opacity: dismissed ? 0 : Math.max(0.3, 1 - Math.abs(swipeDx) / 200),
    transition: swiping ? 'none' : 'transform 240ms ease, opacity 240ms ease',
    touchAction: 'pan-y',
    userSelect: 'none',
    cursor: onTap && (toast.go || toast.act) ? 'pointer' : 'grab'
  }} onPointerDown={onPointerDown} onPointerMove={onPointerMove} onPointerUp={onPointerUp} onPointerCancel={onPointerUp}>
      <Ico size={18} className="toast-icon" aria-hidden="true" />
      <span className="toast-msg">{label}</span>
      <button type="button" className="toast-x" aria-label={TEXT.dismiss} onClick={() => onDismiss(toast.id)}><X size={16} /></button>
    </div>;
}

/** Where the previous UI's #/diagnostics cards live now. Notification rows that UI wrote stay in
 *  localStorage for 24 hours carrying go "#/diagnostics" and a card intent, and still act here. */
const CARD_PAGE: Record<string, string> = {
  log: '#/settings/logs',
  crash: '#/settings/logs',
  firmware: '#/settings/updates'
};

/**
 * What a tapped toast or history row does - one path for both, so the two can never act
 * differently on the same entry. "fix" opens the walk-through; anything else navigates with its
 * intent riding src/lib/toast.js's handoff. Standing on the destination already, an identical hash
 * fires no hashchange and remounts nothing, so that page hears a window event instead (the Settings
 * pages listen while mounted).
 */
function runAct(t: Toast, onFix: () => void) {
  if (t.act === 'fix') {
    onFix();
    return;
  }
  let go = t.go;
  if (!go) return;
  if (/^#\/diagnostics\/?$/.test(go)) go = CARD_PAGE[(t.intent as { card?: string } | undefined)?.card ?? ''] ?? '#/settings/device';
  if (t.intent) toastIntent(t.intent);
  if (location.hash === go || `${location.hash}/` === go) window.dispatchEvent(new CustomEvent('toast-intent'));
  else location.hash = go;
}

/**
 * Every event that raises a toast, watched here because the shell lives exactly as long as one
 * device's session (it remounts per remote-control target, which resets the once-per-session
 * guards for a genuinely different device). Toasts are the app's whole out-of-band vocabulary
 * since the amber banners were retired (owner decision, September 2026). A failed request toasts
 * only when it was a write - see onWriteError in src/lib/device.js for why reads stay out of it.
 */
function useToastSources({
  connected,
  blocked,
  device,
  states
}: {
  connected: boolean;
  blocked: boolean;
  device: any;
  states: Record<string, any>;
}) {
  useEffect(() => onWriteError(() => toast({
    kind: 'err',
    key: 'write',
    ttl: 6000,
    title: TEXT.write_failed,
    sub: TEXT.write_failed_go,
    go: '#/settings/logs',
    intent: { card: 'log' }
  })), []);
  useEffect(() => onLogAlert(({ lvl, tag, text, at }: { lvl: string; tag: string; text: string; at: number }) => {
    const err = lvl !== 'W';
    toast({
      kind: err ? 'err' : 'warn',
      // Per level and component, so ten wifi warnings are one toast wearing ×10 while an unrelated
      // error still gets its own. The source already stays quiet while a log view has a reader.
      key: `log-${err ? 'E' : 'W'}-${tag}`,
      ttl: 6000,
      title: (err ? TEXT.log_toast_err : TEXT.log_toast_warn).replace('%s', tag || TEXT.log_toast_dev),
      sub: TEXT.write_failed_go,
      go: '#/settings/logs',
      // The line rides by the same {at, text} identity the log ring holds (src/lib/device.js stamps
      // both from one clock), so the reveal lands on the exact line - and a coalescing burst adopts
      // the newest line as it counts up.
      intent: { card: 'log', level: err ? 'E' : 'W', line: { at, text } }
    });
  }), []);
  // The nudge, on the rising edge only: once on entering a blocked app (arriving through the gate's
  // Continue included), again only if the state genuinely re-enters - never per re-render.
  useEffect(() => {
    if (blocked) toast({
      kind: 'warn',
      key: 'blocked',
      ttl: 8000,
      title: TEXT.blocked_toast_t,
      sub: TEXT.blocked_toast_s,
      act: 'fix'
    });
  }, [blocked]);
  // A sticky while the stream is down, resolved by its own reconnect, and one moment on the way
  // back. `sawDown` is apart from the handle because a ✕'d sticky's recovery is still news.
  const lost = useRef<{ resolve: () => void } | null>(null);
  const sawDown = useRef(false);
  useEffect(() => {
    if (!connected) {
      sawDown.current = true;
      if (!lost.current) lost.current = toast({
        kind: 'warn',
        key: 'stream',
        title: TEXT.stream_lost,
        sub: TEXT.write_failed_go,
        go: '#/settings/logs',
        intent: { card: 'log' }
      });
    } else {
      lost.current?.resolve();
      lost.current = null;
      if (sawDown.current) {
        sawDown.current = false;
        toast({ kind: 'ok', key: 'stream-ok', ttl: 4000, title: TEXT.stream_back });
      }
    }
  }, [connected]);
  // A crash count that moves mid-session: the device just rebooted from a crash under us. Seeded on
  // the first reading, because history is the crash card's story.
  const crashSeen = useRef<number | null>(null);
  const crash = device?.crash;
  useEffect(() => {
    if (crash == null) return;
    if (crashSeen.current != null && crash > crashSeen.current) toast({
      kind: 'err',
      key: 'crash',
      ttl: 12000,
      title: TEXT.crash_toast_t,
      sub: TEXT.crash_toast_s,
      go: '#/settings/logs',
      intent: { card: 'crash' }
    });
    crashSeen.current = crash;
  }, [crash]);
  // Update news, once per session. The guard resets with the shell's remount, so a device switch can
  // report the other device's update - which is correct: it is different news.
  const updTold = useRef(false);
  const upd = device?.e?.firmware ? states[device.e.firmware] : undefined;
  const updState = upd?.state;
  useEffect(() => {
    if (updTold.current || updState !== 'UPDATE AVAILABLE') return;
    updTold.current = true;
    toast({
      kind: 'info',
      key: 'update',
      ttl: 10000,
      title: TEXT.update_toast_t.replace('%s', upd.value || ''),
      sub: TEXT.update_toast_s,
      go: '#/settings/updates',
      intent: { card: 'firmware' }
    });
    // upd.value rides updState: the entity publishes both in one message.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [updState]);
}
/**
 * The signed-in app: one device's session. Owns the data hooks every tab reads through ctx, the
 * hash route, the toast and notification surfaces, the device switcher and the Home Assistant gate.
 */
export function Satellite1Now({
  primeKey,
  remote,
  localMac,
  onRemote,
  onLocal,
  onAuthLost
}: {
  primeKey: { current: string | null };
  remote: Remote;
  localMac: string | null;
  onRemote: (target: { base: string; key: string }, mac: string | null) => void;
  onLocal: () => void;
  onAuthLost: () => void;
}) {
  const appRef = useRef<HTMLElement>(null);
  const headerRef = useRef<HTMLElement>(null);
  useLayoutEffect(() => {
    const measure = () => {
      const hh = headerRef.current?.offsetHeight ?? 72;
      appRef.current?.style.setProperty('--header-h', hh + 'px');
    };
    measure();
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, []);
  const [route, setRoute] = useState<Route>(() => parseRoute(location.hash));
  const shownRoute = useRef(route);
  useEffect(() => {
    // A bookmark from the previous UI (#/controls, #/config, #/diagnostics) or an unknown hash is
    // rewritten in place to the page it resolved to, so the address bar never disagrees with what
    // is on screen.
    const on = () => {
      const r = parseRoute(location.hash);
      const canon = routeHash(r.tab, r.sub ?? undefined);
      if (location.hash && location.hash !== canon) history.replaceState(null, '', canon);
      slide(shownRoute.current, r, () => setRoute(r));
      shownRoute.current = r;
    };
    on();
    window.addEventListener('hashchange', on);
    return () => window.removeEventListener('hashchange', on);
  }, []);
  const tab = route.tab as Tab;
  const sub: string | null = route.sub;
  // Settings remembers its page across a trip to another tab, as the design's own state did.
  const lastSub = useRef('device-info');
  if (sub) lastSub.current = sub;
  const go = useCallback((t: Tab, s?: string) => {
    location.hash = routeHash(t, s);
  }, []);
  const navLayer = tab === 'SETTINGS' ? 1 : 0;
  const [sheet, setSheet] = useState<Sheet>(null);
  // The gate is up from the first signed-in render - the first authenticated moment, whether the
  // session came from the sign-in screen a second ago or from a 90-day cookie - and gone for good
  // once it fades. State rather than anything remembered, so every page load gets the same honest
  // check. The fix is the same screen reopened on demand.
  const [gateDone, setGateDone] = useState(false);
  const [fixOpen, setFixOpen] = useState(false);

  // Settings wants fresh heap and loop figures; everywhere else the only thing that goes stale is
  // the Home Assistant dot, worth one request every ten seconds so it starts telling the truth again
  // on its own after the connection comes back.
  const { device, deviceError } = useDeviceState(tab === 'SETTINGS' ? 2000 : 10000);
  const events = useEvents();
  const ha = useHaData();
  const selection = useSelection();

  // Dual-origin cookie priming: one CORS sign-in against the device's other entrance (.local when
  // on the IP, the IP when on .local), once the state payload names it. The key is dropped the
  // moment it is used - it lives nowhere but the cookie after this. Never while remote: `device` is
  // the peer's there, and priming the peer's other origin with the local key can only fail.
  useEffect(() => {
    if (remote || !primeKey.current || !device) return;
    primeOtherOrigin(primeKey.current, device.name, device.ip);
    primeKey.current = null;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [device, primeKey]);
  // A session dying under a running app - the password changed, or Sign out everywhere pressed
  // somewhere else. The state poll is the heartbeat that notices.
  useEffect(() => {
    if (deviceError === 'HTTP 401') onAuthLost();
  }, [deviceError, onAuthLost]);
  // The notification and sparkline histories are bucketed per MAC; point both at this device the
  // moment it is known, which also covers a remote-control retarget (the shell remounts per device).
  const mac = device?.mac;
  useEffect(() => {
    if (mac) {
      setNotifDevice(mac);
      setSparkDevice(mac);
    }
  }, [mac]);
  const ctx: Ctx = {
    device,
    deviceError,
    ...events,
    ...ha,
    ...selection,
    onShowFix: () => setFixOpen(true),
    tab,
    sub,
    go
  };

  // The sensor sparklines accrue on every tab, not only while Home is open. Values are the
  // entities' native units (°C - the °F flip is display-only). The change effects catch a moved
  // value, and the 60s beat keeps flat periods accruing points: the stream's merge reducer swallows
  // no-change publishes, so a timer is the only way to see them.
  const sparkTemp = Number(entity(ctx, 'temp')?.value);
  const sparkHum = Number(entity(ctx, 'humidity')?.value);
  const sparkLux = Number(entity(ctx, 'lux')?.value);
  useEffect(() => sparkRecord('temp', sparkTemp), [sparkTemp]);
  useEffect(() => sparkRecord('humidity', sparkHum), [sparkHum]);
  useEffect(() => sparkRecord('lux', sparkLux), [sparkLux]);
  const sparkNow = useRef<Record<string, number>>({});
  sparkNow.current = { temp: sparkTemp, humidity: sparkHum, lux: sparkLux };
  useEffect(() => {
    const t = setInterval(() => {
      const s = sparkNow.current;
      for (const k in s) sparkRecord(k, s[k]);
    }, 60000);
    return () => clearInterval(t);
  }, []);
  // Resolved once here, so the header, the side nav and the switcher cannot disagree about what this
  // device is called. The tab title follows it; the sign-in screen already set the hostname as the
  // fallback for the time before.
  const { name: label, area } = deviceIdentity(device, ha.ha);
  useEffect(() => {
    if (label) document.title = label;
  }, [label]);

  // The store's visible toast, and a ghost of the one leaving so its exit can animate - an element
  // unmounted the moment it is dismissed is gone before any transition can run.
  const [cur, setCur] = useState<Toast | null>(null);
  const [ghost, setGhost] = useState<Toast | null>(null);
  const prevToast = useRef<Toast | null>(null);
  useEffect(() => subscribeToasts(setCur), []);
  useEffect(() => {
    const was = prevToast.current;
    prevToast.current = cur;
    if (was && (!cur || cur.id !== was.id)) {
      setGhost(was);
      const t = setTimeout(() => setGhost(null), 300);
      return () => clearTimeout(t);
    }
    return undefined;
  }, [cur]);
  // `blocked` waits for the gate to leave: while it stands, the verdict is its story to tell, and the
  // toast's job is to keep the fix reachable afterwards.
  useToastSources({
    connected: events.connected,
    blocked: gateDone && haBlocked(ha.ha),
    device,
    states: events.states
  });
  const [notifN, setNotifN] = useState(notifCount);
  useEffect(() => subscribeNotifs(() => setNotifN(notifCount())), []);
  const bellLabel = notifN > 0 ? `${TEXT.notif_bell} (${notifN})` : TEXT.notif_bell;

  const [orb, setOrb] = useState<Orb>(readOrb);
  const onOrbColor = useCallback((a: string, b: string) => {
    setOrb(o => o.a === a && o.b === b ? o : { a, b });
    try {
      localStorage.setItem(ORB_KEY, JSON.stringify({ a, b }));
    } catch {
      /* the choice holds for this session */
    }
  }, []);
  useLayoutEffect(() => paintOrb(orb.a, orb.b), [orb]);
  // This browser only - Sign out everywhere lives on Settings > Security, where its blast radius can
  // be explained. The reload lands on the boot probe, which finds no session and shows sign-in.
  const signOut = async () => {
    setSheet(null);
    await logout();
    location.reload();
  };
  // The header dot is Home Assistant's connection, not the device's reachability: this is the
  // device serving the page, so that would be a light that could never go out. The address is the
  // device sheet's, a tap away - too technical for the chip (owner, October 2026). While a peer is
  // remote-controlled the chip says so in one word, because every device's name has the same shape
  // and the single-origin switch changes nothing else about the page.
  const haText = device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected;
  const openFix = () => setFixOpen(true);
  const activeToast = cur;
  return <main className="app" ref={appRef}><header ref={headerRef} className={`app-header${activeToast ? ' toast-active' : ''}`}><div className="header-left"><div className="header-device-slot"><button className="device-chip" onClick={() => {
                tipDone('switch');
                setSheet('device');
              }}><span className={'dot' + (device?.ha ? '' : ' off')} title={haText} /> <span>{label || 'Satellite1'}</span>{remote && <small>{TEXT.remote_tag}</small>}</button></div><div className="header-toast-slot" aria-live="polite">{ghost && <ToastPill key={`leaving-${ghost.id}`} toast={ghost} leaving onDismiss={() => {}} />}{activeToast && <ToastPill key={`active-${activeToast.id}`} toast={activeToast} onDismiss={dismissToast} onTap={() => {
              tapToast(activeToast.id);
              runAct(activeToast, openFix);
            }} />}</div></div><div className="header-actions"><button className="icon-button" onClick={() => setSheet('notice')} aria-label={bellLabel} title={bellLabel}><Icon name="bell" size={20} />{notifN > 0 && <b>{notifN > 99 ? '99+' : notifN}</b>}</button><ThemeMenu className="theme-toggle" /></div></header>
    <div key={tab} className="tab-content-enter">{tab === 'NOW' && <HomeTab ctx={ctx} orb={orb} onOrbColor={onOrbColor} />}
    {tab !== 'NOW' && <ControlPanel key={tab} tab={tab} sub={sub} ctx={ctx} />}</div>
    <nav className="side-nav" aria-label="Sections">{TABS.map(item => {
      const I = TAB_ICON[item];
      return <button key={item} className={tab === item ? 'selected' : ''} aria-current={tab === item ? 'page' : undefined} onClick={() => go(item, item === 'SETTINGS' ? lastSub.current : undefined)}><I size={18} strokeWidth={1.9} aria-hidden="true" /><span>{TAB_LABEL[item]}</span></button>;
    })}{tab === 'SETTINGS' && <div className="side-sub" role="list">{SETTINGS_ROUTES.map(r => <button key={r.slug} role="listitem" className={'side-sub-item' + (sub === r.slug ? ' on' : '')} onClick={() => go('SETTINGS', r.slug)}>{r.label}</button>)}</div>}<div className="side-nav-foot"><span className="side-nav-name">{label || 'Satellite1'}</span><span className="side-nav-meta">{[device?.ip, device?.fw && `Firmware ${device.fw}`].filter(Boolean).join(' · ')}</span><button type="button" className="signout" onClick={signOut}><LogOut size={14} aria-hidden="true" /><span>{TEXT.logout}</span></button></div></nav>
    <nav className="tabs" data-tab={tab} aria-label="Sections"><div className="tabs-track" style={{
        transform: navLayer === 0 ? 'translateX(0%)' : 'translateX(-50%)'
      }}><div className="tabs-layer tabs-main">{TABS.map(item => {
          const I = TAB_ICON[item];
          return <button key={item} className={tab === item ? 'selected' : ''} aria-current={tab === item ? 'page' : undefined} onClick={() => go(item, item === 'SETTINGS' ? lastSub.current : undefined)}><span className="tab-ico"><I size={20} strokeWidth={1.9} aria-hidden="true" /></span><span className="tab-lbl">{TAB_LABEL[item]}</span></button>;
        })}</div><div className="tabs-layer tabs-sub"><button className="tabs-back" aria-label="Back to main menu" onClick={() => {
            lastSub.current = 'device-info';
            go('NOW');
          }}><ChevronLeft size={20} strokeWidth={2} aria-hidden="true" /></button><div className="tabs-sub-scroll">{SETTINGS_ROUTES.map(r => <button key={r.slug} className={'tabs-sub-pill' + (sub === r.slug ? ' on' : '')} onClick={() => go('SETTINGS', r.slug)}>{r.label}</button>)}</div></div></div></nav><MediaBar ctx={ctx} />
    <Presence>{sheet === 'device' && <Drawer label={TEXT.switcher_label} onClose={() => setSheet(null)}><DeviceSheet ctx={ctx} label={label} area={area} remote={remote} localMac={localMac} onRemote={onRemote} onLocal={onLocal} onSignOut={signOut} /></Drawer>}</Presence>
    <Presence>{sheet === 'notice' && <Drawer label={TEXT.notif_title} onClose={() => setSheet(null)}><Notice onAct={t => {
          setSheet(null);
          runAct(t, openFix);
        }} /></Drawer>}</Presence>
    {!gateDone && <HaGate ctx={ctx} onDone={() => setGateDone(true)} />}
    {fixOpen && <HaGate ctx={ctx} fix onDone={() => setFixOpen(false)} />}
  </main>;
}
const hostOf = (u: unknown) => String(u || '').replace(/^https?:\/\//, '').replace(/[/:].*$/, '');
const isUp = (d: any[]) => d[6] === 1 || d[6] === '1';
/* The variant fields are the payload's own encoding - 'e'/'w', 2450/2410, 1/0 - so a row from older
   firmware, which sends none of the three, shows nothing rather than something wrong. */
const radarModel = (r: unknown) => r === 2450 || r === 2410 ? `LD${r}` : '';
const netName = (n: unknown) => n === 'e' ? 'Ethernet' : n === 'w' ? 'Wi\u2011Fi' : '';
/** "LD2450 · present", or nothing for a device whose firmware sends no radar fields. */
const radarText = (r: unknown, present: unknown) => {
  const m = radarModel(r);
  return m && `${m} · ${present === 1 || present === true ? 'present' : 'no presence'}`;
};

/**
 * The device switcher: this device on top, then every other Satellite1 the Home Assistant payload
 * lists, online first. The roster is the `dev` block on /api/sat1/ha - row fields [model, name,
 * area, mac, sw, url, up, pw, net, radar, present] - so nothing here discovers anything; a jump
 * signs in to the peer first and, when the peer's firmware allows it, retargets this app instead of
 * navigating.
 *
 * No discovery because a browser can do none: mDNS browsing is not available to a page, and probing
 * peers directly fails too - Digest credentials are scoped per origin and a cross-origin probe dies
 * on the preflight. So each row's link is the peer's `configuration_url` (the address behind Home
 * Assistant's "Visit device") and its dot is Home Assistant's availability view, which is better
 * data anyway: it knows a device is off the moment it disconnects, where a probe would wait out a
 * timeout. The caveat is that the payload is the device's cache, so the sheet re-syncs it (below).
 *
 * Satellite1 models only: a Nexus is in `dev` too, and a row that jumps to a device with no page to
 * serve is a trap. This device is dropped by MAC rather than by name, because the name is exactly
 * the field owners change. A row with no URL renders unlinked rather than hidden - a device that
 * exists but cannot be jumped to is still worth seeing. There is no add-by-address field (removed
 * at the owner's request); a peer Home Assistant cannot list is reachable by typing its address.
 * The variant fields are described in docs/web-ui.md, "Why the browser never calls Home Assistant".
 */
function DeviceSheet({
  ctx,
  label,
  area,
  remote,
  localMac,
  onRemote,
  onLocal,
  onSignOut
}: {
  ctx: Ctx;
  label: string;
  area: string;
  remote: Remote;
  localMac: string | null;
  onRemote: (target: { base: string; key: string }, mac: string | null) => void;
  onLocal: () => void;
  onSignOut: () => void;
}) {
  const { device, ha, haRefresh, states } = ctx;
  const hash = routeHash(ctx.tab, ctx.sub ?? undefined);
  const mac = (device?.mac || '').toLowerCase();
  // Re-sync the roster the moment the sheet opens: otherwise the cache is rebuilt only when the Audio
  // or Wake tab asks (haSyncOnce, once per page load), and a peer powered off in between kept a
  // green dot for days.
  // Then, while the sheet stays open, the same sync on a 5s beat - what makes a peer's presence tag
  // follow the room. Gated three ways: never while a sync is running (haRefresh sleeps ~3s inside,
  // so an ungated interval would stack them), never while the tab is hidden (a background poll
  // spends Home Assistant action calls on a sheet nobody sees), and only while the last answer came
  // on rung 1, the recorder.get_statistics fast path. The rung 2 fallback costs ~3s a call through
  // conversation.process and would sit on the request queue the jump itself needs; those
  // installations keep the once-per-open sync.
  const haNow = useRef(ha);
  haNow.current = ha;
  useEffect(() => {
    let busy = false;
    const sync = async () => {
      if (busy) return;
      busy = true;
      try {
        await haRefresh();
      } finally {
        busy = false;
      }
    };
    sync();
    const t = setInterval(() => {
      if (document.hidden || haNow.current?.rung !== 1) return;
      sync();
    }, 5000);
    return () => clearInterval(t);
    // haRefresh is stable for the life of the app; this is per-open by design.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  // Sorted by area then name, so a house full of these groups by room, matching the tree on Audio.
  const roster: any[][] = ha?.d?.dev || [];
  const peers = roster.filter(d => /satellite1/i.test(d?.[0] || '') && (d?.[3] || '').toLowerCase() !== mac).sort((x, y) => `${x[2]}\u0000${x[1]}`.localeCompare(`${y[2]}\u0000${y[1]}`));
  const ordered = [...peers.filter(isUp), ...peers.filter(d => !isUp(d))];

  // This device's own tags read the live stream rather than the cached roster, so its presence
  // lights the moment the radar does; the roster row (kept out of `peers` by the mac filter) is the
  // fallback. While remote these are the controlled peer's own. "Room Presence" is the auto-detect
  // build's runtime registration and "Presence" the pinned builds' YAML: names the firmware owns,
  // not ones an owner renames (what the entity table exists to protect against), so they are
  // referenced by id.
  const mineRow = roster.find(d => (d?.[3] || '').toLowerCase() === mac);
  const netLive = String(states?.[device?.e?.network]?.value || '');
  const myNet = netLive.startsWith('Eth') ? 'e' : netLive.startsWith('WiFi') ? 'w' : mineRow?.[8];
  const modLive = String(states?.[device?.e?.radar_module]?.value || '').toLowerCase();
  const myRadar = modLive.includes('2450') ? 2450 : modLive.includes('2410') ? 2410 : mineRow?.[9];
  const presLive = states?.['binary_sensor/Room Presence'] || states?.['binary_sensor/Presence'];
  const myPres = presLive ? presLive.value === true || presLive.state === 'ON' : mineRow?.[10];
  const myLit = !!radarModel(myRadar) && (myPres === 1 || myPres === true);
  // The open page rides along, so moving to another device keeps it. The hash never reaches either
  // server, so this works against old firmware too - it just falls off to whatever that serves at /.
  const peerHref = (base: string) => `${String(base).replace(/\/+$/, '')}/${hash}`;

  // The seamless jump: sign in to the peer before leaving so its page opens as the app rather than
  // as its sign-in screen. The peer's password rides the roster (the Web UI Password sensor every
  // device already publishes to Home Assistant, via web_ui_ha.yaml) and the sign-in is the same
  // challenge-response the password form uses. Anything that declines - older peer firmware, a
  // missing password, mDNS trouble - falls back to plain navigation and the peer's own sign-in. The
  // href stays real underneath, so middle-click and open-in-new-tab keep working (they skip the
  // sign-in and land on the fallback).
  const jump = async (e: React.MouseEvent, url: string, pw: string | undefined) => {
    if (!pw || e.button !== 0 || e.metaKey || e.ctrlKey) return;
    e.preventDefault();
    const origin = String(url).replace(/\/+$/, '');
    const peer = await peerLogin(origin, pw);
    if (!peer) {
      location.href = peerHref(url);
      return;
    }
    // The single-origin switch, tried first: one gated read with the fresh key says whether the
    // peer accepts remote control and speaks this app's API contract (see probePeer). A yes means
    // the app retargets and remounts with no navigation, so the iOS home-screen app never meets
    // Safari's in-app sheet. The base is the roster's IP origin rather than the .local upgrade below:
    // this target lives in memory for the session, so DHCP stability buys nothing, and skipping mDNS
    // removes the one way the switch could land on a browser error page. The mac rides along only on
    // the way out of the local device, so the sheet can offer the way back (isHome).
    if (await probePeer(origin, peer.key)) {
      onRemote({ base: origin, key: peer.key }, remote ? null : mac);
      return;
    }
    // Older peer firmware: plain navigation, landing through ?key= so the peer sets its cookie
    // first-party, where third-party cookie blocking cannot eat it. When the login body carries the
    // peer's hostname and this page is itself on .local - proof this browser resolves mDNS - it goes
    // straight to the peer's .local origin, skipping the IP-then-redirect double load; peers run the
    // same firmware, so this page's port is the peer's. The name is regex-checked before it becomes
    // a URL because it arrives in a CORS-readable body. Known accepted risk: this proves our mDNS
    // works, not the peer's - a peer with mdns: disabled lands on a browser error page. The fleet
    // ships mDNS on, and the failure is recoverable: back, or middle-click the real href.
    const base = location.hostname.endsWith('.local') && peer.name && /^[a-z0-9-]+$/i.test(peer.name) ? `http://${peer.name}.local${location.port ? `:${location.port}` : ''}` : origin;
    location.href = `${base}/?key=${peer.key}${hash}`;
  };
  // While remote, the device serving this page is just another roster row (the mac filter excludes
  // the controlled device, not the serving one). Going home is a state reset on a session this
  // browser already holds - no cross-sign-in, no probe, nothing that can fail.
  const isHome = (d: any[]) => !!remote && !!localMac && (d?.[3] || '').toLowerCase() === localMac;
  // Room first: it is what a person scans for, and the address is the fallback when two devices
  // share a room (owner's call).
  const meta = (d: any[], url: string) => isHome(d) ? TEXT.switcher_home : !isUp(d) ? 'Offline' : [d[2], hostOf(url), netName(d[8]), radarText(d[9], d[10])].filter(Boolean).join(' · ');
  const peerRow = (d: any[]) => {
    // peerOrigin, not d[5] raw: a .local configuration_url is swapped for the row's live IP when the
    // roster carries one, so the jump works where mDNS does not (see src/lib/device.js).
    const url = peerOrigin(d);
    const up = isUp(d);
    const cls = up ? '' : 'offline';
    const body = <><span className="dot" title={up ? TEXT.peer_up : TEXT.peer_down} /><span><b>{d[1]}</b><small>{meta(d, url)}</small></span><ChevronRight size={18} className="peer-go" aria-hidden="true" /></>;
    if (isHome(d)) return <a key={d[3]} className={cls} href={`${location.origin}${location.pathname}${hash}`} onClick={e => {
      if (e.button !== 0 || e.metaKey || e.ctrlKey) return;
      e.preventDefault();
      onLocal();
    }}>{body}</a>;
    // Behind the ingress proxy a peer's plain-http origin is out of reach: navigating this (possibly
    // https) HA panel there in place is blocked as mixed content, and so are the cross-origin
    // sign-in fetches the jump rides. But the peer's own ingress panel shares this page's origin, so
    // target="_top" navigates there natively inside HA, companion app included (a first build's
    // target="_blank" IP link tossed iOS users out to Safari - owner's report, September 2026). The
    // entry path is "/" plus the panel's YAML key, which is the peer's mDNS hostname under the same
    // slug rule the HA Side Panel card's generated YAML uses (panelSlug in src/lib/auth.js). The
    // hostname comes from the raw configuration_url, not peerOrigin(), which swaps in the IP. A row
    // with no usable hostname falls back to a new-tab link: a top-level http navigation is allowed
    // where an embedded one is not, and a possibly-dead panel link would be strictly worse.
    if (url && proxied) {
      let peerSlug = '';
      try {
        const h = new URL(String(d?.[5] || '')).hostname.replace(/\.local$/i, '');
        if (h && !/^\d+\.\d+\.\d+\.\d+$/.test(h) && !h.includes(':')) peerSlug = h.toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
      } catch {
        /* no configuration_url on this row; the mac derivation below */
      }
      if (!peerSlug) {
        // The rung that fires on most installs: HA's ESPHome integration writes configuration_url
        // with the IP it connects on, and an IP names no panel. But the fleet's hostnames are
        // <base>-<last six hex of mac> (name_add_mac_suffix), so the peer's is this device's base
        // plus the peer's suffix. The endsWith check is the honesty test: a device renamed away
        // from the convention proves the base unknowable, and the new-tab fallback beats a guessed
        // link to a panel that does not exist.
        const ownName = String(device?.name || '').toLowerCase();
        const ownSuffix = mac.replace(/[^a-z0-9]/g, '').slice(-6);
        const peerSuffix = String(d?.[3] || '').toLowerCase().replace(/[^a-z0-9]/g, '').slice(-6);
        if (ownSuffix.length === 6 && peerSuffix.length === 6 && ownName.endsWith(ownSuffix)) peerSlug = (ownName.slice(0, -6) + peerSuffix).replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
      }
      if (peerSlug) {
        // Two panel layouts exist and the link must serve both. Flat: every device is its own
        // sidebar entry at /<slug>. Nested: one visible entry and the rest hidden behind
        // hass_ingress's parent: option at /<parent>/<slug> - the layout that keeps the sidebar to
        // a single "Satellite1 Fleet" item. HA answers a hard 404 for unregistered routes and the
        // panel page is same-origin, so one HEAD probe of the flat path decides: 404 means nested,
        // and the parent is wherever the top window stands (on a child page the first segment is
        // still the parent). The href stays the flat form for middle-click.
        const goPanel = async (e: React.MouseEvent) => {
          if (e.button !== 0 || e.metaKey || e.ctrlKey) return;
          e.preventDefault();
          let target = `/${peerSlug}`;
          try {
            const r = await fetch(target, { method: 'HEAD', cache: 'no-store', signal: AbortSignal.timeout(3000) });
            if (r.status === 404) {
              const seg = window.top!.location.pathname.split('/')[1] || '';
              if (seg && seg !== peerSlug) target = `/${seg}/${peerSlug}`;
            }
          } catch {
            /* an unanswerable probe changes nothing: the flat link is the best guess standing */
          }
          // The proxied twin of the seamless sign-in: the peer cannot be signed in from here, so its
          // password is left in shared HA-origin localStorage under its slug for the peer's own app
          // to claim on boot (see putPanelHandoff, and the boot in src/App.tsx). No handoff, no
          // harm: the peer's own sign-in screen takes over.
          const pw = d?.[7];
          if (pw) putPanelHandoff(peerSlug, pw);
          try {
            window.top!.location.href = target;
          } catch {
            location.href = target;
          }
        };
        return <a key={d[3]} className={cls} href={`/${peerSlug}`} target="_top" onClick={goPanel}>{body}</a>;
      }
      return <a key={d[3]} className={cls} href={peerHref(url)} target="_blank" rel="noopener">{body}</a>;
    }
    return url ? <a key={d[3]} className={cls} href={peerHref(url)} onClick={e => jump(e, url, d[7])}>{body}</a> : <a key={d[3]} className={cls + ' nolink'}>{body}</a>;
  };
  const ownMeta = [area, device?.ip].filter(Boolean).join(' · ');
  const ownRadar = radarModel(myRadar);
  const ownNet = netName(myNet);
  // An empty roster has two honest readings: nothing else in the house, or a roster the device is
  // not allowed to fetch. While actions are blocked the empty line says so, instead of promising
  // rows that cannot arrive.
  return <div className="device-sheet"><span className="eyebrow">DEVICE SWITCHER</span><h2>{label || 'This device'}</h2><p className="muted">{ownMeta}{ownRadar && <>{ownMeta ? ' · ' : ''}<span className={myLit ? 'green' : 'dim'} title={myLit ? TEXT.presence_on : TEXT.presence_off}>●</span> {ownRadar}</>}{ownNet && ` · ${ownNet}`}</p><div className="peer-list">{ordered.map(peerRow)}</div>{peers.length === 0 && <div className="empty">{haBlocked(ha) ? TEXT.no_devices_blocked : TEXT.no_devices}</div>}<button type="button" className="signout" onClick={onSignOut}><LogOut size={14} aria-hidden="true" /><span>{TEXT.logout}</span></button></div>;
}

/** "just now", "12m ago", "3h ago" - the history covers 24 hours, so hours are the ceiling. */
function ago(ts: number) {
  const m = Math.round((Date.now() - ts) / 60000);
  if (m < 1) return TEXT.notif_now;
  return TEXT.notif_ago.replace('%s', m < 60 ? `${m}m` : `${Math.floor(m / 60)}h`);
}
const FILTERS: [string, string][] = [['all', TEXT.notif_all], ['info', TEXT.notif_info], ['warn', TEXT.notif_warn], ['err', TEXT.notif_err], ['arch', TEXT.notif_arch]];
const KIND_LABEL: Record<string, string> = { err: 'ERROR', warn: 'WARN', info: 'INFO', ok: 'INFO' };

/**
 * The bell's sheet: src/lib/notif.js's past 24 hours. Pending entries are left out - their toast is
 * still on screen and may yet be tapped. A tap archives the row and acts as the toast would have.
 * Archived rows stay listed under their own filter, dimmed but tappable: archive means handled,
 * not deleted.
 */
function Notice({ onAct }: { onAct: (t: Toast) => void }) {
  const [filter, setFilter] = useState('all');
  const [, bump] = useState(0);
  useEffect(() => subscribeNotifs(() => bump(n => n + 1)), []);
  const rows = (listNotifs() as (Toast & { ts: number; state: string })[]).filter(e => e.state !== 'pending' && (filter === 'arch' ? e.state === 'archived' : e.state !== 'archived' && (filter === 'all' || filter === (e.kind === 'ok' ? 'info' : e.kind))));
  return <div className="notice-sheet"><div className="sheet-top"><span className="eyebrow">{TEXT.notif_title.toUpperCase()}</span>{notifCount() > 0 && <button onClick={() => archiveAllNotifs()}>{TEXT.notif_clear}</button>}</div><div className="filters">{FILTERS.map(([id, text]) => <button className={filter === id ? 'active' : ''} key={id} onClick={() => setFilter(id)}>{text}</button>)}</div>{rows.length ? rows.map(e => <article key={e.id} className={`notice ${e.kind === 'ok' ? 'info' : e.kind}${e.state === 'archived' ? ' archived' : ''}`} role="button" tabIndex={0} onClick={() => {
      if (e.state !== 'archived') archiveNotif(e.id);
      onAct(e);
    }} onKeyDown={k => {
      if (k.key === 'Enter') (k.currentTarget as HTMLElement).click();
    }}><b>{KIND_LABEL[e.kind] || 'INFO'}{(e.count ?? 0) > 1 ? ` \u00d7${e.count}` : ''}</b><strong>{e.title}</strong><small>{[ago(e.ts), e.sub].filter(Boolean).join(' · ')}</small></article>) : <div className="empty">{filter === 'arch' ? TEXT.notif_empty_arch : TEXT.notif_empty}</div>}<p className="notice-foot">{TEXT.notif_foot}</p></div>;
}
function ControlPanel({
  tab,
  sub,
  ctx
}: {
  tab: Tab;
  sub: string | null;
  ctx: Ctx;
}) {
  if (tab === 'WAKE') return <WakeTab ctx={ctx} />;
  if (tab === 'PRESENCE') return <PresenceTab ctx={ctx} onGoDevice={() => ctx.go('SETTINGS', 'device-info')} />;
  if (tab === 'AUDIO') return <AudioTab ctx={ctx} />;
  return <DiagnosticsTab ctx={ctx} subRoute={sub || 'device-info'} onSubRouteChange={s => ctx.go('SETTINGS', s)} />;
}
