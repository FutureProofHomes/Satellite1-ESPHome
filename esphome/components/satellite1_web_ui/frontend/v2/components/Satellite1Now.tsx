import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { createPortal } from 'react-dom';
import { TEXT } from '../../src/copy.js';
import { logout, peerLogin, primeOtherOrigin, probePeer, putPanelHandoff } from '../../src/lib/auth.js';
import { deviceIdentity, entity, haBlocked, onLogAlert, onWriteError, peerOrigin, proxied, useDeviceState, useEvents, useHaData, useSelection } from '../../src/lib/device.js';
import { archiveAllNotifs, archiveNotif, listNotifs, notifCount, setNotifDevice, subscribeNotifs } from '../../src/lib/notif.js';
import { setSparkDevice, sparkRecord } from '../../src/lib/sparkhist.js';
import { dismissToast, subscribeToasts, tapToast, toast, toastIntent } from '../../src/lib/toast.js';
import type { Ctx, Orb, Tab } from '../ctx';
import { AlertTriangle, Info, XCircle, Clock, X, LogOut } from '../icons';
import { parseRoute, routeHash } from '../lib/routes.js';
import { AudioTab } from './AudioTab';
import { Icon, useTheme } from './bits';
import { DiagnosticsTab, SETTINGS_ROUTES } from './DiagnosticsTab';
import { HaGate } from './HaGate';
import { HomeTab } from './HomeTab';
import { MediaBar } from './MediaBar';
import { PresenceTab } from './PresenceTab';
import { WakeTab } from './WakeTab';
type Sheet = 'media' | 'device' | 'notice' | null;
type ToastKind = 'info' | 'warn' | 'error' | 'timer';
/** lib/toast.js's visible toast, and lib/notif.js's history row: the same anatomy. */
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
const TABS: Tab[] = ['NOW', 'WAKE', 'PRESENCE', 'AUDIO', 'SETTINGS'];
const TOAST_ICONS = {
  info: Info,
  warn: AlertTriangle,
  error: XCircle,
  timer: Clock
};
/** The store's kinds in the design's palette. "ok" has no colour of its own there and wears info's. */
const pillKind = (k: string): ToastKind => k === 'err' ? 'error' : k === 'warn' || k === 'timer' ? k : 'info';

/**
 * The header's toast: a view of whatever lib/toast.js says is visible. The store owns the timing,
 * the queue and the ×N coalescing; the pill only draws, swipes and reports taps.
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
      <button type="button" className="toast-x" aria-label={TEXT.dismiss} onClick={() => onDismiss(toast.id)}><X size={14} /></button>
    </div>;
}

/** The pages v1's #/diagnostics cards became, for history rows recorded before the switch. */
const CARD_PAGE: Record<string, string> = {
  log: '#/settings/logs',
  crash: '#/settings/logs',
  firmware: '#/settings/updates'
};

/**
 * What a tapped toast or history row does - one path for both, so the two can never act
 * differently on the same entry. "fix" opens the walk-through; anything else navigates with its
 * intent riding lib/toast.js's handoff. Standing on the destination already fires no hashchange,
 * so that page hears a window event instead.
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
 * guards for a genuinely different device).
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
      // Per level and component, so ten wifi warnings are one toast wearing ×10.
      key: `log-${err ? 'E' : 'W'}-${tag}`,
      ttl: 6000,
      title: (err ? TEXT.log_toast_err : TEXT.log_toast_warn).replace('%s', tag || TEXT.log_toast_dev),
      sub: TEXT.write_failed_go,
      go: '#/settings/logs',
      intent: { card: 'log', level: err ? 'E' : 'W', line: { at, text } }
    });
  }), []);
  // The nudge, on the rising edge only: once on entering a blocked app, again only if it re-enters.
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
function hexToRgba(hex: string, a: number): string {
  if (!hex.startsWith('#')) return `color-mix(in srgb, ${hex} ${Math.round(a * 100)}%, transparent)`;
  const h = hex.replace('#', '');
  const full = h.length === 3 ? h.split('').map(c => c + c).join('') : h;
  const n = parseInt(full, 16);
  return `rgba(${n >> 16 & 255},${n >> 8 & 255},${n & 255},${a})`;
}
function Sheet({
  kind,
  close,
  children
}: {
  kind: Sheet;
  close: () => void;
  children: React.ReactNode;
}) {
  const panelRef = useRef<HTMLElement>(null);
  const dragStartY = useRef<number | null>(null);
  useEffect(() => {
    if (!kind) return;
    document.body.classList.add('has-drawer');
    return () => document.body.classList.remove('has-drawer');
  }, [kind]);
  if (!kind) return null;
  const dismiss = close;
  return createPortal([<div key="scrim" className="scrim" onClick={close} />, <aside key="sheet" ref={panelRef} className="sheet" onClick={e => e.stopPropagation()}><div className="handle" role="button" aria-label="Close" style={{
      touchAction: 'none',
      cursor: 'grab'
    }} onPointerDown={e => {
      if (window.innerWidth >= 1024) return;
      dragStartY.current = e.clientY;
      e.currentTarget.setPointerCapture(e.pointerId);
      if (panelRef.current) panelRef.current.style.transition = 'none';
    }} onPointerMove={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (panelRef.current) {
        panelRef.current.style.transform = `translateX(-50%) translateY(${dy}px)`;
        panelRef.current.style.opacity = String(Math.max(0, 1 - dy / 220));
      }
    }} onPointerUp={e => {
      if (dragStartY.current === null) return;
      const dy = Math.max(0, e.clientY - dragStartY.current);
      if (dy > 80) {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%) translateY(120%)';
          panelRef.current.style.opacity = '0';
          setTimeout(dismiss, 210);
        } else dismiss();
      } else {
        if (panelRef.current) {
          panelRef.current.style.transition = 'transform .22s ease, opacity .22s ease';
          panelRef.current.style.transform = 'translateX(-50%)';
          panelRef.current.style.opacity = '1';
        }
      }
      dragStartY.current = null;
      setTimeout(() => {
        if (panelRef.current) {
          panelRef.current.style.transition = '';
          panelRef.current.style.transform = '';
          panelRef.current.style.opacity = '';
        }
      }, 250);
    }} />{children}</aside>], document.body);
}
const ORB_KEY = 'sat1.orb';
const ORB_DEFAULT: Orb = { a: '#a78bfa', b: '#818cf8' };
function readOrb(): Orb {
  try {
    const o = JSON.parse(localStorage.getItem(ORB_KEY) || 'null');
    if (typeof o?.a === 'string' && typeof o?.b === 'string') return o;
  } catch {
    /* private mode or a hand-edited value: the default stands */
  }
  return ORB_DEFAULT;
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
  const [theme, toggleTheme] = useTheme();
  const [route, setRoute] = useState(() => parseRoute(location.hash));
  useEffect(() => {
    // A v1 bookmark or an unknown hash is rewritten in place to the page it resolved to, so the
    // address bar never disagrees with what is on screen.
    const on = () => {
      const r = parseRoute(location.hash);
      const canon = routeHash(r.tab, r.sub ?? undefined);
      if (location.hash && location.hash !== canon) history.replaceState(null, '', canon);
      setRoute(r);
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
  const [playing, setPlaying] = useState(true);
  // The gate is up from the first signed-in render and gone for good once it fades; the fix is the
  // same screen reopened on demand.
  const [gateDone, setGateDone] = useState(false);
  const [fixOpen, setFixOpen] = useState(false);

  // Settings wants fresh heap and loop figures; everywhere else the only thing that goes stale is
  // the Home Assistant dot, worth one request every ten seconds.
  const { device, deviceError } = useDeviceState(tab === 'SETTINGS' ? 2000 : 10000);
  const events = useEvents();
  const ha = useHaData();
  const selection = useSelection();

  // Dual-origin cookie priming, once the state payload names the other entrance. Never while
  // remote: `device` is the peer's there.
  useEffect(() => {
    if (remote || !primeKey.current || !device) return;
    primeOtherOrigin(primeKey.current, device.name, device.ip);
    primeKey.current = null;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [device, primeKey]);
  useEffect(() => {
    if (deviceError === 'HTTP 401') onAuthLost();
  }, [deviceError, onAuthLost]);
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

  // The sensor sparklines accrue on every tab, not only while Home is open: the change effects
  // catch a moved value, and the 60s beat keeps flat periods accruing points.
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
  const { name: label, area } = deviceIdentity(device, ha.ha);
  useEffect(() => {
    if (label) document.title = label;
  }, [label]);

  // The store's visible toast, and a ghost of the one leaving so its exit can animate.
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
  // `blocked` waits for the gate to leave: while it stands, the verdict is its story to tell.
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
  const orbVars = {
    '--orb-a': orb.a,
    '--orb-b': orb.b,
    '--orb-a20': hexToRgba(orb.a, 0.2),
    '--orb-a35': hexToRgba(orb.a, 0.35),
    '--orb-a55': hexToRgba(orb.a, 0.55)
  } as React.CSSProperties;
  const signOut = async () => {
    setSheet(null);
    await logout();
    location.reload();
  };
  const haText = device?.ha ? TEXT.ha_connected : TEXT.ha_disconnected;
  const openFix = () => setFixOpen(true);
  const activeToast = cur;
  return <main className="app" ref={appRef} data-theme={theme} style={orbVars}><header ref={headerRef} className={`app-header${activeToast ? ' toast-active' : ''}`}><div className="header-left"><div className="header-device-slot"><button className="device-chip" onClick={() => setSheet('device')}><span className={'dot' + (device?.ha ? '' : ' off')} title={haText} /> <span>{label || 'Satellite1'}</span><small>{device?.ip || '\u00a0'}{remote ? ` · ${TEXT.remote_tag}` : ''}</small></button></div><div className="header-toast-slot" aria-live="polite">{ghost && <ToastPill key={`leaving-${ghost.id}`} toast={ghost} leaving onDismiss={() => {}} />}{activeToast && <ToastPill key={`active-${activeToast.id}`} toast={activeToast} onDismiss={dismissToast} onTap={() => {
              tapToast(activeToast.id);
              runAct(activeToast, openFix);
            }} />}</div></div><div className="header-actions"><button className="icon-button" onClick={() => setSheet('notice')} aria-label={bellLabel} title={bellLabel}><Icon name="bell" />{notifN > 0 && <b>{notifN > 99 ? '99+' : notifN}</b>}</button><button className="theme-toggle" onClick={toggleTheme} aria-label={theme === 'dark' ? TEXT.theme_to_light : TEXT.theme_to_dark}><Icon name={theme === 'dark' ? 'sun' : 'moon'} /></button></div></header>
    <div key={tab} className="tab-content-enter">{tab === 'NOW' && <HomeTab ctx={ctx} orb={orb} onOrbColor={onOrbColor} />}
    {tab !== 'NOW' && <ControlPanel key={tab} tab={tab} sub={sub} ctx={ctx} />}</div>
    <nav className="side-nav" aria-label="Sections">{TABS.map(item => <button key={item} className={tab === item ? 'selected' : ''} onClick={() => go(item, item === 'SETTINGS' ? lastSub.current : undefined)}><span>{item === 'NOW' ? 'HOME' : item}</span></button>)}{tab === 'SETTINGS' && <div className="side-sub" role="list">{SETTINGS_ROUTES.map(r => <button key={r.slug} role="listitem" className={'side-sub-item' + (sub === r.slug ? ' on' : '')} onClick={() => go('SETTINGS', r.slug)}>{r.label}</button>)}</div>}<div className="side-nav-foot" style={{
        marginTop: 'auto',
        padding: '12px 8px 88px',
        borderTop: '1px solid var(--line)',
        display: 'flex',
        flexDirection: 'column',
        gap: 2
      }}><span style={{
          fontSize: 13,
          fontWeight: 600,
          color: 'var(--text)',
          opacity: 0.85
        }}>{label || 'Satellite1'}</span><span style={{
          fontSize: 11,
          color: 'var(--muted)'
        }}>{[device?.ip, device?.fw && `Firmware ${device.fw}`].filter(Boolean).join(' · ')}</span><button type="button" onClick={signOut} style={{
          marginTop: 10,
          alignSelf: 'flex-start',
          display: 'inline-flex',
          alignItems: 'center',
          gap: 8,
          background: 'transparent',
          border: '1px solid var(--line)',
          borderRadius: 999,
          padding: '6px 12px',
          minHeight: 34,
          fontSize: 12,
          fontWeight: 600,
          color: 'var(--muted)'
        }}><LogOut size={14} aria-hidden="true" /><span>Sign out</span></button></div></nav>
    <nav className="tabs" data-tab={tab} style={{
      translate: "0px -8px"
    }}><div className="tabs-track" style={{
        transform: navLayer === 0 ? 'translateX(0%)' : 'translateX(-50%)'
      }}><div className="tabs-layer tabs-main">{TABS.map(item => <button key={item} className={tab === item ? 'selected' : ''} onClick={() => go(item, item === 'SETTINGS' ? lastSub.current : undefined)}>{item === 'NOW' ? 'HOME' : item}</button>)}</div><div className="tabs-layer tabs-sub"><button className="tabs-back" aria-label="Back to main menu" onClick={() => {
            lastSub.current = 'device-info';
            go('NOW');
          }}><svg width="16" height="16" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="m10 4-5 4 5 4" /></svg></button><div className="tabs-sub-scroll">{SETTINGS_ROUTES.map(r => <button key={r.slug} className={'tabs-sub-pill' + (sub === r.slug ? ' on' : '')} onClick={() => go('SETTINGS', r.slug)}>{r.label}</button>)}</div></div></div></nav><MediaBar ctx={ctx} playing={playing} setPlaying={setPlaying} />
    <Sheet kind={sheet} close={() => setSheet(null)}>{sheet === 'device' && <DeviceSheet ctx={ctx} label={label} area={area} remote={remote} localMac={localMac} onRemote={onRemote} onLocal={onLocal} onSignOut={signOut} />}{sheet === 'notice' && <Notice onAct={t => {
          setSheet(null);
          runAct(t, openFix);
        }} />}</Sheet>
    {!gateDone && <HaGate ctx={ctx} onDone={() => setGateDone(true)} />}
    {fixOpen && <HaGate ctx={ctx} fix onDone={() => setFixOpen(false)} />}
  </main>;
}
const hostOf = (u: unknown) => String(u || '').replace(/^https?:\/\//, '').replace(/[/:].*$/, '');
const isUp = (d: any[]) => d[6] === 1 || d[6] === '1';
const radarModel = (r: unknown) => r === 2450 || r === 2410 ? `LD${r}` : '';
const netName = (n: unknown) => n === 'e' ? 'Ethernet' : n === 'w' ? 'Wi\u2011Fi' : '';
/** "LD2450 · present", or nothing for a device whose firmware sends no radar fields. */
const radarText = (r: unknown, present: unknown) => {
  const m = radarModel(r);
  return m && `${m} · ${present === 1 || present === true ? 'present' : 'no presence'}`;
};

/**
 * The device switcher (src/shell.jsx's SwitcherSheet in the design's sheet): this device on top,
 * then every other Satellite1 the Home Assistant payload lists, online first. The roster is the
 * `dev` block on /api/sat1/ha - row fields [model, name, area, mac, sw, url, up, pw, net, radar,
 * present] - so nothing here discovers anything; a jump signs in to the peer first and, when the
 * peer's firmware allows it, retargets this app instead of navigating.
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
  // Re-sync the roster on open, then on a 5s beat while the last answer came on rung 1 (the fast
  // statistics path; rung 2 costs ~3s a call and would sit on the queue the jump needs).
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
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const roster: any[][] = ha?.d?.dev || [];
  const peers = roster.filter(d => /satellite1/i.test(d?.[0] || '') && (d?.[3] || '').toLowerCase() !== mac).sort((x, y) => `${x[2]}\u0000${x[1]}`.localeCompare(`${y[2]}\u0000${y[1]}`));
  const ordered = [...peers.filter(isUp), ...peers.filter(d => !isUp(d))];

  // This device's own tags read the live stream rather than the cached roster, so its presence
  // lights the moment the radar does. While remote these are the controlled peer's own.
  const mineRow = roster.find(d => (d?.[3] || '').toLowerCase() === mac);
  const netLive = String(states?.[device?.e?.network]?.value || '');
  const myNet = netLive.startsWith('Eth') ? 'e' : netLive.startsWith('WiFi') ? 'w' : mineRow?.[8];
  const modLive = String(states?.[device?.e?.radar_module]?.value || '').toLowerCase();
  const myRadar = modLive.includes('2450') ? 2450 : modLive.includes('2410') ? 2410 : mineRow?.[9];
  const presLive = states?.['binary_sensor/Room Presence'] || states?.['binary_sensor/Presence'];
  const myPres = presLive ? presLive.value === true || presLive.state === 'ON' : mineRow?.[10];
  const myLit = !!radarModel(myRadar) && (myPres === 1 || myPres === true);
  const peerHref = (base: string) => `${String(base).replace(/\/+$/, '')}/${hash}`;

  // The seamless jump: sign in to the peer before leaving so its page opens as the app, and try the
  // single-origin switch first. Anything that declines falls back to plain navigation; the href
  // stays real underneath for middle-click and open-in-new-tab.
  const jump = async (e: React.MouseEvent, url: string, pw: string | undefined) => {
    if (!pw || e.button !== 0 || e.metaKey || e.ctrlKey) return;
    e.preventDefault();
    const origin = String(url).replace(/\/+$/, '');
    const peer = await peerLogin(origin, pw);
    if (!peer) {
      location.href = peerHref(url);
      return;
    }
    if (await probePeer(origin, peer.key)) {
      onRemote({ base: origin, key: peer.key }, remote ? null : mac);
      return;
    }
    const base = location.hostname.endsWith('.local') && peer.name && /^[a-z0-9-]+$/i.test(peer.name) ? `http://${peer.name}.local${location.port ? `:${location.port}` : ''}` : origin;
    location.href = `${base}/?key=${peer.key}${hash}`;
  };
  const isHome = (d: any[]) => !!remote && !!localMac && (d?.[3] || '').toLowerCase() === localMac;
  const meta = (d: any[], url: string) => isHome(d) ? TEXT.switcher_home : !isUp(d) ? 'Offline' : [d[2], hostOf(url), netName(d[8]), radarText(d[9], d[10])].filter(Boolean).join(' · ');
  const peerRow = (d: any[]) => {
    const url = peerOrigin(d);
    const up = isUp(d);
    const cls = up ? '' : 'offline';
    const body = <><span className="dot" title={up ? TEXT.peer_up : TEXT.peer_down} /><span><b>{d[1]}</b><small>{meta(d, url)}</small></span><Icon name="chevron" /></>;
    if (isHome(d)) return <a key={d[3]} className={cls} href={`${location.origin}${location.pathname}${hash}`} onClick={e => {
      if (e.button !== 0 || e.metaKey || e.ctrlKey) return;
      e.preventDefault();
      onLocal();
    }}>{body}</a>;
    // Behind the ingress proxy a peer's plain-http origin is out of reach, but its own ingress
    // panel is not: same origin as this page, at "/" plus the slug of its mDNS hostname.
    if (url && proxied) {
      let peerSlug = '';
      try {
        const h = new URL(String(d?.[5] || '')).hostname.replace(/\.local$/i, '');
        if (h && !/^\d+\.\d+\.\d+\.\d+$/.test(h) && !h.includes(':')) peerSlug = h.toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
      } catch {
        /* no configuration_url on this row; the mac derivation below */
      }
      if (!peerSlug) {
        // HA usually records the IP, which names no panel - but the fleet's hostnames are
        // <base>-<last six hex of mac>, so the peer's is this device's base plus its suffix.
        const ownName = String(device?.name || '').toLowerCase();
        const ownSuffix = mac.replace(/[^a-z0-9]/g, '').slice(-6);
        const peerSuffix = String(d?.[3] || '').toLowerCase().replace(/[^a-z0-9]/g, '').slice(-6);
        if (ownSuffix.length === 6 && peerSuffix.length === 6 && ownName.endsWith(ownSuffix)) peerSlug = (ownName.slice(0, -6) + peerSuffix).replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
      }
      if (peerSlug) {
        // Flat panels live at /<slug>, nested ones at /<parent>/<slug>; HA 404s an unregistered
        // route, so one HEAD probe of the flat path decides.
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
  return <div className="device-sheet"><span className="eyebrow">DEVICE SWITCHER</span><h2>{label || 'This device'}</h2><p className="muted">{ownMeta}{ownRadar && <>{ownMeta ? ' · ' : ''}<span className={myLit ? 'green' : 'dim'} title={myLit ? TEXT.presence_on : TEXT.presence_off}>●</span> {ownRadar}</>}{ownNet && ` · ${ownNet}`}</p><div className="peer-list">{ordered.map(peerRow)}</div>{peers.length === 0 && <div className="empty">{haBlocked(ha) ? TEXT.no_devices_blocked : TEXT.no_devices}</div>}<button type="button" onClick={onSignOut} style={{
      marginTop: 10,
      alignSelf: 'flex-start',
      display: 'inline-flex',
      alignItems: 'center',
      gap: 8,
      background: 'transparent',
      border: '1px solid var(--line)',
      borderRadius: 999,
      padding: '6px 12px',
      minHeight: 34,
      fontSize: 12,
      fontWeight: 600,
      color: 'var(--muted)'
    }}><LogOut size={14} aria-hidden="true" /><span>Sign out</span></button></div>;
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
 * The bell's sheet: lib/notif.js's past 24 hours. Pending entries are left out - their toast is
 * still on screen and may yet be tapped. A tap archives the row and acts as the toast would have.
 */
function Notice({ onAct }: { onAct: (t: Toast) => void }) {
  const [filter, setFilter] = useState('all');
  const [, bump] = useState(0);
  useEffect(() => subscribeNotifs(() => bump(n => n + 1)), []);
  const rows = (listNotifs() as (Toast & { ts: number; state: string })[]).filter(e => e.state !== 'pending' && (filter === 'arch' ? e.state === 'archived' : e.state !== 'archived' && (filter === 'all' || filter === (e.kind === 'ok' ? 'info' : e.kind))));
  return <div className="notice-sheet"><div className="sheet-top"><span className="eyebrow">{TEXT.notif_title.toUpperCase()}</span><button onClick={() => archiveAllNotifs()}>{TEXT.notif_clear}</button></div><div className="filters">{FILTERS.map(([id, text]) => <button className={filter === id ? 'active' : ''} key={id} onClick={() => setFilter(id)}>{text}</button>)}</div>{rows.length ? rows.map(e => <article key={e.id} className={`notice ${e.kind === 'ok' ? 'info' : e.kind}${e.state === 'archived' ? ' archived' : ''}`} role="button" tabIndex={0} onClick={() => {
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
