/**
 * Satellite1 Web UI - the on-device control app, recreated from the FutureProofHomes
 * Satellite1-ESPHome repository (esphome/components/satellite1_web_ui/frontend).
 *
 * The stylesheet is the firmware's own app.css, ported verbatim into index.css; the markup
 * below mirrors shell.jsx with the five routes, the media footer and the drawers, running on
 * local mock state in place of the device APIs.
 */
import React, { useEffect, useState } from 'react';
import { Chevron, Hint, N_AUDIO, N_BELL, N_DIAG, N_HOME, N_OUT, N_PRES, N_SEARCH, N_WAKE, T_ICONS, useDrawer, useSheetDrag } from './ui';
import { DEVICE, NOTIFS, PEERS, Notif } from './mock';
import { HomeRoute } from './HomeRoute';
import { WakeWordsRoute } from './WakeWordsRoute';
import { AudioRoute } from './AudioRoute';
import { PresenceRoute } from './PresenceRoute';
import { DiagnosticsRoute } from './DiagnosticsRoute';
import { MediaFooter } from './MediaFooter';
import { LoginScreen } from './LoginScreen';
import { SetupWizard } from './SetupWizard';
const ROUTES = [{
  id: 'home',
  label: 'Home',
  view: HomeRoute,
  icon: N_HOME
}, {
  id: 'wake-word',
  label: 'Wake Words',
  view: WakeWordsRoute,
  icon: N_WAKE
}, {
  id: 'audio',
  label: 'Audio',
  view: AudioRoute,
  icon: N_AUDIO
}, {
  id: 'presence',
  label: 'Presence',
  view: PresenceRoute,
  icon: N_PRES
}, {
  id: 'diagnostics',
  label: 'Diagnostics',
  view: DiagnosticsRoute,
  icon: N_DIAG
}];

/* ------------------------------------------------------------------ */
/* Theme                                                               */
/* ------------------------------------------------------------------ */

function ThemeSwitch() {
  const [theme, setTheme] = useState<'light' | 'dark'>('light');
  const dark = theme === 'dark';
  const label = dark ? 'Switch to the light theme' : 'Switch to the dark theme';
  const toggle = () => {
    const next = dark ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    setTheme(next);
  };
  return <button className="icon theme" aria-label={label} title={label} onClick={toggle}>
      {dark ? <svg className="theme-i" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round">
          <circle cx="8" cy="8" r="3.1" />
          <path d="M8 1v1.5M8 13.5V15M1 8h1.5M13.5 8H15M3.1 3.1l1.1 1.1M11.8 11.8l1.1 1.1M12.9 3.1l-1.1 1.1M4.2 11.8l-1.1 1.1" />
        </svg> : <svg className="theme-i" viewBox="0 0 16 16" fill="currentColor">
          <path d="M13.9 10.4A6.1 6.1 0 0 1 5.6 2.1 6.5 6.5 0 1 0 13.9 10.4Z" />
        </svg>}
    </button>;
}

/* ------------------------------------------------------------------ */
/* The route menu                                                      */
/* ------------------------------------------------------------------ */

function NavPane({
  route,
  go,
  open,
  onClose,
  onLogout
}: {
  route: string;
  go: (id: string) => void;
  open: boolean;
  onClose: () => void;
  onLogout: () => void;
}) {
  useDrawer('nav', open, onClose);
  return <div className={`scrim navscrim${open ? ' on' : ''}`} onClick={onClose} aria-hidden={!open}>
      <nav className="navpane" onClick={e => e.stopPropagation()}>
        {ROUTES.map(r => <button key={r.id} className={`navpane-item${route === r.id ? ' on' : ''}`} onClick={() => {
        go(r.id);
        onClose();
      }}>
            {r.icon}
            {r.label}
          </button>)}
        <div className="navpane-foot">
          <div className="navpane-dev">{DEVICE.label}</div>
          <div className="navpane-fw">Firmware {DEVICE.fw}</div>
          {/* The real reload-into-the-boot-probe, in preview terms: the session is gone, so the
              login screen is the next thing this browser sees. */}
          <button className="btn ghost sm navpane-logout" onClick={() => {
          onClose();
          onLogout();
        }}>
            {N_OUT}
            Sign out on this browser
          </button>
        </div>
      </nav>
    </div>;
}

/* ------------------------------------------------------------------ */
/* The device switcher                                                 */
/* ------------------------------------------------------------------ */

function SwitcherSheet({
  onClose
}: {
  onClose: () => void;
}) {
  useDrawer('switcher', true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, -1);
  const [showOff, setShowOff] = useState(false);
  const varTags = (net: string, radar: number, present: boolean) => <>
      <span className={`vtag${present ? ' lit' : ''}`} title={present ? 'Presence detected' : 'No presence'}>
        {`LD${radar}`}
      </span>
      <span className="vtag">{net === 'e' ? 'Ethernet' : 'WiFi'}</span>
    </>;
  const peerSub = (room: string, host: string) => <div className="peer-sub">
      <strong>{room}</strong>
      {' \u2502 '}
      {host}
    </div>;
  const online = PEERS.filter(p => p.up);
  const offline = PEERS.filter(p => !p.up);
  const peerRow = (p: (typeof PEERS)[0]) => <a key={p.host} className={`peer go${p.up ? '' : ' off'}`} href={`http://${p.host}/`} onClick={e => e.preventDefault()}>
      <div className="row">
        <span className={`dot${p.up ? ' ok' : ''}`} title={p.up ? 'Reachable' : 'Offline'} />
        <span className="grow">{p.name}</span>
        {varTags(p.net, p.radar, p.present)}
      </div>
      {peerSub(p.area, p.host)}
    </a>;
  return <div className="scrim" onClick={onClose}>
      <div className="sheet" style={dragStyle || undefined} {...drag} onClick={e => e.stopPropagation()}>
        <div className="mgroup-head dim sm" data-grab>
          Device Switcher
          <Hint text="Every Satellite1 Home Assistant can see. Tapping a row signs you in over there - the page keeps this route." />
        </div>
        <div className="peer here">
          <div className="row">
            <span className="dot ok" title="Home Assistant connected" />
            <span className="grow">{DEVICE.label}</span>
            {varTags('w', 2450, true)}
          </div>
          {peerSub(DEVICE.area, DEVICE.ip)}
        </div>
        {online.map(peerRow)}
        {offline.length > 0 && <>
            <button className="offhead" aria-expanded={showOff} onClick={() => setShowOff(!showOff)}>
              <span className="grow">{`Offline (${offline.length})`}</span>
              <Chevron down={showOff} cls="caret-s" />
            </button>
            {showOff && offline.map(peerRow)}
          </>}
        <button className="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
      </div>
    </div>;
}

/* ------------------------------------------------------------------ */
/* Notifications: the bell, the toast, the drawer                      */
/* ------------------------------------------------------------------ */

function ago(ts: number) {
  const m = Math.round((Date.now() - ts) / 60000);
  if (m < 1) return 'just now';
  return m < 60 ? `${m}m ago` : `${Math.floor(m / 60)}h ago`;
}
const NOTIF_FILTERS: [string, string][] = [['all', 'All'], ['info', 'Info'], ['warn', 'Warnings'], ['err', 'Errors'], ['arch', 'Archive']];
function NotifDrawer({
  notifs,
  onArchive,
  onClearAll,
  onClose
}: {
  notifs: Notif[];
  onArchive: (id: number) => void;
  onClearAll: () => void;
  onClose: () => void;
}) {
  useDrawer('notif', true, onClose);
  const [dragStyle, drag] = useSheetDrag(onClose, -1);
  const [filter, setFilter] = useState('all');
  const rows = notifs.filter(e => {
    if (filter === 'arch') return e.state === 'archived';
    return e.state === 'active' && (filter === 'all' || e.kind === filter);
  });
  return <div className="scrim" onClick={onClose}>
      <div className="sheet npane" style={dragStyle || undefined} {...drag} onClick={ev => ev.stopPropagation()}>
        <div className="mgroup-head dim sm" data-grab>
          Notifications
          <Hint text="The last 24 hours of toasts. Tap a row to act on it, or swipe it right to archive." />
          {notifs.some(e => e.state === 'active') && <button className="btn ghost sm nclear" onClick={onClearAll}>
              Clear all
            </button>}
        </div>
        <div className="npills" role="tablist">
          {NOTIF_FILTERS.map(([id, label]) => <button key={id} className={`npill p-${id}${filter === id ? ' on' : ''}`} role="tab" aria-selected={filter === id} onClick={() => setFilter(id)}>
              {label}
            </button>)}
        </div>
        <div className="nlist">
          {rows.map(e => <div key={e.id} className="nrow-wrap">
              <div className={`nrow k-${e.kind}${e.state === 'archived' ? ' arch' : ''}`}>
                <button className="nrow-hit" onClick={() => onArchive(e.id)}>
                  <span className="ntoast-i">{T_ICONS[e.kind]}</span>
                  <span className="ntoast-b">
                    <span className="ntoast-t">{e.title}</span>
                    <span className="ntoast-s">
                      {ago(e.ts)}
                      {e.sub ? ` \u00b7 ${e.sub}` : ''}
                    </span>
                  </span>
                  {e.count > 1 && <span className="ntoast-n">{`\u00d7${e.count}`}</span>}
                </button>
                {e.state !== 'archived' && <button className="ntoast-x" aria-label="Dismiss" onClick={() => onArchive(e.id)}>
                    <svg viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" aria-hidden="true">
                      <path d="M2.8 2.8l6.4 6.4M9.2 2.8l-6.4 6.4" />
                    </svg>
                  </button>}
              </div>
            </div>)}
          {rows.length === 0 && <p className="dim sm nempty">{filter === 'arch' ? 'Nothing archived yet.' : 'All caught up.'}</p>}
        </div>
        <p className="sheet-foot">Kept for 24 hours, per device, in this browser.</p>
        <button className="mpanel-handle" data-grab aria-label="Close" onClick={onClose} />
      </div>
    </div>;
}

/** One demo toast beside the route tab, a few seconds in - tap for Diagnostics, ✕ dismisses. */
function ToastHost({
  go,
  onLive
}: {
  go: (id: string) => void;
  onLive: (v: boolean) => void;
}) {
  const [shown, setShown] = useState(false);
  const [out, setOut] = useState(false);
  useEffect(() => {
    const t = setTimeout(() => {
      setShown(true);
      onLive(true);
    }, 5000);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const dismiss = () => {
    setOut(true);
    onLive(false);
    setTimeout(() => setShown(false), 240);
  };
  if (!shown) return null;
  return <div className="ntoasts">
      <div className={`ntoast k-info${out ? ' out' : ''}`}>
        <button className="ntoast-hit" disabled={out} onClick={() => {
        dismiss();
        go('diagnostics');
      }}>
          <span className="ntoast-i">{T_ICONS.info}</span>
          <span className="ntoast-b">
            <span className="ntoast-t">Firmware 25.9.4 is available</span>
            <span className="ntoast-s">Tap for Diagnostics</span>
          </span>
        </button>
        <button className="ntoast-x" aria-label="Dismiss" disabled={out} onClick={dismiss}>
          <svg viewBox="0 0 12 12" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" aria-hidden="true">
            <path d="M2.8 2.8l6.4 6.4M9.2 2.8l-6.4 6.4" />
          </svg>
        </button>
      </div>
    </div>;
}

/* ------------------------------------------------------------------ */
/* Shell                                                               */
/* ------------------------------------------------------------------ */

/**
 * The gatekeeper around the app, mirroring shell.jsx's boot phases: the app when a session
 * exists, the login screen after sign-out, and the onboarding wizard as the factory-fresh
 * entry (reachable from the login screen's preview link; the wizard's ending hands over to
 * the login screen with the VoiceTap window already opening - the "magical ending").
 */
export function Satellite1WebUI() {
  const [phase, setPhase] = useState<'in' | 'login' | 'setup'>('in');
  const [autoPair, setAutoPair] = useState(false);
  if (phase === 'setup') {
    return <SetupWizard onDone={() => {
      setAutoPair(true);
      setPhase('login');
    }} />;
  }
  if (phase === 'login') {
    return <LoginScreen autoPair={autoPair} onSignedIn={() => {
      setAutoPair(false);
      setPhase('in');
    }} onSetup={() => setPhase('setup')} />;
  }
  return <AppInner onLogout={() => setPhase('login')} />;
}
function AppInner({
  onLogout
}: {
  onLogout: () => void;
}) {
  const [route, setRoute] = useState('home');
  const [nav, setNav] = useState(false);
  const [switcher, setSwitcher] = useState(false);
  const [notifOpen, setNotifOpen] = useState(false);
  const [toastLive, setToastLive] = useState(false);
  const [notifs, setNotifs] = useState<Notif[]>(NOTIFS);
  // The search drawer behind the magnifying glass: the opener is the top bar's, the drawer
  // renders inside MediaFooter (which owns the Music Assistant connection it runs on) - the
  // same split shell.jsx draws.
  const [searchOpen, setSearchOpen] = useState(false);

  // Light is the default; make sure a remount starts clean.
  useEffect(() => {
    if (!document.documentElement.dataset.theme) document.documentElement.dataset.theme = 'light';
  }, []);
  const go = (id: string) => {
    setRoute(id);
    window.scrollTo({
      top: 0
    });
  };
  const active = ROUTES.find(r => r.id === route) || ROUTES[0];
  const View = active.view;
  const badge = notifs.filter(n => n.state === 'active').length;
  return <div className="app">
      <div className="stickhead">
        <header className="topbar">
          <button className="icon" aria-label="Menu" aria-expanded={nav} onClick={() => setNav(v => !v)}>
            <span className="burger" />
          </button>
          <button className="title" onClick={() => setSwitcher(true)}>
            <span className="dot ok" title="Home Assistant connected" aria-label="Home Assistant connected" />
            <span className="tname">{DEVICE.label}</span>
            <Chevron down cls="caret" />
          </button>
          <button className="icon nsearch" aria-label="Search music" title="Search music" onClick={() => setSearchOpen(true)}>
            {N_SEARCH}
          </button>
          <button className="icon nbell" aria-label={`Notifications (${badge})`} title="Notifications" onClick={() => setNotifOpen(true)}>
            {N_BELL}
            {badge > 0 && <span className="nbadge">{badge}</span>}
          </button>
          <ThemeSwitch />
        </header>
      </div>

      <main className="wrap">
        <div className={`rtab-row${toastLive ? ' tlive' : ''}`}>
          <div className="rtab">
            <span className="rtab-i">{active.icon}</span>
            <span className="rtab-l">{active.label}</span>
          </div>
        </div>
        <View go={go} />
      </main>

      <MediaFooter search={searchOpen} onSearchClose={() => setSearchOpen(false)} />

      <ToastHost go={go} onLive={setToastLive} />

      <NavPane route={route} go={go} open={nav} onClose={() => setNav(false)} onLogout={onLogout} />
      {switcher && <SwitcherSheet onClose={() => setSwitcher(false)} />}
      {notifOpen && <NotifDrawer notifs={notifs} onArchive={id => setNotifs(ns => ns.map(n => n.id === id ? {
      ...n,
      state: 'archived' as const
    } : n))} onClearAll={() => setNotifs(ns => ns.map(n => n.state === 'active' ? {
      ...n,
      state: 'archived' as const
    } : n))} onClose={() => setNotifOpen(false)} />}
    </div>;
}