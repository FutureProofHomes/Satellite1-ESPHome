/**
 * What the shell hands every tab: the results of the session-wide hooks in src/lib/device.js, plus
 * the router. Those hooks run once per device session in src/components/Satellite1Now.tsx rather
 * than per tab, so changing tabs never re-asks the device (or, behind it, Home Assistant) for data
 * that changes hourly at most. Hooks only one tab needs (useVoice, useMedia, useWakeSlots, useAssist,
 * useRadar, useMaData) are called by that tab, so their polls run only while it is open.
 *
 * Loose on purpose: the payloads are the device's, documented where src/lib/device.js reads them,
 * and the modules that produce them are plain JS.
 */
export type Tab = 'NOW' | 'WAKE' | 'PRESENCE' | 'AUDIO' | 'SETTINGS';

export type Ctx = {
  /** GET /api/sat1/state, polled; null until the first answer. `device.e` maps entity keys to ids. */
  device: any;
  deviceError: string | null;
  /** Entity states from /events, keyed by id ("sensor/Temperature"). */
  states: Record<string, any>;
  /** Whether /events is up. Starts true. */
  connected: boolean;
  log: any[];
  logSeq: number;
  pausedRef: { current: boolean };
  clearLog: () => void;
  /** Call while something displays the log; returns the release. */
  logWatch: () => () => void;
  /** GET /api/sat1/ha: the Home Assistant payload (`ha.d` the data, `ha.rung`, `ha.actions`). */
  ha: any;
  haRefresh: () => Promise<void>;
  haRefreshing: boolean;
  haRead: () => Promise<void>;
  haStale: boolean;
  /** GET /api/sat1/sel: the speaker selection Audio edits. Read once per session: re-fetching it on
   *  every tab change would be a chance for a stale copy to overwrite an edit just made. */
  sel: any;
  selError: string | null;
  selWrite: (next: any) => Promise<any>;
  /** Opens the Home Assistant actions walk-through. On the ctx so any tab's "Show fix" link can open
   *  it without threading a prop through every card in between. */
  onShowFix: () => void;
  /** The open tab and, on Settings, the open page (a SETTINGS_ROUTES slug). */
  tab: Tab;
  sub: string | null;
  /** Navigates by hash, so the back button and bookmarks work. */
  go: (tab: Tab, sub?: string) => void;
};

/** The orb's two colours, as the Home tab's picker sets them and the shell persists them. */
export type Orb = { a: string; b: string };
