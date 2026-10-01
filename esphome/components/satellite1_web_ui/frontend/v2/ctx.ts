/**
 * What the shell hands every tab: the same object src/shell.jsx builds for the v1 routes, from the
 * hooks in src/lib/device.js, plus the router. Hooks only one tab needs (useVoice, useMedia,
 * useWakeSlots, useAssist, useRadar, useMaData) are called by that tab, as in v1, so their polls run
 * only while it is open.
 *
 * Loose on purpose: the payloads are the device's, documented where device.js reads them, and the
 * v1 code that consumes them is plain JS.
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
  /** GET /api/sat1/sel: the speaker selection Audio edits. */
  sel: any;
  selError: string | null;
  selWrite: (next: any) => Promise<any>;
  /** Opens the Home Assistant actions walk-through. */
  onShowFix: () => void;
  /** The open tab and, on Settings, the open page (a SETTINGS_ROUTES slug). */
  tab: Tab;
  sub: string | null;
  /** Navigates by hash, so the back button and bookmarks work. */
  go: (tab: Tab, sub?: string) => void;
};

/** The orb's two colours, as the Home tab's picker sets them and the shell persists them. */
export type Orb = { a: string; b: string };
