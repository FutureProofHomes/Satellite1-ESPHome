/**
 * Mock data standing in for the device's live APIs (/api/sat1/state, /events, /api/sat1/ha).
 * Values are shaped after real payloads so every card renders populated.
 */

export const DEVICE = {
  name: 'satellite1-a4c2f8',
  label: 'Living Room Satellite',
  area: 'Living Room',
  ip: '192.168.4.31',
  mac: '74:4d:bd:a4:c2:f8',
  fw: '25.9.3',
  esphome: '2025.9.1',
  xmos: 'v1.3.2',
  built: 'Sep 24 2026, 18:12',
  radarModule: 'LD2450',
  radarFw: 'V2.02.23',
};

/** Deterministic pseudo-random walk for the sensor sparklines. */
function walk(seed: number, n: number, base: number, amp: number) {
  const pts: number[] = [];
  let v = base;
  let s = seed;
  for (let i = 0; i < n; i++) {
    s = (s * 1103515245 + 12345) & 0x7fffffff;
    v += ((s / 0x7fffffff) - 0.5) * amp;
    pts.push(v);
  }
  return pts;
}

export const SPARKS: Record<string, number[]> = {
  temp: walk(7, 24, 21.4, 0.35),
  humidity: walk(13, 24, 46, 1.6),
  lux: walk(29, 24, 180, 28),
};

export const TRANSCRIPT = [
  { heard: true, text: 'turn on the living room lights' },
  { heard: false, text: 'Turned on the lights.' },
  { heard: true, text: 'set a timer for ten minutes' },
  { heard: false, text: 'Timer set for 10 minutes.' },
];

export const PEERS = [
  { name: 'Kitchen Satellite', area: 'Kitchen', host: '192.168.4.32', up: true, net: 'w', radar: 2450, present: true },
  { name: 'Office Satellite', area: 'Office', host: '192.168.4.35', up: true, net: 'e', radar: 2410, present: false },
  { name: 'Bedroom Satellite', area: 'Bedroom', host: '192.168.4.38', up: false, net: 'w', radar: 2450, present: false },
];

export type Notif = {
  id: number;
  kind: 'err' | 'warn' | 'ok' | 'info';
  title: string;
  sub?: string;
  ts: number;
  count: number;
  state: 'active' | 'archived';
};

export const NOTIFS: Notif[] = [
  { id: 1, kind: 'info', title: 'Firmware 25.9.4 is available', sub: 'Tap for Diagnostics', ts: Date.now() - 14 * 60000, count: 1, state: 'active' },
  { id: 2, kind: 'warn', title: 'Warning from wifi', sub: 'Tap for the device log', ts: Date.now() - 3 * 3600000, count: 3, state: 'active' },
  { id: 3, kind: 'ok', title: 'Live updates restored', ts: Date.now() - 5 * 3600000, count: 1, state: 'archived' },
];

/** The Home Assistant payload's area/player tree. Rows: [id, name, caps, avail]. */
export const HA_AREAS = [
  {
    i: 'living_room',
    n: 'Living Room',
    p: [
      ['media_player.living_room_sonos', 'Living Room Sonos', 3, 1],
      ['media_player.satellite1_a4c2f8', 'Living Room Satellite', 4, 1],
      ['media_player.tv_living_room', 'Living Room TV', 2, 1],
    ] as [string, string, number, number][],
  },
  {
    i: 'kitchen',
    n: 'Kitchen',
    p: [
      ['media_player.kitchen_display', 'Kitchen Display', 3, 1],
      ['media_player.satellite1_kitchen', 'Kitchen Satellite', 3, 1],
    ] as [string, string, number, number][],
  },
  {
    i: 'office',
    n: 'Office',
    p: [
      ['media_player.office_homepod', 'Office HomePod', 1, 0],
      ['media_player.satellite1_office', 'Office Satellite', 3, 1],
    ] as [string, string, number, number][],
  },
];

export const HA_LOOSE: [string, string, number, number][] = [
  ['media_player.all_sonos', 'All Sonos', 3, 1],
  ['media_player.chromecast_shadow', 'Chromecast', 1, 1],
];

export const LOG_LINES: { lvl: string; at: string; tag: string; text: string }[] = [
  { lvl: 'I', at: '18:42:01.118', tag: 'app', text: 'ESPHome version 2025.9.1 compiled on Sep 24 2026' },
  { lvl: 'D', at: '18:42:03.402', tag: 'sensor', text: "'Temperature': Sending state 21.43750 °C with 1 decimals of accuracy" },
  { lvl: 'D', at: '18:42:03.921', tag: 'micro_wake_word', text: 'Streaming inference latency 41 ms' },
  { lvl: 'I', at: '18:42:05.010', tag: 'voice_assistant', text: 'Waiting for wake word' },
  { lvl: 'W', at: '18:42:11.223', tag: 'wifi', text: 'Rate limit hit, retrying in 640 ms' },
  { lvl: 'D', at: '18:42:12.850', tag: 'ld2450', text: 'Target 1 moved to (-38, 214) cm, v=12 cm/s' },
  { lvl: 'I', at: '18:42:14.771', tag: 'media_player', text: "State changed to 'playing'" },
  { lvl: 'D', at: '18:42:16.204', tag: 'sensor', text: "'Ambient Light': Sending state 182.00000 lx" },
  { lvl: 'V', at: '18:42:17.001', tag: 'api', text: 'Connection from Home Assistant established (encrypted)' },
  { lvl: 'D', at: '18:42:18.532', tag: 'tas2780', text: 'DVC set to 74% (mode 2, high gain)' },
];

export const WAKE_SOURCES = [
  { id: 'mic', label: 'Device microphones', on: true },
  { id: 'voicetap', label: 'VoiceTap (browser)', on: true },
  { id: 'ha', label: 'Home Assistant Assist', on: false },
];
