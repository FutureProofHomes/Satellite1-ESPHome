/**
 * The hash routes, and their translation to the design's tab and settings-page names.
 *
 * Routing is on the hash because ESPHome's httpd has a single wildcard handler per method and no
 * notion of client-side routes, so a path router would 404 on every reload but "/". The hash never
 * reaches the server, so a bookmark survives a firmware update - which is why the old names keep
 * resolving (docs/web-ui.md, "What it is").
 *
 * The URL names are not the design's ids on purpose: the design calls Home "NOW", and the names
 * the URL shows are the ones a person would type.
 */

/** URL name -> design tab id, in nav order. */
const TABS = [
  ["home", "NOW"],
  ["wake-word", "WAKE"],
  ["presence", "PRESENCE"],
  ["audio", "AUDIO"],
  ["settings", "SETTINGS"],
];

/** URL name -> the design's SETTINGS_ROUTES slug. The first is the default. */
const SETTINGS_SUBS = [
  ["device", "device-info"],
  ["updates", "updates"],
  ["security", "security"],
  ["logs", "logs"],
  ["integrations", "integrations"],
  ["recovery", "recovery"],
  // Developer builds only (config/satellite1.dev.yaml). Shell code hides it elsewhere, and on a
  // release build Settings sends it to Device Info.
  ["developer", "developer"],
  ["community", "community"],
];

/** URL name -> Audio's page slug. Routing is the default, and the only page while the running XMOS
 *  has no speaker EQ: the shell then hides DSP, and Audio sends #/audio/dsp on to Routing. */
const AUDIO_SUBS = [
  ["routing", "routing"],
  ["dsp", "dsp"],
];

/** The tabs that have pages of their own. */
const SUBS_OF = { SETTINGS: SETTINGS_SUBS, AUDIO: AUDIO_SUBS };

/** Hashes the previous UI used: #/controls and #/config from before the September 2026 rename,
 *  #/diagnostics, whose cards became the Settings pages, and #/settings/amp, whose amplifier card
 *  moved to Developer in October 2026. Bookmarks outlive firmware updates by design, so they must
 *  keep landing somewhere better than the default. */
const LEGACY = { controls: "home", config: "audio/routing", diagnostics: "settings/device", "settings/amp": "settings/developer" };

/**
 * `#/settings/logs?x=1` -> { tab: "SETTINGS", sub: "logs" }. Anything unknown is Home; a Settings
 * or Audio hash with an unknown or missing page is its first page. `sub` is null on the other tabs.
 */
export function parseRoute(hash) {
  let path = String(hash || "").replace(/^#\/?/, "").split("?")[0].toLowerCase().replace(/\/+$/, "");
  path = LEGACY[path] || path;
  const [name, page] = path.split("/");
  const tab = (TABS.find(([n]) => n === name) || TABS[0])[1];
  const subs = SUBS_OF[tab];
  if (!subs) return { tab, sub: null };
  return { tab, sub: (subs.find(([n]) => n === page) || subs[0])[1] };
}

/** The inverse: { tab: "SETTINGS", sub: "device-info" } -> "#/settings/device". */
export function routeHash(tab, sub) {
  const [name, id] = TABS.find(([, t]) => t === tab) || TABS[0];
  const subs = SUBS_OF[id];
  if (!subs) return `#/${name}`;
  return `#/${name}/${(subs.find(([, s]) => s === sub) || subs[0])[0]}`;
}

/** A route's place in reading order: the tabs in nav order, then each tab's pages in theirs. */
const rank = (r) =>
  TABS.findIndex(([, t]) => t === r.tab) * 100 + Math.max(0, (SUBS_OF[r.tab] || []).findIndex(([, s]) => s === r.sub));

/**
 * Which way the page slides between two routes: "fwd" toward a later tab or Settings page, "back"
 * toward an earlier one, null when both name the same page and nothing should move.
 */
export function routeDir(from, to) {
  const a = rank(from);
  const b = rank(to);
  return a === b ? null : b > a ? "fwd" : "back";
}
