/**
 * The hash routes, and their translation to the design's tab and settings-page names.
 *
 * Routing stays on the hash for the reason src/shell.jsx gives: ESPHome's httpd has no client-side
 * routes, so a path router would 404 on every reload but "/", and a hash never reaches the server,
 * so a bookmark survives a firmware update. That last part is why the old names keep resolving.
 *
 * The URL names are not the design's ids on purpose: the design calls Home "NOW" and the
 * amplifier page "audio", and #/settings/audio beside #/audio would read as the same page.
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
const SUBS = [
  ["device", "device-info"],
  ["updates", "updates"],
  ["security", "security"],
  ["logs", "logs"],
  ["integrations", "integrations"],
  ["recovery", "recovery"],
  ["amp", "audio"],
  ["community", "community"],
];

/** Hashes from before the redesign. */
const LEGACY = { controls: "home", config: "audio", diagnostics: "settings/device" };

/**
 * `#/settings/logs?x=1` -> { tab: "SETTINGS", sub: "logs" }. Anything unknown is Home; a settings
 * hash with an unknown or missing page is its first page. `sub` is null off Settings.
 */
export function parseRoute(hash) {
  let path = String(hash || "").replace(/^#\/?/, "").split("?")[0].toLowerCase().replace(/\/+$/, "");
  path = LEGACY[path] || path;
  const [name, page] = path.split("/");
  const tab = (TABS.find(([n]) => n === name) || TABS[0])[1];
  if (tab !== "SETTINGS") return { tab, sub: null };
  return { tab, sub: (SUBS.find(([n]) => n === page) || SUBS[0])[1] };
}

/** The inverse: { tab: "SETTINGS", sub: "audio" } -> "#/settings/amp". */
export function routeHash(tab, sub) {
  const name = (TABS.find(([, t]) => t === tab) || TABS[0])[0];
  if (name !== "settings") return `#/${name}`;
  return `#/settings/${(SUBS.find(([, s]) => s === sub) || SUBS[0])[0]}`;
}
