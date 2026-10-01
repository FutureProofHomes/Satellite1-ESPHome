/**
 * The setup wizard's decisions, out of the component so they can be tested. The component
 * (src/components/SetupWizard.tsx) carries the hardware findings behind the launcher and the
 * redirect; docs/web-ui-copy.md ("The onboarding wizard") walks the whole flow.
 */

/**
 * Whether the page is inside an OS captive-portal sheet rather than a real browser. The iOS/macOS
 * sheet is WebKit without the `Safari/` token every real browser on Apple platforms carries (Chrome
 * and Firefox on iOS carry it too); Android's is a bare WebView (the `; wv)` marker) or names its
 * CaptivePortalLogin app. A wrong "true" costs one tap on Continue here instead, a wrong "false"
 * strands someone in a sheet that closes before the flow's ending, so the Apple test leans toward
 * sheet.
 */
export function inCaptiveSheet(ua) {
  if (/AppleWebKit/i.test(ua) && !/Safari\//i.test(ua)) return true;
  return /; wv\)/i.test(ua) || /CaptivePortalLogin/i.test(ua);
}

/**
 * Where a fresh load lands, from the device's own facts, so a reload finds the person where they
 * are: mid-AP, mid-join, or back on the home network mid-wizard. A connected station means the WiFi
 * half is done however it was done, so Home Assistant is next. Otherwise only the captive sheet
 * gets the launcher, and a real browser - including one whose join failed or is retrying - gets the
 * network list. `?setup=prime` is the launcher's own reload (straight to its button), and
 * `?setup=go` is its button's link, which a sheet that kept the tap must not loop back from: the
 * network list works in the sheet, and the joining step's fallback address covers that path's
 * ending.
 */
export function entryStep(wifi, search, ua) {
  if (wifi?.connected) return { step: "haconnect", launched: false };
  if (/[?&]setup=prime\b/.test(search)) return { step: "launcher", launched: true };
  if (inCaptiveSheet(ua) && !/[?&]setup=go\b/.test(search)) return { step: "launcher", launched: false };
  return { step: "network", launched: false };
}

/** A scan read: an empty answer while a rescan runs keeps the list already on screen. */
export const mergeScan = (prev, list) => (list ? (list.length || !prev ? list : prev) : prev);

/**
 * What stops a join before it is sent: "ssid" with no name, "short" for a key the device refuses.
 * An empty key is allowed even on a secured row, because the manual form is always "secured" and a
 * hidden network can be open.
 */
export function joinProblem(ssid, password, secured) {
  if (!ssid.trim()) return "ssid";
  if (secured && password.length > 0 && password.length < 8) return "short";
  return null;
}

/** Polls still answering but never connecting past this read as a wrong password. */
export const JOIN_SLOW_MS = 45000;

/**
 * One wifi/status read during the join. "connected" and "gone" (the setup AP closed under the
 * page, so the join succeeded) both arm the redirect probes; probing earlier would see the device
 * answer its own .local name over the AP.
 */
export function joinRead(st, elapsed) {
  if (!st) return "gone";
  if (st.connected) return "connected";
  return elapsed > JOIN_SLOW_MS ? "slow" : "trying";
}

/** Whether the page is already served from the device's home-network name: no gap left to cross. */
export const onHomeOrigin = (hostname, host) => !!host && hostname.toLowerCase() === `${host.toLowerCase()}.local`;

/**
 * Where the redirect probes look: the .local name, and the station IP once wifi/status has shown
 * it during the brief AP+STA overlap, because Android browsers cannot resolve .local.
 */
export function probeOrigins(host, ip) {
  const origins = [];
  if (host) origins.push(`http://${host}.local`);
  if (ip) origins.push(`http://${ip}`);
  return origins;
}

/**
 * The redirect's trigger: two consecutive answers from the same origin. The phone's hop off the
 * dying AP passes through cellular, then the home WiFi, then validation, and one answer can thread
 * a gap the navigation a beat later cannot.
 */
export function probeStreak(need = 2) {
  const runs = {};
  return (origin, ok) => {
    runs[origin] = ok ? (runs[origin] || 0) + 1 : 0;
    return runs[origin] >= need;
  };
}

/** The actions verdict that lets sign-in speak its code: allowed (1), or a Home Assistant too old
 *  to answer the probe whose calls still run (3). */
export const actionsOk = (actions) => actions === 1 || actions === 3;

/**
 * Where the Home Assistant connect step goes on a setup/status read: nowhere until the device
 * reports onboarding done (it completes itself when the API attaches), then sign-in, or the
 * actions step first while the checkbox is off or not yet probed. The fork is an owner request
 * (September 26 2026): with the box ticked before sign-in, the VoiceTap window sign-in opens by
 * itself can speak its 4-digit code instead of falling back to the wake-word challenge.
 */
export function afterAdd(st) {
  if (st?.setup !== 0) return null;
  return actionsOk(st.actions) ? "done" : "haactions";
}
