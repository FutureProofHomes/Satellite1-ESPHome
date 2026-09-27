/**
 * The onboarding wizard's device layer: the setup endpoints on the session gate and the WiFi
 * provisioning trio on the handler.
 *
 * Bare fetch like auth.js and for the same reason - everything here happens before the app's data
 * layer exists (there is no session yet), and a failure is a state the wizard shows rather than an
 * error to toast. Every call is same-origin: the wizard runs on the device that serves it, whether
 * that is the setup AP at 192.168.4.1 or the home-network address the handoff lands on.
 *
 * Timeouts are shorter than auth.js's 15s. The wizard's polls are its heartbeat - the joining step
 * reads a dead poll as "the AP closed under me, show the handoff copy" - and a 15s wait would sit
 * on that verdict long past the moment it became true.
 */

import { BASE } from "./device.js";

const f = (path, opts) => fetch(BASE + path, { cache: "no-store", signal: AbortSignal.timeout(8000), ...opts });

/** Form-encoded like the gate's login POSTs: the device parses it for free. */
const form = (fields) => ({
  method: "POST",
  headers: { "Content-Type": "application/x-www-form-urlencoded" },
  body: new URLSearchParams(fields).toString(),
});

/**
 * Whether the setup wizard is owed, and where it left off: `{setup, mode, wizard, ha, name, fn}`.
 * Public on the device by design (the caller has no session yet). Null when the device does not
 * answer - the boot sequence then proceeds to the normal login flow, which is the right fallback
 * for every kind of unreachable.
 */
export async function setupStatus() {
  try {
    const j = await (await f("/api/sat1/setup/status")).json();
    return j && typeof j.setup === "number" ? j : null;
  } catch {
    return null;
  }
}

/**
 * The joining step's redirect probe: the same status endpoint, read cross-origin at the device's
 * home-network address while this page still lives on the setup AP's origin. An answer is proof
 * the phone is back on the home network and the device is reachable there - the caller then
 * navigates this very page to that origin, where the wizard resumes. The endpoint grants CORS to
 * LAN origins for exactly this read. Short timeout: this runs inside a poll, and a hung probe
 * would stall the verdict it exists to speed up.
 */
export async function probeSetup(origin) {
  try {
    const r = await fetch(`${origin}/api/sat1/setup/status`, {
      mode: "cors",
      credentials: "omit",
      cache: "no-store",
      signal: AbortSignal.timeout(3000),
    });
    if (!r.ok) return null;
    const j = await r.json();
    return j && typeof j.setup === "number" ? j : null;
  } catch {
    return null;
  }
}

/** The networks the device's last scan saw. `refresh` asks for a rescan, whose results arrive on a
 *  later poll - the radio hops channels to scan and answers this request before it does. */
export async function wifiScan(refresh) {
  try {
    const j = await (await f(`/api/sat1/wifi/scan${refresh ? "?refresh=1" : ""}`)).json();
    return Array.isArray(j?.aps) ? j.aps : null;
  } catch {
    return null;
  }
}

/** The station's truth: `{connected, has_sta, ap, ip, host, ssid}`. Null when the device does not
 *  answer - which during the joining step is itself information (the AP closed). */
export async function wifiStatus() {
  try {
    const j = await (await f("/api/sat1/wifi/status")).json();
    return j && typeof j.connected === "number" ? j : null;
  } catch {
    return null;
  }
}

/** Hands the device the chosen network. The device answers before it acts (the join runs on its
 *  main loop), so `ok` means "accepted", and the joining step's polls own what happens next. */
export async function wifiJoin(ssid, password) {
  const res = await f("/api/sat1/wifi/join", form(password ? { ssid, password } : { ssid }));
  const body = await res.json().catch(() => ({}));
  return { ok: res.ok && body.ok === 1, host: body.host || null, invalid: body.invalid === 1 };
}

/** Records the Connect Mode choice. `soon` is the honest refusal for the modes the wizard shows
 *  as coming - kept as a distinct shape so a stale bundle offering them gets copy, not confusion.
 *  There is no setup/password call anymore: onboarding's last step is the device noticing Home
 *  Assistant, which the connect step watches through setupStatus polls. */
export async function setupMode(mode) {
  const res = await f("/api/sat1/setup/mode", form({ mode }));
  const body = await res.json().catch(() => ({}));
  return { ok: res.ok && body.ok === 1, soon: body.soon === 1, done: body.done === 1 };
}

/** The launcher's first tap: asks the device to answer the OS connectivity probes "online" for the
 *  next minute and a half, which flips the captive sheet into its connected state - the one state
 *  from which the launcher's absolute Continue link opens in the real browser. Awaitable because
 *  the caller navigates immediately after (the navigation is what makes the sheet re-probe), and a
 *  navigation that outruns this request would tear the window open a beat too late. Never throws:
 *  a lost request only means the link opens in-sheet once more, where ?setup=go routes past the
 *  launcher. */
export function portalPass() {
  return f("/api/sat1/setup/browser", form({})).catch(() => {});
}
