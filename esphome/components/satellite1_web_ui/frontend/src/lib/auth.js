/**
 * The auth layer: the challenge-response login, the device-presence pairing flow, the smart
 * redirect and the dual-origin cookie priming.
 *
 * Everything here talks to the session gate on the device (session_gate.cpp), which holds the other
 * half of each contract. Requests use bare fetch rather than the request queue in device.js: they
 * all happen before the app's data layer exists, they are few, and a 401 here is a state to show
 * rather than an error to toast.
 *
 * The hashing is hand-rolled because it has to be: crypto.subtle exists only on secure origins, and
 * this app's whole context is plain HTTP on a LAN. SHA-256 below is the standard FIPS 180-4
 * construction with the round constants derived at load (the fractional parts of the cube roots of
 * the first 64 primes - deriving them costs microseconds once and reads better than 64 magic
 * numbers). Verified against node:crypto over random inputs before shipping.
 */

import { apiUrl, updateRemoteKey } from "./device.js";

/* ------------------------------------------------------------------ */
/* SHA-256 and HMAC                                                    */
/* ------------------------------------------------------------------ */

const PRIMES = [];
for (let n = 2; PRIMES.length < 64; n++) if (PRIMES.every((p) => n % p)) PRIMES.push(n);
/** First 32 bits of the fractional part, which is how the spec defines both constant sets. */
const frac32 = (x) => Math.floor((x - Math.floor(x)) * 2 ** 32);
const K = PRIMES.map((p) => frac32(Math.cbrt(p)));
const H_INIT = PRIMES.slice(0, 8).map((p) => frac32(Math.sqrt(p)));

const rotr = (x, n) => (x >>> n) | (x << (32 - n));

/** SHA-256 of a Uint8Array, as a fresh Uint8Array(32). */
export function sha256(bytes) {
  // Padding: the message, 0x80, zeros to 56 mod 64, then the bit length as a 64-bit big-endian.
  const bitLen = bytes.length * 8;
  const padded = new Uint8Array(((bytes.length + 8) >> 6 << 6) + 64);
  padded.set(bytes);
  padded[bytes.length] = 0x80;
  const dv = new DataView(padded.buffer);
  dv.setUint32(padded.length - 8, Math.floor(bitLen / 2 ** 32));
  dv.setUint32(padded.length - 4, bitLen >>> 0);

  const h = H_INIT.slice();
  const w = new Array(64);
  for (let at = 0; at < padded.length; at += 64) {
    for (let i = 0; i < 16; i++) w[i] = dv.getUint32(at + i * 4);
    for (let i = 16; i < 64; i++) {
      const s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >>> 3);
      const s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >>> 10);
      w[i] = (w[i - 16] + s0 + w[i - 7] + s1) >>> 0;
    }
    let [a, b, c, d, e, f, g, hh] = h;
    for (let i = 0; i < 64; i++) {
      const S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const ch = (e & f) ^ (~e & g);
      const t1 = (hh + S1 + ch + K[i] + w[i]) >>> 0;
      const S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const maj = (a & b) ^ (a & c) ^ (b & c);
      const t2 = (S0 + maj) >>> 0;
      hh = g; g = f; f = e; e = (d + t1) >>> 0; d = c; c = b; b = a; a = (t1 + t2) >>> 0;
    }
    h[0] = (h[0] + a) >>> 0; h[1] = (h[1] + b) >>> 0; h[2] = (h[2] + c) >>> 0; h[3] = (h[3] + d) >>> 0;
    h[4] = (h[4] + e) >>> 0; h[5] = (h[5] + f) >>> 0; h[6] = (h[6] + g) >>> 0; h[7] = (h[7] + hh) >>> 0;
  }

  const out = new Uint8Array(32);
  const ov = new DataView(out.buffer);
  h.forEach((x, i) => ov.setUint32(i * 4, x));
  return out;
}

/** HMAC-SHA256, RFC 2104: block size 64, long keys hashed first. */
export function hmacSha256(key, msg) {
  if (key.length > 64) key = sha256(key);
  const inner = new Uint8Array(64 + msg.length);
  const outer = new Uint8Array(64 + 32);
  for (let i = 0; i < 64; i++) {
    inner[i] = 0x36 ^ (key[i] || 0);
    outer[i] = 0x5c ^ (key[i] || 0);
  }
  inner.set(msg, 64);
  outer.set(sha256(inner), 64);
  return sha256(outer);
}

const utf8 = (s) => new TextEncoder().encode(s);
export const toHex = (bytes) => [...bytes].map((b) => b.toString(16).padStart(2, "0")).join("");

/* ------------------------------------------------------------------ */
/* The gate's endpoints                                                */
/* ------------------------------------------------------------------ */

const f = (path, opts) => fetch(path, { signal: AbortSignal.timeout(15000), cache: "no-store", ...opts });

/** Form-encoded on purpose: the device parses it for free (<=1KB rides web_server_idf's own form
 *  path), and cross-origin it is a CORS "simple request", so the priming call needs no preflight. */
const form = (fields) => ({
  method: "POST",
  headers: { "Content-Type": "application/x-www-form-urlencoded" },
  body: new URLSearchParams(fields).toString(),
});

/**
 * The challenge-response password login. The password itself never crosses the wire - the property
 * digest auth had, kept: the device issues a single-use nonce and this answers
 * HMAC-SHA256(SHA-256(password), nonce). A success sets the 90-day session cookie and hands back
 * the sign-in key for the dual-origin priming below.
 */
export async function loginPassword(password) {
  const nres = await f("/api/sat1/login/nonce");
  const nonce = (await nres.json()).n;
  if (!nonce) return { ok: false };
  const answer = toHex(hmacSha256(sha256(utf8(password)), utf8(nonce)));
  const res = await f("/api/sat1/login", form({ n: nonce, r: answer }));
  const body = await res.json().catch(() => ({}));
  return { ok: res.ok && body.ok === 1, key: body.key, locked: body.locked === 1, retry: body.retry || 0 };
}

/** The bearer path: a sign-in link's key, or a key handed back by a poll or an earlier login. */
export async function loginKey(key) {
  const res = await f("/api/sat1/login", form({ key }));
  const body = await res.json().catch(() => ({}));
  return { ok: res.ok && body.ok === 1, key: body.key || key, locked: body.locked === 1, retry: body.retry || 0 };
}

/** Opens the pairing window. `pending` is someone else's window - the confused-deputy copy's cue.
 *  `hab` says voice-via-Home-Assistant was unavailable (the actions checkbox), `seq` is the offline
 *  challenge as symbol digits ("012") for the on-screen chips - both only ever served to this
 *  window's owner. */
export async function pairStart() {
  const res = await f("/api/sat1/login/start", form({}));
  const body = await res.json().catch(() => ({}));
  return {
    ok: res.ok && body.ok === 1,
    mode: body.mode,
    left: body.left,
    hw: body.hw,
    hab: body.hab === 1,
    seq: body.seq || null,
    pending: body.pending === 1,
    retry: body.retry || 0,
  };
}

/** One poll: {s: "pending"|"ok"|"expired"|"denied"|"busy"|"none", mode, left, key, hw, hab, seq, p}.
 *  `p` is the matched-prefix count for a seq window - how many challenge words the device has heard
 *  back correctly so far - which drives the chips' bold-to-plain progress. */
export async function pairPoll() {
  const res = await f("/api/sat1/login/poll");
  return await res.json().catch(() => ({ s: "none" }));
}

/** Closes this browser's own pending window (proven by the pair cookie), so the LEDs stop and an
 *  immediate retry is not refused as busy. Fire-and-forget; an expired window ignores it. */
export function pairCancel() {
  f("/api/sat1/login/cancel", form({})).catch(() => {});
}

/** This browser only: expires the session cookie here and nowhere else. The caller reloads, and
 *  the boot probe lands on the login screen. */
export async function logout() {
  try {
    await f("/api/sat1/logout", form({}));
  } catch {
    /* The reload will show whatever the truth is. */
  }
}

export async function logoutAll() {
  // Through apiUrl, unlike everything else in this file: the rest of the gate's endpoints are about
  // *this browser's* session on *this origin* (login, logout, pairing), but "sign out everywhere"
  // belongs to whichever device the page is showing - while remote-controlling a peer, that is the
  // peer. The regenerate takes our own key with it, so the fresh one in the body is adopted on the
  // spot; the next poll then carries it and the Launch card re-renders from there.
  const res = await f(apiUrl("/api/sat1/logout_all"), form({}));
  const body = await res.json().catch(() => ({}));
  if (res.ok && body.key) updateRemoteKey(body.key);
  return { ok: res.ok && body.ok === 1, key: body.key };
}

/** The device's mDNS hostname, from the one deliberately public fact endpoint. */
export async function whoami() {
  try {
    return (await (await f("/api/sat1/whoami")).json()).name || null;
  } catch {
    return null;
  }
}

/* ------------------------------------------------------------------ */
/* Origins: the smart redirect and the cookie priming                  */
/* ------------------------------------------------------------------ */

const isIpHost = (h) => /^\d+\.\d+\.\d+\.\d+$/.test(h) || h.includes(":");
const portSuffix = () => (location.port ? `:${location.port}` : "");

/**
 * From an IP-literal origin to the device's .local name, before any login - cookies are per-origin,
 * and a session on the mDNS origin survives DHCP handing the device a new address, where an
 * IP-origin session dies with the old one. Probe-first because not every client resolves mDNS
 * (Chrome on Android famously does not): the redirect only happens once the .local origin has
 * actually answered, so those clients silently stay on the IP and everything still works.
 *
 * A failed probe is remembered for a day (localStorage, per-origin), and while the memory is fresh
 * the whole attempt - the whoami round trip included, since the probe is the name's only consumer -
 * is skipped. An mDNS-less browser pays the timeout once a day instead of on every visit, and the
 * per-origin scoping means one device's flag cannot bleed onto another device's page, while a DHCP
 * address change (a new origin) resets it naturally. The flag is cleared on any successful probe.
 *
 * The probe timeout stays at 2500ms on purpose: with the cache it is paid rarely, and a shorter
 * timeout would raise the stakes of a false negative - one slow mDNS answer on a busy network would
 * pin an mDNS-capable browser to the IP origin for a day, the exact outcome this redirect prevents.
 *
 * Resolves true when the page is navigating away, which is the caller's signal to render nothing.
 */
const MDNS_FAIL_KEY = "sat1.mdns_fail";
const MDNS_FAIL_TTL = 24 * 60 * 60 * 1000;

/**
 * Whether this browser recently proved it cannot resolve the device's .local name - the redirect
 * probe's own memory, read by the Launch card so the copyable sign-in link can switch to the IP
 * (a .local link pasted on this network would be a dead end). False on a .local origin (being here
 * is proof mDNS works) and false when nothing is known, so the .local form stays the default.
 */
export function mdnsLooksBroken() {
  if (!isIpHost(location.hostname)) return false;
  try {
    return Date.now() - (+localStorage.getItem(MDNS_FAIL_KEY) || 0) < MDNS_FAIL_TTL;
  } catch {
    return false;
  }
}

export async function maybeRedirectLocal() {
  if (!isIpHost(location.hostname)) return false;
  // Guarded like every localStorage read here: it throws with site data blocked, and re-running
  // the probe is the survivable cost of not knowing.
  try {
    if (Date.now() - (+localStorage.getItem(MDNS_FAIL_KEY) || 0) < MDNS_FAIL_TTL) return false;
  } catch {
    /* fall through to the probe */
  }
  const name = await whoami();
  if (!name) return false;
  const target = `http://${name}.local${portSuffix()}`;
  try {
    // no-cors: an opaque success is all this needs - "did the name resolve and answer at all".
    await fetch(`${target}/manifest.webmanifest`, {
      mode: "no-cors",
      cache: "no-store",
      signal: AbortSignal.timeout(2500),
    });
  } catch {
    try {
      localStorage.setItem(MDNS_FAIL_KEY, String(Date.now()));
    } catch {
      /* Not remembering the failure is survivable; the next visit probes again. */
    }
    return false;
  }
  try {
    localStorage.removeItem(MDNS_FAIL_KEY);
  } catch {
    /* ignore */
  }
  // Search and hash ride along, so a ?key= sign-in link lands its cookie on the stable origin.
  location.replace(`${target}${location.pathname}${location.search}${location.hash}`);
  return true;
}

/**
 * Pulls ?key= out of the URL and scrubs it from the address bar before anything can screenshot or
 * bookmark it. replaceState rewrites the history entry itself, so the tokenized form is not in the
 * back stack either.
 */
export function takeUrlKey() {
  const url = new URL(location.href);
  const key = url.searchParams.get("key");
  if (!key) return null;
  url.searchParams.delete("key");
  history.replaceState(null, "", url.pathname + (url.searchParams.size ? `?${url.searchParams}` : "") + url.hash);
  return key;
}

/**
 * One CORS login against the device's other origin, so a sign-in on the IP also signs in the .local
 * name and vice versa. Fire-and-forget: the other origin answering with Set-Cookie is the whole
 * point, and a failure (mDNS not resolving here, device renamed) costs one extra sign-in there
 * someday rather than anything now.
 */
export function primeOtherOrigin(key, name, ip) {
  let other = null;
  if (isIpHost(location.hostname)) {
    if (name) other = `http://${name}.local${portSuffix()}`;
  } else if (ip) {
    other = `http://${ip}${portSuffix()}`;
  }
  if (!key || !other || other === location.origin) return;
  fetch(`${other}/api/sat1/login`, {
    ...form({ key }),
    mode: "cors",
    credentials: "include",
    signal: AbortSignal.timeout(4000),
  }).catch(() => {});
}

/** The copyable sign-in link the Launch section renders: on the .local origin, so a pasted
 *  dashboard button - long-lived by nature - starts life on the stable hostname. */
export function signInLink(name, key) {
  const base = name ? `http://${name}.local${portSuffix()}` : location.origin;
  return `${base}/?key=${key}`;
}

/** The QR's variant of the same link: built on the device's current IP, because the QR is scanned
 *  live off the screen - the address is fresh by construction - and a .local QR is a dead end for
 *  phones that cannot resolve mDNS (Android Chrome, the HA app's webview). Phones that can resolve
 *  it still end up on .local: maybeRedirectLocal carries ?key= along when it upgrades the origin.
 *  Null when the IP is not known yet (the first state poll); the caller falls back to the .local
 *  link rather than rendering a broken QR. */
export function qrSignInLink(ip, key) {
  return ip ? `http://${ip}${portSuffix()}/?key=${key}` : null;
}

/**
 * The device switcher's silent cross-sign-in: the same challenge-response as loginPassword, run
 * against a peer device's origin with the peer's password (which rides the HA payload - it is the
 * Web UI Password sensor every device already publishes). Returns `{ key, name }` - the peer's
 * sign-in key plus its mDNS hostname (newer firmware puts it in the login body so the switcher can
 * jump straight to the peer's .local origin; older firmware omits it and `name` is null) - or null
 * when anything along the way declines - older peer firmware without the endpoints, a stale
 * password, mDNS not resolving - in which case the switcher just navigates normally and the peer's
 * login page takes it from there.
 *
 * The caller navigates to `?key=` with the result rather than trusting the Set-Cookie on this
 * cross-origin response: third-party cookie blocking (Safari always, Chrome increasingly) may drop
 * that cookie silently, but a key in the body is readable under plain CORS, and the ?key= landing
 * sets the cookie first-party where nothing blocks it.
 */
/**
 * Whether a peer can be remote-controlled rather than navigated to: one gated read with the key
 * peerLogin just returned, judged on the `apiv` contract number the state payload carries.
 *
 * This one probe answers everything the single-origin switch needs to know before it commits:
 * that the peer's firmware accepts ?key= on gated routes at all (older firmware answers 401 and
 * the catch below turns it into null), that the key is good, and that the peer speaks the same
 * /api/sat1/* shapes this app was built against. Anything short of a clean yes means the switcher
 * navigates the old way and the peer's own self-consistent UI takes over - an old device must
 * never be driven by an app that misunderstands it.
 *
 * Returns the peer's state payload (so the caller starts with fresh facts) or null.
 */
export async function probePeer(origin, key) {
  try {
    const r = await fetch(`${origin}/api/sat1/state?key=${key}`, {
      mode: "cors",
      credentials: "omit",
      signal: AbortSignal.timeout(4000),
      cache: "no-store",
    });
    if (!r.ok) return null;
    const state = await r.json();
    return state && state.apiv === 1 ? state : null;
  } catch {
    return null;
  }
}

export async function peerLogin(origin, password) {
  try {
    const opts = { mode: "cors", credentials: "omit", signal: AbortSignal.timeout(4000), cache: "no-store" };
    const nres = await fetch(`${origin}/api/sat1/login/nonce`, opts);
    const nonce = (await nres.json()).n;
    if (!nonce) return null;
    const answer = toHex(hmacSha256(sha256(utf8(password)), utf8(nonce)));
    const res = await fetch(`${origin}/api/sat1/login`, { ...form({ n: nonce, r: answer }), ...opts });
    const body = await res.json().catch(() => ({}));
    return res.ok && body.ok === 1 && body.key ? { key: body.key, name: body.name || null } : null;
  } catch {
    return null;
  }
}
