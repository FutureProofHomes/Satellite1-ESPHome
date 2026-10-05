/**
 * Logs from several Satellite1s on one timeline (Settings > Developer). A snapshot, not a tail:
 * Collect reads each chosen device's log history once (GET /api/sat1/log) and merges the lines by
 * wall-clock time. Nothing polls.
 *
 * Each device stamps its lines with its own uptime, so the merge needs each device's clock pinned
 * to the browser's. The history's header carries `now`, the device's uptime when it began
 * answering; the browser timed the request from send to headers. The device wrote `now` somewhere
 * in that window, so the midpoint is the estimate and half the round trip is the error bar. The
 * merged file's header prints every device's round trip, the honest measure of how closely the
 * lines from two devices can be compared.
 */

import { peerLogin } from "./auth.js";
import { apiUrl, peerOrigin } from "./device.js";
import { LOG_LEVEL, msSince, parseLogHistory } from "./loghistory.js";

const ANSI = /\u001b\[[0-9;]*m/g;
const OPTS = { mode: "cors", credentials: "omit", cache: "no-store" };

/**
 * The devices Collect can read: this one first, then every other Satellite1 in Home Assistant's
 * roster (dev rows: model d[0], name d[1], mac d[3], origin d[5]/d[11], available d[6],
 * password d[7]) - the switcher's and peer muting's reading of the same rows.
 */
export function logDevices(ha, self) {
  const mac = String(self?.mac || "").toLowerCase();
  const out = [{ id: mac || "self", name: self?.name || "This device", self: true, up: true }];
  for (const d of ha?.d?.dev || []) {
    if (!/satellite1/i.test(d?.[0] || "")) continue;
    const m = String(d?.[3] || "").toLowerCase();
    if (!m || m === mac) continue;
    out.push({
      id: m,
      name: String(d?.[1] || m),
      self: false,
      up: d?.[6] === 1 || d?.[6] === "1",
      origin: peerOrigin(d),
      password: d?.[7] || "",
    });
  }
  return out;
}

/**
 * A parsed history pinned to wall-clock time. `sentAt` and `headAt` are Date.now() before the
 * request and when its headers arrived. Lines inherit the level of the header before them, as on
 * the device's own log card.
 */
export function pinLines(hist, sentAt, headAt) {
  const rtt = Math.max(0, headAt - sentAt);
  const nowWall = sentAt + rtt / 2;
  let lvl = "?";
  const lines = hist.lines.map((l) => {
    const text = l.text.replace(ANSI, "");
    const m = LOG_LEVEL.exec(text);
    lvl = m ? m[1] : lvl;
    return { wall: nowWall + msSince(l.ms, hist.now), ms: l.ms, lvl, text };
  });
  return { rtt, lines };
}

/** Every device's pinned lines in one wall-clock order; a tie keeps device order, then line order. */
export function mergeLines(results) {
  const all = [];
  results.forEach((r, d) => {
    if (!r.lines) return;
    r.lines.forEach((l, i) => all.push({ ...l, device: r.name, d, i }));
  });
  all.sort((a, b) => a.wall - b.wall || a.d - b.d || a.i - b.i);
  return all;
}

const pad = (n, w = 2) => String(n).padStart(w, "0");
export function wallStamp(ms) {
  const t = new Date(ms);
  return `${t.getFullYear()}-${pad(t.getMonth() + 1)}-${pad(t.getDate())} ${pad(t.getHours())}:${pad(t.getMinutes())}:${pad(t.getSeconds())}.${pad(t.getMilliseconds(), 3)}`;
}

/** The download: a header naming each device and its clock's error bar, then the merged lines. */
export function formatMerged(results, merged, collectedAt) {
  const head = [`# Satellite1 logs, collected ${wallStamp(collectedAt)}`];
  for (const r of results) {
    if (r.lines) {
      head.push(`# ${r.name}: ${r.lines.length} lines, firmware ${r.fw || "?"}, boot ${r.boot}, round trip ${Math.round(r.rtt)} ms (timestamps +/- ${Math.round(r.rtt / 2)} ms)`);
    } else {
      head.push(`# ${r.name}: ${r.error}`);
    }
  }
  const width = Math.max(0, ...results.map((r) => r.name.length));
  const body = merged.map((l) => `${wallStamp(l.wall)} ${l.device.padEnd(width)} ${l.text.replace(/\n/g, "\n    ")}`);
  return `${head.join("\n")}\n\n${body.join("\n")}\n`;
}

async function readHistory(url, opts) {
  const sentAt = Date.now();
  const r = await fetch(url, { ...opts, signal: AbortSignal.timeout(20000) });
  const headAt = Date.now();
  if (r.status === 404) return { error: "no log history (older firmware)" };
  if (!r.ok) return { error: `HTTP ${r.status}` };
  const hist = parseLogHistory(await r.text());
  if (!hist) return { error: "no log history (older firmware)" };
  return { hist, sentAt, headAt };
}

async function readFw(url, opts) {
  try {
    const r = await fetch(url, { ...opts, signal: AbortSignal.timeout(5000) });
    return r.ok ? (await r.json()).fw || null : null;
  } catch {
    return null;
  }
}

/** Reads every chosen device in parallel. Each result is {name, lines, rtt, boot, fw} or {name, error}. */
export async function collectLogs(devices, { selfFw } = {}) {
  return Promise.all(
    devices.map(async (d) => {
      try {
        let got;
        let fw = null;
        if (d.self) {
          got = await readHistory(apiUrl("/api/sat1/log"), { cache: "no-store" });
          fw = selfFw || null;
        } else {
          if (!d.up) return { name: d.name, error: "offline" };
          if (!d.origin || !d.password) return { name: d.name, error: "can't sign in (no password in Home Assistant)" };
          const peer = await peerLogin(d.origin, d.password);
          if (!peer) return { name: d.name, error: "can't sign in" };
          [got, fw] = await Promise.all([readHistory(`${d.origin}/api/sat1/log?key=${peer.key}`, OPTS), readFw(`${d.origin}/api/sat1/state?key=${peer.key}`, OPTS)]);
        }
        if (got.error) return { name: d.name, error: got.error };
        const { rtt, lines } = pinLines(got.hist, got.sentAt, got.headAt);
        return { name: d.name, lines, rtt, boot: got.hist.boot, fw };
      } catch (e) {
        return { name: d.name, error: e?.name === "TimeoutError" ? "timed out" : "unreachable" };
      }
    })
  );
}
