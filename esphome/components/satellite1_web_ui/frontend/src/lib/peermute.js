/**
 * Tune-time peer muting (September 2026, owner request; own plan file). While a Wake Word Tuner
 * session runs, every OTHER Satellite1 in the same area gets its software mute held on - the
 * customer is saying the wake word over and over, and the neighbours must not answer - and
 * released when the session ends.
 *
 * The shape mirrors the tune session's own keepalive design, deliberately: this module only asks
 * and reminds. Each peer owns its hold - it remembers its own prior mute state and restores it on
 * release OR when its 60s TTL lapses, so a vanished browser leaves no satellite deaf, and a hand
 * on the peer itself (unmuting at the device or through Home Assistant) cancels the hold for good.
 * See satellite1_web_ui's loop() for that whole lifecycle; POST /api/sat1/mutehold is the ask.
 *
 * Peers come from the roster the HA payload already carries (`dev` rows: name d[1], area d[2],
 * mac d[3], origin URL d[5], password d[7]), the same rows the device switcher jumps through, and
 * the writes ride the same machinery: peerLogin's silent cross-sign-in, then the session gate's
 * ?key= bearer. A peer HA holds in no area is absent from `dev` entirely (the roster is built by
 * walking areas) and cannot be muted - the known blind spot, accepted in the plan.
 *
 * Failures are explicit by owner decision: the caller gets every same-area peer's name with its
 * verdict, and the tuner says who could not be muted (old firmware 404s the route, a dead peer
 * times out) rather than muting what it can in silence.
 */

import { peerLogin } from "./auth.js";
import { peerOrigin } from "./device.js";

const OPTS = { mode: "cors", credentials: "omit", cache: "no-store" };

/** One held peer: enough to keep alive and release. */
// { name, origin, key }

/** The same-area rows worth attempting: online Satellite1 models only, minus the tuning device
 *  itself. Both tests are the switcher's own (shell.jsx) - the roster is one picture and the two
 *  features must read it the same way:
 *  - Model: `dev` also carries every other FutureProofHomes device Home Assistant knows - a
 *    Nexus, its Conversation and AI Task devices - and none of those has microphones to mute;
 *    attempting them put "Couldn't mute Nexus Conversation" on the status line (owner's report,
 *    September 23 2026, first hardware test).
 *  - Availability (d[6], the switcher's isUp): an unplugged device cannot answer the wake word,
 *    so listing it as "may answer while you tune" is a false alarm - three powered-off units did
 *    exactly that (owner's report, same night). Skipped silently, like the switcher folds them
 *    behind its Offline disclosure. The trade, accepted by that decision: a peer whose HA
 *    availability lags a reboot by a few seconds is neither muted nor listed for that moment. */
function sameAreaPeers(ha, selfMac) {
  const rows = ha?.d?.dev;
  if (!Array.isArray(rows)) return null; // no roster at all (HA down / offline mode)
  const mac = (selfMac || "").toLowerCase();
  const mine = rows.find((d) => (d?.[3] || "").toLowerCase() === mac);
  const area = String(mine?.[2] || ha?.d?.area || "").toLowerCase();
  if (!area) return [];
  return rows.filter(
    (d) =>
      /satellite1/i.test(d?.[0] || "") &&
      (d?.[6] === 1 || d?.[6] === "1") &&
      (d?.[3] || "").toLowerCase() !== mac &&
      String(d?.[2] || "").toLowerCase() === area
  );
}

async function postHold(origin, key, on) {
  const r = await fetch(`${origin}/api/sat1/mutehold?on=${on ? 1 : 0}&key=${key}`, {
    ...OPTS,
    method: "POST",
    signal: AbortSignal.timeout(4000),
  });
  return r.ok;
}

/**
 * Asks every same-area peer to hold its mute. Resolves to
 *   { held: [{name, origin, key}], failed: [names], unknown }
 * where `unknown` means the roster itself was absent (HA unreachable) and nothing could even be
 * attempted. Peers are attempted in parallel; a peer that declines anywhere along the way - no
 * URL on an old roster, cross-sign-in refused, the route 404ing on old firmware, a timeout -
 * lands on `failed` under its roster name.
 */
export async function holdPeerMutes(ha, selfMac) {
  const peers = sameAreaPeers(ha, selfMac);
  if (peers === null) return { held: [], failed: [], unknown: true };
  const held = [];
  const failed = [];
  await Promise.all(
    peers.map(async (d) => {
      const name = String(d?.[1] || d?.[3] || "?");
      // peerOrigin, not d[5] raw: a peer added to Home Assistant by its .local hostname carries a
      // .local configuration_url, which dies on an mDNS-broken network - the roster's live IP
      // (d[11]) substitutes when the row has one. Same helper the switcher's jump uses.
      const origin = peerOrigin(d);
      const pw = d?.[7];
      try {
        if (!origin || !pw) throw new Error("no address");
        const peer = await peerLogin(origin, pw);
        if (!peer) throw new Error("sign-in refused");
        if (!(await postHold(origin, peer.key, true))) throw new Error("hold refused");
        held.push({ name, origin, key: peer.key });
      } catch {
        failed.push(name);
      }
    })
  );
  return { held, failed, unknown: false };
}

/** The keepalive, on the tuner's own 20s cadence: reminds every held peer the session is still
 *  live. Fire-and-forget - a missed reminder is what the peer's TTL exists for. */
export function keepPeerMutes(held) {
  for (const p of held || []) postHold(p.origin, p.key, true).catch(() => {});
}

/** Releases every held peer; each restores its own prior mute state. Best effort - the TTL
 *  backstops whatever this misses. */
export function releasePeerMutes(held) {
  for (const p of held || []) postHold(p.origin, p.key, false).catch(() => {});
}
