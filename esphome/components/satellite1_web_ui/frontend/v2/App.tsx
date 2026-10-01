import { useEffect, useRef, useState } from 'react';
import { loginKey, loginPassword, maybeRedirectLocal, panelSlug, takePanelHandoff, takeUrlKey, whoami } from '../src/lib/auth.js';
import { BASE, proxied, setRemoteTarget } from '../src/lib/device.js';
import { setupStatus } from '../src/lib/setup.js';
import { LoginScreen } from './components/LoginScreen';
import { Satellite1Now } from './components/Satellite1Now';
import { SetupWizard } from './components/SetupWizard';

/**
 * The gatekeeper: nothing that polls mounts until the session question is answered, so a tab with
 * no session costs the device one probe rather than a storm of 401s.
 *
 * The boot sequence, in order and each for a reason. Onboarding first: the wizard is the page while
 * it is pending, and checking it before the smart redirect spares the setup AP 2.5 seconds probing a
 * .local name inside a captive sheet that may not resolve it. Then the smart redirect, so everything
 * after - the cookie a sign-in sets most of all - lands on the stable .local origin rather than an
 * IP that DHCP can reassign. Then a ?key= from a sign-in link is redeemed (takeUrlKey has already
 * scrubbed it from the URL). Then the session probe: one tiny gated GET whose 401 is the difference
 * between the sign-in screen and the app.
 */
type Remote = { base: string; key: string } | null;

export function App() {
  const [phase, setPhase] = useState<'boot' | 'setup' | 'login' | 'in'>('boot');
  // The sign-in key, held only until the shell primes the device's other origin with it.
  const primeKey = useRef<string | null>(null);
  // The onboarding facts from /api/sat1/setup/status, held for the wizard.
  const setupInfo = useRef<any>(null);
  // Set when the wizard hands over: the sign-in screen opens a VoiceTap window by itself. State
  // rather than a ref so the ordinary sign-in paths (sign-out, a lost session) render without it;
  // cleared the moment a sign-in succeeds.
  const [autoPair, setAutoPair] = useState(false);
  // The peer this tab is remote-controlling, or null for the device serving it: single-origin
  // device switching, because the iOS home-screen app must never navigate cross-origin or Safari
  // wraps the peer in its in-app sheet. A change remounts the shell, because entity ids collide
  // across devices and no hook state may survive it. In memory only, on purpose: a reload lands
  // back on the local device, the one whose session cookie is real.
  const [remote, setRemote] = useState<Remote>(null);
  // The serving device's MAC, captured on the way out to a peer, so the switcher can tell "back to
  // the device serving this page" (a plain state reset) apart from "another peer" (a cross-sign-in).
  const localMac = useRef<string | null>(null);

  // index.html's static splash, removed rather than hidden: it exists only for boot, which happens
  // once per load. While the smart redirect navigates away the phase stays "boot" and the splash
  // stays up, the right cover for a page about to be replaced.
  useEffect(() => {
    if (phase !== 'boot') document.getElementById('splash')?.remove();
  }, [phase]);

  useEffect(() => {
    (async () => {
      // One cheap same-origin read: onboarding pending means the wizard is the page, whatever
      // session or key the URL carries. Unreachable or onboarded falls through to the normal boot.
      const setup: any = await setupStatus();
      if (setup?.setup === 1) {
        setupInfo.current = setup;
        setPhase('setup');
        return;
      }
      if (await maybeRedirectLocal()) return; // The page is navigating away; render nothing.
      const urlKey = takeUrlKey();
      if (urlKey) {
        try {
          const r: any = await loginKey(urlKey);
          if (r.ok) {
            primeKey.current = r.key;
            setPhase('in');
            return;
          }
        } catch {
          /* A dead link falls through to the probe; a live cookie may still exist. */
        }
      }
      try {
        const r = await fetch(`${BASE}/api/sat1/sel`, { cache: 'no-store', signal: AbortSignal.timeout(15000) });
        if (r.status !== 401) {
          setPhase('in');
          return;
        }
        // No session yet. Inside an ingress panel this may be a device switch: the switcher left
        // this device's password in shared HA-origin localStorage under its panel slug before
        // navigating here (see putPanelHandoff in src/lib/auth.js). Identify ourselves by the one
        // public endpoint, claim the handoff, and sign in with it instead of asking for a password.
        if (proxied) {
          const who: any = await whoami();
          const pw = who?.name ? takePanelHandoff(panelSlug(who.name)) : null;
          if (pw) {
            const lr: any = await loginPassword(pw).catch(() => null);
            if (lr?.ok) {
              primeKey.current = lr.key;
              setPhase('in');
              return;
            }
          }
        }
        setPhase('login');
      } catch {
        setPhase('login');
      }
    })();
  }, []);

  if (phase === 'boot') return null;
  // Onboarding just completed (the device noticed Home Assistant), so the sign-in screen opens a
  // VoiceTap pairing window by itself: the ring breathes, and one press signs the customer in.
  if (phase === 'setup') return <SetupWizard status={setupInfo.current} onDone={() => {
    setAutoPair(true);
    setPhase('login');
  }} />;
  if (phase === 'login') return <LoginScreen autoPair={autoPair} onSignedIn={key => {
    setAutoPair(false);
    primeKey.current = key || null;
    setPhase('in');
  }} />;
  // Keyed on the target so two devices' state can never blend: a new target is a new app.
  return <Satellite1Now key={remote ? remote.base : 'local'} primeKey={primeKey} remote={remote} localMac={localMac.current} onRemote={(target, mac) => {
    if (mac) localMac.current = mac;
    setRemoteTarget(target);
    setRemote(target);
  }} onLocal={() => {
    setRemoteTarget(null);
    setRemote(null);
  }}
  // A 401 while remote means the peer's sessions were regenerated; the local cookie is a separate
  // fact, so the honest response is home, not the sign-in screen.
  onAuthLost={remote ? () => {
    setRemoteTarget(null);
    setRemote(null);
  } : () => setPhase('login')} />;
}
