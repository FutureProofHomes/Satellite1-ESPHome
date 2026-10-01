import { useEffect, useRef, useState } from 'react';
import { loginKey, loginPassword, maybeRedirectLocal, panelSlug, takePanelHandoff, takeUrlKey, whoami } from '../src/lib/auth.js';
import { BASE, proxied, setRemoteTarget } from '../src/lib/device.js';
import { setupStatus } from '../src/lib/setup.js';
import { LoginScreen } from './components/LoginScreen';
import { Satellite1Now } from './components/Satellite1Now';
import { SetupWizard } from './components/SetupWizard';

/**
 * The gatekeeper: nothing that polls mounts until the session question is answered, so a tab with
 * no session costs the device one probe rather than a storm of 401s. The order is src/shell.jsx's,
 * each step for its reason there: onboarding first (the wizard is the page while it is pending),
 * then the move to the stable .local origin, then a ?key= sign-in link, then the session probe.
 */
type Remote = { base: string; key: string } | null;

export function App() {
  const [phase, setPhase] = useState<'boot' | 'setup' | 'login' | 'in'>('boot');
  // The sign-in key, held only until the shell primes the device's other origin with it.
  const primeKey = useRef<string | null>(null);
  const setupInfo = useRef<any>(null);
  // Set when the wizard hands over: the sign-in screen opens a VoiceTap window by itself.
  const [autoPair, setAutoPair] = useState(false);
  // The peer this tab is remote-controlling, or null for the device serving it. A change remounts
  // the shell, because entity ids collide across devices and no hook state may survive it.
  const [remote, setRemote] = useState<Remote>(null);
  const localMac = useRef<string | null>(null);

  useEffect(() => {
    (async () => {
      const setup: any = await setupStatus();
      if (setup?.setup === 1) {
        setupInfo.current = setup;
        setPhase('setup');
        return;
      }
      if (await maybeRedirectLocal()) return;
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
        // Inside an ingress panel, a device switch leaves this device's password under its slug.
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
  if (phase === 'setup') return <SetupWizard status={setupInfo.current} onDone={() => {
    setAutoPair(true);
    setPhase('login');
  }} />;
  if (phase === 'login') return <LoginScreen autoPair={autoPair} onSignedIn={key => {
    setAutoPair(false);
    primeKey.current = key || null;
    setPhase('in');
  }} />;
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
