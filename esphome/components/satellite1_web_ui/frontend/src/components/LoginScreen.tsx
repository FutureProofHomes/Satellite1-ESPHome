import { useEffect, useRef, useState } from 'react';
import { loginPassword, pairCancel, pairPoll, pairStart, whoami } from '../lib/auth.js';
import { TEXT } from '../copy.js';
import { Icon, Logo, useTheme } from './bits';

/**
 * The sign-in screen: the first thing a browser without a session sees, and for most people the last
 * time they see it for 90 days.
 *
 * Two ways in, in the order we would rather they used. VoiceTap opens the pairing window - the ring
 * breathes, the device may speak, and a press of the action button or the right spoken answer is the
 * approval; no secret is ever typed. The password underneath is the universal fallback,
 * challenge-response so the password never crosses the wire (docs/web-ui.md, "Ways to sign in").
 *
 * The instructions follow what the poll reports, because the device picks the mode when the window
 * opens: the button alone while the mics are muted, a spoken code when Home Assistant can announce
 * one, the wake-word challenge when it cannot.
 */
type Pair = {
  s: 'pending' | 'busy' | 'expired' | 'denied' | 'error';
  mode?: string;
  left?: number;
  hw?: boolean;
  hab?: boolean;
  seq?: string | null;
  p?: number;
} | null;

const clock = (s: number) => `${Math.floor(s / 60)}:${String(Math.max(0, s) % 60).padStart(2, '0')}`;

/** hw rides the poll: a button-only window forced by the hardware mute slider gets the copy that says
 *  how to get voice sign-in back. hab rides it too: a voice window that fell back from the spoken
 *  code because Home Assistant may not perform actions gets the copy that says why, so the wake-word
 *  challenge reads as a fallback rather than a malfunction. */
const modeText = (mode?: string, hw?: boolean, hab?: boolean) =>
  mode === 'code' ? TEXT.login_mode_code
  : mode === 'seq' ? (hab ? TEXT.login_mode_seq_hab : TEXT.login_mode_seq)
  : hab ? TEXT.login_mode_button_hab
  : hw ? TEXT.login_mode_button_hw
  : TEXT.login_mode_button;

const ENDED: Record<string, string> = {
  busy: TEXT.login_busy,
  expired: TEXT.login_expired,
  denied: TEXT.login_denied,
  error: TEXT.login_start_failed
};

export function LoginScreen({
  onSignedIn,
  autoPair
}: {
  onSignedIn: (key: string | null) => void;
  autoPair?: boolean;
}) {
  const [theme, toggleTheme] = useTheme();
  const [pair, setPair] = useState<Pair>(null);
  const [password, setPassword] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  // Read on submit rather than trusting the state: a password manager's autofill followed by an
  // immediate Enter can outrun the re-render, and a submit that silently does nothing is
  // indistinguishable from a broken page.
  const pwRef = useRef<HTMLInputElement>(null);
  const pollTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
  const live = useRef(true);
  // Which device this is, so a row of identical sign-in pages can be told apart: the friendly name,
  // else the mDNS name, else the host the browser is already on. Seeded with the host so the line is
  // never empty and never jumps from blank to filled; whoami upgrades it in place.
  const [dev, setDev] = useState(location.hostname);
  useEffect(() => {
    whoami().then((who: any) => {
      if (!who) return;
      // `fn` is absent only on an older firmware body, which cannot happen inside one image but
      // costs nothing to survive.
      const label = who.fn || (who.name ? `${who.name}.local` : null);
      if (label) setDev(label);
      // The friendly name beats the hostname in a row of tabs, and the app proper uses the same
      // name after sign-in.
      document.title = who.fn || who.name;
    });
    return () => {
      live.current = false;
      clearTimeout(pollTimer.current);
    };
  }, []);

  // Chained timeouts rather than an interval, so a slow answer stretches the gap instead of
  // stacking requests.
  const startPair = async () => {
    setErr(null);
    let opened: any;
    try {
      opened = await pairStart();
    } catch {
      setPair({ s: 'error' });
      return;
    }
    if (opened.pending) {
      // Someone else's window: a person at the device "helping" would be approving a stranger.
      setPair({ s: 'busy' });
      return;
    }
    if (!opened.ok) {
      setPair({ s: 'error' });
      return;
    }
    setPair({ s: 'pending', mode: opened.mode, left: opened.left, hw: opened.hw, hab: opened.hab, seq: opened.seq, p: 0 });
    const tick = async () => {
      let p: any;
      try {
        p = await pairPoll();
      } catch {
        p = null;
      }
      if (!live.current) return;
      if (p && p.s === 'ok') {
        onSignedIn(p.key);
        return;
      }
      if (p && p.s === 'pending') {
        setPair({ s: 'pending', mode: p.mode, left: p.left, hw: p.hw, hab: p.hab === 1, seq: p.seq || null, p: p.p || 0 });
        // Faster while a challenge listens: the chips dimming is the "it heard me" feedback, and
        // 1.2s behind the utterance reads as "it missed me". Nothing else on screen moves per word.
        pollTimer.current = setTimeout(tick, p.mode === 'seq' ? 500 : 1200);
        return;
      }
      if (p && (p.s === 'expired' || p.s === 'denied' || p.s === 'busy')) {
        setPair({ s: p.s });
        return;
      }
      // A dropped poll: the device may be mid-announcement, and a window that has really gone
      // resolves as expired on the next answer.
      pollTimer.current = setTimeout(tick, 1500);
    };
    pollTimer.current = setTimeout(tick, 1200);
  };
  const cancelPair = () => {
    clearTimeout(pollTimer.current);
    setPair(null);
    // Tell the device too: the window is this browser's own (the pair cookie proves it), so cancel
    // stops the breathing ring now and lets the next attempt start straight away, instead of
    // leaving an abandoned window that refuses the next click as busy for up to a minute.
    pairCancel();
  };

  // The onboarding wizard hands over to this screen with the pairing window already opening (owner
  // request, September 26 2026): the customer returns from tapping Add in Home Assistant to a
  // breathing ring. Mount-once on purpose: a cancelled window falls back to the ordinary screen.
  useEffect(() => {
    if (autoPair) startPair();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // The submit button is disabled only while a sign-in is in flight, never because the field looks
  // empty: browsers suppress Enter-to-submit when the default button is disabled, and an autofilled
  // value can be real before the re-render that would enable it.
  const submit = async (e: Event) => {
    e.preventDefault();
    const value = pwRef.current?.value ?? password;
    if (!value) {
      setErr('Enter a password to continue.');
      return;
    }
    if (busy) return;
    setBusy(true);
    setErr(null);
    try {
      const r: any = await loginPassword(value);
      if (r.ok) {
        onSignedIn(r.key);
        return;
      }
      setErr(r.locked ? TEXT.login_locked.replace('%s', String(r.retry || 60)) : TEXT.login_wrong);
    } catch {
      setErr(TEXT.login_unreachable);
    } finally {
      setBusy(false);
    }
  };

  const voice = pair?.mode === 'code' || pair?.mode === 'seq';
  return <main className="auth auth-screen" data-theme={theme}><div className="auth-brand"><Logo cls="login-logo" /><span className="auth-eyebrow">SATELLITE1</span><h1 className="auth-headline"><span className="auth-line white">Your home.</span><span className="auth-line violet">Your voice.</span><span className="auth-line white">Your AI.</span></h1></div><div className="auth-panel"><div className="auth-form">{pair?.s === 'pending' ? <section className="voice-login" role="status"><div className="voice-pulse"><Icon name="mic" size={28} /></div><span className="eyebrow">{voice ? 'LISTENING' : 'WAITING'} · {clock(pair.left ?? 0)}</span><p className="voice-mode">{modeText(pair.mode, pair.hw, pair.hab)}</p>{pair.mode === 'seq' && pair.seq && <><div className="challenge" role="list" aria-label="Challenge words, in order">{[...pair.seq].map((c, i) => <b key={i} role="listitem" className={i < (pair.p || 0) ? 'done' : ''}>{TEXT.login_seq_words[+c] || '?'}</b>)}</div><p className="voice-hint">{TEXT.login_seq_peers_hint}</p></>}<button className="text-button" onClick={cancelPair}>{TEXT.login_cancel}</button></section> : pair ? <section className="voice-login" role="status"><p className={'voice-mode' + (pair.s === 'busy' ? ' warn' : '')}>{ENDED[pair.s]}</p><button className="voicetap-btn" onClick={startPair}><span>{TEXT.login_retry}</span></button><button className="text-button" onClick={() => setPair(null)}>Use the password instead</button></section> : <div className="auth-stack"><button className="voicetap-btn" onClick={startPair}><svg className="voicetap-icon" width="20" height="20" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d="M8 2a2 2 0 0 1 2 2v4a2 2 0 0 1-4 0V4a2 2 0 0 1 2-2Zm-4 6a4 4 0 0 0 8 0m-4 4v3m-2 0h4" /></svg><span>Sign in with VoiceTap</span></button><div className="or"><span>{TEXT.login_or}</span></div><form onSubmit={submit}><input ref={pwRef} aria-label={TEXT.login_pw_placeholder} type="password" placeholder={TEXT.login_pw_placeholder} autoComplete="current-password" value={password} onInput={e => {
              setPassword((e.target as HTMLInputElement).value);
              setErr(null);
            }} />{err && <p className="error">{err}</p>}<button className="secondary wide" type="submit" disabled={busy}>{TEXT.login_pw_submit}</button></form><p className="voice-hint">{TEXT.login_pw_hint}</p></div>}</div></div><button className="theme-toggle auth-theme" onClick={toggleTheme} aria-label={theme === 'dark' ? TEXT.theme_to_light : TEXT.theme_to_dark}><Icon name={theme === 'dark' ? 'sun' : 'moon'} /></button><div className="login-dev">{dev}</div></main>;
}
