/**
 * The login screen: VoiceTap sign-in (the pairing window with its breathing pulse and countdown)
 * over the challenge-response password fallback. Ported from login.jsx.
 *
 * The device picks the window's mode when it opens - the button alone when the microphones are
 * muted, the spoken code when Home Assistant can announce one, the wake-word challenge when it
 * cannot. The preview shows all three: each manual open cycles button -> code -> challenge, and
 * the wizard's magical ending arrives in code mode. Every mode "approves" a few seconds in, the
 * way a press or a spoken answer would.
 */
import React, { useEffect, useRef, useState } from 'react';
import { Logo } from './ui';
import { DEVICE } from './mock';
const TEXT = {
  sub: 'Your home. Your voice. Your AI.',
  tap: 'Use VoiceTap Sign-In',
  mode_button: 'Press the action button on top of your Satellite1 - the ring is breathing while it waits.',
  mode_code: 'Press the action button, or say the code.',
  mode_seq: 'Say the six wake words below, in order - or press the action button on top of your Satellite1.',
  seq_peers_hint: "If other Satellite1 devices are within earshot, mute them first so they don't answer the challenge.",
  cancel: 'Cancel',
  expired: 'Nothing answered in time, so the sign-in closed. Start again, then press the button or answer while the countdown runs.',
  retry: 'Try again',
  or: 'or use the password',
  pw_placeholder: 'Password',
  pw_submit: 'Sign in',
  show_pw: 'Show password',
  hide_pw: 'Hide password',
  wrong: "That's not the password.",
  pw_hint: 'See "Web UI Password" on this device\u2019s page in Home Assistant.'
};
const SEQ_WORDS = ['Hey Jarvis', 'Okay Nabu', 'Stop'];
const MODES = ['button', 'code', 'seq'] as const;
type Mode = (typeof MODES)[number];
const clock = (s: number) => `${Math.floor(s / 60)}:${String(Math.max(0, s) % 60).padStart(2, '0')}`;

/** The challenge words as ordered chips, un-bolding as the device hears each one back. */
const SeqChips = ({
  seq,
  p
}: {
  seq: string;
  p: number;
}) => <div className="login-seq" role="list" aria-label="Challenge words, in order">
    {[...seq].map((c, i) => <span key={i} role="listitem" className={`login-chip${i < (p || 0) ? ' done' : ''}`}>
        {SEQ_WORDS[+c] || '?'}
      </span>)}
  </div>;
type Pair = {
  s: 'pending' | 'expired';
  mode?: Mode;
  left?: number;
  seq?: string;
  p?: number;
} | null;

/* Module-scoped so the cycle survives the screen remounting on every sign-out: each opened
   window shows the next of the three modes the device can pick. */
let opensGlobal = 0;
export function LoginScreen({
  onSignedIn,
  autoPair,
  onSetup
}: {
  onSignedIn: () => void;
  autoPair?: boolean;
  onSetup?: () => void;
}) {
  const [pair, setPair] = useState<Pair>(null);
  const [pw, setPw] = useState('');
  const [showPw, setShowPw] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [shake, setShake] = useState(0);
  const timers = useRef<number[]>([]);
  const clear = () => {
    timers.current.forEach(t => window.clearInterval(t));
    timers.current = [];
  };
  useEffect(() => clear, []);

  /** The pairing window. Each mode resolves the way its real approval would: the button press
   *  and the spoken code land ~8s in; the challenge un-bolds a chip every couple of seconds. */
  const startPair = (forced?: Mode) => {
    setErr(null);
    clear();
    const mode = forced ?? MODES[opensGlobal % MODES.length];
    opensGlobal += 1;
    const seq = '012102';
    setPair({
      s: 'pending',
      mode,
      left: 120,
      seq,
      p: 0
    });
    let left = 120;
    let heard = 0;
    const t = window.setInterval(() => {
      left -= 1;
      if (mode === 'seq') {
        if (left <= 114 && left % 2 === 0 && heard < seq.length) heard += 1;
        if (heard >= seq.length) {
          clear();
          onSignedIn();
          return;
        }
      } else if (left <= 112) {
        // The action button press (or the spoken code), heard.
        clear();
        onSignedIn();
        return;
      }
      if (left <= 0) {
        clear();
        setPair({
          s: 'expired'
        });
        return;
      }
      setPair({
        s: 'pending',
        mode,
        left,
        seq,
        p: heard
      });
    }, 1000);
    timers.current.push(t);
  };
  const cancelPair = () => {
    clear();
    setPair(null);
  };
  useEffect(() => {
    // The wizard's magical ending: onboarding just completed and Home Assistant can speak, so
    // the window opens itself in code mode - one press (or four digits) and they are in.
    if (autoPair) startPair('code');
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  const submitPw = (e: React.FormEvent) => {
    e.preventDefault();
    if (!pw) return;
    // The demo accepts anything but the empty string - "wrong" (literally) shows the error path.
    if (pw.toLowerCase() === 'wrong') {
      setErr(TEXT.wrong);
      setShake(n => n + 1);
      return;
    }
    onSignedIn();
  };
  const pending = pair?.s === 'pending';
  const ended = pair && !pending;
  const modeText = pair?.mode === 'code' ? TEXT.mode_code : pair?.mode === 'seq' ? TEXT.mode_seq : TEXT.mode_button;
  return <div className="login">
      <div className="login-glow" aria-hidden="true" />
      <div className="login-hero">
        <Logo />
        <h1 className="login-name">Satellite1</h1>
        <div className="login-sub">{TEXT.sub}</div>
      </div>

      <div className="card login-card">
        {!pair && <button className="btn solid login-device" onClick={() => startPair()}>
            {TEXT.tap}
          </button>}

        {pending && pair && <div className="login-pending" role="status">
            <div className="login-mode">{modeText}</div>
            {pair.mode === 'seq' && pair.seq && <>
                <SeqChips seq={pair.seq} p={pair.p || 0} />
                <div className="login-peers-hint">{TEXT.seq_peers_hint}</div>
              </>}
            <div className="login-left-row">
              <span className="login-pulse" aria-hidden="true" />
              <div className="login-left">{clock(pair.left ?? 0)}</div>
            </div>
            <button className="btn ghost sm" onClick={cancelPair}>
              {TEXT.cancel}
            </button>
          </div>}

        {ended && <div className="login-ended" role="status">
            <div className="login-endmsg">{TEXT.expired}</div>
            <button className="btn sm" onClick={() => startPair()}>
              {TEXT.retry}
            </button>
          </div>}

        <div className="login-or" aria-hidden="true">
          <span>{TEXT.or}</span>
        </div>

        <form className="login-form" onSubmit={submitPw}>
          <div className="login-row">
            <div className={`login-field${err ? ' err' : ''}`} key={shake}>
              <input type={showPw ? 'text' : 'password'} placeholder={TEXT.pw_placeholder} autoComplete="current-password" value={pw} onInput={e => setPw((e.target as HTMLInputElement).value)} aria-label={TEXT.pw_placeholder} />
              <button type="button" className="login-eye" aria-label={showPw ? TEXT.hide_pw : TEXT.show_pw} onClick={() => setShowPw(v => !v)}>
                {showPw ? <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round">
                    <path d="M2 8s2.2-3.8 6-3.8S14 8 14 8s-2.2 3.8-6 3.8S2 8 2 8Z" />
                    <circle cx="8" cy="8" r="1.7" />
                    <path d="M3 13 13 3" />
                  </svg> : <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round">
                    <path d="M2 8s2.2-3.8 6-3.8S14 8 14 8s-2.2 3.8-6 3.8S2 8 2 8Z" />
                    <circle cx="8" cy="8" r="1.7" />
                  </svg>}
              </button>
            </div>
            <button className="btn solid login-submit" type="submit">
              {TEXT.pw_submit}
            </button>
          </div>
          {err && <div className="login-err">{err}</div>}
          <div className="login-hint">{TEXT.pw_hint}</div>
        </form>
      </div>

      <div className="login-dev">{`${DEVICE.name}.local`}</div>

      {/* Preview-only: the wizard is what a factory-fresh device serves instead of this screen,
          so the canvas needs a door to it. The real login page has no such link. */}
      {onSetup && <button className="btn ghost sm" onClick={onSetup}>
          Preview the first-boot setup wizard
        </button>}
    </div>;
}