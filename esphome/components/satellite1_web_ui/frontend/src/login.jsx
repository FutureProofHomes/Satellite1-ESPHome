/**
 * The login screen: the first thing a browser without a session sees, and for most people the last
 * time they see it for 90 days.
 *
 * Two ways in, in the order we would rather they used. "Sign in on this device" opens the pairing
 * window - the ring breathes, the device may speak, and a press of the action button or the right
 * spoken answer is the approval; no secret is ever typed. The password form underneath is the
 * universal fallback, challenge-response so the password never crosses the wire.
 *
 * The instructions under the pending spinner follow what the poll reports, because the device picks
 * the mode when the window opens: the button alone when the microphones are muted, the spoken code
 * when Home Assistant is there to speak it, the wake-word challenge when it is not.
 */
import { useEffect, useRef, useState } from "preact/hooks";

import { loginPassword, pairCancel, pairPoll, pairStart, whoami } from "./lib/auth.js";
import { TEXT } from "./copy.js";
/* The FutureProofHomes mark lived here while this screen was its only wearer; it moved to ui.jsx
   when the splash overlay (splash.jsx) started opening every session with the same mark. */
import { Logo } from "./ui.jsx";

/** m:ss out of seconds, for the window countdown. */
const clock = (s) => `${Math.floor(s / 60)}:${String(Math.max(0, s) % 60).padStart(2, "0")}`;

/** hw rides the poll: a button-only window forced by the hardware mute slider gets the copy that
 *  says how to get voice sign-in back. */
const modeText = (mode, hw) =>
  mode === "code" ? TEXT.login_mode_code
  : mode === "seq" ? TEXT.login_mode_seq
  : hw ? TEXT.login_mode_button_hw
  : TEXT.login_mode_button;

export function LoginScreen({ onSignedIn }) {
  // null = idle; otherwise {s, mode, left} mirroring the poll, plus the local "error" state.
  const [pair, setPair] = useState(null);
  const [pw, setPw] = useState("");
  const [showPw, setShowPw] = useState(false);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState(null);
  // Bumped on every wrong password so the animation re-runs; the value is only a key.
  const [shake, setShake] = useState(0);
  // The submit reads the field itself rather than trusting the state to have caught up: a password
  // manager's autofill followed by an immediate Enter can outrun the re-render, and a submit that
  // silently does nothing is indistinguishable from a broken page.
  const pwRef = useRef(null);
  const pollTimer = useRef(null);
  const live = useRef(true);

  useEffect(() => {
    // The tab title, pre-login: the hostname is the only name this page may know (the friendly
    // name lives behind the session), and it beats an address in a row of tabs. The app proper
    // upgrades it to the friendly name after sign-in.
    whoami().then((n) => {
      if (n) document.title = n;
    });
    return () => {
      live.current = false;
      clearTimeout(pollTimer.current);
    };
  }, []);

  /** The pairing loop: open the window, then poll until it resolves. Chained timeouts rather than
   *  an interval, so a slow answer stretches the gap instead of stacking requests. */
  const startPair = async () => {
    setErr(null);
    let opened;
    try {
      opened = await pairStart();
    } catch {
      setPair({ s: "error" });
      return;
    }
    if (opened.pending) {
      // Someone else's window. The copy matters more than the state: a person standing at the
      // device "helping" by pressing the button would be approving a stranger's request.
      setPair({ s: "busy" });
      return;
    }
    if (!opened.ok) {
      setPair({ s: "error" });
      return;
    }
    setPair({ s: "pending", mode: opened.mode, left: opened.left, hw: opened.hw });

    const tick = async () => {
      let p;
      try {
        p = await pairPoll();
      } catch {
        p = null;
      }
      if (!live.current) return;
      if (p && p.s === "ok") {
        onSignedIn(p.key);
        return;
      }
      if (p && p.s === "pending") {
        setPair({ s: "pending", mode: p.mode, left: p.left, hw: p.hw });
        pollTimer.current = setTimeout(tick, 1200);
        return;
      }
      if (p && (p.s === "expired" || p.s === "denied" || p.s === "busy")) {
        setPair({ s: p.s });
        return;
      }
      // A dropped poll or "none": keep trying briefly - the device may be mid-announcement - but a
      // window that has truly gone resolves as expired on the next answer.
      pollTimer.current = setTimeout(tick, 1500);
    };
    pollTimer.current = setTimeout(tick, 1200);
  };

  const cancelPair = () => {
    clearTimeout(pollTimer.current);
    setPair(null);
    // Tell the device too: the window is this browser's own (the pair cookie proves it), so cancel
    // stops the breathing ring now and clears the way for an immediate fresh start, instead of
    // leaving an abandoned window that refuses the next click as busy for up to a minute.
    pairCancel();
  };

  const submitPw = async (e) => {
    e.preventDefault();
    const value = pwRef.current?.value ?? pw;
    if (!value || busy) return;
    setBusy(true);
    setErr(null);
    try {
      const r = await loginPassword(value);
      if (r.ok) {
        onSignedIn(r.key);
        return;
      }
      setErr(r.locked ? TEXT.login_locked.replace("%s", String(r.retry || 60)) : TEXT.login_wrong);
      setShake((n) => n + 1);
    } catch {
      setErr(TEXT.login_unreachable);
    } finally {
      setBusy(false);
    }
  };

  const pending = pair?.s === "pending";
  const ended = pair && !pending;

  return (
    <div class="login">
      <div class="login-glow" aria-hidden="true" />
      <div class="login-hero">
        <Logo />
        {/* The product, not the hostname - "satellite1-d2256c" belongs in the tab title, where the
            whoami effect above puts it. */}
        <h1 class="login-name">Satellite1</h1>
        <div class="login-sub">{TEXT.login_sub}</div>
      </div>

      <div class="card login-card">
        {!pair && (
          <button class="btn solid login-device" onClick={startPair}>
            {TEXT.login_tap}
          </button>
        )}

        {pending && (
          <div class="login-pending" role="status">
            <span class="login-pulse" aria-hidden="true" />
            <div class="login-mode">{modeText(pair.mode, pair.hw)}</div>
            <div class="login-left">{clock(pair.left ?? 0)}</div>
            <button class="btn ghost sm" onClick={cancelPair}>
              {TEXT.login_cancel}
            </button>
          </div>
        )}

        {ended && (
          <div class="login-ended" role="status">
            <div class={`login-endmsg${pair.s === "busy" ? " warn" : ""}`}>
              {pair.s === "busy" && TEXT.login_busy}
              {pair.s === "expired" && TEXT.login_expired}
              {pair.s === "denied" && TEXT.login_denied}
              {pair.s === "error" && TEXT.login_start_failed}
            </div>
            <button class="btn sm" onClick={startPair}>
              {TEXT.login_retry}
            </button>
          </div>
        )}

        <div class="login-or" aria-hidden="true">
          <span>{TEXT.login_or}</span>
        </div>

        <form class="login-form" onSubmit={submitPw}>
          {/* One row: the field grows, the submit sits beside it. */}
          <div class="login-row">
          <div class={`login-field${err ? " err" : ""}`} key={shake}>
            <input
              ref={pwRef}
              type={showPw ? "text" : "password"}
              placeholder={TEXT.login_pw_placeholder}
              autocomplete="current-password"
              autofocus
              value={pw}
              onInput={(e) => setPw(e.currentTarget.value)}
              aria-label={TEXT.login_pw_placeholder}
            />
            <button
              type="button"
              class="login-eye"
              aria-label={showPw ? TEXT.login_hide_pw : TEXT.login_show_pw}
              onClick={() => setShowPw((v) => !v)}
            >
              {showPw ? (
                <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round">
                  <path d="M2 8s2.2-3.8 6-3.8S14 8 14 8s-2.2 3.8-6 3.8S2 8 2 8Z" />
                  <circle cx="8" cy="8" r="1.7" />
                  <path d="M3 13 13 3" />
                </svg>
              ) : (
                <svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round">
                  <path d="M2 8s2.2-3.8 6-3.8S14 8 14 8s-2.2 3.8-6 3.8S2 8 2 8Z" />
                  <circle cx="8" cy="8" r="1.7" />
                </svg>
              )}
            </button>
          </div>
          {/* Disabled only while a login is in flight, not while the field looks empty: browsers
              suppress Enter-to-submit when the default button is disabled, and an autofilled value
              can be real before the re-render that would enable it. An empty submit is simply
              ignored by the handler. */}
          <button class="btn solid login-submit" type="submit" disabled={busy}>
            {busy ? <span class="login-spin" aria-hidden="true" /> : TEXT.login_pw_submit}
          </button>
          </div>
          {err && <div class="login-err">{err}</div>}
          <div class="login-hint">{TEXT.login_pw_hint}</div>
        </form>
      </div>
    </div>
  );
}
