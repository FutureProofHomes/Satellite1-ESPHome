import { useState } from 'react';
import { CONFIRM, HINTS, TEXT } from '../../../src/copy.js';
import { changePassword, logoutAll, mdnsLooksBroken, qrSignInLink, signInLink } from '../../../src/lib/auth.js';
import { qrSvgPath } from '../../../src/lib/qr.js';
import { toast } from '../../../src/lib/toast.js';
import type { Ctx } from '../../ctx';
import { passwordProblem } from '../../lib/settings.js';
import { DxCard, DxConfirm, useCopied } from './dx';

/**
 * The sign-in link and its QR, both carrying the key GET /api/sat1/state hands a signed-in browser.
 * The QR is built on the current IP (scanned live, and phones that cannot resolve mDNS still reach
 * it); the copyable link keeps the .local name, which survives DHCP - unless this browser has
 * proven it cannot resolve .local, where a .local link would be a dead end. Both are bearer
 * credentials, and "Sign out everywhere" beside them is what makes offering them defensible: every
 * session, link and QR dies, this browser is re-keyed in place, and the next state poll carries
 * the new key here. Ways to sign in, in docs/web-ui.md, has the full reasoning.
 */
export function AuthTokenCard({
  ctx
}: {
  ctx: Ctx;
}) {
  const d = ctx.device;
  const [copied, copy] = useCopied();
  if (!d?.key) return null;
  const noMdns = mdnsLooksBroken();
  const ipLink = qrSignInLink(d.ip, d.key);
  const link = noMdns && ipLink ? ipLink : signInLink(d.name, d.key);
  const qr = qrSvgPath(ipLink || link);
  return <DxCard title="Auth Token" collapsible defaultOpen={true} hint={HINTS.launch}>
      <div className="dx-launch">
        {qr && <svg className="dx-qr" viewBox={`-2 -2 ${qr.size + 4} ${qr.size + 4}`} role="img" aria-label="Sign-in QR code">
            <path d={qr.path} />
          </svg>}
        <div className="dx-launch-side">
          <div className="dx-launch-link">{link}</div>
          {noMdns && ipLink && <p className="dx-muted dx-sm dx-flush">{TEXT.launch_mdns_hint}</p>}
          <div className="dx-launch-actions">
            <button className="dx-btn" onClick={() => copy(link)}>{copied ? TEXT.launch_copied : TEXT.launch_copy}</button>
            <DxConfirm label={TEXT.launch_regen} danger title={TEXT.launch_regen_title} body={TEXT.launch_regen_body} confirmLabel={TEXT.launch_regen_confirm} onConfirm={() => logoutAll().catch(() => {})} />
          </div>
        </div>
      </div>
    </DxCard>;
}

/**
 * The authenticated password change. The current password is proven with the login's nonce
 * challenge and never crosses the wire; the new one is checked against the firmware's own rule at
 * the moment of commitment, so a mismatch typed after the dialog opened is still caught and nothing
 * invalid leaves the browser. A success re-keys this browser in place and kills every other
 * session, pasted link and QR - the logout_all contract, which is why the submit goes through a
 * confirm that says so. The Auth Token card picks up the new key on the next state poll.
 */
function ChangePassword() {
  const [cur, setCur] = useState('');
  const [next, setNext] = useState('');
  const [again, setAgain] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const submit = async () => {
    if (busy) return;
    setBusy(true);
    setErr(null);
    try {
      const r: any = await changePassword(cur, next);
      if (r.ok) {
        setCur('');
        setNext('');
        setAgain('');
        toast({
          kind: 'ok',
          title: TEXT.pw_changed,
          sub: TEXT.pw_changed_sub,
          ttl: 6000
        });
      } else if (r.locked) {
        setErr(TEXT.login_locked.replace('%s', String(r.retry || 60)));
      } else if (r.fixed) {
        setErr(TEXT.pw_fixed_note);
      } else if (r.invalid) {
        setErr(TEXT.pw_chars);
      } else {
        setErr(TEXT.pw_wrong);
      }
    } catch {
      setErr(TEXT.login_unreachable);
    } finally {
      setBusy(false);
    }
  };
  return <div className="dx-pwc">
      <p className="dx-pwc-title">{TEXT.pw_title}</p>
      <input className="dx-in" type="password" value={cur} placeholder={TEXT.pw_current} autoComplete="current-password" aria-label={TEXT.pw_current} onChange={e => setCur(e.currentTarget.value)} />
      <input className="dx-in" type="password" value={next} placeholder={TEXT.pw_new} autoComplete="new-password" aria-label={TEXT.pw_new} onChange={e => setNext(e.currentTarget.value)} />
      <input className="dx-in" type="password" value={again} placeholder={TEXT.pw_again} autoComplete="new-password" aria-label={TEXT.pw_again} onChange={e => setAgain(e.currentTarget.value)} />
      <div className="dx-pwc-actions">
        <DxConfirm label={busy ? TEXT.pw_busy : TEXT.pw_title} title={CONFIRM.pw_change.t} body={CONFIRM.pw_change.b} confirmLabel={TEXT.pw_title} danger disabled={busy || !cur || !next || !again} onConfirm={() => {
        const bad = passwordProblem(next, again);
        if (bad) {
          setErr(TEXT[bad]);
          return;
        }
        submit();
      }} />
      </div>
      {err && <p className="dx-err" role="alert">{err}</p>}
    </div>;
}
/**
 * Hidden on pw_fixed builds: a YAML-pinned fleet password is re-imposed on every boot, so a change
 * would silently revert, and the device refuses one anyway.
 */
export function ChangePasswordCard({
  ctx
}: {
  ctx: Ctx;
}) {
  if (ctx.device?.pw_fixed) return null;
  return <DxCard title="Change Password" collapsible defaultOpen={true}>
      <ChangePassword />
    </DxCard>;
}
