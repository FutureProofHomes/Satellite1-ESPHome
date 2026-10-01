import { useEffect, useRef, useState } from 'react';
import { HINTS, TEXT } from '../../copy.js';
import { maCfgSave, maSettings, scanForMa } from '../../lib/ma.js';
import type { Tiers } from './model';

type Scan = null | { done: number; total: number; list?: undefined; err?: undefined } | { list: { id: string; url: string; name: string; version: string }[] } | { err: 'none' | 'https' };

/**
 * The direct Music Assistant connection, folded shut at the bottom of Now Playing and opened in
 * place by search's "Set up connection" (`defaultOpen`, because there the person has already asked
 * for it). The one place the app asks for a credential, and deliberately buried: the bar is
 * complete without it. The token is a long-lived one from MA's profile settings; it lands in this
 * browser's storage and, behind the sign-in, on the device (owner's request, September 2026), so
 * the next browser seeds itself from it - src/lib/ma.js has the trust reasoning. The dot on the fold
 * is the connection, so the panel can stay shut once it works. "Find my server" sweeps the device's
 * own /24 on MA's port and verifies each listener over the WebSocket hello (scanForMa); `ip` is the
 * device's address, which is on that subnet by definition.
 */
export function MaPanel({
  tiers,
  ip,
  defaultOpen
}: {
  tiers: Tiers;
  ip?: string;
  defaultOpen?: boolean;
}) {
  const { maCfg, setMaCfg, ws } = tiers;
  const [open, setOpen] = useState(!!defaultOpen);
  const [url, setUrl] = useState(maCfg.url);
  const [token, setToken] = useState(maCfg.token);
  const configured = !!(maCfg.url && maCfg.token);
  const status = configured ? ws.status : 'off';

  // A panel closed mid-sweep drops the answer rather than setting dead state.
  const [scan, setScan] = useState<Scan>(null);
  const scanLive = useRef(false);
  useEffect(() => () => {
    scanLive.current = false;
  }, []);
  const startScan = () => {
    scanLive.current = true;
    setScan({ done: 0, total: 0 });
    scanForMa(ip, (done: number, total: number) => scanLive.current && setScan(s => s && 'list' in s ? s : { done, total })).then((list: any[]) => scanLive.current && setScan(list.length ? { list } : { err: 'none' })).catch(() => scanLive.current && setScan({ err: 'https' }));
  };

  const connect = () => {
    const u = url.trim();
    const t = token.trim();
    if (!u || !t) return;
    maSettings.set(u, t);
    setMaCfg({ url: u, token: t });
    maCfgSave(u, t);
  };
  const disconnect = () => {
    maSettings.clear();
    setMaCfg({ url: '', token: '' });
    maCfgSave('', '');
  };

  const players = Object.values(ws.players || {}).filter((p: any) => p.available).length;
  const note = !configured ? 'Not connected' : status === 'on' ? `Connected \u00b7 ${players} ${players === 1 ? 'player' : 'players'} available` : status === 'error' ? TEXT.ma_error : TEXT.search_connecting;

  return <div className="mapanel">
    <button className="mapanel-head" onClick={() => setOpen(!open)} aria-expanded={open}><span className={'madot' + (status === 'on' ? ' on' : '')} /><span>{TEXT.ma_title}</span><svg width="16" height="16" viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" style={{
        transform: open ? 'rotate(180deg)' : 'none'
      }} aria-hidden="true"><path d="m5 7 3 3 3-3" /></svg></button>
    {open && <div className="mapanel-body">
      <p className="mapanel-hint">{HINTS.ma_connect}</p>
      <input aria-label="Music Assistant address" value={url} onInput={e => setUrl(e.currentTarget.value)} placeholder={TEXT.ma_url_ph} />
      {!scan && <button className="secondary mascan-btn" onClick={startScan}>{TEXT.ma_scan_btn}</button>}
      {scan && !('list' in scan) && !('err' in scan) && <small>{TEXT.ma_scanning.replace('%s', scan.total ? `${scan.done}/${scan.total}` : '')}</small>}
      {scan && 'err' in scan && <small>{scan.err === 'https' ? TEXT.ma_scan_https : TEXT.ma_scan_none}</small>}
      {scan && 'list' in scan && scan.list.map(s => <button key={s.id} className={'mascan-row' + (url.trim() === s.url ? ' on' : '')} aria-pressed={url.trim() === s.url} onClick={() => setUrl(s.url)}><b>{s.name}</b><small>{s.version ? `${s.version} \u00b7 ` : ''}{s.url.replace(/^https?:\/\//, '')}</small></button>)}
      <input aria-label="Music Assistant token" type="password" value={token} onInput={e => setToken(e.currentTarget.value)} placeholder={TEXT.ma_token_ph} />
      <div className="mapanel-actions"><button className="primary" onClick={connect}>{TEXT.ma_connect_btn}</button>{configured && <button className="secondary" onClick={disconnect}>{TEXT.ma_disconnect_btn}</button>}</div>
      <small role="status">{note}</small>
    </div>}
  </div>;
}
