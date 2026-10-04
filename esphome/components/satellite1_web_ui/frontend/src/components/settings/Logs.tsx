import { useEffect, useRef, useState } from 'react';
import { CONFIRM, HINTS, TEXT } from '../../copy.js';
import { apiUrl, post, request, requestJson } from '../../lib/device.js';
import type { Ctx } from '../../ctx';
import { ALL_CHIPS, chipCounts, crashWhen, filterLog, LOG_CHIPS, logExport, logFileName, logParts, stamp, uptime } from '../../lib/settings.js';
import { Caret, DxCard, DxConfirm, DxRow, saveBlob } from './dx';

type LogLine = {
  lvl: string;
  text: string;
  at: number;
};
export type LogIntent = {
  card: 'log';
  levels?: string[];
  line?: {
    at: number;
    text: string;
  };
} | null;

/* ESPHome's own console palette, so nobody learns a second scheme. Verbose alone stays grey: it is
   the chatter you filter out, and colouring it would leave nothing dim to compare against. */
const LVL_CLASS: Record<string, string> = {
  E: 'dx-err',
  C: 'dx-cfg',
  W: 'dx-warn',
  I: 'dx-ok',
  D: 'dx-dbg',
  V: 'dx-dim',
  VV: 'dx-dim'
};

/** Ring entries are stable objects, so each line is parsed once rather than on every new line. */
const partsCache = new WeakMap<LogLine, {
  tag: string;
  msg: string;
}>();
const partsOf = (l: LogLine) => {
  let p = partsCache.get(l);
  if (!p) {
    p = logParts(l.text);
    partsCache.set(l, p);
  }
  return p;
};

/**
 * The device's own log, live from /events. Registered as the log's reader while mounted, which
 * both lets new lines re-render it and silences the warning toasts for the person already reading;
 * the ring fills either way, which is what shows the recent past on arrival. A toast's intent seeds
 * the lit chips, so a warning's tap lands on the warnings rather than the debug firehose that buried
 * them, and names a line to scroll to and flash.
 */
export function LogsCard({
  ctx,
  intent
}: {
  ctx: Ctx;
  intent: LogIntent;
}) {
  const {
    log,
    logSeq,
    pausedRef,
    logWatch,
    clearLog
  } = ctx;
  const [paused, setPaused] = useState(false);
  useEffect(() => logWatch(), []);
  const [chips, setChips] = useState<string[]>(intent?.levels || ALL_CHIPS);
  const [hl, setHl] = useState(intent?.line || null);
  // An intent arriving while the card already stands: a write-failed toast, or a notification
  // history row carrying a line. Log toasts cannot, being silenced while this card watches.
  useEffect(() => {
    if (intent?.levels) setChips(intent.levels);
    if (intent?.line) setHl(intent.line);
  }, [intent]);
  const [filter, setFilter] = useState('');
  const box = useRef<HTMLDivElement>(null);
  const atBottom = useRef(true);
  const lines: LogLine[] = filterLog(log, chips, filter);
  const counts = chipCounts(log, filter);
  const toggleChip = (c: string) => setChips(on => on.includes(c) ? on.filter(x => x !== c) : [...on, c]);

  // Follows the newest line only while the view is already at the bottom: reading anything on a
  // chatty device is impossible if every line yanks the view away.
  useEffect(() => {
    if (!paused && atBottom.current && box.current) box.current.scrollTop = box.current.scrollHeight;
  }, [logSeq, paused, filter, chips]);
  useEffect(() => () => {
    pausedRef.current = false;
  }, [pausedRef]);

  // The panel scrolls itself rather than scrollIntoView, which would fight the page's own scroll to
  // the card, and atBottom is parked so the live stream cannot yank the view away mid-flash. A line
  // that has left the 1000-line ring (or never entered it, arriving while paused) falls back to the
  // freshest view, unflashed rather than lighting a stranger.
  useEffect(() => {
    if (!hl) return undefined;
    const el = box.current?.querySelector<HTMLElement>('.dx-log-line.hl');
    if (el && box.current) {
      box.current.scrollTop = Math.max(0, el.offsetTop - box.current.clientHeight / 2);
      atBottom.current = false;
    } else if (box.current) {
      box.current.scrollTop = box.current.scrollHeight;
    }
    const t = setTimeout(() => setHl(null), 2600);
    return () => clearTimeout(t);
  }, [hl]);
  const togglePause = () => {
    const next = !paused;
    setPaused(next);
    // Paused stops the ring taking lines, not just the scrolling: a noisy boot would otherwise push
    // the line being read out of the 1000-line ring.
    pausedRef.current = next;
  };
  return <DxCard title="Device Logs" collapsible defaultOpen={true} forceOpen={!!intent} hint={HINTS.log} id="card-log">
      <div className="dx-log-bar">
        <input className="dx-log-search" type="search" placeholder="Filter…" aria-label="Filter logs" value={filter} onChange={e => setFilter(e.currentTarget.value)} />
        <div className="dx-log-chips" role="group" aria-label="Log levels">
          {LOG_CHIPS.filter(([c]) => c !== 'V' || counts.V).map(([c, label]) => {
          const on = chips.includes(c);
          return <button key={c} className={`dx-log-chip${on ? ' on' : ''}`} data-l={c} aria-pressed={on} onClick={() => toggleChip(c)}>
                {label}<span className="dx-log-chip-n">{counts[c] || 0}</span>
              </button>;
        })}
        </div>
      </div>
      <div className="dx-log" ref={box} onScroll={e => {
      const el = e.currentTarget;
      // Slack, because a fractional scrollHeight on a zoomed display never lands exactly on the
      // bottom, and the panel would stop following.
      atBottom.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
    }}>
        {lines.length === 0 && <p className="dx-muted dx-sm">{log.length === 0 ? 'Waiting for the device to say something.' : chips.length === 0 ? 'Every level is off. Turn one on to see its lines.' : 'No lines match. Every line is filtered out.'}</p>}
        {lines.map((l, i) => {
        const p = partsOf(l);
        return <div key={i} className={`dx-log-line ${LVL_CLASS[l.lvl] || ''}${hl && l.at === hl.at && l.text === hl.text ? ' hl' : ''}`}>
              <span className="dx-log-ts">{stamp(l.at)}</span>
              <span className="dx-log-lvl">{l.lvl === '?' ? '' : l.lvl}</span>
              <span className="dx-log-tag">{p.tag}</span>
              <span className="dx-log-txt">{p.msg}</span>
            </div>;
      })}
      </div>
      <div className="dx-log-foot">
        <span className="dx-muted dx-xs" style={{
        padding: 0,
        flex: 1
      }}>{lines.length === log.length ? `${log.length} lines` : `${lines.length} of ${log.length} lines`}</span>
        <button className={`dx-btn sm${!paused ? ' active' : ''}`} aria-pressed={!paused} onClick={togglePause}>{paused ? 'Resume' : 'Following'}</button>
        <button className="dx-btn sm" disabled={log.length === 0} onClick={() => {
        setHl(null);
        clearLog();
      }}>Clear</button>
        <button className="dx-btn sm" disabled={lines.length === 0} onClick={() => saveBlob(new Blob([logExport(lines)], {
        type: 'text/plain'
      }), logFileName(new Date()))}>Export</button>
      </div>
    </DxCard>;
}

/**
 * The crash history the firmware kept across reboots: GET /api/sat1/crash, re-read when the state
 * poll's crash count moves (rarely - a new crash only arrives with a reboot) and after an erase.
 * The pre-crash log is its own lazy read behind Show: 4KB that most visits never read. A build
 * without crash_report answers 404, and the card is not drawn. Crash reports in docs/web-ui.md
 * covers the capture layers and how to decode a dump.
 */
export function CrashCard({
  ctx,
  reveal
}: {
  ctx: Ctx;
  reveal: boolean;
}) {
  const [data, setData] = useState<any>(null);
  const [tail, setTail] = useState<string | null>(null);
  const [showLog, setShowLog] = useState(false);
  // One backtrace open at a time: the addresses are for copying, and two walls of hex help nobody.
  const [openBt, setOpenBt] = useState(-1);
  const count = ctx.device?.crash;
  const load = () => requestJson('/api/sat1/crash').then((d: any) => d && setData(d)).catch(() => {});
  useEffect(() => {
    load();
  }, [count]);
  if (!data) return null;
  const toggleLog = () => {
    const next = !showLog;
    setShowLog(next);
    if (next && tail === null) request('/api/sat1/crash/log').then((r: any) => setTail(r.ok ? r.text : '')).catch(() => setTail(''));
  };
  // A raw fetch, because request() would decode the binary as UTF-8 text and corrupt it; the
  // session rides fetch's same-origin cookie, or apiUrl's key for a remote device.
  const downloadDump = () => fetch(apiUrl('/api/sat1/crash/dump.bin')).then(r => r.ok ? r.blob() : Promise.reject(new Error(String(r.status)))).then(b => saveBlob(b, `${ctx.device?.name || 'satellite1'}-coredump.bin`)).catch(() => {});
  const records: any[] = data.records || [];
  return <DxCard title={TEXT.crash_title} collapsible defaultOpen={true} forceOpen={reveal} hint={HINTS.crash} id="card-crash">
      {records.length === 0 && <p className="dx-muted">{TEXT.crash_none}</p>}
      {records.map((r, k) => <div className="dx-crash" key={k}>
          <div className="dx-crash-head">
            <span className="dx-crash-t">{r.txt || r.rs}</span>
            <span className="dx-crash-ran">{TEXT.crash_ran.replace('%s', uptime(r.up))}</span>
          </div>
          <p className="dx-crash-when">{crashWhen(r, data.boot, ctx.device?.uptime)}</p>
          {r.task && <p className="dx-crash-at">
              {r.task} &middot; cause {r.cause} &middot; PC {r.pc}
              {r.vaddr && r.vaddr !== '0x00000000' ? ` \u00b7 addr ${r.vaddr}` : ''}
            </p>}
          {r.bt && r.bt.length > 0 && <>
              <button className="dx-btn sm dx-crash-bt" aria-expanded={openBt === k} onClick={() => setOpenBt(openBt === k ? -1 : k)}>
                {r.cor ? TEXT.crash_bt_corrupt : TEXT.crash_bt} <Caret open={openBt === k} size={12} />
              </button>
              {openBt === k && <p className="dx-crash-at">{r.bt.join(' ')}</p>}
            </>}
        </div>)}
      {data.log > 0 && <>
          <DxRow label={TEXT.crash_log_row} hint={HINTS.crash_log}>
            <button className="dx-btn sm dx-crash-bt" aria-expanded={showLog} onClick={toggleLog}>
              {showLog ? TEXT.crash_log_hide : TEXT.crash_log_show} <Caret open={showLog} size={12} />
            </button>
          </DxRow>
          {showLog && <div className="dx-log dx-crash-log">{tail === null ? '\u2026' : tail || TEXT.crash_log_none}</div>}
        </>}
      {data.dump > 0 && <DxRow label={TEXT.crash_dump_row} hint={HINTS.crash_dump}>
          <button className="dx-btn" onClick={downloadDump}>{TEXT.crash_download}</button>
        </DxRow>}
      {(data.dump > 0 || records.length > 0) && <DxRow label={TEXT.crash_erase_row} hint={HINTS.crash_erase}>
          <DxConfirm label={TEXT.crash_erase} title={CONFIRM.crash_erase.t} body={CONFIRM.crash_erase.b} confirmLabel={TEXT.crash_erase} danger onConfirm={() => post('/api/sat1/crash/erase').then(() => {
          setTail(null);
          setShowLog(false);
          load();
        })} />
        </DxRow>}
      {!data.part && <p className="dx-muted dx-xs">{TEXT.crash_no_part}</p>}
    </DxCard>;
}
