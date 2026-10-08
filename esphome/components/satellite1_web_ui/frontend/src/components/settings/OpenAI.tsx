import { useEffect, useMemo, useRef, useState } from 'react';
import { HINTS, TEXT } from '../../copy.js';
import { toast } from '../../lib/toast.js';
import type { Ctx } from '../../ctx';
import { canConnect, canSave, deriveEndpoints, discover, discovery, isLocalHost, keyWillBeDropped, modelChoices, normalizeBaseUrl, originOf, protocolLabel, readOpenAI, reconcile, saveOpenAI, SAVE_ERRORS, stateLabel, testConnection, validKey, validModel, validVoice, voiceChoices } from '../../lib/openai.js';
import { Switch } from '../controls';
import { DxCard, DxFact, DxFacts, DxRow, DxSelect } from './dx';

const POLL_MS = 700;
const plural = (forms: string[], n: number) => forms[n === 1 ? 0 : 1].replace('%d', String(n));
const DEBOUNCE_MS = 700;

/**
 * Settings > OpenAI. Connect first, then choose: the device fetches the model and voice lists from
 * the server on screen (its /models and /audio/voices) whenever the base URL or a typed key
 * changes, and on opening the page with the saved ones. The model and voice pickers stay locked
 * until that succeeds for the address on screen, so they only ever offer what that server has.
 *
 * The device keeps the connection (GET/POST /api/sat1/openai) and never hands the key back; the
 * page edits drafts and saves them in one go.
 */
export function OpenAICards({
  ctx
}: {
  ctx: Ctx;
}) {
  const [saved, setSaved] = useState<any>(null);
  const [loadErr, setLoadErr] = useState(false);
  const [enabled, setEnabled] = useState(false);
  const [base, setBase] = useState('');
  const [key, setKey] = useState('');
  const [clearKey, setClearKey] = useState(false);
  const [model, setModel] = useState('');
  const [voice, setVoice] = useState('marin');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [discGen, setDiscGen] = useState<number | null>(null);
  const [testGen, setTestGen] = useState<number | null>(null);
  const seeded = useRef(false);
  const reconciledGen = useRef<number | null>(null);

  const reload = async () => {
    const s = await readOpenAI().catch(() => null);
    if (!s) {
      setLoadErr(true);
      return null;
    }
    setLoadErr(false);
    setSaved(s);
    if (!seeded.current) {
      seeded.current = true;
      setEnabled(!!s.enabled);
      setBase(s.base_url || '');
      setModel(s.model || '');
      setVoice(s.voice || 'marin');
    }
    return s;
  };

  const norm = normalizeBaseUrl(base);
  const ep = deriveEndpoints(base, model);
  const disc = discovery(saved?.models, norm, discGen);
  const t = saved?.test;
  const testMine = testGen != null && t?.gen === testGen;
  const waiting = disc.state === 'loading' || testMine && t?.st === 'running' || testGen != null && !testMine;

  useEffect(() => {
    reload();
  }, []);
  useEffect(() => {
    const id = setInterval(reload, waiting ? POLL_MS : 4000);
    return () => clearInterval(id);
  }, [waiting]);

  // Connect: on opening the page (with the saved address and key) and whenever the address or a
  // typed key changes. Debounced so typing a URL does not fire a request per keystroke; skipped
  // while the address or key is not valid yet.
  const connect = async () => {
    if (!ep || !validKey(key)) return;
    setErr(null);
    const r: any = await discover(norm, key).catch(() => null);
    if (r?.ok) setDiscGen(r.gen);else setErr(TEXT.oai_save_failed);
  };
  useEffect(() => {
    if (!saved || !ep || !validKey(key)) return undefined;
    const id = setTimeout(connect, DEBOUNCE_MS);
    return () => clearTimeout(id);
  }, [norm, key, saved != null]);

  const sameServer = !!saved && originOf(base) === originOf(saved.base_url);
  const modelIds = disc.ready ? disc.ids : [];
  const voiceIds = disc.ready ? disc.voices.map((v: any) => v[0]) : [];

  // A different server than the saved one: preselect what it offers, once per discovery.
  useEffect(() => {
    if (!disc.ready || reconciledGen.current === discGen) return;
    reconciledGen.current = discGen;
    if (disc.hasModelList) setModel(m => reconcile(m, modelIds, sameServer));
    if (disc.voices.length) setVoice(v => reconcile(v, voiceIds, sameServer));
  }, [disc.ready, discGen]);

  const models = useMemo(() => modelChoices(modelIds, model, TEXT.oai_not_offered), [modelIds.join('|'), model]);
  const voices = useMemo(() => voiceChoices(disc.ready ? disc.voices : [], voice, TEXT.oai_not_offered), [disc.ready && disc.voices, voice]);

  const hasKey = !!key || !!saved?.key_set && !clearKey && !keyWillBeDropped(saved, base, key);
  const dropWarn = keyWillBeDropped(saved, base, key) && !clearKey;
  const draft = { enabled, base, model, voice, key, clearKey };
  const saveOk = canSave({ saved, draft, disc });
  const dirty = !!saved && (enabled !== !!saved.enabled || norm !== saved.base_url || model !== saved.model || voice !== saved.voice || !!key || clearKey);

  const save = async () => {
    if (busy || !saveOk) return;
    setBusy(true);
    setErr(null);
    const body: any = {
      enabled,
      base_url: norm,
      model,
      voice
    };
    if (key) body.api_key = key;else if (clearKey) body.clear_key = true;
    try {
      const r = await saveOpenAI(body);
      if (r.ok) {
        setKey('');
        setClearKey(false);
        seeded.current = false;
        await reload();
        toast({
          kind: 'ok',
          title: TEXT.oai_saved,
          ttl: 4000
        });
      } else {
        setErr((SAVE_ERRORS as any)[r.err || ''] || TEXT.oai_save_failed);
      }
    } catch {
      setErr(TEXT.oai_save_failed);
    } finally {
      setBusy(false);
    }
  };

  const test = async () => {
    const r: any = await testConnection().catch(() => null);
    if (r?.ok) setTestGen(r.gen);else setErr(TEXT.oai_save_failed);
  };

  if (!saved) {
    return <DxCard title={TEXT.oai_card} collapsible defaultOpen={true}>
        {loadErr ? <p className="dx-err" role="alert">{TEXT.oai_load_failed}</p> : <p className="dx-muted">Reading…</p>}
      </DxCard>;
  }

  const local = isLocalHost(base);
  const connNote = disc.state === 'loading' ? TEXT.oai_connecting : disc.state === 'error' ? disc.error : disc.state === 'ok' ? disc.hasModelList ? TEXT.oai_connected.replace('%m', plural(TEXT.oai_n_models, disc.ids.length)).replace('%v', plural(TEXT.oai_n_voices, disc.voices.length)) : TEXT.oai_connected_no_list : '';
  const locked = !disc.ready;

  return <>
      <DxCard title={TEXT.oai_card} collapsible defaultOpen={true} hint={HINTS.oai_connection}>
        <DxRow label={TEXT.oai_enabled} hint={HINTS.oai_enabled}>
          <Switch on={enabled} label={TEXT.oai_enabled} onChange={setEnabled} />
        </DxRow>
        <div className="dx-pwc oai-form">
          <p className="oai-step">{TEXT.oai_step_connect}</p>
          <label className="oai-field">
            <span className="oai-label">{TEXT.oai_base_url}</span>
            <input className="dx-in" type="url" inputMode="url" spellcheck={false} autoComplete="off" value={base} placeholder="https://api.openai.com/v1" onInput={e => setBase(e.currentTarget.value)} aria-invalid={!ep} />
            <span className={ep ? 'oai-note' : 'oai-note oai-bad'}>{ep ? `${TEXT.oai_connects_to} ${ep.realtimeUrl}` : TEXT.oai_bad_url}</span>
            {ep && local && ep.secure && <span className="oai-note oai-warn">{TEXT.oai_local_tls}</span>}
          </label>

          <label className="oai-field">
            <span className="oai-label">{TEXT.oai_api_key}{local && <span className="oai-opt"> {TEXT.oai_optional_local}</span>}</span>
            <input className="dx-in" type="password" autoComplete="new-password" spellcheck={false} value={key} placeholder={saved.key_set && !clearKey && !dropWarn ? TEXT.oai_key_stored.replace('%s', saved.key_hint || '') : TEXT.oai_key_ph} onInput={e => {
            setKey(e.currentTarget.value.trim());
            setClearKey(false);
          }} aria-invalid={!validKey(key)} />
            {!validKey(key) && <span className="oai-note oai-bad">{SAVE_ERRORS.api_key}</span>}
            {dropWarn && <span className="oai-note oai-warn">{TEXT.oai_key_dropped}</span>}
            {saved.key_set && !key && !dropWarn && <span className="oai-note">
                <button type="button" className="oai-link" onClick={() => setClearKey(v => !v)}>{clearKey ? TEXT.oai_key_keep : TEXT.oai_key_remove}</button>
              </span>}
          </label>

          <div className="oai-conn" role="status">
            <span className={`oai-dot ${disc.state}`} aria-hidden="true" />
            <span className={disc.state === 'error' ? 'oai-note oai-bad' : 'oai-note'}>{connNote || TEXT.oai_not_connected}</span>
            <button type="button" className="dx-btn oai-connect" disabled={!ep || !validKey(key) || disc.state === 'loading'} onClick={connect}>
              {disc.state === 'ok' ? TEXT.oai_refresh : TEXT.oai_connect}
            </button>
          </div>

          <p className="oai-step">{TEXT.oai_step_choose}</p>
          <div className={`oai-field${locked ? ' oai-locked' : ''}`}>
            <span className="oai-label" id="oai-model-label">{TEXT.oai_model}</span>
            {disc.ready && !disc.hasModelList ? <input className="dx-in" type="text" spellcheck={false} autoComplete="off" value={model} placeholder="model name" aria-labelledby="oai-model-label" onInput={e => setModel(e.currentTarget.value.trim())} aria-invalid={!validModel(model)} /> : <DxSelect label={TEXT.oai_model} value={model} options={models} onChange={setModel} disabled={locked} placeholder={TEXT.oai_connect_first_models} />}
            {disc.ready && !disc.hasModelList && <span className="oai-note">{TEXT.oai_no_model_list}</span>}
          </div>

          <div className={`oai-field${locked ? ' oai-locked' : ''}`}>
            <span className="oai-label" id="oai-voice-label">{TEXT.oai_voice}</span>
            {disc.ready && !disc.voices.length ? <input className="dx-in" type="text" spellcheck={false} autoComplete="off" value={voice} placeholder="voice name" aria-labelledby="oai-voice-label" onInput={e => setVoice(e.currentTarget.value.trim())} aria-invalid={!validVoice(voice)} /> : <DxSelect label={TEXT.oai_voice} value={voice} options={voices} onChange={setVoice} disabled={locked} placeholder={TEXT.oai_connect_first_voices} />}
            {disc.ready && !disc.voices.length && <span className="oai-note">{TEXT.oai_no_voice_list}</span>}
            {disc.ready && disc.serverVoices > 0 && <span className="oai-note">{TEXT.oai_server_voices.replace('%d', String(disc.serverVoices))}</span>}
          </div>

          {enabled && !canConnect(base, model, hasKey) && <p className="oai-note oai-warn">{TEXT.oai_needs_key}</p>}

          <div className="dx-pwc-actions oai-actions">
            <button type="button" className="dx-btn solid" disabled={busy || !saveOk} onClick={save} title={dirty && !saveOk && !disc.ready ? TEXT.oai_save_connect_first : undefined}>{busy ? TEXT.oai_saving : TEXT.oai_save}</button>
            <button type="button" className="dx-btn" disabled={dirty || !saved.configured || testMine && t.st === 'running' || saved.state !== 'idle' && !testMine} onClick={test} title={dirty ? TEXT.oai_test_save_first : undefined}>
              {testMine && t.st === 'running' ? TEXT.oai_testing : TEXT.oai_test}
            </button>
          </div>
          {dirty && !saveOk && !disc.ready && <p className="oai-note">{TEXT.oai_save_connect_first}</p>}
          {testMine && !dirty && t.st !== 'running' && t.st !== 'idle' && <p className={t.st === 'ok' ? 'oai-note oai-ok' : 'oai-note oai-bad'} role="status">{t.msg}</p>}
          {err && <p className="dx-err" role="alert">{err}</p>}
        </div>
      </DxCard>

      <DxCard title={TEXT.oai_status_card} collapsible defaultOpen={true}>
        <DxFacts>
          <DxFact label={TEXT.oai_status} value={stateLabel(saved.state, saved.phase)} />
          <DxFact label={TEXT.oai_in_use} value={saved.enabled && saved.configured ? TEXT.oai_in_use_yes : TEXT.oai_in_use_no} />
          <DxFact label={TEXT.oai_protocol} value={protocolLabel(saved.proto)} hint={HINTS.oai_protocol} />
          {saved.err && <DxFact label={TEXT.oai_last_error} value={saved.err} tone="err" />}
        </DxFacts>
      </DxCard>
    </>;
}
