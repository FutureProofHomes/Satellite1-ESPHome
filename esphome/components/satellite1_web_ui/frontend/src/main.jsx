/*
 * Phase 2 step one: the probe.
 *
 * This is deliberately not the app. It exists to answer, in one flash, the five things the rest
 * of Phase 2 is built on top of and that cannot be verified from the source alone:
 *
 *   1. Does our handler win "/" ahead of web_server, or does the ESPHome dashboard still show?
 *   2. Does our handler sit behind digest auth like every other one?
 *   3. Does EventSource authenticate at all? It cannot set an Authorization header itself, so it
 *      depends on the browser answering the 401 challenge for it. check_digest_auth is stateless
 *      - the nonce it issues is never tracked - so nothing should expire under a long-lived
 *      stream, but "should" is why this page exists.
 *   4. Does the codegen gzip -> progmem_array path serve a document the browser accepts?
 *   5. Does the ETag round trip work, given that ESPHome's response API cannot express 304 and
 *      the handler has to reach past it to httpd_resp_set_status?
 *
 * Replaced by the real shell once those come back green.
 */
// app.css is a separate esbuild entry point that build.mjs inlines into the document, so it is
// deliberately not imported here - that would pull it through the JS bundle instead.
import { render } from "preact";
import { useEffect, useState } from "preact/hooks";

/** Strip the ANSI colour runs the logger writes into every message before the "[D][tag:line]". */
const stripAnsi = (s) => s.replace(/\u001b\[[0-9;]*m/g, "");

function Probe() {
  const [state, setState] = useState(null);
  const [stateErr, setStateErr] = useState(null);
  const [sse, setSse] = useState("connecting");
  const [ids, setIds] = useState([]);
  const [logs, setLogs] = useState([]);
  const [etag, setEtag] = useState("not tried");
  const [uptime, setUptime] = useState(null);

  useEffect(() => {
    fetch("/api/sat1/state")
      .then((r) => (r.ok ? r.json() : Promise.reject(new Error(`HTTP ${r.status}`))))
      .then(setState)
      .catch((e) => setStateErr(String(e)));

    const es = new EventSource("/events");
    es.onopen = () => setSse("open");
    es.onerror = () => setSse((s) => (s === "open" ? "dropped, retrying" : "failed"));
    es.addEventListener("state", (e) => {
      const d = JSON.parse(e.data);
      setIds((prev) => (prev.includes(d.id) ? prev : [...prev, d.id].sort()));
    });
    es.addEventListener("ping", (e) => setUptime(JSON.parse(e.data).uptime));
    es.addEventListener("log", (e) => setLogs((l) => [...l.slice(-19), stripAnsi(e.data)]));
    return () => es.close();
  }, []);

  /*
   * An explicit conditional request rather than a reload: fetch surfaces 304 as a real status
   * when the app sets If-None-Match itself, where a reload would be answered from cache and
   * prove nothing. The ETag comes off an unconditional HEAD-equivalent first.
   */
  const tryEtag = async () => {
    setEtag("checking");
    const first = await fetch("/", { cache: "no-store" });
    const tag = first.headers.get("ETag");
    if (!tag) {
      setEtag("no ETag header on /");
      return;
    }
    const second = await fetch("/", { cache: "no-store", headers: { "If-None-Match": tag } });
    setEtag(second.status === 304 ? `304 on ${tag} - correct` : `${second.status} on ${tag} - wrong, wanted 304`);
  };

  return (
    <div class="wrap">
      <div class="card">
        <h2>satellite1_web_ui probe</h2>
        <div>
          <p class="dim" style="margin-top:0">
            You are reading this from PROGMEM inside the external component, which means our handler
            won <code>/</code> ahead of <code>web_server</code> and digest auth let you through.
          </p>
          <Line k="GET /api/sat1/state" v={stateErr ?? (state ? "ok" : "…")} bad={!!stateErr} />
          <Line k="/events" v={sse} bad={sse === "failed"} />
          <Line k="entities seen on /events" v={String(ids.length)} />
          <Line k="ping uptime" v={uptime == null ? "none yet" : `${uptime} s`} />
          <div class="row">
            <span class="dim">ETag / 304</span>
            <span class="grow" />
            <span class="mono">{etag}</span>
            <button onClick={tryEtag}>check</button>
          </div>
        </div>
      </div>

      {state && (
        <div class="card">
          <h2>/api/sat1/state</h2>
          <div>
            <pre class="mono" style="margin:0;white-space:pre-wrap;word-break:break-all;font-size:11px">
              {JSON.stringify(state, null, 1)}
            </pre>
          </div>
        </div>
      )}

      <div class="card">
        <h2>
          Entities <span class="dim">{ids.length}</span>
        </h2>
        <div class="mono" style="font-size:11px;max-height:220px;overflow:auto">
          {ids.map((id) => (
            <div key={id}>{id}</div>
          ))}
        </div>
      </div>

      <div class="card">
        <h2>Log</h2>
        <div
          class="mono"
          style="font-size:10.5px;max-height:200px;overflow:auto;background:var(--bg-sunk);white-space:pre-wrap;word-break:break-all"
        >
          {logs.length === 0 ? <span class="dim">nothing yet</span> : logs.map((l, i) => <div key={i}>{l}</div>)}
        </div>
      </div>
    </div>
  );
}

function Line({ k, v, bad }) {
  return (
    <div class="row">
      <span class="dim">{k}</span>
      <span class="grow" />
      <span class="mono" style={bad ? "color:var(--err)" : ""}>
        {v}
      </span>
    </div>
  );
}

render(<Probe />, document.getElementById("app"));
