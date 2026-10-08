# OpenAI Realtime conversations

The Satellite1 can hold a live, full-duplex voice conversation with an OpenAI Realtime model,
straight from the device over a WebSocket, as an alternative to the Home Assistant Assist pipeline.
It uses the same microphones, wake words, LED ring and web UI orb. Device control still goes through
Home Assistant: the model is given one `home_assistant` tool that hands the request to Assist.

- Component: `esphome/components/openai_realtime/`
- Package: `config/common/openai_realtime.yaml` (included by `satellite1.base.yaml`)
- Web UI: **Settings > OpenAI** (`#/settings/openai`)
- Tests: `tests/openai_realtime/` (C++, host) and `frontend/test/openai.test.mjs`

## Setting it up

1. Open the device's web UI, **Settings > OpenAI**.
2. **Connect.** Enter the **base URL** - `https://api.openai.com/v1` for OpenAI, or the root of
   any compatible server, including one on your network (`http://192.168.1.20:8000/v1`) - and the
   **API key** (a local server may need none). The page shows the WebSocket URL a session will
   open. A pasted `wss://…/realtime` URL works too, and query parameters some servers need
   (`api-version=`) are kept.
   The device connects by itself as soon as the address and key look valid (and on opening the
   page, with the saved ones): it fetches `<base URL>/models` and `<base URL>/audio/voices`. The
   connection line shows the result; **Refresh lists** fetches again.
3. **Choose.** The **Model** and **Voice** pickers are locked until that connection succeeds for
   the address on screen, and then offer exactly what that server has:
   - Models: the server's Realtime models (`/models`, filtered to names containing "realtime";
     a server whose names never do shows all of them).
   - Voices: for api.openai.com, the ten built-in Realtime voices (`marin` and `cedar` sound
     best) plus any custom voices of your organisation; for other servers, their `/audio/voices`.
   - A server that has no model or voice list still connects; the page then lets you type the
     name it expects.
   - Switching to a different server preselects its first model and voice; a saved choice the
     saved server no longer offers is kept and marked "not offered by this server".
4. **Save**, then **Test connection** - a real Realtime handshake with the saved settings, no audio.
   Its result names the protocol the server spoke.
5. Turn on **Use for conversations** (saving the switch alone works even while the server is
   down; changing the address, key, model or voice needs a successful connection first).

With the switch on, the wake word, the orb's tap and the action button start a Realtime
conversation instead of the Assist pipeline. With it off nothing changes; the component idles.

The YAML values in `openai_realtime.yaml` (`base_url`, `api_key`, `model`, `voice`, `enabled`) are
only a fresh device's defaults. Anything saved in the web UI wins. A fleet can ship a key in
`secrets.yaml` via `api_key: !secret openai_api_key`; a key saved in the UI overrides it.

## Local and compatible servers

Anything that speaks the Realtime WebSocket protocol works: OpenAI, Azure OpenAI, hosted
proxies, and self-hosted servers on your network.

- **Address.** Use `http://` (or `ws://`) for a server on your network. `https://` is fine for
  servers with a certificate from a public authority (the device carries the usual root bundle),
  but a self-signed certificate is refused - there is no "trust anyway" switch, on purpose.
- **Key.** Optional for anything but api.openai.com. A key saved for one host is never sent to
  another (see below).
- **Protocol.** There are two Realtime wire formats in use: the GA format api.openai.com speaks
  today (`session.type: "realtime"`, nested `audio.input/output`, `response.output_audio.*`) and
  the earlier beta format most self-hosted servers still speak (flat session with
  `input_audio_format: "pcm16"`, `response.audio.*`). The device reads which one from the
  server's `session.created`, sends the matching `session.update`, and understands both spellings
  of every server event. A server that sends no `session.created` gets GA; set
  `api_version: beta` in YAML for one that needs beta without saying so (that also sends the
  `OpenAI-Beta: realtime=v1` header OpenAI requires for beta).
- **Confirmation.** A server that never answers `session.update` with `session.updated` is taken
  as configured after 3 s without an error, so it does not sit in "connecting".
- **Model lists.** Only top-level `data[].id` (or `models[].id|name`) entries count - nested ids
  such as vLLM's `permission[].id` are not models.
- **Voices.** `GET <base>/audio/voices` in OpenAI's shape (`data[]` with `id`/`name`) or the
  common local shapes (`voices[]` of strings, as Kokoro-FastAPI returns, or of objects with
  `id`, `voice_id` or `name`). OpenAI custom voices (`voice_…` ids) are sent as
  `{"id": "voice_…"}`, as the GA session requires; everything else as a plain string.

## During a conversation

| What happens | LED / orb phase |
| --- | --- |
| Session connecting, then waiting for you | waiting for command |
| Server VAD hears you start | listening |
| You stop; the model is working | thinking |
| The model's audio is playing | replying |

- **Barge-in.** Talk over the model and it stops at once; the part of the answer you did not hear is
  truncated from the conversation (`conversation.item.truncate`), so "what was that last thing?"
  refers to what you actually heard.
- **Ending.** Say goodbye (the model calls `end_conversation` after its farewell), say the wake word
  again, press the action button or tap the orb, flip the mute switch, or stay silent for
  `idle_timeout` (20 s). `max_duration` (15 min) is a hard cap; the server's own limit is 60 min.
- **Home control.** "Turn off the kitchen lights" makes the model call `home_assistant`, which runs
  `conversation.process` in Home Assistant and returns its whole answer; the model words the
  outcome. This needs *Allow the device to perform Home Assistant actions* on the ESPHome
  integration entry. Without a Home Assistant connection the tool answers with an error and the
  model says so.
- **Media** keeps playing, ducked, as it is during an Assist response.

## How it works

```
sat1_mics ch0 (16 kHz, XMOS AEC) -> 3/2 polyphase upsampler -> mic ring (256 KB)
   -> worker task: input_audio_buffer.append every 60 ms ------------------> WebSocket
WebSocket -> ws task: response.output_audio.delta -> base64 in place -> playback ring
   -> main loop -> realtime_resampling_speaker (24->48 kHz) -> realtime_mixing_input -> mixer -> I2S
```

Three contexts, never mixed:

- **Microphone task**: upsamples and writes the mic ring (oldest audio dropped if the uplink stalls).
- **Worker task** (`oai_rt`): owns the socket - connects, sends `session.update`, drains queued
  control messages (truncate, function results, `response.create`) ahead of audio, sends audio, and
  closes/destroys the client. Audio waits for `session.updated` (3 s grace).
- **WebSocket task** (the client's own): reassembles fragmented frames into a 512 KB PSRAM buffer,
  fast-paths audio deltas without a JSON parse, decides barge-in (so the next delta of the
  interrupted item is already discarded), and hands every other event to the loop through a queue.
- **Main loop**: triggers, phases, feeding the speaker, timeouts, truncation.

The truncation estimate is "bytes the speaker chain accepted, bounded by wall-clock playback time,
minus `playback_latency`" (700 ms in the package: resampler 300 ms + mixer input 100 ms + I2S 500 ms,
usually about two thirds full). The I2S speaker's own buffer is shared with media, so up to its
length of the interrupted answer can still be heard after a barge-in.

Settings live in NVS (`openai_realtime_settings_v2`; settings saved by the first release under
`_v1` are migrated on boot). A key saved for one host is never sent to
another: changing the base URL to a different origin without typing a key drops the stored key,
and the model-list fetch only reuses the stored key for the stored origin.

## YAML reference

```yaml
openai_realtime:
  id: oai_rt
  microphone: { microphone: sat1_mics, channels: 0 }   # 16 kHz source, required
  speaker: realtime_resampling_speaker                 # takes 24 kHz mono PCM16, required
  enabled: false                       # default for "Use for conversations"
  base_url: https://api.openai.com/v1  # default
  api_key: ""                          # default
  model: gpt-realtime-2.1              # default
  voice: marin                         # default (up to 63 characters)
  api_version: auto                    # auto | ga | beta - wire format, normally detected
  instructions: "..."
  turn_detection:
    type: semantic_vad                 # semantic_vad | server_vad | none
    eagerness: auto                    # semantic_vad: auto | low | medium | high
    threshold: 0.5                     # server_vad
    prefix_padding: 300ms              # server_vad
    silence_duration: 500ms            # server_vad
  noise_reduction: none                # none | near_field | far_field (XMOS already cleans ch0)
  input_transcription_model: ""        # e.g. gpt-4o-mini-transcribe, for on_user_transcript
  tools:                               # function tools; answered from on_function_call
    - name: home_assistant
      description: "..."
      parameters: { type: object, properties: { command: { type: string } }, required: [command] }
  end_conversation_tool: true
  half_duplex: false                   # true: do not send the mic while the model speaks
  idle_timeout: 20s
  max_duration: 15min
  playback_latency: 600ms
  playback_buffer_size: 786432         # bytes of 24 kHz audio buffered ahead (~16 s)
  extra_headers: {}                    # sent on the WebSocket and the /models request
  muted: !lambda return id(master_mute_switch).state;
  # triggers: on_start, on_ready, on_listening, on_speech_started, on_speech_stopped,
  # on_response_started, on_response_finished, on_interrupted, on_end,
  # on_error (code, message), on_user_transcript (x), on_assistant_transcript (x),
  # on_function_call (name, arguments, call_id)
```

Actions: `openai_realtime.start`, `openai_realtime.stop`,
`openai_realtime.send_function_result` (`call_id`, `output`, `respond: true`),
`openai_realtime.send_text` (`text`). Conditions: `openai_realtime.is_active`,
`openai_realtime.is_enabled` (switch on *and* configured).

Every function call must be answered with `send_function_result`; the model waits for it. With no
`on_function_call` automation the component answers each call with an error itself.

## Web API

All behind the session gate.

- `GET /api/sat1/openai` - `{enabled, configured, base_url, model, voice, key_set, key_hint,
  realtime_url, state, phase, proto, err, models: {…}, test: {gen, st, msg}}`. `proto` is `ga`,
  `beta` or empty - the format the last session or test used.
  Never the key.
- `POST /api/sat1/openai` - `{enabled, base_url, model, voice[, api_key][, clear_key]}`. No
  `api_key` keeps the stored one (unless the origin changed). 400 `{ok:0, err:<field>}` on refusal.
- `POST /api/sat1/openai/models` - `{base_url[, api_key]}` starts the background discovery
  (`/models`, then `/audio/voices`); an empty `api_key` uses the stored one only for the stored
  key's own origin. Returns `{ok:1, gen}`; the result rides the GET as
  `models: {gen, st, base, err, list, ids[], voices: [[id, label, from_server], …]}`. `st` is
  `ok` whenever the server was reachable and accepted the key - `list: false` then means it has no
  model list.
- `POST /api/sat1/openai/test` - returns `{ok:1, gen}`; the result rides the GET.
- `GET /api/sat1/state` gains `"oai": {"on", "st"}` when the component is in the build.

## Build notes

- Adds `espressif/esp_websocket_client` 1.6.0 from the component registry and re-includes the
  `esp-tls`, `tcp_transport` and `esp_http_client` IDF components (excluded by default in ESPHome
  2026.9), and requests the certificate bundle.
- PSRAM per session: ~512 KB receive buffer + 768 KB playback ring + small buffers, released when
  the session ends; the 256 KB mic ring is kept once allocated. Internal RAM: the 8 KB worker stack
  and the client's 6 KB task stack, only during a session.
- Dashboard builds pull external components from the `staging` branch, so they need this merged
  there first.

## Testing

```
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime \
    tests/openai_realtime/test_rt_util.cpp esphome/components/openai_realtime/rt_util.cpp -o /tmp/t && /tmp/t
cd esphome/components/satellite1_web_ui/frontend && npm test
```

`test_rt_messages.cpp` covers every JSON message the device builds (see `tests/README.md` for
the ArduinoJson checkout it needs). Messages are built with `esphome::json::build_json` from the
`fill_*` functions in `rt_messages.cpp`; only the 60 ms microphone frame is written by hand, and it
is tested against what ArduinoJson would produce. Free text is passed through a filter that turns
control characters ArduinoJson would write raw (all of 0x00-0x1F but its six named escapes) into
spaces, and YAML tool definitions containing them are refused at build time.

On hardware, worth checking first: barge-in while the device speaks loudly (residual echo would
show as the model interrupting itself - raise `turn_detection` eagerness to `low`, or use
`half_duplex`), and `playback_latency` against what you actually heard when interrupted.
