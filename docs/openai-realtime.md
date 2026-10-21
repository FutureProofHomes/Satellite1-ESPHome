# OpenAI Realtime conversations

The Satellite1 can hold a live, full-duplex voice conversation with an OpenAI Realtime model,
straight from the device over a WebSocket, as an alternative to the Home Assistant Assist
pipeline. It uses the same microphones, wake words, action button and LED ring. Device control
still goes through Home Assistant: the model is given one `home_assistant` tool that hands the
request to Assist.

It works with OpenAI and with any server that speaks the Realtime protocol, including a
self-hosted one on the local network.

- Component: `esphome/components/openai_realtime/`
- Package: `config/common/openai_realtime.yaml` (optional, off by default)
- Tests: `tests/openai_realtime/` (host)

## Turning it on

The connection is configured in YAML at build time. In your device YAML:

```yaml
substitutions:
  openai_api_key: !secret openai_api_key      # not needed for a local server without auth
  # openai_base_url: http://192.168.1.20:8000/v1
  # openai_model: gpt-realtime-2.1
  # openai_voice: marin

packages:
  openai_realtime: !include common/openai_realtime.yaml
```

or uncomment the `openai_realtime` line under OPTIONAL COMPONENTS in `satellite1.base.yaml`.

| Substitution | Default | Meaning |
| --- | --- | --- |
| `openai_base_url` | `https://api.openai.com/v1` | The API root, in the OpenAI SDK's `baseURL` convention (the same value an OpenAI SDK would take). The device connects to `<base_url>/realtime?model=<model>`: `http://` becomes `ws://`, `https://` becomes `wss://`. A `ws://` or `wss://` URL works too, with or without `/realtime`; query parameters a server needs (`api-version=`) are kept. |
| `openai_api_key` | empty | Sent as `Authorization: Bearer <key>`. Required for api.openai.com (the build fails without it); optional for other servers. |
| `openai_model` | `gpt-realtime-2.1` | Letters, digits and `. _ : / -`, up to 63. |
| `openai_ca_certificate` | empty | For an `https://` server whose certificate is not from a public authority (self-signed or a private CA): that certificate, or the CA's, in PEM. See [Local and compatible servers](#local-and-compatible-servers). |
| `openai_verify_certificate_name` | `true` | `false` accepts that certificate whatever host name or IP address it was issued for. |
| `openai_realtime_restore_mode` | `RESTORE_DEFAULT_ON` | The "Use OpenAI Realtime" switch's state on first boot; afterwards it keeps its last state. `RESTORE_DEFAULT_OFF` starts on Assist. |
| `openai_full_duplex_restore_mode` | `RESTORE_DEFAULT_ON` | The "OpenAI Realtime full duplex" switch's first-boot state. `RESTORE_DEFAULT_OFF` starts turn-based. |
| `openai_voice` | `marin` | Letters, digits and `. _ : / + -`, up to 63. For OpenAI: `marin` and `cedar` sound best; `alloy`, `ash`, `ballad`, `coral`, `echo`, `sage`, `shimmer`, `verse` also exist. OpenAI custom voices (`voice_…` ids) are sent in the `{"id": …}` form they require. |

## Switching between Realtime and Assist

The package adds a **Use OpenAI Realtime** switch (a configuration entity, also in Home Assistant):

- **On:** the wake word and the action button start a Realtime conversation.
- **Off:** they start the Home Assistant Assist pipeline, exactly as without the package.

Both engines are built in, so the switch takes effect at the next wake word or button press with
no reboot; a conversation already running finishes normally. The wake word or the button still
stops whichever one is running. The switch remembers its state across reboots; it starts on
(`openai_realtime_restore_mode` changes that). Automations can flip it like any switch, e.g. to use
Realtime only during the day.

## Full duplex or turn-based

The package also adds an **OpenAI Realtime full duplex** switch (configuration entity, also in
Home Assistant):

- **On** (default): full duplex - talk over the model to interrupt it.
- **Off:** turn-based, as the Assist pipeline works - useful when a setup hears itself (an external
  speaker on the line-out far from the device) or when you prefer strict turns.

Both still use the Realtime API; the mode only changes how the microphone and speaker take turns.
In turn-based mode the device sends `interrupt_response: false`, so the server never cancels an
answer itself, and it sends microphone audio only during your turn. A conversation keeps the mode
it started with; the switch applies from the next one. It remembers its state across reboots
(`openai_full_duplex_restore_mode` sets the first-boot state). From an automation or YAML:
`openai_realtime.set_full_duplex: { full_duplex: false }`.

Without the package nothing changes: the component is not even compiled, and the
`openai_realtime_selected` / `openai_realtime_start` / `openai_realtime_stop` hooks in
`voice_assistant.yaml` stay empty, so the wake word and the button start the Assist pipeline.

## Local and compatible servers

- **Address.** Both spellings work: `http://host:port/v1` and `ws://host:port/v1` both give a
  plain `ws://` connection, `https://` and `wss://` an encrypted `wss://` one. Match what the
  server listens for: a server on `https://` needs `https://` (or `wss://`) here.
- **Certificates.** `https://` is accepted out of the box only for certificates from a public
  authority (the device carries the usual root bundle). For a self-signed certificate or a private
  CA, give that certificate in PEM - the server's own for a self-signed one, the CA's otherwise:

  ```yaml
  # secrets.yaml
  local_server_cert: |-
    -----BEGIN CERTIFICATE-----
    MIIB...
    -----END CERTIFICATE-----
  # device YAML, substitutions:
  openai_ca_certificate: !secret local_server_cert
  ```

  The certificate must name the address in the base URL (for `https://192.168.0.100:5015/v1`, a
  subjectAltName of `IP:192.168.0.100`), or set `openai_verify_certificate_name: "false"`. On the
  server: `openssl s_client -connect 192.168.0.100:5015 -showcerts` prints the certificate it
  presents. On a trusted network plain `http://` avoids all of this.
- **Protocol.** There are two Realtime wire formats in use: the GA format api.openai.com speaks
  (`session.type: "realtime"`, nested `audio.input/output`, `response.output_audio.*`) and the
  earlier beta format most self-hosted servers still speak (flat session with
  `input_audio_format: "pcm16"`, `response.audio.*`). The device reads which one from the server's
  `session.created`, sends the matching `session.update`, and understands both spellings of every
  server event. A server that sends no `session.created` gets GA; set `api_version: beta` on the
  component for one that needs beta without saying so (that also sends the
  `OpenAI-Beta: realtime=v1` header OpenAI requires for beta).
- **What the device relies on.** It works best with a server that:
  - answers `session.update` with `session.updated` (otherwise the session is taken as configured
    after 1.5 s, which delays the first answer by that much);
  - sends `response.created` before an answer and `response.done` after it (without
    `response.created` the first audio delta marks the start; without `response.done` the answer
    counts as finished 5 s after its last event);
  - **keeps reading the WebSocket at all times**, including while it streams or paces an answer
    and while it handles an interruption. A server that stops reading closes its TCP window, and
    the device can then send nothing - not even the audio that would interrupt it. The device rides
    out a pause of up to 10 s (and drops the microphone audio that went stale meanwhile, with a
    `The server did not read for … ms` warning); a longer one ends the session with
    `connect: The connection to the Realtime server failed`;
  - honours `turn_detection` from `session.update`. Turn-based mode sends `interrupt_response:
    false`; a server that ignores it may still cut its own answer short on noise. The device's
    default is `semantic_vad`; for a server that only knows `server_vad`, add to the device YAML:

    ```yaml
    openai_realtime:
      turn_detection:
        type: server_vad
    ```

## During a conversation

| What happens | LED ring phase |
| --- | --- |
| Session connecting, then waiting for you | waiting for command |
| Server VAD hears you start | listening |
| You stop; the model is working | thinking |
| The model's audio is playing | replying |

- **Full duplex and barge-in** (the default). The microphone stays open while the device speaks
  (the XMOS echo canceller keeps its own voice out). Talk over the model and it stops at once; the
  part of the answer you did not hear is truncated from the conversation
  (`conversation.item.truncate`), so "what was that last thing?" refers to what you actually heard.
- **Turn-based** (see below). Like the Assist pipeline: the microphone closes when you stop
  talking, stays closed while the model thinks and answers, and reopens 500 ms after the answer has
  played (so the speaker's tail is not taken for your next turn). The answer always plays out;
  only the wake word or the button interrupts it.
- **Ending.** Say goodbye (the model calls `end_conversation` after its farewell), say the wake word
  again, press the action button, flip the mute switch, or stay silent for `idle_timeout` (20 s).
  `max_duration` (15 min) is a hard cap; OpenAI's own limit is 60 min.
- **Home control.** "Turn off the kitchen lights" makes the model call `home_assistant`, which runs
  `conversation.process` in Home Assistant and returns its whole answer; the model words the
  outcome. This needs *Allow the device to perform Home Assistant actions* on the ESPHome
  integration entry. Without a Home Assistant connection the tool answers with an error and the
  model says so.
- **Media** keeps playing, ducked by 20 dB, as during an Assist response.

## How it works

```
sat1_mics ch0 (16 kHz, XMOS AEC) -> 3/2 polyphase upsampler -> mic ring (256 KB)
   -> worker task: input_audio_buffer.append every 60 ms ------------------> WebSocket
WebSocket -> ws task: response.(output_)audio.delta -> base64 in place -> playback ring
   -> main loop -> realtime_resampling_speaker (24->48 kHz) -> realtime_mixing_input -> mixer -> I2S
```

- **Microphone task**: upsamples and writes the mic ring (the oldest audio is dropped if the uplink
  stalls). About 5 s are buffered, so words spoken while the session connects are kept.
- **Worker task** (`oai_rt`): owns the socket - connects, waits for `session.created`, sends
  `session.update`, drains queued control messages (truncate, function results,
  `response.create`) ahead of audio, sends audio, and closes the client.
- **WebSocket task** (the client's own): reassembles fragmented frames into a 512 KB PSRAM buffer,
  fast-paths audio deltas without a JSON parse, decides barge-in (so the next delta of the
  interrupted item is already discarded), and hands every other event to the loop through a queue.
- **Main loop**: triggers, phases, feeding the speaker, timeouts, truncation.

The truncation estimate is "bytes the speaker chain accepted, bounded by wall-clock playback time,
minus `playback_latency`" (700 ms in the package: resampler 300 ms + mixer input 100 ms + I2S
500 ms, usually about two thirds full).

Every JSON message is built with `esphome::json::build_json` from the `fill_*` functions in
`rt_messages.cpp`; only the 60 ms microphone frame is written by hand. Free text is filtered for
control characters ArduinoJson would write raw (all of 0x00-0x1F but its six named escapes), and
YAML tool definitions containing them are refused at build time.

## YAML reference

```yaml
openai_realtime:
  id: oai_rt
  base_url: https://api.openai.com/v1  # see "Turning it on"
  api_key: !secret openai_api_key
  model: gpt-realtime-2.1
  voice: marin
  microphone: { microphone: sat1_mics, channels: 0 }   # 16 kHz source, required
  speaker: realtime_resampling_speaker                 # takes 24 kHz mono PCM16, required
  api_version: auto                    # auto | ga | beta - wire format, normally detected
  instructions: "..."
  turn_detection:
    type: server_vad                 # semantic_vad | server_vad | none
    eagerness: auto                    # semantic_vad: auto | low | medium | high
    threshold: 0.2                     # server_vad
    prefix_padding: 300ms              # server_vad
    silence_duration: 500ms            # server_vad
  noise_reduction: none                # none | near_field | far_field (XMOS already cleans ch0)
  input_transcription_model: ""        # e.g. gpt-4o-mini-transcribe, for on_user_transcript
  tools:                               # function tools; answered from on_function_call
    - name: home_assistant
      description: "..."
      parameters: { type: object, properties: { command: { type: string } }, required: [command] }
  end_conversation_tool: true
  full_duplex: true                    # default mode; false = turn-based (the switch overrides it)
  idle_timeout: 20s
  max_duration: 15min
  playback_latency: 600ms
  playback_buffer_size: 786432         # bytes of 24 kHz audio buffered ahead (~16 s)
  extra_headers: {}                    # sent on the WebSocket upgrade (e.g. Azure's api-key)
  ca_certificate: ""                   # PEM, for a wss:// server with its own certificate
  verify_certificate_name: true        # false: ignore the name/IP the certificate was issued for
  muted: !lambda return id(master_mute_switch).state;
  # triggers: on_start, on_ready, on_listening, on_speech_started, on_speech_stopped,
  # on_response_started, on_response_finished, on_interrupted, on_end,
  # on_error (code, message), on_user_transcript (x), on_assistant_transcript (x),
  # on_function_call (name, arguments, call_id)
```

Actions: `openai_realtime.start`, `openai_realtime.stop`,
`openai_realtime.send_function_result` (`call_id`, `output`, `respond: true`),
`openai_realtime.send_text` (`text`), `openai_realtime.set_full_duplex` (`full_duplex`).
Condition: `openai_realtime.is_active`.

Every function call must be answered with `send_function_result`; the model waits for it. With no
`on_function_call` automation the component answers each call with an error itself.

## Build notes

- Adds `espressif/esp_websocket_client` 1.6.0 from the component registry, re-includes the
  `esp-tls` and `tcp_transport` IDF components (excluded by default in ESPHome 2026.9), and
  requests the certificate bundle.
- PSRAM per session: ~512 KB receive buffer + 768 KB playback ring + small buffers, released when
  the session ends; the 256 KB mic ring and the worker's 8 KB stack are kept once allocated (the
  build sets `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY`). Internal RAM, only during a session: the
  client's 6 KB task stack and its two 4 KB frame buffers. Internal RAM is tight on this board
  (plain `malloc` does not spill into PSRAM), so these are kept small; the log shows the free
  internal RAM at each start (`Internal RAM: … free, largest block …`).
- Dashboard builds pull external components from the `staging` branch, so they need this merged
  there first.

## Testing

```
g++ -std=c++17 -O2 -Wall -I esphome/components/openai_realtime \
    tests/openai_realtime/test_rt_util.cpp esphome/components/openai_realtime/rt_util.cpp -o /tmp/t && /tmp/t
```

`test_rt_messages.cpp` covers every JSON message the device builds; see `tests/README.md` for the
ArduinoJson checkout it needs.

On hardware, worth checking first: barge-in while the device speaks loudly (residual echo would
show as the model interrupting itself - set `turn_detection: { eagerness: low }`, or switch to
turn-based), and `playback_latency` against what you actually heard when interrupted.
