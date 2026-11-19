# Speech-to-text wake check (`wake_verify`)

A second opinion on every wake word detection. When microWakeWord fires, the device sends the last 1.8 s of microphone
audio to Home Assistant's speech-to-text API and checks that the transcript actually contains a wake word. A false
fire from TV dialogue is dropped without a sound, and a wake that the wrong model caught goes ahead with the word that
was really said.

It runs entirely in the firmware (`esphome/components/wake_verify/`, wired up in `config/common/wake_verify.yaml`).
There is no Home Assistant add-on or custom integration: the device calls Home Assistant's built-in
`/api/stt/<engine>` REST endpoint with a long-lived access token, using the same speech-to-text engines the Assist
pipelines use.

**It is off by default.** With the mode `off`, no audio is buffered and every wake goes ahead at once, exactly as
without the check.

## How a wake goes through it

1. **Detection.** micro_wake_word fires and `on_wake_word_detected` runs. The loader's detection record, the sign-in
   and test-window gates and the web app's detection ring all run first, so they see every firing, including ones the
   check will reject. A ringing timer and the mute switch are handled there at once. Everything else is handed to
   `wake_verify.check`.
2. **Skip list.** A word in `skip_wake_words` (Stop) passes straight through, so Stop always acts instantly.
3. **Recording.** The component keeps a rolling 1.8 s buffer (in PSRAM) of the processed channel micro_wake_word scores,
   at the same gain factor. After the detection it waits a further 200 ms (`post_roll`) so the end of the word is in
   the clip.
4. **Level.** A quiet clip is boosted to a −6 dBFS peak, by at most 32×. The Satellite1's clips peak around 200–500
   (out of 32768), so they usually get the full 32×.
5. **Transcription.** The clip goes to `POST <ha_url>/api/stt/<stt_engine>` as 16 kHz 16-bit mono PCM, from a separate
   task so the main loop keeps running.
6. **Matching.** The transcript is normalised (lower case, punctuation dropped, "ok" read as "okay", and "okay now boo"
   read as "okay nabu", along with "who", "wu", "lou" and a few other second halves speech-to-text writes for "Nabu"),
   and every run of neighbouring words is compared with each enabled wake word, spaces removed, by edit distance. The
   score is 1 − distance / length, so "Okay, Navu." scores 0.88 against Okay Nabu.
7. **Verdict**, with `match_threshold` 0.65:
   - **ACCEPT**: the detected word scores at least 0.65. The wake goes ahead.
   - **REROUTE**: the detected word doesn't, but another enabled wake word does. The wake goes ahead with that word,
     so Home Assistant runs that word's pipeline.
   - **REJECT**: no enabled wake word matches. Nothing happens: no chime, no stop.
8. **Action.** The `start_wake` script in `config/common/voice_assistant.yaml` runs with the verdict's word. It holds
   what `on_wake_word_detected` used to do directly: if the assistant is running it is stopped (with the remote stop
   fan-out); if an announcement is playing it is stopped (with the broadcast stop); otherwise the remote wake chime,
   the finished-speaking-detection hand-off, the wake sound and `voice_assistant.start` run.

"Enabled" means the words in the device's two wake word slots at that moment. A model compiled in but not in a slot is
neither a match nor a reroute target.

### Two models on one utterance

Two models can fire on the same speech. A detection that arrives while another is waiting for its verdict joins that
check instead of starting its own: the check already weighs every enabled word, so the wake goes ahead once, with the
word that was said. The log shows `'<word>' joins the check of '<word>'`.

### Stops

A wake word heard while the assistant is listening or replying, or while an announcement plays, normally stops it.
That stop also goes through the check, so a false fire in the middle of a conversation no longer cuts it off. Stop
itself skips the check, and a ringing timer is silenced by any wake word without it.

### When the check can't run

The check never makes the device deaf. The detected word goes ahead unchecked if:

- the mode is `off`, or there is no token;
- speech-to-text fails or returns an error (`'<word>' not checked, waking anyway: …`);
- no verdict arrives within `decision_timeout` (1.5 s) (`No verdict … waking unchecked`);
- a new detection arrives while an earlier check is still running but no longer waiting (log mode, or a check that has
  already timed out).

So while Home Assistant or the speech-to-text engine is down, false fires wake the device as they would without the
check.

## Modes

| Mode | Behaviour |
|---|---|
| `off` (default) | No check and no audio buffer. `on_wake` fires at once with the detected word. |
| `log` | `on_wake` fires at once with the detected word; the check runs in the background and is only logged, as `WOULD ACCEPT`, `WOULD REROUTE` or `WOULD REJECT`. A good way to see what it would do before relying on it. |
| `reroute` | The wake waits for the verdict, as described above. |

## Setup

1. **Token.** In Home Assistant, open your profile → Security → Long-lived access tokens, create one, and add it to
   `secrets.yaml`, for example as `ha_token`.
2. **Engine.** Pick the speech-to-text entity to use (Settings → Voice assistants shows the ones you have, e.g.
   `stt.faster_whisper`). A fast local engine keeps the added latency low.
3. **Substitutions.** In your device file:

   ```yaml
   substitutions:
     wake_verify_mode: log            # try log first, then reroute
     wake_verify_ha_url: http://homeassistant.local:8123
     wake_verify_ha_token: !secret ha_token
     wake_verify_stt_engine: stt.faster_whisper
   ```

4. Flash, and watch the log for the lines below. Once the `WOULD …` verdicts look right, switch to `reroute`.

The component's other options (clip length, post-roll, timeouts, threshold, skip list, language) are listed in the
docstring of `esphome/components/wake_verify/__init__.py`.

## Latency

About 0.2 s plus the speech-to-text round trip per wake in `reroute` mode. With a local OpenAI-compatible Whisper
engine the round trip measured 130–340 ms (median about 185 ms), so a wake is roughly 0.4 s later. Stopping a reply
with a wake word takes the same extra time; Stop does not.

## Reading the log

Each check logs one line at INFO:

```
ACCEPT 'Okay Nabu' score=1.00 heard='Okay, Nabu.' (235 ms, 1.8s clip, peak 696 x23.5) [1 checked: 0 rerouted, 0 rejected]
REJECT 'Hey Jarvis' score=0.33 heard='The older approach is probably.' (206 ms, 1.8s clip, peak 436 x32.0) [...]
```

- `score`: how well the **detected** word matched the transcript.
- `heard`: the transcript.
- `(… ms)`: the speech-to-text round trip only.
- `peak N xG`: the clip's loudest sample before boosting, and the boost applied.
- The counters run since boot.

## Tuning notes

- **Threshold 0.65.** At the component default of 0.75, genuine wakes heard as "Okay, now Wu." (0.67) were rejected.
  Speech-to-text often splits "Nabu" into "now" plus another word ("Okay, now who?", 0.62). Lowering the threshold to
  0.62 would also pass ordinary "Okay, now …" speech, so instead those exact splits ("now" followed by boo, bu, who,
  wu, woo, lou, poo, vu or voo, right after "okay") are read as "Nabu" and score 1.00. "Okay, now we go." still scores
  0.62 and is rejected.
- **Lower model cutoffs.** With the check catching false fires, a model's probability cutoff can be set lower than
  its published value to catch more real calls, letting the check reject the extra false fires.
- **Speech-to-text inventions.** On noise some engines return text that was never said ("Transcribe audio to text
  (language: en).", "是的。"). These score near zero and are rejected.
