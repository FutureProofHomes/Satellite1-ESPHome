# Web UI copy

Every explanatory string the on-device web app shows, so support can quote the device's own wording
back to a customer without a device in front of them, and so the wording can be reviewed by someone
who does not write Preact.

This mirrors
[`esphome/components/satellite1_web_ui/frontend/src/copy.js`](../esphome/components/satellite1_web_ui/frontend/src/copy.js),
which is the source of truth — the app reads that file, not this one. Change both together.

## House style

Say what the control does to the device, not what the widget is. A hint that reads "toggles the
microphone mute state" has told the reader nothing they could not see. Prefer the concrete failure
mode over the abstraction: "the television sets it off" lands where "reduces false positives" does
not.

Explain the surprising thing. Most of these strings exist because a reading or a control looks wrong
until you know one fact about the hardware — the temperature sensor sits in a warm enclosure, the
mute is in silicon rather than software, PSRAM's total is smaller than the chip.

## Hints

Attached to the small **i** beside a label. One is open at a time.

### Controls

| Key | Where | Text |
| --- | --- | --- |
| `temp` | Temperature pill | Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room and the offset is stored on the device. |
| `humidity` | Humidity pill | Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room. |
| `lux` | Ambient light pill | Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger. |
| `presence` | Presence pill | From the mmWave radar, not the microphones. It sees through the enclosure and does not need line of sight, so it reports an empty room faster than a motion sensor would. |
| `calibrate` | Calibration editor | Enter what a trusted instrument in the same room reads. The difference is saved as an offset and survives a restart. It does not change the raw reading, only what the device reports. |
| `mute` | Mute microphones | Cuts the microphones in hardware, on the XMOS chip, not in software. Wake word detection stops with them. The mute button on the device does the same thing. |
| `wake_sound` | Wake chime | Plays a short chime on the speaker the moment the wake word is detected. |
| `wake_sensitivity` | Wake word sensitivity | How readily the wake word fires. Raise it if the device misses you from across the room; lower it if the television sets it off. |
| `voice_override` | Assistant volume | Speaker volume used for assistant replies only, independent of media volume. Set to zero to follow the media volume instead. |
| `speaker_channel` | Speaker channel | Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want. |
| `led_ring` | LED ring | The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error. |
| `timers` | Timers card | Timers set by voice, held on the device. They keep counting and still ring if Home Assistant goes away. |

### Diagnostics

| Key | Where | Text |
| --- | --- | --- |
| `heap` | Internal RAM free | Internal RAM still available. This is the number that matters: it is what audio buffers and the network stack allocate from, and it is far scarcer than PSRAM. |
| `psram` | PSRAM free | External RAM, used for the large buffers. Installed is the size of the chip; the free and total figures cover the region handed to the allocator, which is smaller because the cache and early allocations sit outside it. |
| `loop` | Longest loop | Longest single pass through the main loop since this panel last read the value. Tens of milliseconds is normal. Sustained hundreds means something is blocking, and audio will stutter before anything else does. |
| `reset` | Last restart | Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware. |
| `xmos` | Audio chip | The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's. |
| `xmos_flash` | Reflash its firmware | Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power. |
| `xmos_erase` | Erase its firmware | Erases the audio chip's firmware. It will not process audio again until you flash it. Only useful when a flash has gone wrong and the chip will not accept a new image. |
| `safe_mode` | Safe mode | Restarts with everything but wifi and the updater switched off. Use it when the device is crash-looping and will not stay up long enough to accept an update. |
| `factory_reset` | Factory reset | Erases every setting stored on the ESP32, including the wifi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched. |
| `beta` | Pre-release firmware | Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on. |
| `log` | Log | The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request. |

### Config

Two controls here still write to entities in `config/common/tts_routing.yaml` and
`config/common/area_ducking.yaml`, so their wording has to agree with what Home Assistant shows. The
two trees do not: they write the device's own selection at `/api/sat1/sel`, and they are the only face
that selection has.

| Key | Where | Text |
| --- | --- | --- |
| `remote_tts_volume` | Remote TTS volume | How loud the answer is on the remote speakers. It does not touch this device's own level - that is Voice Override, on the Audio card. |
| `remote_wake_chime` | Remote wake chime | Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to. |
| `duck_volume` | Duck volume | The level everything you have chosen drops to while the assistant is listening and answering. Players already quieter than this are left alone, so a whole-house group does not get turned up. |

Six hints were deleted rather than reworded, and the reason is worth recording: they explained
switches that no longer exist (`tts_routing`, `tts_local_speaker`, `duck_area`, `duck_tts_targets`) or
described the tree in prose when the tree now shows the same thing directly (`tts_targets`,
`duck_players`). `tts_manual_ids` went with the free-text entity id field it belonged to, replaced by
the "No Area Assigned" group, which names those players instead of asking someone to know their ids.

The trees carry no ⓘ hints at all. A list of rooms with checkboxes and `Local Speaker` at the top of it
is the explanation; a tooltip on top of that would be describing a control the reader is already
looking at.

## Standing text

The `ha_*` block below is the Home Assistant data layer explaining its own absence. Each string is a
different cause with a different fix, which is why they are not collapsed into one "unavailable"
message: `ha_pending` is a five-second wait on a fresh boot rather than a fault, `ha_refused` is the
only one that asks the customer to change a setting, and the last three are properties of their Home
Assistant rather than of the device. `sel_failed` is not one of them: it is the device refusing a
write, and it is shown as a banner rather than a silent revert because the checkbox has already moved
back, which on its own looks like a page that ignores clicks.

| Key | Text |
| --- | --- |
| `ha_connected` | Connected to Home Assistant |
| `ha_disconnected` | Not connected to Home Assistant |
| `ha_disconnected_detail` | Everything on this page still works - it talks to the device directly. Media and anything that needs your smart home will be unavailable until the connection returns. |
| `stream_lost` | Lost the connection to the device. Retrying. |
| `no_devices` | Only this device. Other Satellite1s on your network will appear here once they are running firmware with this web app. |
| `confirm` | Confirm |
| `cancel` | Cancel |
| `copied` | Copied |
| `ha_pending` | Asking Home Assistant which speakers you have. |
| `ha_never` | Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns. |
| `ha_refused` | Home Assistant did not answer. Either it is older than 2025.12, or this device is not allowed to perform actions: Settings › Devices & services › ESPHome › this device › CONFIGURE, then tick "Allow the device to perform Home Assistant actions". |
| `ha_no_area` | This device is not in a Home Assistant area, so "Route TTS To All Area Players" and "Duck All Area Players" have no room to refer to. You can still pick any room below. Assign it to an area in Home Assistant and refresh. |
| `ha_no_players` | Home Assistant has no media players at all, so there is nothing to choose between. |
| `ha_truncated` | Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below. |
| `sel_failed` | That change was not saved. The device rejected it, or the connection dropped. |
| `ha_refresh` | Refresh |
| `ha_refreshing` | Asking… |

## Strings that are not in `copy.js`

A few short labels sit inline in the components because they are part of the layout rather than
explanations, and lifting them out would make the markup harder to read than it makes them easier to
edit:

- The calibration editor's three row labels — `sensor reads`, `offset`, `shows` — and its `Done`.
- `Tap a reading to calibrate it. Presence opens the radar tuner.` under the sensor pills.
- `Tap the wheel to set a colour. Hue around the edge, whiter towards the middle.` under the wheel.
- `Press a button on the device; it lights up here.` under the Buttons card.
- `Waiting for the device to say something.` in an empty log.
- The confirm-button verbs: `Reflash now`, `Erase it`, `Restart into safe mode`, `Erase everything`.
- The Remote TTS volume disclosure, which is a paragraph rather than a hint because it is three
  sentences and belongs on screen rather than behind an ⓘ.
- `Local Speaker`, `No Area Assigned`, `whole area` and the `n/m` counts in the two trees. All four are
  structure rather than explanation - the first two are row labels and the last two are state readouts.
- `Play responses on` and `Quieten while talking`, the two tree headings.
- `From Home Assistant Ns ago`, next to the Refresh button.

The voice assistant phase names — Idle, Waiting for a command, Listening, Thinking, Replying, Not
ready, Error — are in `frontend/src/lib/device.js`, keyed by the `voice_assist_*_phase_id`
substitutions in `config/common/voice_assistant.yaml`. They have to stay aligned with those numbers.
