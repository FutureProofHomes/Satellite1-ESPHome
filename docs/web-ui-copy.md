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

## Standing text

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

The voice assistant phase names — Idle, Waiting for a command, Listening, Thinking, Replying, Not
ready, Error — are in `frontend/src/lib/device.js`, keyed by the `voice_assist_*_phase_id`
substitutions in `config/common/voice_assistant.yaml`. They have to stay aligned with those numbers.
