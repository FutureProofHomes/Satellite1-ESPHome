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
| `temp` | Calibration editor, title row | Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room. |
| `humidity` | Calibration editor, title row | Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room. |
| `lux` | Calibration editor, title row | Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger. |
| `calibrate` | Calibration editor, offset row | Enter what a trusted instrument in the same room reads. The difference is saved as an offset and survives a restart. It does not change the raw reading, only what the device reports. |
| `mute` | Mute microphones | Cuts the microphones in hardware, on the XMOS chip, not in software. Wake word detection stops with them. The mute button on the device does the same thing. |
| `wake_sound` | Wake chime | Plays a short chime on the speaker the moment the wake word is detected. |
| `wake_sensitivity` | Wake word sensitivity | How readily the wake word fires. Raise it if the device misses you from across the room; lower it if the television sets it off. |
| `voice_override` | Assistant volume | Speaker volume used for assistant replies only, independent of media volume. Set to zero to follow the media volume instead. |
| `speaker_channel` | Speaker channel | Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want. |
| `led_ring` | LED ring | The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error. |
| `timers` | Timers card | Timers set by voice, held on the device. They keep counting and still ring if Home Assistant goes away. |

The first four sit on the ⓘ inside the calibration editor, which opens by tapping a sensor chip — not on
the chips themselves. A chip is 50–80px wide, and a finger-sized ⓘ inside one is indistinguishable from a
tap meant to open the editor. The chips carry a grey chevron under the label instead, pointing down at the
editor and flipping up while it is open; the Presence chip carries an arrow, because it leaves the page
rather than opening anything.

`temp` used to end "and the offset is stored on the device", which is what `calibrate` says two rows below
it in the same editor. The instrument to compare against stays, since that part is specific to each sensor
and the shared hint cannot name it.

### Presence

The radar module's own settings. These five replaced two captions that sat under the sliders and only
appeared at one end of their range — the note explaining what zero meant was invisible at every value
except zero, which is the one moment the reader does not need it.

Only the LD2450 set is listed. `radar_timeout` is shared with the LD2410, which means the same thing on
both; the LD2410's movement and stillness gates have no hints yet.

`presence` is on the card rather than on the sensor chip that links here, for the chip-width reason above.
It is the first thing a reader can question after following that chip, and the answer matters in a product
with microphones in it: people assume presence is heard rather than sensed.

| Key | Where | Text |
| --- | --- | --- |
| `presence` | LD2450 / LD2410 card | From the mmWave radar, not the microphones. It sees through the enclosure and does not need line of sight, so it reports an empty room faster than a motion sensor would. |
| `radar_range` | Detection range | How far out the radar looks, in centimetres. Anything beyond this is ignored even if the radar can see it, which is how you stop it detecting the hallway or the room next door. At zero it uses the module's own default. |
| `radar_stability` | Stability | How much agreement it needs before it changes its mind. Higher is steadier and slower: fewer false triggers from a curtain moving, but a longer wait before it admits the room is empty. Lower reacts faster and twitches more. |
| `radar_timeout` | Timeout | How long presence is held after the radar stops seeing anyone, in seconds. This is what stops the lights going out while you sit still. At zero it clears the moment you are lost, which is usually too eager. |
| `radar_multi` | Multi-target | Tracks several people at once instead of only the strongest return. Needed for the target plot to show more than one person, and for the zone counts to be right in a busy room. |
| `radar_bt` | Bluetooth | The radar module's own Bluetooth radio, used by the manufacturer's configuration app. Nothing here needs it, and leaving it on means an unauthenticated radio in the room, so it is off unless you are pairing that app. |

### Diagnostics

| Key | Where | Text |
| --- | --- | --- |
| `heap` | Internal RAM free | Internal RAM still available. This is the number that matters: it is what audio buffers and the network stack allocate from, and it is far scarcer than PSRAM. |
| `psram` | PSRAM free | External RAM, used for the large buffers. Installed is the size of the chip; the free and total figures cover the region handed to the allocator, which is smaller because the cache and early allocations sit outside it. |
| `loop` | Longest loop | Longest single pass through the main loop since this panel last read the value. Tens of milliseconds is normal. Sustained hundreds means something is blocking, and audio will stutter before anything else does. |
| `esp_temp` | ESP32 Temp | Temperature of the ESP32 itself, not the room. It reads well above ambient because it sits inside a sealed case next to an amplifier, so a warm number here is normal; sustained readings above 80 °C are worth investigating. The temperature on the Controls page is the one that measures the room. |
| `reset` | Last restart | Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware. |
| `xmos` | XMOS firmware, and the XMOS card title | The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's. |
| `xmos_flash` | Reflash XMOS vX.X.X | Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power. |
| `safe_mode` | Safe mode | Restarts with everything but wifi and the updater switched off. Use it when the device is crash-looping and will not stay up long enough to accept an update. |
| `factory_reset` | Factory reset | Erases every setting stored on the ESP32, including the wifi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched. |
| `beta` | Beta updates | Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on. |
| `log` | Log | The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request. |

`xmos_erase` was deleted along with the row it explained. Erasing the audio chip leaves it blank — no
microphones, no speaker, no wake word — and the only way back is the Reflash row directly above it, which
needs the chip it just erased to be responding. Reflash overwrites without erasing first, so nothing in the
app ever needed it. The entity is also unmapped in `config/common/web_ui.yaml`, so the app cannot reach it
at all rather than merely not showing it; the ESPHome button still exists for a bench recovery.

`xmos_flash`'s row label carries the version currently on the chip, from `Satellite1::status_string()`. That
returns `v1.2.3` when the chip is talking and `XMOS not responding` or `Flashing Mode` when it is not, so
the label falls back to a bare "Reflash XMOS" rather than printing a status where a version should be.

The Buttons card is no longer on this route. It moved to the foot of Controls: it answers "does this button
work", which is a question about the object in your hand rather than about its internals, and it sits beside
the volume and mute it duplicates in hardware.

### Config

Two controls here still write to entities in `config/common/tts_routing.yaml` and
`config/common/area_ducking.yaml`, so their wording has to agree with what Home Assistant shows. The
two trees do not: they write the device's own selection at `/api/sat1/sel`, and they are the only face
that selection has.

| Key | Where | Text |
| --- | --- | --- |
| `remote_routing` | Remote routing card title | Plays the assistant's spoken answer on other speakers as well as this one. Tick a room to include every player in it, or open the room and pick players individually. Local Speaker is this device's own speaker - untick it and the answer is heard only where you have chosen. |
| `area_ducking` | Area ducking card title | Turns other speakers down while the assistant is busy, then puts them back where they were. It runs from the wake word to the end of the answer, so the room is quiet while it listens to you as well as while it answers. Tick a room to cover every player in it. |
| `remote_tts_volume` | Remote TTS volume | How loud the answer is on the remote speakers. It does not touch this device's own level - that is Voice Override, on the Audio card. Sonos reads the level off the announcement; another Satellite1 has its Voice Override set and put back; anything else has its media volume set and restored. |
| `remote_wake_chime` | Remote wake chime | Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to. |
| `duck_volume` | Duck volume | The level they drop to. Players already quieter than this are left alone, so a whole-house group does not get turned up. |

Six hints were deleted rather than reworded, and the reason is worth recording: they explained
switches that no longer exist (`tts_routing`, `tts_local_speaker`, `duck_area`, `duck_tts_targets`) or
described the tree in prose when the tree now shows the same thing directly (`tts_targets`,
`duck_players`). `tts_manual_ids` went with the free-text entity id field it belonged to, replaced by
the "No Area Assigned" group, which names those players instead of asking someone to know their ids.

The trees carry no ⓘ of their own, because they have no row to hang one off — `remote_routing` and
`area_ducking` sit on the card titles instead and are where each tree is explained. That is also the only
place `Local Speaker` is described: it is the one tick whose effect does not follow from its name, since
it silences this device rather than adding a target.

`duck_volume` deliberately no longer says when the ducking happens. That is in `area_ducking` directly
above it, and stating the window in both made the two bubbles read as descriptions of different features.

### Header

| Key | Where | Text |
| --- | --- | --- |
| `switcher` | Device switcher sheet title | Each Satellite1 serves its own copy of this page, so everything you change here applies to the device named above. This is where you move to a different one without typing in its address, and it keeps you on the page you were already on. |

This one answers a question the app's shape provokes rather than one the hardware does. Every Satellite1
serves its own copy of the page, so "which device am I changing?" has a real answer that nothing else on
screen states.

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
| `theme_to_dark` | Switch to dark theme |
| `theme_to_light` | Switch to light theme |
| `ha_pending` | Asking Home Assistant which speakers you have. |
| `ha_never` | Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns. |
| `ha_refused` | Home Assistant did not answer. Either it is older than 2025.12, or this device is not allowed to perform actions: Settings › Devices & services › ESPHome › this device › CONFIGURE, then tick “Allow the device to perform Home Assistant actions”. |
| `ha_no_area` | This device is not in a Home Assistant area, so “Route TTS To All Area Players” and “Duck All Area Players” have no room to refer to. You can still pick any room below. Assign it to an area in Home Assistant and refresh. |
| `ha_no_players` | Home Assistant has no media players at all, so there is nothing to choose between. |
| `ha_truncated` | Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below. |
| `sel_failed` | That change was not saved. The device rejected it, or the connection dropped. |

`copied` replaces the Log card's Copy button for 1.4 seconds after a successful copy, and it exists for a
better reason than politeness. The button used to be `navigator.clipboard?.writeText(...)`, which copied
nothing on any real device: the Clipboard API needs a secure context, this app is served over plain HTTP,
so `navigator.clipboard` was undefined and the `?.` made the whole thing a silent no-op. It appeared to
work in development only because `127.0.0.1` gets a secure-context exemption that `192.168.x.x` does not.
There is now an `execCommand` fallback, which is deprecated but is not restricted by origin, and this
string is what would make the same failure visible next time.

`theme_to_dark` and `theme_to_light` are the light/dark button's accessible label and tooltip in the
header. The button itself is only an icon, so this text is the whole of its name.

`ha_refresh` and `ha_refreshing` are gone with the button they labelled. The device's cached list is now
synced once per load of the app, when the Config route first appears, rather than on demand — so the
gesture that fetches a newly added speaker is reloading the page. There is no interval behind it: the
device otherwise only re-asks Home Assistant five seconds after the native API connects.

## Strings that are not in `copy.js`

A few short labels sit inline in the components because they are part of the layout rather than
explanations, and lifting them out would make the markup harder to read than it makes them easier to
edit:

- The calibration editor's three row labels — `sensor reads`, `offset`, `shows`. It has no confirm button:
  every press is already written to the device, so it closes when you look away from it.
- `Press a button on the device; it lights up here.` under the Buttons card, now at the foot of Controls.
- `Waiting for the device to say something.` in an empty log.
- The confirm-button verbs: `Reflash now`, `Restart into safe mode`, `Erase everything`. `Erase it` went
  with the XMOS erase row.
- `Update available`, `Install <version>` and `Release notes` in the firmware update panel, plus `Up to
  date` under the Sat1 firmware version. The panel appears only when there is an update; the version is on
  the button rather than described beside one, because pressing it is the whole point of the panel.
- `Local Speaker`, `No Area Assigned`, `whole area` and the `n/m` counts in the two trees. All four are
  structure rather than explanation - the first two are row labels and the last two are state readouts.
- `Play assistant responses on selected players` and `Lower the volume on selected players upon wake word
  detection`, the two tree headings. Both say what ticking something does, and leave the rest to the card's
  ⓘ. The second was `Quieten while talking`, which was inaccurate: the duck also covers the listening half
  of the interaction. It wraps to two lines at every phone width, which is deliberate — the alternative was
  a heading that stopped short of naming the trigger.
- The four route names in the nav drawer — `Controls`, `Presence`, `Config`, `Diagnostics` — which are the
  same strings as the routes themselves. The drawer has no heading; the device name is in the bar above it.

The voice assistant phase names — Idle, Waiting for a command, Listening, Thinking, Replying, Not
ready, Error — are in `frontend/src/lib/device.js`, keyed by the `voice_assist_*_phase_id`
substitutions in `config/common/voice_assistant.yaml`. They have to stay aligned with those numbers.
