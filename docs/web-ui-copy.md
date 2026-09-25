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

### Home

The route was called Controls until the September 2026 rename pass; the page is the one you land on,
so it is named for that. Two hints moved here with their rows in the same pass: `mute` and
`voice_override`, both now in the Assistant card (the renamed Voice card, whose transcript rows are
labelled User and Assist rather than heard and said).

| Key | Where | Text |
| --- | --- | --- |
| `mute` | Assistant card, Mute microphones | Cuts the microphones in hardware, not software - wake word detection stops with them. The mute button on the device does the same thing. |
| `voice_override` | Assistant card, Voice Volume Override | How loud this device speaks when the assistant replies, separate from media volume. Zero follows the media volume instead. Speakers you route answers to have their own level - Remote TTS volume, on the Audio page. |
| `finished_speaking` | Assistant card, Finished speaking detection | How long the assistant waits after you stop talking before it answers. Aggressive answers fastest; Relaxed tolerates longer mid-sentence pauses. This is a Home Assistant setting - the same control as on its device page - so it only appears while Home Assistant is connected. |
| `temp` | Calibration editor, title row | Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room. |
| `temp_unit` | Temperature editor, Fahrenheit row | Shows temperatures in Fahrenheit throughout this app. The device stores and reports Celsius either way, so Home Assistant and the saved calibration are unaffected. |
| `humidity` | Calibration editor, title row | Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room. |
| `lux` | Calibration editor, title row | Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger. |
| `calibrate` | Calibration editor, offset row | Adjust until the reading matches a trusted instrument in the same room. The correction is stored on the device and survives restarts. |
| `led_ring` | LED ring | The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error. |
| `timers` | Timers card | Timers set by voice, held on the device - they keep counting and still ring if Home Assistant goes away. Voice is also how they are managed: name one when you set it ("set a pizza timer for ten minutes"), pause or cancel it the same way. |
| `ma_connect` | Media footer, Music Assistant panel | Connecting this page straight to your Music Assistant server makes the controls instant and unlocks search. Find your server below, then create a long-lived token in Music Assistant under Settings, then your profile, and paste it here. Both are remembered by this browser and, behind your sign-in, on this device - so your other browsers and phones connect without re-entering them. |

The media drawer and the players drawer carried `media` and `media_group` hints on their title rows until
September 2026, when the titles were cut (the artwork names one drawer, the speaker rows the other), and
the hints went with them.

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
| `presence` | LD2450 / LD2410 card | Sensed by radar, not the microphones. It works through the case and notices people even when they sit completely still. |
| `distance_unit` | Distance pill, Feet row | Shows every distance on this page in feet. The radar keeps measuring and storing centimetres either way, so Home Assistant and the saved settings are unaffected. |
| `radar_range` | Detection range | How far the radar looks. Anything farther away is ignored - use it to stop the radar seeing into the hallway or the next room. All the way left is the full 6m reach. |
| `radar_stability` | Stability | How sure the radar must be before it changes its answer. Higher is steadier but slower to notice changes. Lower is quicker but can flicker. |
| `radar_timeout` | Timeout | How long the room still counts as occupied after the radar loses sight of everyone. Set it high enough that the lights stay on while you sit still. |
| `radar_multi` | Multi-target | Track up to three people at once instead of only the clearest one. |
| `radar_bt` | Bluetooth | The radar's own Bluetooth, used only by the manufacturer's phone app. Leave it off unless you use that app. |
| `radar_zones` | Zones card | Draw shapes on the map above. Each zone reports on its own whether someone is inside it. The ignore area is the opposite: anything in it never counts - useful for fans and curtains. |
| `radar_resolution` | Distance resolution | How finely distance is split into the nine rows above: 0.75m steps reach the whole room, 0.2m steps reach less far but with more detail up close. Changing it changes what each row means, so re-check your levels after. |
| `gate_move` | Movement column title | How much movement the radar sees at each distance, live. Drag a notch to set the trigger level for that distance - anything above it counts as a person moving. |
| `gate_still` | Stillness column title | How much tiny motion - like breathing - the radar sees at each distance, live. Drag a notch to set the trigger level - anything above it counts as a person holding still. |
| `gate_max_move` | Furthest movement gate | The farthest distance that counts for movement. Rows past it dim in the chart above and are ignored. |
| `gate_max_still` | Furthest stillness gate | The farthest distance that counts for stillness. Rows past it dim in the chart above and are ignored. |

There is no `radar_save`: the save button it explained was removed in favour of an automatic flash save
ten seconds after the last settings write (flushed early if the page is left with one pending).

`PRESENCE` is the one table here that shortens the firmware's words rather than explaining them, so it is a
display map rather than a hint: the key is what the radar reports and the value is what the pill shows.

| Key | Where | Text |
| --- | --- | --- |
| `Approaching` | Presence pill on the home page | Closer |
| `Moving Away` | Presence pill on the home page | Away |

Only these two are listed because only these two are too long. The firmware also reports `Still` and
`Clear` on an LD2450, and `Clear`, `Moving` and `Still` on an LD2410, and anything absent from the table
falls through unchanged — which is what should happen when a future radar module reports a state nobody has
seen yet. The measurement behind it: at the pill's 17px, "Approaching" wants 103px, and 70px even at 11px,
against 60px of room at a 360px viewport. The pill used to halve its own font past eight characters, which
made one chip look like a different design from the three beside it. The full phrase is still the pill's
hover title, and the firmware's wording is untouched, so Home Assistant is unaffected.

These two rows were missing until the mirror check was widened to cover `PRESENCE` as well as `HINTS` and
`TEXT`. Worth noting rather than quietly fixing: a checker that reported "all strings mirror" while never
looking at a third of them was the more expensive problem.

### Diagnostics

| Key | Where | Text |
| --- | --- | --- |
| `heap` | Internal RAM free | Internal RAM still available. Audio buffers and the network stack allocate from it, and it runs out long before PSRAM does. |
| `psram` | PSRAM free | External RAM, used for the large buffers. Free and total cover the region the firmware can allocate from, which is smaller than the chip installed. |
| `loop` | Longest loop | The longest single pass through the main loop since the last reading. Tens of milliseconds is normal; sustained hundreds means something is blocking, and audio will stutter first. |
| `esp_temp` | ESP32 Temp | The ESP32 chip's own temperature, not the room's. It reads well above ambient inside the sealed case, so warm is normal; sustained readings above 80 °C / 176 °F are worth investigating. The room's temperature is on the home page. |
| `reset` | Last restart | Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware. |
| `usb_power` | USB-C Power Supply | What the USB-C power supply agreed to deliver. Chargers that speak USB Power Delivery negotiate a voltage and the most current they will supply at it - the ~ marks that ceiling: the device draws only what it needs, usually far less. 5 V is a plain supply that negotiated nothing. 9 V or more is what lets the speaker amplifier run at High gain - the Speaker amplifier card just below. |
| `speaker_amp` | Speaker amplifier card title | The chip that drives the built-in speaker. What you hear is three levels multiplied together: the power gain mode the amplifier picks from your USB-C supply, the digital volume the firmware computes from your sliders, and the analog gain set below. |
| `amp_mode` | Speaker amplifier, Power gain mode | Picked automatically from the measured power supply - it is a reading, not a setting. High gain: a 9 V or higher USB-PD supply is connected and the speaker can reach full loudness. Low gain: the device is on a plain 5 V supply, so maximum loudness is reduced. Off: the amplifier is shut down, normally because line out is selected. A supply that negotiates 9 V or more (the USB-C Power Supply row in the Device card above) is what unlocks High gain. |
| `amp_dvc` | Speaker amplifier, Digital volume | How far open the amplifier's digital volume control is right now - the level the firmware computes from the volume buttons, Voice Volume Override and ducking. Shown so you can see what the amplifier is actually being fed; it is managed automatically and has no handle here. To change it, use the volume controls. |
| `amp_gain` | Speaker amplifier, Analog gain | The amplifier's output-level ceiling, applied on top of the digital volume. The notch on the track is the factory default, 15 dBV - the right everyday setting, and the slider snaps to it. Higher values make everything louder but can add distortion, and on a 5 V supply the chip's built-in limiter will duck the sound to protect the power rail. Lower it if the speaker distorts at high volume. |
| `speaker_channel` | Speaker amplifier, Channel | Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want. |
| `launch` | Launch card title | Scan the code with a phone, or paste the link into a Home Assistant dashboard button, and that browser lands here already signed in - no password, no button press. The code carries the device's current network address, so any phone on your network can scan it; the link carries the device's permanent name, the right form to paste somewhere that keeps it. Anyone who has either can sign in with it, so treat them like the password. Sign out everywhere revokes them and every session, then issues a new one. |
| `ha_ingress` | Home Assistant card title | Puts your Satellite1s in Home Assistant's sidebar, proxied through Home Assistant itself - so it works wherever Home Assistant does, on your local network or over a public https address. hass_ingress is a third-party integration, not part of this firmware. The YAML covers every Satellite1 Home Assistant knows about: this device is the one visible "Satellite1 Fleet" entry, the rest sit hidden behind it, and the device switcher reaches them all from inside the panel. Anyone who can open the panel reaches the devices' sign-in pages through it; the YAML limits the panel to admin users - remove the require_admin line to show it to everyone. |
| `xmos` | XMOS firmware, and the XMOS Recovery card title | The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's. |
| `xmos_flash` | Reflash XMOS vX.X.X | Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power. |
| `radar_recovery` | LD2410 / LD2450 Recovery card title | The radar module's own recovery actions. Restart just power-cycles the module. Factory reset erases the settings stored on the module itself - detection range, gate thresholds, zones, everything tuned on the Presence page - and does not touch this device's settings. |
| `maintenance` | ESP32 Recovery card title | Ways to restart or reset this device. None of them are part of everyday use. Restart is always safe - your settings survive it. Factory reset is the only row here that erases anything. |
| `safe_mode` | Safe mode | Restarts with everything but Wi-Fi and the updater switched off. Use it when the device is crash-looping too fast to accept an update. |
| `factory_reset` | Factory reset | Erases every setting stored on the ESP32, including the Wi-Fi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched. |
| `beta` | Beta updates | Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on. |
| `log` | Logs card title | The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request. |
| `crash` | Crash Reports card title | What the device remembers about its own crashes: when each one happened, which task died and where, the last log lines before it went down, and a downloadable crash dump for the deepest look. Attach the dump and the log to a support request, or erase them once the cause is found. A crash that cuts power entirely leaves less detail than one the device rebooted itself from. |
| `crash_log` | Pre-crash log row | The last log lines the device wrote before it went down, recovered across the reboot. The most recent crash only - a newer crash replaces it. |
| `crash_dump` | Crash dump row | A complete snapshot of what every task was doing at the moment of the most recent crash, analyzed offline against this exact firmware build. It may contain fragments of anything the device held in memory, so share it like a log, not like a screenshot. |
| `crash_erase` | Crash history row | Deletes the recorded crashes, the pre-crash log and the crash dump. The device keeps working either way; erase once a cause is found, so the next crash is unmistakably new. |

The **Crash Reports** card sits under Device, whose Last Restart row is the question it answers. Its
standing text: `crash_none` "No crashes recorded." (the empty state, a good day); `crash_ran`
"ran %s" (how long the crashed session lived); `crash_restart_ago` / `crash_restarts_ago`
"%1 after power-on · N restart(s) ago" (the wording for a crash the flight recorder could not
wall-clock-stamp); `crash_bt` "Backtrace" and `crash_bt_corrupt` "Backtrace (corrupted)"; the
`crash_log_row` "Pre-crash log" row with its Show/Hide toggle and `crash_log_none` "No log survived
from before the crash."; the `crash_dump_row` "Crash dump" row with `crash_download` "Download";
the `crash_erase_row` "Crash history" row with `crash_erase` "Erase history"; and
`crash_no_part` — shown on a device that took this firmware over OTA — "This
device can list crashes but can't keep full crash dumps: that needs a one-time flash over USB, which
also resets stored settings. Everything else here works without it."

`launch`'s QR and copyable link deliberately differ: the QR is minted on the device's current IP —
scanned live off the screen, so the address is fresh by construction, and an IP works on phones that
cannot resolve `.local` names — while the link keeps the permanent `.local` form for anything
long-lived, like a dashboard button. The hint is the only place this is explained; on the card the
two simply work.

The **Home Assistant** card sits under Launch (same subject: ways to reach this UI) and stays
collapsed by default — it is setup, not status. Its strings: `hai_pre` "Put your Satellite1 fleet
in the Home Assistant sidebar with the third-party ", `hai_link` "hass_ingress" (the external
link), `hai_post` ' integration: install it from HACS, add the block below to your
configuration.yaml (replacing any ingress: section you already have), and restart Home Assistant.
One "Satellite1 Fleet" item appears in the sidebar, and the device switcher works inside it.'; the
copy button `hai_copy` "Copy YAML" / `hai_copied` "Copied"; and `hai_dhcp_hint` "The YAML carries
each device's current IP address. Give your Satellite1s DHCP reservations so the addresses stay
valid - if one changes, update its url and host lines to match." The YAML block itself is generated
from the fleet's live names and IPs (this device as the one visible "Satellite1 Fleet" entry, every
roster peer as a hidden `parent:` child), so it is not mirrored here; its inline comments are part
of the snippet on purpose (they travel with the paste into configuration.yaml).

`xmos_erase` was deleted along with the row it explained. Erasing the audio chip leaves it blank — no
microphones, no speaker, no wake word — and the only way back is the Reflash row directly above it, which
needs the chip it just erased to be responding. Reflash overwrites without erasing first, so nothing in the
app ever needed it. The entity is also unmapped in `config/common/web_ui.yaml`, so the app cannot reach it
at all rather than merely not showing it; the ESPHome button still exists for a bench recovery.

`xmos_flash`'s row label carries the version currently on the chip, from `Satellite1::status_string()`. That
returns `v1.2.3` when the chip is talking and `XMOS not responding` or `Flashing Mode` when it is not, so
the label falls back to a bare "Reflash XMOS" rather than printing a status where a version should be.

Two cards were renamed in the September 2026 pass: **Log** became **Logs**, and **Sat1 Device** became
**ESP32 Recovery** (briefly "Maintenance", then "Power & Recovery") — the final name pairs it with
**XMOS Recovery** above it: the two cards do the same job for the two chips. ESP32 Recovery is expanded
by default (Restart is the row people come for); XMOS Recovery stays collapsed. All three collapsible
cards now put the caret after the title instead of before it, so their titles line up with every other
card's.

The Buttons card is no longer on this route. It moved to the foot of the home page: it answers "does this button
work", which is a question about the object in your hand rather than about its internals, and it sits beside
the volume and mute it duplicates in hardware.

### Wake Words

Its own route since the September 2026 pass, rebuilt three times that month — around **runtime
wake word loading**, then the **Monitored Margin**, and finally the **Living Graph** (the
`wake-words-tuner-v2` canvas is the design of record for every tuning, graph and word-selection
surface; the older mockups canvas keeps the rest). One idea carries the page now: the graph stops
being a report you read after tuning and becomes the single object you tune, watch, and correct.
Two cards remain: **Wake Words** (one row per word, the stop word's permanent third row, and the
chime bell in the card's own header) and **Wake Word Sources**. The Recent detections card and the
Wake Word Settings card are both retired — the graphs absorbed one, the header bell the other.

Each word's **row** is the whole story, with every control visible on it (no chevron menu, no
modal, no `tuned` badge): the **pill** — the quoted phrase with its own chevron inside the border,
which expands the **word picker inline** exactly where the graph sits (choosing another word is
the change, unchecking the current one is the removal); the **Living Graph** once tuned — the
graph's presence *is* the tuned state; the halo-ringed **Tune it!** button (`ww_tune_btn`) on an
untuned row, sitting exactly where a tuned word's status would be; and the **Voice Pipeline**
select. Mid-swap the card tells two truths at once: the old row dimmed with `mb_still` ("still
listening", because it is), the new row downloading beneath with `mb_swap_note`. `+ Add word`
expands the same inline tree while a slot is free and disappears entirely with both taken.

The **Living Graph** is one SVG in two sizes. Collapsed on the row it is the word's standing
24-hour record: two color worlds split at the threshold **knob** — left of it the amber frost
(everything there is blurred, veiled, ignored), right of it a whisper of accent blue with every
**real firing** crisp on top, a dot at the confidence the register actually measured, fading as it
ages, landing with an orbit ripple in the same beat the row flashes (there is no "Heard it" text —
the animation is the announcement). Hollow amber marks are close calls, sunk in the frost. Tap a
dot for its story (`pct · time-ago`, `lg_ignored_short` suffixing an almost-fire); tap the knob
and placement reopens over everything the graph knows. A tuned row wears the radar's live-dot
grammar in green (`lg_listening`, renamed from `lg_live`/"Live" — "listening" says what the
device is doing).

The **stop word** is a permanent third row displaying as **"Stop"** (capitalized everywhere; the
model reports it lowercase), and its pill IS its switch — a filled iOS-style toggle with the title
inside the button, soft green (`--tgl-on`) with the white knob at the right edge while armed,
soft red (`--tgl-off`) with the knob slid left while off. Its status label is honest about the
runtime state: the stop model only actually runs during spoken answers, routed announcements and
ringing timers, so the row reads a yellow-dot **`lg_paused`** ("Paused") while armed but idle and
flips to green `lg_listening` the moment the firmware arms the model (the `stop_word_active`
sensor, published by the same scripts that flip it — the running state wins even with the switch
off, because a ringing timer arms it regardless). Off hides the graph entirely
(`stop_off_note`): a disabled model is unloaded, so there is nothing to draw. The **wake chime**
in the card header wears the same switch grammar with the bell inside — green and ringing, or red
and struck through — each with an ⓘ beside it.

| Key | Where | Text |
| --- | --- | --- |
| `wake_words` | Wake Words card title | This device listens for up to two wake words at once - one row each below - plus the "Stop" word that interrupts answers and alarms. Words beyond the built-in ones download from their source when you choose them, and fetch again at every restart. A tuned word wears its graph: everything left of the handle is ignored, every dot right of it is a real firing from the last 24 hours, fading as it ages. Tap the handle to retune, tap a dot for its story, tap the chevron on a word to change or remove it. |
| `living_graph` | the graph (accessible name + tuner ⓘ) | A confidence scale from 0 to 100%. Everything left of the handle sits under the frost - the device ignores it. Everything right of it is live: solid dots are real firings at the confidence they scored, fading over 24 hours; hollow marks almost fired. Drag the handle to move the line; if dots you don't recognize sit just past it, drag past them. |
| `wake_sources` | Wake Word Sources card title | The wake word list is fetched live from these places - nothing is copied to our servers. Anyone can publish a wake word model, and quality varies: the included words are hand-tuned, community ones may fire too eagerly or miss you. Adding a source here only grows the list; a word starts being used when you pick it above. |
| `wake_sound` | the chime bell's ⓘ | Plays a short chime on the speaker the moment the wake word is detected. |
| `stop_word` | the Stop pill's ⓘ | While an answer is playing - this device’s own, or one another Satellite1 routed here - saying "stop" cuts it off everywhere it is playing. A ringing timer can always be silenced this way, whichever way this is set. |

The **Wake Word Tuner** is two user-paced phases on the same graph (v2, replacing the four-phase
ritual — no room-listening phase, no confirmation phase). The **ready gate** (`tn2_ready`, the
stop variant `tn2_ready_stop` still says *start some music first*) opens with the room's last 24
hours already on the graph as the **amber smatter** — one faint dot per hourly high-water bucket,
newest opaque, oldest nearly gone, a fresh register reading landing live with its ripple; legend
`tn_legend_hist`, "wake history" (owner wording). No aggregate tick, no "24h peak" label: the
room's reach is wherever the smatter ends. Then the **voice rounds** (`tn2_near/far/other` —
near first, twice is plenty; once from across the room; round three, skippable, is anyone else in
the house), attempts landing as labeled accent dots. Then **placement**: the knob seeds at a
sensible spot inside the gap (Apply-without-dragging is a correct answer) and is dragged — dots
sink into or surface out of the frost live — while the readout narrates the tradeoff (`tn2_ok` +
`tn2_under` in the safe zone; `tn2_high`, amber-toned, at the quietest attempt; `tn2_low`,
red-toned, when amber dots sit past the line — copy never attributes a firing to a source, owner
call: "we don't know it's the TV"). Apply is the end: the session closes, the row returns wearing
`lg_listening`, and the first real firing confirms itself by landing with its ripple. The knob-tap
**quick edit** (`tn2_title_quick`, '"Okay Nabu" · move the line') reopens placement with no
session and no re-recording — the persisted voice stats feed the readout as data, never drawn as a
band — with **Redo voice rounds** (`tn2_redo`) beside Apply for the full re-measure. Reset default
is gone. The no-gap failure stays diagnosed by side (`tn_nogap_room` vs `tn_nogap_voice`);
`tn_nocap` and `tn_gone` keep their jobs.

While a session runs, the same-area Satellite1s are **held muted** so they don't answer the word
being said over and over (tune-time peer muting, September 2026 — each peer restores its own prior
mute state when the session ends or its 60s TTL lapses, and a hand unmuting at the peer itself
outranks the session for good). The status is explicit by owner decision, a line under the phase
copy: `tn_pm_one` / `tn_pm_many` ("Muted 2 nearby satellites for this session") with the roster
names in parentheses, `tn_pm_failed` ("Couldn't mute Kitchen - it may answer while you tune",
amber — old firmware, a stale password and a dead peer are indistinguishable and the person only
needs the consequence), and `tn_pm_unknown` ("Can't check for nearby satellites - Home Assistant
is unreachable", worded as a limit rather than an error). Silence only when the roster answered
and named no same-area peer — nothing to say about an empty room.

Under the hood everything rides the one vendored addition this repo carries: the **high-water
register** in `micro_wake_word`'s `streaming_model` (see
`esphome/components/micro_wake_word/FPH_VENDOR.md` — byte-identical to the pinned ESPHome release
except that one `// FPH:`-marked block; upstream PR intended). It is the sole source of every
sub-threshold score: the hourly buckets behind the smatter (`day` on the payload, 24 values per
track, newest first), the scored close calls within ~5% below a cutoff (`near`), and — new in v2 —
**each real firing's confidence**: `notify_detection` drains the register at the moment of firing
and keeps the value instead of discarding it, `push_wake_detection` collects it a few actions
later in the same automation, and the detection ring's entries carry it (`hist` grew a third
element), which is what places every dot on every graph, on every build, debug or not. Tune stats
(`n`/`f`/`h` on the cutoff endpoint) persist per word so the readout survives reloads; a session
expires two minutes after the app's last keepalive and always restores the configured threshold.

The **word picker** expands inline (the modal sheet is retired) from a pill's chevron, `+ Add
word`, or the empty state's one button (`ww_choose`, under `ww_route_none` — "on the device, no
cloud" leads for a first-time owner deciding whether to trust a microphone). Inside, it wears the
/audio route's tree clothes — the sunken scrolling box, group headers with a caret and a count
pill, the drawn checkbox — because that is the app's one selection-list pattern. A row's small
facts — languages ("en"), model size ("61 KB"), the `ww_unverified` caveat, the
training-generation tag that disambiguates twins ("v3"), and `ww_on_other` on the row the other
slot holds — all wear that same count-pill shape; only the speak button keeps its own glyph. The
inline footer carries the request/train links. Groups are the sources: **Built-In wake words**
first, then one group per source. There is no Disabled entry: a slot empties by unchecking the
word it holds. The slots are still not entities: everything rides `GET/POST /api/sat1/wakewords`
and its `slot`/`cutoff`/`tune` sub-endpoints, owned by the `mww_runtime_loader` component.

The swap's lifecycle lives on the rows (see above): the byte count in the new row's right slot,
then either the settled row or `ww_failed` plus the specific `WW_ERR` reason with Retry as the
row's button. The picker never offers what the firmware is certain to refuse: manifests the browser
can read but that are not version 2 are filtered out at enumeration. The reasons are keyed to the
firmware's SlotError numbers; the one customers will meet most is 2, the
not-a-microWakeWord-model line, because the most popular wake word collections on GitHub are for a
different engine. Key 0 is the fallback for a write the device refused outright or that never got
an answer. Nothing celebrates a landed swap (owner: the row offers exactly the two next steps -
the pipeline and Tune it! - and nothing shouts at the moment a first-time user is most easily
overwhelmed; a real firing announces itself as its dot landing with the ripple, wordlessly).

Home Assistant still owns which pipeline answers which word: the **Voice Pipeline** select
(renamed from Assistant — it is Home Assistant's own word for the thing being picked) sits on each
word's row, and is `Preferred` plus the customer's pipelines (the old `Off` entry
is gone — silencing a slot is removing its word). Its tooltip (`voice_pipeline`) says pipelines
are built in Home Assistant under Settings › Voice assistants, ending in the `vp_docs` link to the
FutureProofHomes walkthrough. The slot-sync keeps Home Assistant's two pairings equal to the two
slots, and a wake word change made from Home Assistant's own select is adopted back into the slots
by the firmware, which stays the source of truth. A downloaded word reaches the ESPHome device
page's wake word field by its real phrase: the firmware prettifies slug phrases from bulk-trained
manifests ("hey_alice" becomes "Hey Alice") before anything displays them, and since Home Assistant
only re-reads the wake word list on connect, the loader drops the API connection a moment after the
advertised set changes - Home Assistant reconnects onto the fresh list, and the app waits out that
reconnect (and re-reads the payload) before writing the pairing. With Home Assistant unreachable the dropdown does not
render and one of three lines says what that costs, most specific cause first: `assistant_blocked`
when the actions checkbox is off (with a **Show fix** link), `ha_too_old` when Home Assistant
predates 2025.12, and `assistant_needs_ha` for plain unreachable.

The Wake Word Sources card lists each source with a live word count and a remove ✕ (whose
confirmation, `ws_remove_b`, owns the one surprising fact: a picked word keeps working, because the
device remembers its link rather than the source). `ws_ph` invites a GitHub repo or model `.json`
URL; the foot reads, verbatim, "Don't see your wake word? Request one, or train your own
microWakeWord." — the first link landing on the Tater catalog's request-a-wake-word README section
(the free issue-driven trainer), the second on TaterTotterson/microWakeWord-Trainer-Nvidia-Docker.

Diagnostics' **Recent wake detections** card retired here (September 22 2026), and the route's own
Recent detections card followed it in the v2 pass: every word's Living Graph carries the same
story in place — what fired, how confidently, and (a tap away) when — so the standalone record has
no job left. Only `det_just_now` survives, in the dots' popovers.

### Audio

The route was called Config until the September 2026 rename pass. Mute microphones and Assistant
volume left for the home page, the wake words card became the Wake Word route above, and the
speaker's own wiring - the old Audio Output card, grown into Speaker amplifier - moved to
Diagnostics under the Device card (owner call, September 2026), where it sits beside the USB-C
Power Supply reading that decides the amp's gain mode. What remains here is where sound goes: the
two trees.

Two controls here still write to entities in `config/common/tts_routing.yaml` and
`config/common/area_ducking.yaml`, so their wording has to agree with what Home Assistant shows. The
two trees do not: they write the device's own selection at `/api/sat1/sel`, and they are the only face
that selection has.

| Key | Where | Text |
| --- | --- | --- |
| `remote_routing` | Audio routing card title | Plays this device's audio on other speakers as well as this one: the assistant's spoken answers, sign-in prompts, ringing timers, and the wake chime if you turn that on below. Tick a room to include every player in it, or open the room and pick players. Local Speaker is this device's own speaker - untick it and answers play only where you have chosen. |
| `area_ducking` | Area ducking card title | Turns other speakers down while the assistant listens and answers, then puts them back where they were. Tick a room to cover every player in it. |
| `remote_tts_volume` | Remote TTS volume | How loud answers are on the remote speakers. This device's own level is Voice Volume Override, on the home page. Sonos reads the level from the announcement itself; anything else has its volume set for the answer and put back afterwards. |
| `remote_wake_chime` | Remote wake chime | Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to. On Sonos and similar speakers the chime can land up to a second late - their clip playback has a fixed startup cost the device cannot remove. |
| `remote_timer_ring` | Remote timer ring | Rings a finished timer on the target speakers too, until the alarm is stopped. Saying "stop" at any speaker silences it everywhere. |
| `remote_sync_guard` | Remote sync guard | How long this device keeps its microphone closed after a routed answer, so it cannot hear its own answer from a speaker running slightly behind and mistake it for you. It only engages when the assistant is about to listen again - a continued conversation or voice sign-in; one-shot answers skip it entirely. |
| `duck_volume` | Duck volume | The level they drop to. Players already quieter than this are left alone, so a whole-house group does not get turned up. |

`voice_override` and `remote_tts_volume` are the pair most easily confused, and with Voice Volume Override
back on the home page they are on separate routes again. Each hint therefore names the speakers it
moves and points at the other by its on-screen label and page, because the hints are now the only
thing keeping the pair apart.

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
| `switcher` | Device switcher sheet title | Everything on this page applies to the device named above. Pick another device and this page becomes its controls - no address to type, no leaving the page. A device on older firmware opens its own page instead. A radar tag lights up while that device detects someone in its room. |

This one answers a question the app's shape provokes rather than one the hardware does. Every Satellite1
serves its own copy of the page, so "which device am I changing?" has a real answer that nothing else on
screen states. The wording changed with single-origin device switching (September 2026): picking a
device now retargets this page at the peer rather than navigating to it - the change that keeps the iOS
home-screen app out of Safari's in-app sheet - so the sentence owns both outcomes: the takeover for
fleet-current firmware, the old navigation for anything older.

Every route opens with its name worn as a **folder tab** growing out of the top card's top-left
corner (the v2 chrome, replacing the short-lived sticky route title bar): the route's name and
glyph from the same table the drawer reads, in the card's own background, the card's corner
squared beneath it so tab and card read as one piece of paper. It carries no copy of its own; the
open run to the tab's right is deliberately empty — reserved for the coming toast redesign, which
will dock there. "Wake Word" became "Wake Words" in the same pass (the route holds two words plus
Stop, and the singular read as a typo in a title).

On Home, the Assistant card's transcript grew **one tab per wake word** (September 22 2026): each
exchange arrives tagged with the word that initiated it, the last-uttered word's tab is active and
follows fresh firings until the person picks another by hand, and a single word (or an untagged
old-firmware transcript) shows no tabs at all. The tabs carry only the quoted words themselves —
"Stop" capitalized like everywhere else — so they add no new copy.

## Standing text

The `ha_*` block below is the Home Assistant data layer explaining its own absence. Each string is a
different cause with a different fix, which is why they are not collapsed into one "unavailable"
message: `ha_pending` is a five-second wait on a fresh boot rather than a fault, `ha_blocked` is the
only one that asks the customer to change a setting (its **Show fix** link opens the walk-through
drawer), and the rest are properties of their Home Assistant rather than of the device. `ha_refused`
- the old string that hedged "either it is older than 2025.12, or actions are not allowed" - was
split into `ha_blocked` and `ha_too_old` when the firmware started reporting which one it is (the
`actions` field on `/api/sat1/ha`, September 2026).

These render as quiet dim text above the trees they explain, not banners: the amber banners were
retired app-wide in the same pass (owner decision). `sel_failed` went with them - a refused selection
write now reports through the shared `write_failed` toast like every other write, and the checkbox
still puts itself back.

`stream_lost` also changed surface, not wording: it is the one sticky toast, standing until the event
stream reconnects, where the transient toasts (`write_failed`, the blocked nudge below) time out.

The toasts moved again in the notch redesign (September 2026): they render one at a time in the run
beside the route's folder tab, colour-coded by kind (red error, amber warning, green success, accent
info), and bursts sharing a key coalesce into one card wearing a ×N badge. New strings with them:
`stream_back` is the sticky's green ending on reconnect; `log_toast_err` / `log_toast_warn` announce
a warning or error line arriving on the event stream, with `%s` the component tag out of its
`[W][wifi:123]` header (`log_toast_dev` is the subject when the header did not parse) - suppressed
while the Logs card is on screen; `crash_toast_*` fire when the crash count moves mid-session (the
device rebooted from a crash while the page watched - history stays the Crash Reports card's story);
`update_toast_*` carry firmware news once per session, the version in the title via `%s`. Tapping a
toast acts - navigation with a reveal (the Logs card forced open at the right level filter, the Crash
or Firmware card scrolled into view) or the fix drawer for the blocked nudge; the ✕ (`dismiss`)
dismisses without acting.

| Key | Text |
| --- | --- |
| `ha_connected` | Connected to Home Assistant |
| `ha_disconnected` | Not connected to Home Assistant |
| `ha_disconnected_detail` | Everything on this page still works - it talks to the device directly. Media and anything that needs your smart home will be unavailable until the connection returns. |
| `stream_lost` | Lost the connection to the device. Retrying. |
| `no_devices` | Only this device. Other Satellite1s appear here once Home Assistant lists them. |
| `no_devices_blocked` | Only this device. Other Satellite1s cannot be listed while Home Assistant actions are off. |
| `peer_up` | Available, according to Home Assistant |
| `peer_down` | Unavailable, according to Home Assistant |
| `presence_on` | Presence detected |
| `presence_off` | No presence detected |
| `remote_tag` | Remote |
| `switcher_home` | This page |

`remote_tag` is the one word in the top bar while a peer is being controlled (single-origin device
switching): the name beside it already identifies the device, so the badge adds only the mode. The
same pill marks the switcher row for the device actually serving the page - `switcher_home` - which
is the way back: tapping it is a plain reset to the local session, not another cross-sign-in.

`presence_on` / `presence_off` title the radar tag on every switcher row (September 2026): each row's
right edge carries the device's transport (Ethernet or WiFi) and radar model (LD2450 or LD2410), and
the radar tag doubles as a presence light. Unattributed, unlike the dots' "according to Home
Assistant": on the serving device's own row the light is the radar's live state over the page's
stream, and on a peer's it is at most one sync beat (~5s, while the sheet is open) behind.

`peer_manual`, `peer_add_ph` and `peer_add` were deleted with the switcher's add-by-address feature
(owner's call: a fallback that needs a paragraph to explain is not pulling its weight). The sheet also
changed shape in the same pass: its header is the top bar's twin — where the bar shows
"☰ &lt;device name&gt; ˅", the sheet shows "✕ Device Switcher ˄", built from the same pieces so spacing
and type match exactly. Both the ✕ and the title close the sheet (Escape works too, on the sheet and
on the nav drawer), and peers Home Assistant reports unavailable are folded into a collapsed
"Offline (n)" disclosure at the bottom instead of sitting dimmed in the main list. Opening the sheet
triggers a Home Assistant re-sync, so the availability dots correct themselves within a second or two
rather than showing the cache from the last sync.
| `write_failed` | A change didn't reach the device, so it hasn't been applied. |
| `write_failed_go` | Tap for the device log. |
| `stream_back` | Connection to the device restored. |
| `log_toast_err` | Error from %s |
| `log_toast_warn` | Warning from %s |
| `log_toast_dev` | the device |
| `crash_toast_t` | The device crashed and restarted. |
| `crash_toast_s` | Tap for the crash report. |
| `update_toast_t` | Update %s is available. |
| `update_toast_s` | Tap to install from Diagnostics. |
| `dismiss` | Dismiss |
| `notif_bell` / `notif_title` | Notifications |
| `notif_all` | All |
| `notif_info` | Info |
| `notif_warn` | Warnings |
| `notif_err` | Errors |
| `notif_arch` | Archive |
| `notif_empty` | Nothing here from the past 24 hours. |
| `notif_empty_arch` | Nothing archived yet. |
| `notif_clear` | Clear All |
| `notif_foot` | Notifications clear after 24 hours. |
| `notif_now` | just now |
| `notif_ago` | %s ago |

The `notif_*` block is the notification center (September 2026): a badged bell beside the theme
toggle opening a top-hung drawer of the past 24 hours' err/warn/info toasts (green ok toasts are
never kept). The badge is a live count of toasts that faded away untapped; tapping a row navigates
with the toast's own reveal (the log line flashed, the card pulsed), and ✕ or a Gmail-style swipe
right files the row under Archive - handled, kept a day, still tappable. Tapping a live toast on
the notch archives it directly, so it never reaches the badge. `HINTS.notif` carries the drawer's
ⓘ explanation.
| `nothing_said` | Voice transcriptions are displayed here. |
| `media_idle` | Nothing playing. Group streams and media sent from Home Assistant show up here. |
| `media_idle_bar` | Nothing playing |
| `media_src_group` | group stream |
| `media_src_local` | this speaker |
| `media_players_title` | Players |
| `media_add_speaker` | Add a speaker |
| `media_group_loading` | Asking Music Assistant… |
| `media_no_tiers` | Grouping and seeking need Home Assistant with Music Assistant - or connect this page to your Music Assistant server from the expanded media view. |
| `media_group_volume` | Group volume |
| `ma_title` | Music Assistant |
| `ma_url_ph` | http://music-assistant.local:8095 |
| `ma_token_ph` | Long-lived token |
| `ma_connect_btn` | Connect |
| `ma_disconnect_btn` | Disconnect |
| `ma_connected` | Connected to Music Assistant |
| `ma_error` | Could not connect. Check the address and token, and that this browser can reach the server. |
| `ma_scan_btn` | Find my server |
| `ma_scanning` | Scanning your network… %s |
| `ma_scan_none` | No Music Assistant server found. It may be on another network or a non-standard port - enter its address above. |
| `ma_scan_https` | Scanning needs this page on its local http address; enter the server's address above instead. |

The search drawer (Plan 20), behind the top bar's magnifying glass. It rides the direct Music
Assistant connection only, so its empty states carry the honest work: what search needs, and where
to set it up. `search_kind` is the singular on each row's sub-line ("Track · Oliver Tree"), one
entry per media type; the plural forms below serve both the filter pills and the section heads.

| `search_open` | Search music |
| `search_title` | Search Music Assistant |
| `search_ph` | Search Music Assistant… |
| `search_clear` | Clear search |
| `search_recent` | Recent searches |
| `search_forget` | Forget this search |
| `search_searching` | Searching… |
| `search_none` | Nothing found for “%s”. |
| `search_more` | Show more |
| `search_all` / `search_track` / `search_artist` / `search_album` / `search_playlist` / `search_radio` / `search_podcast` / `search_audiobook` | All / Tracks / Artists / Albums / Playlists / Radio / Podcasts / Audiobooks |
| `search_play_now` | Play now |
| `search_play_now_sub` | replaces queue |
| `search_play_next` | Play next |
| `search_add` | Add to queue |
| `search_playing` | Playing |
| `search_queued` | Queued |
| `search_play_failed` | Music Assistant couldn't play that. |
| `search_play_failed_sub` | Try again in a moment. |
| `search_target` | Plays on *(followed by the speaker or group name)* |
| `search_via` | via Music Assistant |
| `search_need_ma_t` | Search needs Music Assistant |
| `search_need_ma_b` | Connect this browser to your Music Assistant server to search and play music. The address and token are set once and remembered here. |
| `search_setup_btn` | Set up connection |
| `search_connecting` | Connecting to Music Assistant… |
| `zi_first` | Tap the map to place the first corner. |
| `zi_more` | Keep tapping. A zone needs at least 3 corners. |
| `zi_adjust` | Drag a corner to reshape. Drag the middle to move the shape. Tap a corner to select it. |
| `zi_selected` | Corner selected. Tap Remove corner below to delete it. |
| `zones_set` | Tap a zone button, or a shape on the map, to edit it. |
| `zones_none` | No zones yet. The whole field of view counts as presence. |
| `zone_excl` (hint) | Makes this shape an ignore area instead of a zone: anything inside it never counts as presence. Useful for fans, curtains and pets. There is one ignore area - saving a new one replaces it. |

Every instruction prints in one place: the line between the plot and the zone buttons (or the editor
subcard, which itself stays pure controls). Not editing it shows `zones_set` or `zones_none`; while
editing the `zi_*` strings walk the person from first corner to reshape to corner removal.
`zone_edit_help` (the static paragraph all of this replaced), `zi_tap` and `zone_needs_three` are gone -
the last because `zi_more` already says a zone needs three corners.
| `gate_thresholds_help` | Each row is a band of distance. The bar shows what the radar sees there right now; drag the notch to set where it triggers. Dimmed rows are out of range and ignored. Changes save automatically. |
| `no_sensor_lead` | A presence sensor was not detected in your Sat1. Please |
| `no_sensor_docs` | read our docs to learn more *(links to the presence-sensor docs)* |
| `no_sensor_mid` | , you can purchase a presence sensor |
| `no_sensor_buy` | here *(links to the LD2450 product page)* |
| `no_wake_words` | No wake words are on, so the device will not respond to being spoken to. The mute button and Home Assistant still work. |
| `pipeline_off` | Off |
| `pipeline_preferred` | Preferred |
| `assistant_needs_ha` | Home Assistant keeps which assistant answers each wake word, so that cannot be set from here until it is reachable. Turning a wake word on and off is the device's own setting and still works. |
| `assistant_blocked` | Home Assistant is not letting this device perform actions, so the assistants cannot be listed. Wake words switched here still change the device, but Home Assistant may switch them back when it reconnects. |
| `assistant_slots_full` | Home Assistant can pair only two wake words with an assistant of their own. The rest are answered by the first one's assistant, which is what their dropdowns show. |
| `confirm` | Confirm |
| `cancel` | Cancel |
| `theme_to_dark` | Switch to dark theme |
| `theme_to_light` | Switch to light theme |
| `ha_pending` | Asking Home Assistant which speakers you have. |
| `ha_never` | Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns. |
| `ha_blocked` | Home Assistant is not letting this device perform actions, so your areas and speakers cannot be listed - and answers play only on this speaker until it can. |
| `ha_too_old` | This Home Assistant is older than 2025.12, which cannot answer the calls these lists are built from. Update Home Assistant to choose speakers here. |
| `show_fix` | Show fix |
| `ha_no_area` | This device is not in a Home Assistant area, so “Route TTS To All Area Players” and “Duck All Area Players” have no room to refer to. You can still pick any room below. Assign it to an area in Home Assistant and refresh. |
| `ha_no_players` | Home Assistant has no media players at all, so there is nothing to choose between. |
| `ha_truncated` | Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below. |
| `cap_no_media` | Can't play media |
| `cap_no_volume` | No volume control |
| `cap_self` | This device |
| `player_offline` | Offline |
| `cl_aria` | FutureProofHomes community links |
| `cl_docs` | Docs |
| `cl_github` | GitHub |
| `cl_youtube` | YouTube |
| `cl_discord` | Discord |

The `cl_*` block is the community links row at the very bottom of Diagnostics (September 2026): four
quiet icon-and-word links under the ESP32 Recovery card — the documentation site
(docs.futureproofhomes.net), the FutureProofHomes GitHub organisation, the YouTube channel and the
Discord invite (discord.futureproofhomes.net). Each opens in a new tab; `cl_aria` names the row's
`<nav>` for screen readers, since it has no visible heading. Organisation links, not device links,
so controlling a peer remotely shows the same row.

`copied` was deleted along with the Logs card's Copy button. The button never worked honestly on this
origin - the Clipboard API needs a secure context, this app is served over plain HTTP, so it ran on a
deprecated `execCommand` fallback - and the owner cut it in favour of **Export** (briefly "Dump"),
which downloads the same thing copy produced: the lines on screen, filters and all, each line stamped
with its arrival time. Export Logs sits in the card header beside the level menu. The stream toggle
lives at the card's foot, to the right of the line count; it shows the state it is in - **Live**,
accent-blue, while lines flow; **Paused** once they are held - rather than the action it offers. The
level filter is a drawn menu rather than a native select (Safari's native popup ignores option
styling): the funnel icon sits inside the control, and the current level and every option wear the
same colours as the lines they admit.
The Show/Hide invert and the Clear button are gone too - the search fields' own ✕ clears them, and
Clear's real job (emptying the log buffer) is a page reload now.

`pipeline_off` and `pipeline_preferred` are labels rather than names, and both sit above the pipeline list in
every wake word's dropdown. Home Assistant stores `no_wake_word` for a slot holding nothing and `preferred`
for "whichever pipeline is marked preferred"; the dropdown has to show something readable for each, but
neither can be matched by its label, because a customer who names one of their own pipelines "Preferred" or
"Off" would otherwise get two indistinguishable entries and a wake word pointed at the wrong one. So both
travel as values and this is the only place their display text exists. Reusing `no_wake_word` rather than
inventing a sentinel for Off keeps the entire value space Home Assistant's.

`assistant_needs_ha` appears under the wake words when the four selects could not be read and no more
specific cause applies; `assistant_blocked` (actions checkbox off, with the **Show fix** link) and
`ha_too_old` take precedence when the `actions` verdict names one of them. All three say which half of
the card is affected on purpose — whether the device listens is the device's own setting and keeps working,
so a single "unavailable" over the whole card would be false. `assistant_blocked` additionally warns that
Home Assistant may push its own slot state back over locally flipped wake words when it reconnects, because
the slot sync that normally prevents that is itself an action call.

`assistant_slots_full` needs three or more wake words listening at once, which this product cannot reach: it
ships two models. It is written anyway because the payload and the card are both general, and because the
failure it describes is a dropdown quietly disagreeing with itself — Home Assistant sends anything it cannot
match to the first slot's assistant, so a third wake word works but shares, and its dropdown shows the shared
answer rather than whatever was last chosen for it.

`theme_to_dark` and `theme_to_light` are the light/dark button's accessible label and tooltip in the
header. The button itself is only an icon, so this text is the whole of its name.

`ha_refresh` and `ha_refreshing` are gone with the button they labelled. The device's cached list is now
synced once per load of the app, when the Config route first appears, rather than on demand — so the
gesture that fetches a newly added speaker is reloading the page. There is no interval behind it: the
device otherwise only re-asks Home Assistant five seconds after the native API connects. The device
switcher also asks for a sync each time it opens, so its availability dots are at most a couple of
seconds old.

## The verdict splash, the fix drawer, and the blocked toast

The splash is the overlay that holds the app's first paint after sign-in (or straight away for a
returning cookie) while the boot calls land. On the happy path nobody reads it - it fades in under a
second. The strings below are for the paths that need a person, and the star among them is the
blocked card: the "Allow the device to perform Home Assistant actions" checkbox, off for every newly
added ESPHome device, walked through step by step. The same walk-through reappears in the fix drawer,
opened by the blocked toast and by every in-place **Show fix** link, so the fix stays one tap away
after "Continue without Home Assistant actions".

Every line on the blocked card was read on a phone and cut down with the owner (September 2026) -
resist re-expanding them. The body leads with what ticking the box buys rather than what is broken.
Step 3 quotes the checkbox verbatim so it matches what Home Assistant renders. Step 2 comes in two
forms because of a catch-22: Home Assistant's display name for the device travels only over the
action channel the card exists to unblock, so on first onboarding only the firmware name is known
and the `_unnamed` hedge ("unless you renamed it") is honest, while a device that ever synced still
holds the real name and gets the short form. In both, `%c` is where the guide draws the mdi:cog
glyph inline - the same icon Home Assistant puts on the device's row, matched by sight - and `%s`
is the device's name. `blocked_open_ha` (centred, so it lines up with Continue into one action
column) carries two URLs: on phones a tap tries the companion app's own scheme first
(`homeassistant://navigate/config/integrations/integration/esphome`), which opens the app directly
at the integrations page with no interstitial tab and no internet needed; if no app claims it - or
on a desktop - it falls back to the My Home Assistant redirect
(`my.home-assistant.io/redirect/integration/?domain=esphome`), which opens the user's own
installation at the same page. Home Assistant has no URL that lands on the Configure dialog itself,
so the last two taps stay written out. The web fallback needs the browser to have internet; without
it, the steps stand alone.

The watching line under the button is one row beside a small spinning ring (the boot ring at text
size - spinning reads "actively checking" where the old breathing dot read as merely alive):
`blocked_watching`, then `blocked_recheck` as an inline link that runs the manual check, then a
period. It leads with "Advances automatically" because that is the fact people missed when the
sentence led with "watching". The Continue button under the card wears a real button face (the
ghost did not read as pressable) and its label admits only what is missing: `splash_continue_actions`
on the blocked and too-old cards, where Home Assistant itself is fine, and the plain
`splash_continue` on the rest.

| Key | Where | Text |
| --- | --- | --- |
| `splash_connecting` | status line, first moments | Connecting to your Satellite1… |
| `splash_asking` | status line, Home Assistant sync in flight | Asking Home Assistant which speakers you have… |
| `splash_connected` | status line, the beat before the fade after a recovery | Connected to Home Assistant. |
| `splash_slow` | card, after 15s of asking | This is taking longer than it should. The device may still be starting up, or Home Assistant may be slow to answer. |
| `splash_error` | card, device answering errors | The device answered with an error, so the app cannot start. |
| `splash_retry` | button under `splash_error` | Try again |
| `splash_continue` | button under the not-connected, slow and error cards | Continue without Home Assistant |
| `splash_continue_actions` | button under the blocked and too-old cards | Continue without Home Assistant actions |
| `splash_continue_sub` | line under that button | Your experience will be limited. |
| `blocked_title` | blocked card and fix drawer title | Allow Home Assistant actions |
| `blocked_body` | blocked card and fix drawer | One checkbox in Home Assistant lets this device list your areas and media players. |
| `blocked_step1` | step 1 | In Home Assistant, open Settings › Devices & services › ESPHome. |
| `blocked_step2` | step 2, name known to be Home Assistant's | Tap the %c cog next to %s. |
| `blocked_step2_unnamed` | step 2, firmware-name fallback | Tap the %c cog next to this device - %s, unless you renamed it. |
| `blocked_step3` | step 3 | Tick “Allow the device to perform Home Assistant actions”, then Submit. |
| `blocked_open_ha` | link button, centred | Open Home Assistant |
| `blocked_watching` | watching line, beside the small ring | Advances automatically - or |
| `blocked_recheck` | inline link ending the watching line | manually check again |
| `blocked_check` | button on the slow card | Check again |
| `noha_title` | not-connected card title | Not connected to Home Assistant |
| `noha_body` | not-connected card | The device has no connection to Home Assistant right now, so it cannot list your areas, media players, or other Satellite1 devices. Its own controls all work. This screen continues on its own when the connection returns. |
| `old_ha_title` | old-Home-Assistant card title | Home Assistant needs an update |
| `old_ha_body` | old-Home-Assistant card | This Home Assistant is older than 2025.12, which cannot answer the calls the app's speaker and assistant lists are built from. Update Home Assistant to use them - everything on the device itself works now. |
| `blocked_toast_t` | toast title, once on entering the app while blocked | Home Assistant actions are off. |
| `blocked_toast_s` | toast second line | Speaker lists, assistants and other devices are unavailable. Tap for the fix. |

The blocked and not-connected cards recover on their own: the app polls the device's cached verdict
every three seconds while one is up, and ticking the checkbox makes Home Assistant reload the device's
connection, which triggers a fresh sync - so the card melts into `splash_connected` and the fade
without anyone pressing anything. `blocked_check` exists for the impatient; the fix drawer closes
itself the same way.

## Confirmation modals (`CONFIRM`)

Every Diagnostics action that interrupts or erases asks first, in a modal: the title restates the
action as a question, the body says what happens next — how long, what stops working, what survives —
and the proceed button names the outcome rather than saying "OK". Cancel, the scrim, and Escape all
back out.

| Key | Title | Body |
| --- | --- | --- |
| `update` | Install this update? | The device downloads the new firmware and restarts itself when it finishes. The assistant and any audio stop until it is back - a few minutes. Keep it powered the whole time. |
| `xmos_restart` | Restart the audio chip? | The microphones and speaker drop out for a few seconds while it comes back. Nothing is erased and no settings change. |
| `xmos_flash` | Reflash the audio chip? | Rewrites the audio chip's firmware from a known-good copy. It takes about a minute, the microphones and speaker are silent throughout, and the device must stay powered. |
| `pw_change` | Change the password? | Every other signed-in browser, pasted sign-in link and QR code stops working the moment it changes. This browser stays signed in, and Home Assistant's Web UI Password sensor shows the new value. |
| `radar_restart` | Restart the radar module? | Presence detection drops out for a few seconds while the module comes back. Nothing is erased and no settings change. |
| `radar_factory` | Reset the radar to factory defaults? | Erases the settings stored on the radar module itself - detection range, gate thresholds and zones, everything tuned on the Presence page - and restarts it. This device's own settings are not touched, but the radar tuning cannot be brought back. |
| `restart` | Restart this device? | It reboots right away and is back in under a minute. All of your settings survive a restart. |
| `safe_mode` | Restart into safe mode? | The device comes back with only its network connection and update tools running - no assistant, no audio - so a bad update can be recovered. Restarting again returns it to normal. |
| `factory_reset` | Erase everything? | Every setting stored on this device is wiped, including its Wi-Fi credentials, and it restarts as if new from the box. You will have to set it up again. |
| `crash_erase` | Erase the crash history? | The recorded crashes, the pre-crash log and the crash dump are deleted from this device. If you are still chasing the cause, download the dump first - there is no way back to it. |

## Strings that are not in `copy.js`

A few short labels sit inline in the components because they are part of the layout rather than
explanations, and lifting them out would make the markup harder to read than it makes them easier to
edit:

- The calibration editor's two row labels — `sensor reads` and `offset`. It has no confirm button:
  every press is already written to the device, so it closes when you look away from it. (A third row,
  `shows`, restated the corrected value; it went because the pill above the editor is that number,
  live, as the offset moves.)
- `Press a button on the device; it lights up here.` under the Buttons card, now at the foot of the home page.
- `Waiting for the device to say something.` in an empty log.
- The confirm-modal proceed verbs: `Install <version>`, `Restart XMOS`, `Reflash now`, `Restart`,
  `Restart into safe mode`, `Erase everything`. They live at the call sites so they can carry the
  version number. `Erase it` went with the XMOS erase row.
- `Update available`, `Install <version>` and `Release notes` in the firmware update panel, plus `Up to
  date` under the Sat1 firmware version. The panel appears only when there is an update; the version is on
  the button rather than described beside one, because pressing it is the whole point of the panel.
- `Local Speaker`, `No Area Assigned`, `whole area` and the `n/m` counts in the two trees. All four are
  structure rather than explanation - the first two are row labels and the last two are state readouts.
- `Play assistant audio on selected players` and `Lower the volume on selected players upon wake word
  detection`, the two tree headings. Both say what ticking something does, and leave the rest to the card's
  ⓘ. The second was `Quieten while talking`, which was inaccurate: the duck also covers the listening half
  of the interaction. It wraps to two lines at every phone width, which is deliberate — the alternative was
  a heading that stopped short of naming the trigger. The first was `Play assistant responses on selected
  players`, widened alongside the card's retitle from `Remote routing` to `Audio routing` when the feature
  grew past responses (sign-in prompts, timer rings, the chime); the Home Assistant entities keep their
  legacy TTS names, since renaming an ESPHome entity orphans it.
- The five route names in the nav drawer — `Home`, `Wake Word`, `Audio`, `Presence`, `Diagnostics`, in
  that order — which are the same strings as the routes themselves. The drawer has no heading; the
  device name is in the bar above it.
- `Connected` and `Nothing plugged in` on Speaker amplifier's Line out row, and the three volume sliders'
  zero readouts: `follow media` on Voice Volume Override, `follow device` on Remote TTS volume (zero means
  "leave every target's volume alone", per tts_routing.yaml), and `mute` on Duck volume — deliberately
  not "follow device", because area_ducking.yaml is explicit that zero is literal there: ducked players
  are set to 0% for the length of the interaction. All are readouts of a value rather than descriptions
  of a control.
- The Speaker amplifier card's own readouts: the Analog gain slider speaks dBV, with the factory
  default (15 dBV) marked as a notch on the track that the drag snaps to rather than named in the
  readout — "where was it before I touched it" never needs support, and the hint spells the number
  out. Digital volume reads `Muted` instead of a misleading `0%` while the DVC mute is engaged, and
  Power gain mode words its states as `High gain · Running from the USB-PD supply`, `Low gain ·
  Running from the 5 V rail`, `Off · Line out selected or amplifier shut down` and `Measuring… ·
  Sampling the power supply` — an unexpected mode shows raw as `PWR_MODE N` so a future firmware
  that uses one reaches the screen without an app release.
- The USB-C Power Supply row re-words the entity it reads: the Home Assistant sensor keeps the
  firmware's contract string (`3.25A (max) @ 20V`, untouched because automations may parse it), while
  the app shows `20V @ 3.25A~` as the headline with the wattage spelled out beneath (`65 watts`) — the
  trailing ~ carries what `(max)` meant: the current is the charger's ceiling, not a live draw. A
  contract string the app does not recognise shows raw rather than hiding.

The wake word switch labels are not copy at all: they are the wake words themselves, read from the device
at `GET /api/sat1/wakewords`, which reports each model's friendly name from the manifest it was built from.
So the rows on a device with a custom model are named by that model, and nothing here needs changing for it.

The voice assistant phase names — Idle, Waiting for a command, Listening, Thinking, Replying, Not
ready, Error — are in `frontend/src/lib/device.js`, keyed by the `voice_assist_*_phase_id`
substitutions in `config/common/voice_assistant.yaml`. They have to stay aligned with those numbers.
They sit in the header of the Assistant card on the home page, above the transcript: the phase and the
words it produced belong together. The transcript rows are labelled `User` and `Assist` — the two
parties — rather than the earlier `heard` and `said`, which read as verbs about the device.
