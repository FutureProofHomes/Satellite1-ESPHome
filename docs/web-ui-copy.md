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
| `voice_override` | Assistant card, Assistant volume | How loud this device speaks when the assistant replies, separate from media volume. Zero follows the media volume instead. Speakers you route answers to have their own level - Remote TTS volume, on the Audio page. |
| `temp` | Calibration editor, title row | Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room. |
| `humidity` | Calibration editor, title row | Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room. |
| `lux` | Calibration editor, title row | Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger. |
| `calibrate` | Calibration editor, offset row | Adjust until the reading matches a trusted instrument in the same room. The correction is stored on the device and survives restarts. |
| `led_ring` | LED ring | The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error. |
| `timers` | Timers card | Timers set by voice, held on the device - they keep counting and still ring if Home Assistant goes away. Voice is also how they are managed: name one when you set it ("set a pizza timer for ten minutes"), pause or cancel it the same way. |
| `media` | Media footer, expanded view | What this device is playing, from either source: a group stream, or media sent to it by Home Assistant. Track skipping, shuffle, repeat and the progress bar work on group streams - media sent directly to this speaker is one URL with none of that. The volume here is the media volume; the assistant's voice has its own level, in the Assistant card on the home page. |
| `media_group` | Media footer, Grouped speakers | Speakers playing this stream together, with each one's own volume. Adding a speaker asks Music Assistant to bring it into the group; removing one lets it fall silent. The big slider above moves the whole group. |
| `ma_connect` | Media footer, Music Assistant panel | Connecting this page straight to your Music Assistant server makes the controls instant and unlocks anything Home Assistant cannot relay. Create a long-lived token in Music Assistant under Settings, then your profile, and paste it here with the server's address. Both stay in this browser only - the device never sees them. |

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
| `esp_temp` | ESP32 Temp | The ESP32 chip's own temperature, not the room's. It reads well above ambient inside the sealed case, so warm is normal; sustained readings above 80 °C are worth investigating. The room's temperature is on the home page. |
| `reset` | Last restart | Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware. |
| `xmos` | XMOS firmware, and the XMOS Recovery card title | The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's. |
| `xmos_flash` | Reflash XMOS vX.X.X | Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power. |
| `maintenance` | ESP32 Recovery card title | Ways to restart or reset this device. None of them are part of everyday use. Restart is always safe - your settings survive it. Factory reset is the only row here that erases anything. |
| `safe_mode` | Safe mode | Restarts with everything but Wi-Fi and the updater switched off. Use it when the device is crash-looping too fast to accept an update. |
| `factory_reset` | Factory reset | Erases every setting stored on the ESP32, including the Wi-Fi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched. |
| `beta` | Beta updates | Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on. |
| `log` | Logs card title | The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request. |

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

### Wake Word

Its own route since the September 2026 pass: these rows are the first thing a new owner goes looking
for, and they were the third card down a route named Audio. One card, in the owner's row order - the
wake word dropdowns, then **Wake words sensitivity** (plural, renamed from "Wake word sensitivity":
it is one sensitivity shared by every word above), Wake chime, and Say "stop" to interrupt last.

| Key | Where | Text |
| --- | --- | --- |
| `wake_words` | first wake word row | Which wake words this device answers, and which assistant answers each one. Off stops it responding to that word and frees processing for the ones you use. The assistants are your Home Assistant voice pipelines - Preferred follows whichever is marked preferred there. Home Assistant stores the pairing, so it must be reachable to change one. |
| `wake_sensitivity` | Wake words sensitivity | How readily the wake word fires. Raise it if the device misses you from across the room; lower it if the television sets it off. |
| `wake_sound` | Wake chime | Plays a short chime on the speaker the moment the wake word is detected. |
| `stop_word` | Say "stop" to interrupt | While an answer is playing - this device’s own, or one another Satellite1 routed here - saying "stop" cuts it off everywhere it is playing. A ringing timer can always be silenced this way, whichever way this is set. |

`wake_words` is one hint for the whole group, on the first row, rather than the same sentence repeated with
a different wake word in it. The switches it describes are the only control in the app with no entity
behind them: `micro_wake_word` creates neither a switch nor a select for its models, so they never reach
`/events`, and the list comes from `GET /api/sat1/wakewords`.

It also dropped an earlier claim — "Home Assistant can change these too, and both are reading the same
setting" — which was true about the stored flag and misleading about everything else. Home Assistant keeps
which assistant answers which wake word, in two slots of its own, and it answers any change to one of those
slots by pushing the whole active set back to the device. So the app clears a wake word's slot when that
word is switched off and claims one when it is switched on, and the hint says as much, because giving up a
pairing is a consequence the old wording implied did not exist.

Each wake word is **one dropdown**, not a switch and a separate assistant row. `Off` is the first item,
then `Preferred`, then the customer's pipelines sorted case-insensitively. There is only one decision on
that row — whether this device answers to this word, and if so which assistant answers — and that is the
shape Home Assistant stores, since a slot either names a wake word and a pipeline or holds `no_wake_word`
and means nothing. Two controls made it look like two settings, one of which could contradict the other.
An earlier attempt did have two rows, with a second hint called `assistant`; it is gone, and its content is
folded into `wake_words`.

The row label quotes the wake word — `"Hey Jarvis" wake word` — because it is a phrase someone says out
loud rather than the name of a setting. Unquoted beside a dropdown full of assistant names, `Hey Jarvis`
reads like another one of them.

With Home Assistant unreachable the row falls back to a plain on/off toggle, because there are no assistants
to list and a dropdown holding one real option would be a worse lie than a switch. `assistant_needs_ha`
says what that costs.

### Audio

The route was called Config until the September 2026 rename pass. Mute microphones and Assistant
volume left for the home page, and the wake words card became the Wake Word route above, so what
remains is where sound goes: the two trees first, and Audio Output last - channel and line-out are
wiring you set once at install, where the trees are revisited whenever the household's speakers
change.

Two controls here still write to entities in `config/common/tts_routing.yaml` and
`config/common/area_ducking.yaml`, so their wording has to agree with what Home Assistant shows. The
two trees do not: they write the device's own selection at `/api/sat1/sel`, and they are the only face
that selection has.

| Key | Where | Text |
| --- | --- | --- |
| `speaker_channel` | Audio Output, Channel | Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want. |
| `remote_routing` | Remote routing card title | Plays the assistant's spoken answers on other speakers as well as this one. Tick a room to include every player in it, or open the room and pick players. Local Speaker is this device's own speaker - untick it and answers play only where you have chosen. |
| `area_ducking` | Area ducking card title | Turns other speakers down while the assistant listens and answers, then puts them back where they were. Tick a room to cover every player in it. |
| `remote_tts_volume` | Remote TTS volume | How loud answers are on the remote speakers. This device's own level is Assistant volume, on the home page. Sonos reads the level from the announcement itself; anything else has its volume set for the answer and put back afterwards. |
| `remote_wake_chime` | Remote wake chime | Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to. |
| `duck_volume` | Duck volume | The level they drop to. Players already quieter than this are left alone, so a whole-house group does not get turned up. |

`voice_override` and `remote_tts_volume` are the pair most easily confused, and with Assistant volume
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
| `switcher` | Device switcher sheet title | Each Satellite1 serves its own copy of this page, so everything you change applies to the device named above. Use this list to move to another device without typing its address - you stay on the page you are on. |

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
| `no_devices` | Only this device. Other Satellite1s appear here once Home Assistant lists them. |
| `peer_up` | Available, according to Home Assistant |
| `peer_down` | Unavailable, according to Home Assistant |

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
| `nothing_said` | A transcription of your voice interactions are displayed here. |
| `media_idle` | Nothing playing. Group streams and media sent from Home Assistant show up here. |
| `media_idle_bar` | Nothing playing |
| `media_src_group` | group stream |
| `media_src_local` | this speaker |
| `media_players_title` | Players |
| `media_add_speaker` | Add a speaker |
| `media_group_loading` | Asking Music Assistant… |
| `media_no_tiers` | Grouping, favorites and seeking need Home Assistant with Music Assistant - or connect this page to your Music Assistant server from the expanded media view. |
| `media_like` | Add to favorites |
| `media_liked` | Added to favorites |
| `ma_title` | Music Assistant |
| `ma_url_ph` | http://music-assistant.local:8095 |
| `ma_token_ph` | Long-lived token |
| `ma_connect_btn` | Connect |
| `ma_disconnect_btn` | Disconnect |
| `ma_connected` | Connected to Music Assistant |
| `ma_error` | Could not connect. Check the address and token, and that this browser can reach the server. |
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
| `assistant_slots_full` | Home Assistant can pair only two wake words with an assistant of their own. The rest are answered by the first one's assistant, which is what their dropdowns show. |
| `confirm` | Confirm |
| `cancel` | Cancel |
| `theme_to_dark` | Switch to dark theme |
| `theme_to_light` | Switch to light theme |
| `ha_pending` | Asking Home Assistant which speakers you have. |
| `ha_never` | Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns. |
| `ha_refused` | Home Assistant did not answer. Either it is older than 2025.12, or this device is not allowed to perform actions: Settings › Devices & services › ESPHome › this device › CONFIGURE, then tick “Allow the device to perform Home Assistant actions”. |
| `ha_no_area` | This device is not in a Home Assistant area, so “Route TTS To All Area Players” and “Duck All Area Players” have no room to refer to. You can still pick any room below. Assign it to an area in Home Assistant and refresh. |
| `ha_no_players` | Home Assistant has no media players at all, so there is nothing to choose between. |
| `ha_truncated` | Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below. |
| `cap_no_media` | Can't play media |
| `cap_no_volume` | No volume control |
| `sel_failed` | That change was not saved. The device rejected it, or the connection dropped. |

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

`assistant_needs_ha` appears under the wake words when the four selects could not be read at all: Home
Assistant unreachable, older than 2025.10, or those entities disabled in its registry. It says which half of
the card is affected on purpose — whether the device listens is the device's own setting and keeps working,
so a single "unavailable" over the whole card would be false.

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
| `restart` | Restart this device? | It reboots right away and is back in under a minute. All of your settings survive a restart. |
| `safe_mode` | Restart into safe mode? | The device comes back with only its network connection and update tools running - no assistant, no audio - so a bad update can be recovered. Restarting again returns it to normal. |
| `factory_reset` | Erase everything? | Every setting stored on this device is wiped, including its Wi-Fi credentials, and it restarts as if new from the box. You will have to set it up again. |

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
- `Play assistant responses on selected players` and `Lower the volume on selected players upon wake word
  detection`, the two tree headings. Both say what ticking something does, and leave the rest to the card's
  ⓘ. The second was `Quieten while talking`, which was inaccurate: the duck also covers the listening half
  of the interaction. It wraps to two lines at every phone width, which is deliberate — the alternative was
  a heading that stopped short of naming the trigger.
- The five route names in the nav drawer — `Home`, `Wake Word`, `Audio`, `Presence`, `Diagnostics`, in
  that order — which are the same strings as the routes themselves. The drawer has no heading; the
  device name is in the bar above it.
- `Connected` and `Nothing plugged in` on Audio Output's Line out row, and the three volume sliders'
  zero readouts: `follow media` on Assistant volume, `follow device` on Remote TTS volume (zero means
  "leave every target's volume alone", per tts_routing.yaml), and `mute` on Duck volume — deliberately
  not "follow device", because area_ducking.yaml is explicit that zero is literal there: ducked players
  are set to 0% for the length of the interaction. All are readouts of a value rather than descriptions
  of a control.

The wake word switch labels are not copy at all: they are the wake words themselves, read from the device
at `GET /api/sat1/wakewords`, which reports each model's friendly name from the manifest it was built from.
So the rows on a device with a custom model are named by that model, and nothing here needs changing for it.

The voice assistant phase names — Idle, Waiting for a command, Listening, Thinking, Replying, Not
ready, Error — are in `frontend/src/lib/device.js`, keyed by the `voice_assist_*_phase_id`
substitutions in `config/common/voice_assistant.yaml`. They have to stay aligned with those numbers.
They sit in the header of the Assistant card on the home page, above the transcript: the phase and the
words it produced belong together. The transcript rows are labelled `User` and `Assist` — the two
parties — rather than the earlier `heard` and `said`, which read as verbs about the device.
