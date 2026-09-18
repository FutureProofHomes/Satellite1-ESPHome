/**
 * Every explanatory string in the app, in one file.
 *
 * Two reasons it is not scattered through the components. It is the thing most likely to be edited
 * by someone who does not write Preact, and it is mirrored into docs/web-ui-copy.md so support can
 * quote the device's own wording back to a customer. Keep the two in step.
 *
 * House style: say what the control does to the device, not what the widget is. A hint that reads
 * "toggles the microphone mute state" has told the reader nothing they could not see.
 */

export const HINTS = {
  // tts_routing, tts_local_speaker, tts_targets and tts_manual_ids are gone. The first two described
  // switches that no longer exist; the tree is now the whole control and explains itself by being a
  // list of rooms with "Local Speaker" at the top of it. The manual id field was replaced by the "No
  // Area Assigned" group, which shows those players by name instead of asking for their entity ids.

  /* The switcher sheet. What needs saying is not "this is a list of devices", which is visible, but that
     each device has its own copy of this app - the single fact that makes the whole page make sense, and
     the one someone who has just changed a setting on the wrong device wishes they had been told. Does not
     repeat why the list has one entry in it; that sentence is already at the foot of the same sheet. */
  switcher:
    "Each Satellite1 serves its own copy of this page, so everything you change applies to the device named above. Use this list to move to another device without typing its address - you stay on the page you are on.",

  /* On the card title rather than a row, because what needs explaining is the tree, and the tree has no
     row of its own to hang an ⓘ off. This is also the only place Local Speaker is explained: it is the
     one tick in there whose effect is not obvious from its name, since it silences this device rather
     than adding a target. */
  remote_routing:
    "Plays the assistant's spoken answers on other speakers as well as this one. Tick a room to include every player in it, or open the room and pick players. Local Speaker is this device's own speaker - untick it and answers play only where you have chosen.",

  /* The last sentence used to be three sentences of its own paragraph under the slider. It is here
     because it explains the control rather than the page, and a permanent paragraph for something you
     need to read once is what a tooltip is for. Kept because one slider with three behaviours behind it
     is the kind of thing that gets reported as a bug. */
  remote_tts_volume:
    "How loud answers are on the remote speakers. This device's own level is Assistant volume, on the home page. Sonos reads the level from the announcement itself; anything else has its volume set for the answer and put back afterwards.",

  remote_wake_chime:
    "Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to.",

  // duck_area, duck_players and duck_tts_targets are gone for the same reason. Ducking is now the same
  // tree, and the answer to "which players" is visibly the thing being ticked.

  /* The window is stated here and nowhere else. It is wider than people assume: the duck goes out on the
     voice assistant's on_start, which is the wake word, and the volumes come back on on_end - so it is
     quiet while it listens to you, not only while it answers. The old label said "while talking", which
     described half of it. */
  area_ducking:
    "Turns other speakers down while the assistant listens and answers, then puts them back where they were. Tick a room to cover every player in it.",

  /* Deliberately does not repeat when the ducking happens - that is on the card above this row, and
     saying it twice made the two bubbles look like they were describing different things. What is left is
     the part that only applies to this slider: it is a floor, not a reduction. */
  duck_volume:
    "The level they drop to. Players already quieter than this are left alone, so a whole-house group does not get turned up.",

  // "and the offset is stored on the device" used to end this. It is what `calibrate` says two rows below
  // it in the same editor, so it was cut. The instrument to compare against stays: that part is specific to
  // this sensor, and the generic hint cannot name it.
  temp: "Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room.",

  humidity:
    "Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room.",

  lux: "Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger.",

  presence: "Sensed by radar, not the microphones. It works through the case and notices people even when they sit completely still.",

  /* The radar's own settings, on Presence. Rewritten plain and short at the owner's request: one idea
     per hint, no jargon, and the "why you'd change this" up front. */
  radar_range: "How far the radar looks. Anything farther away is ignored - use it to stop the radar seeing into the hallway or the next room. All the way left is the full 6m reach.",
  radar_stability: "How sure the radar must be before it changes its answer. Higher is steadier but slower to notice changes. Lower is quicker but can flicker.",
  radar_timeout: "How long the room still counts as occupied after the radar loses sight of everyone. Set it high enough that the lights stay on while you sit still.",
  radar_multi: "Track up to three people at once instead of only the clearest one.",
  radar_bt: "The radar's own Bluetooth, used only by the manufacturer's phone app. Leave it off unless you use that app.",
  radar_zones: "Draw shapes on the map above. Each zone reports on its own whether someone is inside it. The ignore area is the opposite: anything in it never counts - useful for fans and curtains.",
  radar_resolution: "How finely distance is split into the nine rows above: 0.75m steps reach the whole room, 0.2m steps reach less far but with more detail up close. Changing it changes what each row means, so re-check your levels after.",
  gate_move: "How much movement the radar sees at each distance, live. Drag a notch to set the trigger level for that distance - anything above it counts as a person moving.",
  gate_still: "How much tiny motion - like breathing - the radar sees at each distance, live. Drag a notch to set the trigger level - anything above it counts as a person holding still.",
  gate_max_move: "The farthest distance that counts for movement. Rows past it dim in the chart above and are ignored.",
  gate_max_still: "The farthest distance that counts for stillness. Rows past it dim in the chart above and are ignored.",
  zone_excl:
    "Makes this shape an ignore area instead of a zone: anything inside it never counts as presence. Useful for fans, curtains and pets. There is one ignore area - saving a new one replaces it.",

  // "Enter what a trusted instrument reads" was wrong twice over: the control is a ± stepper, not a
  // field, and what you adjust is the correction, not the instrument's number.
  calibrate:
    "Adjust until the reading matches a trusted instrument in the same room. The correction is stored on the device and survives restarts.",

  mute: "Cuts the microphones in hardware, not software - wake word detection stops with them. The mute button on the device does the same thing.",

  wake_sound: "Plays a short chime on the speaker the moment the wake word is detected.",

  // One hint for the whole group, on the first row, rather than one per wake word saying the same thing
  // with a different name in it.
  //
  // Covers the whole control, because the control is now one dropdown rather than a switch and a separate
  // assistant row that each had their own bubble. It also drops an earlier claim - "Home Assistant can
  // change these too, and both are reading the same setting" - which was true about the stored flag and
  // misleading about everything else: Home Assistant answers any change to a pairing by pushing the whole
  // active set back to the device, so switching a wake word off does cost something.
  wake_words:
    "Which wake words this device answers, and which assistant answers each one. Off stops it responding to that word and frees processing for the ones you use. The assistants are your Home Assistant voice pipelines - Preferred follows whichever is marked preferred there. Home Assistant stores the pairing, so it must be reachable to change one.",

  // Names the two directions it reaches - answers this device sends elsewhere and answers other
  // devices send here - because the remote half is the invisible one, and names the timer case
  // because it works whichever way this is set and a reader who just turned this off should not
  // think they lost it.
  stop_word:
    'While an answer is playing - this device\u2019s own, or one another Satellite1 routed here - saying "stop" cuts it off everywhere it is playing. A ringing timer can always be silenced this way, whichever way this is set.',

  wake_sensitivity:
    "How readily the wake word fires. Raise it if the device misses you from across the room; lower it if the television sets it off.",

  // Names the speaker it moves, and names the other slider by its on-screen label and page, because
  // the pair live on different routes again (this one in the home page's Assistant card, the other in
  // Remote routing on Audio) and the hints are the only thing keeping them apart.
  voice_override:
    "How loud this device speaks when the assistant replies, separate from media volume. Zero follows the media volume instead. Speakers you route answers to have their own level - Remote TTS volume, on the Audio page.",

  speaker_channel:
    "Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want.",

  led_ring:
    "The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error.",

  timers:
    'Timers set by voice, held on the device - they keep counting and still ring if Home Assistant goes away. Voice is also how they are managed: name one when you set it ("set a pizza timer for ten minutes"), pause or cancel it the same way.',

  media:
    "What this device is playing, from either source: a group stream, or media sent to it by Home Assistant. Track skipping, shuffle, repeat and the progress bar work on group streams - media sent directly to this speaker is one URL with none of that. The slider on the media bar is the media volume; the assistant's voice has its own level, in the Assistant card on the home page.",

  // The grouped-speakers section of the expanded media view. Speaks in Music Assistant's terms
  // because grouping is its feature - the device only relays the request.
  media_group:
    "Speakers playing this stream together, with each one's own volume. Adding a speaker asks Music Assistant to bring it into the group; removing one lets it fall silent. With two or more grouped, the Group volume slider moves them all together.",

  // The Music Assistant connection panel. What the token is, where it comes from, and where it goes -
  // which is only this browser, a fact worth stating because pasting tokens into web pages should
  // make people ask.
  ma_connect:
    "Connecting this page straight to your Music Assistant server makes the controls instant and unlocks anything Home Assistant cannot relay. Create a long-lived token in Music Assistant under Settings, then your profile, and paste it here with the server's address. Both stay in this browser only - the device never sees them.",

  heap: "Internal RAM still available. Audio buffers and the network stack allocate from it, and it runs out long before PSRAM does.",

  psram:
    "External RAM, used for the large buffers. Free and total cover the region the firmware can allocate from, which is smaller than the chip installed.",

  loop: "The longest single pass through the main loop since the last reading. Tens of milliseconds is normal; sustained hundreds means something is blocking, and audio will stutter first.",

  esp_temp:
    "The ESP32 chip's own temperature, not the room's. It reads well above ambient inside the sealed case, so warm is normal; sustained readings above 80 \u00b0C are worth investigating. The room's temperature is on the home page.",
  reset: "Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware.",

  xmos: "The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's.",

  xmos_flash:
    "Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power.",

  // xmos_erase was here. The row it explained is gone from the app: erasing leaves the audio chip blank,
  // which takes the microphones, the speaker and the wake word with it, and the only way back is the
  // Reflash row - which needs the chip it just erased to be talking. Reflash overwrites anyway.

  // On the ESP32 Recovery card's title. The card was "Sat1 Device", which described the page rather
  // than the card; the ⓘ says what actually unites the rows, and calms the two scary ones.
  maintenance:
    "Ways to restart or reset this device. None of them are part of everyday use. Restart is always safe - your settings survive it. Factory reset is the only row here that erases anything.",

  safe_mode:
    "Restarts with everything but Wi-Fi and the updater switched off. Use it when the device is crash-looping too fast to accept an update.",

  factory_reset:
    "Erases every setting stored on the ESP32, including the Wi-Fi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched.",

  beta: "Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on.",

  log: "The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request.",
};

export const TEXT = {
  ha_connected: "Connected to Home Assistant",
  ha_disconnected: "Not connected to Home Assistant",
  ha_disconnected_detail:
    "Everything on this page still works - it talks to the device directly. Media and anything that needs your smart home will be unavailable until the connection returns.",

  stream_lost: "Lost the connection to the device. Retrying.",

  // Only when the sheet has no peer rows at all. It used to also name the add-by-address field that
  // ended the sheet; the field is gone (owner's call), so Home Assistant's roster is the one way a
  // row appears and the sentence says only that.
  no_devices: "Only this device. Other Satellite1s appear here once Home Assistant lists them.",

  // Titles on the peer dots. Attributed to Home Assistant rather than stated as fact, because that is
  // the only witness: the page cannot probe a peer itself, and its view can lag a reboot by seconds.
  // peer_manual, peer_add_ph and peer_add went with the manual add-by-address feature.
  peer_up: "Available, according to Home Assistant",
  peer_down: "Unavailable, according to Home Assistant",

  // The failed-write toast, shown on any route. First line is the one fact every failure shares;
  // the control involved has already put its old value back, which is why "hasn't been applied"
  // needs saying - the reverted switch alone reads as a page that ignored the tap. Second line is
  // where the specific reason lives.
  write_failed: "A change didn't reach the device, so it hasn't been applied.",
  write_failed_go: "Tap for the device log.",

  // Shown in place of the transcript, not instead of the card. The card carries the assistant's phase in
  // its header, so it has something to say from the moment the device answers; this only fills the space
  // where the lines will go. Reworded to the owner's text, September 2026.
  nothing_said: "A transcription of your voice interactions are displayed here.",

  // The media footer's idle line. The footer stays on every page - the volume inside still sets the
  // level whatever plays next arrives at - so this fills the space the track title will use.
  media_idle: "Nothing playing. Group streams and media sent from Home Assistant show up here.",
  // The collapsed bar's version, short enough to share a 56px bar with the play button.
  media_idle_bar: "Nothing playing",

  /* The expanded media view. */
  // The slide-up the bar's speaker button opens, named what Music Assistant names its own
  // (owner's reference screenshots, September 2026). Replaced media_group_title ("Grouped
  // speakers") when the group section moved out of the expanded view and into this panel.
  media_players_title: "Players",
  media_add_speaker: "Add a speaker",
  // While the tier that serves the member list has not answered yet.
  media_group_loading: "Asking Music Assistant\u2026",
  // When no tier can answer at all: Home Assistant never delivered a Music Assistant player for
  // this device (not connected, actions off, or no MA install) and no direct connection is set up.
  // Names the way out - the connection fold at the bottom of the expanded view, since this line
  // renders in the players panel where "below" would point at nothing.
  media_no_tiers:
    "Grouping and seeking need Home Assistant with Music Assistant - or connect this page to your Music Assistant server from the expanded media view.",
  // media_like / media_liked went with the favorites feature, cut app-wide September 2026.
  // The players panel's whole-group slider, shown only while two or more speakers are grouped -
  // named so it cannot be mistaken for any one speaker's row below it.
  media_group_volume: "Group volume",

  /* The Music Assistant connection panel. */
  ma_title: "Music Assistant",
  ma_url_ph: "http://music-assistant.local:8095",
  ma_token_ph: "Long-lived token",
  ma_connect_btn: "Connect",
  ma_disconnect_btn: "Disconnect",
  ma_connected: "Connected to Music Assistant",
  ma_error: "Could not connect. Check the address and token, and that this browser can reach the server.",

  /* The Presence route: zone editing, gate thresholds, and the no-sensor card. The zi_* strings are
     the one instruction line under the plot, swapped by what the person is doing right now - one short
     sentence about the next step, instead of a paragraph explaining every gesture at once. */
  zi_first: "Tap the map to place the first corner.",
  zi_more: "Keep tapping. A zone needs at least 3 corners.",
  zi_adjust: "Drag a corner to reshape. Drag the middle to move the shape. Tap a corner to select it.",
  zi_selected: "Corner selected. Tap Remove corner below to delete it.",
  zones_set: "Tap a zone button, or a shape on the map, to edit it.",
  zones_none: "No zones yet. The whole field of view counts as presence.",
  gate_thresholds_help:
    "Each row is a band of distance. The bar shows what the radar sees there right now; drag the notch to set where it triggers. Dimmed rows are out of range and ignored. Changes save automatically.",
  // The owner's wording, verbatim, for a Sat1 with no module fitted. Split into segments because the
  // two links render as anchors; the URLs live here too so the copy review sees the whole sentence.
  no_sensor_lead: "A presence sensor was not detected in your Sat1. Please ",
  no_sensor_docs: "read our docs to learn more",
  no_sensor_docs_url: "https://docs.futureproofhomes.net/satellite1-presence-sensors/#connecting-mmwave-sensors",
  no_sensor_mid: ", you can purchase a presence sensor ",
  no_sensor_buy: "here",
  no_sensor_buy_url: "https://futureproofhomes.net/products/ld2450-mmwave-human-presence-sensor",
  // The source labels in the card header. "Group stream" rather than the protocol's name, because
  // what the customer configured is Music Assistant or something like it, and the fact it speaks
  // Sendspin underneath is not their vocabulary.
  media_src_group: "group stream",
  media_src_local: "this speaker",

  // Only when every wake word is off. Phrased as a consequence rather than a warning: it is a reasonable
  // thing to want, and the way back is the switches directly above it.
  no_wake_words: "No wake words are on, so the device will not respond to being spoken to. The mute button and Home Assistant still work.",

  // The first two entries of each wake word's dropdown, both labels for values rather than pipeline names.
  // `no_wake_word` is what Home Assistant stores in a slot holding nothing, and `preferred` means whichever
  // pipeline Home Assistant has been told to prefer. Labelled rather than matched by name so that a
  // customer who calls one of their own pipelines "Preferred" still gets two distinguishable entries.
  pipeline_off: "Off",
  pipeline_preferred: "Preferred",

  // Shown under the wake words when the assistant selects could not be read: Home Assistant unreachable,
  // too old, or its four selects disabled. Says which half of the card is affected, because whether the
  // device listens is its own to answer and that half keeps working - as a switch, since with no
  // assistants to list a dropdown would have nothing in it.
  assistant_needs_ha:
    "Home Assistant keeps which assistant answers each wake word, so that cannot be set from here until it is reachable. Turning a wake word on and off is the device's own setting and still works.",

  // Only with three or more wake words on at once, which needs a build with more models than this product
  // ships. Home Assistant has two pairings, and sends anything it cannot match to the first one - so the
  // surplus words work and share an assistant, and their dropdowns show the shared answer rather than
  // whatever was last picked for them.
  assistant_slots_full:
    "Home Assistant can pair only two wake words with an assistant of their own. The rest are answered by the first one's assistant, which is what their dropdowns show.",

  confirm: "Confirm",
  confirm_title: "Are you sure?",
  cancel: "Cancel",

  /* The Home Assistant data layer. Each of these is a different reason the area and player lists are
     not here, and each has a different fix - which is the whole point of not collapsing them into one
     "unavailable" message. */
  ha_pending: "Asking Home Assistant which speakers you have.",

  ha_never:
    "Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns.",

  ha_refused:
    "Home Assistant did not answer. Either it is older than 2025.12, or this device is not allowed to perform actions: Settings \u203A Devices & services \u203A ESPHome \u203A this device \u203A CONFIGURE, then tick \u201CAllow the device to perform Home Assistant actions\u201D.",

  ha_no_area:
    "This device is not in a Home Assistant area, so \u201CRoute TTS To All Area Players\u201D and \u201CDuck All Area Players\u201D have no room to refer to. You can still pick any room below. Assign it to an area in Home Assistant and refresh.",

  ha_no_players:
    "Home Assistant has no media players at all, so there is nothing to choose between.",

  /* Why a greyed player row in a target tree cannot be ticked - it lacks the capability the tree's
     call needs (media_player.play_media for routing, media_player.volume_set for ducking). One
     short reason per tree, not per integration: the row has no room for a diagnosis, and "what this
     speaker cannot do" is the part its owner can act on. */
  cap_no_media: "Can't play media",
  cap_no_volume: "No volume control",

  ha_truncated:
    "Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below.",

  /* The selection endpoint refused a write. Shown as a banner rather than a silent revert, because the
     checkbox has already moved back and that on its own looks like a page that ignores clicks. */
  sel_failed: "That change was not saved. The device rejected it, or the connection dropped.",

  /* The theme toggle. Deliberately not offering an "Auto" that follows the phone, because the device is
     often used in a room whose lighting has nothing to do with what the phone last decided. Phrased as
     an action rather than a state: the button shows the theme it switches to, and a bare "Dark theme"
     read aloud gives no clue whether that is the current setting or the one on offer. */
  theme_to_dark: "Switch to dark theme",
  theme_to_light: "Switch to light theme",
};

/**
 * The "Are you sure?" modals on Diagnostics, one per action that interrupts or erases.
 *
 * `t` is the modal's question, `b` the sentence or two under it. The question restates the action so
 * a mis-tap is caught by reading either line; the body says what actually happens next - how long,
 * what stops working, what survives - because these six buttons are exactly the ones whose
 * consequences a person cannot be expected to know. The proceed button's label lives at the call
 * site, where it can carry the version number ("Reflash XMOS v1.2.3").
 */
export const CONFIRM = {
  update: {
    t: "Install this update?",
    b: "The device downloads the new firmware and restarts itself when it finishes. The assistant and any audio stop until it is back - a few minutes. Keep it powered the whole time.",
  },
  xmos_restart: {
    t: "Restart the audio chip?",
    b: "The microphones and speaker drop out for a few seconds while it comes back. Nothing is erased and no settings change.",
  },
  xmos_flash: {
    t: "Reflash the audio chip?",
    b: "Rewrites the audio chip's firmware from a known-good copy. It takes about a minute, the microphones and speaker are silent throughout, and the device must stay powered.",
  },
  restart: {
    t: "Restart this device?",
    b: "It reboots right away and is back in under a minute. All of your settings survive a restart.",
  },
  safe_mode: {
    t: "Restart into safe mode?",
    b: "The device comes back with only its network connection and update tools running - no assistant, no audio - so a bad update can be recovered. Restarting again returns it to normal.",
  },
  factory_reset: {
    t: "Erase everything?",
    b: "Every setting stored on this device is wiped, including its Wi-Fi credentials, and it restarts as if new from the box. You will have to set it up again.",
  },
};

/**
 * Short forms of the radar's target state, for the sensor pill on the home page.
 *
 * The firmware's own words are Approaching, Moving Away, Still and Clear on an LD2450, and Clear, Moving
 * and Still on an LD2410. Those are the right words for Home Assistant, where a row is as wide as the
 * screen. They do not fit a quarter of a phone: measured in the shipped font, "Approaching" wants 103px
 * at the pill's 17px and 70px even at 11px, against 60px of room at a 360px viewport. That is why the
 * pill used to halve its own font size past eight characters, which made one chip look like a different
 * design from the three beside it.
 *
 * So the display is shortened and the size left alone. Anything not listed falls through unchanged, which
 * is what should happen when a future radar module reports a state this table has never heard of. The
 * firmware's wording is untouched, and the full phrase is still in the pill's hover title.
 */
export const PRESENCE = {
  Approaching: "Closer",
  "Moving Away": "Away",
};
