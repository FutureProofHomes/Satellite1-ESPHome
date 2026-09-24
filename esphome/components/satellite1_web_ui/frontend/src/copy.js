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

  /* The switcher sheet. What needs saying is not "this is a list of devices", which is visible, but
     which device the page's controls currently belong to - the single fact that makes the whole page
     make sense, and the one someone who has just changed a setting on the wrong device wishes they
     had been told. The second sentence owns the switch's two outcomes honestly: the single-origin
     takeover for fleet-current firmware, the old navigation for anything older. Does not repeat why
     the list has one entry in it; that sentence is already at the foot of the same sheet. The third
     sentence explains the one mark on the sheet that is not self-evident: a lit radar tag. The tags
     themselves (Ethernet/WiFi, LD2450/LD2410) are their own words and need no glossary here. */
  switcher:
    "Everything on this page applies to the device named above. Pick another device and this page becomes its controls - no address to type, no leaving the page. A device on older firmware opens its own page instead. A radar tag lights up while that device detects someone in its room.",

  /* The Launch card on Diagnostics. The warning is the substance: the link is a bearer credential,
     and saying so here is what makes offering it defensible. The QR-versus-link sentence lives here
     rather than as visible copy on the card: the divergence (QR on the current IP so every phone
     can scan it, link on the permanent .local name for pasting somewhere long-lived) is invisible
     in the moment and only needs explaining to someone who goes looking. */
  launch:
    "Scan the code with a phone, or paste the link into a Home Assistant dashboard button, and that browser lands here already signed in - no password, no button press. The code carries the device's current network address, so any phone on your network can scan it; the link carries the device's permanent name, the right form to paste somewhere that keeps it. Anyone who has either can sign in with it, so treat them like the password. Sign out everywhere revokes them and every session, then issues a new one.",

  /* On the card title rather than a row, because what needs explaining is the tree, and the tree has no
     row of its own to hang an ⓘ off. This is also the only place Local Speaker is explained: it is the
     one tick in there whose effect is not obvious from its name, since it silences this device rather
     than adding a target. */
  remote_routing:
    "Plays this device's audio on other speakers as well as this one: the assistant's spoken answers, sign-in prompts, ringing timers, and the wake chime if you turn that on below. Tick a room to include every player in it, or open the room and pick players. Local Speaker is this device's own speaker - untick it and answers play only where you have chosen.",

  /* The last sentence used to be three sentences of its own paragraph under the slider. It is here
     because it explains the control rather than the page, and a permanent paragraph for something you
     need to read once is what a tooltip is for. Kept because one slider with three behaviours behind it
     is the kind of thing that gets reported as a bug. */
  remote_tts_volume:
    "How loud answers are on the remote speakers. This device's own level is Voice Volume Override, on the home page. Sonos reads the level from the announcement itself; anything else has its volume set for the answer and put back afterwards.",

  remote_wake_chime:
    "Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to. On Sonos and similar speakers the chime can land up to a second late - their clip playback has a fixed startup cost the device cannot remove.",

  remote_timer_ring:
    'Rings a finished timer on the target speakers too, until the alarm is stopped. Saying "stop" at any speaker silences it everywhere.',

  remote_sync_guard:
    "How long this device keeps its microphone closed after a routed answer, so it cannot hear its own answer from a speaker running slightly behind and mistake it for you. It only engages when the assistant is about to listen again - a continued conversation or voice sign-in; one-shot answers skip it entirely.",

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

  /* The unit toggle in the temperature editor. What needs saying is the boundary: this converts what
     the app shows, and nothing else - Home Assistant does its own unit conversion and the stored
     calibration never leaves Celsius, so flipping this can never drift it. */
  temp_unit:
    "Shows temperatures in Fahrenheit throughout this app. The device stores and reports Celsius either way, so Home Assistant and the saved calibration are unaffected.",

  humidity:
    "Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room.",

  lux: "Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger.",

  presence: "Sensed by radar, not the microphones. It works through the case and notices people even when they sit completely still.",

  /* The unit toggle under the Distance pill. Same boundary sentence as temp_unit: display versus
     stored, because a customer who tunes gates in feet deserves to know the device never left
     centimetres. */
  distance_unit:
    "Shows every distance on this page in feet. The radar keeps measuring and storing centimetres either way, so Home Assistant and the saved settings are unaffected.",

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

  // On the Wake Words card's title. The card is the whole route now: rows, graphs, the chime bell
  // in this very header - so the hint tells the one story that makes all of it legible.
  wake_words:
    'This device listens for up to two wake words at once - one row each below - plus the "Stop" word that interrupts answers and alarms. Words beyond the built-in ones download from their source when you choose them, and fetch again at every restart. A tuned word wears its graph: everything left of the handle is ignored, every dot right of it is a real firing from the last 24 hours, fading as it ages. Tap the handle to retune, tap a dot for its story, tap the chevron on a word to change or remove it.',

  // The Voice Pipeline row's tooltip: what a pipeline is and where it lives, because this is the
  // one dropdown on the card whose contents the device did not make. The docs link renders after
  // this sentence (TEXT.vp_docs).
  voice_pipeline:
    "Voice pipelines decide who answers this wake word: which speech-to-text, which conversation agent, and which voice replies. They are built in Home Assistant under Settings, then Voice assistants - this list is whatever yours offers, and Preferred follows the one marked preferred there.",

  /* wake_advanced, margin_bar, detections and scoreboard are retired with the surfaces they
     explained (the v2 Living Graph replaced the margin bar, the detections card and the
     scoreboard in one stroke - the tuner-v2 canvas is the design of record). */

  // The Living Graph: its accessible name, and the tuner card's ⓘ. One sentence per element,
  // read left to right - the two colors are the entire explanation of what a threshold is.
  living_graph:
    "A confidence scale from 0 to 100%. Everything left of the handle sits under the frost - the device ignores it. Everything right of it is live: solid dots are real firings at the confidence they scored, fading over 24 hours; hollow marks almost fired. Drag the handle to move the line; if dots you don't recognize sit just past it, drag past them.",

  // The Wake Word Sources card title. The trust caveat lives here, once, rather than on every
  // community row in the picker.
  wake_sources:
    "The wake word list is fetched live from these places - nothing is copied to our servers. Anyone can publish a wake word model, and quality varies: the included words are hand-tuned, community ones may fire too eagerly or miss you. Adding a source here only grows the list; a word starts being used when you pick it above.",

  // Names the two directions it reaches - answers this device sends elsewhere and answers other
  // devices send here - because the remote half is the invisible one, and names the timer case
  // because it works whichever way this is set and a reader who just turned this off should not
  // think they lost it.
  stop_word:
    'While an answer is playing - this device\u2019s own, or one another Satellite1 routed here - saying "stop" cuts it off everywhere it is playing. A ringing timer can always be silenced this way, whichever way this is set.',

  // wake_sensitivity is gone from the app: the shared three-step select confused exactly the
  // person it needed to help (it silently skipped downloaded words - owner, September 2026), so
  // sensitivity became each word card's own row. The "Wake word sensitivity" select entity still
  // exists for Home Assistant; the two write the same cutoffs, last writer wins, and the per-word
  // override is re-asserted at boot.

  // Names the speaker it moves, and names the other slider by its on-screen label and page, because
  // the pair live on different routes again (this one in the home page's Assistant card, the other in
  // Remote routing on Audio) and the hints are the only thing keeping them apart.
  voice_override:
    "How loud this device speaks when the assistant replies, separate from media volume. Zero follows the media volume instead. Speakers you route answers to have their own level - Remote TTS volume, on the Audio page.",

  /* The Assistant card's VAD-sensitivity select, under the slider above. Two things need saying:
     what the three options trade against each other, and that the setting is Home Assistant's -
     which is also why the row vanishes when Home Assistant is away. */
  finished_speaking:
    "How long the assistant waits after you stop talking before it answers. Aggressive answers fastest; Relaxed tolerates longer mid-sentence pauses. This is a Home Assistant setting - the same control as on its device page - so it only appears while Home Assistant is connected.",

  speaker_channel:
    "Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want.",

  led_ring:
    "The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error.",

  timers:
    'Timers set by voice, held on the device - they keep counting and still ring if Home Assistant goes away. Voice is also how they are managed: name one when you set it ("set a pizza timer for ten minutes"), pause or cancel it the same way.',

  // `media` and `media_group` hints rode the drawers' title rows until September 2026, when the
  // owner cut the titles themselves: the artwork names the media drawer and the speaker rows name
  // the players drawer, so the ⓘ had nowhere left to sit.

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
    "The ESP32 chip's own temperature, not the room's. It reads well above ambient inside the sealed case, so warm is normal; sustained readings above 80 \u00b0C / 176 \u00b0F are worth investigating. The room's temperature is on the home page.",
  reset: "Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware.",

  xmos: "The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's.",

  xmos_flash:
    "Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power.",

  // xmos_erase was here. The row it explained is gone from the app: erasing leaves the audio chip blank,
  // which takes the microphones, the speaker and the wake word with it, and the only way back is the
  // Reflash row - which needs the chip it just erased to be talking. Reflash overwrites anyway.

  /* The LD2410/LD2450 Recovery card's title. The distinction that needs drawing is against the ESP32
     card two rows down: this factory reset erases settings stored on the radar module itself - the
     tuning the Presence page edits - not the device's. */
  radar_recovery:
    "The radar module's own recovery actions. Restart just power-cycles the module. Factory reset erases the settings stored on the module itself - detection range, gate thresholds, zones, everything tuned on the Presence page - and does not touch this device's settings.",

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

  // The notification drawer's ⓘ. The badge arithmetic is the one thing the surface cannot say for
  // itself, so it leads; the 24-hour window and the swipe are the two behaviours people ask about.
  notif:
    "Toasts from the past 24 hours. The badge counts the ones that faded away before you could tap them; tapping a notification takes you to the same place the toast would have, and swiping it right (or tapping its ✕) files it under Archive. Everything clears itself after a day.",

  // The Crash Reports card. The second sentence is the workflow this card exists for; the third
  // manages the one expectation that surprises people (a full power cut takes the flight recorder's
  // memory with it, so unplugged crashes carry less detail).
  crash:
    "What the device remembers about its own crashes: when each one happened, which task died and where, the last log lines before it went down, and a downloadable crash dump for the deepest look. Attach the dump and the log to a support request, or erase them once the cause is found. A crash that cuts power entirely leaves less detail than one the device rebooted itself from.",

  // The three rows on that card.
  crash_log: "The last log lines the device wrote before it went down, recovered across the reboot. The most recent crash only - a newer crash replaces it.",
  crash_dump:
    "A complete snapshot of what every task was doing at the moment of the most recent crash, analyzed offline against this exact firmware build. It may contain fragments of anything the device held in memory, so share it like a log, not like a screenshot.",
  crash_erase:
    "Deletes the recorded crashes, the pre-crash log and the crash dump. The device keeps working either way; erase once a cause is found, so the next crash is unmistakably new.",
};

export const TEXT = {
  /* The login screen. The device sign-in is offered first and the password second, because the
     password is the thing iOS made miserable enough to build all of this around. The three mode
     lines follow what the poll reports - the device picks how it can be answered when the window
     opens, and the page's only job is to say what to do right now. */
  login_sub: "Your home. Your voice. Your AI.",
  login_tap: "Use VoiceTap Sign-In",
  // The pending instructions, one per mode. Each names the physical thing to do and nothing else;
  // the countdown beside them says how long it can wait.
  login_mode_button: "Press the action button on top of your Satellite1 - the ring is breathing while it waits.",
  // The button mode's variant when the poll says the hardware mute slider forced it: same
  // instruction, plus the one thing only the person in the room can fix.
  login_mode_button_hw:
    "Press the action button on top of your Satellite1 - the ring is breathing while it waits. (Voice sign-in is off while the hardware mute switch is on - slide it off and try again to sign in by voice.)",
  login_mode_code: "Press the action button, or say the code.",
  // Matches what the device now says aloud ("To sign in, press the action button, or say the wake
  // words on your screen") - the chips below the line are the screen it means. The device no
  // longer recites the words itself, and the challenge is six words (owner decision, September 23
  // 2026 - 729 combinations against the voice lockout, where three gave 27).
  login_mode_seq:
    "Say the six wake words below, in order - or press the action button on top of your Satellite1.",
  // The hab variant (Home Assistant connected but not allowed to perform actions) says the same
  // thing the device's own prompt says, no explanation - owner's wording, September 23 2026; the
  // post-login splash owns explaining the checkbox.
  login_mode_seq_hab: "Press the action button or say the wake words on your screen.",
  login_mode_button_hab:
    "Home Assistant isn't allowed to speak for this device yet, so sign in by pressing the action button on top of your Satellite1 - the ring is breathing while it waits.",
  // The challenge words by symbol index (0/1/2), matching the models pinned in
  // common/voice_assistant.yaml and the clips in config/sounds/. Shown as ordered chips that
  // un-bold as the device hears each one back (the poll's `p` matched-prefix count) - the security
  // tradeoff of displaying the sequence at all is documented at SessionGate::open_window_.
  login_seq_words: ["Hey Jarvis", "Okay Nabu", "Stop"],
  // Under the chips: the words are wake words, and every other Satellite1 in earshot will answer
  // them. Peer-muting cannot run pre-login (the roster and its passwords sit behind the session),
  // so the page asks the person instead.
  login_seq_peers_hint:
    "If other Satellite1 devices are within earshot, mute them first so they don't answer the challenge.",
  login_cancel: "Cancel",
  // The confused-deputy warning, and the one string here that is doing security work: the action
  // button approves whoever asked first, so a person "helping" a stuck screen by pressing it could
  // be letting someone else in.
  login_busy:
    "Another sign-in is already waiting on this device. If that isn't you, don't press the button - wait for the light to stop.",
  // Says how to recover, not just what happened: the action button only approves while the
  // countdown runs, and a press after expiry falls through to the assistant - "nothing happened"
  // in a real customer's words (support case, September 2026).
  login_expired: "Nothing answered in time, so the sign-in closed. Start again, then press the button or answer while the countdown runs.",
  login_denied: "The spoken answer didn't match, so the sign-in closed.",
  login_start_failed: "Couldn't reach the device to start. Check the connection and try again.",
  login_retry: "Try again",
  login_or: "or use the password",
  login_pw_placeholder: "Password",
  login_pw_submit: "Sign in",
  login_show_pw: "Show password",
  login_hide_pw: "Hide password",
  login_wrong: "That's not the password.",
  login_locked: "Too many tries. Wait %s seconds and try again.",
  login_unreachable: "Couldn't reach the device. Check the connection and try again.",
  // Where the password lives, for the person standing in front of this form for the first time.
  login_pw_hint: 'See "Web UI Password" on this device\'s page in Home Assistant.',
  // The side nav's sign-out: this browser only, unlike Diagnostics' sign-out-everywhere.
  logout: "Sign out",

  /* The Launch section on Diagnostics. */
  launch_copy: "Copy link",
  launch_copied: "Copied",
  // Shown when this browser's .local probe failed (lib/auth.js remembers it for a day): the
  // copyable link switches to the device's IP - the .local form is a dead end here - and this line
  // says what that trade costs and how to make it safe.
  launch_mdns_hint:
    "Your network doesn't resolve .local names, so this link uses the device's IP address. It stops working if your router hands the device a new address - give it a DHCP reservation to make the link permanent.",
  launch_regen: "Sign out everywhere",
  launch_regen_title: "Sign out everywhere?",
  launch_regen_body:
    "Every signed-in browser and every sign-in link or QR code stops working immediately. This browser stays signed in, and the new link appears here.",
  launch_regen_confirm: "Sign out everywhere",

  /* The change-password form at the foot of the Launch card. The bounds sentence states the
     firmware's exact rule so a refusal is never a mystery; the "wrong" line is deliberately the
     login form's tone. pw_changed_sub restates the logout_all consequence at the moment it
     happened, because the person's other devices are about to show login screens. */
  pw_title: "Change password",
  pw_current: "Current password",
  pw_new: "New password",
  pw_again: "Confirm new password",
  pw_busy: "Changing\u2026",
  pw_len: "The new password needs 8 to 31 characters.",
  pw_chars: "Letters, numbers, and punctuation only - no quotes, backslashes, or spaces at the ends.",
  pw_mismatch: "The two copies of the new password don't match.",
  pw_wrong: "That's not the current password.",
  pw_fixed_note: "This device's password is set in its build configuration and can't be changed here.",
  pw_changed: "Password changed",
  pw_changed_sub: "Every other signed-in browser and sign-in link has been signed out.",

  ha_connected: "Connected to Home Assistant",
  ha_disconnected: "Not connected to Home Assistant",
  ha_disconnected_detail:
    "Everything on this page still works - it talks to the device directly. Media and anything that needs your smart home will be unavailable until the connection returns.",

  // On the shared toast surface since the amber banners were retired (September 2026), and the one
  // sticky toast: it describes an ongoing state, so it stays until the stream reconnects rather than
  // timing out. Tapping it goes to the device log, same as the write-failed toast.
  stream_lost: "Lost the connection to the device. Retrying.",

  // The sticky's green ending, on the falling edge only - a session that never lost the stream
  // never sees it. No second line: there is nothing to do about good news.
  stream_back: "Connection to the device restored.",

  // Log-driven toasts (notch toast redesign, September 2026): a warning or error line arriving on
  // the stream, named by the component tag out of its "[W][wifi:123]" header so the coalesce badge
  // counts one component's burst rather than mixing strangers. The fallback subject covers a line
  // whose header did not parse. Second line is write_failed_go - the same tap, the same log.
  log_toast_err: "Error from %s",
  log_toast_warn: "Warning from %s",
  log_toast_dev: "the device",

  // A crash count that moved mid-session: the device rebooted from a crash while the page watched.
  // Only mid-session moves toast - history is the Crash Reports card's story, told there.
  crash_toast_t: "The device crashed and restarted.",
  crash_toast_s: "Tap for the crash report.",

  // Update news, once per session, the version in the title because it is the fact worth reading.
  update_toast_t: "Update %s is available.",
  update_toast_s: "Tap to install from Diagnostics.",

  // The ✕ on every toast: dismiss without acting, against the card's tap which acts.
  dismiss: "Dismiss",

  /* The notification center (September 2026): the bell, its drawer, and the filter pills. The
     bell's aria-label carries the live count in parentheses when there is one. "Archive" is both
     the pill and the swipe well's word - one name for one place. */
  notif_bell: "Notifications",
  notif_title: "Notifications",
  notif_all: "All",
  notif_info: "Info",
  notif_warn: "Warnings",
  notif_err: "Errors",
  notif_arch: "Archive",
  // Two empty states: the filters share one (a quiet day is the good outcome and reads like it),
  // the archive names itself so an empty archive does not read as a bug.
  notif_empty: "Nothing here from the past 24 hours.",
  notif_empty_arch: "Nothing archived yet.",
  // One press, every unhandled row to Archive, the badge to zero. "Clear", not "delete": the rows
  // survive under the Archive pill for their 24 hours.
  notif_clear: "Clear All",
  notif_foot: "Notifications clear after 24 hours.",
  // Relative times on the rows: minutes under an hour, hours to the 24-hour edge.
  notif_now: "just now",
  notif_ago: "%s ago",

  // Only when the sheet has no peer rows at all. It used to also name the add-by-address field that
  // ended the sheet; the field is gone (owner's call), so Home Assistant's roster is the one way a
  // row appears and the sentence says only that.
  no_devices: "Only this device. Other Satellite1s appear here once Home Assistant lists them.",

  // The same foot-line while the actions checkbox is off, which is a fixable reason rather than an
  // empty house - so it points at the fix instead of promising rows that cannot arrive.
  no_devices_blocked: "Only this device. Other Satellite1s cannot be listed while Home Assistant actions are off.",

  // Titles on the peer dots. Attributed to Home Assistant rather than stated as fact, because that is
  // the only witness: the page cannot probe a peer itself, and its view can lag a reboot by seconds.
  // peer_manual, peer_add_ph and peer_add went with the manual add-by-address feature.
  peer_up: "Available, according to Home Assistant",
  peer_down: "Unavailable, according to Home Assistant",

  // Titles on the radar tag, which doubles as the presence light. Unattributed, unlike the dots:
  // on the serving device's row this is the radar's own live state over the page's stream, and on a
  // peer's it is at most one sync beat behind - hedging both to cover the beat would cost the
  // sentence more than the lag costs the truth.
  presence_on: "Presence detected",
  presence_off: "No presence detected",

  // The one word in the top bar while a peer is being controlled (single-origin device switching).
  // "Remote" and not the peer's address: the name beside it already identifies the device, and what
  // the badge adds is only the mode - this page is serving as another device's controls.
  remote_tag: "Remote",

  // On the roster row that is the device serving this page, visible only while another device is
  // being controlled. It marks the way back - a plain return, not another cross-sign-in.
  switcher_home: "This page",

  // The failed-write toast, shown on any route. First line is the one fact every failure shares;
  // the control involved has already put its old value back, which is why "hasn't been applied"
  // needs saying - the reverted switch alone reads as a page that ignored the tap. Second line is
  // where the specific reason lives.
  write_failed: "A change didn't reach the device, so it hasn't been applied.",
  write_failed_go: "Tap for the device log.",

  // Shown in place of the transcript, not instead of the card. The card carries the assistant's phase in
  // its header, so it has something to say from the moment the device answers; this only fills the space
  // where the lines will go. Reworded to the owner's text, September 2026.
  nothing_said: "Voice transcriptions are displayed here.",

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

  // no_wake_words was retired to a comment in September 2026 and deleted in the v2 sweep: the
  // empty state's ww_route_none says everything it said.

  // The label for Home Assistant's `preferred` pipeline value, which needs a label rather than a
  // name so a customer who calls one of their own pipelines "Preferred" still gets two
  // distinguishable entries. (The old "Off" entry is gone: silencing a slot is the picker's None.)
  pipeline_preferred: "Preferred",

  /* The Wake Words card (the v2 Living Graph redesign, September 2026 - the tuner-v2 canvas is
     the copy's design of record) and the massive word list, which expands inline under a word's
     pill now. ww_pick_title, ww_card1/2 and ww_sources_btn retired with the modal sheet that
     carried them. */
  ww_card: "Wake Words",
  // ww_settings ("Wake Word Settings") is retired with its card: the chime - the card's only
  // control - moved into the Wake Words card's own header as the bell toggle (v2, September 2026).
  ww_search: "Search wake words",
  // The empty route: one explanation, one action. "On the device, no cloud" is the fact worth
  // leading with for a first-time owner deciding whether to trust a microphone.
  ww_route_none: "This device isn't listening for anything yet. Pick a phrase and it starts listening for it - on the device, no cloud.",
  ww_choose: "Choose a wake word",
  ww_add: "+ Add word",
  // On the picker row the other slot already holds - the reason it is disabled, on the row itself.
  ww_on_other: "on the other slot",

  /* The Living Graph's row vocabulary (v2: no badge, no menu - the graph's presence IS the tuned
     state, and the two colors are the explanation). */
  // The untuned row's one loud button - short on purpose (owner: "less text").
  ww_tune_btn: "Tune it!",
  // The chime bell toggle's accessible name (the control itself is the bell).
  ww_chime: "Wake chime",
  // The radar's live-dot grammar on a freshly tuned row: the graph is streaming.
  lg_live: "Live",
  // A tapped hollow mark's suffix in its popover: it almost fired, and didn't.
  lg_ignored_short: "almost",
  // The axis label every 0-100% graph carries in its lower-left (owner call - it replaced the
  // frost's "ignored" caption; the veil says that on its own).
  tn_axis: "Confidence",
  // The dimmed old row mid-swap: truthfully still listening until the replacement is ready.
  mb_still: "still listening",
  mb_off: "off",
  mb_swap_note: "\u201c%1\u201d is still listening until \u201c%2\u201d is ready.",

  /* The stop word's row: its pill IS the switch (green listening, red off), and the graph is gone
     entirely while off - a disabled model is unloaded, so there is nothing to draw. Capitalized
     "Stop" everywhere it displays, owner call. */
  stop_off_note: "Not listening. Tap \u201cStop\u201d to turn it back on.",
  // "Built-In", not "Included": the owner's word for the two words compiled into the firmware,
  // and it reads as the guarantee it is - these two exist without any download.
  ww_included: "Built-In wake words",
  // ww_experimental is gone with the experiments folder itself: excluded at enumeration (owner,
  // September 2026) - their own README says "minimally trained and tested, not supported in any
  // way", and a word that never fires reads as our bug.
  //
  // What the collapsed picker says when the slot holds nothing. The explicit "Disabled" list entry
  // is gone (owner, September 2026 - it read as one of the words): a slot empties by unchecking
  // the word it holds, and this line is the state that leaves behind.
  ww_none: "No wake word selected.",
  ww_all_langs: "All languages",
  // The n-more line when a search inside ~800 words still matches a crowd.
  ww_more: "more match - keep typing",
  ww_source_loading: "Reading the list\u2026",
  ww_source_failed: "Couldn't read this source. It may be rate-limited - try again in a minute.",
  // On an entry whose manifest the browser could not read (big repo without a catalog, or a host
  // that refuses the browser). The device still verifies everything when the word is picked.
  ww_unverified: "unchecked until picked",

  /* The voice pipeline row (renamed from Assistant, owner's call - it is Home Assistant's own
     word for the thing being picked). The hint explains where pipelines come from; the link opens
     the FutureProofHomes walkthrough for building one. */
  vp_label: "Voice Pipeline",
  vp_docs: "Learn how to build one.",
  vp_docs_url: "https://docs.futureproofhomes.net/satellite1-set-up-voice-control/",

  /* The swap in flight, and its two endings. */
  ww_downloading: "Downloading\u2026",
  ww_loading: "Loading\u2026",
  // A slot that persisted a URL and is between the boot and the fetch. The device retries by
  // itself, so neither line offers a button.
  ww_waiting: "Waiting for the network to fetch this word\u2026",
  ww_retrying: "The device retries on its own.",
  ww_retry: "Retry",
  ww_dismiss: "Dismiss",
  // Prefixes the specific reason from WW_ERR below. The previous word is still listening, which is
  // worth a clause because a failed swap otherwise reads as a device left deaf.
  ww_failed: "Couldn't load this wake word - the previous one is still active.",

  // The apply-to-all strings are gone with the feature (owner, September 2026): a general
  // apply-to-peers mechanism is planned for many controls at once, and this one-off predated it.

  /* The Wake Word Sources card. */
  ws_title: "Wake Word Sources",
  ws_add: "Add source",
  ws_ph: "Paste a GitHub repo or model .json URL",
  ws_bad_url: "That doesn't look like a GitHub repository or a model .json link.",
  ws_restore: "Restore default sources",
  // The footer sentence, split where its two links render: "Don't see your wake word? Request one,
  // or train your own microWakeWord."
  ws_footer_q: "Don't see your wake word? ",
  ws_request: "Request one",
  ws_or: ", or ",
  ws_train: "train your own microWakeWord",
  ws_words: "words",
  // The remove confirmation's one important fact.
  ws_remove_t: "Remove this source?",
  ws_remove_b: "The list above loses its words. A word you already picked keeps working - the device remembers its link, not the source.",
  ws_remove_c: "Remove",

  /* The v2 Wake Word Tuner: two user-paced phases on the Living Graph (the tuner-v2 canvas is the
     design of record). No room-listening phase - the room's last 24 hours is already on the graph
     as the amber smatter; no confirmation phase - the first real firing lands with its ripple and
     that is the confirmation. It speaks in percentages because the scores are percentages. */
  // The tuner card's title, quick edit included: instructions live in the grey line under the
  // graph, never in a title (owner call - the "move the line" variant read as a second heading).
  tn_title: 'Tune \u201c%s\u201d',
  // The ready gate's one grey paragraph (owner's wording, shortened): what the dots are, what
  // they're for, and that nothing here is final.
  tn2_ready:
    'Follow the prompts and walk the room saying \u201c%s\u201d. Blue dots are live detections; amber dots are the past 24 hours. You\u2019ll use them to place the firing line - and you can move it any time.',
  // The stop word's one twist: it fires over playback, so measure it the way it will be used.
  tn2_ready_stop:
    'Start some music first - \u201cStop\u201d matters most while something plays. Then follow the prompts: blue dots are live detections, amber dots the past 24 hours.',
  tn2_start: "Start",
  // The positional rounds: three close-range tries by one voice overestimate the household's
  // floor, so the far and other-voice rounds are the cheap insurance.
  tn2_near: 'Round 1 of 3 - say \u201c%s\u201d from where you usually are. Twice is plenty.',
  tn2_far: "Round 2 of 3 - now once from across the room.",
  tn2_other: "Round 3 of 3 - hand it to anyone else who uses this device.",
  tn2_skip: "Skip this round",
  // The attempt dots' round labels on the graph.
  tn_r_near: "near",
  tn_r_far: "far",
  tn_r_other: "them",
  tn_legend_you: "you",
  // The amber smatter's legend label (owner wording, September 2026).
  tn_legend_hist: "wake history",
  // The placement readout, narrating the knob live. Safe names the margin; the edges push back.
  tn2_ok: "Fires above %s \u00b7 every room dot in the frost",
  tn2_under: " \u00b7 %s pts under your quietest try",
  tn2_high: "Above your quietest try (%s) - real calls from across the room will be missed.",
  tn2_low: "Amber dots sit past your line - the room has scored this high in the last day. Expect false firings.",
  tn_apply: "Apply",
  // The placement row's other verbs: back into the voice rounds, and the red one that erases the
  // track's remembered 24 hours (dots and smatter both - it is a measurement, hence the red).
  tn2_retune: "Re-Tune",
  tn2_clear: "Clear history",
  // The no-gap failure, diagnosed by side: a loud room and a weak word are different problems.
  tn_nogap_room: "The room is too loud for this word right now - its noise reaches the word's scores. Try a quieter moment, or move the device.",
  tn_nogap_voice: "The word isn't scoring strongly enough - move closer, speak up. Or honestly: this model may be weak; try a different word.",
  // A wake model fired but the voice-activity model refused it. The person can fix this one.
  tn_vad: "That didn't register as speech - move closer or speak up, and try again.",
  // The capability fallback: the score channel needs debug logging compiled in (it is, on stock
  // firmware; a custom build may have turned it off).
  tn_nocap: "This firmware build can't score attempts, so tuning isn't available. Stock firmware can - this build was compiled without debug logging.",
  tn_gone: "The tuning session ended. Open it again to start over.",

  /* Tune-time peer muting (September 2026): same-area Satellite1s are held muted for the session
     so they don't answer the word being said over and over, each restoring its own prior mute
     state when the session ends. Explicit by owner decision - the line names who is muted and who
     could not be, rather than muting what it can in silence. The names in parentheses are the
     roster's own device names. */
  tn_pm_one: "Muted 1 nearby satellite for this session",
  tn_pm_many: "Muted %s nearby satellites for this session",
  // Old firmware without the endpoint, a stale password, a dead peer - the tuner cannot tell
  // which, and the person only needs to know that device may answer mid-tune.
  tn_pm_failed: "Couldn't mute %s - it may answer while you tune.",
  // The roster itself was absent: nothing could even be looked for. Worded as a limit, not an
  // error - tuning works fine, the neighbours just aren't covered.
  tn_pm_unknown: "Can't check for nearby satellites - Home Assistant is unreachable.",

  /* The dots' popover vocabulary (the retired detections card's exact-time story, told in place). */
  det_just_now: "just now",

  /* Diagnostics: the Crash Reports card. The empty state is a good day and reads like one. The
     no-partition note names the one fix (a USB flash) rather than describing the partition table,
     which nobody flashing a device needs to picture. */
  crash_title: "Crash Reports",
  crash_none: "No crashes recorded.",
  crash_no_part:
    "This device can list crashes but can't keep full crash dumps: that needs a one-time flash over USB, which also resets stored settings. Everything else here works without it.",
  crash_ran: "ran %s",
  crash_restarts_ago: "%1 after power-on \u00b7 %2 restarts ago",
  crash_restart_ago: "%1 after power-on \u00b7 1 restart ago",
  crash_bt: "Backtrace",
  crash_bt_corrupt: "Backtrace (corrupted)",
  crash_log_row: "Pre-crash log",
  crash_log_show: "Show",
  crash_log_hide: "Hide",
  crash_log_none: "No log survived from before the crash.",
  crash_dump_row: "Crash dump",
  crash_download: "Download",
  crash_erase_row: "Crash history",
  crash_erase: "Erase history",

  // Shown under the wake words when the assistant selects could not be read and no more specific
  // reason applies (the two below cover blocked actions and an old Home Assistant). Says which half
  // of the card is affected, because whether the device listens is its own to answer and that half
  // keeps working - as a switch, since with no assistants to list a dropdown would have nothing in it.
  assistant_needs_ha:
    "Home Assistant keeps which assistant answers each wake word, so that cannot be set from here until it is reachable. Turning a wake word on and off is the device's own setting and still works.",

  // The same line's precise variant when the actions checkbox is the reason, with the Show fix link
  // rendered after it. Warns about the drift the blocked state actually causes here: the slot sync
  // that keeps Home Assistant agreeing with these toggles is itself an action call.
  assistant_blocked:
    "Home Assistant is not letting this device perform actions, so the assistants cannot be listed. Wake words switched here still change the device, but Home Assistant may switch them back when it reconnects.",

  // assistant_slots_full is gone: the two-slot pickers make three active wake words unrepresentable,
  // which is better than explaining what happened when there were.

  confirm: "Confirm",
  confirm_title: "Are you sure?",
  cancel: "Cancel",

  /* The Home Assistant data layer. Each of these is a different reason the area and player lists are
     not here, and each has a different fix - which is the whole point of not collapsing them into one
     "unavailable" message. */
  ha_pending: "Asking Home Assistant which speakers you have.",

  ha_never:
    "Not connected to Home Assistant, so the device does not know which areas or speakers exist. The controls below still hold their current settings and will apply as soon as the connection returns.",

  /* ha_refused is gone. It hedged "either it is older than 2025.12, or this device is not allowed to
     perform actions" because the frontend could not tell the two apart; the `actions` field on
     /api/sat1/ha now says which, so each gets its own precise line. The blocked one leans on the fix
     drawer (the Show fix link rendered beside it) rather than walking the whole path itself. */
  ha_blocked:
    "Home Assistant is not letting this device perform actions, so your areas and speakers cannot be listed - and answers play only on this speaker until it can.",

  ha_too_old:
    "This Home Assistant is older than 2025.12, which cannot answer the calls these lists are built from. Update Home Assistant to choose speakers here.",

  /* The link that opens the fix drawer, wherever the blocked state is mentioned in place. */
  show_fix: "Show fix",

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
  /* This device's own media player, shown greyed rather than omitted. Routing to yourself is the
     Local Speaker row's job, and ducking your own volume while you talk is never right. */
  cap_self: "This device",

  /* A player Home Assistant cannot currently reach. Greyed like the capability rows above, but the
     checkbox keeps working and a prior selection stays ticked: being offline is transient, and the
     call-time walks already skip the player, so unticking it would throw away a setting its owner
     still wants the day the speaker comes back. */
  player_offline: "Offline",

  ha_truncated:
    "Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below.",

  /* sel_failed is gone with the amber banners (owner decision, September 2026): a refused selection
     write now flows through the write-failed toast like every other write - one failure, one surface. */

  /* The splash: the verdict overlay that holds the app's first paint while the boot calls land
     (splash.jsx). One status line per waiting phase, then either a graceful fade or one of the
     failure cards below. */
  splash_connecting: "Connecting to your Satellite1\u2026",
  splash_asking: "Asking Home Assistant which speakers you have\u2026",
  splash_connected: "Connected to Home Assistant.",
  splash_slow:
    "This is taking longer than it should. The device may still be starting up, or Home Assistant may be slow to answer.",
  splash_error: "The device answered with an error, so the app cannot start.",
  splash_retry: "Try again",
  // The escape into the degraded app, on every failure card. Two labels: the cards where Home
  // Assistant itself is fine and only the actions channel is shut (blocked, too-old) say so, because
  // "without Home Assistant" there would claim more is broken than is; the cards with no Home
  // Assistant at all keep the plain form. One shared sub-line, cut to five words from a fourteen-word
  // list of survivors (owner, September 2026: "a lot of words down there").
  splash_continue: "Continue without Home Assistant",
  splash_continue_actions: "Continue without Home Assistant actions",
  splash_continue_sub: "Your experience will be limited.",

  /* The blocked card: the actions checkbox walk-through, shared by the splash and the fix drawer.
     Every line here was read on a phone and cut down with the owner (September 2026) - resist
     re-expanding them. The body leads with what ticking the box buys, not what is broken: the title
     already names the problem. The checkbox label in step 3 is quoted verbatim so it matches what
     Home Assistant renders. In step 2, %c is where BlockedGuide draws the cog icon - the same
     mdi:cog glyph Home Assistant puts on the device's row, matched by sight rather than by name -
     and %s is the device's name. Two step-2 forms because of a catch-22: Home Assistant's display
     name for the device travels only over the action channel this card exists to unblock, so on
     first onboarding the firmware name is all there is and the hedge is honest, while a device that
     ever synced (the box was ticked once, then unticked) still holds the real name and earns the
     short form. deviceIdentity's `named` says which. */
  blocked_title: "Allow Home Assistant actions",
  blocked_body: "One checkbox in Home Assistant lets this device list your areas and media players.",
  blocked_step1: "In Home Assistant, open Settings \u203A Devices & services \u203A ESPHome.",
  blocked_step2: "Tap the %c cog next to %s.",
  blocked_step2_unnamed: "Tap the %c cog next to this device - %s, unless you renamed it.",
  blocked_step3: "Tick \u201CAllow the device to perform Home Assistant actions\u201D, then Submit.",
  // Two URLs behind one button - see openHomeAssistant in splash.jsx for the handoff. The app URL
  // is the companion app's own scheme, tried first on phones because it lands inside the app
  // directly (the My Home Assistant redirect always stops at an interstitial tab first - owner hit
  // it on iOS, September 2026) and needs no internet. The https redirect stays as the anchor's real
  // href and the fallback: desktops, and phones without the app. Neither can land on the Configure
  // dialog itself - Home Assistant has no URL for an options flow, and the device-page link would
  // need a registry id that rides the exact payload a blocked device cannot fetch - so steps 2 and
  // 3 cover the last two taps. The web redirect needs the browser to have internet; without it the
  // written steps stand alone. The device itself serves nothing external either way.
  blocked_open_ha: "Open Home Assistant",
  blocked_open_ha_url: "https://my.home-assistant.io/redirect/integration/?domain=esphome",
  blocked_open_ha_app_url: "homeassistant://navigate/config/integrations/integration/esphome",
  // One row beside a small spinner, and the manual escape rides inside the sentence as a link
  // instead of standing as its own button (owner, September 2026). "Advances automatically" leads
  // because it is the fact people missed when the sentence led with "watching"; the trigger goes
  // unsaid because step 3 directly above is the trigger. blocked_recheck is the link's text; the
  // rendering adds the period after it so the sentence closes whatever the link's tap state.
  blocked_watching: "Advances automatically - or",
  blocked_recheck: "manually check again",
  // The slow card's button, not the blocked card's (that one's manual check is the link above).
  blocked_check: "Check again",

  /* The other two failure cards the splash can land on. */
  noha_title: "Not connected to Home Assistant",
  noha_body:
    "The device has no connection to Home Assistant right now, so it cannot list your areas, media players, or other Satellite1 devices. Its own controls all work. This screen continues on its own when the connection returns.",
  old_ha_title: "Home Assistant needs an update",
  old_ha_body:
    "This Home Assistant is older than 2025.12, which cannot answer the calls the app's speaker and assistant lists are built from. Update Home Assistant to use them - everything on the device itself works now.",

  /* The blocked nudge, on the shared toast surface: shown once when the app is entered while actions
     are off, so the fix stays one tap away after the splash's Continue. */
  blocked_toast_t: "Home Assistant actions are off.",
  blocked_toast_s: "Speaker lists, assistants and other devices are unavailable. Tap for the fix.",

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
  pw_change: {
    t: "Change the password?",
    b: "Every other signed-in browser, pasted sign-in link and QR code stops working the moment it changes. This browser stays signed in, and Home Assistant's Web UI Password sensor shows the new value.",
  },
  radar_restart: {
    t: "Restart the radar module?",
    b: "Presence detection drops out for a few seconds while the module comes back. Nothing is erased and no settings change.",
  },
  radar_factory: {
    t: "Reset the radar to factory defaults?",
    b: "Erases the settings stored on the radar module itself - detection range, gate thresholds and zones, everything tuned on the Presence page - and restarts it. This device's own settings are not touched, but the radar tuning cannot be brought back.",
  },
  safe_mode: {
    t: "Restart into safe mode?",
    b: "The device comes back with only its network connection and update tools running - no assistant, no audio - so a bad update can be recovered. Restarting again returns it to normal.",
  },
  factory_reset: {
    t: "Erase everything?",
    b: "Every setting stored on this device is wiped, including its Wi-Fi credentials, and it restarts as if new from the box. You will have to set it up again.",
  },
  crash_erase: {
    t: "Erase the crash history?",
    b: "The recorded crashes, the pre-crash log and the crash dump are deleted from this device. If you are still chasing the cause, download the dump first - there is no way back to it.",
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

/**
 * Why a wake word swap failed, keyed by the loader's SlotError numbers (mww_runtime_loader.h - the
 * numbers are a contract, appended to and never renumbered). Each line is what the person can act
 * on, not what the firmware saw; the "previous word still active" reassurance is ww_failed's job,
 * said once above these.
 */
export const WW_ERR = {
  // 0 is ERR_NONE on the wire, so it only reaches this table when the write itself was refused or
  // never answered - a link too long for the device's 224-byte slot, or a request lost to the
  // network. Without this line those failures showed ww_failed with a blank where the reason goes.
  0: "The device didn't accept the request. The link may be too long.",
  1: "The link couldn't be reached.",
  2: "This link is not a microWakeWord model. This device only runs microWakeWord models.",
  3: "This model's format is a version this firmware doesn't know.",
  4: "This model was built for a different audio setup and can't run on this device.",
  5: "This model needs a newer firmware than this device is running. Update the device and try again.",
  6: "This model is too large for this device.",
  7: "The download didn't finish. Check the connection and try again.",
  8: "The downloaded file isn't a usable wake word model.",
  9: "The device refused the model.",
  10: "Not enough free memory for this model right now. Restart the device and try again.",
};
