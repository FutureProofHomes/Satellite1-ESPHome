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

  remote_tts_volume:
    "How loud the answer is on the remote speakers. It does not touch this device's own level - that is Voice Override, on the Audio card.",

  remote_wake_chime:
    "Plays the wake chime on the target speakers too, so you can hear that the device heard you from the room the sound is going to.",

  // duck_area, duck_players and duck_tts_targets are gone for the same reason. Ducking is now the same
  // tree, and the answer to "which players" is visibly the thing being ticked.

  duck_volume:
    "The level everything you have chosen drops to while the assistant is listening and answering. Players already quieter than this are left alone, so a whole-house group does not get turned up.",

  temp: "Reads high by design - the sensor sits inside a warm enclosure, next to the board. Calibrate against a thermometer in the same room and the offset is stored on the device.",

  humidity:
    "Measured at the board, so it drifts with the enclosure temperature. Calibrate against a hygrometer in the same room.",

  lux: "Ambient light at the front face. Useful for dimming the LED ring automatically from Home Assistant, or for a light-level trigger.",

  presence:
    "From the mmWave radar, not the microphones. It sees through the enclosure and does not need line of sight, so it reports an empty room faster than a motion sensor would.",

  calibrate:
    "Enter what a trusted instrument in the same room reads. The difference is saved as an offset and survives a restart. It does not change the raw reading, only what the device reports.",

  mute: "Cuts the microphones in hardware, on the XMOS chip, not in software. Wake word detection stops with them. The mute button on the device does the same thing.",

  wake_sound: "Plays a short chime on the speaker the moment the wake word is detected.",

  wake_sensitivity:
    "How readily the wake word fires. Raise it if the device misses you from across the room; lower it if the television sets it off.",

  voice_override:
    "Speaker volume used for assistant replies only, independent of media volume. Set to zero to follow the media volume instead.",

  speaker_channel:
    "Which side of a stereo source reaches the single speaker. Mono sums both, which is usually what you want.",

  led_ring:
    "The ring the assistant animates. Colour and brightness set here are the resting state - the device still overrides both while it is listening, thinking or reporting an error.",

  timers:
    "Timers set by voice, held on the device. They keep counting and still ring if Home Assistant goes away.",

  heap: "Internal RAM still available. This is the number that matters: it is what audio buffers and the network stack allocate from, and it is far scarcer than PSRAM.",

  psram:
    "External RAM, used for the large buffers. Installed is the size of the chip; the free and total figures cover the region handed to the allocator, which is smaller because the cache and early allocations sit outside it.",

  loop: "Longest single pass through the main loop since this panel last read the value. Tens of milliseconds is normal. Sustained hundreds means something is blocking, and audio will stutter before anything else does.",

  reset: "Why the device last restarted. 'USB peripheral' means it was flashed. 'Power glitch' or 'Brownout' points at the power supply rather than at the firmware.",

  xmos: "The audio chip. It owns the microphones, the speaker, the mute button and the LED ring, and runs its own firmware separate from the ESP32's.",

  xmos_flash:
    "Rewrites the audio chip's firmware from the image embedded in this build. The device is deaf and mute until it finishes, which takes about a minute. Do not cut power.",

  xmos_erase:
    "Erases the audio chip's firmware. It will not process audio again until you flash it. Only useful when a flash has gone wrong and the chip will not accept a new image.",

  safe_mode:
    "Restarts with everything but wifi and the updater switched off. Use it when the device is crash-looping and will not stay up long enough to accept an update.",

  factory_reset:
    "Erases every setting stored on the ESP32, including the wifi credentials, and restarts. You will have to set the device up again from scratch. The audio chip's firmware is not touched.",

  beta: "Offers pre-release firmware to the updater. Useful for testing a fix; not what you want on a device you rely on.",

  log: "The device's own log, live. This is the first place to look when something misbehaves, and the most useful thing to attach to a support request.",
};

export const TEXT = {
  ha_connected: "Connected to Home Assistant",
  ha_disconnected: "Not connected to Home Assistant",
  ha_disconnected_detail:
    "Everything on this page still works - it talks to the device directly. Media and anything that needs your smart home will be unavailable until the connection returns.",

  stream_lost: "Lost the connection to the device. Retrying.",

  no_devices:
    "Only this device. Other Satellite1s on your network will appear here once they are running firmware with this web app.",

  confirm: "Confirm",
  cancel: "Cancel",

  copied: "Copied",

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

  ha_truncated:
    "Too many areas to send in one go, so the list is cut short. Players already chosen are still used, whether or not they appear below.",

  /* The selection endpoint refused a write. Shown as a banner rather than a silent revert, because the
     checkbox has already moved back and that on its own looks like a page that ignores clicks. */
  sel_failed: "That change was not saved. The device rejected it, or the connection dropped.",

  ha_refresh: "Refresh",
  ha_refreshing: "Asking\u2026",
};
