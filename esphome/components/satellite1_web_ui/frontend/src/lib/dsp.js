/**
 * Whether the Audio tab has its DSP page: exactly when the running XMOS firmware has the speaker EQ.
 *
 * The ESP32 asks the XMOS while it connects, before it publishes the XMOS version (satellite1_dsp's
 * connected hook), and serves the answer as GET /api/sat1/state's "dsp" field. So the first paint
 * needs no extra request, and a single GET /api/sat1/dsp/caps each time the xmos_firmware entity
 * becomes a version string again - after boot, a reflash or a picker install - always gets the final
 * answer. Nothing polls for it.
 *
 * While the entity reads anything else ("Flashing... (40%)", "XMOS not responding"), the previous
 * answer stands, so a reflash does not make the page blink out and back.
 */

/** 1 supported, 0 not, 2 not known yet (no XMOS has connected since boot), null while the state
 *  has not loaded. Firmware without the speaker EQ component sends no field at all: that is a 0. */
export function dspSupport(device) {
  if (!device) return null;
  return device.dsp === 1 || device.dsp === 2 ? device.dsp : 0;
}

export function dspVisible(support) {
  return support === 1;
}

/** The sub-pages the nav lists for a tab. Settings' are whatever the caller passes, so the DSP
 *  answer never touches them; Audio lists its pages only while DSP is visible, because Routing
 *  alone is nothing to choose between. */
export function navSubs(tab, support, settingsRoutes, audioRoutes) {
  if (tab === "SETTINGS") return settingsRoutes;
  if (tab === "AUDIO" && dspVisible(support)) return audioRoutes;
  return [];
}

/** #/audio/dsp is sent on to Routing only on a definite no. While the answer is 2 the page waits
 *  for the connect, and while it is null for the state. */
export function dspRedirect(support) {
  return support === 0;
}

/** What the ESP32 publishes for a connected XMOS: "v1.1.0", "v1.1.0-dev.8". */
export function isXmosVersion(value) {
  return /^v\d+\.\d+\.\d+/.test(String(value ?? ""));
}

/**
 * Whether the xmos_firmware entity's move from `prev` to `next` calls for the caps re-check.
 * `prev` is undefined for the first value this page sees, and then the check runs only when the
 * state payload could not already answer. A repeat of the same version is not a connect; a progress
 * update is not a version.
 */
export function dspRecheckDue(prev, next, support) {
  if (!isXmosVersion(next)) return false;
  if (prev === undefined) return support !== 0 && support !== 1;
  return !isXmosVersion(prev) || prev !== next;
}
