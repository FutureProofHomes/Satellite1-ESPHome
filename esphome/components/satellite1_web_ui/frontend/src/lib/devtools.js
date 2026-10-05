/**
 * Which developer tools this device's firmware carries, from GET /api/sat1/state alone.
 *
 * Everything config/satellite1.dev.yaml adds shows on Settings > Developer and nowhere else, and a
 * release build carries none of it, so the page exists exactly when one of these is true:
 *   - xmos: the firmware picker's five entity keys are all in the key table (`e`);
 *   - amp: the amplifier's analog gain key is in `e` - dev.yaml maps it together with
 *     speaker_amp_id, so no probe of /api/sat1/amp is needed;
 *   - mic, sysmon: the state flags those two modules print.
 * Read from the key table rather than the entity values, which ride /events a beat later.
 */

const XMOS_KEYS = ["xmos_fw_choice", "xmos_fw_install", "xmos_fw_refresh", "xmos_fw_list", "xmos_fw_status"];

export function devTools(device) {
  const e = device?.e || {};
  return {
    xmos: XMOS_KEYS.every((k) => !!e[k]),
    amp: !!e.amp_gain,
    mic: !!device?.mic,
    sysmon: !!device?.sysmon,
  };
}

/** Whether Settings > Developer exists for this device. False until the state has loaded. */
export function hasDevTools(device) {
  const t = devTools(device);
  return t.xmos || t.amp || t.mic || t.sysmon;
}
