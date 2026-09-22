# XMOS Flash Recovery Hardware Tests

This suite validates XMOS flashing, power-loss recovery, flash identity checks,
and ESP factory-reset sequencing on physical Satellite1 hardware.

## Safety

- Use a development unit whose ESP preferences and XMOS flash may be erased.
- A power-loss step must interrupt every ESP power source, including USB power.
- Do not remove or install a HAT until USB and main power are both disconnected.
- Capture boot logs with an isolated UART connection that does not power the
  target. If that is unavailable, reconnect network or USB logging after power
  is restored and note any early boot lines that could not be captured.
- The production factory-reset test erases ESP preferences and requires the
  device to be provisioned again.
- Do not run these tests on a device that contains the only copy of required
  credentials or calibration data.

## Test Firmware

The test configuration adds the diagnostic XMOS reset, embedded-flash, and
full-erase controls to the normal firmware configuration.

```sh
source scripts/setup_build_env.sh
esphome run config/satellite1.xmos_recovery_test.yaml --device <serial-port>
```

Capture serial logs for every case:

```sh
esphome logs config/satellite1.xmos_recovery_test.yaml --device <serial-port>
```

Record the following before testing:

| Item | Value |
|---|---|
| ESP firmware commit | |
| XMOS firmware version | |
| Primary HAT flash UID | |
| Secondary HAT flash UID, if available | |
| Power source | |
| Serial port | |

The XMOS flash UID is printed as `XMOS flash UID: ...` when flashing starts.

## Pass Evidence

For each case, retain:

- The complete serial log from the initiating action through the terminal state.
- The exact point where power was removed.
- Whether Wi-Fi and Home Assistant provisioning survived.
- The HAT flash UID used before and after recovery.
- The final XMOS firmware version and audio result.

## HWT-XMOS-01: Same-Boot Flash With Speaker

1. Select the internal speaker output.
2. Start media playback and confirm audible output.
3. Enable and press the disabled-by-default diagnostic entity
   `Flash XMOS Firmware (Same Boot)`.
4. Do not interrupt power.

Pass criteria:

- The log contains `Audio shutdown complete; XMOS is ready for direct flashing`.
- Wake word, voice assistant, media player, microphone RX, speaker TX, line-out,
  and TAS2780 all stop before direct flashing begins.
- Flashing reaches 100 percent without an I2S ownership or DAC activation error.
- XMOS reconnects and the internal speaker route becomes usable again.
- ESP provisioning is unchanged.

## HWT-XMOS-02: Same-Boot Flash With Line-Out

1. Connect the line-out jack and select line-out.
2. Start media playback and confirm line-out audio.
3. Press `Flash XMOS Firmware (Same Boot)`.
4. Do not interrupt power.

Pass criteria:

- The shutdown-complete log appears before flashing begins.
- PCM5122 is muted during direct flashing without changing the saved user mute
  preference.
- Flashing succeeds, XMOS reconnects, and line-out is restored.
- No pop, sustained noise, or stale playback is observed.

## HWT-XMOS-03: Power Loss During Full Erase

1. Press the diagnostic entity `XMOS Full Erase and Reinstall`.
2. Allow the requested reboot to occur.
3. After erase progress is visible but before
   `XMOS erase complete; writing embedded image`, remove all device power,
   including USB power.
4. Restore power and reconnect logging if necessary.
5. Allow recovery to complete.

Pass criteria:

- Boot logs contain `Resuming interrupted XMOS full erase`.
- Erase progress restarts and covers the complete 8 MiB device.
- The embedded image is written and XMOS reconnects.
- ESP Wi-Fi and Home Assistant provisioning remain intact.
- No ESP factory reset occurs because this is the diagnostic full-erase action.

## HWT-XMOS-04: Power Loss During Image Write

1. Start `XMOS Full Erase and Reinstall`.
2. Wait for `XMOS erase complete; writing embedded image`.
3. Remove all device power, including USB power, immediately while image
   writing is in progress.
4. Restore power, reconnect logging if necessary, and allow recovery to
   complete.

Pass criteria:

- Boot logs contain `Resuming interrupted XMOS full erase`.
- Recovery repeats the full erase rather than only erasing the 1 MiB boot
  partition.
- The embedded image is rewritten successfully and XMOS reconnects.
- ESP provisioning remains intact.

## HWT-XMOS-05: Failed Factory Reset Preserves ESP Preferences

1. Provision the ESP and record a setting that proves preferences are intact.
2. Hold the physical action button for at least 22 seconds to request factory
   reset.
3. After the requested reboot, wait until erase progress begins, then remove all
   device power, including USB power.
4. Confirm both boards are unpowered, disconnect the HAT, then restore power to
   the ESP to force flash initialization failure.
5. Wait for the flashing failure indication.

Pass criteria:

- XMOS flashing fails without pressing the ESP factory-reset button.
- Existing Wi-Fi and Home Assistant provisioning remain intact.
- The device remains available for logs, OTA, or a later recovery reboot.
- Reconnecting the original HAT and rebooting resumes the factory-reset full
  erase.

Restore the original HAT with all power removed before continuing.

## HWT-XMOS-06: Replacement HAT Is Rejected During Recovery

This case requires a second HAT with a different flash UID.

1. Start `XMOS Full Erase and Reinstall` with the primary HAT.
2. Remove all device power, including USB power, during erase progress.
3. Confirm both boards are unpowered, then replace the primary HAT with the
   secondary HAT.
4. Restore power and capture serial logs.

Pass criteria:

- The secondary UID differs from the persisted primary UID.
- The log contains `XMOS flash UID changed; refusing to resume interrupted recovery`.
- No erase progress begins on the secondary HAT.
- ESP preferences remain intact.

Restore the primary HAT while power is removed, reboot, and allow recovery to
complete.

## HWT-XMOS-07: Successful Production Factory Reset

Run this case last because it intentionally erases ESP preferences.

1. Hold the physical action button for at least 22 seconds.
2. Do not interrupt the requested reboot, full erase, or image write.
3. Wait for XMOS flashing success and the subsequent ESP reboot.

Pass criteria:

- The boot log contains `Starting requested XMOS factory-reset full erase`.
- All 8 MiB are erased and the embedded image is written successfully.
- ESP factory reset occurs only after XMOS flashing succeeds.
- Previous Wi-Fi and Home Assistant provisioning are removed.
- After reprovisioning, XMOS boots with the embedded firmware and both audio
  routes operate normally.

## Results

| Test | Result | Log path | Notes |
|---|---|---|---|
| HWT-XMOS-01 | | | |
| HWT-XMOS-02 | | | |
| HWT-XMOS-03 | | | |
| HWT-XMOS-04 | | | |
| HWT-XMOS-05 | | | |
| HWT-XMOS-06 | | | |
| HWT-XMOS-07 | | | |
