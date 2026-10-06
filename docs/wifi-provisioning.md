# WiFi Provisioning

## Scope Revision

This change is boot-only provisioning. Manual credential reset is deferred to a
separate upstream WiFi API PR. There is no recovery chord, credential-erasure
action, GPIO0 shutdown guard, or application-created credential tombstone.
Satellite1 uses builtin WiFi from the pinned ESPHome 2026.9.1, not a WiFi override.
The existing Action-button gestures, factory reset, XMOS checks, and wake-word
startup logic remain unchanged from `develop`.

## Configuration Modes

Production firmware includes `config/common/wifi_provisioning.yaml`. Dashboard
firmware offers two modes in its existing remote package file list:

1. **Provisioning (default):** keep the provisioning package and omit all YAML
   WiFi networks. Credentials are supplied over BLE Improv or USB Improv.
2. **Managed credentials:** remove the provisioning package and set `wifi.ssid`
   and `wifi.password` (or `wifi.networks`) in YAML. This mode has no BLE/USB
   Improv or boot provisioning policy.

The base package includes only `common/wifi.yaml`. The ESP32-only
`wifi_provisioning` component is a schema-only validator with no runtime component
or ID. It rejects normalized nonempty `wifi.networks`, including shorthand SSIDs
and credentials added by packages. It adds no feature build flags or Ethernet
dependency. Both external-component source lists select only this validator,
not WiFi.

### Existing Dashboard Configurations

Updating remote packages does not rewrite an already-adopted local dashboard
configuration. Choose a mode explicitly using the updated dashboard template:

- For provisioned credentials, add `config/common/wifi_provisioning.yaml` before
  `config/satellite1.base.yaml` in the package file list and omit YAML WiFi networks.
- For configuration-managed credentials, omit the provisioning package and retain
  the configured `wifi.ssid`/`wifi.password` or `wifi.networks`.

Serial Improv is no longer implicitly included by the base package. Existing
configurations without the optional provisioning package can still reconnect
using saved credentials, but do not include Improv. This change does not erase
saved WiFi credentials, API keys, or other preferences.

## Boot Lifecycle

- BLE starts disabled. After WiFi setup loads saved credentials (priority 250),
  the priority-200 boot hook checks `has_sta()`.
- With no credentials, BLE is enabled immediately. With saved credentials,
  the device gets a 90-second connection grace period before enabling BLE.
- The first WiFi connection latches `wifi_ever_connected` and cancels the boot
  script immediately. A later outage never enables BLE in that boot, even if
  WiFi internally requests that Improv start again.
- Each connection runs a restartable finish script. If BLE is active, it waits
  five seconds for Improv results, then requests BLE disable. It waits for
  `not: ble.enabled` and retains the existing one-second allocation-release
  heuristic. Inactivity is not proof of completed teardown. Normal saved-credential
  connections skip the five-second delay.
- Disconnect cancels the finish script and disables BLE if WiFi has connected
  at least once in this boot. Failed initial provisioning does not close its
  BLE session. Reconnect runs the finish script without reopening provisioning.
- Production appends its update check to the finish script using `!extend`.
  Dashboard firmware has no reference to the production update component.
  The provisioning package precedes the base package so voice-assistant client
  readiness waits for the finish script.

The existing 15-minute WiFi reboot timeout is unchanged. An outage may therefore
eventually reboot the device, creating a new boot with a fresh grace period.
Production has no new provisioning-window timeout or runtime BLE recovery policy.

If WiFi drops during the five-second Improv response delay, BLE shutdown can
interrupt delivery of the final response. The client must not infer successful
provisioning from that disconnect alone. WiFi retries the saved credentials;
only a new boot can open another fallback provisioning session.

## Reprovisioning Now

Reboot the device when the saved network is unavailable, then wait for the
90-second boot fallback and supply replacement credentials through Improv.
The fallback retains the saved credentials while waiting; it does not erase
them. If the saved network connects during that boot, BLE provisioning will not
open. USB Improv remains available in provisioning builds for supplying new
credentials. The existing factory-reset workflow remains a broader fallback
when a complete reset is wanted. No new recovery gesture is needed or provided.
Managed-credential builds must instead update their YAML credentials and firmware.

## Separate HIL Profile

Test-only firmware lives on branch `local/satellite1-hil-tests`, at
`tests/hardware/firmware/satellite1.improv-hil.yaml`; its procedure and acceptance
matrix are in `tests/hardware/README.md` and `tests/hardware/acceptance.md` on that
branch. The no-authorizer, forced-window profile is not part of this production
proposal and must never be distributed as production firmware. Historical logs
and artifacts from the earlier profile remain historical evidence, not validation
of the reorganized build. Physical HIL execution requires separate approval.

## Validation

`tests/config/wifi_provisioning.yaml` and `wifi_credentials.yaml` are local
dashboard equivalents, both including `dashboard_build.yaml` and a local component
source. The credential-only fixture omits the provisioning package.
`wifi_provisioning_invalid_ssid.yaml` and
`wifi_provisioning_invalid_networks.yaml` must fail with the validator's credential
conflict error. Production validation checks the extended update script.

`tests/test_wifi_provisioning.py` inspects the actual YAML lifecycle and validates
the actual merged configuration graphs using ESPHome and its existing `yaml_util`.
It checks boot ordering, the initial timer's cancellation, disconnect cancellation
without BLE enable, response/teardown delay ordering, dashboard package omission,
and rejection of normalized shorthand and package-supplied networks. These are
configuration regression tests, not a hardware model or proof of BLE teardown,
physical provisioning, or firmware execution. Production and both local
dashboard-equivalent variants also compile with unmodified ESPHome 2026.9.1.
Production boot fallback and runtime-disconnect policies still require direct
device testing; the separate forced-window HIL profile does not establish them.
