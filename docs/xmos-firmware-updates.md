# XMOS firmware versions and automatic flashing

Satellite1 embeds the XMOS image selected by `substitutions.xmos_fw_version`
in the ESP32 firmware. The Satellite1 `memory_flasher` platform requires that
the resolved embedded `image_version` is **v1.1.0-alpha.0 or newer**, including
later alpha iterations, beta/RC releases and stable releases. Version numbers
are compared numerically, not alphabetically.

If validation rejects an older image, update your device YAML:

```yaml
substitutions:
  xmos_fw_version: "v1.1.0-alpha.0"
```

Alternatively, remove your stale `xmos_fw_version` override to use the package
default. Update both the YAML package and external component references to a
repository revision containing this protection. Customers pinned to older
repository/component revisions cannot receive the new validator until they
update those references; changing the version substitution alone does not
update the validator.

## Automatic versus manual flashing

Updating YAML selects the image embedded during the next build; it does not
flash hardware by itself. Build and install the ESP32 firmware to apply it.
By default, production, dashboard and custom builds based on
`satellite1.base.yaml` automatically flash that embedded image on startup if
XMOS does not respond or its version differs. Existing image-presence,
initialization, pending-flash and once-per-boot attempt guards still apply.

To retain manual control, override:

```yaml
substitutions:
  xmos_auto_flash_on_boot: "false"
```

This disables automatic boot flashing, **not** the embedded-image minimum
version check. The dashboard's manual XMOS flash button remains available and
uses `memory_flasher.request_embedded_flash_reboot` to request flashing across
a reboot.

## Compatibility-check opt-out

For an intentionally older embedded image, an advanced configuration can opt
out of the minimum check. Merge this into the existing flasher by its ID;
do not add a second flasher:

```yaml
memory_flasher:
  - id: !extend xflash
    skip_version_check: true
```

In a standalone flasher definition, put `skip_version_check: true` alongside
`platform: satellite1` and `embed_flash_image`, not inside the image block.
It defaults to `false`. Enabling it emits an explicit compatibility-bypass
warning and permits potentially incompatible versions. It bypasses **only**
the minimum-version comparison: version syntax, checksum validation and all
flashing safeguards remain enabled.

`skip_version_check` is independent of `xmos_auto_flash_on_boot`. Opting out
of the compatibility check does not disable automatic flashing; set the
auto-flash substitution to `"false"` separately if manual flashing is required.
The minimum is a build-time check of the selected embedded image, not a
runtime check of the version already running on XMOS, and does not add a
Satellite1 policy to the generic `memory_flasher` component or download actions.
