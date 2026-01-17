# Satellite1 DoA (SPI)

Reads direction-of-arrival results computed on the XMOS firmware over SPI.

## Example
```yaml
sensor:
  - platform: satellite1
    satellite1: satellite1_core
    azimuth:
      name: "DoA Azimuth"
    confidence:
      name: "DoA Confidence"
    delay_offsets_samples: [0.0, -0.2, 0.05, 0.12]
```

Notes:
- Requires XMOS firmware with DoA capability enabled.
- The component probes DoA capability via `GET_CAPS` before reading results.
- Offsets require capability version >= 0x02.
