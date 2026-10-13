# LD2410 Reply Validation Host Tests

Run from the repository root with a C++17 compiler and Python 3:

```sh
python3 tests/ld2410/run_tests.py
CXX=clang++ python3 tests/ld2410/run_tests.py stream_recover
```

The optional argument selects test names by substring. The runner compiles into a
temporary directory, enables AddressSanitizer and UndefinedBehaviorSanitizer, keeps
assertions enabled, and runs each named case in a separate process. It prints
PASS/FAIL, assertion/sanitizer diagnostics, and totals, returning nonzero on failure.
No build artifacts are written into the checkout. No hardware or external Python
packages are required.

## Scope

These tests cover A5a only: validation and atomic adoption of complete successful
LD2410 replies, plus UART stream framing/resynchronization. They compile the actual
`esphome/components/satellite1_radar/ld2410_handler.cpp`, its actual header, and
`radar_entities.h`. There is no copied parser, replacement radar entity header, or
production test macro. A subclass exposes the protected ACK entry point and waiting
state; the stream tests inject bytes into the UART consumed by the real `loop()`.

The common fake ESPHome header and thin include wrappers implement only the APIs
needed to compile this handler. Entity configuration/registration is discarded;
firmware text publication is recorded. Preferences record saved bytes and save/sync
counts, never load a value, and the fake clock is explicitly controlled. Initialization
uses real `setup()` but clears its query queue before receive tests. Each fixture sets
a distinct initial configuration and clears its initialization persistence counts.
Configuration comparisons are fieldwise rather than comparing structure padding.

Fixtures cover the 38-byte parameter reply (marker AA, supported gate 8, current gates
7/6, distinct thresholds, timeout `0x1234`), all truncated prefixes and shorter complete
frames with corrected declared lengths and preserved footers, framing corruption,
command high byte, both status bytes, invalid gates/thresholds/resolution, 16-byte
resolution replies, 20-byte real/disabled-sentinel MAC replies, and 22-byte firmware
version replies (including rejection of 18 bytes), and 18-byte configuration-entry
replies. Malformed replies must preserve configuration, avoid save/sync/publication,
and keep the ACK wait held. Complete failed ACKs check both status bytes, preserve
configuration, and release the wait without requiring successful-response data or
delaying the next queued command. Valid successful replies
must adopt the expected complete configuration and persist exactly once where
appropriate. Stream cases cover every two-part split, bytewise input, noise/partial
header retention, malformed candidates consuming subsequent header bytes, oversize
and impossible lengths, consecutive frames, and the 128-byte loop budget.

Successful valid readback still overwrites desired configuration: these tests
intentionally preserve that current behavior. A5b desired-vs-observed separation,
apply/restart behavior, and factory-reset issues remain out of scope, as do hardware
timing, full ESPHome integration, command correlation, and general data-frame decoding.
The Bluetooth ACK test checks only fake-clock delayed readback scheduling.

The suite is expected to fail against the pre-A5a implementation.

## Integration Validation

Compile the supported firmware with the ESPHome version pinned in `requirements.txt`:

```sh
esphome compile config/satellite1.yaml
```

The A5a branch was checked with ESPHome 2026.9.1: the full firmware compiled and
393 host cases passed under AddressSanitizer/UndefinedBehaviorSanitizer.
No hardware was flashed or tested as part of this validation.

Before hardware sign-off, capture a real parameter response from an LD2410 with
known, distinctive gates, all 18 thresholds, and timeout. Compare the decoded and
persisted fields with the UART capture, check resolution and Bluetooth MAC replies,
and confirm presence reporting continues after queries. Do not treat successful
readback as verification of an apply or reset; those guarantees belong to A5b-A5d.
