#!/usr/bin/env python3
"""Create an opt-in local test image without changing FPH's XMOS selection."""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def prepare(artifacts: Path, output: Path) -> None:
    manifest = json.loads((artifacts / "manifest.json").read_text())
    metadata = manifest["speech_tracking"]
    if metadata.get("target") != "satellite1_firmware_speech_tracking" or not metadata.get("host_hints"):
        raise SystemExit("The artifact manifest does not identify a supporting speech_tracking image")
    image = (artifacts / "speech_tracking/satellite1_xmos.factory.bin").resolve()
    md5 = image.with_suffix(".md5")
    data = image.read_bytes()
    if hashlib.sha256(data).hexdigest() != metadata["sha256"] or len(data) != metadata["bytes"]:
        raise SystemExit("XMOS image does not match its verified manifest")
    digest = hashlib.md5(data).hexdigest()
    if digest != metadata["md5"] or md5.read_text().strip() != digest:
        raise SystemExit("XMOS MD5 does not match the test image")
    # Output beside the production config so relative component/secrets paths
    # have exactly the same meaning. The generated filename is ignored by Git.
    if output.resolve() != (ROOT / "config/satellite1.speech-tracking-test.yaml").resolve():
        raise SystemExit("Use config/satellite1.speech-tracking-test.yaml; production files must not be overwritten")
    quote = json.dumps
    output.write_text(f'''# Generated local test only; FPH's default image/version/URLs are unchanged.
substitutions:
  node_name: sat1-tracking-test
  friendly_name: Sat1 Speech Tracking Test
  esp32_fw_version: speech-tracking-test
  xmos_audio_hints: "true"
  xmos_fw_version: {quote(metadata["version"])}

packages:
  firmware: !include satellite1.yaml

memory_flasher:
  - id: !extend xflash
    embed_flash_image:
      image_version: {quote(metadata["version"])}
      image_file:
        type: local
        path: {quote(str(image))}
      md5_file:
        type: local
        path: {quote(str(md5))}
''')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xmos-artifacts", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "config/satellite1.speech-tracking-test.yaml")
    args = parser.parse_args()
    prepare(args.xmos_artifacts.resolve(), args.output.resolve())
    print(args.output.resolve())


if __name__ == "__main__":
    main()
