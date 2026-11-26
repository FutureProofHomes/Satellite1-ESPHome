import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    "prepare_speech_tracking_test", Path(__file__).resolve().parents[1] / "scripts/prepare_speech_tracking_test.py")
PREPARE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARE)


class SpeechTrackingPackageTest(unittest.TestCase):
    def test_overlay_verifies_payload_and_keeps_production_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "config").mkdir()
            production = root / "config/satellite1.yaml"
            production.write_text("production image settings\n")
            artifacts = root / "artifacts"
            image = artifacts / "speech_tracking/satellite1_xmos.factory.bin"
            image.parent.mkdir(parents=True)
            image.write_bytes(b"new XMOS image")
            digest = hashlib.md5(image.read_bytes()).hexdigest()
            image.with_suffix(".md5").write_text(digest + "\n")
            metadata = {"target": "satellite1_firmware_speech_tracking", "host_hints": True,
                        "version": "v1.2.0-dev.16", "md5": digest,
                        "sha256": hashlib.sha256(image.read_bytes()).hexdigest(), "bytes": image.stat().st_size}
            manifest = artifacts / "manifest.json"
            manifest.write_text(json.dumps({"speech_tracking": metadata}))
            output = root / "config/satellite1.speech-tracking-test.yaml"
            with patch.object(PREPARE, "ROOT", root):
                PREPARE.prepare(artifacts, output)
                text = output.read_text()
                self.assertIn('xmos_audio_hints: "true"', text)
                self.assertIn('xmos_fw_version: "v1.2.0-dev.16"', text)
                self.assertIn(str(image), text)
                self.assertEqual(production.read_text(), "production image settings\n")
                with self.assertRaisesRegex(SystemExit, "production files"):
                    PREPARE.prepare(artifacts, production)
                image.write_bytes(b"corrupted image")
                with self.assertRaisesRegex(SystemExit, "verified manifest"):
                    PREPARE.prepare(artifacts, output)
                image.write_bytes(b"new XMOS image")
                image.with_suffix(".md5").write_text("wrong\n")
                with self.assertRaisesRegex(SystemExit, "MD5"):
                    PREPARE.prepare(artifacts, output)
                metadata["host_hints"] = False
                manifest.write_text(json.dumps({"speech_tracking": metadata}))
                with self.assertRaisesRegex(SystemExit, "supporting"):
                    PREPARE.prepare(artifacts, output)


if __name__ == "__main__":
    unittest.main()
