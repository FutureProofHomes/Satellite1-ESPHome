from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "esphome"
    / "components"
    / "ethernet"
    / "w5500_psram_patch.py"
)
SPEC = importlib.util.spec_from_file_location("w5500_psram_patch", MODULE_PATH)
assert SPEC and SPEC.loader
w5500_psram_patch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(w5500_psram_patch)


def _unpatched_source() -> str:
    patch_lines = w5500_psram_patch.W5500_PSRAM_PATCH.read_text(
        encoding="utf-8"
    ).splitlines()
    hunk_start = next(i for i, line in enumerate(patch_lines) if line.startswith("@@"))
    source_lines = [
        line[1:]
        for line in patch_lines[hunk_start + 1 :]
        if line.startswith((" ", "-"))
    ]
    return "\n" * 682 + "\n".join(source_lines) + "\n"


class W5500PsramPatchTest(unittest.TestCase):
    def _idf_tree(self) -> tuple[Path, Path]:
        path = Path(self.temporary_directory)
        target = path / "components/esp_eth/src/spi/w5500/esp_eth_mac_w5500.c"
        target.parent.mkdir(parents=True)
        return path, target

    def setUp(self) -> None:
        self.temporary_directory = self.enterContext(tempfile.TemporaryDirectory())

    def test_applies_patch_once_and_accepts_verified_state(self) -> None:
        idf_path, target = self._idf_tree()
        target.write_text(_unpatched_source(), encoding="utf-8")

        dry_run = w5500_psram_patch._run_patch(idf_path, dry_run=True)
        self.assertEqual(dry_run.returncode, 0, dry_run.stdout.decode())
        self.assertTrue(w5500_psram_patch.ensure_w5500_psram_patch(idf_path))
        self.assertIn("heap_caps_malloc", target.read_text(encoding="utf-8"))
        self.assertFalse(w5500_psram_patch.ensure_w5500_psram_patch(idf_path))

    def test_rejects_unknown_framework_source(self) -> None:
        idf_path, target = self._idf_tree()
        target.write_text("unknown framework source\n", encoding="utf-8")

        with self.assertRaises(w5500_psram_patch.W5500PsramPatchError):
            w5500_psram_patch.ensure_w5500_psram_patch(idf_path)

    def test_rejects_source_with_both_allocation_modes(self) -> None:
        idf_path, target = self._idf_tree()
        target.write_text(
            _unpatched_source() + w5500_psram_patch.PATCHED_ALLOCATION + "\n",
            encoding="utf-8",
        )

        with self.assertRaises(w5500_psram_patch.W5500PsramPatchError):
            w5500_psram_patch.ensure_w5500_psram_patch(idf_path)
