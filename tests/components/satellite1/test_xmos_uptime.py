r"""Host regressions for the production XMOS boot-settle guard.

Run from the repository root using its virtual environment:
    .venv/bin/python -m unittest discover -s tests/components/satellite1 -v
Baseline (expected regression failures):
    SATELLITE1_TEST_REVISION=d70ae7ce5ee111a7d7de5ec5d00c5fd2ae486c52 \
    .venv/bin/python -m unittest discover \
    -s tests/components/satellite1 -v
Requires a C++17 compiler (CXX, or c++). No ESPHome installation required.
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


HERE: Path = Path(__file__).resolve().parent
ROOT: Path = HERE.parents[2]
COMPONENT: Path = Path("esphome/components/satellite1")


class XmosUptimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls: type["XmosUptimeTest"]) -> None:
        cls.temp = tempfile.TemporaryDirectory(prefix="satellite1-uptime-")
        cls.addClassCleanup(cls.temp.cleanup)
        build = Path(cls.temp.name)
        revision = os.environ.get("SATELLITE1_TEST_REVISION")
        source = ROOT / COMPONENT
        if revision:
            source = build / "baseline"
            source.mkdir()
            for name in ("satellite1.cpp", "satellite1.h"):
                contents = subprocess.check_output(
                    ["git", "show", f"{revision}:{COMPONENT / name}"], cwd=ROOT
                )
                (source / name).write_bytes(contents)

        # Only platform boundaries are stubbed; compile the complete production TU.
        for name in (
            "esp_rom_gpio.h",
            "esphome/core/log.h",
            "esphome/core/component.h",
            "esphome/core/gpio.h",
            "esphome/components/spi/spi.h",
        ):
            header = build / name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text('#include "host_stubs.h"\n')

        cls.binary = build / "xmos_uptime"
        command = shlex.split(os.environ.get("CXX", "c++")) + [
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Wno-unused-parameter",
            "-Wno-unused-variable",
            "-I",
            str(build),
            "-I",
            str(HERE),
            "-I",
            str(source),
            str(source / "satellite1.cpp"),
            str(HERE / "xmos_uptime.cpp"),
            "-o",
            str(cls.binary),
        ]
        # Baseline lacks this field. Keep behavioral tests runnable against it,
        # rather than treating a missing member/compiler error as regression proof.
        if "xmos_boot_settle_pending_" in (source / "satellite1.h").read_text():
            command.insert(1, "-DHAS_SETTLE_PENDING")
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(f"Host compilation failed:\n{result.stdout}{result.stderr}")

    def run_case(self, name: str) -> None:
        result = subprocess.run([str(self.binary), name], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_fresh_high_uptime(self) -> None:
        self.run_case("fresh_high")

    def test_fresh_uint32_wrap(self) -> None:
        self.run_case("fresh_wrap")

    def test_release_boundary(self) -> None:
        self.run_case("boundary")

    def test_loop_clears_pending(self) -> None:
        self.run_case("loop_clear")

    def test_transfer_clears_without_loop(self) -> None:
        self.run_case("transfer_clear")

    def test_settled_long_uptime_does_not_reblock(self) -> None:
        self.run_case("long_uptime")

    def test_deadline_wrap(self) -> None:
        self.run_case("deadline_wrap")

    def test_repeated_release_rearms(self) -> None:
        self.run_case("rearm")

    def test_direct_access_blocks(self) -> None:
        self.run_case("direct")

    def test_boot_recovery_loop_guard(self) -> None:
        self.run_case("recovery")


if __name__ == "__main__":
    unittest.main()
