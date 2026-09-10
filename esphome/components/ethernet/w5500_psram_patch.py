"""Apply the required ESP-IDF 5.5.5 W5500 receive-buffer patch."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


W5500_PSRAM_PATCH = (
    Path(__file__).parent / "patches" / "esp-idf-5.5.5-w5500-psram-rx-buffer.patch"
)
W5500_SOURCE = Path("components/esp_eth/src/spi/w5500/esp_eth_mac_w5500.c")
UNPATCHED_ALLOCATION = "*buf = malloc(copy_len);"
PATCHED_ALLOCATION = (
    "*buf = heap_caps_malloc(copy_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"
)


class W5500PsramPatchError(RuntimeError):
    """The active ESP-IDF source cannot safely receive the required patch."""


def _run_patch(
    idf_path: Path, *, dry_run: bool = False
) -> subprocess.CompletedProcess[bytes]:
    command = ["patch", "--batch", "--forward"]
    if dry_run:
        command.append("--dry-run")
    command.extend(["-p1", "-d", str(idf_path)])
    return subprocess.run(
        command,
        input=W5500_PSRAM_PATCH.read_bytes(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def ensure_w5500_psram_patch(idf_path: Path | None = None) -> bool:
    """Apply the patch and return whether it changed the framework source."""
    if idf_path is None:
        idf_path_value = os.environ.get("IDF_PATH")
        if not idf_path_value:
            raise W5500PsramPatchError(
                "IDF_PATH is required to identify the native ESP-IDF framework."
            )
        idf_path = Path(idf_path_value)

    source_path = idf_path / W5500_SOURCE
    if not source_path.is_file():
        raise W5500PsramPatchError(f"Missing ESP-IDF W5500 source: {source_path}")
    if not W5500_PSRAM_PATCH.is_file():
        raise W5500PsramPatchError(f"Missing W5500 PSRAM patch: {W5500_PSRAM_PATCH}")

    source = source_path.read_text(encoding="utf-8")
    if PATCHED_ALLOCATION in source:
        if UNPATCHED_ALLOCATION not in source:
            return False
        raise W5500PsramPatchError(
            "The active ESP-IDF W5500 source has conflicting receive-buffer "
            "allocations. Use a clean ESP-IDF 5.5.5 framework."
        )
    if UNPATCHED_ALLOCATION not in source:
        raise W5500PsramPatchError(
            "The active ESP-IDF W5500 source has an unknown receive-buffer "
            "allocation. Use a clean ESP-IDF 5.5.5 framework."
        )

    try:
        if _run_patch(idf_path, dry_run=True).returncode == 0:
            if _run_patch(idf_path).returncode == 0:
                source = source_path.read_text(encoding="utf-8")
                if PATCHED_ALLOCATION in source and UNPATCHED_ALLOCATION not in source:
                    return True
    except OSError as err:
        raise W5500PsramPatchError("The system 'patch' command is required.") from err

    raise W5500PsramPatchError(
        "The active ESP-IDF source does not match the expected W5500 PSRAM "
        "patch state. Use a clean ESP-IDF 5.5.5 framework."
    )
