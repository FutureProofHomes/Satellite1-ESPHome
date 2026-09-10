"""Apply the required ESP-IDF 5.5.5 W5500 receive-buffer patch."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess


W5500_PSRAM_PATCH = (
    Path(__file__).parent / "patches" / "esp-idf-5.5.5-w5500-psram-rx-buffer.patch"
)


class W5500PsramPatchError(RuntimeError):
    """The active ESP-IDF source cannot safely receive the required patch."""


def _run_patch(
    idf_path: Path, *, reverse: bool = False, dry_run: bool = False
) -> subprocess.CompletedProcess[bytes]:
    command = ["patch", "--batch", "--forward"]
    if dry_run:
        command.append("--dry-run")
    if reverse:
        command.append("-R")
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

    if not W5500_PSRAM_PATCH.is_file():
        raise W5500PsramPatchError(f"Missing W5500 PSRAM patch: {W5500_PSRAM_PATCH}")

    try:
        if _run_patch(idf_path, dry_run=True).returncode == 0:
            if _run_patch(idf_path).returncode == 0:
                return True
        elif _run_patch(idf_path, reverse=True, dry_run=True).returncode == 0:
            return False
    except OSError as err:
        raise W5500PsramPatchError("The system 'patch' command is required.") from err

    raise W5500PsramPatchError(
        "The active ESP-IDF source does not match the expected W5500 PSRAM "
        "patch state. Use a clean ESP-IDF 5.5.5 framework."
    )
