#!/usr/bin/env python3
"""Compile the real LD2410 handler and run isolated, sanitizer-enabled assertions."""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filter", nargs="?", default="", help="Run names containing this substring")
    args = parser.parse_args()
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    with tempfile.TemporaryDirectory(prefix="ld2410-tests-") as temporary:
        executable = Path(temporary) / "test_ld2410"
        command = shlex.split(os.environ.get("CXX", "c++")) + [
            "-std=c++17", "-g", "-O1", "-UNDEBUG", "-fno-omit-frame-pointer",
            "-fsanitize=address,undefined", "-I", str(here / "fakes"), "-I", str(root),
            str(here / "test_ld2410.cpp"),
            str(root / "esphome/components/satellite1_radar/ld2410_handler.cpp"),
            "-o", str(executable),
        ]
        print("Compile:", shlex.join(command), flush=True)
        subprocess.run(command, check=True)
        env = dict(os.environ, UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1")
        names = subprocess.check_output([executable, "--list"], text=True, env=env).splitlines()
        names = [name for name in names if args.filter in name]
        if not names:
            parser.error("No tests matched")
        failed = []
        for name in names:
            result = subprocess.run([executable, name], capture_output=True, text=True, env=env)
            if result.returncode:
                failed.append(name)
                print(f"FAIL {name} (exit {result.returncode})")
                print(result.stdout + result.stderr, end="", flush=True)
            else:
                print(result.stdout, end="", flush=True)
        print(f"Result: {len(names) - len(failed)} passed, {len(failed)} failed, {len(names)} total")
        return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
