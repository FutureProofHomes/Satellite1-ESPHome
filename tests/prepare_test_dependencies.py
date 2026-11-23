"""Cache the pinned upstream ArduinoJson header used by the host protocol tests."""
import hashlib
import pathlib
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "build/test-dependencies/ArduinoJson.h"
URL = "https://github.com/bblanchon/ArduinoJson/releases/download/v7.4.2/ArduinoJson-v7.4.2.h"
SHA256 = "a05ac98f4481d2398c103ca5ffcce8e2fcd7fa2fa1d8d9f38d5757454f448d97"


def prepare():
    data = HEADER.read_bytes() if HEADER.exists() else urllib.request.urlopen(URL, timeout=30).read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise RuntimeError("ArduinoJson test dependency checksum mismatch")
    HEADER.parent.mkdir(parents=True, exist_ok=True)
    HEADER.write_bytes(data)
    print("ArduinoJson host-test header verified.")


if __name__ == "__main__":
    prepare()
