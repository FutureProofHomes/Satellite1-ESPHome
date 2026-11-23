"""Exercise the actual firmware TCP client against independent Wyoming wire fixtures."""
import json
import pathlib
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "esphome/components/stt_wake_verifier"


def event(kind, data=None, payload=b"", inline=False):
    header = {"type": kind}
    body = b""
    if data is not None:
        if inline:
            header["data"] = data
        else:
            body = json.dumps(data, ensure_ascii=False).encode()
            header["data_length"] = len(body)
    if payload:
        header["payload_length"] = len(payload)
    return json.dumps(header).encode() + b"\n" + body + payload


class WyomingClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.binary = pathlib.Path(cls.work.name) / "wyoming-client"
        dependency = ROOT / "build/test-dependencies"
        if not (dependency / "ArduinoJson.h").exists():
            raise RuntimeError("Run python tests/prepare_test_dependencies.py first")
        subprocess.run([
            shutil.which("c++") or "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-I", str(COMPONENT), "-I", str(dependency),
            str(ROOT / "tests/wyoming_client_host.cpp"), str(COMPONENT / "wyoming_client.cpp"),
            "-o", str(cls.binary),
        ], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.work.cleanup()

    def exchange(self, response, *, delay=0, fragmented=False, timeout=1500, pcm_size=64000):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(3)
        port = server.getsockname()[1]
        errors = []

        def serve():
            try:
                with server, server.accept()[0] as client:
                    client.settimeout(3)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    reader = client.makefile("rb")
                    types, audio = [], bytearray()
                    while True:
                        header = json.loads(reader.readline())
                        types.append(header["type"])
                        data = dict(header.get("data", {}))
                        if header.get("data_length"):
                            data.update(json.loads(reader.read(header["data_length"])))
                        payload = reader.read(header.get("payload_length", 0))
                        if header["type"] in ("audio-start", "audio-chunk"):
                            self.assertEqual(data, {"rate": 16000, "width": 2, "channels": 1})
                        if header["type"] == "audio-chunk":
                            self.assertLessEqual(len(payload), 4096)
                            audio.extend(payload)
                        if header["type"] == "audio-stop":
                            break
                    self.assertEqual(types[:2], ["transcribe", "audio-start"])
                    self.assertTrue(all(t == "audio-chunk" for t in types[2:-1]))
                    self.assertEqual(audio, bytes(i % 251 for i in range(pcm_size)))
                    time.sleep(delay)
                    if response:
                        if fragmented:
                            for i in range(0, len(response), 7):
                                client.sendall(response[i:i + 7])
                                time.sleep(0.001)
                        else:
                            client.sendall(response)
                    reader.close()
            except (BrokenPipeError, ConnectionResetError):
                if not delay:
                    errors.append("Unexpected client disconnect")
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=serve, daemon=True)
        thread.start()
        started = time.monotonic()
        result = subprocess.run([
            str(self.binary), "run", f"127.0.0.1:{port}", str(timeout), str(pcm_size),
        ], text=True, capture_output=True, check=True, timeout=4)
        elapsed = time.monotonic() - started
        thread.join(4)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors, errors)
        return result.stdout.split("\n", 1), elapsed

    def test_server_addresses(self):
        for address, host, port in [
            ("192.168.1.50:10300", "192.168.1.50", "10300"),
            ("  stt-server.local:10300  ", "stt-server.local", "10300"),
            ("tcp://localhost:1", "localhost", "1"),
            ("wyoming://stt:65535", "stt", "65535"),
            ("[::1]:10300", "::1", "10300"),
        ]:
            with self.subTest(address=address):
                result = subprocess.check_output([str(self.binary), "parse", address], text=True)
                self.assertEqual(result.splitlines(), [host, port])
        for address in ["", "stt", ":10300", "stt:0", "stt:65536", "stt:-1", "stt:one",
                        "http://stt:10300", "stt:10300/path", "stt\nother:10300", "::1:10300",
                        "[]:10300", "stt:9999999999999999999"]:
            with self.subTest(address=address):
                self.assertEqual(subprocess.run([str(self.binary), "parse", address]).returncode, 2)

    def test_raw_pcm_and_fragmented_transcript(self):
        result, _ = self.exchange(event("transcript", {"text": "Hey Jarvis — café"}), fragmented=True)
        self.assertEqual(result, ["ok", "Hey Jarvis — café"])

    def test_inline_text_and_empty_result(self):
        for text in ["Hey Jarvis", ""]:
            with self.subTest(text=text):
                result, _ = self.exchange(event("transcript", {"text": text}, inline=True), pcm_size=16000)
                self.assertEqual(result, ["ok", text])

    def test_skip_metadata_payload_and_streaming_revisions(self):
        response = event("info", {"asr": []}, payload=b"ignored payload\n")
        response += event("transcript-start")
        response += event("transcript-chunk", {"text": "Hey"})
        response += event("transcript-chunk", {"text": "Hey Jarvis"})
        response += event("transcript-stop")
        result, _ = self.exchange(response, fragmented=True)
        self.assertEqual(result, ["ok", "Hey Jarvis"])

    def test_invalid_and_failed_responses(self):
        fixtures = [
            event("error", {"text": "Provider details are not surfaced"}),
            b'not json\n', b'{"type":"transcript","data_length":-1}\n',
            b'{"type":"transcript","data_length":4097}\n',
            b'{"type":"transcript","payload_length":8193}\n',
            event("transcript", {"text": 42}), event("transcript", {"text": "x" * 256}),
            b'{"type":"transcript","data_length":100}\n{}',
        ]
        for response in fixtures:
            with self.subTest(response=response[:50]):
                result, _ = self.exchange(response)
                self.assertEqual(result[0], "error")
                self.assertNotIn("Provider details", result[1])

    def test_timeout_and_disconnect(self):
        result, elapsed = self.exchange(event("transcript", {"text": "late"}), delay=0.4, timeout=100)
        self.assertEqual(result, ["error", "Wyoming STT timeout"])
        self.assertLess(elapsed, 0.8)
        result, _ = self.exchange(b"")
        self.assertEqual(result, ["error", "Wyoming STT response incomplete"])


if __name__ == "__main__":
    unittest.main()
