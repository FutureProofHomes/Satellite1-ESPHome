"""Compare the firmware scorer with the bridge algorithm, and exercise PCM/gating."""
import difflib
import pathlib
import random
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def normalize(value):
    return " ".join(re.findall(r"[a-z0-9]+", value.replace("_", " ").replace("-", " ").lower()))


def bridge_score(transcript, phrase):
    target = normalize(phrase)
    words = normalize(transcript).split()
    if not target or not words:
        return 0.0
    return max(
        difflib.SequenceMatcher(None, " ".join(words[start:start + size]), target).ratio()
        for size in range(1, len(target.split()) + 2)
        for start in range(len(words) - size + 1)
    )


class WakeVerifierTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.work = tempfile.TemporaryDirectory()
        cls.binary = pathlib.Path(cls.work.name) / "wake-check"
        subprocess.run([
            shutil.which("c++") or "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "esphome/components/stt_wake_verifier"),
            str(ROOT / "tests/wake_verifier_core.cpp"), "-o", str(cls.binary),
        ], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.work.cleanup()

    def test_gate_and_real_audio_queue(self):
        subprocess.run([str(self.binary)], check=True)

    def test_scoring_matches_bridge(self):
        fixtures = [
            ("Hey, Jarvis! Turn the lights on", "hey_jarvis"),
            ("okay naboo", "okay nabu"), ("hey tater", "hey_tater"),
            ("ordinary conversation", "hey jarvis"), ("", "hey jarvis"),
            ("Hey Jarvis", ""), ("hey jervis turn off office", "hey jarvis"),
            ("jarvis", "hey jarvis"), ("Okay Nabu what time is it", "okay nabu"),
            ("hi there hey potato", "hey tater"),
        ]
        rng = random.Random(312)
        vocabulary = ["hey", "jarvis", "jervis", "okay", "nabu", "naboo", "tater", "the", "turn", "off", "lights"]
        fixtures += [
            (" ".join(rng.choices(vocabulary, k=rng.randint(1, 12))),
             " ".join(rng.choices(vocabulary, k=rng.randint(1, 4))))
            for _ in range(200)
        ]
        for transcript, phrase in fixtures:
            with self.subTest(transcript=transcript, phrase=phrase):
                actual = float(subprocess.check_output([str(self.binary), transcript, phrase], text=True))
                self.assertAlmostEqual(actual, bridge_score(transcript, phrase), places=6)


if __name__ == "__main__":
    unittest.main()
