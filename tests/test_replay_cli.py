"""Session command-line validation, exercised before opening a window."""

from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def run(*arguments):
    return subprocess.run([str(ROOT / "gta-like"), *arguments], cwd=ROOT,
                          text=True, capture_output=True, check=False, timeout=10)


class ReplayCommandLineTests(unittest.TestCase):
    def test_help_explains_recording_and_seed(self):
        result = run("--help")
        self.assertEqual(result.returncode, 0)
        for option in ("--record=", "--replay=", "--record-frames=", "--seed="):
            self.assertIn(option, result.stdout)

    def test_invalid_session_options_fail_before_opening_a_window(self):
        cases = (
            (("--record=a", "--replay=b"), "record and replay"),
            (("--bench", "--record=a"), "benchmark"),
            (("--seed=-1",), "seed"),
            (("--seed=4294967296",), "seed"),
            (("--seed=wrong",), "seed"),
            (("--record-frames=4",), "record-frames"),
            (("--record=a", "--record-frames=0"), "record-frames"),
            (("--record=",), "path"),
            (("--replay=",), "path"),
            (("--record=a", "--time=25"), "workload"),
            (("--record=a", "--hour-seconds=-1"), "workload"),
            (("--record=a", "--traffic=-1"), "workload"),
        )
        with tempfile.TemporaryDirectory(prefix="gta-like-replay-invalid-") as name:
            for index, (arguments, message) in enumerate(cases):
                arguments = tuple("--record=" + str(Path(name) / str(index))
                                  if argument == "--record=a" else argument
                                  for argument in arguments)
                if any(a.startswith("--record=") for a in arguments) and not any(
                        a.startswith("--record-frames=") for a in arguments):
                    arguments += ("--record-frames=1",)
                with self.subTest(arguments=arguments):
                    result = run(*arguments)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(message, result.stderr)
                    self.assertNotIn("window", result.stderr)

    def test_recording_preserves_an_existing_file(self):
        with tempfile.TemporaryDirectory(prefix="gta-like-replay-cli-") as name:
            path = Path(name) / "record.log"
            path.write_text("preserve this log\n")
            result = run("--record=" + str(path), "--record-frames=3")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("already exists", result.stderr)
            self.assertEqual(path.read_text(), "preserve this log\n")

    def test_replay_restores_header_settings_and_rejects_conflicting_overrides(self):
        with tempfile.TemporaryDirectory(prefix="gta-like-replay-cli-") as name:
            path = Path(name) / "session.log"
            path.write_text("gta-replay 1 7 0 0x1.4p+3 0x1.ep+4 0x0p+0 "
                            "0 0 0 0 0\nscene test track\nvehicle hatchback\n")
            for option in ("--seed=8", "--time=11", "--hour-seconds=20",
                           "--rain", "--traffic=1", "--pedestrians=1"):
                with self.subTest(option=option):
                    result = run("--replay=" + str(path), option)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("conflicts with replay", result.stderr)
                    self.assertNotIn("window", result.stderr)
            path.write_text(path.read_text().replace("1 7 0 ", "1 7 1 "))
            result = run("--replay=" + str(path))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("requires a matching city", result.stderr)


if __name__ == "__main__":
    unittest.main()
