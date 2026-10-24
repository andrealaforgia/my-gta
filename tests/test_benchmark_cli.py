"""Exercise benchmark command-line validation without opening a window."""

from pathlib import Path
import os
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "gta-like"


def run(*arguments):
    return subprocess.run(
        [str(BINARY), *arguments], cwd=ROOT, text=True, capture_output=True,
        check=False,
    )


class BenchmarkCommandLineTests(unittest.TestCase):
    def test_help_describes_the_fixed_simulated_workload(self):
        result = run("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("30 simulated seconds", result.stdout)
        self.assertIn("--bench-baseline", result.stdout)
        self.assertIn("--bench-output", result.stdout)

    def test_benchmark_requires_a_city_and_valid_workload(self):
        for arguments in (
            ("--bench",),
            ("--bench", "--scene=track"),
            ("--bench", "--city=fixture", "--traffic=-1"),
            ("--bench", "--city=fixture", "--time=nan"),
            ("--bench", "--city=fixture", "--traffic=twenty"),
            ("--bench", "--city=fixture", "--traffic=12cars"),
            ("--bench", "--city=fixture", "--time=10hours"),
            ("--bench", "--city=fixture", "--width=1280pixels"),
            ("--bench", "--city=fixture", "--bench-baseline="),
        ):
            with self.subTest(arguments=arguments):
                result = run(*arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("requires a city and valid workload", result.stderr)

    def test_report_file_options_require_benchmark_mode(self):
        result = run("--bench-output=unused.cfg")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("file options require --bench", result.stderr)

    def test_a_report_cannot_overwrite_its_reference_or_an_alias(self):
        with tempfile.TemporaryDirectory(prefix="gta-like-benchmark-cli-") as name:
            directory = Path(name)
            baseline = directory / "reference.cfg"
            baseline.write_text("preserve the reference\n")
            hard_link = directory / "hard-link.cfg"
            os.link(baseline, hard_link)
            symbolic_link = directory / "symbolic-link.cfg"
            symbolic_link.symlink_to(baseline)
            for output in (baseline, hard_link, symbolic_link):
                with self.subTest(output=output.name):
                    result = run(
                        "--bench", "--city=fixture",
                        "--bench-baseline=" + str(baseline),
                        "--bench-output=" + str(output),
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("must not overwrite its baseline", result.stderr)
                    self.assertEqual(baseline.read_text(), "preserve the reference\n")


if __name__ == "__main__":
    unittest.main()
