"""Exercise ARM/config interleavings through the real parser and firmware loop."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CASES = (
    "arm_config", "arm_config_wrap", "arm_stop", "arm_old_stop", "arm_session",
    "arm_io_stop", "arm_io_session", "arm_io_config",
)


class ArmConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise unittest.SkipTest("No native C++ compiler")
        cls.directory = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.directory.name) / "robot-arm-config-sil"
        subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
             "-Wno-misleading-indentation", "-I", str(ROOT / "tests/sil"),
             "-I", str(ROOT), str(ROOT / "tests/sil/arm_config.cpp"),
             "-o", str(cls.binary)],
            check=True, capture_output=True, text=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_arm_interleavings(self):
        for scenario in CASES:
            with self.subTest(scenario=scenario):
                result = subprocess.run(
                    [str(self.binary), scenario], capture_output=True,
                    text=True, timeout=10,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("PASS " + scenario, result.stdout)


if __name__ == "__main__":
    unittest.main()
