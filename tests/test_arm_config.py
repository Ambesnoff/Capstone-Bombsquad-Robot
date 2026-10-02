"""Exercise ARM/config interleavings through the real parser and firmware loop."""
from pathlib import Path
import subprocess
import unittest

try:
    from native_toolchain import STRICT, build_native
except ImportError:
    from tests.native_toolchain import STRICT, build_native

ROOT = Path(__file__).resolve().parents[1]
CASES = (
    "arm_config", "arm_config_wrap", "arm_stop", "arm_old_stop", "arm_session",
    "arm_io_stop", "arm_io_session", "arm_io_config",
)


class ArmConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = build_native(cls, ROOT / "tests/sil/arm_config.cpp", "robot-arm-config-sil",
                                  *STRICT, "-I", str(ROOT / "tests/sil"))

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
