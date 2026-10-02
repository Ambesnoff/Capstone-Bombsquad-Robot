"""Execute actual firmware with independently advancing deterministic physics.

The native harness asserts safety/state outcomes and exits nonzero on failure.
It is software evidence; its mechanics and heating are not hardware calibration.
"""
from pathlib import Path
import subprocess
import unittest
try:from native_toolchain import STRICT,build_native
except ImportError:from tests.native_toolchain import STRICT,build_native
ROOT=Path(__file__).resolve().parents[1]
CASES=("ramp","physics","boot_modes","arm_mode_failure","faulted_stop_polling",
       "combined_motor_fault","stop_motor_fault","command_reply_motor_fault",
       "old_stop","repeat_stop_deadline","session_replay","applied_complete","profiles_ceiling","full_profiles","boost_budget",
       "stop_boost_refill","boost_stale","hot_driving_neutral","gentle_overcurrent","hold_neutral_overcurrent","thermal","hold_wrap","encoder_long_run","hold_independent","inner_zero",
       "disarmed_hold","stall","reversal","abnormal_current","command_expiry",
       "crc_recovery","config_motion_reject","parser_noise","bounded_io","bounded_drain","progress_watchdog","rollover")
class HatFirmwareNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary=build_native(cls,ROOT/'tests/sil/main.cpp','robot-sil',*STRICT,'-I',str(ROOT/'tests/sil'))
    def test_deterministic_fault_scenarios(self):
        for scenario in CASES:
            with self.subTest(scenario=scenario):
                result=subprocess.run([str(self.binary),scenario],capture_output=True,text=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('PASS '+scenario,result.stdout)
if __name__=='__main__':unittest.main()
