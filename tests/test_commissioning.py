"""Qualification must fail on missing measurements and unsafe envelopes."""
import copy
import json
from pathlib import Path
import unittest
from tools.commissioning import qualification_errors

ROOT = Path(__file__).resolve().parents[1]


def completed_record():
    record = json.loads((ROOT / 'commissioning/trial.example.json').read_text())
    record['hardware'].update(motor_power_cutoff_tested=True, computer_power_preserved=True, load_kg=27)
    record['release'].update(firmware_build_id='test', configuration_id='test', release_sha256='test')
    record['acceptance_targets'].update(max_stop_time_s=1, max_stop_distance_m=.5, max_hold_drift_m=.01,
                                        hold_duration_s=30, max_temperature_c=60)
    record['measurements'].update(stop_time_s=.3, stop_distance_m=.2, hold_drift_m=.005, hold_duration_s=35,
        peak_temperature_c=40, peak_motor_current_a=.8, worst_temperature_age_ms=500,
        worst_sweep_ms=15, worst_command_application_ms=30)
    record['checks'] = {name: True for name in record['checks']}
    record['unresolved_conditions'] = []
    return record


class CommissioningTests(unittest.TestCase):
    def test_unmeasured_template_never_qualifies(self):
        record = json.loads((ROOT / 'commissioning/trial.example.json').read_text())
        self.assertGreater(len(qualification_errors(record)), 20)

    def test_measured_targets_and_all_required_checks_must_pass(self):
        record = completed_record()
        self.assertEqual(qualification_errors(record), [])
        record['checks'].pop('independent_cutoff')
        self.assertTrue(any('independent_cutoff' in error for error in qualification_errors(record)))

    def test_stop_limit_failure_and_insufficient_hold_dwell_rejected(self):
        record = completed_record()
        record['measurements']['stop_distance_m'] = .6
        record['measurements']['hold_duration_s'] = 29
        errors = qualification_errors(record)
        self.assertTrue(any('stop_distance_m' in error for error in errors))
        self.assertTrue(any('hold_duration_s' in error for error in errors))

    def test_boolean_nan_and_overload_without_separate_acceptance_rejected(self):
        for value in (True, float('nan'), None):
            record = completed_record()
            record['measurements']['stop_time_s'] = value
            self.assertTrue(qualification_errors(record))
        record = completed_record()
        record['effective_current_ceiling_a'] = 2.5
        self.assertTrue(any('Separate measured' in error for error in qualification_errors(record)))
        record['higher_current_acceptance'] = True
        self.assertEqual(qualification_errors(record), [])


if __name__ == '__main__':
    unittest.main()
