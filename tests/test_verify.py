"""The verification gate pins Python 3.14.8, fails on failures, errors and skips, and finds real GNU GCC."""
import contextlib
import io
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from tools import verify

DEBIAN_GCC = 'g++ (Debian 12.2.0-14) 12.2.0 Copyright (C) 2022 Free Software Foundation, Inc.'
HOMEBREW_GCC = 'g++-16 (Homebrew GCC 16.2.0) 16.2.0'
APPLE_CLANG = 'Apple clang version 21.0.0 (clang-2100.1.1.101)'
STEPS = ('protocol generation check', 'firmware build identity check',
         'config.example.json check', 'config.legacy.example.json check')


def summary(run, verdict='OK'):
    return f'....\n{"-" * 70}\nRan {run} tests in 1.000s\n\n{verdict}\n'


class UnittestSummaryTests(unittest.TestCase):
    def test_clean_run_passes(self):
        counts, problems = verify.unittest_problems(0, summary(93))
        self.assertEqual((counts['run'], problems), (93, []))

    def test_skips_fail_although_unittest_exits_zero(self):
        output = "test_x (m.C.test_x) ... skipped 'no compiler'\n" + summary(5, 'OK (skipped=2)')
        counts, problems = verify.unittest_problems(0, output)
        self.assertEqual(counts['skipped'], 2)
        self.assertIn('2 skipped (a skipped test fails verification)', problems)
        self.assertIn("test_x (m.C.test_x) ... skipped 'no compiler'", problems)

    def test_each_count_is_parsed_separately(self):
        counts = verify.parse_unittest(summary(
            9, 'FAILED (failures=1, errors=2, skipped=3, expected failures=4, unexpected successes=5)'))
        self.assertEqual([counts[name] for name in verify.COUNTS], [1, 2, 3, 4, 5])

    def test_bad_exit_empty_timeout_and_missing_summary_fail(self):
        self.assertIn('exit status 1', verify.unittest_problems(1, summary(3))[1])
        self.assertIn('no tests ran', verify.unittest_problems(0, summary(0, 'NO TESTS RAN'))[1])
        self.assertIn('no unittest summary found', verify.unittest_problems(0, 'Traceback (most recent call last):')[1])
        self.assertIn(f'timed out after {verify.STEP_TIMEOUT_S}s', verify.unittest_problems(None, '')[1])

    def test_colored_summary_is_not_trusted(self):
        self.assertIsNone(verify.parse_unittest('Ran 4 tests in 0.001s\n\n\x1b[32mOK\x1b[0m\n'))


class CompilerDetectionTests(unittest.TestCase):
    def detect(self, compilers):
        """gnu_gcc() on a PATH holding only fake compilers that print the given version banners."""
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {'PATH': directory}):
            for name, banner in compilers.items():
                path = Path(directory, name)
                path.write_text(f"#!/bin/sh\necho '{banner}'\n")
                path.chmod(0o755)
            found = verify.gnu_gcc()
            return found and found[0]

    def test_gnu_gcc_is_recognized_and_clang_is_not(self):
        self.assertTrue(verify.is_gnu_gcc(DEBIAN_GCC))
        self.assertTrue(verify.is_gnu_gcc(HOMEBREW_GCC))
        self.assertFalse(verify.is_gnu_gcc(APPLE_CLANG))
        self.assertFalse(verify.is_gnu_gcc('Debian clang version 14.0.6'))

    def test_highest_numbered_gnu_gxx_wins_numerically(self):
        self.assertEqual(self.detect({'g++-9': DEBIAN_GCC, 'g++-16': HOMEBREW_GCC, 'g++-12': DEBIAN_GCC,
                                      'g++': APPLE_CLANG}), 'g++-16')

    def test_versioned_gxx_that_is_clang_is_skipped(self):
        self.assertEqual(self.detect({'g++-16': APPLE_CLANG, 'g++-12': DEBIAN_GCC}), 'g++-12')

    def test_plain_gxx_counts_only_when_it_is_gnu(self):
        self.assertEqual(self.detect({'g++': DEBIAN_GCC}), 'g++')
        self.assertIsNone(self.detect({'g++': APPLE_CLANG}))
        self.assertIsNone(self.detect({}))


def gate(version, *args, failing=(), compilers=(('clang++', '/x/clang++'), ('g++-16', '/x/g++-16'))):
    """main() on a pretended Python with every step stubbed: (exit code, printed output, step calls)."""
    calls = []

    def fake_run(label, argv, compiler='-', env=None, tests=False):
        calls.append((label, compiler, env, tests))
        return verify.Result(label, compiler, problems=['boom'] if label in failing else [])

    pretend = SimpleNamespace(version_info=(*version, 'final', 0), executable='/py')
    output = io.StringIO()
    with patch.object(verify, 'sys', pretend), patch.object(verify, 'run', fake_run), \
            patch.object(verify, 'resolve_compilers', return_value=list(compilers)), \
            patch.object(verify, 'version_text', return_value='cc 1.0'), contextlib.redirect_stdout(output):
        code = verify.main(list(args))
    return code, output.getvalue(), calls


class GateTests(unittest.TestCase):
    def test_python_3_14_8_is_required(self):
        self.assertEqual(verify.REQUIRED_PYTHON, (3, 14, 8))
        with self.assertRaisesRegex(SystemExit, r'requires Python 3\.14\.8.*this is Python 3\.14\.7'):
            gate((3, 14, 7))

    def test_mismatch_is_diagnostic_only_but_still_runs_everything(self):
        code, output, calls = gate((3, 14, 7), '--allow-python-mismatch')
        self.assertEqual(code, 0)
        self.assertEqual(len(calls), len(STEPS) + 2)
        self.assertIn('NOT A FULL VERIFICATION: running on Python 3.14.7', output)
        self.assertIn('DIAGNOSTIC PASS only', output)

    def test_exact_python_is_a_full_verification(self):
        code, output, _ = gate((3, 14, 8))
        self.assertEqual(code, 0)
        self.assertIn('RESULT: PASS (full verification', output)
        self.assertNotIn('NOT A FULL VERIFICATION', output)

    def test_any_failed_step_exits_nonzero(self):
        for failing in (STEPS[0], 'unittest discover'):
            code, output, _ = gate((3, 14, 8), failing=(failing,))
            self.assertEqual(code, 1, failing)
            self.assertIn('RESULT: FAIL', output)

    def test_unittest_runs_once_per_compiler_with_native_tests_required(self):
        _, _, calls = gate((3, 14, 8))
        suites = [(compiler, env['CXX'], env['ROBOT_REQUIRE_NATIVE']) for _, compiler, env, tests in calls if tests]
        self.assertEqual(suites, [('clang++', '/x/clang++', '1'), ('g++-16', '/x/g++-16', '1')])

    def test_no_compiler_fails_instead_of_skipping_native_tests(self):
        code, output, calls = gate((3, 14, 8), compilers=())
        self.assertEqual(code, 1)
        self.assertFalse([call for call in calls if call[3]])
        self.assertIn('no C++ compiler found', output)


if __name__ == '__main__':
    unittest.main()
