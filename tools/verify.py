"""Full verification gate: generated-file checks, example configs and the whole test suite.

The unittest suite runs once per available C++ compiler (clang++ and GNU g++-N) with
ROBOT_REQUIRE_NATIVE=1 and CXX set, so a missing compiler cannot skip the native firmware tests.
Any failure, error, skipped or expected-failure test fails the gate.
Full verification requires Python 3.14.8; --allow-python-mismatch is diagnostic only.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
REQUIRED_PYTHON = (3, 14, 8)
STEP_TIMEOUT_S = 900
HEAD_LINES, TAIL_LINES = 120, 40
SUMMARY = re.compile(r'^Ran (\d+) tests? in [\d.]+s\n\n(OK|FAILED|NO TESTS RAN)([^\n]*)$', re.MULTILINE)
SKIPPED = re.compile(r'^.* \.\.\. skipped .*$', re.MULTILINE)
RULE = '\n' + '=' * 70  # unittest prints this before each failure or error report
COUNTS = ('failures', 'errors', 'skipped', 'expected failures', 'unexpected successes')


@dataclass
class Result:
    step: str
    compiler: str = '-'
    counts: dict | None = None
    seconds: float = 0.0
    problems: list = field(default_factory=list)
    output: str = ''

    @property
    def ok(self) -> bool:
        return not self.problems


def dotted(version: tuple) -> str:
    return '.'.join(map(str, version))


def parse_unittest(output: str) -> dict | None:
    """Counts from the last unittest summary in output, or None when there is none."""
    found = SUMMARY.findall(output)
    if not found:
        return None
    run, verdict, details = found[-1]
    given = {key.strip(): int(value) for key, value in re.findall(r'([a-z ]+)=(\d+)', details)}
    return {'run': int(run), 'verdict': verdict, **{name: given.get(name, 0) for name in COUNTS}}


def unittest_problems(code: int | None, output: str) -> tuple:
    """(counts, problems): failures, errors, skips and a missing or empty summary all fail the gate."""
    counts, problems = parse_unittest(output), []
    if code is None:
        problems.append(f'timed out after {STEP_TIMEOUT_S}s')
    elif code:
        problems.append(f'exit status {code}')
    if counts is None:
        problems.append('no unittest summary found')
        return counts, problems
    if not counts['run']:
        problems.append('no tests ran')
    if counts['verdict'] != 'OK':
        problems.append(f'unittest reported {counts["verdict"]}')
    problems.extend(f'{counts[name]} {name}' + (' (a skipped test fails verification)' if name == 'skipped' else '')
                    for name in COUNTS if counts[name])
    problems.extend(SKIPPED.findall(output)[:10])  # `-v` names each skipped test and its reason
    return counts, problems


def is_gnu_gcc(version_text: str) -> bool:
    text = version_text.lower()
    return ('free software foundation' in text or 'gcc' in text) and 'clang' not in text


def version_text(path: str) -> str:
    """Output of `path --version`, or '' when the compiler cannot be run."""
    try:
        return subprocess.run([path, '--version'], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ''


def gnu_gcc() -> tuple | None:
    """(name, path) of the highest g++-N on PATH, else g++ if it really is GNU GCC (Apple's g++ is clang)."""
    pattern, numbers = re.compile(r'g\+\+-(\d+)'), set()
    for directory in os.environ.get('PATH', '').split(os.pathsep):
        try:
            names = os.listdir(directory or '.')
        except OSError:
            continue
        numbers.update(int(match[1]) for match in map(pattern.fullmatch, names) if match)
    for name in [f'g++-{number}' for number in sorted(numbers, reverse=True)] + ['g++']:
        path = shutil.which(name)
        if path and is_gnu_gcc(version_text(path)):
            return name, path
    return None


def resolve_compilers(spec: str | None) -> list:
    """Compilers to test as (name, path): --compilers if given, else clang++ and GNU GCC when present."""
    if spec is not None:
        names = [name.strip() for name in spec.split(',') if name.strip()]
        missing = [name for name in names if not shutil.which(name)]
        if not names or missing:
            raise SystemExit('--compilers: ' + ('not found on PATH: ' + ', '.join(missing) if missing else 'no compiler named'))
        return [(name, shutil.which(name)) for name in names]
    clang, gcc = shutil.which('clang++'), gnu_gcc()
    if not clang:
        print('note: clang++ not found on PATH; it will not be exercised')
    if not gcc:
        print('note: no GNU GCC (g++-N, or a g++ that is not clang) found on PATH; it will not be exercised')
    return ([('clang++', clang)] if clang else []) + ([gcc] if gcc else [])


def run(label: str, argv: list, compiler: str = '-', env: dict | None = None, tests: bool = False) -> Result:
    print(label + (f' [{compiler}]' if compiler != '-' else '') + ' ...', end=' ', flush=True)
    start = time.monotonic()
    try:
        done = subprocess.run(argv, cwd=ROOT, env=env, capture_output=True, text=True,
                              stdin=subprocess.DEVNULL, timeout=STEP_TIMEOUT_S)
        code, output = done.returncode, done.stdout + '\n' + done.stderr
    except subprocess.TimeoutExpired as error:
        code, output = None, ''.join(part.decode(errors='replace') if isinstance(part, bytes) else part or ''
                                     for part in (error.stdout, error.stderr))
    result = Result(label, compiler, seconds=time.monotonic() - start, output=output)
    if tests:
        result.counts, result.problems = unittest_problems(code, output)
        if result.counts and RULE not in output:
            result.output = ''  # nothing failed or errored, so there is no report to show
    elif code is None:
        result.problems.append(f'timed out after {STEP_TIMEOUT_S}s')
    elif code:
        result.problems.append(f'exit status {code}')
    print(('PASS' if result.ok else 'FAIL') + f' ({result.seconds:.1f}s)', flush=True)
    return result


def table(results: list) -> str:
    rows = [('step', 'compiler', 'tests', 'failures', 'errors', 'skips', 'seconds', 'result')]
    for r in results:
        counts = [str(r.counts[key]) if r.counts else '-' for key in ('run', 'failures', 'errors', 'skipped')]
        rows.append((r.step, r.compiler, *counts, f'{r.seconds:.1f}', 'PASS' if r.ok else 'FAIL'))
    widths = [max(len(row[column]) for row in rows) for column in range(len(rows[0]))]
    lines = ['  '.join(cell.ljust(width) if column in (0, 1, 7) else cell.rjust(width)
                       for column, (cell, width) in enumerate(zip(row, widths))).rstrip() for row in rows]
    lines.insert(1, '  '.join('-' * width for width in widths))
    return '\n'.join(lines)


def excerpt(output: str) -> str:
    """A failed step's output: a unittest run from its first failure report on, long output cut in the middle."""
    lines = output[max(output.find(RULE) + 1, 0):].rstrip().splitlines()
    if len(lines) > HEAD_LINES + TAIL_LINES:
        lines[HEAD_LINES:-TAIL_LINES] = [f'... {len(lines) - HEAD_LINES - TAIL_LINES} lines omitted ...']
    return '\n'.join(lines)


def banner(*lines: str) -> None:
    print('\n'.join(['=' * 78, *(' ' + line for line in lines), '=' * 78]))


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--allow-python-mismatch', action='store_true',
                        help=f'run on a Python other than {dotted(REQUIRED_PYTHON)} anyway; diagnostic only, NOT a full verification')
    parser.add_argument('--compilers', help='comma-separated C++ compilers to test instead of the detected clang++ and GNU g++-N')
    args = parser.parse_args(argv)
    actual, full = dotted(sys.version_info[:3]), sys.version_info[:3] == REQUIRED_PYTHON
    if not full and not args.allow_python_mismatch:
        raise SystemExit(f'Full verification requires Python {dotted(REQUIRED_PYTHON)}, but this is Python {actual} '
                         f'({sys.executable}). Run it with Python {dotted(REQUIRED_PYTHON)} (create the venv with python3.14), '
                         'or pass --allow-python-mismatch for a diagnostic-only run.')
    started = time.monotonic()
    notice = (f'NOT A FULL VERIFICATION: running on Python {actual}, but full verification requires {dotted(REQUIRED_PYTHON)}.',
              'Results are diagnostic only and must not be recorded as a verified release.')
    if not full:
        banner(*notice)
    compilers = resolve_compilers(args.compilers)
    print(f'Python {actual} ({sys.executable})')
    for name, path in compilers:
        print(f'  {name}: {path} -- {(version_text(path).strip().splitlines() or ["version unavailable"])[0]}')
    python = sys.executable
    env = {key: value for key, value in os.environ.items() if key != 'FORCE_COLOR'}
    env.update(PYTHON_COLORS='0', NO_COLOR='1')  # Python 3.14 colors the unittest summary under FORCE_COLOR
    results = [
        run('protocol generation check', [python, 'generate_protocol.py', '--check'], env=env),
        run('firmware build identity check', [python, 'hat_firmware/generate_build_identity.py', '--check'], env=env),
        run('config.example.json check', [python, 'robot_main.py', 'check', '--config', 'config.example.json'], env=env),
        run('config.legacy.example.json check', [python, 'robot_main.py', 'check', '--config', 'config.legacy.example.json'], env=env)]
    for name, path in compilers:
        results.append(run('unittest discover', [python, '-m', 'unittest', 'discover', '-s', 'tests', '-v'], name,
                           {**env, 'CXX': path, 'ROBOT_REQUIRE_NATIVE': '1'}, tests=True))
    if not compilers:
        results.append(Result('unittest discover', '(none)',
                              problems=['no C++ compiler found; the native firmware tests may not be skipped']))
    for result in results:
        if not result.ok:
            print(f'\n--- FAILED: {result.step} [{result.compiler}]', *(f'  - {problem}' for problem in result.problems), sep='\n')
            if result.output:
                print(excerpt(result.output))
    print('\n' + table(results) + '\n')
    failed = [result for result in results if not result.ok]
    if not full:
        banner(*notice)
    elapsed = f'{time.monotonic() - started:.1f}s'
    if failed:
        print(f'RESULT: FAIL ({len(failed)} of {len(results)} steps failed, {elapsed})')
    elif full:
        print(f'RESULT: PASS (full verification on Python {actual}, {elapsed})')
    else:
        print(f'RESULT: DIAGNOSTIC PASS only; this is NOT a full verification (Python {actual}, {elapsed})')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
