"""C++ compiler selection and build for the native firmware and protocol tests.

CXX names the compiler explicitly and must exist. Without it g++, clang++ or c++ is used.
With no compiler the native tests skip, unless ROBOT_REQUIRE_NATIVE=1 makes that an error.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STRICT = ('-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation')


def native_compiler():
    explicit = os.environ.get('CXX')
    compiler = shutil.which(explicit) if explicit else (
        shutil.which('g++') or shutil.which('clang++') or shutil.which('c++'))
    if explicit and not compiler:
        raise RuntimeError(f'CXX={explicit!r} is not an executable compiler on PATH')
    if not compiler:
        if os.environ.get('ROBOT_REQUIRE_NATIVE') == '1':
            raise RuntimeError('ROBOT_REQUIRE_NATIVE=1 but no native C++ compiler (g++, clang++ or CXX) was found')
        raise unittest.SkipTest('No native C++ compiler')
    return compiler


def build_native(case, source, name, *flags):
    """Compile source for one test class and return the binary; the build directory goes with the class."""
    compiler = native_compiler()
    directory = tempfile.TemporaryDirectory()
    case.addClassCleanup(directory.cleanup)
    binary = Path(directory.name) / name
    result = subprocess.run([compiler, '-std=c++17', *flags, '-I', str(ROOT), str(source), '-o', str(binary)],
                            capture_output=True, text=True)
    if result.returncode:
        raise AssertionError(f'{compiler} failed to build {Path(source).name}:\n{result.stderr}')
    return binary
