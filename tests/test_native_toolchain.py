"""Native tests honor CXX and cannot silently skip when ROBOT_REQUIRE_NATIVE=1."""
import os
import unittest
from unittest.mock import patch

try:from native_toolchain import native_compiler
except ImportError:from tests.native_toolchain import native_compiler


def select(available, **env):
    """native_compiler() on a PATH holding only `available` compilers, with exactly `env` set."""
    with patch.dict(os.environ, env), patch('shutil.which', lambda name: f'/bin/{name}' if name in available else None):
        for name in {'CXX', 'ROBOT_REQUIRE_NATIVE'} - set(env):
            os.environ.pop(name, None)
        return native_compiler()


class NativeCompilerSelectionTests(unittest.TestCase):
    def test_cxx_selects_the_named_compiler(self):
        self.assertEqual(select({'g++', 'g++-16'}, CXX='g++-16'), '/bin/g++-16')

    def test_missing_cxx_is_an_error_even_when_other_compilers_exist(self):
        with self.assertRaisesRegex(RuntimeError, 'CXX'):
            select({'g++', 'clang++'}, CXX='g++-99')

    def test_default_order_is_gxx_then_clang(self):
        self.assertEqual(select({'g++', 'clang++'}), '/bin/g++')
        self.assertEqual(select({'clang++'}), '/bin/clang++')

    def test_no_compiler_skips_unless_native_is_required(self):
        with self.assertRaises(unittest.SkipTest):
            select(set())
        with self.assertRaisesRegex(RuntimeError, 'ROBOT_REQUIRE_NATIVE'):
            select(set(), ROBOT_REQUIRE_NATIVE='1')


if __name__ == '__main__':
    unittest.main()
