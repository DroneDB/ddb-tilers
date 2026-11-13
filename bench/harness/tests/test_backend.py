# SPDX-License-Identifier: MPL-2.0
import os
import tempfile
import unittest

from tests import _path  # noqa: F401
import run


class BackendSpec(unittest.TestCase):
    def test_lib_cli_and_env(self):
        with tempfile.NamedTemporaryFile("w", suffix=".env", delete=False) as f:
            f.write("export DDBT_BACKEND_TEST=1\n")
        try:
            b = run.parse_backend("A=legacy:/x/libddb.so:/x/ddb:" + f.name)
            self.assertEqual((b.name, b.kind, b.lib, b.cli, b.env_file), ("A", "legacy", "/x/libddb.so", "/x/ddb", f.name))
            self.assertEqual(b.describe()["env_file"], f.name)
        finally:
            os.unlink(f.name)

    def test_without_env(self):
        b = run.parse_backend("B=ddbt:/y/libddbt.so")
        self.assertEqual((b.lib, b.env_file), ("/y/libddbt.so", None))
        self.assertNotIn("env_file", b.describe())

    def test_missing_env_file_is_refused(self):
        with self.assertRaises(SystemExit):
            run.parse_backend("C=legacy:/x/libddb.so:/x/ddb:/nonexistent.env")


if __name__ == "__main__":
    unittest.main()
