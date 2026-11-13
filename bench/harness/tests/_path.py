# SPDX-License-Identifier: MPL-2.0
"""Puts bench/harness on sys.path for the tests."""
import os
import sys

HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if HARNESS not in sys.path:
    sys.path.insert(0, HARNESS)
