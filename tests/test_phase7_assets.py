"""
Test suite for Phase 7 offline navigation assets and deterministic replay packages.
Validates MOBILE-01, MOBILE-02, and MOBILE-03 packaging invariants.
"""

import unittest
from pathlib import Path
from tools.phase7.validate_assets import validate_all_assets


class TestPhase7Assets(unittest.TestCase):
    def test_asset_validation(self):
        repo_root = Path(__file__).resolve().parents[1]
        valid, errors, warnings = validate_all_assets(repo_root)
        self.assertTrue(valid, f"Asset validation failed with errors: {errors}")
        self.assertEqual(len(errors), 0)


if __name__ == "__main__":
    unittest.main()

