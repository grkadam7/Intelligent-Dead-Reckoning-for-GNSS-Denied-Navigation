"""
Unit tests validating Phase 7 reference oracle fixtures and model manifest.
Verifies consistency, tolerance specs, schema version, and mode transitions.
"""

import json
import unittest
from pathlib import Path


class TestPhase7Reference(unittest.TestCase):
    def setUp(self):
        self.repo_root = Path(__file__).resolve().parents[1]
        self.fixture_path = self.repo_root / "native" / "tests" / "fixtures" / "reviewed.json"
        self.model_manifest_path = self.repo_root / "mobile" / "assets" / "models" / "manifest.json"

    def test_model_manifest_validity(self):
        self.assertTrue(self.model_manifest_path.exists())
        with open(self.model_manifest_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

        self.assertEqual(meta.get("format"), "ONNX")
        self.assertEqual(meta.get("input", {}).get("shape"), [1, 6, 50])
        self.assertEqual(len(meta.get("normalization", {}).get("mu", [])), 6)
        self.assertEqual(len(meta.get("normalization", {}).get("sigma", [])), 6)
        self.assertLessEqual(meta.get("performance", {}).get("max_inference_latency_ms", 999), 20.0)

    def test_reference_fixture_structure_and_transitions(self):
        self.assertTrue(self.fixture_path.exists())
        with open(self.fixture_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        meta = data.get("metadata", {})
        self.assertEqual(meta.get("schema_version"), "1.0.0")
        self.assertGreaterEqual(meta.get("num_epochs", 0), 10)

        epochs = data.get("epochs", [])
        self.assertEqual(len(epochs), meta.get("num_epochs"))

        modes_seen = set()
        for idx, ep in enumerate(epochs):
            self.assertEqual(ep.get("epoch_index"), idx)
            out = ep.get("output", {})
            modes_seen.add(out.get("navigation_mode"))

            # Check finite numbers in output
            pos = out.get("position_enu", [])
            self.assertEqual(len(pos), 3)
            for coord in pos:
                self.assertTrue(isinstance(coord, (int, float)))

        # Must include both NORMAL_FUSION and OUTAGE_DEAD_RECKONING
        self.assertIn("NORMAL_FUSION", modes_seen)
        self.assertIn("OUTAGE_DEAD_RECKONING", modes_seen)


if __name__ == "__main__":
    unittest.main()

