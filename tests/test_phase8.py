"""
Unit and Integration Test Suite for Phase 8: SIH Benchmark Deliverables.
SIH Problem Statement ID: 26168 | ISRO

Validates:
- All required figures exist and have non-zero file sizes
- JSON benchmark output exists and contains valid metrics
- All evaluated scenarios meet the SIH <10% drift requirement
- Sub-50ms outage detection latency is maintained
"""

import json
import unittest
from pathlib import Path


class TestPhase8Deliverables(unittest.TestCase):
    def setUp(self):
        self.repo_root = Path(__file__).resolve().parents[1]
        self.figures_dir = self.repo_root / "reports" / "figures"
        self.results_json = self.repo_root / "reports" / "sih_benchmark_results.json"

    def test_figures_exist_and_valid(self):
        expected_figures = [
            "sih_4way_trajectory_comparison.png",
            "sih_multi_scenario_drift_benchmark.png",
            "sih_blackout_reconvergence_profile.png",
        ]
        for fig_name in expected_figures:
            fig_path = self.figures_dir / fig_name
            self.assertTrue(fig_path.exists(), f"Missing figure: {fig_name}")
            self.assertGreater(fig_path.stat().st_size, 10000, f"Figure too small: {fig_name}")

    def test_benchmark_metrics_meet_sih_limits(self):
        self.assertTrue(self.results_json.exists(), "Missing sih_benchmark_results.json")
        with open(self.results_json, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.assertEqual(data.get("overall_status"), "PASSED_ALL_BENCHMARKS")
        scenarios = data.get("scenarios", [])
        self.assertGreaterEqual(len(scenarios), 3)

        for sc in scenarios:
            name = sc.get("scenario_name")
            drift_pct = sc.get("idr_drift_pct")
            self.assertLess(
                drift_pct, 10.0,
                f"Scenario '{name}' failed SIH drift requirement: {drift_pct}% >= 10.0%"
            )
            self.assertTrue(sc.get("sih_target_met"), f"Scenario '{name}' flagged as not met")
            self.assertLess(
                sc.get("detection_latency_ms", 999.0), 50.0,
                f"Scenario '{name}' detection latency exceeded 50 ms"
            )


if __name__ == "__main__":
    unittest.main()
