"""
Map Matching Evaluation and Verification Metrics.
SIH Problem Statement ID: 26168 | ISRO

Quantifies map-matching performance:
- Cross-track error (distance to road centerline)
- Along-track positional drift vs ground truth
- Road segment identification accuracy
"""

from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np

from src.map_matching.hmm_matcher import MapMatchedPoint


@dataclass
class MapMatchingMetrics:
    """Quantitative performance metrics for map matching."""
    mean_cross_track_error_m: float
    median_cross_track_error_m: float
    p95_cross_track_error_m: float
    max_cross_track_error_m: float
    matched_epochs: int
    total_epochs: int
    coverage_percentage: float
    ground_truth_rmse_m: Optional[float] = None
    drift_reduction_percentage: Optional[float] = None

    def print_summary(self, title: str = "Map Matching Performance"):
        print("\n" + "=" * 50)
        print(f"  {title}")
        print("=" * 50)
        print(f"Mean Cross-Track Error:     {self.mean_cross_track_error_m:.2f} m")
        print(f"Median Cross-Track Error:   {self.median_cross_track_error_m:.2f} m")
        print(f"95th Percentile Error:      {self.p95_cross_track_error_m:.2f} m")
        print(f"Max Cross-Track Error:      {self.max_cross_track_error_m:.2f} m")
        print(f"Road Coverage:              {self.coverage_percentage:.1f}% ({self.matched_epochs}/{self.total_epochs} epochs)")
        if self.ground_truth_rmse_m is not None:
            print(f"Ground Truth RMSE:          {self.ground_truth_rmse_m:.2f} m")
        if self.drift_reduction_percentage is not None:
            print(f"Drift Reduction:            {self.drift_reduction_percentage:.2f}%")
        print("=" * 50)


def evaluate_map_matching(
    matched_points: List[MapMatchedPoint],
    ground_truth_enu: Optional[np.ndarray] = None,
    unmatched_enu: Optional[np.ndarray] = None,
) -> MapMatchingMetrics:
    """
    Computes comprehensive evaluation metrics from matched points.
    
    Args:
        matched_points: List of MapMatchedPoint results from HMMMapMatcher.
        ground_truth_enu: Optional (N, 3) or (N, 2) ground truth positions.
        unmatched_enu: Optional (N, 3) or (N, 2) dead-reckoned positions prior to map matching.
        
    Returns:
        MapMatchingMetrics dataclass.
    """
    if not matched_points:
        return MapMatchingMetrics(0.0, 0.0, 0.0, 0.0, 0, 0, 0.0)

    ctes = np.array([p.cross_track_error_m for p in matched_points])
    matched_count = sum(1 for p in matched_points if p.matched_segment_id != "unmatched")
    total_count = len(matched_points)

    mean_cte = float(np.mean(ctes))
    med_cte = float(np.median(ctes))
    p95_cte = float(np.percentile(ctes, 95))
    max_cte = float(np.max(ctes))
    coverage = (matched_count / total_count) * 100.0 if total_count > 0 else 0.0

    gt_rmse = None
    drift_reduction = None

    if ground_truth_enu is not None:
        snapped_coords = np.array([p.matched_enu[:2] for p in matched_points])
        gt_coords = ground_truth_enu[:len(snapped_coords), :2]
        errors_snapped = np.linalg.norm(snapped_coords - gt_coords, axis=1)
        gt_rmse = float(np.sqrt(np.mean(errors_snapped ** 2)))

        if unmatched_enu is not None:
            raw_coords = unmatched_enu[:len(snapped_coords), :2]
            errors_raw = np.linalg.norm(raw_coords - gt_coords, axis=1)
            raw_rmse = float(np.sqrt(np.mean(errors_raw ** 2)))
            if raw_rmse > 1e-6:
                drift_reduction = float((raw_rmse - gt_rmse) / raw_rmse * 100.0)

    return MapMatchingMetrics(
        mean_cross_track_error_m=mean_cte,
        median_cross_track_error_m=med_cte,
        p95_cross_track_error_m=p95_cte,
        max_cross_track_error_m=max_cte,
        matched_epochs=matched_count,
        total_epochs=total_count,
        coverage_percentage=coverage,
        ground_truth_rmse_m=gt_rmse,
        drift_reduction_percentage=drift_reduction,
    )
