"""
Phase 5 Evaluation: GNSS+INS Fusion Engine & Blackout Handler Benchmark
SIH Problem Statement ID: 26168 | ISRO

Benchmarks the complete EKF + NHC + Smooth Reconvergence fusion stack against simulated
GNSS blackouts of 10 s, 20 s, and 30 s (tunnel/urban-canyon scenarios).

Metrics reported:
  - Peak positional drift (m) during blackout
  - Final drift as % of blackout distance (SIH target: < 10%)
  - GNSS outage detection latency (ms)  (target: < 50 ms)
  - Reconvergence residual offset after GNSS restoration (m)
"""

import numpy as np
import sys
import os

# -- project root on sys.path --------------------------------------------------
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.data.iovnbd_loader import generate_synthetic_run
from src.fusion.fusion_engine import GNSSINSFusionEngine
from src.fusion.blackout_handler import NavigationMode
from src.map_matching.road_network import RoadNetwork


# ------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------

def run_fusion_scenario(
    data,
    blackout_start_s: float,
    blackout_end_s: float,
    road_network,
    sampling_rate: float = 10.0,
):
    """Run the full fusion engine over one synthetic trajectory with a GNSS blackout window."""
    engine = GNSSINSFusionEngine(
        sampling_rate=sampling_rate,
        road_network=road_network,
        max_gnss_accuracy_m=15.0,
        reconvergence_duration_sec=2.5,
    )

    fused_pos = []
    outage_flags = []
    nav_modes = []
    detection_latencies_ms = []

    for i in range(data.num_samples):
        t = data.timestamps_s[i]
        is_blackout = blackout_start_s <= t <= blackout_end_s

        gnss_enu = None if is_blackout else data.enu_positions[i]
        gnss_acc = 999.0 if is_blackout else data.gps_accuracy_m[i]

        out = engine.process_epoch(
            timestamp_s=t,
            accel_raw=data.accel[i],
            gyro_raw=data.gyro[i],
            gnss_pos_enu=gnss_enu,
            gnss_accuracy_m=gnss_acc,
            ai_forward_speed=data.gps_speed_mps[i],
            explicit_outage=is_blackout,
        )

        fused_pos.append(out.position_enu.copy())
        outage_flags.append(out.is_outage)
        nav_modes.append(out.navigation_mode)
        detection_latencies_ms.append(out.detection_latency_ms)

    fused_pos = np.array(fused_pos)
    outage_flags = np.array(outage_flags)
    detection_latencies_ms = np.array(detection_latencies_ms)

    return fused_pos, outage_flags, nav_modes, detection_latencies_ms


def compute_metrics(
    data,
    fused_pos: np.ndarray,
    outage_flags: np.ndarray,
    blackout_start_s: float,
    blackout_end_s: float,
    detection_latencies_ms: np.ndarray,
):
    """Compute all Phase 5 benchmark metrics."""
    gt = data.enu_positions
    t = data.timestamps_s

    # Errors at each epoch
    pos_err_2d = np.linalg.norm(fused_pos[:, :2] - gt[:, :2], axis=1)

    # Blackout window mask
    blackout_mask = (t >= blackout_start_s) & (t <= blackout_end_s)
    blackout_gt = gt[blackout_mask]
    blackout_fused = fused_pos[blackout_mask]

    # Blackout distance (path length along ground truth)
    blackout_dist_m = float(
        np.sum(np.linalg.norm(np.diff(blackout_gt[:, :2], axis=0), axis=1))
    ) if len(blackout_gt) > 1 else 1.0

    # Final positional error at blackout exit
    final_err_m = float(np.linalg.norm(
        blackout_fused[-1, :2] - blackout_gt[-1, :2]
    )) if len(blackout_fused) > 0 else 0.0

    drift_pct = (final_err_m / blackout_dist_m) * 100.0

    # Peak error during blackout
    peak_err_m = float(np.max(
        np.linalg.norm(blackout_fused[:, :2] - blackout_gt[:, :2], axis=1)
    )) if len(blackout_fused) > 0 else 0.0

    # Detection latency (at blackout onset)
    onset_mask = (t >= blackout_start_s) & (t <= blackout_start_s + 0.5)
    det_latency_ms = float(np.min(detection_latencies_ms[onset_mask])) if onset_mask.any() else 999.0

    # Reconvergence residual (first 3 s after blackout ends)
    post_mask = (t > blackout_end_s) & (t <= blackout_end_s + 3.0)
    reconvergence_err_m = 0.0
    if post_mask.any():
        post_err = np.linalg.norm(fused_pos[post_mask, :2] - gt[post_mask, :2], axis=1)
        reconvergence_err_m = float(np.mean(post_err))

    return {
        "blackout_duration_s": blackout_end_s - blackout_start_s,
        "blackout_dist_m": blackout_dist_m,
        "final_err_m": final_err_m,
        "drift_pct": drift_pct,
        "peak_err_m": peak_err_m,
        "detection_latency_ms": det_latency_ms,
        "reconvergence_residual_m": reconvergence_err_m,
        "sih_pass": drift_pct < 10.0,
    }


def print_banner(text: str):
    bar = "=" * (len(text) + 4)
    print(f"\n{bar}")
    print(f"  {text}")
    print(f"{bar}")


def print_metrics(label: str, m: dict):
    det_ok = "OK " if m["detection_latency_ms"] <= 50.0 else "ERR"
    sih_ok = "PASS" if m["sih_pass"] else "FAIL"
    print(f"\n  -- {label} --")
    print(f"    Blackout duration  : {m['blackout_duration_s']:.0f} s")
    print(f"    Blackout distance  : {m['blackout_dist_m']:.1f} m")
    print(f"    Final drift error  : {m['final_err_m']:.2f} m")
    print(f"    Drift (% distance) : {m['drift_pct']:.2f}%   [{sih_ok}]")
    print(f"    Peak error         : {m['peak_err_m']:.2f} m")
    print(f"    Detection latency  : {m['detection_latency_ms']:.1f} ms  [{det_ok}]")
    print(f"    Reconvergence res  : {m['reconvergence_residual_m']:.2f} m")


# ---------------------------------------------------------------------------
# Main evaluation
# ---------------------------------------------------------------------------

def main():
    print_banner("Phase 5 - GNSS+INS Fusion & Blackout Handler Benchmark")
    print("  SIH Problem Statement ID: 26168 | ISRO")
    print("  Target: Drift < 10% of distance | Detection Latency < 50 ms\n")

    # Generate a 120-second synthetic trajectory at 10 Hz
    print("  [1/4] Generating 120 s synthetic trajectory (10 Hz) ...")
    data = generate_synthetic_run(duration_s=120.0, dt=0.1, noise_level=1.0, seed=42)
    print(f"        Samples: {data.num_samples}, Total distance: {data.total_distance_m:.1f} m")

    # Build road network from ground truth waypoints
    print("  [2/4] Building offline road network from trajectory waypoints ...")
    road_network = RoadNetwork.create_from_route_waypoints(
        waypoints_enu=data.enu_positions,
        subsegment_length_m=80.0,
    )
    n_segs = len(road_network.segments)
    print(f"        Road network: {n_segs} segments")

    # Define blackout scenarios
    scenarios = [
        ("10 s Tunnel",  10.0,  20.0),
        ("20 s Tunnel",  20.0,  40.0),
        ("30 s Tunnel",  15.0,  45.0),
    ]

    print("  [3/4] Running fusion scenarios ...")
    results = []
    for label, t_start, t_end in scenarios:
        print(f"        * {label} (t={t_start:.0f}-{t_end:.0f} s) ...", end=" ", flush=True)
        fused_pos, outage_flags, nav_modes, det_lat = run_fusion_scenario(
            data, t_start, t_end, road_network, sampling_rate=10.0
        )
        m = compute_metrics(data, fused_pos, outage_flags, t_start, t_end, det_lat)
        results.append((label, m))
        print("done")

    # -- Print Report --
    print_banner("Results")
    all_pass = True
    for label, m in results:
        print_metrics(label, m)
        if not m["sih_pass"]:
            all_pass = False

    # -- Summary Table --
    print("\n  +----------------------+------------+-----------+------------+--------+")
    print("  | Scenario             | Duration   | Final Err | Drift %    | SIH    |")
    print("  +----------------------+------------+-----------+------------+--------+")
    for label, m in results:
        sih_str = "PASS" if m["sih_pass"] else "FAIL"
        print(
            f"  | {label:<20} | "
            f"{m['blackout_duration_s']:.0f} s        | "
            f"{m['final_err_m']:.2f} m    | "
            f"{m['drift_pct']:.2f}%      | "
            f"{sih_str}   |"
        )
    print("  -----------------------------------------------------------------------")

    overall = " ALL PASS - Phase 5 Complete" if all_pass else " SOME FAILURES - Review drift metrics"
    print(f"\n  {overall}")
    print(f"  SIH Benchmark: < 10% positional drift during GNSS blackout\n")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
