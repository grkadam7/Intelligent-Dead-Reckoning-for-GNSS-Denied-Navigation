"""
Export deterministic reviewed reference oracle fixtures for native C/C++ parity testing.
Exports to native/tests/fixtures/reviewed.json.
"""

import json
import os
import sys
from pathlib import Path
import numpy as np

# Add project root to sys.path
repo_root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(repo_root))

from src.data.iovnbd_loader import generate_synthetic_run
from src.fusion.fusion_engine import GNSSINSFusionEngine, NavigationMode
from src.map_matching.road_network import RoadNetwork


def export_reference_oracle(output_file=None):
    if output_file is None:
        output_file = repo_root / "native" / "tests" / "fixtures" / "reviewed.json"
    else:
        output_file = Path(output_file)

    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Generate small deterministic synthetic run (20 epochs = 2.0 s at 10 Hz)
    dt = 0.1
    duration_s = 2.0
    data = generate_synthetic_run(duration_s=duration_s, dt=dt, seed=12345)

    # Create road network
    road_net = RoadNetwork.create_from_route_waypoints(
        waypoints_enu=data.enu_positions,
        subsegment_length_m=50.0,
        road_name="Reference Corridor",
        origin_geodetic=data.origin_geodetic,
    )

    engine = GNSSINSFusionEngine(
        sampling_rate=10.0,
        road_network=road_net,
        max_gnss_accuracy_m=15.0,
        reconvergence_duration_sec=1.5,
    )

    epochs = []
    # Blackout window between t = 1.0s and t = 1.5s (epochs 10 to 15)
    blackout_start = 1.0
    blackout_end = 1.5

    for i in range(len(data.timestamps_s)):
        t = float(data.timestamps_s[i])
        is_blackout = (blackout_start <= t <= blackout_end)

        accel_raw = data.accel[i].astype(float)
        gyro_raw = data.gyro[i].astype(float)

        gnss_pos = None if is_blackout else data.enu_positions[i].astype(float)
        gnss_vel = None if is_blackout else np.array([float(data.gps_speed_mps[i]), 0.0, 0.0])
        gnss_acc = 999.0 if is_blackout else float(data.gps_accuracy_m[i])

        out = engine.process_epoch(
            timestamp_s=t,
            accel_raw=accel_raw,
            gyro_raw=gyro_raw,
            gnss_pos_enu=gnss_pos,
            gnss_vel_enu=gnss_vel,
            gnss_accuracy_m=gnss_acc,
            ai_forward_speed=float(data.gps_speed_mps[i]),
            explicit_outage=is_blackout,
        )

        mode_str = out.navigation_mode.name

        epoch_record = {
            "epoch_index": i,
            "timestamp_s": round(t, 4),
            "input": {
                "accel_raw": [round(float(x), 6) for x in accel_raw],
                "gyro_raw": [round(float(x), 6) for x in gyro_raw],
                "gnss_pos_enu": [round(float(x), 4) for x in gnss_pos] if gnss_pos is not None else None,
                "gnss_accuracy_m": round(gnss_acc, 2),
                "is_outage": is_blackout,
            },
            "intermediate": {
                "ekf_position": [round(float(x), 4) for x in out.position_enu],
                "ekf_velocity": [round(float(x), 4) for x in out.velocity_enu],
                "heading_deg": round(float(out.heading_deg), 2),
            },
            "output": {
                "position_enu": [round(float(x), 4) for x in out.position_enu],
                "navigation_mode": mode_str,
                "is_outage": bool(out.is_outage),
                "outage_duration_s": round(float(out.outage_duration_s), 3),
                "detection_latency_ms": round(float(out.detection_latency_ms), 2),
            }
        }
        epochs.append(epoch_record)

    fixture = {
        "metadata": {
            "generator": "tools/phase7/export_reference.py",
            "schema_version": "1.0.0",
            "origin_geodetic": list(data.origin_geodetic),
            "num_epochs": len(epochs),
            "dt_sec": dt,
        },
        "tolerances": {
            "position_enu_m": 0.0001,
            "velocity_enu_mps": 0.0001,
            "heading_deg": 0.05,
            "detection_latency_ms": 1.0,
        },
        "epochs": epochs
    }

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(fixture, f, indent=2)

    print(f"Exported {len(epochs)} reference epochs to {output_file}")


if __name__ == "__main__":
    export_reference_oracle()

