"""
Asset generator for Phase 7 Mobile Navigation Application.
Generates:
1. mobile/assets/regions/urban_route.geojson (GeoJSON vector road network)
2. mobile/assets/replay/navigation.json (60s at 10 Hz = 600 deterministic NavSnapshots)
3. mobile/assets/regions/manifest.json (Package inventory with checksums, bbox, and licensing)
"""

import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np

# Coordinates for origin (Coventry, UK - IO-VNBD dataset area)
LAT0, LON0, ALT0 = 52.40166, -1.50529, 147.5
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_E2 = 2.0 * WGS84_F - WGS84_F**2


def enu_to_geodetic(east, north, up, lat0, lon0, alt0):
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    sin_lat = math.sin(lat0_rad)
    cos_lat = math.cos(lat0_rad)
    sin_lon = math.sin(lon0_rad)
    cos_lon = math.cos(lon0_rad)

    n0 = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat**2)
    x0 = (n0 + alt0) * cos_lat * cos_lon
    y0 = (n0 + alt0) * cos_lat * sin_lon
    z0 = (n0 * (1.0 - WGS84_E2) + alt0) * sin_lat

    # Rotation matrix: ENU to ECEF
    dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
    dy =  cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
    dz =  cos_lat * north + sin_lat * up

    x = x0 + dx
    y = y0 + dy
    z = z0 + dz

    # ECEF to geodetic (Bowring's algorithm)
    p = math.sqrt(x**2 + y**2)
    theta = math.atan2(z * WGS84_A, p * WGS84_B)
    e_prime_sq = (WGS84_A**2 - WGS84_B**2) / (WGS84_B**2)

    lat = math.atan2(
        z + e_prime_sq * WGS84_B * (math.sin(theta)**3),
        p - WGS84_E2 * WGS84_A * (math.cos(theta)**3)
    )
    lon = math.atan2(y, x)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * (math.sin(lat)**2))
    alt = p / math.cos(lat) - n

    return math.degrees(lat), math.degrees(lon), alt


def generate_assets():
    repo_root = Path(__file__).resolve().parents[2]
    regions_dir = repo_root / "mobile" / "assets" / "regions"
    replay_dir = repo_root / "mobile" / "assets" / "replay"
    regions_dir.mkdir(parents=True, exist_ok=True)
    replay_dir.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------
    # 1. Generate 60-second 10 Hz Trajectory
    # ---------------------------------------------------------
    duration_s = 60.0
    hz = 10.0
    dt = 1.0 / hz
    n_samples = int(duration_s * hz)  # 600 samples

    snapshots = []
    e, n = 0.0, 0.0
    yaw_deg = 90.0  # Heading East initially

    # Keep track of waypoints for road network
    route_coords_wgs84 = []
    min_lat, max_lat = LAT0, LAT0
    min_lon, max_lon = LON0, LON0

    for i in range(n_samples):
        t = i * dt
        seq = i + 1

        # Speed and yaw profile
        if t < 10.0:
            speed = 1.4 * t  # Accel 0 to 14 m/s
            yaw_rate = 0.0
            mode = "gnss_aided"
            mode_reason = "GNSS fix nominal (HDOP 0.8, 12 SVs)"
            cov = 1.8
        elif t < 30.0:
            speed = 14.0  # Cruise in tunnel (GNSS Blackout)
            yaw_rate = 0.0
            mode = "dead_reckoning"
            mode_reason = "GNSS blackout detected (<50ms): AI-Speed + INS + NHC + HMM Active"
            # Covariance grows slightly during blackout
            cov = 1.8 + 0.15 * (t - 10.0)
        elif t < 35.0:
            speed = 10.0  # Slowing down, turning left (recovering GNSS)
            yaw_rate = 18.0  # 90 deg in 5s
            mode = "recovering"
            mode_reason = "GNSS restored: Exponential decay reconvergence active"
            cov = max(1.8, 4.8 - 0.6 * (t - 30.0))
        elif t < 50.0:
            speed = 14.0  # Cruise heading North
            yaw_rate = 0.0
            mode = "gnss_aided"
            mode_reason = "GNSS fix nominal (HDOP 0.9, 14 SVs)"
            cov = 1.8
        else:
            speed = max(0.0, 14.0 - 1.4 * (t - 50.0))  # Decel to 0 m/s
            yaw_rate = 0.0
            mode = "gnss_aided"
            mode_reason = "ZUPT stop detected" if speed == 0 else "GNSS fix nominal"
            cov = 1.5 if speed == 0 else 1.8

        yaw_deg = (yaw_deg - yaw_rate * dt) % 360.0  # Mathematical angle vs Navigation heading
        # In navigation: 0 = North, 90 = East
        nav_heading = (450.0 - yaw_deg) % 360.0

        vx = speed * math.cos(math.radians(yaw_deg))
        vy = speed * math.sin(math.radians(yaw_deg))
        e += vx * dt
        n += vy * dt

        lat, lon, alt = enu_to_geodetic(e, n, 0.0, LAT0, LON0, ALT0)
        min_lat = min(min_lat, lat)
        max_lat = max(max_lat, lat)
        min_lon = min(min_lon, lon)
        max_lon = max(max_lon, lon)

        route_coords_wgs84.append([lon, lat])

        # Simulated raw offset (inertial drift before map matching)
        if mode == "dead_reckoning":
            drift_m = 0.05 * (t - 10.0)
            raw_lat, raw_lon, _ = enu_to_geodetic(e + drift_m * 0.7, n + drift_m * 0.7, 0.0, LAT0, LON0, ALT0)
        else:
            raw_lat, raw_lon = lat, lon

        snapshot = {
            "schema_version": "1.0.0",
            "sequence": seq,
            "source": "replay",
            "timestamp_ms": int(t * 1000),
            "state_age_ms": 25,
            "latitude": round(lat, 7),
            "longitude": round(lon, 7),
            "altitude_m": round(alt, 2),
            "raw_latitude": round(raw_lat, 7),
            "raw_longitude": round(raw_lon, 7),
            "speed_mps": round(speed, 2),
            "speed_kmh": round(speed * 3.6, 1),
            "speed_valid": True,
            "heading_deg": round(nav_heading, 1),
            "heading_valid": True,
            "navigation_mode": mode,
            "mode_reason": mode_reason,
            "horizontal_uncertainty_m": round(cov, 2),
            "uncertainty_valid": True,
            "is_matched": True,
            "matched_link_id": "corridor_link_0" if t < 30 else ("turn_link_1" if t < 35 else "corridor_link_2"),
            "calibration_stage": "calibrated",
            "roll_deg": 1.2,
            "pitch_deg": -0.8,
            "yaw_deg": round(nav_heading, 1),
            "simulated": True
        }
        snapshots.append(snapshot)

    # Write replay file
    replay_file = replay_dir / "navigation.json"
    with open(replay_file, "w", encoding="utf-8") as f:
        json.dump(snapshots, f, indent=2)
    print(f"Generated {len(snapshots)} snapshots at {replay_file}")

    # ---------------------------------------------------------
    # 2. Generate GeoJSON Vector Road Network
    # ---------------------------------------------------------
    geojson_features = [
        {
            "type": "Feature",
            "id": "transit_corridor_main",
            "properties": {
                "id": "transit_corridor_main",
                "name": "A45 Coventry Highway Transit Corridor",
                "highway": "primary",
                "maxspeed": "50 km/h",
                "lanes": 2,
                "surface": "asphalt"
            },
            "geometry": {
                "type": "LineString",
                "coordinates": route_coords_wgs84
            }
        }
    ]

    # Add parallel and cross streets to create realistic connected offline map grid
    # Cross street at origin
    cross_start_lat, cross_start_lon, _ = enu_to_geodetic(0.0, -100.0, 0.0, LAT0, LON0, ALT0)
    cross_end_lat, cross_end_lon, _ = enu_to_geodetic(0.0, 100.0, 0.0, LAT0, LON0, ALT0)
    geojson_features.append({
        "type": "Feature",
        "id": "cross_street_south",
        "properties": {
            "id": "cross_street_south",
            "name": "Station Approach Road",
            "highway": "secondary",
            "maxspeed": "30 km/h"
        },
        "geometry": {
            "type": "LineString",
            "coordinates": [[cross_start_lon, cross_start_lat], [cross_end_lon, cross_end_lat]]
        }
    })

    geojson_data = {
        "type": "FeatureCollection",
        "name": "Coventry_IDR_Offline_Corridor",
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}
        },
        "features": geojson_features
    }

    geojson_file = regions_dir / "urban_route.geojson"
    with open(geojson_file, "w", encoding="utf-8") as f:
        json.dump(geojson_data, f, indent=2)
    print(f"Generated GeoJSON vector map at {geojson_file}")

    # ---------------------------------------------------------
    # 3. Compute Checksums & Write Manifest
    # ---------------------------------------------------------
    def sha256_file(filepath):
        h = hashlib.sha256()
        with open(filepath, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                h.update(chunk)
        return h.hexdigest()

    # Expand bounding box slightly for map padding
    padding = 0.002
    bbox = [
        round(min_lon - padding, 6),
        round(min_lat - padding, 6),
        round(max_lon + padding, 6),
        round(max_lat + padding, 6)
    ]

    manifest_data = {
        "package_id": "coventry_urban_transit_v1",
        "title": "Coventry Urban Transit Corridor & Test Grid",
        "version": "1.0.0",
        "schema_version": "1.0",
        "crs": "EPSG:4326",
        "center": [round(LON0, 6), round(LAT0, 6)],
        "bounds": bbox,
        "zoom_levels": {
            "min_zoom": 12,
            "max_zoom": 18,
            "default_zoom": 15
        },
        "license": {
            "type": "CC-BY-4.0",
            "attribution": "ISRO SIH 26168 Synthetic Evaluation Benchmark / OpenStreetMap Contributors",
            "commercial_use_allowed": True,
            "redistribution_allowed": True
        },
        "files": {
            "road_network": {
                "path": "urban_route.geojson",
                "format": "geojson",
                "size_bytes": geojson_file.stat().st_size,
                "sha256": sha256_file(geojson_file)
            },
            "replay_stream": {
                "path": "../replay/navigation.json",
                "format": "json_snapshots",
                "sample_rate_hz": 10.0,
                "duration_seconds": duration_s,
                "num_snapshots": len(snapshots),
                "size_bytes": replay_file.stat().st_size,
                "sha256": sha256_file(replay_file)
            }
        }
    }

    manifest_file = regions_dir / "manifest.json"
    with open(manifest_file, "w", encoding="utf-8") as f:
        json.dump(manifest_data, f, indent=2)
    print(f"Generated manifest at {manifest_file}")


if __name__ == "__main__":
    generate_assets()
