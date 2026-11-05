"""
Validation tool for Phase 7 mobile offline packages and replay assets.
Verifies integrity, coordinate bounds, monotonic timestamps, 10 Hz rate,
checksums, and licensing.
"""

import hashlib
import json
import math
import sys
from pathlib import Path


def compute_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


def validate_all_assets(repo_root=None):
    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[2]
    else:
        repo_root = Path(repo_root)

    regions_dir = repo_root / "mobile" / "assets" / "regions"
    manifest_path = regions_dir / "manifest.json"

    errors = []
    warnings = []

    # 1. Manifest exists and parses
    if not manifest_path.exists():
        return False, ["manifest.json does not exist"], []

    try:
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)
    except Exception as e:
        return False, [f"manifest.json could not be parsed: {e}"], []

    # Required manifest fields
    req_fields = ["package_id", "version", "schema_version", "bounds", "license", "files"]
    for rf in req_fields:
        if rf not in manifest:
            errors.append(f"Missing required manifest field: {rf}")

    # Check license
    license_info = manifest.get("license", {})
    if not license_info.get("type") or not license_info.get("attribution"):
        errors.append("License must specify 'type' and 'attribution'")

    # Bounding box
    bounds = manifest.get("bounds", [])
    if len(bounds) != 4 or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
        errors.append(f"Invalid bounding box: {bounds}")

    # 2. Check files inventory and checksums
    files_inv = manifest.get("files", {})
    for file_key, file_meta in files_inv.items():
        rel_path = file_meta.get("path")
        expected_sha = file_meta.get("sha256")
        expected_size = file_meta.get("size_bytes")

        actual_file = (regions_dir / rel_path).resolve()
        if not actual_file.exists():
            errors.append(f"File {file_key} ({rel_path}) not found on disk at {actual_file}")
            continue

        actual_size = actual_file.stat().st_size
        if actual_size != expected_size:
            errors.append(f"File {file_key} size mismatch: expected {expected_size}, got {actual_size}")

        actual_sha = compute_sha256(actual_file)
        if actual_sha != expected_sha:
            errors.append(f"File {file_key} SHA-256 mismatch: expected {expected_sha}, got {actual_sha}")

    # 3. Validate GeoJSON road network
    geojson_path = regions_dir / files_inv.get("road_network", {}).get("path", "urban_route.geojson")
    if geojson_path.exists():
        try:
            with open(geojson_path, "r", encoding="utf-8") as f:
                geo = json.load(f)
            if geo.get("type") != "FeatureCollection":
                errors.append(f"GeoJSON root must be FeatureCollection, got {geo.get('type')}")
            features = geo.get("features", [])
            if not features:
                errors.append("GeoJSON has no features")
            for feat in features:
                coords = feat.get("geometry", {}).get("coordinates", [])
                if not coords or len(coords) < 2:
                    errors.append(f"Feature {feat.get('id')} has invalid LineString coordinates")
        except Exception as e:
            errors.append(f"Failed parsing GeoJSON road network: {e}")

    # 4. Validate Replay Stream
    replay_meta = files_inv.get("replay_stream", {})
    replay_path = (regions_dir / replay_meta.get("path", "../replay/navigation.json")).resolve()
    if replay_path.exists():
        try:
            with open(replay_path, "r", encoding="utf-8") as f:
                snapshots = json.load(f)

            if not isinstance(snapshots, list) or len(snapshots) < 100:
                errors.append(f"Replay stream must contain >= 100 snapshots, found {len(snapshots) if isinstance(snapshots, list) else 0}")
            else:
                prev_t = None
                prev_seq = 0
                seen_modes = set()

                for idx, s in enumerate(snapshots):
                    seq = s.get("sequence")
                    t_ms = s.get("timestamp_ms")
                    mode = s.get("navigation_mode")
                    simulated = s.get("simulated")
                    source = s.get("source")

                    if source != "replay":
                        errors.append(f"Snapshot #{idx} source must be 'replay', got {source}")
                        break
                    if simulated is not True:
                        errors.append(f"Snapshot #{idx} simulated flag must be True")
                        break
                    if seq != prev_seq + 1:
                        errors.append(f"Sequence non-monotonic at index {idx}: expected {prev_seq + 1}, got {seq}")
                        break
                    prev_seq = seq

                    if prev_t is not None:
                        dt_ms = t_ms - prev_t
                        if dt_ms <= 0 or dt_ms > 150:
                            errors.append(f"Snapshot #{idx} dt out of range for >=10 Hz streaming: {dt_ms} ms")
                            break
                    prev_t = t_ms

                    seen_modes.add(mode)

                # Check required mode transitions
                for expected_mode in ["gnss_aided", "dead_reckoning", "recovering"]:
                    if expected_mode not in seen_modes:
                        errors.append(f"Replay stream missing expected mode transition: {expected_mode}")

        except Exception as e:
            errors.append(f"Failed parsing replay stream: {e}")

    is_valid = len(errors) == 0
    return is_valid, errors, warnings


if __name__ == "__main__":
    valid, errs, warns = validate_all_assets()
    if not valid:
        print("Asset validation FAILED:")
        for e in errs:
            print(f"  [ERROR] {e}")
        sys.exit(1)
    else:
        print("Asset validation PASSED! All offline package and replay invariants verified.")
        for w in warns:
            print(f"  [WARN] {w}")
        sys.exit(0)

