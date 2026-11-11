"""Fetch Hubli-Dharwad road network from OpenStreetMap (Overpass API).

Writes the app's offline region assets:
  mobile/assets/regions/hubli_dharwad.geojson
  mobile/assets/regions/hubli_dharwad_manifest.json

Run from the repo root: python tools/fetch_hubli_roads.py
"""

import json
import os
import urllib.request

# Hubli-Dharwad urban corridor: [south, west, north, east]
BBOX = (15.30, 74.95, 15.48, 75.20)
HIGHWAYS = (
    "motorway|trunk|primary|secondary|tertiary|"
    "residential|unclassified|living_street"
)
OUT_DIR = os.path.join("mobile", "assets", "regions")
OVERPASS = "https://overpass-api.de/api/interpreter"
MIRRORS = [
    OVERPASS,
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]

QUERY = f"""[out:json][timeout:240];
way["highway"~"^({HIGHWAYS})$"]({BBOX[0]},{BBOX[1]},{BBOX[2]},{BBOX[3]});
out geom;"""


def fetch() -> dict:
    last_error = None
    for endpoint in MIRRORS:
        try:
            req = urllib.request.Request(
                endpoint,
                data=QUERY.encode(),
                headers={"User-Agent": "idr-map-fetch/1.0", "Accept": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=300) as response:
                print(f"fetched from {endpoint}")
                return json.load(response)
        except Exception as error:  # noqa: BLE001 - try the next mirror
            last_error = error
            print(f"{endpoint} failed: {error}")
    raise SystemExit(f"all Overpass mirrors failed: {last_error}")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    data = fetch()

    features = []
    for element in data["elements"]:
        if element.get("type") != "way":
            continue
        geometry = element.get("geometry") or []
        coords = [
            [round(point["lon"], 5), round(point["lat"], 5)]
            for point in geometry
            if "lon" in point and "lat" in point
        ]
        # Drop consecutive duplicates created by rounding.
        deduped = [coords[0]] if coords else []
        for point in coords[1:]:
            if point != deduped[-1]:
                deduped.append(point)
        if len(deduped) < 2:
            continue

        tags = element.get("tags") or {}
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "id": str(element["id"]),
                    "name": tags.get("name", "Unnamed Road"),
                    "highway": tags.get("highway", "unclassified"),
                },
                "geometry": {"type": "LineString", "coordinates": deduped},
            }
        )

    geojson = {
        "type": "FeatureCollection",
        "name": "Hubli_Dharwad_OSM_Roads",
        "features": features,
    }
    geojson_path = os.path.join(OUT_DIR, "hubli_dharwad.geojson")
    with open(geojson_path, "w", encoding="utf-8") as handle:
        json.dump(geojson, handle, separators=(",", ":"))

    manifest = {
        "package_id": "hubli_dharwad_osm_v1",
        "title": "Hubli-Dharwad Road Network (OpenStreetMap)",
        "version": "1.0.0",
        "schema_version": "1.0",
        "crs": "EPSG:4326",
        "center": [round((BBOX[1] + BBOX[3]) / 2, 5), round((BBOX[0] + BBOX[2]) / 2, 5)],
        "bounds": [BBOX[1], BBOX[0], BBOX[3], BBOX[2]],
        "zoom_levels": {"min_zoom": 12, "max_zoom": 18, "default_zoom": 15},
        "license": {
            "type": "ODbL-1.0",
            "attribution": "OpenStreetMap contributors",
        },
        "files": {
            "road_network": {
                "path": "hubli_dharwad.geojson",
                "format": "geojson",
                "size_bytes": os.path.getsize(geojson_path),
            }
        },
    }
    manifest_path = os.path.join(OUT_DIR, "hubli_dharwad_manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)

    print(f"{len(features)} road ways")
    print(f"{os.path.getsize(geojson_path) / 1024 / 1024:.1f} MB -> {geojson_path}")


if __name__ == "__main__":
    main()
