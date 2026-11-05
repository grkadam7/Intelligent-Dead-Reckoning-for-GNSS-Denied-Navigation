# Phase 7 Plan 01: Offline Map & Replay Navigation Client Summary

**Execution Date:** 2026-11-05  
**Phase:** 07-mobile-navigation-application  
**Plan:** 01  
**Status:** Completed  

## Deliverables & Achievements

1. **Architecture Decision Record (`mobile/docs/architecture_decision.md`):**
   - Formalized Flutter + MapLibre Native compatible offline vector architecture targeting Android.
   - Decoupled high-rate C numerical core from mobile UI via typed snapshot contracts (`NavSnapshot`).

2. **Offline Package & Replay Assets (`mobile/assets/`):**
   - **Offline Vector Road Network (`mobile/assets/regions/urban_route.geojson`):** GeoJSON transit corridor and connected grid for the Coventry evaluation track.
   - **Deterministic 10 Hz Replay (`mobile/assets/replay/navigation.json`):** 600 timestamped snapshots (60s) exercising nominal GNSS-aided cruise, sub-50ms blackout transition to AI-Speed + INS + NHC + HMM dead reckoning, smooth exponential reconvergence upon GNSS recovery, and ZUPT stops.
   - **Package Manifest (`mobile/assets/regions/manifest.json`):** Bounding box, zoom levels, SHA-256 checksums, and CC-BY-4.0 / OpenStreetMap licensing.

3. **Validation & Automated Tests (`tools/phase7/`, `tests/test_phase7_assets.py`):**
   - `validate_assets.py` verifies file checksums, coordinate boundaries, monotonic sequence counters, and $\ge 10$ Hz epoch intervals.
   - Passed 100% in pytest test suite (`tests/test_phase7_assets.py`).

4. **Mobile Navigation Client Implementation (`mobile/lib/`):**
   - `NavSnapshot` typed contract with navigation mode, calibration stages, speed, heading, and 95% horizontal uncertainty radius.
   - `ReplaySource` stream publisher at 10 Hz with pause, resume, seek, and loop support.
   - `OfflineRegionStore` vector feature parser and spatial boundary validator.
   - `NavigationScreen` featuring the mandatory persistent **"Simulation — deterministic replay"** banner, high-contrast dynamic status chips ("GNSS Aided (Active)", "GNSS Outage (Dead Reckoning Mode)"), offline vector road renderer, vehicle heading chevron cursor, uncertainty circle, speed HUD, calibration sheet, and diagnostics sheet.
