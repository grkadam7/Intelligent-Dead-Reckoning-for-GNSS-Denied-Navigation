# Phase 7: Mobile Navigation Architecture Decision Record (ADR)

**Status:** Approved  
**Date:** 2026-11-05  
**Deciders:** User & Antigravity  

## Context

Phase 7 implements the client-side user interface and mobile runtime for the Intelligent Dead Reckoning (IDR-INS) navigation system (SIH Problem Statement 26168).
Requirements:
- `MOBILE-01`: Offline map interface with real-time vehicle position cursor at $\ge 10$ Hz update rate.
- `MOBILE-02`: Real-time visual status indicators: "GNSS Aided (Active)", "GNSS Outage (Dead Reckoning Mode)", confidence/uncertainty bounds.
- `MOBILE-03`: In-app sensor calibration status showing phone orientation relative to vehicle.

## Decision

1. **Framework:**
   - **Flutter (Dart)** selected for cross-platform rendering, consistent 60+ FPS UI compositing, declarative reactive state management, and direct C-ABI foreign function interface (`dart:ffi`).
   - Rendering engine: Vector road network and offline vector tile/GeoJSON rendering via offline map viewport (`MapLibre Native` compatible vector layer contract).

2. **Target Platform:**
   - **Android-first** deployment target.
   - Core numerical operations bind to the existing C ABI (`include/idr_edge_engine.h`) compiled as a shared library (`.so` via CMake/Android NDK), with deterministic simulation/replay available standalone.

3. **Offline Operation & Packaging:**
   - Fully standalone operation without external network requests or tile download APIs during driving.
   - Packaged offline region bundle: Versioned GeoJSON road network with spatial bounding box, verified checksums, and vector attributes.
   - High-rate deterministic simulation replay feed providing $\ge 10$ Hz timestamped `NavSnapshot` streams with GNSS-aided, blackout (IDR), and recovery transitions.

4. **Lifecycle & Safety Rules:**
   - Foreground-only navigation guarantees. Inactive/background state explicitly transitions to `suspended` status.
   - Strict differentiation between raw dead-reckoning trajectory and road-matched coordinates.
   - Explicit uncertainty bounds derived from covariance ($2\sigma$ / 95% horizontal uncertainty ellipse/radius).
