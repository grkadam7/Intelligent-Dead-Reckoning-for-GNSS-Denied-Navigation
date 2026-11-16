# AI-ML Based Intelligent Dead Reckoning System for Seamless Navigation (IDR-INS)
*SIH Problem Statement ID: 26168 | Organization: Indian Space Research Organisation (ISRO)*

## What This Is
An edge-deployable software engine and mobile application that transforms standalone smartphones and external IMU sensors into an Intelligent Dead Reckoning (IDR) navigation system with seamless GNSS fusion. When GNSS outages occur (tunnels, underpasses, multi-level parking, deep urban canyons, jamming), the system instantly transitions to inertial tracking (INS), estimating vehicle speed and road trajectory without requiring any physical connection to the vehicle's internal computer or OBD-II port, and seamlessly transitions back when GNSS returns.

## Core Value
Restrict positional drift to less than 10% of total distance travelled during GNSS signal blackouts using low-cost noisy MEMS smartphone sensors without OBD-II feeds, ensuring uninterrupted, lane-level navigation across Indian road conditions.

## Current Milestone: v1.1 Mobile Navigation App Rebuild

**Goal:** Turn the demo app into an offline navigation client whose dead reckoning survives a real bus outage, with native engine integration, live map matching, and traceable field testing.

**Target features:**
- DR correctness: speed continuity through outages, no spurious ZUPT freeze, monotonic uncertainty growth, repeatable 20 s outage test.
- Native engine: Android NDK/CMake build of `libidr_mobile` and FFI integration with a Dart fallback.
- Live map layer: snap the drifting live trajectory to the bundled Hubli-Dharwad road graph; add a user-approved local map download flow.
- Foreground tracking and drive recording: keep 10 Hz updates with the screen off; export raw IMU/GNSS/snapshot traces.
- UI and diagnostics redesign: honest mode, age, uncertainty and calibration display, outage control, diagnostics export.

**Key context:** v1.0 (8 phases) is complete. A Nov 2026 bus recording shows GNSS-aided mode healthy (~50 km/h, ±2.5 m) but dead reckoning collapses speed from 50 to 1 km/h at tunnel entry and freezes position. Offline-only, no OBD-II. Flutter 3.47; GitHub Actions produces the release APK.

## Business & Stakeholder Context
- **Organization**: Indian Space Research Organisation (ISRO), Department of Space
- **Target Beneficiaries**: Commercial logistics, ride-hailing (Ola/Uber/Rapido), quick commerce (Blinkit/Zepto/Swiggy), emergency first responders, and millions of Indian two-wheeler/four-wheeler drivers without factory-fitted wheel INS.
- **Success Metrics**:
  - Positional drift < 10% of distance during GNSS outages (e.g. < 5m over 50m in < 1 min; < 100m over 1km at 60 km/h).
  - Continuous 10 Hz position update rate on mobile; up to 200 Hz update rate on edge engine using high-grade/FOG IMU data.
  - Sub-100 millisecond seamless transition between GNSS-aided INS and Dead Reckoning.
  - Accurate forward speed prediction solely from IMU noise and vibrations.

## Requirements

### Validated
(None yet — initialize development to validate)

### Active (v1.1)
- [ ] **DR-01**: Vehicle speed stays continuous at GNSS-outage entry and through a 20 s blackout (no ZUPT collapse at constant speed).
- [ ] **DR-02**: Horizontal uncertainty grows monotonically with time since last fix and reflects accumulated dead-reckoning drift.
- [ ] **DR-03**: A repeatable 20 s simulated outage drives the observable GNSS → Dead Reckoning → Recovering → GNSS state sequence in Live mode.
- [ ] **NENG-01**: Android release APK builds `libidr_mobile.so` from the C/C++ core via NDK/CMake and bundles it as an FFI-loadable library.
- [ ] **NENG-02**: Real IMU/GNSS samples are pushed through the native C ABI and published as 10 Hz NavSnapshots, with a documented fallback if the library is unavailable.
- [ ] **MAP-01**: Live trajectories are snapped to the bundled Hubli-Dharwad road graph, with the matched road visible in diagnostics.
- [ ] **MAP-02**: User can approve and download an OSM road region for their current area; the downloaded region becomes the active offline map without a rebuild.
- [ ] **TRK-01**: Live tracking continues at 10 Hz while the screen is off or the app is backgrounded.
- [ ] **TRK-02**: User can record and export a drive trace (raw IMU, GNSS fixes, NavSnapshots) for post-trip analysis.
- [ ] **UI-01**: HUD and diagnostics show honest mode, fix age, uncertainty, speed and calibration state, plus an export action.
- [ ] **UI-02**: Calibration is derived from an actual stationary window instead of being set on every near-gravity IMU sample.

### Out of Scope
- **OBD-II / CAN Bus Hardware Integration**: Solution must be strictly non-invasive; relying on vehicle internal computer defeats accessibility for older vehicles and two-wheelers.
- **Continuous Cloud Processing**: Real-time navigation and inference cannot depend on active 4G/5G connectivity inside tunnels or remote valleys; all inference and map-matching must function offline on-device.

## Context
- **Sensor Challenges**: Consumer-grade smartphone MEMS IMUs suffer from run-to-run bias, thermo-mechanical drift, scale factor errors, chassis vibration, engine harmonics, and sudden shocks from Indian road potholes.
- **Kinematic Constraints**: Land vehicles obey Non-Holonomic Constraints (NHC)—vehicles do not slide sideways or fly upwards. Applying NHC ($v_y \approx 0, v_z \approx 0$ in vehicle frame) dramatically dampens perpendicular drift.
- **Dataset**: IO-VNBD (Inertial and Odometry benchmark dataset for ground vehicle positioning) provides ground truth odometry, GNSS trajectories, and raw IMU readings for rigorous model training and evaluation.

## Constraints
- **Hardware**: Must run on standard commercial Android/iOS smartphones without external sensors for the mobile tier, and standard edge compute (e.g. Raspberry Pi / Jetson / x86) for the edge tier.
- **Latency**: Position update $\le 100$ ms on mobile (10 Hz); $\le 5$ ms on edge (200 Hz).
- **Offline Capability**: Offline map tiles and routing graphs must be pre-bundled or cached locally.

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Hybrid Architecture (Cloud Train, Edge Infer) | Complex deep learning models (temporal CNN/LSTM/Transformer) can be trained on large datasets (IO-VNBD) and exported as quantized ONNX/TFLite models for sub-millisecond mobile inference. | — Pending |
| Coordinate System Normalization | Phone frame must be dynamically rotated to Vehicle Frame (Forward, Lateral, Vertical) using gravity vector (accelerometer static window) and principal acceleration/gyro axis during initial motion. | — Pending |
| Multi-Stage Fusion (Physics EKF + AI Residual) | Pure physics EKF/UKF provides baseline stability and kinematic bounds; AI models predict forward speed and sensor bias residuals to prevent drift explosion. | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-11-16 after v1.1 milestone start (bus-test findings)*
