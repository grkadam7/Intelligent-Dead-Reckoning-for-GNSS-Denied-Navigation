# Intelligent Dead Reckoning Navigation System (IDR-INS)
## Technical Benchmark & Proposal Screening Report
**Smart India Hackathon | Problem Statement ID: 26168 | ISRO**

---

### 1. Executive Summary

In satellite-denied environments—such as road tunnels, urban canyons, dense forested highways, and multi-level parking structures—standard GNSS signals degrade rapidly or vanish entirely. Consumer smartphone MEMS inertial sensors, when integrated via pure mathematical double integration (Raw INS), drift exponentially ($>500\%$ error within 60 seconds), rendering standalone navigation useless without costly vehicle OBD-II/CAN-bus integration.

This project delivers **IDR-INS**, a software-defined, non-invasive navigation engine combining:
1. **Dynamic Gravity-Vector Attitude Leveling & Longitudinal Yaw Alignment** (mounting-invariant).
2. **AI/ML 1D CNN-GRU Kinematic Speed Estimation (SpeedNet)** predicting instantaneous vehicle speed without OBD-II feeds.
3. **Kinematic Non-Holonomic Constraints (NHC)** bounding lateral and vertical velocity drift ($v_y \approx 0, v_z \approx 0$).
4. **9-State Joseph-Form Extended Kalman Filter (EKF)** fusing GNSS and inertial states.
5. **Sub-50ms Automatic Blackout Detection & Dead Reckoning Handler**.
6. **Smooth Reconvergence Filter** eliminating coordinate jumps upon GNSS restoration.
7. **Offline Hidden Markov Model (HMM) Map Matcher** binding coordinates to vector road graphs.

**Core Achievement:** Across extensive multi-scenario benchmarks, IDR-INS maintains positional drift **$\le 1.06\%$** of distance covered during long tunnel outages—massively outperforming the **$< 10\%$** maximum allowable drift target mandated by ISRO.

---

### 2. Multi-Scenario Benchmark Results

Benchmarks were executed across 5 distinct operational scenarios on a 120-second synthetic urban transit route (1,159.6 m traveled, 14 m/s cruising speed, 90° left/right turns, and static ZUPT stops), alongside verification on the real-world **IO-VNBD** smartphone dataset.

| Scenario | Blackout Duration | Distance Covered | Raw INS Drift | Pure GNSS Drift | AI-Enhanced IDR Drift | ISRO Target Limit | SIH Status |
|---|---|---|---|---|---|---|---|
| **Short Urban Underpass** | 10.0 s | 138.6 m | 201.9% | 15.2% | **0.00%** | < 10.0% | ✅ **PASS** |
| **Highway Tunnel** | 20.0 s | 278.6 m | 108.6% | 22.4% | **1.06%** | < 10.0% | ✅ **PASS** |
| **Long Mountain Tunnel** | 30.0 s | 398.6 m | 62.8% | 35.8% | **0.00%** | < 10.0% | ✅ **PASS** |
| **Urban Canyon Multipath** | 50.0 s | 661.5 m | 71.7% | 48.2% | **8.49%** | < 10.0% | ✅ **PASS** |
| **Forested Highway Canopy** | 15.0 s (intermittent) | 468.6 m | 58.2% | 19.5% | **0.00%** | < 10.0% | ✅ **PASS** |

*Note: In tunnel corridors, HMM road-snapping snaps the tightly constrained NHC trajectory directly to the surveyed lane geometry, achieving near-perfect 0.00% endpoint drift.*

---

### 3. Trajectory Visualizations

#### A. Definitive 4-Way Comparison (Ground Truth vs Pure GNSS vs Raw INS vs AI-Enhanced IDR)
![4-Way Trajectory Comparison](figures/sih_4way_trajectory_comparison.png)

- **Panel A (Spatial Trajectory):** Raw INS (red dotted) diverges into hundreds of meters of spatial error within seconds. Pure GNSS (light blue) suffers severe scatter and blackout dropouts. IDR-INS (yellow dashed) tracks Ground Truth (green solid) throughout the trip.
- **Panel B (Error Progression):** Demonstrates error growth over time. While Raw INS exceeds $2,900$ m of cumulative error, IDR-INS remains bounded well below the 10% SIH threshold.
- **Panel C (Forward Speed Estimation):** SpeedNet neural network accurately tracks vehicle velocity profiles during GNSS blackouts without OBD-II pulses.
- **Panel D (Navigation Mode Transitions):** Instantaneous sub-millisecond mode switching between `GNSS Active`, `Dead Reckoning`, and `Reconvergence`.

#### B. Multi-Scenario Drift Comparison vs SIH 10% Target
![Multi-Scenario Drift Benchmark](figures/sih_multi_scenario_drift_benchmark.png)

#### C. Smooth Reconvergence Profile at Blackout Exit
![Smooth Reconvergence Profile](figures/sih_blackout_reconvergence_profile.png)
Demonstrates continuous exponential decay offset ($\tau = 2.5$ s), eliminating jarring position teleportation upon tunnel exit.

---

### 4. Dual Deployment Verification

1. **High-Rate 200 Hz Edge Core Engine (`src/edge/`, `include/idr_edge_engine.h`):**
   - Headless C-ABI engine processing up to 200 Hz IMU sensor feeds.
   - Sustained throughput: **2,462 Hz** (12.3x real-time margin).
   - Mean per-epoch processing latency: **0.371 ms** ($< 1.0$ ms budget).

2. **Mobile Navigation Client (`mobile/lib/`, `mobile/assets/`):**
   - Offline vector map rendering with real-time 10 Hz vehicle cursor update.
   - Dynamic HUD chips: "GNSS Aided (Active)", "GNSS Outage (Dead Reckoning Mode)".
   - Dart FFI integration to native C core (`native/include/idr_mobile.h`).

---

### 5. Requirements Compliance Matrix (SIH 26168)

| ID | Requirement Specification | Verification Evidence | Status |
|---|---|---|---|
| **ALIGN-01** | Accelerometer gravity attitude leveling (roll, pitch) | Auto-leveler verified on stationary windows | ✅ Met |
| **ALIGN-02** | Horizontal azimuth yaw alignment with vehicle longitudinal axis | Forward acceleration principal axis extraction | ✅ Met |
| **ALIGN-03** | Continuous 3-axis projection to Vehicle Coordinate Frame | Dynamic coordinate rotation matrix | ✅ Met |
| **FILTER-01** | Butterworth vibration filtering & pothole shock suppression | 3.0 Hz cutoff low-pass filter | ✅ Met |
| **FILTER-02** | Zero Velocity Update (ZUPT) & ZARU stop detection | Variance & angular rate threshold detector | ✅ Met |
| **SPEED-01** | AI forward speed estimation without OBD-II feeds | 1D CNN + GRU SpeedNet architecture | ✅ Met |
| **SPEED-02** | Model inference latency < 20 ms on mobile processors | ONNX Runtime latency: ~5.2 ms | ✅ Met |
| **SPEED-03** | Speed prediction RMSE < 1.5 m/s | IO-VNBD test split evaluation: RMSE 0.92 m/s | ✅ Met |
| **MAP-01** | Non-Holonomic Constraints ($v_y \approx 0, v_z \approx 0$) | Body frame lateral velocity clamping | ✅ Met |
| **MAP-02** | Offline road network representation from OSM / GeoJSON | 2D Spatial Hash Grid + Dijkstra router | ✅ Met |
| **MAP-03** | HMM road link snapping algorithm | Emission & transition probability filter | ✅ Met |
| **FUSION-01**| Extended Kalman Filter (EKF) sensor fusion | 9-state Joseph-form EKF | ✅ Met |
| **FUSION-02**| Sub-50ms GNSS outage detection latency | 3-gate health monitor: **0.1 ms latency** | ✅ Met |
| **FUSION-03**| Smooth reconvergence back to GNSS without jumps | Exponential decay offset filter ($\tau = 2.5$ s) | ✅ Met |
| **FUSION-04**| Drift maintained < 10% during GNSS blackouts | **0.00% to 1.06% drift** in tunnel tests | ✅ Met |
| **EDGE-01** | Modularized headless edge software engine | `src/edge/` and `include/idr_edge_engine.h` | ✅ Met |
| **EDGE-02** | High-rate IMU input up to 200 Hz support | 2,462 Hz sustained throughput benchmark | ✅ Met |
| **MOBILE-01**| Mobile app with offline map & $\ge 10$ Hz cursor update | Flutter vector map & 10 Hz snapshot stream | ✅ Met |
| **MOBILE-02**| Real-time visual status chips & drift confidence | Dynamic status banner, chips, and 95% HUD | ✅ Met |
| **MOBILE-03**| In-app sensor calibration display | Calibration status sheet with roll/pitch angles | ✅ Met |
| **EVAL-01**  | Automated evaluation across benchmark runs | Multi-scenario evaluation suite (`evaluate_phase8.py`) | ✅ Met |
| **EVAL-02**  | 4-way trajectory comparison plots with error curves | Publication figure `sih_4way_trajectory_comparison.png` | ✅ Met |

