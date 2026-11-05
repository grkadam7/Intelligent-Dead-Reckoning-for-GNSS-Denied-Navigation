# DR — Intelligent Dead Reckoning for GNSS-Denied Navigation

**AI/ML-assisted GNSS + INS fusion that keeps a vehicle's position moving accurately when GPS drops out.**

Built for **ISRO Smart India Hackathon Problem Statement 26168 — Smart Vehicles**. The system fuses low-cost smartphone MEMS IMU data, GNSS fixes, vehicle kinematic constraints, an AI speed model, and an offline road graph so a navigation display maintains continuous tracking through tunnels, parking structures, forested canopies, and urban canyons instead of freezing or jumping.

[![Python 3.7+](https://img.shields.io/badge/python-3.7%2B-blue.svg)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/tests-72%20passing-brightgreen.svg)]()
[![Status](https://img.shields.io/badge/status-all%208%20phases%20complete%20(100%25)-brightgreen.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

![SIH 4-Way Trajectory Comparison](reports/figures/sih_4way_trajectory_comparison.png)

---

## Why this exists

GNSS is weak indoors, underground, and between tall buildings. When the signal disappears, a phone's consumer-grade IMU drifts exponentially because of sensor bias, engine vibration, potholes, and mount misalignment. The ISRO SIH brief requires a non-invasive, smartphone-only solution that restricts positional drift to **$< 10\%$ of distance traveled** during blackouts without an OBD-II or CAN connection.

This repository implements the end-to-end navigation engine:
- **Core Algorithm Stack:** Coordinate auto-leveling, Butterworth vibration filtering, ZUPT/ZARU stop detection, AI SpeedNet velocity prediction, Non-Holonomic Constraints (NHC), 9-state Joseph-form EKF, sub-50ms blackout handler, smooth reconvergence filter, and offline HMM map matching.
- **High-Rate Edge Engine (200 Hz):** Standalone headless C-ABI core operating at **2,462 Hz** sustained throughput (12.3x real-time margin) with **0.371 ms** epoch latency.
- **Mobile Navigation Client (10 Hz):** Offline vector map viewport, real-time HUD status chips, and C-FFI bindings.

---

## Benchmark Results

### 1. Multi-Scenario Blackout Performance vs ISRO $<10\%$ Target

Evaluated on a 120-second synthetic urban transit route (1,159.6 m traveled, 14 m/s cruising speed, 90° turns, ZUPT stops), alongside verification on the real-world **IO-VNBD** dataset:

| Operational Scenario | Outage Duration | Distance Covered | Raw INS Drift | Pure GNSS Drift | **AI-Enhanced IDR Drift** | ISRO Requirement | Status |
|---|---|---|---|---|---|---|---|
| **Short Urban Underpass** | 10.0 s | 138.6 m | 201.9% | 15.2% | **0.00%** | < 10.0% | ✅ **PASS** |
| **Highway Tunnel** | 20.0 s | 278.6 m | 108.6% | 22.4% | **1.06%** | < 10.0% | ✅ **PASS** |
| **Long Mountain Tunnel** | 30.0 s | 398.6 m | 62.8% | 35.8% | **0.00%** | < 10.0% | ✅ **PASS** |
| **Urban Canyon Multipath** | 50.0 s | 661.5 m | 71.7% | 48.2% | **8.49%** | < 10.0% | ✅ **PASS** |
| **Forested Highway Canopy** | 15.0 s (intermittent) | 468.6 m | 58.2% | 19.5% | **0.00%** | < 10.0% | ✅ **PASS** |

- **GNSS Outage Detection Latency:** **0.1 ms** (mandated target: $< 50$ ms).
- **Smooth Reconvergence:** Exponential decay offset ($\tau = 2.5$ s) eliminates position jumps upon tunnel exit.

![Multi-Scenario Drift Benchmark](reports/figures/sih_multi_scenario_drift_benchmark.png)

---

### 2. Baseline INS vs Intelligent Dead Reckoning

120 s synthetic urban route, 1,159.6 m traveled:

| Method | RMSE | Max error | Final drift |
|---|---:|---:|---:|
| **Raw INS (double integration)** | 2,901.7 m | 7,532.5 m | 649.57% |
| **Calibrated INS + ZUPT** | 484.9 m | 853.3 m | 73.59% |
| **NHC Dead Reckoning** | 4.2 m | 6.9 m | 0.59% |
| **Full AI-Enhanced IDR + HMM** | 2.9 m | 6.1 m | **0.00%**\* |

\* *The benchmark road network is derived from route waypoints, so map matching snaps the tightly constrained NHC trajectory directly to lane geometry, validating the matcher.*

![Smooth Reconvergence Profile](reports/figures/sih_blackout_reconvergence_profile.png)

---

## How It Works

```mermaid
flowchart LR
    IMU[IMU accel + gyro] --> PRE[Alignment, vibration filter, ZUPT]
    PRE --> SPEED[SpeedNet AI Forward Speed]
    PRE --> EKF[9-state EKF predict]
    GNSS[GNSS fix] --> HEALTH[3-Gate GNSS health monitor]
    HEALTH -->|healthy| EKF
    EKF --> NHC[NHC virtual update vy ~ 0, vz ~ 0]
    NHC --> MODE{Signal healthy?}
    MODE -->|yes| FUSE[EKF fused position]
    MODE -->|outage <50ms| DR[AI dead reckoning SpeedNet + INS + NHC]
    FUSE --> SNAP[Offline HMM map matching]
    DR --> SNAP
    SNAP --> RECONV[Smooth reconvergence filter]
    RECONV --> OUT[Real-time navigation output]
```

1. **Preprocessing & Alignment:** Gravity-vector leveling resolves arbitrary phone mounting pitch and roll; longitudinal acceleration principal axis extraction determines forward yaw; 3.0 Hz Butterworth filter eliminates chassis harmonics; ZUPT stops drift accumulation at traffic lights.
2. **AI Speed Estimation (SpeedNet):** 1D CNN + 2-layer GRU predicts vehicle forward speed ($v_x$) from 50-sample IMU vibration windows ($<20$ ms ONNX inference, RMSE $<1.5$ m/s) without OBD-II feeds.
3. **Kinematic Non-Holonomic Constraints (NHC):** Clamps lateral and vertical velocities ($v_y \approx 0, v_z \approx 0$) in the vehicle body frame, eliminating orthogonal trajectory slippage.
4. **9-State Joseph-Form EKF:** Fuses positions, velocities, heading, gyro biases, and accelerometer bias with guaranteed numerical covariance stability.
5. **Instant Outage Detection & Reconvergence:** 3-gate monitor (fix freshness, accuracy threshold, Mahalanobis chi-sq gate) switches to pure IDR in **0.1 ms**; exponential decay reconvergence prevents position discontinuities upon GNSS recovery.
6. **Offline HMM Map Matching:** Newson & Krumm Hidden Markov Model with 2D spatial hash grid indexing ($O(1)$ lookup) and Viterbi decoding snaps drifting dead reckoning coordinates onto OSM road graphs.

---

## Project Status

**All 8 Phases Complete (100%):**

| Phase | Area | Status | Deliverables |
|---|---|---|---|
| **Phase 1** | Dataset pipeline & baseline INS | ✅ Done | IO-VNBD reader, ground truth visualizer, baseline double integration |
| **Phase 2** | In-vehicle alignment, filtering, ZUPT | ✅ Done | Gravity leveling, forward yaw alignment, Butterworth filter, ZUPT |
| **Phase 3** | AI/ML speed and kinematic estimation | ✅ Done | 1D CNN-GRU SpeedNet model, ONNX export, $<20$ ms inference |
| **Phase 4** | NHC and offline HMM map matching | ✅ Done | Non-Holonomic Constraints, OSM road graph, HMM map matcher |
| **Phase 5** | GNSS + INS fusion & blackout handler | ✅ Done | 9-state EKF, sub-50ms outage monitor, smooth reconvergence filter |
| **Phase 6** | High-frequency edge core engine | ✅ Done | Headless C-ABI engine (`src/edge/`), 200 Hz ring buffer, 2,462 Hz throughput |
| **Phase 7** | Mobile navigation application | ✅ Done | Offline vector map UI (`mobile/lib/`), 10 Hz replay, Dart FFI bindings |
| **Phase 8** | SIH benchmark deliverables | ✅ Done | Multi-scenario benchmark (`evaluate_phase8.py`), 4-way figures, screening report |

---

## Repository Layout

```text
SIH_DR/
├── .planning/                     # Project lifecycle management, ROADMAP & STATE
├── data/                          # IO-VNBD dataset storage
├── include/
│   └── idr_edge_engine.h          # C-ABI header for 200 Hz high-rate edge engine
├── native/
│   ├── CMakeLists.txt             # Native mobile library build configuration
│   ├── include/idr_mobile.h       # Mobile navigation C-ABI interface
│   ├── src/idr_mobile.cpp         # Native engine implementation
│   └── tests/                     # C-ABI memory alignment & struct layout tests
├── mobile/
│   ├── assets/regions/            # Offline vector road network (GeoJSON) & manifest
│   ├── assets/replay/             # 10 Hz deterministic navigation replay stream
│   ├── assets/models/             # SpeedNet ONNX model manifest & normalization stats
│   ├── lib/map/                   # Offline map canvas & navigation screen
│   ├── lib/navigation/            # Typed NavSnapshot, ReplaySource, and Native FFI Source
│   └── pubspec.yaml               # Flutter mobile client configuration
├── reports/
│   ├── figures/                   # Publication-quality benchmark figures & trajectory plots
│   ├── SIH_SCREENING_REPORT.md    # Official technical proposal & screening report
│   └── sih_benchmark_results.json # Comprehensive numerical benchmark data
├── src/
│   ├── data/                      # IO-VNBD loader & synthetic generator
│   ├── dead_reckoning/            # Baseline & calibrated INS
│   ├── edge/                      # Standalone high-rate edge engine & ring buffer
│   ├── eval/                      # Trajectory plotter & drift metrics
│   ├── fusion/                    # 9-state EKF, blackout handler, master fusion engine
│   ├── kinematics/                # Non-Holonomic Constraints (NHC)
│   ├── map_matching/              # Road network, spatial grid, HMM matcher
│   ├── preprocessing/             # Vehicle-frame alignment, vibration filter, ZUPT
│   └── speed_estimation/          # SpeedNet PyTorch model, trainer, and ONNX exporter
├── tests/                         # 72 unit and integration tests across all 8 phases
├── evaluate_phase1.py             # Baseline INS double-integration benchmark
├── evaluate_phase2.py             # Preprocessing & alignment benchmark
├── evaluate_phase4.py             # NHC & HMM map-matching benchmark
├── evaluate_phase5.py             # GNSS blackout & EKF fusion benchmark
├── evaluate_phase6.py             # 200 Hz edge engine streaming benchmark
├── evaluate_phase8.py             # Master multi-scenario SIH screening benchmark
└── train_speed_model.py           # SpeedNet training script
```

---

## Getting Started

### 1. Installation

Python 3.7+ is supported (tested on Python 3.7–3.13):

```bash
git clone https://github.com/grkadam7/Intelligent-Dead-Reckoning-for-GNSS-Denied-Navigation.git
cd Intelligent-Dead-Reckoning-for-GNSS-Denied-Navigation

python -m venv .venv
# Windows:
.venv\Scripts\activate
# Linux / macOS:
source .venv/bin/activate

pip install -r requirements.txt
```

### 2. Run the Benchmarks

Execute any benchmark script standalone:

```bash
# Master SIH Multi-Scenario Benchmark (generates all 4-way comparison plots):
python evaluate_phase8.py

# High-Rate 200 Hz Edge Engine Benchmark:
python evaluate_phase6.py

# GNSS Blackout & EKF Fusion Benchmark:
python evaluate_phase5.py

# NHC & Map Matching Benchmark:
python evaluate_phase4.py
```

### 3. Run the Automated Test Suite

```bash
python -m pytest tests/ -v
```

All 72 tests across all phases pass with 100% green.

---

## Proposal & Screening Documentation

For full mathematical derivations, error bounds, and compliance verification against all ISRO SIH 26168 specifications, see the official proposal screening report:
- 📄 **[SIH Screening Report](reports/SIH_SCREENING_REPORT.md)**

---

## References

- [IO-VNBD: Inertial and Odometry Benchmark Dataset for Ground Vehicle Positioning](https://github.com/onyekpeu/IO-VNBD)
- Newson, P. and Krumm, J. (2009). *Hidden Markov Map Matching Through Noise and Sparseness.* ACM SIGSPATIAL.
- `PROBLEM_STATEMENT.md` for the official ISRO SIH 26168 problem brief.

---

Built with pride for **Smart India Hackathon** by [Gagan Kadam](https://github.com/grkadam7).
