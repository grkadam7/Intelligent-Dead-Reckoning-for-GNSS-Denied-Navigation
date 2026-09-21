# Roadmap: Intelligent Dead Reckoning System (IDR-INS)
*SIH Problem Statement ID: 26168 | ISRO*

## Overview
This roadmap takes the project from the raw problem statement and IO-VNBD dataset ingestion through baseline mathematical dead reckoning, deep-learning forward velocity prediction, robust coordinate frame alignment, offline map-matching with kinematic constraints, GNSS-INS Kalman fusion, and dual deployment (high-rate 200 Hz edge engine and 10 Hz mobile app). The ultimate deliverable is a verified navigation system achieving $< 10\%$ drift during GNSS blackouts, complete with trajectory comparison plots for the SIH proposal evaluation.

## Phases

- [x] **Phase 1: Dataset Pipeline & Baseline INS Dead Reckoning** - Ingest IO-VNBD dataset, establish ground truth trajectory visualizer, and build baseline unassisted double-integration INS dead reckoning to measure raw drift.
- [x] **Phase 2: In-Vehicle Alignment & Signal Preprocessing** - Implement automatic gravity-based pitch/roll leveling, forward yaw axis alignment, Butterworth vibration filtering, and ZUPT/ZARU zero-velocity detectors.
- [x] **Phase 3: AI/ML Speed & Kinematic Estimation** - Build and train deep-learning models (1D CNN/GRU/TCN) to estimate vehicle forward speed from windowed IMU vibrations without OBD-II feeds, and export to ONNX.
- [x] **Phase 4: Non-Holonomic Constraints & Offline Map-Matching** - Implement Non-Holonomic Constraints (NHC) and an offline road-snapping / HMM map matcher using OpenStreetMap road link geometries.
- [ ] **Phase 5: GNSS+INS Fusion & Instant Blackout Handler** - Build Extended/Unscented Kalman Filter fusing GNSS + IMU with sub-50ms outage transition to AI-Dead Reckoning and smooth reconvergence upon GNSS recovery.
- [ ] **Phase 6: High-Frequency Edge Core Engine** - Package the complete IDR-INS core into an edge-deployable modular engine supporting up to 200 Hz for high-grade/FOG external IMUs.
- [ ] **Phase 7: Mobile Navigation Application** - Develop a cross-platform mobile navigation client (10 Hz) featuring offline map rendering, smooth vehicle tracking cursor, and real-time navigation status indicators.
- [ ] **Phase 8: SIH Benchmark Deliverables** - Generate quantitative benchmark metrics, trajectory plots (Ground Truth vs Raw vs IDR), and proposal artifacts demonstrating $<10\%$ drift for SIH screening.

---

## Phase Details

### Phase 1: Dataset Pipeline & Baseline INS Dead Reckoning
**Goal**: Download/parse IO-VNBD dataset, structure data pipeline, build ground truth evaluator, and implement baseline double-integration INS to demonstrate the raw exponential drift problem.  
**Depends on**: Nothing (first phase)  
**Requirements**: EVAL-01, EVAL-02  
**Success Criteria**:
1. IO-VNBD dataset reader successfully parses timestamps, 3-axis accelerometer, 3-axis gyroscope, magnetometer, GNSS coordinates, and ground truth odometry.
2. 2D/3D trajectory plotting script visualizes ground truth vehicle routes.
3. Baseline pure mathematical INS dead reckoning runs on sample runs and quantifies the baseline drift error.
**Plans**: 2 plans
- [x] 01-01: Dataset pipeline and preprocessing script for IO-VNBD benchmark runs.
- [x] 01-02: Ground truth visualizer and baseline unconstrained INS dead reckoning simulation.

### Phase 2: In-Vehicle Alignment & Signal Preprocessing
**Goal**: Solve smartphone mounting orientation variability and filter chassis vibration/potholes.  
**Depends on**: Phase 1  
**Requirements**: ALIGN-01, ALIGN-02, ALIGN-03, FILTER-01, FILTER-02  
**Success Criteria**:
1. Accelerometer gravity leveling accurately computes device pitch and roll angles relative to horizontal plane.
2. Dynamic heading alignment detects longitudinal vehicle axis during acceleration.
3. Butterworth / notch digital filter removes high-frequency engine harmonics and pothole shocks.
4. ZUPT (Zero Velocity Update) detects complete vehicle stops and zeroes out velocity accumulation.
**Plans**: 2 plans
- [x] 02-01: Auto-alignment calibration module rotating sensor frame to vehicle frame (Forward, Lateral, Up).
- [x] 02-02: Vibration band-stop filter and ZUPT/ZARU stop detector.

### Phase 3: AI/ML Speed & Kinematic Estimation
**Goal**: Overcome absence of OBD-II speedometer by training deep learning models to predict forward speed directly from IMU signals.  
**Depends on**: Phase 2  
**Requirements**: SPEED-01, SPEED-02, SPEED-03  
**Success Criteria**:
1. Machine learning architecture (1D CNN-GRU / TCN) trained on IO-VNBD inertial windows.
2. Forward speed prediction achieves RMSE $< 1.5$ m/s compared to ground truth vehicle speed.
3. Model exported to lightweight ONNX format with inference latency $< 20$ ms.
**Plans**: 2 plans
- [x] 03-01: Model architecture design, training loop, and hyperparameter tuning on IO-VNBD splits.
- [x] 03-02: ONNX model quantization, inference engine, and velocity evaluation.

### Phase 4: Non-Holonomic Constraints & Offline Map-Matching
**Goal**: Enforce vehicle kinematic boundaries and snap drifting coordinates onto offline road networks.  
**Depends on**: Phase 3  
**Requirements**: MAP-01, MAP-02, MAP-03  
**Success Criteria**:
1. Non-Holonomic Constraints (NHC) bound lateral and vertical velocities ($v_y \approx 0, v_z \approx 0$).
2. Offline road link parser loads OpenStreetMap (OSM) vector road geometries.
3. Hidden Markov Model (HMM) or spatial projection snaps inertial trajectory onto valid road links.
**Plans**: 2 plans
- [x] 04-01: Kinematic Non-Holonomic Constraints integration in trajectory update.
- [x] 04-02: Offline OSM road geometry parser and HMM map-matching filter.

### Phase 5: GNSS+INS Fusion & Instant Blackout Handler
**Goal**: Seamless state estimation fusing GNSS and IMU with instant transition to IDR during blackouts.  
**Depends on**: Phase 4  
**Requirements**: FUSION-01, FUSION-02, FUSION-03, FUSION-04  
**Success Criteria**:
1. Extended Kalman Filter (EKF) fuses GNSS position/velocity with IMU state during normal conditions.
2. Sub-50ms automatic detection of GNSS signal loss (loss of fix, DOP spike, covariance jump) triggering seamless Dead Reckoning mode.
3. Positional drift maintained $< 10\%$ of distance traveled in simulated GNSS outage segments ($< 5$ m over 50 m; $< 100$ m over 1 km).
4. Smooth reconvergence back to GNSS fix without position discontinuities.
**Plans**: 2 plans
- [ ] 05-01: EKF sensor fusion engine for combined GNSS+INS navigation state.
- [ ] 05-02: Outage detector, seamless state switcher, and smooth reconvergence filter.

### Phase 6: High-Frequency Edge Core Engine
**Goal**: Modularize the complete IDR-INS core engine for standalone deployment on edge hardware up to 200 Hz.  
**Depends on**: Phase 5  
**Requirements**: EDGE-01, EDGE-02  
**Success Criteria**:
1. Standalone, headless software engine with zero UI dependencies.
2. High-throughput processing loop handling up to 200 Hz IMU sensor feeds.
3. Clean C++/Python API for external FOG/tactical IMU streams.
**Plans**: 1 plan
- [ ] 06-01: Standalone edge software engine and high-rate streaming benchmark.

### Phase 7: Mobile Navigation Application
**Goal**: User-facing mobile navigation app with real-time map UI, smooth vehicle cursor, and offline capability.  
**Depends on**: Phase 5  
**Requirements**: MOBILE-01, MOBILE-02, MOBILE-03  
**Success Criteria**:
1. Mobile app UI displaying offline map view with 10 Hz position cursor update.
2. Real-time visual status chips: "GNSS Active", "GNSS Outage (Dead Reckoning)", and calibration indicator.
3. Tested on-device or simulated mobile environment.
**Plans**: 2 plans
- [ ] 07-01: Mobile UI framework setup and offline vector map rendering.
- [ ] 07-02: On-device sensor stream integration, ONNX inference integration, and navigation HUD.

### Phase 8: SIH Benchmark Evaluation & Screening Deliverables
**Goal**: Produce the required submission deliverables, comprehensive benchmark reports, and trajectory plots for ISRO/SIH screening.  
**Depends on**: Phase 5, Phase 6, Phase 7  
**Requirements**: EVAL-01, EVAL-02, FUSION-04  
**Success Criteria**:
1. Position trajectory plots clearly demonstrating Ground Truth vs Pure GNSS vs Raw INS vs AI-Enhanced IDR.
2. Comprehensive drift percentage table across various test scenarios (tunnels, urban canyons, forested highways).
3. Packaged proposal documentation and demo assets ready for SIH screening.
**Plans**: 1 plan
- [ ] 08-01: Comprehensive benchmark run, comparison plots generation, and SIH submission report package.

---

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Dataset Pipeline & Baseline INS | 2/2 | ✅ Complete | 2026-09-10 |
| 2. In-Vehicle Alignment & Preprocessing | 2/2 | ✅ Complete | 2026-09-10 |
| 3. AI/ML Speed & Kinematic Estimation | 2/2 | ✅ Complete | 2026-09-10 |
| 4. NHC & Offline Map-Matching | 2/2 | ✅ Complete | 2026-09-21 |
| 5. GNSS+INS Fusion & Blackout Handler | 0/2 | Not started | - |
| 6. Edge Core Engine | 0/1 | Not started | - |
| 7. Mobile Navigation Application | 0/2 | Not started | - |
| 8. SIH Benchmark Deliverables | 0/1 | Not started | - |
