# Requirements: Intelligent Dead Reckoning System (IDR-INS)
*SIH Problem Statement ID: 26168 | ISRO*

**Defined:** 2026-09-10  
**Core Value:** Restrict positional drift to <10% of total distance during GNSS outages using low-cost smartphone MEMS IMUs without OBD-II feeds, ensuring uninterrupted lane-level navigation.

---

## v1 Requirements

### 1. In-Vehicle Alignment & Auto-Calibration (ALIGN)
- [ ] **ALIGN-01**: The system must automatically determine the 3D attitude (roll, pitch) of the device relative to Earth's gravity vector using accelerometer measurements during static or quasi-static periods.
- [ ] **ALIGN-02**: The system must estimate the device's horizontal azimuth/yaw relative to the vehicle's forward longitudinal axis by detecting the principal axis of acceleration and heading changes during forward motion.
- [ ] **ALIGN-03**: The system must continuously project raw 3-axis accelerometer and 3-axis gyroscope measurements into the Vehicle Coordinate Frame (Forward, Lateral, Up) regardless of arbitrary phone mounting angle or mount shifts.

### 2. Signal Preprocessing & Vibration Filtering (FILTER)
- [ ] **FILTER-01**: The system must apply digital band-stop/Butterworth/wavelet filters to suppress high-frequency chassis vibrations, engine harmonics (idling and RPM shifts), and transient pothole shocks.
- [ ] **FILTER-02**: The system must include a Zero Velocity Update (ZUPT) and Zero Angular Rate Update (ZARU) detector to recognize complete vehicle stops (traffic lights, stops) and reset accumulated drift.

### 3. AI/ML Speed & Kinematic Estimation (SPEED)
- [ ] **SPEED-01**: The system must implement a lightweight machine learning / deep learning model (e.g., 1D CNN / GRU / TCN) trained on ground vehicle kinematics to predict instantaneous forward speed ($v_x$) solely from windowed IMU inputs without OBD-II feeds.
- [ ] **SPEED-02**: The speed prediction model must run inference with latency $< 20$ ms on standard mobile processors (exported as ONNX / TFLite).
- [ ] **SPEED-03**: Forward speed prediction must achieve an RMSE error $< 1.5$ m/s compared against ground truth odometry on the IO-VNBD dataset test split.

### 4. Advanced Map-Matching & Kinematic Constraints (MAP)
- [ ] **MAP-01**: The system must apply Non-Holonomic Constraints (NHC) assuming zero lateral velocity ($v_y \approx 0$) and zero vertical velocity ($v_z \approx 0$) in the vehicle body frame during regular driving.
- [ ] **MAP-02**: The system must integrate an offline road network representation (from OpenStreetMap / GeoJSON road segments) for geometric route snapping.
- [ ] **MAP-03**: The system must employ a Hidden Markov Model (HMM) or geometric road-snapping algorithm to bind drifting dead-reckoned coordinates to the most likely topological road link.

### 5. GNSS+INS Fusion & Blackout Handler (FUSION)
- [ ] **FUSION-01**: The system must implement an Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF) combining GNSS position/velocity with IMU state propagation.
- [ ] **FUSION-02**: The system must detect GNSS outage / degradation in $\le 50$ milliseconds based on signal metrics (HDOP/PDOP, satellite count, covariance jumps) and instantly switch to pure Dead Reckoning (AI-Speed + INS + NHC + Map Matching).
- [ ] **FUSION-03**: Upon restoration of GNSS signal, the system must smoothly re-converge the estimated state back to GNSS without abrupt position jumps.
- [ ] **FUSION-04**: The system must maintain positional drift $< 10\%$ of distance traveled during simulated or real GNSS blackouts ($< 5$ m drift over 50 m in $< 1$ min; $< 100$ m drift over 1 km at 60 km/h).

### 6. Edge Core Software Engine (EDGE)
- [ ] **EDGE-01**: The core dead reckoning and fusion algorithms must be modularized into an edge-deployable library (C++ / Python) independent of mobile UI.
- [ ] **EDGE-02**: The edge engine must support high-rate IMU inputs up to 200 Hz for tactical / FOG (Fiber Optic Gyro) grade external IMUs.

### 7. Mobile Application & Real-Time Navigation UI (MOBILE)
- [ ] **MOBILE-01**: A mobile application displaying an offline map interface with real-time vehicle position cursor at $\ge 10$ Hz update rate.
- [ ] **MOBILE-02**: Visual indicators showing current navigation state: "GNSS Aided (Active)", "GNSS Outage (Dead Reckoning Mode)", and estimated confidence / drift bounds.
- [ ] **MOBILE-03**: In-app sensor calibration status showing phone orientation relative to vehicle.

### 8. Benchmark Evaluation & Verification (EVAL)
- [ ] **EVAL-01**: Automated evaluation scripts that process the IO-VNBD benchmark dataset (`https://github.com/onyekpeu/IO-VNBD`) across multiple vehicle runs and road types.
- [ ] **EVAL-02**: Generation of 2D/3D trajectory comparison plots (Ground Truth vs Pure GNSS vs Raw INS vs AI-Enhanced IDR) with quantitative cumulative error metrics.

---

## v2 Requirements (Deferred)
- **MULTI-01**: Multi-vehicle collaborative dead reckoning sharing map-matched road bias via V2X / local mesh.
- **ELEV-01**: Multi-story parking deck level determination using integrated barometric pressure sensor fusion.

---

## Out of Scope
| Feature | Reason |
|---------|--------|
| OBD-II / CAN Bus Hardware Cable | Explicitly excluded by problem statement; solution must work on standard smartphones and older vehicles without on-board vehicle diagnostic ports. |
| Cloud-dependent Live Map Streaming | Must navigate seamlessly in deep underground tunnels, forests, and remote valleys where cellular network is completely unavailable. |

---

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| EVAL-01, EVAL-02 | Phase 1 | Completed |
| ALIGN-01, ALIGN-02, ALIGN-03 | Phase 2 | Completed |
| FILTER-01, FILTER-02 | Phase 2 | Completed |
| SPEED-01, SPEED-02, SPEED-03 | Phase 3 | Completed |
| MAP-01, MAP-02, MAP-03 | Phase 4 | Completed |
| FUSION-01, FUSION-02, FUSION-03, FUSION-04 | Phase 5 | Pending |
| EDGE-01, EDGE-02 | Phase 6 | Pending |
| MOBILE-01, MOBILE-02, MOBILE-03 | Phase 7 | Pending |

**Coverage:**
- v1 requirements: 20 total
- Mapped to phases: 20
- Unmapped: 0

---
*Requirements defined: 2026-09-10*
