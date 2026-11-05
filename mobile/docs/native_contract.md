# Phase 7: Mobile Native Navigation Contract & Reference Specification

**Version:** 1.0.0  
**Phase:** 07-mobile-navigation-application (Plan 02)  
**Status:** Frozen Reference Specification  

## 1. Overview & Purpose

This document freezes the mathematical, architectural, and data interface contract between the mobile UI / sensor pipeline and the shared native numerical navigation engine (`libidr_mobile`).

It guarantees:
- Parity between Python algorithm prototypes (Phases 1–6) and the compiled native C/C++ engine.
- Strict timing boundaries ($\ge 10$ Hz snapshot publication, $<100$ ms sensor-to-state latency, $<20$ ms model execution).
- Deterministic error handling without crashing or undefined memory behavior across the C-ABI boundary.

---

## 2. Coordinate Frames & Transformations

1. **Body / Phone Sensor Frame ($X_b, Y_b, Z_b$):**
   - Unconstrained 3D orientation of the smartphone mounted in the vehicle cradle.
   - Accelerometer: $a = [a_x, a_y, a_z]^T$ in $\text{m/s}^2$ (including Earth gravity vector $\approx 9.80665\,\text{m/s}^2$).
   - Gyroscope: $\omega = [\omega_x, \omega_y, \omega_z]^T$ in $\text{rad/s}$.

2. **Vehicle Coordinate Frame ($X_v, Y_v, Z_v$):**
   - $X_v$ = Forward (longitudinal), $Y_v$ = Lateral (right), $Z_v$ = Upward.
   - Rotational alignment:
     - **Attitude leveling (Roll $\phi$, Pitch $\theta$):** Derived from low-pass filtered gravity vector during static / quasi-static periods.
     - **Heading alignment (Yaw $\psi$):** Principal axis of vehicle forward acceleration.

3. **Local Navigation Frame (East-North-Up - ENU):**
   - Cartesian tangent plane centered at fixed origin geodetic datum $(\text{lat}_0, \text{lon}_0, \text{alt}_0)$.
   - All EKF state propagation, dead reckoning, and kinematic constraints operate in ENU metric space.

4. **Geodetic Display Coordinates (WGS84):**
   - Converted from ENU only at the application boundary for vector map rendering.
   - Azimuth heading: Clockwise angle from True North ($0^\circ = \text{North}, 90^\circ = \text{East}$).

---

## 3. Kinematic Estimation & AI Model Contract

1. **Model Specification:**
   - Architecture: Lightweight 1D CNN + GRU SpeedNet.
   - Input shape: `(batch=1, channels=6, time=50)` float32.
   - Channel ordering: `[a_x, a_y, a_z, \omega_x, \omega_y, \omega_z]`.
   - Normalization: $z = \frac{x - \mu}{\sigma}$, where $\mu$ and $\sigma$ are frozen channel statistics.
   - Output: instantaneous forward velocity $v_x \in [0, 45]\,\text{m/s}$ (float32).
   - Inference latency target: $<20$ ms on ARM64 mobile CPUs.

2. **Non-Holonomic Constraints (NHC):**
   - Enforces physical vehicle boundary conditions:
     $$v_{\text{lateral}} \approx 0 \pm 0.1\,\text{m/s}, \quad v_{\text{vertical}} \approx 0 \pm 0.1\,\text{m/s}$$

3. **Zero Velocity Updates (ZUPT / ZARU):**
   - Detected when filtered acceleration variance $\sigma_a^2 < 0.05\,\text{m}^2/\text{s}^4$ and angular rate norm $\|\omega\| < 0.03\,\text{rad/s}$ over $\ge 0.5$ s window.
   - Immediately resets forward velocity accumulation to $0.0\,\text{m/s}$.

---

## 4. EKF State Vector & Fusion Mechanics

1. **State Vector (9 states):**
   $$x = \begin{bmatrix} p_e & p_n & v_e & v_n & \psi & b_{\omega x} & b_{\omega y} & b_{\omega z} & b_{ax} \end{bmatrix}^T$$
   - $p_e, p_n$: East and North positions (m).
   - $v_e, v_n$: East and North velocities (m/s).
   - $\psi$: Heading angle (rad).
   - $b_\omega$: Gyroscope bias vector (rad/s).
   - $b_{ax}$: Longitudinal accelerometer bias ($\text{m/s}^2$).

2. **Joseph-Form Covariance Update:**
   $$P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T$$
   Guarantees positive semi-definiteness under floating-point roundoff.

3. **GNSS Health Monitor & Outage Transition:**
   - Three-gate integrity check:
     - Fix arrival interval $\Delta t \le 1.5$ s.
     - Horizontal accuracy $\le 15.0$ m.
     - Mahalanobis chi-squared residual distance gate ($\chi^2 \le 16.27$ for 3 DOF at $\alpha=0.001$).
   - Instant transition to **Dead Reckoning Mode** in $\le 50$ ms upon signal loss or gate failure.

4. **Smooth Reconvergence Filter:**
   - Upon GNSS restoration, eliminates abrupt coordinate jumps using exponential decay residual offset:
     $$\Delta x(t) = \Delta x_0 \cdot \exp\left(-\frac{\ln 2}{\tau} \cdot t\right), \quad \tau = 2.5\,\text{s}$$

---

## 5. Frozen Reference Tolerances

When comparing the compiled native engine against the reviewed Python oracle:
- Normalized IMU window values: Absolute tolerance $\le 1 \times 10^{-5}$.
- Model forward speed prediction: Absolute tolerance $\le 1 \times 10^{-4}\,\text{m/s}$.
- EKF position state $(p_e, p_n)$: Absolute tolerance $\le 1 \times 10^{-4}$ m.
- EKF velocity state $(v_e, v_n)$: Absolute tolerance $\le 1 \times 10^{-4}$ m/s.
- Covariance matrix elements: Relative tolerance $\le 1 \times 10^{-4}$.
- Mode transitions and state flags: Exact match (`gnss_aided` $\leftrightarrow$ `dead_reckoning` $\leftrightarrow$ `recovering`).
