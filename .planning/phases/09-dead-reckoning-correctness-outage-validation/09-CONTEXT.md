# Phase 9 — Context: Dead Reckoning Correctness & Outage Validation

## Source evidence
Bus drive recording, 2026-11-16 (about 53 s, frames sampled every 5 s). The app was in Live mode with Hubli-Dharwad roads rendered.

- GNSS-aided portion: 48–50 km/h, horizontal uncertainty ±2.4–2.5 m, epoch age 2–128 ms, vehicle cursor and map following correctly.
- Diagnostics sheet at one sample: `Sequence #458`, `Epoch Age 128 ms`, mode reason `GNSS fix nominal (Accuracy: 12.3 m)`, uncertainty 12.27 m.
- A 20 s simulated GNSS tunnel blackout was triggered from Diagnostics. Mode sequence green → orange (Dead Reckoning) → blue (Recovering GNSS) → green worked, and recorded fix age climbed to ~20 s, so the outage switch itself fires.
- **Critical defect:** at blackout entry the displayed speed collapsed from 50 km/h to 1 km/h and the vehicle cursor stopped advancing. Horizontal uncertainty stayed around ±2.7–3.2 m for the whole outage instead of growing.

## Root-cause hypothesis (to confirm, not an accepted fact)
`LiveSensorSource._processInertialKinematics()` treats `|accelerometer| ≈ 9.80665 ± 0.2 m/s²` plus `|gyro| < 0.03 rad/s` as "vehicle stationary" (ZUPT) and zeroes `_speedMps`. A bus at constant speed on a straight, smooth road has almost no specific force and little yaw rate, so it satisfies the same test. The detector cannot distinguish stationary from constant-velocity motion from IMU alone. GNSS speed (available before the outage) or a wheel/OBD feed is required; OBD-II is forbidden by the problem statement.

## Decisions locked for this phase
- Fix the shared Dart `LiveSensorSource` (the engine actually used by the app in this milestone); do not wire the native C engine here (Phase 10).
- ZUPT may only zero speed when the vehicle was already stationary before the outage (GNSS speed below walking threshold for a sustained window) and the IMU stillness signature still holds.
- On outage entry, carry the last valid GNSS speed and heading into dead reckoning; propagate from IMU without resetting speed to zero due to stillness heuristics.
- Uncertainty must grow monotonically with time/distance since the last fix and reset smoothly on reconvergence.
- The existing simulated-outage control stays and becomes the repeatable validation harness; fix age must remain real (it already reports ms since last fix).
- Keep the fix offline, on-device, and within the existing NavSnapshot contract.
- **Testability seam:** the engine currently calls `DateTime.now()` directly and advances epochs with a real `Timer`. The ~14 s reconvergence and 20 s fix-age growth cannot be tested quickly without a seam. Add an injectable clock and a public single-epoch `tick()` that the timer calls; default behavior stays identical. Tests use a manual clock and `tick()` to fast-forward deterministically.

## Scope fences
IN: speed continuity, ZUPT gating, uncertainty growth, deterministic tests driven by a bus-like sensor trace, outage state sequence assertions, engine test seam.
OUT: native C engine / NDK build (Phase 10), live map matching and local map downloads (Phase 11), foreground service and trace export (Phase 12), visual redesign (Phase 13), OBD-II, cloud processing, iOS.

## Verification intent
Separate fast deterministic tests for speed/position (DR-01), uncertainty growth (DR-02), and mode sequence (DR-03), plus a focused stationary-ZUPT regression case. All use the injectable clock and manual `tick()`; no multi-second wall-clock sleeps. Every verification command names exactly one `flutter test <path>` invocation so regressions cannot be silently skipped.
