# 09-01 — Summary: bus-like outage keeps speed and position alive

## What shipped
- Injected clock + public `tick()` seam in `LiveSensorSource` (`clock` defaults to `DateTime.now`; the timer calls `tick()`), so navigation epochs can be fast-forwarded deterministically without changing production timing.
- ZUPT is gated on GNSS-confirmed sustained stop (`_stationaryFixCount >= 2` at <=0.7 m/s). Near-gravity accelerometer magnitude and low gyro rate alone no longer zero speed, which was the bus failure.
- On GNSS-outage entry, the last valid GNSS speed is carried into dead reckoning. DR speed is smoothed, non-negative and capped at 70 m/s.
- Uncertainty now grows with time and speed while blind (0.3 + 0.08·speed m/s, capped at 500 m) and returns to fix accuracy after reconvergence.
- Four deterministic test artifacts: manual clock, reusable bus trace fixture, DR-01 speed/position test, DR-02 uncertainty test, DR-03 mode-sequence test, plus a stationary-ZUPT regression case in the existing live-source tests.

## Verification
- `cd mobile; flutter analyze` — no errors.
- `cd mobile; flutter test` — 13/13 tests pass.
- Each focused test command was also run individually during RED→GREEN:
  - `flutter test test/dead_reckoning_speed_test.dart` (RED: speed collapsed to 0.00 m/s, then GREEN)
  - `flutter test test/dead_reckoning_uncertainty_test.dart` (RED: 2.5 → 3.4 m, then GREEN)
  - `flutter test test/dead_reckoning_mode_sequence_test.dart` (GREEN)
  - `flutter test test/live_sensor_source_test.dart` (GREEN, including stationary ZUPT)

## Decisions / notes
- Root cause matched the plan: ZUPT treated constant-speed straight driving as a stop.
- The min 20 s outage and ~14 s reconvergence are fast-forwarded via the manual clock; no multi-second sleeps.
- No native engine, map matching, background service or UI redesign touched (Phases 10–13).

## Status
Complete — DR-01, DR-02, DR-03 satisfied at unit level. Field confirmation on a bus is still required from the user after the APK rebuild.
