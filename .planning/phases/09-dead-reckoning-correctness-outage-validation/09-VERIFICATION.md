# Phase 9 — Goal-backward verification

| Goal / must-have | Evidence | Result |
|------------------|----------|--------|
| Speed does not collapse at outage entry | `dead_reckoning_speed_test.dart`: speed stays > 90% of 13.9 m/s after entering DR | PASS |
| Position advances during a blackout | Same test: > 50 m displacement between first and last DR snapshot | PASS |
| Uncertainty grows monotonically while blind | `dead_reckoning_uncertainty_test.dart`: non-decreasing across all DR epochs, +>10 m over 20 s | PASS |
| Mode sequence DR → Recovering → GNSS Aided | `dead_reckoning_mode_sequence_test.dart` with manual clock | PASS |
| Fix age climbs during outage and resets on fix | Same test: > 19 s in DR, < 500 ms after recovery | PASS |
| Genuine stationary vehicle still ZUPTs | `live_sensor_source_test.dart` stationary case | PASS |
| Production timing unchanged | `clock` defaults to `DateTime.now`; timer calls `tick()` | PASS (code reviewed) |
| No scope creep into phases 10–13 | Files changed limited to DR engine and tests | PASS |

Automated suite: `flutter analyze` clean; `flutter test` 13/13.

Outstanding: physical bus re-test is a user-side UAT item. This verification covers deterministic unit evidence only; real sensor noise, phone mounting and actual tunnel conditions are not reproducible here.
