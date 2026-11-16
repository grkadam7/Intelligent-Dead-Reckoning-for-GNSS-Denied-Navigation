# 10-02 — Summary: feed the native engine and keep the Dart fallback

## What shipped
- Native C engine parity (`native/src/idr_mobile.cpp`): valid GNSS fixes establish/snap the ENU position instead of leaving pure IMU drift; blackout is detected from stale GNSS as well as invalid samples; last GNSS speed is carried into dead reckoning; uncertainty grows with time and speed (0.3 + 0.08·speed m/s, capped 500 m); real fix age is published; recovering settles to GNSS aided after 3 s.
- `native/tests/test_bus_outage.cpp` + CMake registration: deterministic bus-like C ABI test covering speed continuity, position advance, monotonic uncertainty, fix age and the recovery sequence.
- `DeviceSensorFeed` forwards IMU/GNSS structs to the native engine with monotonic timestamps and emits explicit invalid fixes during GNSS gaps.
- `NavigationScreen` selects the native stream when `isAvailable`, starts its 100 ms ticker, stops it on replay switch/teardown, and shows the active engine in diagnostics. `main.dart` constructs the native source and passes it through.
- `mobile/test/native_engine_selection_test.dart`: unavailable library publishes nothing, throws nothing, and is safe to push samples into.

## Verification
- Local: `flutter analyze` clean; `flutter test test/` 15/15 pass.
- CI run 43862019750: green; `bus_outage_tests` passed (`2/2 Test #2: bus_outage_tests Passed`) and APK library checks found all three ABIs.
- Artifact: IDR-Navigation-Release-APK from run 43862019750.

## Deviations / notes
- Screen teardown calls `nativeSource.stop()` rather than `dispose()`; the source is owned by `main()` and may be reused if the screen is recreated.
- Field UAT on a bus is still required; deterministic C/Dart tests cannot reproduce phone-specific sensor noise.
