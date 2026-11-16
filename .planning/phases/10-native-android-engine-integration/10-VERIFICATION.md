# Phase 10 — Goal-backward verification

| Goal / must-have | Evidence | Result |
|------------------|----------|--------|
| Dart FFI structs match C ABI | `native_abi_contract_test.dart`: 56 / 64 / 120 bytes | PASS |
| APK contains native library for target ABIs | CI step `Verify Native Library In APK`: found for arm64-v8a, armeabi-v7a, x86_64 | PASS |
| Host C ABI tests pass | CI `test_abi` + `bus_outage_tests` (2/2 passed) | PASS |
| Native DR parity with Phase 9 | `bus_outage_tests`: speed >12.5 m/s at entry, >50 m advance, uncertainty +>10 m, age >19 s, recovery sequence | PASS |
| Live samples reach native C ABI | DeviceSensorFeed forwards IMU/GNSS structs with monotonic timestamps; CI APK builds | PASS (code + CI) |
| 10 Hz native snapshots | Native ticker started at 100 ms in NavigationScreen when available | PASS (code) |
| Fallback when library unavailable | `native_engine_selection_test.dart`: no events, no throw, status text | PASS |
| No scope creep into phases 11-13 | Changes limited to CI, native engine/tests, FFI feed, engine selection | PASS |

Local: `flutter analyze` clean, `flutter test test/` 15/15.
CI: runs 43860155088 (plan 1) and 43862019750 (plan 2) both green.

Outstanding: physical-device UAT. Sensor noise, mounting orientation and real tunnel behavior remain unverified outside deterministic traces.
