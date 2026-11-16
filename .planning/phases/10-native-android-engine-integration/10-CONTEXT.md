# Phase 10 — Context: Native Android Engine Integration

## Existing assets (verified in repo)
- `native/include/idr_mobile.h`: C ABI v1 with `idr_mobile_get_abi_version`, `create`, `destroy`, `push_imu`, `push_gnss`, `step`, `reset`; structs `IDRMobileIMUSample` (56 bytes), `IDRMobileGNSSSample` (64 bytes), `IDRMobileNavOutput`.
- `native/src/idr_mobile.cpp`: self-contained C++14 engine (only `<cmath> <cstring> <vector> <mutex> <chrono>`); no ONNX/AI dependency.
- `native/CMakeLists.txt`: already builds `libidr_mobile` as SHARED, plus optional host `test_abi` when `BUILD_TESTING` is on.
- `native/tests/test_abi.cpp`: host-side ABI size and lifecycle tests.
- `mobile/lib/navigation/native_source.dart`: complete Dart FFI struct definitions and symbol lookups, but `pushImu`/`pushGnss` wrappers were removed earlier because they passed raw doubles instead of struct pointers. It is never constructed in `main.dart`.
- `.github/workflows/build_apk.yml`: on every push it **deletes `mobile/android` and regenerates it with `flutter create`**, injects manifest permissions, then `flutter build apk --release`. Any Gradle/NDK change committed under `mobile/android` would be erased on CI.

## Gaps this phase must close
- No Android NDK/CMake build produces `libidr_mobile.so`; the APK contains no native engine.
- Dart FFI wrappers for pushing IMU/GNSS samples are missing, so the C ABI cannot be driven from the app.
- `DeviceSensorFeed` currently feeds only the Dart `LiveSensorSource`; no engine selection or fallback exists.

## Native engine correctness risks (must not regress Phase 9)
- `idr_mobile_push_gnss()` sets speed/heading/accuracy on a valid fix but never resets ENU position from the GNSS coordinates — position is pure IMU integration from a hardcoded Coventry origin (52.40166, -1.50529). That would drift even in GNSS-aided mode.
- Blackout is entered only when an explicit `is_valid = 0` GNSS sample is pushed. A real tunnel (fixes simply stop) would not trigger Dead Reckoning unless the Dart feed synthesizes invalid samples.
- Uncertainty growth during blackout is 0.05 m/s, the same near-constant behavior Phase 9 just fixed in Dart.
- Therefore NENG-02 is not complete unless the native path satisfies the DR-01..03 bus-like trace, or is explicitly gated behind the Dart engine with a documented fallback.

## Locked decisions / constraints
- Android first; no iOS in this phase (Phase 13 UI, later milestone for iOS).
- Keep the existing regenerated-Android CI strategy; inject NDK/CMake configuration into the generated `app/build.gradle` inside the workflow instead of committing Gradle files that get deleted.
- Pin NDK and CMake versions in CI; verify the APK actually contains `libidr_mobile.so` for each ABI.
- Default behavior must remain safe: if the library cannot load or a symbol is missing, the app continues on the Dart engine and diagnostics state which engine is active.
- Keep the C ABI stable (version 1). Do not change struct layouts without updating both C tests and Dart `sizeOf` assertions.
- No local Android SDK, g++ or CMake on this machine. Local verification is limited to Dart `sizeOf` and `flutter test/analyze`; host C ABI tests, the NDK build and APK inspection run in GitHub Actions (ubuntu-latest), which is the authoritative APK check.

## Scope fences
IN: NDK/CMake build in CI, APK packaging verification, Dart FFI push wrappers + ABI checks, native-path DR parity, engine selection with fallback.
OUT: map matching and local map downloads (Phase 11), foreground service and trace export (Phase 12), UI redesign (Phase 13), ONNX/AI model integration, iOS.

## Verification intent
Local verification is limited to Dart `sizeOf` ABI assertions and `flutter test/analyze` — this machine has no g++, cmake or Android SDK. The host C ABI tests, NDK/CMake build and APK native-library inspection therefore run in GitHub Actions (ubuntu-latest). A deterministic bus-like C ABI test asserts speed continuity, position advance, monotonic uncertainty and mode sequence on the native path.
