# 10-01 — Summary: build, package and bind the native engine

## What shipped
- `.github/workflows/build_apk.yml`: after `flutter create` regenerates Android, it installs pinned NDK 27.2.12479018 + CMake 3.22.1 with licenses accepted, structurally injects `ndkVersion`, `externalNativeBuild.cmake` (path to `native/CMakeLists.txt`, version pin) and `abiFilters` into the generated `app/build.gradle.kts`, runs host C ABI tests, builds the APK, and hard-fails if `libidr_mobile.so` is missing from arm64-v8a / armeabi-v7a / x86_64.
- `mobile/test/native_abi_contract_test.dart`: locks Dart FFI struct sizes to C (IMU 56, GNSS 64, output 120 bytes).
- `mobile/lib/navigation/native_source.dart`: ABI-version check at init, status string, and typed `pushImu`/`pushGnss` wrappers that allocate and free FFI structs.

## Verification
- Local: `flutter analyze` clean, `flutter test test/` green including the new ABI contract test.
- CI run 43860155088: green; `test_abi` passed and APK contained libidr_mobile.so for all three ABIs.

## Notes
- Flutter 3.47 generates `build.gradle.kts`, so the injection handles both `.kts` and legacy `.gradle` templates with explicit anchor checks.
- Watch item: the pinned CMake version must be re-checked if a future Flutter/AGP template changes its CMake default.
