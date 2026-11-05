# Phase 7 Plan 02: Mobile Live Native Navigation, ONNX Contract & Parity Summary

**Execution Date:** 2026-11-06  
**Phase:** 07-mobile-navigation-application  
**Plan:** 02  
**Status:** Completed  

## Deliverables & Achievements

1. **Native Navigation Core C-ABI (`native/include/idr_mobile.h`, `native/src/idr_mobile.cpp`):**
   - Versioned `IDR_MOBILE_ABI_VERSION = 1` interface.
   - 64-bit aligned structs: `IDRMobileIMUSample` (56 bytes), `IDRMobileGNSSSample` (64 bytes), `IDRMobileNavOutput` (120 bytes).
   - Thread-safe lifecycle functions: `idr_mobile_create`, `idr_mobile_destroy`, `idr_mobile_push_imu`, `idr_mobile_push_gnss`, `idr_mobile_step`, and `idr_mobile_reset`.
   - CMake configuration (`native/CMakeLists.txt`) for Android NDK / mobile shared library compilation.

2. **Reviewed Reference Contract & Oracle (`mobile/docs/native_contract.md`, `native/tests/fixtures/reviewed.json`):**
   - Frozen coordinate transformations (Body $\to$ Vehicle $\to$ ENU $\to$ WGS84).
   - Strict numerical tolerances: position $\le 0.0001$ m, velocity $\le 0.0001$ m/s, heading $\le 0.05^\circ$.
   - `export_reference.py` captures 20 reference epochs spanning nominal fusion, sub-50ms outage detection, dead reckoning, and smooth reconvergence.

3. **ONNX Model & Normalization Inventory (`mobile/assets/models/manifest.json`):**
   - Formally documented 1D CNN + GRU SpeedNet specifications.
   - Input shape `[1, 6, 50]` float32 at 10 Hz; frozen channel means ($\mu$) and standard deviations ($\sigma$).
   - Latency target: $<20$ ms on ARM64 mobile hardware.

4. **Dart FFI Mobile Adapter (`mobile/lib/navigation/native_source.dart`):**
   - Direct FFI bindings linking to `libidr_mobile.so` / `idr_mobile.dll`.
   - Translates low-level C structs into typed immutable `NavSnapshot` streams without UI thread blocking.
   - Includes graceful host fallback and error handling.

5. **Test Suite Verification (`tests/`, 19 passing tests):**
   - `test_phase6.py` (13 tests): 200 Hz edge engine, ring buffers, blackout transitions.
   - `test_phase7_assets.py` (1 test): Offline map manifest, checksums, GeoJSON road network, and replay stream.
   - `test_phase7_reference.py` (2 tests): Reference oracle structure, model manifest parameters, and mode transitions.
   - `test_mobile_abi_ctypes.py` (3 tests): 64-bit struct layouts, byte offsets, and padding verification.
