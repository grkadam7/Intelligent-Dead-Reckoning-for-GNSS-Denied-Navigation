---
phase: 07
slug: mobile-navigation-application
status: proposed-unrun
nyquist_compliant: false
wave_0_complete: false
created: 2026-10-31
---

# Phase 7 validation strategy

**Planning only. No automated GSD hooks, pytest, Flutter, CMake, native ABI or device tests ran.** The parent's `python -m pytest tests/test_phase6.py -q` attempt could not launch the terminal because of `libasound.so.2`; this is neither a test failure nor a test pass. Do not retry via terminal in this planning task. This matrix is proposed verification, not completed evidence.

**Phase 6 carry-forward:** execution summary absent; STATE records 6.92% versus 06-01's **<1.5%** drift criterion. Preserve this independent unresolved gate. The project's/mobile **<10%** requirement does not waive it. Roadmap complete markers do not establish verified prerequisites. Planning and replay may proceed per explicit user authorization, not a Phase 6 pass.

## Prerequisites and test creation (future Wave 0)

- Obtain explicit stack/design/device/region decisions at 01-D0/D1 and native-port approval at 02-D1. Flutter/MapLibre/shared C++/ONNX is provisional. No framework/plugin version or target device is approved yet.
- Restore a working execution environment. Discover actual Python/Flutter/CMake/NDK versions; pin compatible dependencies only after the feasibility spike. iOS builds and signing require macOS/Xcode and an actual iOS device for sensor/performance evidence. Windows or simulator results cannot satisfy that gate.
- Create the exact proposed tests below before implementation. For logic, record a genuine RED failure, implement GREEN, then refactor and rerun. Missing toolchains/assets make the test BLOCKED, not skipped-green. Tests loading real artifacts fail closed if they are absent; do not substitute canned speed or zero-valued covariance.
- No binary model, normalization file or map package was found. Validate provenance/license, checksum, schema, sample rate/frame/channel/time contracts, coverage and storage before inference/map tests. Reproducible recovery/training/export may be a prerequisite work package requiring approval; it is not silently included as an already-working dependency.
- Native CMake must register CTest names `abi`, `tracer`, `reference_parity`, `calibration`, `matching`, `health` to corresponding proposed `native/tests/test_*.cpp`. Tests and fixture loading must be platform-portable. Build examples below use multi-config-compatible `--config Debug` / `-C Debug`.

## Task-to-test map

All paths are repository-relative **PROPOSED future files**. Every command below is **FUTURE / UNRUN**. `mobile` means run with that working directory, not a guessed shell `cd`. Integration commands require one selected approved connected target; record its actual identity at execution and repeat on the other platform, never invent a device ID. The probe must run on Android and iOS separately.

| Task | Proposed tests / command | Evidence and stop condition |
|---|---|---|
| 01-D0/D1 | Human decision, no automated substitute | Explicit spike then architecture/UI approval; absent choice blocks implementation |
| 01-S1 | mobile: `flutter test integration_test/toolchain_probe_test.dart` | Local map and real compiled symbol/ORT native load on each platform; iOS unavailable = blocked |
| 01-T1 | mobile: `flutter test test/replay_source_test.dart`; `flutter test integration_test/offline_replay_test.dart` | Deterministic timeline, distinct >=10Hz replay states, offline cold start, visible Simulation label |
| 01-E1 | root: `python -m pytest tests/test_phase7_assets.py -q`; mobile: `flutter test test/offline_region_store_test.dart test/navigation_screen_test.dart`; repeat offline integration test | Corrupt/missing/storage/coverage handling and accessible HUD, no remote fallback |
| 02-R1 | root: `python -m pytest tests/test_phase7_reference.py tests/test_phase7_assets.py -q`; `python tools/phase7/export_reference.py --output native/tests/fixtures/reviewed.json` | Reviewed oracle, frozen tolerances and real model contract; no unreviewed golden output |
| 02-D1 | Human decision | Approve costly port/model strategy, or stop |
| 02-T1 | root: `cmake -S native -B build/phase7 -DBUILD_TESTING=ON`; `cmake --build build/phase7 --config Debug`; `ctest --test-dir build/phase7 -C Debug -R abi --output-on-failure`; mobile: `flutter test test/native_snapshot_test.dart` | Compiled symbol/layout/ownership/reset tests; repeat link/layout on mobile architectures |
| 02-T2 | root: build as above; `ctest --test-dir build/phase7 -C Debug -R "tracer|reference_parity" --output-on-failure` | Real ONNX/operator execution plus reviewed preprocessing/EKF state/covariance parity |
| 02-T3/C1 | mobile: `flutter test test/sensor_contract_test.dart`; `flutter test integration_test/live_navigation_test.dart`; human tracer review | Real units/axes/clocks, bounded worker and live HUD, before expansion |
| 02-E1 | root: build; `ctest --test-dir build/phase7 -C Debug -R calibration --output-on-failure`; mobile: `flutter test test/calibration_panel_test.dart` | Actual leveling/forward alignment/mount shift, covariance and unavailable-state tests |
| 02-E2 | root: build; `ctest --test-dir build/phase7 -C Debug -R "matching|reference_parity" --output-on-failure`; `python -m pytest tests/test_phase7_assets.py -q` | Independent connected graph, full intermediate parity and unmatched states |
| 02-E3 | root: build; `ctest --test-dir build/phase7 -C Debug -R "health|abi|calibration" --output-on-failure`; mobile: live integration test | Invalid versus silent/sparse fixes, recovery continuity, queue/lifecycle/permission faults |
| 02-A1 | root: `python -m pytest tests/test_phase7_metrics.py -q`; `python tools/phase7/evaluate_mobile.py`; full CTest; mobile: `flutter analyze`; `flutter test`; `flutter test integration_test/offline_replay_test.dart integration_test/live_navigation_test.dart` | Device capture/accuracy/timing report, no UI-only acceptance |

`build` in the table means the exact `cmake --build build/phase7 --config Debug` command. `full CTest` means `ctest --test-dir build/phase7 -C Debug --output-on-failure`. No command in this document is claimed to have worked already.

## Measurement contract and acceptance gates

### 1. Offline and artifacts

Cold launch with network blocked and ambient map cache cleared, but intentionally bundled packages intact. Render local tiles/style/glyphs/fonts/sprites/attribution and run local model/graph with no Python server or first-travel network fetch. Record network activity to detect hidden remote dependencies. Validate manifest provenance, license/redistribution rights, hashes, format/schema, bbox, zooms, graph version/origin, package size and storage checks. Test absent/corrupt/incompatible assets and out-of-region operation. No public OSM tile bulk download/prefetch; use self-produced tiles or an offline/redistribution-permitted provider, with OSM attribution/ODbL obligations reviewed. A paid provider/key decision is not pre-approved.

### 2. Native contracts and parity

- Existing `include/idr_edge_engine.h` has no corresponding implemented native exports. Resolve frame comments and reset return mismatch explicitly in the new versioned mobile ABI; don't change staged legacy work.
- Verify compiled symbol calls, fixed-width layouts/offsets against Dart/native adapters, API version/size negotiation, handle validity, resource ownership, errors, concurrency, destroy/reset and session generation. Test reset equivalence to a new engine, including every queue/window/calibration/health/matcher state.
- Review Python oracle corrections independently: causal versus acausal filter behavior, repeated/stale fixes, asynchronous GNSS grace, dt gaps and reconvergence sign/continuity. A bug-for-bug port is not success. Keep reviewed corrections in new fixture tooling unless legacy-source changes are separately authorized.
- At 02-R1 freeze fixture-specific tolerances before native comparison; proposed initial values: normalized windows abs 1e-5; model speed abs 1e-4m/s; float64 state/covariance abs 1e-5 in declared SI units plus relative 1e-5; modes/validity/graph IDs exact with reviewed tie rules. Native discrepancies cannot be fixed by loosening gates after the fact.
- Model manifest must establish real input rate, tensor dimensions/order, frame/gravity/units, normalization, output m/s and effective timestamp, operator/runtime versions, training/held-out splits, licensing and hashes. Current code defaults [1,6,50], channel-first ax/ay/az/yaw/pitch/roll rates, center-window label. Do not assume the unknown sample rate or trailing-window time semantics. At low rates the center-window lag may exceed the age budget; approve causal retraining or delayed-observation handling with current-time propagation before porting.

### 3. Timing: measure different clocks and stages, not animation

Use normalized monotonic clocks with measured cross-domain uncertainty; retain native capture and receipt times. Trace sensor capture → queue → preprocessing/window → ONNX run → fusion state → publication → rendered frame, plus health callback/deadline → decision. Include sequence/session IDs. Report requested and delivered sensor rates, queue capacity/depth/overflow and rejected/disordered samples.

| Metric | Proposed acceptance protocol |
|---|---|
| Genuine publication | >=10Hz distinct advancing states; >=600 unique publications in each steady-state 60s window, no publication gap >100ms after declared warm-up. A repeated snapshot/new animation frame does not count. Use continuous 15min device capture for sustained evidence. |
| Current sensor-to-state age | <=100ms from latest contributing valid IMU capture to publication; retain propagated state time, model effective observation age and clock uncertainty. Never redate an old observation. UI presentation delay measured separately. |
| Model inference | <20ms per measured ONNX execution on each declared device under sustained load; report p50/p95/p99/max, including any violations. Separate cold model load and first-window fill, report both rather than hiding them. Quantization needs renewed parity/accuracy. |
| Health reaction | <=50ms from available explicit-invalid/degraded callback to core mode decision; monitor faster than 10Hz UI. Report timeout-expiry-to-decision separately. |
| Physical signal loss | Independently measured onset → evidence → decision where observable. Silence detection includes the configured freshness deadline/OS callback delay and cannot truthfully be called <=50ms merely because health evaluation is fast. Sparse GNSS may make inherited FUSION-02 physical latency unproven; record the gap without redefining that requirement. |
| Rendering | Smooth cursor follows timestamped states; frame rate/render latency is not estimator publication frequency. Respect reduced motion. |

Warm-up is explicitly not navigating until required calibration/origin/model window is ready. Queue overflow, suspension and sensor gaps must invalidate freshness, not continue a valid-state performance claim. Collect thermal, power and drops over >=15min on each named physical device/OS; this does not imply background operation support.

### 4. Modes, calibration and uncertainty

Test initializing/no origin, waiting for calibration/model, GNSS aided, outage dead reckoning, recovering, stale/suspended and error. Explicit invalid fixes, degraded metrics, missing optional metrics, sparse healthy updates, stale/duplicate fixes and true silence are distinct. Recovery starts continuously and converges within reviewed duration/tolerance; no hidden teleport by switching display coordinates.

Calibration uses real stillness and motion qualification, not elapsed-time completion. Check roll/pitch/yaw in controlled poses, forward-axis sign ambiguity, insufficient excitation, turns, remount and recalibration. Android accelerometer m/s² including gravity and iOS acceleration g require validated signs/axes and gravity convention, gyro rad/s; test both platforms. Convert GNSS WGS84 to common ENU and back at boundaries with known-coordinate/heading tests. Clock steps/resume invalidate old mappings. Permission denial/revocation, missing sensors, pause/resume/screen-off and reset must produce honest not-ready/stale states.

Display estimated horizontal 95% uncertainty only from valid symmetric PSD ENU covariance, with stated filter/Gaussian assumptions (ellipse scaling 5.991 for 2DoF). Report empirical held-out coverage; do not present it as guaranteed accuracy. Invalid/unavailable covariance → **Uncertainty unavailable**. Cross-track error and alignment quality are not navigation confidence. Road snapping/display smoothing cannot silently shrink uncertainty. No lane-level claim follows from matching or <10% drift.

### 5. Held-out accuracy and evidence separation

Use independent held-out IO-VNBD drives and physical-device recordings with independently obtained ground truth. Split by run, never overlapping train/test windows. No truth/GNSS speed supplied to inference/fusion during blackout; no truth-derived matching graph. Record model/norm/graph checksums, route, mounting, sample rates, window contract, blackout start/end and ground-truth synchronization.

Primary drift ratio: `100 * horizontal endpoint position error / ground-truth distance traveled within blackout`. Require **<10% for each declared acceptance blackout**, plus **<5m over 50m** and **<100m over 1km** cases. Report raw and matched trajectories, endpoint/peak horizontal errors, duration, distance definition and run distribution. Near-zero-distance ratios are undefined; report absolute stationary error rather than divide by zero. Include held-out model RMSE **<1.5m/s** when validating inherited SPEED-03. Do not use a synthetic test or UI simulation as physical accuracy evidence. Do not relax Phase 6 **<1.5%**.

Proposed `mobile/integration_test/live_navigation_test.dart` capture export creates `reports/phase7/capture_manifest.json` identifying real capture files, independent ground truth, devices and hashes. Proposed `tools/phase7/evaluate_mobile.py` reads that fixed manifest; missing evidence fails closed. Proposed output: `reports/phase7/device_results.json` and `reports/phase7/acceptance.md`. Metrics logic is tested by `tests/test_phase7_metrics.py` before those reports are trusted. No raw personal routes uploaded; use consented/local data and document retention. Safe passenger/test-rig operation only.

## Manual UI/accessibility acceptance

Follow proposed 07-UI-SPEC.md: Simulation versus Live is persistent; mode uses text and icon, not color alone; actual calibration and unavailable values are distinguishable. Verify TalkBack/VoiceOver, 200% text, minimum 48dp targets, 4.5:1 text / 3:1 non-text contrast, safe areas, portrait/landscape and reduced motion. No 10Hz screen-reader announcements. Missing/corrupt region offers import/manage action, no online fallback; retain textual status outside map coverage.

## Completion ledger (all pending)

| Evidence layer | Current state | Can establish |
|---|---|---|
| Static planning/research | Inspected only | Gaps/contracts, not runtime correctness |
| Replay/simulator | NOT RUN | Offline UI/determinism and simulated state presentation only |
| Reviewed Python/native host | NOT RUN | Numerical/ABI reference behavior, not physical sensor timing |
| Android physical | BLOCKED pending choices/toolchain/assets | Real sensors, offline behavior, device-specific timing/accuracy |
| iOS physical | BLOCKED pending choices/Mac/toolchain/assets | Same on iOS; Android evidence cannot substitute |
| Phase 6 verification | OUTSTANDING | Separate <1.5% criterion and missing-summary resolution |

Before considering Phase 7 complete, every required mobile gate has recorded PASS/FAIL/BLOCKED with raw evidence, no blocked gate represented as green, both physical platforms evaluated, and no UI-only/simulation-only acceptance. Project-wide completion must still acknowledge Phase 6 separately. This planning task does not authorize roadmap/state/threshold changes, commits or branches.
