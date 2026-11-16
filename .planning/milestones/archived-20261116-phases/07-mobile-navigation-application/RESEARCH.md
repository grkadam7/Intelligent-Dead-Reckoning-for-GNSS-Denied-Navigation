# Phase 7 Research: Mobile Navigation Application

**Project:** IDR-INS (SIH 26168)
**Scope:** Planning only for MOBILE-01, MOBILE-02, MOBILE-03.
**Recommendation status:** PROVISIONAL — framework, dependencies, and architecture are not user-approved.
**Evidence:** Static repository inspection and official documentation retrieval only; no runtime validation performed by this researcher.

## Summary

Phase 7 requires a cross-platform application with an offline map, genuine navigation state updates at >=10 Hz, navigation-mode/confidence indicators, and phone-to-vehicle calibration status. The existing Python implementation is a reference, not a compiled mobile library. Missing model/map artifacts and a native deployment path must be explicit work items rather than assumed prerequisites already satisfied.

A remote Python server cannot satisfy offline navigation. Replay is useful for UI development and parity testing, but must be visibly labelled simulation and cannot substitute for live on-device integration.

## Outstanding Phase 6 Verification

The user explicitly authorized Phase 7 planning while Phase 6 verification remains outstanding. Do not change completion markers, state, thresholds, or staged work as part of this research.

- `.planning/phases/06-high-frequency-edge-core-engine/06-01-PLAN.md` requires blackout drift **<1.5%**.
- `.planning/STATE.md` records **6.92%**. This is below the overall <10% requirement, but does not meet the Phase 6 plan target.
- `evaluate_phase6.py:154–160` checks both thresholds separately.
- The inspected Phase 6 directory contains the plan and research, but no execution summary.
- Roadmap completion markers conflict with pending requirement statuses and unresolved verification.
- **Terminal error attribution:** The parent agent actually attempted pytest and reproduced a terminal-launch error involving `libasound.so.2`. This is a parent-observed launch failure, not merely an untested user report. It does not constitute a completed pytest run or evidence that tests passed or failed.

Planning and UI/replay work may proceed; downstream acceptance must not inherit unverified accuracy, timing, or native-export claims. Preserve the Phase 6 <1.5% gate separately from the overall <10% requirement.

## Existing Code and Deployment Gaps

| Existing path | Verified by static inspection |
| --- | --- |
| `src/edge/engine.py` | Python high-rate EKF/NHC/ZUPT engine, asynchronous GNSS ingestion, recovery smoothing. |
| `src/edge/c_api.py` | Ordinary Python functions, integer-handle registry, ctypes memory copies. Not compiled native exports. |
| `include/idr_edge_engine.h` | C declarations and struct layouts; no corresponding native implementation found. |
| `src/fusion/fusion_engine.py` | Python orchestration accepting externally supplied `ai_forward_speed`, with optional road matching. |
| `src/fusion/ekf.py` | Nine-state filter; covariance exists internally. |
| `src/preprocessing/alignment.py` | Static leveling, forward-axis calibration, rotations, calibration flags/confidence. |
| `src/preprocessing/filters.py` | Batch and streaming filters; streaming behavior requires reference validation before porting. |
| `src/map_matching/road_network.py` | Road representation, spatial index, graph-distance logic, GeoJSON parsing. |
| `src/map_matching/hmm_matcher.py` | Python map matcher. |
| `src/data/iovnbd_loader.py` | Coordinate conversion helpers and dataset/synthetic-run support. |
| `tests/test_phase6.py` | Python layout, buffer, engine, and API lifecycle tests; not proof of compiled symbol availability. |

Specific gaps:

1. `HighRateEdgeEngine` does not instantiate SpeedNet inference or a map matcher. Its AI subsampling configuration does not implement ONNX scheduling, and output cross-track error is always `0.0`.
2. The edge streaming loop constructs and uses an aligner but does not drive its calibration procedures.
3. `IDR_NavOutput` exposes neither covariance-derived uncertainty nor calibration state. Cross-track error is not a confidence bound.
4. Header IMU comments describe vehicle/body axes, while the engine performs device-to-vehicle alignment. Resolve the input-frame contract before binding.
5. `idr_engine_reset` returns `int` in Python but `void` in the header.
6. No native implementation/build configuration or mobile scaffold was found. FFI bindings do not transform Python functions into linkable C symbols.
7. `requirements.txt` lists NumPy, SciPy, pandas, and matplotlib, while model tooling imports PyTorch and ONNX Runtime separately. The inspected dependency file does not establish reproducible model tooling versions.

## Model and Map Asset Inventory

No deployable model or map assets were found in the inspected workspace:

- No `.onnx`, `.pt`, `.pth`, `.tflite`, `.ort`, GeoJSON, OSM/PBF, MBTiles, PMTiles, or map database files were found.
- `data/iovnbd/` was empty.
- `train_speed_model.py` generates these expected but currently absent artifacts:
  - `models/speednet_best.pt`
  - `models/speednet.onnx`
  - `models/norm_stats.json`
- `reports/figures/` contains `baseline_ins_real_iovnbd.png`, `baseline_ins_synthetic.png`, and `map_matched_trajectory.png`. These are not offline map data.
- `evaluate_phase5.py` uses synthetic data, passes GPS-derived speed as `ai_forward_speed`, and builds roads from ground-truth waypoints. This demonstrates integration plumbing, not independent learned-speed/offline-road accuracy.

Recover versioned artifacts or plan reproducible training/export, plus an independent regional map package. Do not declare these assets available based solely on source-code default paths.

## Cross-Platform Approach — PROVISIONAL, Not User-Approved

Recommend **Flutter UI + MapLibre Native + shared compiled C++ navigation core with ONNX Runtime**, using Kotlin/Swift sensor/location adapters.

Proposed flow:

`Native sensors/GNSS -> bounded worker queue -> shared core + local ONNX/model/maps -> timestamped navigation snapshots -> Flutter map/HUD`

Rationale and evidence:

- Flutter officially supports C interop through `dart:ffi` [1] and typed native integration through Pigeon/platform channels [2].
- ONNX Runtime documents Android/iOS native packages and on-device inference [3].
- A shared core avoids two independently evolving numerical ports; this is an engineering recommendation, not a proven implementation in this repository.

Required new work includes the numerical core, real C ABI, platform packaging, model/map loading, ownership/error contracts, reset behavior, and enriched telemetry. This is a material native-port effort, not merely attaching UI to an existing mobile-ready library.

A Flutter MapLibre wrapper's offline APIs, maintenance status, and version compatibility remain **unverified**. Spike Android and iOS before choosing/pinning versions. Native platform-view/plugin integration is the fallback if a wrapper cannot expose required offline functionality. Do not invent a dependency version without a compatibility check.

Use a worker for numerical processing and keep callbacks/UI work short; publish compact navigation snapshots rather than serializing every high-rate sample through the UI. Flutter documents channel threading constraints [2].

iOS build/signing requires Mac/Xcode tooling [4]. Its availability is not established in this Windows workspace. Android-first execution must not be represented as completed cross-platform validation.

## Offline Maps Strategy

Maintain two coordinated, versioned local datasets:

1. **Rendering package:** bounded-region vector tiles, style JSON, glyphs/fonts, sprites, attribution, and declared zoom coverage.
2. **Matching graph:** road geometry, shared intersection/node IDs, directionality, connectivity, spatial index, and geographic reference.

MapLibre Android officially supports offline-region creation and database merging [5]. Official search results indicated iOS offline-pack support [11], but fetched iOS documentation returned a JavaScript shell; exact packaging/import behavior remains **unverified**.

Do not assume an arbitrary MBTiles/PMTiles archive works through the chosen Flutter wrapper. Prove the selected storage/import mechanism on both target platforms first. For a bounded initial region, package or explicitly download an offline-permitted region before travel; never require a first network fetch inside the outage.

`RoadNetwork.from_geojson()` generates per-segment endpoint IDs, and `add_segment()` does not merge coincident junctions. Shared-node/topology preparation is required before claiming a connected real-road graph. Rendering tiles are not a routing graph. Do not infer turn-by-turn routing support from map display or geometric snapping; define any routing scope explicitly.

Each package manifest should record coverage, zoom range, version, checksums, size, licenses, and graph origin. Handle absent/corrupt packages, insufficient storage, and leaving coverage explicitly.

Use self-produced tiles or a provider expressly permitting offline use and redistribution. Standard `tile.openstreetmap.org` prohibits offline prefetch/bulk downloading [6]. Preserve OSM attribution and applicable ODbL obligations [7]. Provider fees/keys and redistribution rights must be settled before adoption; no provider is selected here.

## On-Device ONNX and Sensor Integration

### Model contract

`src/speed_estimation/inference.py` and `dataset.py` currently specify:

- Float32 input `[1, 6, 50]` by default, channel-first.
- Channels: `ax, ay, az, yaw-rate, pitch-rate, roll-rate`, not generic XYZ gyro order.
- Stored training `mu`/`sigma` normalization and nonnegative speed output in m/s.
- Export names `imu_window` and `speed_ms`; default exporter opset is 11, but no exported artifact exists to inspect.
- Training labels use center-of-window speed, creating a temporal-alignment issue for trailing-window live inference.

The interface does not encode training sampling rate. Fifty samples alone is not a complete time-window specification. Recover and freeze sampling rate, frame, gravity/sign convention, ordering, normalization, and output timestamp semantics before integration.

Missing normalization must not silently become identity normalization in the mobile acceptance path. Record the model checksum, input/output metadata, normalization version, training provenance, and expected sampling rate together.

Establish CPU float32 parity first. Only then assess quantization or accelerators against accuracy/latency requirements. ONNX Runtime documents CPU/XNNPACK starting points and warns that accelerator performance is device/model specific [3]. GRU/operator support and actual model loadability remain unverified until an artifact is available.

### Sensor contract

Use Android `SensorManager`, iOS Core Motion, and native location adapters:

- Android accelerometer reports m/s² including gravity; gyro reports rad/s [8].
- iOS raw acceleration reports g units; raw gyro reports rad/s [9][10].
- Validate platform signs/axes and gravity handling with controlled poses/rotations. Unit conversion alone is insufficient. The aligner expects a particular gravity convention and removes gravity itself.
- Synchronize asynchronous accel/gyro samples, normalize clock domains, reject duplicate/stale fixes, and distinguish no callback from a declared invalid fix.
- Convert GNSS WGS84 coordinates to a stable ENU origin shared with the road graph, then convert output back for display. Existing reference helpers are in `src/data/iovnbd_loader.py`.
- Missing optional GNSS metrics must remain unavailable rather than fabricated; define handling of accuracy, invalid course/speed, and freshness per platform.

Requested sensor rates are not guaranteed [8][9]. Capture at the validated model rate or resample with appropriate anti-alias filtering. Separate sensor acquisition rate, inference cadence, >=10 Hz navigation publication, and render animation. A 50-sample window's initial fill time is not inference execution latency.

Show actual calibration progress from static leveling and motion alignment. Treat calibration confidence separately from navigation-position uncertainty. Background/screen-off operation needs explicit platform/lifecycle scope; do not promise it from foreground sensor tests.

## Reference-Parity Approach

Freeze a reviewed Python reference and deterministic fixtures before porting. Phase 5 and Phase 6 engines are not equivalent; selecting the authoritative behavior is a planning decision, not something to silently resolve in the mobile port.

Compare intermediate alignment, causal filtering, normalized model windows, ONNX speed, EKF state/covariance, modes, reconvergence, and road-matching outputs. Use explicit tolerances for continuous outputs and exact expectations for discrete states. Validate causal streaming behavior rather than requiring a live filter to match an acausal batch `filtfilt` result.

Known reference hazards requiring explicit resolution or documented exclusions:

- `GNSSHealthMonitor` treats an epoch without a fix as outage, whereas the edge engine uses a grace period. This matters with asynchronous/sparse healthy GNSS updates.
- Its reported detection latency uses the same input timestamp for start/end and is clamped; it does not measure end-to-end physical outage detection.
- `evaluate_phase5.py` supplies GPS speed and ground-truth-derived roads. These cannot establish learned-speed blackout performance or independent map accuracy.
- Calibration is not driven by the edge loop and neither inference nor matching is wired into that engine.

Do not reproduce a known defect just to obtain parity. Version reviewed corrections separately and explain any intentional divergence; this research does not implement those changes.

## Validation Architecture

### Layers and evidence

| Layer | Verification mechanism | Evidence required |
| --- | --- | --- |
| Contracts/assets | Static metadata and package checks | Model/map checksums, schema/version, normalization, sampling rate, licensing, coverage. |
| Python reference | Deterministic replay fixtures and focused existing tests | Reviewed expected intermediate/output values; explicit unresolved reference issues. |
| Compiled core/ABI | Host-native tests plus Android/iOS integration tests | Real symbol linking, compiled `sizeof`/offset checks against bindings, handle lifecycle, ownership/errors/reset. |
| Inference parity | Identical normalized windows on Python/native CPU runtimes | Speed differences within predeclared tolerances; real model load/operator coverage. |
| Sensor adapters | Controlled poses/rotations and recorded stream replay | Units/signs/axes, clock conversion, sample pairing/resampling, stale/duplicate handling. |
| Offline application | Cold launch with networking blocked | Local style/fonts/sprites/tiles, model, and matching graph function without remote processing. |
| UI/HUD | Deterministic replay and platform integration tests | >=10 Hz genuine state publication, mode/calibration/uncertainty visibility, stale-state behavior. |
| Physical devices | Sustained representative recordings | Latency distributions, thermal/battery observations, drops, outage/recovery behavior, independent drift metrics. |

### Acceptance gates

1. **Offline:** Cold start with networking blocked; map resources, inference, and matching work. Test missing/corrupt packages and out-of-coverage behavior. A warm ambient cache is not sufficient proof.
2. **Reference parity:** Deterministic fixtures pass on both platforms with predefined numerical tolerances and exact mode expectations. No ground-truth/GPS speed substitution during blackout.
3. **Timing:** Genuine navigation state publication >=10 Hz, sensor-to-position age <=100 ms, inference <20 ms on named target devices. Report p95/p99/max, sample drops, warm-up, and sustained thermal behavior. Define how scheduling/clock synchronization affects the measurement. Smooth animation is not proof of 10 Hz estimation.
4. **Outage/recovery:** Test explicit invalid fixes, missing/stale fixes, sparse healthy fixes, degraded accuracy, and recovery. Measure callback-to-decision latency separately from physical-signal-loss detection. A 10 Hz UI clock alone cannot guarantee <=50 ms outage response; faster health processing and realistic GNSS freshness expectations are required.
5. **HUD/calibration:** Display actual navigation mode, calibration progress/orientation, and honestly labelled uncertainty. Cross-track error or alignment confidence must not be relabelled as a calibrated position confidence bound. Expose covariance-derived uncertainty with documented assumptions or show it unavailable.
6. **Robustness:** Test denied/revoked permissions, missing sensors/assets, timestamp disorder, stream gaps, mount shifts, pause/resume, and declared screen-off/background scope.
7. **Accuracy:** Independent held-out replay and physical-device recordings evaluate the overall <10% blackout drift requirement. Separate raw and matched trajectories, distance definitions, peak/endpoint error, and map availability. No lane-level accuracy claim follows from road snapping or drift percentage alone.
8. **Phase 6:** Keep its unmet <1.5% gate and missing summary outstanding; do not weaken it to <10% to make mobile acceptance appear green.
9. **Cross-platform evidence:** Simulator/replay can establish UI and deterministic integration, but not real sensor accuracy, battery behavior, or on-device performance. Require Android and iOS device evidence for full mobile validation.

### Current validation status

**No runtime validation performed by this researcher.** Repository/source inspection and official documentation retrieval are the only completed research checks. The parent attempted pytest but terminal launch failed with `libasound.so.2`; no completed test result follows. Native ABI, model loading, mobile builds, timing, battery, and accuracy remain unverified.

## Suggested Planning Slices

Retain roadmap identifiers `07-01` and `07-02`, but break execution into small explicit slices:

1. **Contract/toolchain spike:** Provisional stack approval, Android/iOS native loading, asset readiness, telemetry schema, and authoritative reference selection.
2. **07-01:** Cold-start offline map, labelled deterministic replay, >=10 Hz cursor, and basic mode/calibration HUD.
3. **07-02 foundation:** Shared compiled core, real ABI, ONNX loading, reviewed Python-reference parity, matching graph support.
4. **07-02 live integration:** Sensors/GNSS, calibration, uncertainty, outage/recovery, and physical-device acceptance.

Do not hide the native port or missing assets in a generic “integrate engine” task. Framework/plugin versions, initial map region/provider, artifact provenance, target devices, Mac access, lifecycle scope, and uncertainty semantics remain explicit decisions or prerequisites.

## Official Sources

Retrieved during research. These establish platform capabilities, not successful integration in this repository.

1. Flutter native FFI: https://docs.flutter.dev/platform-integration/bind-native-code
2. Flutter platform channels, Pigeon, and threading: https://docs.flutter.dev/platform-integration/platform-channels
3. ONNX Runtime mobile application development: https://onnxruntime.ai/docs/tutorials/mobile/
4. Flutter iOS toolchain setup: https://docs.flutter.dev/platform-integration/ios/setup
5. MapLibre Android OfflineManager: https://maplibre.org/maplibre-native/android/api/-map-libre%20-native%20-android/org.maplibre.android.offline/-offline-manager/index.html
6. OSMF tile usage policy: https://operations.osmfoundation.org/policies/tiles/
7. OpenStreetMap copyright/ODbL attribution: https://www.openstreetmap.org/copyright
8. Android sensors overview, units, coordinates, delivery rates, and lifecycle: https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview
9. Apple raw accelerometer events (readable official Markdown): https://developer.apple.com/documentation/coremotion/getting-raw-accelerometer-events.md
10. Apple raw gyroscope events (readable official Markdown): https://developer.apple.com/documentation/coremotion/getting-raw-gyroscope-events.md
11. MapLibre iOS offline storage: https://maplibre.org/maplibre-native/ios/latest/documentation/maplibre/mlnofflinestorage/ — official search results exposed offline-pack APIs, but direct fetch returned a JavaScript shell. Exact behavior remains unverified.

DevRelay tools were unavailable; no community sources or community consensus claims were fabricated.
