# Phase 6: High-Frequency Edge Core Engine - Technical Research
*SIH Problem Statement ID: 26168 | ISRO*

## Objective
Package the complete Intelligent Dead Reckoning and Kalman Fusion navigation core into a standalone, headless, edge-deployable software engine capable of sustaining high-rate IMU sensor feeds (up to 200 Hz, 5 ms cycle time) for tactical and Fiber Optic Gyro (FOG) external IMUs with zero UI dependencies and an open C/C++ FFI interface.

---

## 1. Requirements & Traceability

- **EDGE-01**: The core dead reckoning and fusion algorithms must be modularized into an edge-deployable library (C++ / Python) independent of mobile UI.
  - *Target*: Headless library (`src/edge/`), clean C-compatible ABI / ctypes interface (`idr_edge_engine.h`), zero GUI/display framework imports.
- **EDGE-02**: The edge engine must support high-rate IMU inputs up to 200 Hz for tactical / FOG grade external IMUs.
  - *Target*: Sustained 200 Hz processing throughput ($> 200$ samples/sec), mean processing latency per epoch $< 1.0$ ms, zero frame drops over continuous high-frequency streaming runs.

---

## 2. Technical Challenges & Architectural Decisions

### Challenge 1: Multi-Rate Frequency Mismatch
- **Observation**:
  - Tactical / FOG IMUs stream at 100–200 Hz ($5$ ms period).
  - GNSS receivers output at 1–10 Hz ($100$–$1000$ ms period).
  - AI-SpeedNet inference operates on windowed samples ($L=100$ samples, stride 10) and takes $\approx 10$–$15$ ms in ONNX. Running AI inference on every single 200 Hz epoch would exceed the 5 ms time budget!
- **Solution (Multi-Rate Decoupled Architecture)**:
  - **High-Rate Strapdown & EKF Propagation (200 Hz)**: Process raw gyroscope and accelerometer readings through attitude leveling, bias compensation, and EKF state covariance propagation ($F, Q, P$).
  - **Downsampled AI Speed Prediction (10–20 Hz)**: Buffer IMU samples into a circular window buffer; trigger AI SpeedNet inference every $k$ samples (e.g. every 10 samples / 20 Hz) or use inertial kinematics between inference cycles, ensuring the 200 Hz loop runs in $< 0.5$ ms.
  - **Asynchronous GNSS Measurement Updates (1–10 Hz)**: Queue GNSS measurements and apply EKF Kalman update ($H, R, K$) only when a fresh GNSS fix arrives.

### Challenge 2: Jitter & Thread-Safe Ingestion
- **Observation**: Serial or UDP/CAN streams from external FOG IMUs can produce bursts or thread contention if processed synchronously on the sensor reader thread.
- **Solution**:
  - Implement a lock-free or high-performance bounded Ring Buffer (`IMURingBuffer`) in `src/edge/buffer.py`.
  - Provide both synchronous epoch-by-epoch stepping (`step()`) and an asynchronous worker thread runner (`start_streaming()`, `push_imu()`).

### Challenge 3: Edge Portability & C FFI Interface
- **Observation**: Embedded platforms (e.g., NVIDIA Jetson, Raspberry Pi, automotive ECUs, ROS 2 nodes) require interoperability with C/C++.
- **Solution**:
  - Define packed, 64-bit aligned C structs using Python's `ctypes` (`IDR_IMUSample`, `IDR_GNSSSample`, `IDR_NavOutput`).
  - Provide a C header `idr_edge_engine.h` matching the ABI definitions.
  - Expose a clean C-callable function signature suite:
    - `idr_engine_create(double sampling_rate_hz)`
    - `idr_engine_destroy(void* handle)`
    - `idr_engine_push_imu(void* handle, const IDR_IMUSample* sample)`
    - `idr_engine_push_gnss(void* handle, const IDR_GNSSSample* sample)`
    - `idr_engine_step(void* handle, IDR_NavOutput* out)`
    - `idr_engine_get_stats(void* handle, IDR_EngineStats* stats)`

---

## 3. Module Design for `src/edge/`

1. `src/edge/types.py`:
   - `ctypes.Structure` classes for C ABI compatibility:
     - `IDRIMUSample`: `timestamp_s`, `accel_xyz[3]`, `gyro_xyz[3]`
     - `IDRGNSSSample`: `timestamp_s`, `pos_enu[3]`, `vel_enu[3]`, `accuracy_m`, `valid`
     - `IDRNavOutput`: `timestamp_s`, `pos_enu[3]`, `vel_enu[3]`, `heading_deg`, `mode`, `latency_ms`
     - `IDREngineStats`: `processed_samples`, `dropped_samples`, `avg_latency_ms`, `max_latency_ms`, `frequency_hz`

2. `src/edge/buffer.py`:
   - `HighRateRingBuffer`: Fixed-size pre-allocated circular buffer with thread-safe atomic pointer updates for zero-allocation streaming.

3. `src/edge/engine.py`:
   - `HighRateEdgeEngine`: Master edge navigation engine.
   - Configurable for 10 Hz to 200 Hz.
   - Decoupled multi-rate scheduling:
     - 200 Hz INS/Attitude integration & EKF time propagation.
     - Decoupled AI-SpeedNet updates.
     - Sub-50ms GNSS blackout detection.
     - Exponential smooth reconvergence filter.

4. `src/edge/c_api.py`:
   - C-compatible wrapper exposing standard C ABI function signatures.
   - Generates or supplies `idr_edge_engine.h` for external integration.

5. `src/edge/benchmark.py` & `evaluate_phase6.py`:
   - Comprehensive benchmark suite simulating 200 Hz continuous FOG IMU stream.
   - Evaluates:
     - Throughput (sustained $\ge 200$ Hz)
     - Mean processing latency ($< 1$ ms) and 99th percentile jitter ($< 2$ ms)
     - Blackout recovery and drift under 200 Hz operation.

---

## 4. Verification & Testing Strategy

- `tests/test_phase6.py`:
  - `test_c_types_layout`: Verify `ctypes` byte sizes and alignment match C specification.
  - `test_ring_buffer_operations`: Verify FIFO ordering, wrap-around, and overflow handling.
  - `test_200hz_streaming_throughput`: Push 1,000 samples at 200 Hz and assert 0 dropped samples and processing rate $> 200$ Hz.
  - `test_latency_budget`: Measure average per-epoch latency over 200 Hz stream and ensure $< 1.0$ ms.
  - `test_blackout_behavior_at_200hz`: Simulate GNSS outage at 200 Hz and confirm drift $< 1.5\%$ with smooth recovery.
