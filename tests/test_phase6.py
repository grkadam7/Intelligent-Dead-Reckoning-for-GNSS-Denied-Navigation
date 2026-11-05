"""
Comprehensive Unit & Integration Test Suite for Phase 6: High-Frequency Edge Core Engine.
SIH Problem Statement ID: 26168 | ISRO

Validates EDGE-01 and EDGE-02:
- C-compatible ABI structures, memory alignment, and conversions
- HighRateRingBuffer FIFO ordering, capacity, and overflow policies
- HighRateEdgeEngine 200 Hz processing throughput and <1.0 ms epoch latency budget
- Seamless GNSS blackout detection and drift bounding under 200 Hz operation
- C API lifecycle functions and handle registry
"""

import ctypes
import time
import numpy as np
import pytest

from src.edge.types import (
    IDRNavigationMode,
    IDRIMUSample,
    IDRGNSSSample,
    IDRNavOutput,
    IDREngineStats,
)
from src.edge.buffer import HighRateRingBuffer
from src.edge.engine import HighRateEdgeEngine
from src.edge.c_api import (
    idr_engine_create,
    idr_engine_destroy,
    idr_engine_push_imu,
    idr_engine_push_gnss,
    idr_engine_step,
    idr_engine_get_stats,
    idr_engine_reset,
)


class TestCTypesAndLayout:
    """Verifies C ABI compatibility and data structures."""

    def test_imu_sample_layout(self):
        sample = IDRIMUSample.from_values(
            timestamp_s=100.5,
            accel=(1.2, -0.3, 9.81),
            gyro=(0.01, -0.02, 0.05),
        )
        assert ctypes.sizeof(sample) == 56  # 7 * 8 bytes (double)
        t, a, g = sample.to_numpy()
        assert t == 100.5
        np.testing.assert_allclose(a, [1.2, -0.3, 9.81])
        np.testing.assert_allclose(g, [0.01, -0.02, 0.05])

    def test_gnss_sample_layout(self):
        gnss = IDRGNSSSample.from_values(
            timestamp_s=100.0,
            pos_enu=(10.0, 20.0, 0.0),
            vel_enu=(15.0, 0.0, 0.0),
            accuracy_m=1.8,
            valid=True,
        )
        assert gnss.pos_east_m == 10.0
        assert gnss.pos_north_m == 20.0
        assert gnss.vel_east_mps == 15.0
        assert gnss.accuracy_m == 1.8
        assert gnss.valid is True

    def test_nav_output_layout(self):
        out = IDRNavOutput(
            timestamp_s=1.0,
            pos_east_m=10.0,
            pos_north_m=20.0,
            pos_up_m=0.0,
            vel_east_mps=15.0,
            vel_north_mps=0.0,
            vel_up_mps=0.0,
            heading_deg=90.0,
            mode=int(IDRNavigationMode.NORMAL_FUSION),
            is_outage=False,
            outage_duration_s=0.0,
            cross_track_error_m=0.2,
            epoch_latency_ms=0.45,
        )
        d = out.to_dict()
        assert d["mode"] == "NORMAL_FUSION"
        assert d["is_outage"] is False
        assert d["pos_enu"][0] == 10.0

    def test_engine_stats_layout(self):
        stats = IDREngineStats(
            processed_samples=500,
            dropped_samples=0,
            avg_latency_ms=0.35,
            max_latency_ms=0.95,
            frequency_hz=200.0,
        )
        d = stats.to_dict()
        assert d["processed_samples"] == 500
        assert d["frequency_hz"] == 200.0


class TestHighRateRingBuffer:
    """Verifies thread-safe ring buffer operation."""

    def test_buffer_push_pop_fifo(self):
        buf = HighRateRingBuffer(capacity=4)
        assert buf.is_empty
        assert buf.size == 0

        s1 = IDRIMUSample.from_values(1.0, (1, 0, 9.8), (0, 0, 0))
        s2 = IDRIMUSample.from_values(2.0, (2, 0, 9.8), (0, 0, 0))
        s3 = IDRIMUSample.from_values(3.0, (3, 0, 9.8), (0, 0, 0))

        buf.push(s1)
        buf.push(s2)
        buf.push(s3)
        assert buf.size == 3
        assert not buf.is_empty
        assert not buf.is_full

        popped = buf.pop()
        assert popped.timestamp_s == 1.0
        assert buf.size == 2

        # Peek
        peeked = buf.peek()
        assert peeked.timestamp_s == 2.0
        assert buf.size == 2

    def test_buffer_overwrite_policy(self):
        buf = HighRateRingBuffer(capacity=2, overwrite_on_full=True)
        s1 = IDRIMUSample.from_values(1.0, (1, 0, 9.8), (0, 0, 0))
        s2 = IDRIMUSample.from_values(2.0, (2, 0, 9.8), (0, 0, 0))
        s3 = IDRIMUSample.from_values(3.0, (3, 0, 9.8), (0, 0, 0))

        assert buf.push(s1) is True
        assert buf.push(s2) is True
        assert buf.is_full
        # Overwrite oldest (s1 dropped)
        assert buf.push(s3) is True
        assert buf.dropped_count == 1

        p1 = buf.pop()
        assert p1.timestamp_s == 2.0
        p2 = buf.pop()
        assert p2.timestamp_s == 3.0
        assert buf.is_empty

    def test_buffer_drop_newest_policy(self):
        buf = HighRateRingBuffer(capacity=2, overwrite_on_full=False)
        s1 = IDRIMUSample.from_values(1.0, (1, 0, 9.8), (0, 0, 0))
        s2 = IDRIMUSample.from_values(2.0, (2, 0, 9.8), (0, 0, 0))
        s3 = IDRIMUSample.from_values(3.0, (3, 0, 9.8), (0, 0, 0))

        assert buf.push(s1) is True
        assert buf.push(s2) is True
        # Drop newest
        assert buf.push(s3) is False
        assert buf.dropped_count == 1
        assert buf.size == 2

    def test_pop_batch(self):
        buf = HighRateRingBuffer(capacity=10)
        for i in range(5):
            buf.push(IDRIMUSample.from_values(float(i), (0, 0, 9.8), (0, 0, 0)))

        batch = buf.pop_batch(max_samples=3)
        assert len(batch) == 3
        assert [s.timestamp_s for s in batch] == [0.0, 1.0, 2.0]
        assert buf.size == 2


class TestHighRateEdgeEngine:
    """Verifies high-rate 200 Hz navigation loop and performance."""

    def test_engine_initialization(self):
        engine = HighRateEdgeEngine(sampling_rate=200.0)
        assert engine.sampling_rate == 200.0
        assert engine.default_dt == pytest.approx(0.005)
        stats = engine.get_stats()
        assert stats.processed_samples == 0

    def test_single_step_via_buffer(self):
        engine = HighRateEdgeEngine(sampling_rate=200.0)
        sample = IDRIMUSample.from_values(0.005, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
        engine.push_imu(sample)

        out = engine.step()
        assert out is not None
        assert out.timestamp_s == 0.005
        assert out.epoch_latency_ms > 0.0

        # Step when empty returns None
        assert engine.step() is None

    def test_sustained_200hz_streaming_throughput(self):
        """Simulates 1,000 samples at 200 Hz (5 seconds equivalent) and validates latency <1.0 ms."""
        engine = HighRateEdgeEngine(sampling_rate=200.0)
        n_samples = 1000
        dt = 0.005

        # Initialize with static period for leveling
        for i in range(50):
            s = IDRIMUSample.from_values(i * dt, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            engine.step(s)

        # Drive forward at 15 m/s
        latencies = []
        for i in range(50, n_samples):
            t = i * dt
            # GNSS fix at 10 Hz (every 20 IMU samples)
            gnss = None
            if i % 20 == 0:
                gnss = IDRGNSSSample.from_values(
                    timestamp_s=t,
                    pos_enu=(t * 15.0, 0.0, 0.0),
                    vel_enu=(15.0, 0.0, 0.0),
                    accuracy_m=2.0,
                )

            imu = IDRIMUSample.from_values(t, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            out = engine.step(imu_sample=imu, gnss_sample=gnss)
            assert out is not None
            latencies.append(out.epoch_latency_ms)

        stats = engine.get_stats()
        avg_latency = stats.avg_latency_ms
        p99_latency = np.percentile(latencies, 99)

        # Performance Assertion: Mean latency < 1.0 ms (EDGE-02)
        assert avg_latency < 1.0, f"Average latency {avg_latency:.3f} ms exceeds 1.0 ms budget"
        assert p99_latency < 2.5, f"P99 latency {p99_latency:.3f} ms exceeds 2.5 ms"
        assert stats.dropped_samples == 0

    def test_blackout_and_smooth_recovery_at_200hz(self):
        """
        Simulates vehicle moving at 20 m/s with 3-second GNSS blackout at 200 Hz.
        Validates drift remains <1.5% and recovery transitions to RECONVERGING mode.
        """
        engine = HighRateEdgeEngine(sampling_rate=200.0)
        dt = 0.005
        v_true = 20.0

        # Phase A: Normal driving with GNSS (0 to 3s)
        for i in range(600):
            t = i * dt
            gnss = None
            if i % 20 == 0:  # 10 Hz GNSS
                gnss = IDRGNSSSample.from_values(t, (v_true * t, 0.0, 0.0), (v_true, 0.0, 0.0), 2.0)
            imu = IDRIMUSample.from_values(t, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            out = engine.step(imu, gnss)
            assert out.mode == int(IDRNavigationMode.NORMAL_FUSION)

        # Phase B: 3-second GNSS Tunnel Outage (3s to 6s = 600 epochs at 200 Hz)
        outage_start_dist = v_true * 3.0
        blackout_outputs = []
        for i in range(600, 1200):
            t = i * dt
            # GNSS completely absent in tunnel
            imu = IDRIMUSample.from_values(t, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
            out = engine.step(imu, gnss_sample=None, explicit_outage=True)
            assert out.is_outage is True
            assert out.mode == int(IDRNavigationMode.OUTAGE_DEAD_RECKONING)
            blackout_outputs.append(out)

        outage_dist_traveled = v_true * 3.0  # 60 meters
        final_dr_pos = blackout_outputs[-1].pos_east_m
        true_final_pos = v_true * 6.0
        pos_error = abs(final_dr_pos - true_final_pos)
        drift_pct = (pos_error / outage_dist_traveled) * 100.0

        # Assert drift < 1.5% (SIH target <10%)
        assert drift_pct < 1.5, f"Drift percentage {drift_pct:.2f}% exceeds 1.5%"

        # Phase C: GNSS Signal Restoration
        t_rec = 1200 * dt
        rec_gnss = IDRGNSSSample.from_values(t_rec, (v_true * t_rec, 0.0, 0.0), (v_true, 0.0, 0.0), 2.0)
        rec_imu = IDRIMUSample.from_values(t_rec, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
        out_rec = engine.step(rec_imu, rec_gnss)
        assert out_rec.is_outage is False
        assert out_rec.mode == int(IDRNavigationMode.RECONVERGING)


class TestCAPILifecycle:
    """Verifies foreign function interface (C ABI) functions."""

    def test_c_api_full_workflow(self):
        # 1. Create engine
        handle = idr_engine_create(200.0)
        assert handle > 0

        # 2. Push IMU sample
        imu = IDRIMUSample.from_values(0.005, (0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
        assert idr_engine_push_imu(handle, imu) == 0

        # 3. Push GNSS sample
        gnss = IDRGNSSSample.from_values(0.005, (10.0, 20.0, 0.0), (15.0, 0.0, 0.0), 2.0)
        assert idr_engine_push_gnss(handle, gnss) == 0

        # 4. Step engine
        out = IDRNavOutput()
        ret = idr_engine_step(handle, ctypes.byref(out))
        assert ret == 0
        assert out.timestamp_s == 0.005
        assert out.pos_east_m != 0.0 or out.pos_north_m != 0.0

        # 5. Query stats
        stats = IDREngineStats()
        ret_stats = idr_engine_get_stats(handle, ctypes.byref(stats))
        assert ret_stats == 0
        assert stats.processed_samples == 1

        # 6. Reset
        assert idr_engine_reset(handle) == 0

        # 7. Destroy
        idr_engine_destroy(handle)

        # 8. Access after destroy fails
        assert idr_engine_push_imu(handle, imu) == -1
        assert idr_engine_step(handle, ctypes.byref(out)) == -1
