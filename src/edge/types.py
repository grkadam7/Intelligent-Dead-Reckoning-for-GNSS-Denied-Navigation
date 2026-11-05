"""
C-Compatible Data Types and Structs for IDR-INS Edge Core Engine.
SIH Problem Statement ID: 26168 | ISRO

Provides ctypes.Structure specifications matching standard C/C++ ABI layouts
for zero-copy integration with tactical/FOG IMU streams and embedded platforms.
"""

import ctypes
from enum import IntEnum
from typing import Optional, Tuple
import numpy as np


class IDRNavigationMode(IntEnum):
    """Navigation state mode enum matching C ABI."""
    NORMAL_FUSION = 0
    OUTAGE_DEAD_RECKONING = 1
    RECONVERGING = 2


class IDRIMUSample(ctypes.Structure):
    """
    High-rate 3-axis accelerometer and gyroscope inertial sample.
    Compatible with C struct layout (64-bit aligned).
    """
    _pack_ = 8
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("accel_x", ctypes.c_double),
        ("accel_y", ctypes.c_double),
        ("accel_z", ctypes.c_double),
        ("gyro_x", ctypes.c_double),
        ("gyro_y", ctypes.c_double),
        ("gyro_z", ctypes.c_double),
    ]

    @classmethod
    def from_values(
        cls,
        timestamp_s: float,
        accel: Tuple[float, float, float] or np.ndarray,
        gyro: Tuple[float, float, float] or np.ndarray,
    ) -> "IDRIMUSample":
        return cls(
            timestamp_s=float(timestamp_s),
            accel_x=float(accel[0]),
            accel_y=float(accel[1]),
            accel_z=float(accel[2]),
            gyro_x=float(gyro[0]),
            gyro_y=float(gyro[1]),
            gyro_z=float(gyro[2]),
        )

    def to_numpy(self) -> Tuple[float, np.ndarray, np.ndarray]:
        """Returns (timestamp_s, accel_xyz, gyro_xyz)."""
        accel = np.array([self.accel_x, self.accel_y, self.accel_z], dtype=np.float64)
        gyro = np.array([self.gyro_x, self.gyro_y, self.gyro_z], dtype=np.float64)
        return float(self.timestamp_s), accel, gyro


class IDRGNSSSample(ctypes.Structure):
    """
    Asynchronous GNSS position/velocity measurement sample.
    Compatible with C struct layout.
    """
    _pack_ = 8
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("pos_east_m", ctypes.c_double),
        ("pos_north_m", ctypes.c_double),
        ("pos_up_m", ctypes.c_double),
        ("vel_east_mps", ctypes.c_double),
        ("vel_north_mps", ctypes.c_double),
        ("vel_up_mps", ctypes.c_double),
        ("accuracy_m", ctypes.c_double),
        ("valid", ctypes.c_bool),
    ]

    @classmethod
    def from_values(
        cls,
        timestamp_s: float,
        pos_enu: Tuple[float, float, float] or np.ndarray,
        vel_enu: Optional[Tuple[float, float, float] or np.ndarray] = None,
        accuracy_m: float = 2.5,
        valid: bool = True,
    ) -> "IDRGNSSSample":
        ve = float(vel_enu[0]) if vel_enu is not None else 0.0
        vn = float(vel_enu[1]) if vel_enu is not None else 0.0
        vu = float(vel_enu[2]) if vel_enu is not None and len(vel_enu) > 2 else 0.0
        return cls(
            timestamp_s=float(timestamp_s),
            pos_east_m=float(pos_enu[0]),
            pos_north_m=float(pos_enu[1]),
            pos_up_m=float(pos_enu[2]) if len(pos_enu) > 2 else 0.0,
            vel_east_mps=ve,
            vel_north_mps=vn,
            vel_up_mps=vu,
            accuracy_m=float(accuracy_m),
            valid=bool(valid),
        )


class IDRNavOutput(ctypes.Structure):
    """
    Real-time fused navigation state output at each epoch.
    Compatible with C struct layout.
    """
    _pack_ = 8
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("pos_east_m", ctypes.c_double),
        ("pos_north_m", ctypes.c_double),
        ("pos_up_m", ctypes.c_double),
        ("vel_east_mps", ctypes.c_double),
        ("vel_north_mps", ctypes.c_double),
        ("vel_up_mps", ctypes.c_double),
        ("heading_deg", ctypes.c_double),
        ("mode", ctypes.c_int32),
        ("is_outage", ctypes.c_bool),
        ("outage_duration_s", ctypes.c_double),
        ("cross_track_error_m", ctypes.c_double),
        ("epoch_latency_ms", ctypes.c_double),
    ]

    def to_dict(self) -> dict:
        return {
            "timestamp_s": float(self.timestamp_s),
            "pos_enu": [float(self.pos_east_m), float(self.pos_north_m), float(self.pos_up_m)],
            "vel_enu": [float(self.vel_east_mps), float(self.vel_north_mps), float(self.vel_up_mps)],
            "heading_deg": float(self.heading_deg),
            "mode": IDRNavigationMode(self.mode).name,
            "is_outage": bool(self.is_outage),
            "outage_duration_s": float(self.outage_duration_s),
            "cross_track_error_m": float(self.cross_track_error_m),
            "epoch_latency_ms": float(self.epoch_latency_ms),
        }


class IDREngineStats(ctypes.Structure):
    """
    Runtime performance and health metrics for the edge engine.
    """
    _pack_ = 8
    _fields_ = [
        ("processed_samples", ctypes.c_uint64),
        ("dropped_samples", ctypes.c_uint64),
        ("avg_latency_ms", ctypes.c_double),
        ("max_latency_ms", ctypes.c_double),
        ("frequency_hz", ctypes.c_double),
    ]

    def to_dict(self) -> dict:
        return {
            "processed_samples": int(self.processed_samples),
            "dropped_samples": int(self.dropped_samples),
            "avg_latency_ms": float(self.avg_latency_ms),
            "max_latency_ms": float(self.max_latency_ms),
            "frequency_hz": float(self.frequency_hz),
        }
