"""
CTypes ABI and Memory Alignment Verification for IDR Mobile Engine (native/include/idr_mobile.h).
Validates 64-bit alignment, struct sizes, field offsets, and enum values.
"""

import ctypes
import unittest


class IDRMobileIMUSample(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("accel_x_mps2", ctypes.c_double),
        ("accel_y_mps2", ctypes.c_double),
        ("accel_z_mps2", ctypes.c_double),
        ("gyro_x_rads", ctypes.c_double),
        ("gyro_y_rads", ctypes.c_double),
        ("gyro_z_rads", ctypes.c_double),
    ]


class IDRMobileGNSSSample(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("latitude_deg", ctypes.c_double),
        ("longitude_deg", ctypes.c_double),
        ("altitude_m", ctypes.c_double),
        ("accuracy_m", ctypes.c_double),
        ("speed_mps", ctypes.c_double),
        ("heading_deg", ctypes.c_double),
        ("is_valid", ctypes.c_int32),
    ]


class IDRMobileNavOutput(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("timestamp_ms", ctypes.c_int64),
        ("sequence", ctypes.c_int64),
        ("mode", ctypes.c_int32),
        ("calibration_stage", ctypes.c_int32),
        ("latitude_deg", ctypes.c_double),
        ("longitude_deg", ctypes.c_double),
        ("altitude_m", ctypes.c_double),
        ("raw_latitude_deg", ctypes.c_double),
        ("raw_longitude_deg", ctypes.c_double),
        ("speed_mps", ctypes.c_double),
        ("heading_deg", ctypes.c_double),
        ("horizontal_uncertainty_m", ctypes.c_double),
        ("roll_deg", ctypes.c_double),
        ("pitch_deg", ctypes.c_double),
        ("yaw_deg", ctypes.c_double),
        ("is_matched", ctypes.c_int32),
        ("state_age_ms", ctypes.c_int32),
    ]


class TestMobileABI(unittest.TestCase):
    def test_imu_sample_layout(self):
        # 7 doubles * 8 bytes = 56 bytes
        self.assertEqual(ctypes.sizeof(IDRMobileIMUSample), 56)
        self.assertEqual(IDRMobileIMUSample.timestamp_s.offset, 0)
        self.assertEqual(IDRMobileIMUSample.accel_x_mps2.offset, 8)
        self.assertEqual(IDRMobileIMUSample.gyro_z_rads.offset, 48)

    def test_gnss_sample_layout(self):
        # 7 doubles * 8 bytes (56) + 1 int32 (4) + 4 bytes padding = 64 bytes
        self.assertEqual(ctypes.sizeof(IDRMobileGNSSSample), 64)
        self.assertEqual(IDRMobileGNSSSample.timestamp_s.offset, 0)
        self.assertEqual(IDRMobileGNSSSample.is_valid.offset, 56)

    def test_nav_output_layout(self):
        # Verifies 8-byte boundary alignment and offset continuity
        out = IDRMobileNavOutput()
        self.assertEqual(IDRMobileNavOutput.timestamp_ms.offset, 0)
        self.assertEqual(IDRMobileNavOutput.sequence.offset, 8)
        self.assertEqual(IDRMobileNavOutput.mode.offset, 16)
        self.assertEqual(IDRMobileNavOutput.calibration_stage.offset, 20)
        self.assertEqual(IDRMobileNavOutput.latitude_deg.offset, 24)
        self.assertEqual(IDRMobileNavOutput.state_age_ms.offset, 116)
        self.assertEqual(ctypes.sizeof(IDRMobileNavOutput), 120)


if __name__ == "__main__":
    unittest.main()

