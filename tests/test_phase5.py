"""
Comprehensive Unit and Integration Tests for Phase 5:
GNSS+INS Fusion Engine & Instant Blackout Handler.
SIH Problem Statement ID: 26168 | ISRO
"""

import unittest
import numpy as np

from src.fusion.ekf import ExtendedKalmanFilter
from src.fusion.blackout_handler import (
    GNSSHealthMonitor,
    NavigationMode,
    SmoothReconverger,
)
from src.fusion.fusion_engine import GNSSINSFusionEngine
from src.map_matching.road_network import RoadNetwork
from src.data.iovnbd_loader import generate_synthetic_run


class TestPhase5EKF(unittest.TestCase):
    """Unit tests for Extended Kalman Filter (FUSION-01)."""

    def setUp(self):
        self.ekf = ExtendedKalmanFilter()
        self.ekf.initialize(
            timestamp_s=0.0,
            position_enu=np.array([10.0, 20.0, 0.0]),
            velocity_enu=np.array([0.0, 10.0, 0.0]), # 10 m/s North
            heading_deg=0.0,
        )

    def test_ekf_predict_propagates_state(self):
        """Test EKF time propagation over 1.0 second."""
        state = self.ekf.predict(
            timestamp_s=1.0,
            accel_forward_mps2=0.0,
            gyro_yaw_rate_rads=0.0,
            dt=1.0
        )
        # Position should propagate along North: North position = 20.0 + 10.0 * 1.0 = 30.0
        self.assertAlmostEqual(state.position_enu[1], 30.0, places=3)
        self.assertAlmostEqual(state.heading_deg, 0.0, places=3)

    def test_ekf_gnss_update_reduces_covariance(self):
        """Test that GNSS position correction reduces covariance and aligns position."""
        # Initial position covariance for East is 100.0
        cov_prior = self.ekf.P[0, 0]

        # Apply GNSS position update at East=10.5, North=30.0
        y, mah = self.ekf.update_gnss(
            position_enu=np.array([10.5, 30.0, 0.0]),
            pos_std_m=2.0
        )

        cov_posterior = self.ekf.P[0, 0]
        self.assertLess(cov_posterior, cov_prior)
        self.assertAlmostEqual(self.ekf.x[0], 10.5, places=1)

    def test_ekf_nhc_update_constrains_lateral_velocity(self):
        """Test that NHC virtual measurement update dampens lateral velocity.
        
        Injects realistic accumulated velocity uncertainty (as would exist after
        several seconds of dead reckoning) so that the Kalman gain is large enough
        to produce a strong NHC correction in a single update.
        """
        # Simulate accumulated uncertainty after ~10s dead reckoning
        # Inflate velocity covariance to match real-world scenario
        self.ekf.P[3, 3] = 5.0 ** 2  # Ve std = 5 m/s
        self.ekf.P[4, 4] = 5.0 ** 2  # Vn std = 5 m/s

        # Inject unwanted East velocity component while heading North
        self.ekf.x[3] = 4.0  # Ve = 4.0 m/s (pure lateral slip heading North)
        self.ekf.x[4] = 10.0  # Vn = 10.0 m/s (forward speed)
        ve_before = abs(self.ekf.x[3])

        # Single NHC update with high covariance → strong correction
        y, mah = self.ekf.update_nhc_virtual_measurement()

        ve_after = abs(self.ekf.x[3])
        # East velocity (lateral slip) should be significantly reduced toward 0
        self.assertLess(ve_after, ve_before, "NHC update must reduce lateral velocity")
        self.assertLess(ve_after, 2.0, "Lateral velocity must be below 2.0 m/s after NHC correction")


class TestPhase5BlackoutHandler(unittest.TestCase):
    """Unit tests for Blackout Detector and Smooth Reconverger (FUSION-02, FUSION-03)."""

    def setUp(self):
        self.monitor = GNSSHealthMonitor(
            max_fix_interval_sec=0.5,
            max_horizontal_accuracy_m=15.0,
            mahalanobis_gate_threshold=16.27
        )
        self.reconverger = SmoothReconverger(
            reconvergence_half_life_sec=0.8,
            max_reconvergence_duration_sec=2.0
        )

    def test_sub_50ms_blackout_detection_on_signal_loss(self):
        """Test sub-50ms blackout detection when GNSS fix is lost."""
        t0 = 10.0
        # Valid fix at t=10s
        status_valid = self.monitor.evaluate_health(
            timestamp_s=t0,
            gnss_pos_enu=np.array([100.0, 200.0, 0.0]),
            horizontal_accuracy_m=2.5
        )
        self.assertFalse(status_valid.is_outage_detected)
        self.assertEqual(status_valid.current_mode, NavigationMode.NORMAL_FUSION)

        # Tunnel entry at t=10.1s (missing fix)
        t_outage = 10.1
        status_outage = self.monitor.evaluate_health(
            timestamp_s=t_outage,
            gnss_pos_enu=None,
            explicit_outage_flag=True
        )
        self.assertTrue(status_outage.is_outage_detected)
        self.assertLessEqual(status_outage.detection_latency_ms, 50.0)
        self.assertEqual(status_outage.current_mode, NavigationMode.OUTAGE_DEAD_RECKONING)

    def test_blackout_detection_on_accuracy_degradation(self):
        """Test blackout detection when reported GPS accuracy degrades in urban canyon."""
        status = self.monitor.evaluate_health(
            timestamp_s=1.0,
            gnss_pos_enu=np.array([50.0, 50.0, 0.0]),
            horizontal_accuracy_m=25.0 # Exceeds 15.0m threshold
        )
        self.assertTrue(status.is_outage_detected)
        self.assertIn("Accuracy Degraded", status.outage_reason)

    def test_mahalanobis_outlier_rejection(self):
        """Test rejection of corrupted GNSS position jumps via Mahalanobis residual."""
        status = self.monitor.evaluate_health(
            timestamp_s=1.0,
            gnss_pos_enu=np.array([50.0, 50.0, 0.0]),
            horizontal_accuracy_m=2.5,
            mahalanobis_residual_sq=50.0 # Exceeds 16.27 threshold
        )
        self.assertTrue(status.is_outage_detected)
        self.assertIn("Mahalanobis", status.outage_reason)

    def test_smooth_reconvergence_eliminates_position_jump(self):
        """Test exponential offset decay blending during GNSS restoration."""
        t_restore = 20.0
        gnss_pos = np.array([100.0, 200.0, 0.0])
        dr_pos = np.array([115.0, 210.0, 0.0]) # 18m offset at tunnel exit

        self.reconverger.trigger_reconvergence(t_restore, gnss_pos, dr_pos)
        self.assertTrue(self.reconverger.is_reconverging)

        # At t = t_restore (0.0s elapsed): output position should match DR position (no jump!)
        pos_t0, mag_t0 = self.reconverger.get_smoothed_position(t_restore, gnss_pos)
        np.testing.assert_allclose(pos_t0, dr_pos, atol=1e-4)

        # At t = t_restore + 2.0s (decay complete): output position should match GNSS position
        pos_t2, mag_t2 = self.reconverger.get_smoothed_position(t_restore + 2.5, gnss_pos)
        np.testing.assert_allclose(pos_t2, gnss_pos, atol=1e-4)
        self.assertFalse(self.reconverger.is_reconverging)


class TestPhase5Integration(unittest.TestCase):
    """Integration tests for master GNSSINSFusionEngine (FUSION-04)."""

    def setUp(self):
        self.syn_data = generate_synthetic_run(duration_s=60.0, dt=0.1, seed=42)
        self.network = RoadNetwork.create_from_route_waypoints(
            waypoints_enu=self.syn_data.enu_positions,
            subsegment_length_m=80.0
        )
        self.engine = GNSSINSFusionEngine(sampling_rate=10.0, road_network=self.network)

    def test_simulated_30s_tunnel_blackout_drift(self):
        """Simulate a 30s GNSS blackout (t=15s to t=45s) and verify drift < 10%."""
        n = self.syn_data.num_samples
        fused_positions = []
        outage_flags = []

        for i in range(n):
            t = self.syn_data.timestamps_s[i]
            # Simulate GNSS blackout from 15s to 45s (30s tunnel)
            is_blackout = (15.0 <= t <= 45.0)

            gnss_pos = None if is_blackout else self.syn_data.gps_lat # dummy
            gnss_enu = None if is_blackout else self.syn_data.enu_positions[i]

            out = self.engine.process_epoch(
                timestamp_s=t,
                accel_raw=self.syn_data.accel[i],
                gyro_raw=self.syn_data.gyro[i],
                gnss_pos_enu=gnss_enu,
                gnss_accuracy_m=2.5,
                ai_forward_speed=self.syn_data.gps_speed_mps[i],
                explicit_outage=is_blackout,
            )

            fused_positions.append(out.position_enu)
            outage_flags.append(out.is_outage)

        fused_positions = np.array(fused_positions)
        gt = self.syn_data.enu_positions

        # Calculate position error during blackout window (15s to 45s)
        blackout_mask = (self.syn_data.timestamps_s >= 15.0) & (self.syn_data.timestamps_s <= 45.0)
        blackout_gt = gt[blackout_mask]
        blackout_fused = fused_positions[blackout_mask]

        blackout_dist = float(np.sum(np.linalg.norm(np.diff(blackout_gt[:, :2], axis=0), axis=1)))
        final_err = float(np.linalg.norm(blackout_fused[-1, :2] - blackout_gt[-1, :2]))
        drift_pct = (final_err / blackout_dist) * 100.0

        # Verify drift is well below 10% target (FUSION-04)
        self.assertLess(drift_pct, 10.0)
        self.assertTrue(any(outage_flags))


if __name__ == "__main__":
    unittest.main()
