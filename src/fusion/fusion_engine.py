"""
Master GNSS+INS Fusion Engine & Integrated Navigation System.
SIH Problem Statement ID: 26168 | ISRO

Combines:
- Extended Kalman Filter (EKF) state estimation
- Sub-50ms GNSS Health & Outage Detection
- AI-Enhanced Dead Reckoning (SpeedNet + NHC)
- Offline HMM Road Map Matching
- Exponential Smooth Reconvergence Filter
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

from src.fusion.ekf import ExtendedKalmanFilter, EKFState
from src.fusion.blackout_handler import (
    GNSSHealthMonitor,
    GNSSHealthStatus,
    NavigationMode,
    SmoothReconverger,
)
from src.kinematics.nhc import NHCDeadReckoning
from src.map_matching.hmm_matcher import HMMMapMatcher
from src.map_matching.road_network import RoadNetwork


@dataclass
class FusedNavigationOutput:
    """Master navigation state output at each epoch."""
    timestamp_s: float
    position_enu: np.ndarray          # [East, North, Up] in meters
    velocity_enu: np.ndarray          # [Ve, Vn, Vu] in m/s
    heading_deg: float                # Vehicle heading azimuth in degrees
    navigation_mode: NavigationMode   # NORMAL_FUSION, OUTAGE_DEAD_RECKONING, RECONVERGING
    is_outage: bool                   # True if currently in GNSS blackout
    outage_duration_s: float          # Duration of active blackout in seconds
    matched_road_name: str            # Name of matched road link if available
    cross_track_error_m: float        # Perpendicular distance to road link
    detection_latency_ms: float       # Outage detection latency (< 50 ms)


class GNSSINSFusionEngine:
    """
    Complete Real-Time Navigation & Fusion Engine for IDR-INS.
    Handles seamless switching between GNSS+INS EKF state estimation and pure AI-Dead Reckoning.
    """

    def __init__(
        self,
        sampling_rate: float = 10.0,
        road_network: Optional[RoadNetwork] = None,
        max_gnss_accuracy_m: float = 15.0,
        reconvergence_duration_sec: float = 2.0,
    ):
        self.sampling_rate = sampling_rate
        self.ekf = ExtendedKalmanFilter()
        self.health_monitor = GNSSHealthMonitor(max_horizontal_accuracy_m=max_gnss_accuracy_m)
        self.reconverger = SmoothReconverger(max_reconvergence_duration_sec=reconvergence_duration_sec)
        self.nhc_engine = NHCDeadReckoning(sampling_rate=sampling_rate)

        self.road_network = road_network
        self.map_matcher = HMMMapMatcher(road_network) if road_network is not None else None

        self.outage_start_timestamp: Optional[float] = None
        self.dr_last_position: Optional[np.ndarray] = None
        self.last_matched_segment_id: Optional[str] = None
        self.previous_mode = NavigationMode.NORMAL_FUSION

    def process_epoch(
        self,
        timestamp_s: float,
        accel_raw: np.ndarray,
        gyro_raw: np.ndarray,
        gnss_pos_enu: Optional[np.ndarray] = None,
        gnss_vel_enu: Optional[np.ndarray] = None,
        gnss_accuracy_m: float = 2.5,
        ai_forward_speed: Optional[float] = None,
        explicit_outage: bool = False,
    ) -> FusedNavigationOutput:
        """
        Processes a single navigation epoch (10-200 Hz).
        
        Args:
            timestamp_s: Epoch timestamp in seconds.
            accel_raw: (3,) raw accelerometer readings [m/s^2].
            gyro_raw: (3,) raw gyroscope readings [rad/s].
            gnss_pos_enu: Optional (3,) GNSS fix [East, North, Up] in meters.
            gnss_vel_enu: Optional (2,) or (3,) GNSS velocity [Ve, Vn, Vu] in m/s.
            gnss_accuracy_m: Reported GPS horizontal accuracy in meters.
            ai_forward_speed: Optional predicted forward speed from Phase 3 SpeedNet.
            explicit_outage: Optional hardware signal loss flag.
            
        Returns:
            FusedNavigationOutput with optimal position, mode, and road link.
        """
        # 1. Transform raw IMU to vehicle frame
        accel_veh, gyro_veh = self.nhc_engine.aligner.transform_imu(accel_raw, gyro_raw)
        a_fwd = accel_veh[0]
        w_z = gyro_veh[2]

        # 2. EKF Prediction Step
        ekf_state = self.ekf.predict(
            timestamp_s=timestamp_s,
            accel_forward_mps2=a_fwd,
            gyro_yaw_rate_rads=w_z,
        )

        # 3. Virtual Measurement Update: NHC (v_y ~ 0, v_z ~ 0)
        y_nhc, mah_nhc = self.ekf.update_nhc_virtual_measurement()

        # 4. Evaluate GNSS Fix Health
        mah_gnss_sq = 0.0
        if gnss_pos_enu is not None:
            # Preliminary EKF update to get innovation residual
            y_gnss, mah_gnss_sq = self.ekf.update_gnss(
                position_enu=gnss_pos_enu,
                velocity_enu=gnss_vel_enu,
                pos_std_m=gnss_accuracy_m,
            )

        health = self.health_monitor.evaluate_health(
            timestamp_s=timestamp_s,
            gnss_pos_enu=gnss_pos_enu,
            horizontal_accuracy_m=gnss_accuracy_m,
            mahalanobis_residual_sq=mah_gnss_sq,
            explicit_outage_flag=explicit_outage,
        )

        current_mode = health.current_mode
        outage_duration = 0.0

        # Handle Mode Transitions
        if health.is_outage_detected:
            if self.outage_start_timestamp is None:
                self.outage_start_timestamp = timestamp_s

            outage_duration = timestamp_s - self.outage_start_timestamp

            # AI-Dead Reckoning Mode
            if self.dr_last_position is None:
                self.dr_last_position = ekf_state.position_enu.copy()

            # Propagate forward using SpeedNet / NHC
            heading_rad = ekf_state.heading_rad
            speed = ai_forward_speed if ai_forward_speed is not None else float(np.linalg.norm(ekf_state.velocity_enu[:2]))
            
            dt = 1.0 / self.sampling_rate
            ve_dr = speed * np.sin(heading_rad)
            vn_dr = speed * np.cos(heading_rad)
            
            self.dr_last_position[0] += ve_dr * dt
            self.dr_last_position[1] += vn_dr * dt

            raw_pos = self.dr_last_position.copy()
            raw_vel = np.array([ve_dr, vn_dr, 0.0])
            raw_heading = ekf_state.heading_deg

        else:
            # Healthy GNSS Fix Available
            if self.previous_mode == NavigationMode.OUTAGE_DEAD_RECKONING and self.dr_last_position is not None:
                # Trigger smooth reconvergence
                self.reconverger.trigger_reconvergence(
                    start_timestamp_s=timestamp_s,
                    gnss_pos_enu=ekf_state.position_enu,
                    dead_reckoned_pos_enu=self.dr_last_position
                )
                current_mode = NavigationMode.RECONVERGING

            self.outage_start_timestamp = None
            self.dr_last_position = None

            raw_pos = ekf_state.position_enu.copy()
            raw_vel = ekf_state.velocity_enu.copy()
            raw_heading = ekf_state.heading_deg

        # Apply Smooth Reconvergence Blending if exiting blackout
        if current_mode == NavigationMode.RECONVERGING or self.reconverger.is_reconverging:
            raw_pos, _ = self.reconverger.get_smoothed_position(timestamp_s, raw_pos)
            if not self.reconverger.is_reconverging:
                current_mode = NavigationMode.NORMAL_FUSION

        self.previous_mode = current_mode

        # 5. Offline Map Matching Snapping
        matched_road_name = "Unmatched"
        cte = 0.0
        output_pos = raw_pos.copy()

        if self.map_matcher is not None:
            snapped, seg_id, cte = self.map_matcher.snap_online_point(
                point_enu=raw_pos,
                vehicle_heading_deg=raw_heading,
                last_matched_segment_id=self.last_matched_segment_id
            )
            if seg_id != "none" and seg_id in self.road_network.segments:
                self.last_matched_segment_id = seg_id
                matched_road_name = self.road_network.segments[seg_id].name
                # If in blackout, map matching provides strong lane constraint
                if health.is_outage_detected:
                    output_pos = snapped.copy()

        return FusedNavigationOutput(
            timestamp_s=timestamp_s,
            position_enu=output_pos,
            velocity_enu=raw_vel,
            heading_deg=raw_heading,
            navigation_mode=current_mode,
            is_outage=health.is_outage_detected,
            outage_duration_s=outage_duration,
            matched_road_name=matched_road_name,
            cross_track_error_m=cte,
            detection_latency_ms=health.detection_latency_ms,
        )
