"""
Non-Holonomic Constraints (NHC) and Kinematic Dead Reckoning Module.
SIH Problem Statement ID: 26168 | ISRO

Implements MAP-01:
Land vehicle kinematic boundaries enforce zero lateral velocity (v_y ~ 0)
and zero vertical velocity (v_z ~ 0) in the vehicle body frame during regular driving.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Union
import numpy as np

from src.preprocessing.alignment import VehicleFrameAligner, AlignmentResult
from src.preprocessing.filters import VibrationFilter
from src.preprocessing.zupt import ZeroVelocityDetector


@dataclass
class NHCState:
    """Navigation state at a specific epoch with NHC applied."""
    timestamp_s: float
    position_enu: np.ndarray      # [East, North, Up] in meters
    velocity_enu: np.ndarray      # [Ve, Vn, Vu] in m/s
    heading_deg: float            # Azimuth in degrees (0 = North, 90 = East)
    heading_rad: float            # Azimuth in radians
    speed_mps: float              # Forward speed along longitudinal axis
    lateral_velocity_mps: float   # Residual lateral velocity before constraint
    vertical_velocity_mps: float  # Residual vertical velocity before constraint
    is_stationary: bool           # Whether ZUPT stop is active


class NonHolonomicConstraint:
    """
    Non-Holonomic Constraint (NHC) mathematical operator.
    
    In a standard land vehicle, wheels prevent sliding sideways (perpendicular to
    heading) and the chassis stays on the road surface:
        v_y_body ~ 0 (lateral velocity)
        v_z_body ~ 0 (vertical velocity)
        
    This class handles projection between Navigation (ENU) frame and Vehicle frame,
    and enforces hard or soft constraints to eliminate transverse drift.
    """

    def __init__(
        self,
        lateral_constraint_weight: float = 1.0,
        vertical_constraint_weight: float = 1.0,
        max_allowable_lateral_slip_mps: float = 2.0,
    ):
        """
        Args:
            lateral_constraint_weight: 1.0 = hard constraint (v_lat = 0),
                                      0.0 = unconstrained,
                                      (0, 1) = soft damping.
            vertical_constraint_weight: 1.0 = hard constraint (v_up = 0).
            max_allowable_lateral_slip_mps: Maximum expected slip during sharp turns.
        """
        self.w_lat = float(np.clip(lateral_constraint_weight, 0.0, 1.0))
        self.w_vert = float(np.clip(vertical_constraint_weight, 0.0, 1.0))
        self.max_slip = max_allowable_lateral_slip_mps

    @staticmethod
    def forward_velocity_to_enu(v_fwd: Union[float, np.ndarray], heading_rad: Union[float, np.ndarray]) -> np.ndarray:
        """
        Converts scalar forward vehicle speed and heading azimuth into ENU velocity.
        
        Navigation Frame (ENU):
        - East (X_enu):  v_e = v_fwd * sin(heading)
        - North (Y_enu): v_n = v_fwd * cos(heading)
        - Up (Z_enu):    v_u = 0.0
        
        Args:
            v_fwd: Forward velocity in m/s (scalar or (N,) array).
            heading_rad: Heading angle in radians measured clockwise from North.
            
        Returns:
            (3,) or (N, 3) array of [v_east, v_north, v_up].
        """
        v_fwd_arr = np.asarray(v_fwd)
        head_arr = np.asarray(heading_rad)
        
        v_east = v_fwd_arr * np.sin(head_arr)
        v_north = v_fwd_arr * np.cos(head_arr)
        v_up = np.zeros_like(v_east)
        
        if v_fwd_arr.ndim == 0:
            return np.array([float(v_east), float(v_north), 0.0])
        return np.column_stack([v_east, v_north, v_up])

    @staticmethod
    def enu_to_body_velocity(v_enu: np.ndarray, heading_rad: float) -> Tuple[float, float, float]:
        """
        Decomposes an ENU velocity vector [v_e, v_n, v_u] into vehicle body components:
        - v_forward: along vehicle longitudinal axis
        - v_lateral: along vehicle lateral axis (perpendicular to heading)
        - v_vertical: along vertical Up axis
        
        Args:
            v_enu: [v_east, v_north, v_up] in m/s.
            heading_rad: Vehicle heading in radians clockwise from North.
            
        Returns:
            (v_forward, v_lateral, v_vertical)
        """
        v_e, v_n, v_u = float(v_enu[0]), float(v_enu[1]), float(v_enu[2])
        sin_h = np.sin(heading_rad)
        cos_h = np.cos(heading_rad)
        
        # Longitudinal forward projection
        v_forward = v_e * sin_h + v_n * cos_h
        # Lateral transverse projection (orthogonal to heading)
        v_lateral = -v_e * cos_h + v_n * sin_h
        v_vertical = v_u
        
        return v_forward, v_lateral, v_vertical

    def apply_nhc(
        self,
        v_enu: np.ndarray,
        heading_rad: float,
        override_forward_speed: Optional[float] = None
    ) -> Tuple[np.ndarray, float, float]:
        """
        Applies Non-Holonomic Constraints to an estimated ENU velocity vector.
        
        Decomposes velocity into forward, lateral, and vertical components,
        suppresses lateral and vertical components according to constraint weights,
        and reconstructs the constrained ENU velocity vector.
        
        Args:
            v_enu: (3,) [v_east, v_north, v_up] in m/s.
            heading_rad: Vehicle heading azimuth in radians.
            override_forward_speed: Optional externally estimated forward speed
                                    (e.g., from SpeedNet deep learning model).
                                    
        Returns:
            (v_enu_constrained (3,), raw_v_lat, raw_v_vert)
        """
        v_fwd, v_lat, v_vert = self.enu_to_body_velocity(v_enu, heading_rad)
        
        # If external forward speed is provided, use it
        if override_forward_speed is not None:
            effective_fwd = max(0.0, float(override_forward_speed))
        else:
            effective_fwd = max(0.0, v_fwd)  # vehicles generally don't drive in reverse on highway
            
        # Apply constraint weighting
        # w_lat = 1.0 -> constrained_v_lat = 0
        constrained_v_lat = (1.0 - self.w_lat) * v_lat
        constrained_v_vert = (1.0 - self.w_vert) * v_vert
        
        # Clamp lateral slip if soft constraint
        if self.w_lat < 1.0:
            constrained_v_lat = float(np.clip(constrained_v_lat, -self.max_slip, self.max_slip))
            
        # Reconstruct ENU velocity from constrained body components
        sin_h = np.sin(heading_rad)
        cos_h = np.cos(heading_rad)
        
        # Invert the 2D projection:
        # [v_e] = [sin_h  -cos_h] [v_fwd]
        # [v_n]   [cos_h   sin_h] [v_lat]
        v_e_new = effective_fwd * sin_h - constrained_v_lat * cos_h
        v_n_new = effective_fwd * cos_h + constrained_v_lat * sin_h
        v_u_new = constrained_v_vert
        
        return np.array([v_e_new, v_n_new, v_u_new]), v_lat, v_vert


class NHCDeadReckoning:
    """
    Complete Dead Reckoning navigation engine integrating:
    1. Phone-to-vehicle attitude auto-alignment (Phase 2)
    2. Vibration filtering & pothole damping (Phase 2)
    3. Zero-Velocity Update (ZUPT) and bias reset (Phase 2)
    4. Forward speed integration or Phase 3 AI SpeedNet injection
    5. Kinematic Non-Holonomic Constraints (Phase 4, MAP-01)
    """

    def __init__(
        self,
        sampling_rate: float = 10.0,
        enable_filtering: bool = True,
        enable_zupt: bool = True,
        enable_alignment: bool = True,
        lateral_constraint_weight: float = 1.0,
        vertical_constraint_weight: float = 1.0,
        gravity_nominal: float = 9.80665,
    ):
        self.sampling_rate = sampling_rate
        self.enable_filtering = enable_filtering
        self.enable_zupt = enable_zupt
        self.enable_alignment = enable_alignment
        self.g = gravity_nominal

        self.aligner = VehicleFrameAligner(sampling_rate=sampling_rate, gravity_nominal=gravity_nominal)
        self.filter = VibrationFilter(sampling_rate=sampling_rate, cutoff_freq=3.0)
        self.zupt_detector = ZeroVelocityDetector(sampling_rate=sampling_rate, nominal_gravity=gravity_nominal)
        self.nhc = NonHolonomicConstraint(
            lateral_constraint_weight=lateral_constraint_weight,
            vertical_constraint_weight=vertical_constraint_weight
        )

    def calibrate(
        self,
        accel_raw: np.ndarray,
        gyro_raw: np.ndarray,
        initial_static_sec: float = 2.0,
        initial_motion_sec: float = 5.0
    ) -> AlignmentResult:
        """Calibrates mounting attitude (pitch/roll leveling and forward axis)."""
        n_static = max(10, int(self.sampling_rate * initial_static_sec))
        static_accel = accel_raw[:n_static]
        static_gyro = gyro_raw[:n_static]

        self.aligner.calibrate_static(static_accel, static_gyro)

        n_motion = min(len(accel_raw), int(self.sampling_rate * (initial_static_sec + initial_motion_sec)))
        if n_motion > n_static:
            motion_accel = accel_raw[n_static:n_motion]
            motion_gyro_z = gyro_raw[n_static:n_motion, 2]
            self.aligner.align_forward_axis(motion_accel, gyro_z=motion_gyro_z)

        return self.aligner.get_result()

    def propagate(
        self,
        timestamps: np.ndarray,
        accel_raw: np.ndarray,
        gyro_raw: np.ndarray,
        initial_enu_pos: Optional[np.ndarray] = None,
        initial_heading_deg: Optional[float] = None,
        initial_velocity: Optional[np.ndarray] = None,
        ai_forward_speeds: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Propagates dead reckoning trajectory with Non-Holonomic Constraints.
        
        Args:
            timestamps: (N,) elapsed seconds.
            accel_raw: (N, 3) raw accelerometer readings [m/s^2].
            gyro_raw: (N, 3) raw gyroscope readings [rad/s].
            initial_enu_pos: (3,) initial [East, North, Up] position in meters.
            initial_heading_deg: Initial heading in degrees (0 = North, 90 = East).
            initial_velocity: (3,) initial [Ve, Vn, Vu] velocity.
            ai_forward_speeds: Optional (N,) array of forward velocities from Phase 3 SpeedNet.
            
        Returns:
            positions: (N, 3) [East, North, Up] in meters.
            velocities: (N, 3) [Ve, Vn, Vu] in m/s.
            headings_deg: (N,) heading in degrees.
            stationary_mask: (N,) boolean mask of detected stops.
        """
        n = len(timestamps)
        positions = np.zeros((n, 3), dtype=np.float64)
        velocities = np.zeros((n, 3), dtype=np.float64)
        headings = np.zeros(n, dtype=np.float64)
        stationary_mask = np.zeros(n, dtype=bool)

        if initial_enu_pos is not None:
            positions[0] = np.asarray(initial_enu_pos, dtype=np.float64)
        if initial_velocity is not None:
            velocities[0] = np.asarray(initial_velocity, dtype=np.float64)
        if initial_heading_deg is not None:
            headings[0] = float(np.radians(initial_heading_deg))

        # 1. Preprocessing: Digital Vibration Filtering
        if self.enable_filtering:
            accel_clean = self.filter.filter_batch(accel_raw, clamp_potholes=True)
            gyro_clean = self.filter.filter_batch(gyro_raw, clamp_potholes=False)
        else:
            accel_clean = accel_raw.copy()
            gyro_clean = gyro_raw.copy()

        # 2. Coordinate Alignment into Vehicle Frame
        if self.enable_alignment:
            accel_veh, gyro_veh = self.aligner.transform_imu(accel_clean, gyro_clean)
        else:
            accel_veh = accel_clean.copy()
            accel_veh[:, 2] -= self.g
            gyro_veh = gyro_clean.copy()

        # 3. ZUPT Stationary Detection & Gyroscope Bias Estimation
        if self.enable_zupt:
            stationary_mask, gyro_biases = self.zupt_detector.detect_batch(accel_raw, gyro_raw, timestamps)
        else:
            gyro_biases = np.zeros((n, 3))

        # 4. Sequential Dead Reckoning Propagation with NHC
        current_heading = headings[0]
        current_vel = velocities[0].copy()
        current_pos = positions[0].copy()
        current_fwd_speed = float(np.linalg.norm(current_vel[:2]))

        for i in range(1, n):
            dt = timestamps[i] - timestamps[i - 1]
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / self.sampling_rate

            is_stopped = stationary_mask[i]
            if ai_forward_speeds is not None and ai_forward_speeds[i] > 0.5:
                # Cruising at speed cannot be stationary
                is_stopped = False

            if is_stopped:
                current_vel = np.zeros(3, dtype=np.float64)
                current_fwd_speed = 0.0
                # Heading remains frozen during complete stop
            else:
                # Integrate heading from yaw rate (Z-axis of vehicle frame)
                omega_z = gyro_veh[i, 2] - gyro_biases[i, 2]
                current_heading += omega_z * dt
                # Normalize heading to [-pi, pi]
                current_heading = (current_heading + np.pi) % (2.0 * np.pi) - np.pi

                # Forward velocity update
                if ai_forward_speeds is not None:
                    # Direct AI SpeedNet prediction
                    current_fwd_speed = max(0.0, float(ai_forward_speeds[i]))
                else:
                    # Integrate forward acceleration (a_x^v) in vehicle frame
                    a_forward = accel_veh[i, 0]
                    current_fwd_speed = max(0.0, current_fwd_speed + a_forward * dt)

                # Convert forward speed + heading to ENU velocity under NHC
                # v_east = v_fwd * sin(heading), v_north = v_fwd * cos(heading), v_up = 0
                current_vel = NonHolonomicConstraint.forward_velocity_to_enu(
                    current_fwd_speed, current_heading
                )

            # Trapezoidal position integration
            current_pos += current_vel * dt

            positions[i] = current_pos.copy()
            velocities[i] = current_vel.copy()
            headings[i] = current_heading

        # Convert headings to degrees in [0, 360)
        headings_deg = (np.degrees(headings) + 360.0) % 360.0

        return positions, velocities, headings_deg, stationary_mask
