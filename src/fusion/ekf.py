"""
Extended Kalman Filter (EKF) Engine for GNSS+INS Sensor Fusion.
SIH Problem Statement ID: 26168 | ISRO

Implements FUSION-01:
9-State Extended Kalman Filter combining GNSS position/speed updates with continuous
IMU propagation and Non-Holonomic Constraint (NHC) virtual measurements.
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np


@dataclass
class EKFState:
    """Dataclass storing estimated navigation state and covariance."""
    timestamp_s: float
    position_enu: np.ndarray       # [East, North, Up] in meters
    velocity_enu: np.ndarray       # [Ve, Vn, Vu] in m/s
    heading_rad: float             # Heading azimuth in radians
    heading_deg: float             # Heading azimuth in degrees (0 = North, 90 = East)
    gyro_bias_rads: float          # Gyro Z-axis bias in rad/s
    accel_bias_mps2: float         # Accelerometer forward bias in m/s^2
    covariance: np.ndarray         # (9, 9) State estimation error covariance matrix P


class ExtendedKalmanFilter:
    """
    9-State Extended Kalman Filter for Land Vehicle Navigation.
    
    State Vector x = [p_E, p_N, p_U, v_E, v_N, v_U, heading, b_w, b_a]^T
    - Position (3): East, North, Up in local metric frame
    - Velocity (3): East, North, Up velocity
    - Heading (1): Yaw angle azimuth relative to North
    - Gyro Bias (1): Z-axis gyroscope drift
    - Accel Bias (1): Forward accelerometer bias
    """

    def __init__(
        self,
        process_noise_accel: float = 0.5,    # Accel process noise std (m/s^2)
        process_noise_gyro: float = 0.01,    # Gyro process noise std (rad/s)
        process_noise_bias_w: float = 1e-4,  # Gyro bias random walk std
        process_noise_bias_a: float = 1e-3,  # Accel bias random walk std
        gnss_pos_std: float = 2.5,           # Default GNSS position measurement std (m)
        gnss_vel_std: float = 0.3,           # Default GNSS velocity measurement std (m/s)
        nhc_lateral_std: float = 0.2,        # NHC lateral velocity virtual measurement std (m/s)
        nhc_vertical_std: float = 0.1,       # NHC vertical velocity virtual measurement std (m/s)
    ):
        self.q_a = process_noise_accel
        self.q_w = process_noise_gyro
        self.q_bw = process_noise_bias_w
        self.q_ba = process_noise_bias_a
        self.r_gnss_pos = gnss_pos_std
        self.r_gnss_vel = gnss_vel_std
        self.r_nhc_lat = nhc_lateral_std
        self.r_nhc_vert = nhc_vertical_std

        # State vector: [pE, pN, pU, vE, vN, vU, psi, bw, ba]
        self.x = np.zeros(9, dtype=np.float64)
        # Covariance matrix P (9x9)
        self.P = np.diag([
            10.0**2, 10.0**2, 5.0**2,   # Pos (E, N, U)
            1.0**2, 1.0**2, 0.5**2,     # Vel (E, N, U)
            np.radians(10.0)**2,        # Heading
            0.01**2,                     # Gyro bias
            0.1**2                       # Accel bias
        ]).astype(np.float64)

        self.last_timestamp: Optional[float] = None
        self.is_initialized = False

    def initialize(
        self,
        timestamp_s: float,
        position_enu: np.ndarray,
        velocity_enu: Optional[np.ndarray] = None,
        heading_deg: float = 0.0,
        initial_gyro_bias: float = 0.0,
        initial_accel_bias: float = 0.0,
    ):
        """Initializes state vector and timestamp."""
        self.x = np.zeros(9, dtype=np.float64)
        self.x[:3] = np.asarray(position_enu, dtype=np.float64)
        if velocity_enu is not None:
            self.x[3:6] = np.asarray(velocity_enu, dtype=np.float64)
        self.x[6] = float(np.radians(heading_deg))
        self.x[7] = float(initial_gyro_bias)
        self.x[8] = float(initial_accel_bias)

        self.last_timestamp = float(timestamp_s)
        self.is_initialized = True

    def predict(
        self,
        timestamp_s: float,
        accel_forward_mps2: float,
        gyro_yaw_rate_rads: float,
        dt: Optional[float] = None,
    ) -> EKFState:
        """
        EKF Time Propagation / Prediction Step using IMU measurements.
        
        Args:
            timestamp_s: Current epoch timestamp in seconds.
            accel_forward_mps2: Forward longitudinal acceleration in vehicle frame (a_x^v).
            gyro_yaw_rate_rads: Yaw angular velocity from gyroscope in vehicle frame (w_z^v).
            dt: Optional time step in seconds.
        """
        if not self.is_initialized:
            self.initialize(timestamp_s, np.zeros(3), heading_deg=0.0)

        if dt is None:
            dt = timestamp_s - (self.last_timestamp if self.last_timestamp is not None else timestamp_s)
            if dt <= 0.0 or dt > 1.0:
                dt = 0.1

        self.last_timestamp = float(timestamp_s)

        # Unpack state
        pe, pn, pu, ve, vn, vu, psi, bw, ba = self.x

        # Correct sensor readings with estimated biases
        w_corr = gyro_yaw_rate_rads - bw
        a_corr = accel_forward_mps2 - ba

        # State propagation
        psi_new = (psi + w_corr * dt + np.pi) % (2.0 * np.pi) - np.pi
        
        # Accelerations in East-North navigation frame
        sin_h = np.sin(psi_new)
        cos_h = np.cos(psi_new)
        ae = a_corr * sin_h
        an = a_corr * cos_h
        au = 0.0 # Vertical acceleration handled separately or assumed zero

        ve_new = ve + ae * dt
        vn_new = vn + an * dt
        vu_new = vu # damped

        pe_new = pe + ve * dt + 0.5 * ae * dt**2
        pn_new = pn + vn * dt + 0.5 * an * dt**2
        pu_new = pu + vu * dt

        # Update state vector
        self.x = np.array([pe_new, pn_new, pu_new, ve_new, vn_new, vu_new, psi_new, bw, ba], dtype=np.float64)

        # Compute Jacobian F = df/dx (9x9)
        F = np.eye(9, dtype=np.float64)
        F[0, 3] = dt  # d(pe)/d(ve)
        F[1, 4] = dt  # d(pn)/d(vn)
        F[2, 5] = dt  # d(pu)/d(vu)

        F[0, 6] = 0.5 * a_corr * cos_h * (dt**2)  # d(pe)/d(psi)
        F[1, 6] = -0.5 * a_corr * sin_h * (dt**2) # d(pn)/d(psi)
        F[3, 6] = a_corr * cos_h * dt             # d(ve)/d(psi)
        F[4, 6] = -a_corr * sin_h * dt            # d(vn)/d(psi)

        F[6, 7] = -dt                            # d(psi)/d(bw)
        F[3, 8] = -sin_h * dt                    # d(ve)/d(ba)
        F[4, 8] = -cos_h * dt                    # d(vn)/d(ba)

        # Process noise covariance Q (9x9)
        Q = np.diag([
            (0.5 * self.q_a * dt**2)**2, (0.5 * self.q_a * dt**2)**2, 0.01,
            (self.q_a * dt)**2, (self.q_a * dt)**2, 0.01,
            (self.q_w * dt)**2,
            (self.q_bw * dt)**2,
            (self.q_ba * dt)**2
        ]).astype(np.float64)

        # Propagate covariance P = F P F^T + Q
        self.P = F @ self.P @ F.T + Q

        return self.get_state(timestamp_s)

    def update_gnss(
        self,
        position_enu: np.ndarray,
        velocity_enu: Optional[np.ndarray] = None,
        pos_std_m: Optional[float] = None,
        vel_std_mps: Optional[float] = None,
    ) -> Tuple[np.ndarray, float]:
        """
        EKF Measurement Correction Step using GNSS Position and Velocity.
        
        Args:
            position_enu: (3,) [East, North, Up] position from GNSS fix in meters.
            velocity_enu: Optional (2,) or (3,) [Ve, Vn, Vu] velocity from GNSS in m/s.
            pos_std_m: Reported GNSS horizontal position accuracy in meters.
            vel_std_mps: Reported GNSS velocity accuracy in m/s.
            
        Returns:
            (innovation_y, mahalanobis_distance_sq)
        """
        if not self.is_initialized:
            return np.zeros(3), 0.0

        r_pos = (pos_std_m if pos_std_m is not None else self.r_gnss_pos)**2
        r_vel = (vel_std_mps if vel_std_mps is not None else self.r_gnss_vel)**2

        if velocity_enu is not None:
            # 5D Measurement: [pE, pN, pU, vE, vN]
            z = np.array([
                position_enu[0], position_enu[1], position_enu[2],
                velocity_enu[0], velocity_enu[1]
            ], dtype=np.float64)

            H = np.zeros((5, 9), dtype=np.float64)
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0
            H[3, 3] = 1.0
            H[4, 4] = 1.0

            R = np.diag([r_pos, r_pos, r_pos * 2.0, r_vel, r_vel]).astype(np.float64)
        else:
            # 3D Position Measurement: [pE, pN, pU]
            z = np.array(position_enu[:3], dtype=np.float64)

            H = np.zeros((3, 9), dtype=np.float64)
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0

            R = np.diag([r_pos, r_pos, r_pos * 2.0]).astype(np.float64)

        # Innovation residual y = z - H x
        y = z - H @ self.x
        # Innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + R
        
        # Mahalanobis distance sq d_mah^2 = y^T S^-1 y
        try:
            S_inv = np.linalg.inv(S)
            d_mah_sq = float(y.T @ S_inv @ y)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
            d_mah_sq = float(y.T @ S_inv @ y)

        # Kalman Gain K = P H^T S^-1
        K = self.P @ H.T @ S_inv

        # State update x = x + K y
        self.x += K @ y
        # Wrap heading angle to [-pi, pi]
        self.x[6] = (self.x[6] + np.pi) % (2.0 * np.pi) - np.pi

        # Joseph form covariance update P = (I - K H) P (I - K H)^T + K R K^T
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return y, d_mah_sq

    def update_nhc_virtual_measurement(self) -> Tuple[np.ndarray, float]:
        """
        EKF Virtual Measurement Correction enforcing Non-Holonomic Constraints (NHC):
        v_y_body = -v_E * cos(psi) + v_N * sin(psi) ~ 0
        v_z_body = v_U ~ 0
        """
        if not self.is_initialized:
            return np.zeros(2), 0.0

        ve, vn, vu = self.x[3], self.x[4], self.x[5]
        psi = self.x[6]

        sin_h = np.sin(psi)
        cos_h = np.cos(psi)

        # Lateral velocity v_y^b
        v_lat = -ve * cos_h + vn * sin_h
        v_vert = vu

        # Virtual measurement z = [0, 0]
        y = np.array([0.0 - v_lat, 0.0 - v_vert], dtype=np.float64)

        # Measurement Jacobian H_nhc (2x9)
        H = np.zeros((2, 9), dtype=np.float64)
        H[0, 3] = -cos_h                             # d(v_lat)/d(ve)
        H[0, 4] = sin_h                              # d(v_lat)/d(vn)
        H[0, 6] = ve * sin_h + vn * cos_h            # d(v_lat)/d(psi)
        H[1, 5] = 1.0                                # d(v_vert)/d(vu)

        R = np.diag([self.r_nhc_lat**2, self.r_nhc_vert**2]).astype(np.float64)

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)

        d_mah_sq = float(y.T @ S_inv @ y)
        K = self.P @ H.T @ S_inv

        self.x += K @ y
        self.x[6] = (self.x[6] + np.pi) % (2.0 * np.pi) - np.pi

        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        return y, d_mah_sq

    def get_state(self, timestamp_s: float = 0.0) -> EKFState:
        """Returns current EKF state object."""
        return EKFState(
            timestamp_s=float(timestamp_s),
            position_enu=self.x[:3].copy(),
            velocity_enu=self.x[3:6].copy(),
            heading_rad=float(self.x[6]),
            heading_deg=float((np.degrees(self.x[6]) + 360.0) % 360.0),
            gyro_bias_rads=float(self.x[7]),
            accel_bias_mps2=float(self.x[8]),
            covariance=self.P.copy(),
        )
