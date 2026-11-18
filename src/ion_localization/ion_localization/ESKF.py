"""
-----WORK IN PROGRESS-----

This node is responsible for localization of the vehicle using sensor data from the IMU, GPS, and wheel encoders. It subscribes to noise_node.py for
noise injected sensor data and publishes the estimated vehicle pose to the state_estimate topic.

The localization algorithm is based on an Extended Kalman Filter, that uses IMU data predict the vehicle state and GPS and wheel encoder data to
correct the state estimate. Furthermore, the wheel encoder data will be subjected to a Fuzzy Logic algorithm to detect wheel slip and only consider
wheel encoder data for the correction step if no wheel slip is detected.

Later, the localization node will also consider LiDAR data to produce a first run map of the environment. Together with the EKF this node will 
constitute a SLAM algorithm that will be used to localize the vehicle in the environment.

Much of this module is based on the work presented by Joan Sola in the paper "Quaternion kinematics for the error-state Kalman filter".
"""

# ------------------------------ #
# ---------- Imports ----------- #
# ------------------------------ #
import numpy as np
from dataclasses import dataclass


# ------------------------------ #
# - Quaternion Helper Functions- #
# ------------------------------ #

# Quaternion Multiplication
def q_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])


# Quaternion Conjugate
def q_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


# Normalization of the Quaternion
def q_normalize(q):
    q = q / np.linalg.norm(q)
    return q


# Exponential Representation
def q_exponential(rotation_vector):
    theta = np.linalg.norm(rotation_vector)
    if theta < 1e-6:
        return np.array([1, 0, 0, 0])
    axis = rotation_vector / theta
    return np.concatenate([[np.cos(theta / 2)], np.sin(theta / 2) * axis])


# Inverse Exponential
def q_logarithm(q):
    if q[0] < 0:
        q = -q
    n = np.linalg.norm(q[1:])
    if n < 1e-12:
        return 2*q[1:]
    return 2 * np.arctan2(n, q[0]) * q[1:] / n


# Transform Quaternion to Rotation Matrix
def q2R(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return R


# Transform Quaternion to Euler Angles
def q2euler(q):
    R = q2R(q)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arcsin(-R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


# Transform Euler Angles to Quaternion
def euler2q(roll, pitch, yaw):
    qx = q_exponential([roll, 0, 0])
    qy = q_exponential([0, pitch, 0])
    qz = q_exponential([0, 0, yaw])
    return q_normalize(q_multiply(q_multiply(qz, qy), qx))


# Create Scew Symmetric Matrix
def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


# Transform Rotation Vector to Rotation Matrix
def r2R(r):
    r = np.array(r)
    theta = np.linalg.norm(r)
    K = skew(r)
    if theta < 1e-8:
        return np.eye(3) + K + 0.5*K @ K
    return np.eye(3) + (np.sin(theta)/theta)*K + ((1-np.cos(theta))/theta**2)*K @ K


# ------------------------------ #
# ----- Tunable Parameters ----- #
# ------------------------------ #
@dataclass
class ESKFParams:
    accel_noise: float = 0.05
    gyro_noise: float = 0.02
    accel_bias_walk: float = 0.0001
    gyro_bias_walk: float = 0.00001
    gravity: tuple = (0, 0, -9.81)
    init_pos: float = 1.0
    init_vel: float = 0.5
    init_roll: float = 0.02
    init_pitch: float = 0.02
    init_yaw: float = 0.5
    init_accel_bias: float = 0.1
    init_gyro_bias: float = 0.005


# -------------------------------------- #
# - Error State Extended Kalman Filter - #
# -------------------------------------- #
class ESKF:
    def __init__(self, params: ESKFParams):
        self.params = params
        self.g = np.array(self.params.gravity)  # World Frame Gravity Vector
        self.p = np.zeros(3)                    # World Frame Position Vector
        self.v = np.zeros(3)                    # World Frame Velocity Vector
        self.q = np.array([1, 0, 0, 0])         # World Frame Orientation Quaternion
        self.a_b = np.zeros(3)                  # Accelerometer Bias
        self.w_b = np.zeros(3)                  # Gyroscope Bias
        self.P = np.eye(15)                     # Error State Covariance Matrix
        self._a_m = np.zeros(3)                 # IMU Acceleration Measurement
        self._w_m = np.zeros(3)                 # IMU Angular Velocity Measurement


    # Initialization
    def initialize(self, a0, w0, yaw0=0, p0=(0, 0, 0)):
        a = np.array(a0)
        w = np.array(w0)
        roll = np.arctan2(a[1], a[2])
        pitch = np.arctan2(-a[0], np.hypot(a[1], a[2]))
        yaw = yaw0
        self.q = euler2q(roll, pitch, yaw)
        self.p = np.array(p0, dtype=float)
        self.v = np.zeros(3)
        self.a_b = np.zeros(3)
        self.w_b = np.zeros(3)
        self._a_m = a
        self._w_m = w
        params = self.params
        self.P = np.diag(
            np.concatenate([
                np.full(3, params.init_pos**2),
                np.full(3, params.init_vel**2),
                [params.init_roll**2, params.init_pitch**2, params.init_yaw**2],
                np.full(3, params.init_accel_bias**2),
                np.full(3, params.init_gyro_bias**2)
            ])
        )
        
    # Build the Transition Matrix F Used to Predict the Propagation of the Error States
    # dx(k+1) = F*dx(k)
    def transition_matrix(self, a_m, w_m, dt):
        R = q2R(self.q)
        a = np.array(a_m) - self.a_b
        w = np.array(w_m) - self.w_b
        I3 = np.eye(3)
        F = np.eye(15)

        F[0:3, 3:6] = I3*dt
        F[0:3, 6:9] = -0.5*R @ skew(a)*dt**2
        F[0:3, 9:12] = -0.5*R*dt**2
        F[3:6, 6:9] = -R @ skew(a)*dt
        F[3:6, 9:12] = -R*dt
        F[6:9, 6:9] = r2R(w*dt).T
        F[6:9, 12:15] = -I3*dt

        return F


    # Build the Process Covariance Matrix
    def covariance_matrix(self, dt):
        params = self.params
        Q = np.zeros((12, 12))
        Q[0:3, 0:3] = np.eye(3) * (params.accel_noise*dt)**2
        Q[3:6, 3:6] = np.eye(3) * (params.gyro_noise*dt)**2
        Q[6:9, 6:9] = np.eye(3) * params.accel_bias_walk**2 * dt
        Q[9:12, 9:12] = np.eye(3) * params.gyro_bias_walk**2 * dt

        return Q


    # Predict the Next Evolution of the States
    def predict(self, a_m, w_m, dt):
        a_m = np.array(a_m)
        w_m = np.array(w_m)
        self._a_m, self._w_m = a_m, w_m

        F = self.transition_matrix(a_m, w_m, dt)
        Q = self.covariance_matrix(dt)
        G = np.zeros((15, 12))
        G[3:15, :] = np.eye(12)
        self.P = F @ self.P @F.T + G @ Q @ G.T
        self.P = 0.5*(self.P + self.P.T)
        R = q2R(self.q)
        a = a_m - self.a_b
        w = w_m - self.w_b
        a_w = R @ a + self.g
        self.p += self.v*dt + 0.5*a_w*dt**2
        self.v += a_w*dt
        self.q = q_normalize(q_multiply(self.q, q_exponential(w*dt)))


    # Apply Error to Nominal Value, x_true = x_nom + dx
    def apply_error(self, dx):
        dx = np.array(dx)
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = q_normalize(q_multiply(self.q, q_exponential(dx[6:9])))
        self.a_b += dx[9:12]
        self.w_b += dx[12:15]


    # Correct the Predicted States
    def _correct(self, y, hx, H, Rm):
        innovation = y - hx
        S = H @ self.P @ H.T + Rm
        K = np.linalg.solve(S, H @ self.P).T
        dx = K @ innovation
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.apply_error(dx)
        G = np.eye(15)
        G[6:9, 6:9] = np.eye(3) - skew(0.5 * dx[6:9])
        self.P = G @ self.P @ G.T
        self.P = 0.5 * (self.P + self.P.T)


    @staticmethod
    def _cov(std, n):
        std = np.array(std, dtype=float)
        if std.ndim == 0:
            return np.eye(n) * std**2
        if std.ndim == 1:
            return np.diag(std**2)
        return std


    # GPS Measurement Model
    def gps_model(self, lever_arm=None):
        l = np.zeros(3) if lever_arm is None else np.array(lever_arm)
        R = q2R(self.q)
        hx = self.p + R @ l
        H = np.zeros((3, 15))
        H[:, 0:3] = np.eye(3)
        H[:, 6:9] = -R @ skew(l)
        return hx, H


    # Use GPS Measurements to Update Predicted States
    def update_gps(self, z, std, lever_arm=None):
        hx, H = self.gps_model(lever_arm)
        return self._correct(np.array(z), hx, H, self._cov(std, 3))


    # Wheel Speed Measurement Model
    def wheel_model(self, lever_arm=None):
        l = np.zeros(3) if lever_arm is None else np.array(lever_arm)
        R = q2R(self.q)
        v_b = R.T @ self.v
        omega = self._w_m - self.w_b
        hx = v_b + np.cross(omega, l)
        H = np.zeros((3, 15))
        H[:, 3:6] = R.T
        H[:, 6:9] = skew(v_b)
        H[:, 12:15] = skew(l)

        return hx, H


    # Use Wheel Speed Measurements to Update Predicted States
    def update_wheel(self, v_fwd, std_fwd=0.05, std_lat=None, std_vert=None, lever_arm=None):
        hx, H = self.wheel_model(lever_arm)
        rows, z, var = [], [], []
        if v_fwd is not None:
            rows.append(0), z.append(v_fwd), var.append(std_fwd**2)
        if std_lat is not None:
            rows.append(1), z.append(0), var.append(std_lat**2)
        if std_vert is not None:
            rows.append(2), z.append(0), var.append(std_vert**2)
        if not rows:
            return None
        return self._correct(np.array(z), hx[rows], H[rows], np.diag(var))
            