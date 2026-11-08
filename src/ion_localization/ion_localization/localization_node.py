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

# Imports
import numpy as np

# Quaternion Helper Functions
def q_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])


def q_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def q_normalize(q):
    q = q / np.linalg.norm(q)
    return q


def q_exponential(rotation_vector):
    theta = np.linalg.norm(rotation_vector)
    if theta < 1e-6:
        return np.array([1, 0, 0, 0])
    axis = rotation_vector / theta
    return np.concatenate([[np.cos(theta / 2)], np.sin(theta / 2) * axis])


def q_logarithm(q):
    if q[0] < 0:
        q = -q
    n = np.linalg.norm(q[1:])
    if n < 1e-12:
        return 2*q[1:]
    return 2 * np.arctan2(n, q[0]) * q[1:] / n


def q2R(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return R


def q2euler(q):
    R = q2R(q)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arcsin(-R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def euler2q(roll, pitch, yaw):
    qx = q_exponential([roll, 0, 0])
    qy = q_exponential([0, pitch, 0])
    qz = q_exponential([0, 0, yaw])
    return q_normalize(q_multiply(q_multiply(qz, qy), qx))
    


# Error State Extended Kalman Filter
class ESKF:
    def __init__(self, params: ESKFParams):
        self.params = params
        self.g = np.array(self.params.g)        # World Frame Gravity Vector
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
        self.p = np.array(p0)
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
                np.full(3, params.init_a_b**2),
                np.full(3, params.init_w_b**2)
            ])
        )
        

        