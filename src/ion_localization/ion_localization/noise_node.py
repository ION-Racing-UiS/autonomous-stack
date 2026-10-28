import copy
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from fs_msgs.msg import WheelStates

# Calculate standard deviations from noise densities given in VN-100 datasheet
Fs_imu = 66
bw_imu = Fs_imu / 2
g = 9.81
imu_acc_std = 0.14e-3 * g * math.sqrt(bw_imu)
imu_gyro_std = 0.0035 * math.sqrt(bw_imu) * math.pi / 180

wheel_fields = ['front_left', 'front_right', 'rear_left', 'rear_right']

# Node Setup
class SensorNoise(Node):
    def __init__(self):
        super().__init__('sensor_noise')                                                                        # Node will be known as sensor_noise by ROS2

        # Declare defaul parameters for noise std, may be changed in ROS2
        self.declare_parameter('imu_acc_std', 0)                                                                # Set to 0, simulator adds noise to IMU
        self.declare_parameter('imu_gyro_std', 0)                                                               # Set to 0, simulator adds noise to IMU
        self.declare_parameter('gps_std', 1.0)
        self.declare_parameter('gps_alt_std', 2.0)
        self.declare_parameter('wheel_rpm_std', 9.0)

        # Define publishers for noisy sensor data
        self.imu_pub = self.create_publisher(Imu, '/imu_noisy', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps_noisy', 10)
        self.wheel_pub = self.create_publisher(WheelStates, '/wheel_states_noisy', 10)

        # Define subscribers for original sensor data, calls class functions when new data is received
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(WheelStates, '/wheel_states', self.on_wheel_states, qos_profile_sensor_data)

        def p(self, name):
            return self.get_parameter(name).value

        # Add noise to messages and publish to new topics
        def on_imu(self, msg):
            out = copy.deepcopy(msg)
            a_std, w_std = self.p('imu_acc_std'), self.p('imu_gyro_std')
            out.linear_acceleration.x += random.gauss(0, a_std)
            out.linear_acceleration.y += random.gauss(0, a_std)
            out.linear_acceleration.z += random.gauss(0, a_std)
            out.angular_velocity.x += random.gauss(0, w_std)
            out.angular_velocity.y += random.gauss(0, w_std)
            out.angular_velocity.z += random.gauss(0, w_std)
            self.imu_pub.publish(out)


        def on_gps(self, msg):
            out = copy.deepcopy(msg)
            std = self.p('gps_std')
            north = random.gauss(0, std)
            east = random.gauss(0, std)
            out.latitude += north / 111320
            out.longitude += east / (111320 * math.cos(math.radians(msg.latitude)))
            out.altitude += random.gauss(0, self.p('gps_alt_std'))
            self.gps_pub.publish(out)


        def on_wheel_states(self, msg):
            out = copy.deepcopy(msg)
            std = self.p('wheel_rpm_std')
            for k in wheel_fields:
                setattr(out, k, getattr(msg, k) + random.gauss(0, std))

            self.wheel_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(SensorNoise())


if __name__ == '__main__':
    main()