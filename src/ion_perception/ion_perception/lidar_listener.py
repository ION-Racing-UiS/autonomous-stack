"""Lidar cone detection: finds cones in each lidar scan and publishes them as ion_interfaces/ConeArray.

Subscribes to LIDAR_TOPIC (sensor_msgs/PointCloud2), publishes /perception/cones (one message per scan, an
empty list if no cones) and RViz markers on /lidar/cone_markers. Interface: src/ion_interfaces/perception_msgs.md.

    colcon build --packages-select ion_interfaces ion_perception && . install/setup.bash
    ros2 run ion_perception lidar_listener --ros-args -p use_sim_time:=true
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from ion_interfaces.msg import Cone, ConeArray

# --- Tuning parameters --------------------------------------------------------
LIDAR_TOPIC = '/lidar/points'   # the own sim's lidar

# Filtering (lidar frame: x forward, y left, z up)
MOUNT_HEIGHT = 0.18       # lidar above the ground (m); the car.yaml placeholder, change for the real car
GROUND_Z = -MOUNT_HEIGHT + 0.05   # keep points more than 5 cm above the ground (m)
FOV_DEG = 90              # field of view in front of the car, total (degrees)

# Clustering
CLUSTER_GAP = 0.3         # points closer than this belong to the same object (m)

# Cone check (is_cone)
MAX_CONE_WIDTH = 0.5      # max spread in x and y (m)
MAX_CONE_HEIGHT = 0.55    # max spread in z (m)
NEAR_DIST = 5             # closer than this: need MIN_PTS_NEAR points (m)
MID_DIST = 10            # closer than this: need MIN_PTS_MID points (m)
MIN_PTS_NEAR = 2
MIN_PTS_MID = 1
MIN_PTS_FAR = 1

# RViz markers
MARKER_LIFETIME = 0.2     # seconds a cone marker stays visible
MARKER_WIDTH = 0.3        # (m)
MARKER_HEIGHT = 0.5       # (m)

# Logging
LOG_CONES = True          # print every detected cone (noisy at 10 Hz)
# ------------------------------------------------------------------------------


def cluster(points, max_gap=CLUSTER_GAP):
    """Return a list of clusters, each an array of points."""
    xy = points[:, :2]
    labels = np.full(len(points), -1)   # -1 = not visited yet
    next_label = 0
    for i in range(len(points)):
        if labels[i] != -1:
            continue
        labels[i] = next_label
        queue = [i]
        while queue:
            j = queue.pop()
            dots = np.linalg.norm(xy - xy[j], axis=1)
            near = np.where((dots < max_gap) & (labels == -1))[0]
            labels[near] = next_label
            queue.extend(near)
        next_label += 1
    return [points[labels == k] for k in range(next_label)]

def is_cone(c):
    cx, cy = c[:, 0].mean(), c[:, 1].mean()
    d = np.hypot(cx, cy)
    if d < NEAR_DIST:
        min_pts = MIN_PTS_NEAR
    elif d < MID_DIST:
        min_pts = MIN_PTS_MID
    else:
        min_pts = MIN_PTS_FAR

    if np.ptp(c[:, 0]) > MAX_CONE_WIDTH:
        return False
    if np.ptp(c[:, 1]) > MAX_CONE_WIDTH:
        return False
    if np.ptp(c[:, 2]) > MAX_CONE_HEIGHT:
        return False
    if len(c) < min_pts:
        return False
    return True

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.create_subscription(PointCloud2, LIDAR_TOPIC, self.on_scan, 10)
        self.above_pub = self.create_publisher(PointCloud2, '/lidar/above', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar/cone_markers', 10)
        self.cones_pub = self.create_publisher(ConeArray, '/perception/cones', 10)

    def on_scan(self, msg: PointCloud2):
        # N x 3 array of (x, y, z) in metres, relative to the lidar
        # (x forward, y left, z up).
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        angle = abs(np.arctan2(pts[:, 1], pts[:, 0])) < np.radians(FOV_DEG / 2)
        above = pts[(pts[:, 2] > GROUND_Z) & (pts[:, 0] > 0) & angle]
        markers = MarkerArray()
        cone_array = ConeArray()
        cone_array.header = msg.header
        if len(above) == 0:
            self.get_logger().info('nothing above ground')
        clusters = cluster(above)
        for i, c in enumerate(clusters):
            if is_cone(c):
                cx = float(c[:, 0].mean())
                cy = float(c[:, 1].mean())
                cz = float(c[:, 2].mean())
                if LOG_CONES:
                    self.get_logger().info(f'cone at x={cx:.2f} y={cy:.2f} ({len(c)} pts)')
                

                cone = Cone()
                cone.position.x = cx
                cone.position.y = cy
                cone.position.z = cz
                cone.color = 'unknown'  # the lidar can't see colour; string vs enum is open
                cone.confidence = 1.0 # placeholder value until a real confidence score is made
                cone_array.cones.append(cone)
                
                m = Marker()
                m.header = msg.header
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.lifetime = Duration(seconds=MARKER_LIFETIME).to_msg()
                m.id = i

                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = cz
                m.pose.orientation.w = 1.0

                m.scale.x = MARKER_WIDTH
                m.scale.y = MARKER_WIDTH
                m.scale.z = MARKER_HEIGHT

                m.color.a = 1.0

                if cy > 0:
                    m.color.r = 0.13
                    m.color.g = 0.59
                    m.color.b = 0.95

                else:
                    m.color.r = 1.00
                    m.color.g = 0.76
                    m.color.b = 0.03
                markers.markers.append(m)
        self.marker_pub.publish(markers)
        self.cones_pub.publish(cone_array)
        f_cloud = point_cloud2.create_cloud_xyz32(msg.header, above)
        self.above_pub.publish(f_cloud)   


        self.get_logger().info(f'{len(clusters)} clusters')


def main():
    rclpy.init()
    rclpy.spin(LidarListener())


if __name__ == '__main__':
    main()
