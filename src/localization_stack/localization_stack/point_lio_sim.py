"""
Point-LIO Simulator Node

Mimics the output of the real Point-LIO LiDAR-inertial odometry package so
the rest of the localization stack can be developed and tested without hardware.

On real hardware this node is replaced by the actual Point-LIO C++ package.

Publishes:
  /tf                    — map → odom  transform at 50 Hz
  /localization/odom     — nav_msgs/Odometry at 50 Hz
  /localization/quality  — std_msgs/String  at  1 Hz  (GOOD | DEGRADED | LOST)
  /imu/data              — sensor_msgs/Imu  at 200 Hz (if no external source)
  /odometry/wheel        — nav_msgs/Odometry at 20 Hz (fake wheel odom for EKF)

Subscribes:
  /unilidar/cloud        — sensor_msgs/PointCloud2  (fake lidar)
  /imu/data              — sensor_msgs/Imu          (checked for external source)

Deliberately does NOT publish odom → base_link — that is the EKF's job.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np

from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String, Header
from tf2_ros import TransformBroadcaster


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PointLioSim(Node):
    def __init__(self):
        super().__init__('point_lio_sim')

        # Parameters
        self.declare_parameter('drift_sigma', 0.001)   # m per second
        self.declare_parameter('publish_fake_imu', True)

        self._drift_sigma = self.get_parameter('drift_sigma').value
        self._publish_fake_imu = self.get_parameter('publish_fake_imu').value

        # Simulated pose (map frame)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Track whether an external IMU is providing data
        self._imu_received = False
        self._imu_check_deadline = self.get_clock().now().nanoseconds + int(2e9)  # 2 s

        # Quality state
        self._covariance_level = 0.0   # grows with drift
        self._lidar_received = False
        self._last_lidar_time = None

        # TF broadcaster (map → odom only)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self._odom_pub = self.create_publisher(
            Odometry, '/localization/odom', 10)
        self._quality_pub = self.create_publisher(
            String, '/localization/quality', 10)
        self._wheel_odom_pub = self.create_publisher(
            Odometry, '/odometry/wheel', 10)

        self._imu_pub = None  # created lazily after IMU check window

        # Subscribers
        self.create_subscription(
            PointCloud2, '/unilidar/cloud', self._lidar_cb, 10)
        self.create_subscription(
            Imu, '/imu/data', self._imu_cb, 10)

        # Timers
        self.create_timer(1.0 / 50.0, self._publish_tf_and_odom)   # 50 Hz
        self.create_timer(1.0 / 20.0, self._publish_wheel_odom)    # 20 Hz
        self.create_timer(1.0,        self._publish_quality)        #  1 Hz
        # IMU check fires once after 2 s — cancelled immediately after first call
        self._imu_check_timer = self.create_timer(2.0, self._check_imu_and_start)

        self.get_logger().info('Point-LIO simulator started (map→odom @ 50 Hz)')

    # ------------------------------------------------------------------ #
    # Subscribers                                                          #
    # ------------------------------------------------------------------ #

    def _lidar_cb(self, msg: PointCloud2):
        self._lidar_received = True
        self._last_lidar_time = self.get_clock().now()

    def _imu_cb(self, msg: Imu):
        self._imu_received = True

    # ------------------------------------------------------------------ #
    # IMU check / fake IMU                                                 #
    # ------------------------------------------------------------------ #

    def _check_imu_and_start(self):
        # Cancel so this one-shot check never fires again
        self._imu_check_timer.cancel()
        if not self._imu_received and self._publish_fake_imu:
            self.get_logger().info(
                'No external /imu/data detected — starting internal fake IMU at 200 Hz')
            self._imu_pub = self.create_publisher(Imu, '/imu/data', 10)
            self.create_timer(1.0 / 200.0, self._publish_fake_imu_cb)
        else:
            self.get_logger().info('External /imu/data detected — not publishing fake IMU')

    def _publish_fake_imu_cb(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Robot sitting still: gravity aligned with -z in world, but IMU
        # measures specific force so z = +9.81 when stationary
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001

        # Orientation: identity (robot is level)
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        if self._imu_pub is not None:
            self._imu_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # 50 Hz: map → odom TF + /localization/odom                           #
    # ------------------------------------------------------------------ #

    def _publish_tf_and_odom(self):
        now = self.get_clock().now().to_msg()

        # Add tiny gaussian drift each tick to simulate real localization
        dt = 1.0 / 50.0
        self._x += np.random.normal(0.0, self._drift_sigma * dt)
        self._y += np.random.normal(0.0, self._drift_sigma * dt)
        self._covariance_level += self._drift_sigma * dt

        q = _yaw_to_quat(self._yaw)

        # map → odom  (this is Point-LIO's primary output)
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q
        self._tf_broadcaster.sendTransform(tf)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'odom'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q

        cov = self._covariance_level
        odom.pose.covariance[0]  = cov   # x
        odom.pose.covariance[7]  = cov   # y
        odom.pose.covariance[35] = cov   # yaw

        self._odom_pub.publish(odom)

    # ------------------------------------------------------------------ #
    # 20 Hz: /odometry/wheel  (fake wheel odometry for EKF)               #
    # ------------------------------------------------------------------ #

    def _publish_wheel_odom(self):
        now = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Robot is sitting still — add tiny noise to position
        msg.pose.pose.position.x = np.random.normal(0.0, 0.002)
        msg.pose.pose.position.y = np.random.normal(0.0, 0.002)
        msg.pose.pose.orientation.w = 1.0

        # Covariance: moderate wheel encoder uncertainty
        for i in [0, 7, 14, 21, 28, 35]:
            msg.pose.covariance[i] = 0.1
        for i in [0, 7, 14, 21, 28, 35]:
            msg.twist.covariance[i] = 0.1

        self._wheel_odom_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # 1 Hz: /localization/quality                                          #
    # ------------------------------------------------------------------ #

    def _publish_quality(self):
        # Determine quality from accumulated covariance
        if self._covariance_level < 0.05:
            quality = 'GOOD'
        elif self._covariance_level < 0.15:
            quality = 'DEGRADED'
        else:
            quality = 'LOST'

        # Also degrade if lidar data is stale
        if self._last_lidar_time is not None:
            age = (self.get_clock().now() - self._last_lidar_time).nanoseconds / 1e9
            if age > 2.0 and quality == 'GOOD':
                quality = 'DEGRADED'

        msg = String()
        msg.data = quality
        self._quality_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointLioSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
