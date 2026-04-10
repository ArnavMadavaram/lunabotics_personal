"""
Fake Unitree L2 LiDAR publisher for hazard_detection testing.

Publishes sensor_msgs/PointCloud2 on /unilidar/cloud at 10 Hz in the
unitree_lidar_link frame.  The cloud contains:
  - A flat ground plane (z=0) from x=-2..+4 m, y=-2..+2 m at 0.08 m grid
  - 2 craters (depressions) at (1.5, -0.5) and (2.5, 0.8), z=-0.15 m
  - 2 rocks   (bumps)      at (1.0,  0.3) and (2.0, -0.8), z=+0.15 m
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')

        self._pub = self.create_publisher(PointCloud2, '/unilidar/cloud', 10)

        # Pre-build the point array once — it never changes
        self._cloud_data, self._num_points = self._build_cloud()

        self.create_timer(0.1, self._publish)   # 10 Hz
        self.get_logger().info('Fake LiDAR publisher started (10 Hz)')

    # ------------------------------------------------------------------
    def _build_cloud(self):
        """Return (bytes, N) for the static point cloud."""
        xs = np.arange(-2.0, 4.01, 0.08, dtype=np.float32)
        ys = np.arange(-2.0, 2.01, 0.08, dtype=np.float32)
        XX, YY = np.meshgrid(xs, ys)
        ZZ = np.zeros_like(XX)

        # Craters: depressions at given (cx, cy), radius 0.25 m, z = -0.15
        for cx, cy in [(1.5, -0.5), (2.5, 0.8)]:
            mask = np.sqrt((XX - cx) ** 2 + (YY - cy) ** 2) < 0.25
            ZZ[mask] = -0.15

        # Rocks: bumps at given (rx, ry), radius 0.20 m, z = +0.15
        for rx, ry in [(1.0, 0.3), (2.0, -0.8)]:
            mask = np.sqrt((XX - rx) ** 2 + (YY - ry) ** 2) < 0.20
            ZZ[mask] = 0.15

        pts = np.column_stack([
            XX.flatten(), YY.flatten(), ZZ.flatten()
        ]).astype(np.float32)

        return pts.tobytes(), len(pts)

    def _publish(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'unitree_lidar_link'

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = self._num_points
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12          # 3 × float32
        msg.row_step = 12 * self._num_points
        msg.is_dense = True
        msg.data = self._cloud_data

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
