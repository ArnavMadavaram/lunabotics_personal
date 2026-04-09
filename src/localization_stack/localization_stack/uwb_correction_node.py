"""
UWB Correction Node (stub / simulator)

Simulates UWB anchor-based absolute position correction.  On real hardware
this node would read from actual UWB ranging hardware; here it computes a
trilateration estimate from 4 synthetic anchors with Gaussian noise.

Publishes:
  /uwb/position  — geometry_msgs/PoseWithCovarianceStamped at 1 Hz
                   (only when quality is DEGRADED or uptime > 30 s)

Subscribes:
  /localization/quality — std_msgs/String  (GOOD | DEGRADED | LOST)

Logs "UWB correction fired at x=X y=Y" on each published correction.
"""

import math
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


# Arena corner anchors [m] per CDR dimensions
ANCHORS = [
    (0.0,    0.0),
    (8.14,   0.0),
    (0.0,    9.14),
    (8.14,   9.14),
]

# True robot position (stationary sim — arena centre)
_TRUE_X = 4.07
_TRUE_Y = 4.57


class UwbCorrectionNode(Node):
    def __init__(self):
        super().__init__('uwb_correction_node')

        self.declare_parameter('noise_sigma',    0.05)   # m
        self.declare_parameter('uptime_trigger', 30.0)   # s

        self._noise_sigma    = self.get_parameter('noise_sigma').value
        self._uptime_trigger = self.get_parameter('uptime_trigger').value

        self._quality = 'GOOD'
        self._start_time = self.get_clock().now()

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/uwb/position', 10)

        self.create_subscription(
            String, '/localization/quality', self._quality_cb, 10)

        self.create_timer(1.0, self._maybe_publish)

        self.get_logger().info('UWB correction node started')

    # ------------------------------------------------------------------ #

    def _quality_cb(self, msg: String):
        self._quality = msg.data

    def _maybe_publish(self):
        uptime = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        should_fire = (self._quality == 'DEGRADED') or (uptime > self._uptime_trigger)

        if not should_fire:
            return

        x_est, y_est = self._trilaterate()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x_est
        msg.pose.pose.position.y = y_est
        msg.pose.pose.orientation.w = 1.0

        # UWB position covariance (~10 cm = 0.1 m  → variance 0.01 m²)
        sigma = self._noise_sigma
        msg.pose.covariance[0]  = sigma ** 2   # x variance
        msg.pose.covariance[7]  = sigma ** 2   # y variance
        msg.pose.covariance[35] = 0.1          # yaw unknown — wide prior

        self._pub.publish(msg)
        self.get_logger().info(
            f'UWB correction fired at x={x_est:.3f} y={y_est:.3f}')

    # ------------------------------------------------------------------ #

    def _trilaterate(self) -> tuple:
        """
        Compute a weighted-centroid position estimate from 4 anchors.

        Each anchor contributes a noisy range measurement from the true
        robot position.  We solve a simple weighted least-squares system
        using anchor geometry.  The noise magnitude is self._noise_sigma.
        """
        # Noisy range measurements from each anchor
        ranges = []
        for ax, ay in ANCHORS:
            true_range = math.hypot(_TRUE_X - ax, _TRUE_Y - ay)
            noisy = true_range + np.random.normal(0.0, self._noise_sigma)
            ranges.append(noisy)

        # Linearised least squares: use anchor 0 as reference
        ax0, ay0 = ANCHORS[0]
        r0 = ranges[0]

        A_rows = []
        b_rows = []
        for i in range(1, len(ANCHORS)):
            axi, ayi = ANCHORS[i]
            ri = ranges[i]
            # (xi^2 - x0^2) + (yi^2 - y0^2) - ri^2 + r0^2 = 2*(xi-x0)*x + 2*(yi-y0)*y
            A_rows.append([2.0 * (axi - ax0), 2.0 * (ayi - ay0)])
            b_rows.append(
                axi**2 - ax0**2 + ayi**2 - ay0**2 - ri**2 + r0**2)

        A = np.array(A_rows)
        b = np.array(b_rows)

        # Least-squares solution
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return float(result[0]), float(result[1])


def main(args=None):
    rclpy.init(args=args)
    node = UwbCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
