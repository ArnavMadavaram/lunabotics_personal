"""
Hazard detection node — crater and rock pipeline (CDR spec).

Subscribes : /unilidar/cloud  (sensor_msgs/PointCloud2, unitree_lidar_link)
Publishes  : /unilidar/cloud_processed  (PointCloud2, base_link)
             /hazards/craters/grid      (nav_msgs/OccupancyGrid, base_link)

Pipeline
--------
1. Crop to ROI  (x: roi_x_min..roi_x_max, y: ±roi_y_max) in base_link frame
2. Ground segmentation  — median z of inlier band
3. Classify  — ROCK if z > ground + rock_threshold
               CRATER if z < ground − crater_threshold
4. Invert craters  — z_inv = ground_z + |z_crater − ground_z|
5. Merge rocks + inverted craters into unified obstacle cloud
6. Build OccupancyGrid  — res 0.08 m, lethal=100, inflate by inflation_radius
7. Publish at publish_rate Hz
"""

import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.duration
import rclpy.time
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class HazardNode(Node):
    def __init__(self):
        super().__init__('hazard_node')

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter('roi_x_min',        0.05)
        self.declare_parameter('roi_x_max',        4.0)
        self.declare_parameter('roi_y_max',        2.0)
        self.declare_parameter('ground_tolerance', 0.05)
        self.declare_parameter('rock_threshold',   0.10)
        self.declare_parameter('crater_threshold', 0.08)
        self.declare_parameter('grid_resolution',  0.08)
        self.declare_parameter('inflation_radius', 0.30)
        self.declare_parameter('publish_rate',     8.0)

        p = self.get_parameter
        self._roi_x_min        = p('roi_x_min').value
        self._roi_x_max        = p('roi_x_max').value
        self._roi_y_max        = p('roi_y_max').value
        self._ground_tol       = p('ground_tolerance').value
        self._rock_thr         = p('rock_threshold').value
        self._crater_thr       = p('crater_threshold').value
        self._res              = p('grid_resolution').value
        self._infl_r           = p('inflation_radius').value
        rate                   = p('publish_rate').value

        # Pre-compute grid dimensions (constant for this ROI + resolution)
        self._gw = int(np.ceil(
            (self._roi_x_max - self._roi_x_min) / self._res))   # columns
        self._gh = int(np.ceil(
            2.0 * self._roi_y_max / self._res))                  # rows
        self._infl_cells = int(np.ceil(self._infl_r / self._res))

        # Pre-compute circular inflation kernel offsets
        ic = self._infl_cells
        offsets = []
        for di in range(-ic, ic + 1):
            for dj in range(-ic, ic + 1):
                if di * di + dj * dj <= ic * ic:
                    offsets.append((di, dj))
        self._infl_offsets = offsets

        # ── TF ────────────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── pub / sub ─────────────────────────────────────────────────
        self._cloud_pub = self.create_publisher(
            PointCloud2, '/unilidar/cloud_processed', 10)
        self._grid_pub = self.create_publisher(
            OccupancyGrid, '/hazards/craters/grid', 10)

        self.create_subscription(
            PointCloud2, '/unilidar/cloud', self._cloud_cb, 10)

        self._latest_msg = None
        self.create_timer(1.0 / rate, self._publish_hazards)

        self.get_logger().info(
            f'Hazard node started — grid {self._gw}×{self._gh} cells, '
            f'res={self._res} m, inflation={self._infl_r} m, rate={rate} Hz')

    # ------------------------------------------------------------------
    # Callback — just stash latest message
    def _cloud_cb(self, msg: PointCloud2):
        self._latest_msg = msg

    # ------------------------------------------------------------------
    # Timer callback — full pipeline
    def _publish_hazards(self):
        msg = self._latest_msg
        if msg is None:
            return

        stamp = msg.header.stamp

        # ── parse PointCloud2 → Nx3 float32 ──────────────────────────
        pts = self._parse_cloud(msg)
        if len(pts) == 0:
            return

        # ── transform lidar → base_link ───────────────────────────────
        R, t = self._get_tf('base_link', 'unitree_lidar_link')
        pts_b = (R @ pts.T).T + t          # Nx3 in base_link frame

        # Step 1 — ROI crop
        roi = (
            (pts_b[:, 0] >= self._roi_x_min) &
            (pts_b[:, 0] <= self._roi_x_max) &
            (pts_b[:, 1] >= -self._roi_y_max) &
            (pts_b[:, 1] <=  self._roi_y_max)
        )
        pts_roi = pts_b[roi]
        if len(pts_roi) == 0:
            return

        # Step 2 — Ground segmentation (robust median on height-band inliers)
        z_med = np.median(pts_roi[:, 2])
        inliers = np.abs(pts_roi[:, 2] - z_med) <= self._ground_tol
        ground_z = float(np.median(pts_roi[inliers, 2])) if inliers.any() else float(z_med)

        # Step 3 — Classify
        rock_mask   = pts_roi[:, 2] > ground_z + self._rock_thr
        crater_mask = pts_roi[:, 2] < ground_z - self._crater_thr

        pts_rocks   = pts_roi[rock_mask]
        pts_craters = pts_roi[crater_mask]

        # Step 4 — Invert craters so they appear as positive obstacles
        if len(pts_craters) > 0:
            pts_ci = pts_craters.copy()
            pts_ci[:, 2] = ground_z + np.abs(pts_craters[:, 2] - ground_z)
        else:
            pts_ci = pts_craters          # empty, same shape

        # Step 5 — Merge
        parts = [p for p in (pts_rocks, pts_ci) if len(p) > 0]
        if parts:
            pts_obs = np.vstack(parts).astype(np.float32)
        else:
            pts_obs = np.zeros((0, 3), dtype=np.float32)

        # Step 6 / 7 — Build OccupancyGrid and publish both outputs
        grid_msg = self._build_grid(pts_obs, stamp)
        self._grid_pub.publish(grid_msg)

        if len(pts_obs) > 0:
            cloud_msg = self._make_cloud(pts_obs, stamp, 'base_link')
            self._cloud_pub.publish(cloud_msg)

    # ------------------------------------------------------------------
    # PointCloud2 parsing — pure numpy, no external deps
    def _parse_cloud(self, msg: PointCloud2) -> np.ndarray:
        """Return Nx3 float32 array of (x, y, z) points."""
        fmap = {f.name: f.offset for f in msg.fields}
        xo, yo, zo = fmap['x'], fmap['y'], fmap['z']
        step = msg.point_step
        n = msg.width * msg.height

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

        def _f32(off):
            return np.frombuffer(raw[:, off:off + 4].tobytes(), dtype=np.float32)

        return np.column_stack([_f32(xo), _f32(yo), _f32(zo)])

    # ------------------------------------------------------------------
    # TF helper — returns (R 3×3, t 3,) or identity if not yet available
    def _get_tf(self, target: str, source: str):
        try:
            tf = self._tf_buf.lookup_transform(
                target, source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            tr = tf.transform.translation
            q  = tf.transform.rotation
            t  = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
                [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
            ], dtype=np.float64)
            return R, t
        except Exception:
            return np.eye(3), np.zeros(3)

    # ------------------------------------------------------------------
    # Build nav_msgs/OccupancyGrid with inflation
    def _build_grid(self, pts_obs: np.ndarray, stamp) -> OccupancyGrid:
        gh, gw = self._gh, self._gw
        grid = np.zeros((gh, gw), dtype=np.int8)

        if len(pts_obs) > 0:
            obs_mask = np.zeros((gh, gw), dtype=bool)

            # Map each obstacle point to a cell
            col = ((pts_obs[:, 0] - self._roi_x_min) / self._res).astype(int)
            row = ((pts_obs[:, 1] + self._roi_y_max) / self._res).astype(int)

            valid = (
                (col >= 0) & (col < gw) &
                (row >= 0) & (row < gh)
            )
            col, row = col[valid], row[valid]
            obs_mask[row, col] = True

            # Inflate using numpy roll — no scipy required
            inflated = obs_mask.copy()
            for di, dj in self._infl_offsets:
                if di == 0 and dj == 0:
                    continue
                shifted = np.roll(np.roll(obs_mask, di, axis=0), dj, axis=1)
                # Zero out wrapped edges so we don't get artefacts
                if di > 0:
                    shifted[:di, :] = False
                elif di < 0:
                    shifted[di:, :] = False
                if dj > 0:
                    shifted[:, :dj] = False
                elif dj < 0:
                    shifted[:, dj:] = False
                inflated |= shifted

            grid[inflated] = 100   # lethal

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self._res
        msg.info.width  = gw
        msg.info.height = gh
        msg.info.origin.position.x = float(self._roi_x_min)
        msg.info.origin.position.y = float(-self._roi_y_max)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()
        return msg

    # ------------------------------------------------------------------
    # Build PointCloud2 from Nx3 float32
    def _make_cloud(self, pts: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = HazardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
