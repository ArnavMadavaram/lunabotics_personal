#!/usr/bin/env python3
"""
Fake hazard publisher for Lunabotics costmap demo / presentation.

Publishes:
  /hazards/rocks/grid   - OccupancyGrid rock obstacles
  /hazards/craters/grid - OccupancyGrid crater keepout zones
  /hazards/arena/grid   - OccupancyGrid arena walls + zone boundaries
  /robot_pose_demo      - Animated robot traversing the arena

Arena layout (12m x 12m, origin at -6,-6):
  Outer walls
  Starting zone  (bottom-left):  x=-5..-2, y=-5..-1
  Excavation zone (left):        x=-5..-2, y=-1.. 4
  Obstacle zone  (center):       x=-2.. 2, y=-1.. 4
  Construction zone (right):     x= 2.. 5, y=-1.. 4
  Berm target    (far right):    x= 3.. 5, y= 0.. 3
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

# Grid params — must match costmaps.yaml
RES  = 0.05
COLS = 240
ROWS = 240
OX   = -6.0
OY   = -6.0


def world_to_cell(x, y):
    return int((x - OX) / RES), int((y - OY) / RES)


def fill_circle(data, cx, cy, radius, cost):
    r_cells = int(radius / RES)
    cc, cr = world_to_cell(cx, cy)
    for dr in range(-r_cells, r_cells + 1):
        for dc in range(-r_cells, r_cells + 1):
            if dc*dc + dr*dr <= r_cells*r_cells:
                c, r = cc + dc, cr + dr
                if 0 <= c < COLS and 0 <= r < ROWS:
                    if data[r * COLS + c] < cost:
                        data[r * COLS + c] = cost


def fill_rect(data, x0, y0, x1, y1, cost):
    c0, r0 = world_to_cell(min(x0,x1), min(y0,y1))
    c1, r1 = world_to_cell(max(x0,x1), max(y0,y1))
    for r in range(max(0, r0), min(ROWS, r1+1)):
        for c in range(max(0, c0), min(COLS, c1+1)):
            if data[r * COLS + c] < cost:
                data[r * COLS + c] = cost


def draw_wall(data, x0, y0, x1, y1, thickness=0.15):
    """Draw a wall as a thick filled rectangle."""
    if x0 == x1:  # vertical wall
        fill_rect(data, x0 - thickness, y0, x0 + thickness, y1, 100)
    else:          # horizontal wall
        fill_rect(data, x0, y0 - thickness, x1, y0 + thickness, 100)


def draw_zone_line(data, x0, y0, x1, y1):
    """Draw a thin dashed zone boundary (cost=40, advisory only)."""
    steps = int(max(abs(x1-x0), abs(y1-y0)) / (RES * 6))
    for i in range(steps):
        t = i / steps
        mx = x0 + (x1 - x0) * t
        my = y0 + (y1 - y0) * t
        fill_circle(data, mx, my, 0.06, 40)


# ── Rocks: spread across the obstacle zone with varied sizes ──────────────
ROCKS = [
    # obstacle zone center — slalom challenge
    (-1.6,  0.5, 0.22),
    (-0.8,  1.8, 0.18),
    (-1.8,  3.0, 0.25),
    ( 0.2,  0.8, 0.20),
    ( 1.0,  2.0, 0.16),
    ( 1.6,  0.4, 0.22),
    ( 0.6,  3.2, 0.19),
    (-0.4,  2.5, 0.17),
    # a couple near excavation zone edge
    (-3.5,  1.2, 0.15),
    (-2.8,  3.5, 0.18),
    # one near construction zone
    ( 3.0,  0.8, 0.20),
]

# ── Craters: larger, well-spaced across obstacle zone ─────────────────────
CRATERS = [
    (-1.2,  1.2, 0.50),   # left side
    ( 0.8,  1.6, 0.45),   # center
    (-0.2,  3.4, 0.40),   # upper center
    ( 1.5,  3.0, 0.35),   # upper right
]

# ── Robot path: full mission cycle ────────────────────────────────────────
ROBOT_PATH = [
    (-4.0, -3.0),   # starting zone
    (-3.5, -1.5),
    (-3.8,  0.5),   # entering excavation
    (-4.0,  2.5),   # excavation zone (dig)
    (-3.5,  2.5),
    (-2.5,  1.5),   # leaving excavation
    (-1.8,  0.2),   # obstacle zone entry — slalom begins
    (-1.0,  1.0),   # weaving between rocks/craters
    ( 0.0,  2.2),
    ( 1.2,  1.0),
    ( 1.8,  2.8),
    ( 2.5,  1.5),   # construction zone entry
    ( 3.8,  1.5),   # berm approach
    ( 4.0,  2.0),   # dump
    ( 3.5,  1.5),   # return
    ( 2.2,  2.0),
    ( 1.0,  2.5),
    (-0.5,  2.0),
    (-1.5,  1.0),
    (-2.5,  1.5),
    (-3.8,  2.5),   # back to excavation
]


class FakeHazards(Node):
    def __init__(self):
        super().__init__('fake_hazards')

        self.rock_pub   = self.create_publisher(OccupancyGrid, '/hazards/rocks/grid',   10)
        self.crater_pub = self.create_publisher(OccupancyGrid, '/hazards/craters/grid', 10)
        self.arena_pub  = self.create_publisher(OccupancyGrid, '/hazards/arena/grid',   10)
        self.pose_pub   = self.create_publisher(PoseStamped,   '/robot_pose_demo',      10)

        self._rocks_data  = self._build_rocks_grid()
        self._crater_data = self._build_craters_grid()
        self._arena_data  = self._build_arena_grid()

        self.path_idx = 0
        self.path_t   = 0.0

        self.timer = self.create_timer(0.2, self.publish)
        self.get_logger().info('Fake hazards running — 11 rocks, 4 craters, arena walls')

    def _build_rocks_grid(self):
        data = [0] * (COLS * ROWS)
        for (rx, ry, r) in ROCKS:
            fill_circle(data, rx, ry, r,        100)
            fill_circle(data, rx, ry, r + 0.06,  70)
        return data

    def _build_craters_grid(self):
        data = [0] * (COLS * ROWS)
        for (cx, cy, r) in CRATERS:
            fill_circle(data, cx, cy, r + 0.30, 100)  # keepout
            fill_circle(data, cx, cy, r + 0.45,  60)  # warning
        return data

    def _build_arena_grid(self):
        data = [0] * (COLS * ROWS)

        # Outer arena walls
        draw_wall(data, -5.0,  5.0, -5.0, -5.0)   # left wall   (vertical)
        draw_wall(data,  5.0,  5.0,  5.0, -5.0)   # right wall  (vertical)
        draw_wall(data, -5.0,  5.0,  5.0,  5.0)   # top wall    (horizontal)
        draw_wall(data, -5.0, -5.0,  5.0, -5.0)   # bottom wall (horizontal)

        # Zone divider lines (dashed, advisory cost=40)
        draw_zone_line(data, -2.0, -5.0, -2.0,  5.0)  # excav | obstacle
        draw_zone_line(data,  2.0, -5.0,  2.0,  5.0)  # obstacle | construction
        draw_zone_line(data, -5.0, -1.0,  5.0, -1.0)  # starting zone top

        # Berm target area (light fill, cost=20 so planner can still route through)
        fill_rect(data, 3.0, 0.0, 5.0, 3.0, 20)

        return data

    def _make_grid(self, header, data):
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = RES
        msg.info.width  = COLS
        msg.info.height = ROWS
        msg.info.origin.position.x = OX
        msg.info.origin.position.y = OY
        msg.data = data
        return msg

    def publish(self):
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id='map')
        self.rock_pub.publish(self._make_grid(header, self._rocks_data))
        self.crater_pub.publish(self._make_grid(header, self._crater_data))
        self.arena_pub.publish(self._make_grid(header, self._arena_data))
        self._publish_robot(now)

    def _publish_robot(self, stamp):
        self.path_t += 0.035
        if self.path_t >= 1.0:
            self.path_t = 0.0
            self.path_idx = (self.path_idx + 1) % (len(ROBOT_PATH) - 1)

        x0, y0 = ROBOT_PATH[self.path_idx]
        x1, y1 = ROBOT_PATH[(self.path_idx + 1) % len(ROBOT_PATH)]
        t = self.path_t
        x   = x0 + (x1 - x0) * t
        y   = y0 + (y1 - y0) * t
        yaw = math.atan2(y1 - y0, x1 - x0)

        msg = PoseStamped()
        msg.header = Header(stamp=stamp, frame_id='map')
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeHazards()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
