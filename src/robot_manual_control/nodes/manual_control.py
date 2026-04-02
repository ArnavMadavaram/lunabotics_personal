#!/usr/bin/env python3
"""
manual_control.py — Keyboard teleoperation node for Lunabotics robot.

Reads keyboard input in real-time via a curses UI and publishes:
  /cmd_vel          (geometry_msgs/Twist)  — continuous at 10 Hz
  /excavation/cmd   (std_msgs/Bool)        — on toggle
  /deposition/cmd   (std_msgs/Bool)        — on toggle

Key bindings:
  W / Up     = Forward          S / Down   = Backward
  A / Left   = Turn left        D / Right  = Turn right
  Space      = Stop immediately
  E          = Toggle excavation
  Shift+D    = Toggle deposition
  Q          = Quit (publishes zero Twist before exit)

Word commands (type full word + Enter):
  excavation | deposition | stop | quit
"""

import sys
import threading
import time
import curses
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ── Tunable speed constants ────────────────────────────────────────────────────
# TODO ECE: Set LINEAR_SPEED to the safe maximum forward/backward speed for
#           the drive motors (metres per second).  Start low and ramp up.
LINEAR_SPEED: float = 0.3   # m/s

# TODO ECE: Set ANGULAR_SPEED to the safe maximum yaw rate for the drive motors
#           (radians per second).  Positive = left (CCW when viewed from above).
ANGULAR_SPEED: float = 0.8  # rad/s
# ──────────────────────────────────────────────────────────────────────────────

PUBLISH_RATE_HZ: int = 10    # cmd_vel publish rate
MAX_LOG_LINES:   int = 8     # lines in the on-screen event log


# ─── Shared State ─────────────────────────────────────────────────────────────

class RobotState:
    """Thread-safe container for all node state shared between ROS and UI."""

    def __init__(self) -> None:
        self.lock           = threading.Lock()
        self.linear_x:  float = 0.0
        self.angular_z: float = 0.0
        self.excavation_on: bool  = False
        self.deposition_on: bool  = False
        self.last_key:      str   = '—'
        self.running:       bool  = True
        self.log: deque[str]      = deque(maxlen=MAX_LOG_LINES)

    def log_event(self, msg: str) -> None:
        with self.lock:
            self.log.append(f'[{time.strftime("%H:%M:%S")}] {msg}')


# ─── ROS Node ─────────────────────────────────────────────────────────────────

class ManualControlNode(Node):
    """ROS2 node: publishes cmd_vel at PUBLISH_RATE_HZ and subsystem booleans."""

    def __init__(self, state: RobotState) -> None:
        super().__init__('manual_control')
        self.state = state

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.excav_pub   = self.create_publisher(Bool,  '/excavation/cmd', 10)
        self.deposi_pub  = self.create_publisher(Bool,  '/deposition/cmd', 10)

        # Continuous 10 Hz timer so cmd_vel keeps flowing while a key is held
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cmd_vel)

    # ── Timer callback (ROS thread) ────────────────────────────────────────────
    def _publish_cmd_vel(self) -> None:
        with self.state.lock:
            if not self.state.running:
                return
            lin = self.state.linear_x
            ang = self.state.angular_z

        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self.cmd_vel_pub.publish(msg)

    # ── One-shot publishers (called from UI thread — safe in rclpy/DDS) ────────
    def publish_stop(self) -> None:
        """Publish a zero Twist immediately — used on shutdown."""
        self.cmd_vel_pub.publish(Twist())

    def publish_excavation(self, val: bool) -> None:
        msg = Bool(); msg.data = val
        self.excav_pub.publish(msg)

    def publish_deposition(self, val: bool) -> None:
        msg = Bool(); msg.data = val
        self.deposi_pub.publish(msg)


# ─── Terminal UI ──────────────────────────────────────────────────────────────

def _draw_ui(stdscr, state: RobotState, cmd_buffer: str) -> None:
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    safe_w = w - 1  # avoid writing to the very last column

    # ── Header ────────────────────────────────────────────────────────────────
    title = "=== LUNABOTICS MANUAL CONTROL ==="
    col   = max(0, (w - len(title)) // 2)
    stdscr.addstr(0, col, title, curses.A_BOLD)

    # Capture state snapshot under lock
    with state.lock:
        lin    = state.linear_x
        ang    = state.angular_z
        excav  = state.excavation_on
        deposi = state.deposition_on
        last   = state.last_key
        log    = list(state.log)

    if   lin > 0: motion = "FORWARD"
    elif lin < 0: motion = "BACKWARD"
    elif ang > 0: motion = "TURN LEFT"
    elif ang < 0: motion = "TURN RIGHT"
    else:         motion = "STOPPED"

    stdscr.addstr(2, 2,
        f"Motion:      {motion:<12}  linear.x = {lin:+.2f} m/s   angular.z = {ang:+.2f} rad/s")

    excav_attr  = curses.A_BOLD | curses.color_pair(1) if excav  else curses.A_NORMAL
    deposi_attr = curses.A_BOLD | curses.color_pair(1) if deposi else curses.A_NORMAL
    stdscr.addstr(3, 2, "Excavation:  ")
    stdscr.addstr("ON " if excav  else "OFF", excav_attr)
    stdscr.addstr("       Deposition:  ")
    stdscr.addstr("ON" if deposi else "OFF", deposi_attr)

    stdscr.addstr(4, 2, f"Last key:    {last}")

    if cmd_buffer:
        stdscr.addstr(5, 2, f"Command buf: [{cmd_buffer}_]", curses.A_BOLD)

    # ── Separator ─────────────────────────────────────────────────────────────
    stdscr.addstr(6, 0, "─" * min(safe_w, 64))

    # ── Help ──────────────────────────────────────────────────────────────────
    stdscr.addstr(7, 2, "W/↑ Fwd   S/↓ Back   A/← Left   D/→ Right   SPACE Stop")
    stdscr.addstr(8, 2, "E Excavation   Shift+D Deposition   Q Quit")
    stdscr.addstr(9, 2, "Word commands + Enter: excavation | deposition | stop | quit")

    stdscr.addstr(10, 0, "─" * min(safe_w, 64))

    # ── Event log ─────────────────────────────────────────────────────────────
    stdscr.addstr(11, 2, "Event Log:", curses.A_BOLD)
    for i, line in enumerate(log):
        row = 12 + i
        if row < h - 1:
            stdscr.addstr(row, 2, line[:safe_w - 2])

    stdscr.refresh()


def _run_curses(stdscr, node: ManualControlNode, state: RobotState) -> None:
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    stdscr.nodelay(True)   # non-blocking getch
    stdscr.keypad(True)    # decode arrow-key escape sequences

    cmd_buffer: str = ''
    state.log_event("Manual control started")
    state.log_event(f"LINEAR_SPEED={LINEAR_SPEED} m/s  ANGULAR_SPEED={ANGULAR_SPEED} rad/s")

    while True:
        with state.lock:
            if not state.running:
                break

        _draw_ui(stdscr, state, cmd_buffer)

        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        if key == -1:
            time.sleep(0.04)  # ~25 Hz UI refresh; cmd_vel published independently
            continue

        # ── Arrow keys ────────────────────────────────────────────────────────
        if key == curses.KEY_UP:
            with state.lock:
                state.linear_x  =  LINEAR_SPEED
                state.angular_z =  0.0
                state.last_key  = '↑  (forward)'
            state.log_event("Moving FORWARD")
            continue

        if key == curses.KEY_DOWN:
            with state.lock:
                state.linear_x  = -LINEAR_SPEED
                state.angular_z =  0.0
                state.last_key  = '↓  (backward)'
            state.log_event("Moving BACKWARD")
            continue

        if key == curses.KEY_LEFT:
            with state.lock:
                state.linear_x  =  0.0
                state.angular_z =  ANGULAR_SPEED
                state.last_key  = '←  (turn left)'
            state.log_event("Turning LEFT")
            continue

        if key == curses.KEY_RIGHT:
            with state.lock:
                state.linear_x  =  0.0
                state.angular_z = -ANGULAR_SPEED
                state.last_key  = '→  (turn right)'
            state.log_event("Turning RIGHT")
            continue

        # ── Space = stop ──────────────────────────────────────────────────────
        if key == ord(' '):
            with state.lock:
                state.linear_x  = 0.0
                state.angular_z = 0.0
                state.last_key  = 'SPACE (stop)'
            cmd_buffer = ''
            state.log_event("STOP")
            continue

        # ── Escape = clear word buffer ─────────────────────────────────────────
        if key == 27:
            cmd_buffer = ''
            continue

        # ── Enter = process word command ──────────────────────────────────────
        if key in (ord('\n'), ord('\r'), curses.KEY_ENTER):
            cmd = cmd_buffer.strip().lower()
            cmd_buffer = ''
            if cmd == 'excavation':
                with state.lock:
                    state.excavation_on = not state.excavation_on
                    val = state.excavation_on
                node.publish_excavation(val)
                state.log_event(f"Excavation {'ON' if val else 'OFF'} (word cmd)")
            elif cmd == 'deposition':
                with state.lock:
                    state.deposition_on = not state.deposition_on
                    val = state.deposition_on
                node.publish_deposition(val)
                state.log_event(f"Deposition {'ON' if val else 'OFF'} (word cmd)")
            elif cmd == 'stop':
                with state.lock:
                    state.linear_x  = 0.0
                    state.angular_z = 0.0
                state.log_event("STOP (word cmd)")
            elif cmd in ('quit', 'q', 'exit'):
                state.log_event("Quit (word cmd)")
                with state.lock:
                    state.running = False
                break
            elif cmd:
                state.log_event(f"Unknown command: '{cmd}'")
            continue

        # ── Backspace ─────────────────────────────────────────────────────────
        if key in (curses.KEY_BACKSPACE, 127, 8):
            cmd_buffer = cmd_buffer[:-1]
            continue

        # ── Remaining printable chars ──────────────────────────────────────────
        if not (32 <= key <= 126):
            continue
        ch = chr(key)

        # If we already have a word buffer, accumulate
        if cmd_buffer:
            cmd_buffer += ch
            continue

        # Buffer empty — single-key commands
        if ch in ('w', 'W'):
            with state.lock:
                state.linear_x  =  LINEAR_SPEED
                state.angular_z =  0.0
                state.last_key  = 'W (forward)'
            state.log_event("Moving FORWARD")

        elif ch in ('s', 'S'):
            with state.lock:
                state.linear_x  = -LINEAR_SPEED
                state.angular_z =  0.0
                state.last_key  = 'S (backward)'
            state.log_event("Moving BACKWARD")

        elif ch in ('a', 'A'):
            with state.lock:
                state.linear_x  =  0.0
                state.angular_z =  ANGULAR_SPEED
                state.last_key  = 'A (turn left)'
            state.log_event("Turning LEFT")

        elif ch == 'd':  # lowercase d = turn right
            with state.lock:
                state.linear_x  =  0.0
                state.angular_z = -ANGULAR_SPEED
                state.last_key  = 'd (turn right)'
            state.log_event("Turning RIGHT")

        elif ch == 'D':  # Shift+D = deposition toggle
            with state.lock:
                state.deposition_on = not state.deposition_on
                val = state.deposition_on
                state.last_key = 'Shift+D (deposition)'
            node.publish_deposition(val)
            state.log_event(f"Deposition {'ON' if val else 'OFF'}")

        elif ch in ('e', 'E'):
            with state.lock:
                state.excavation_on = not state.excavation_on
                val = state.excavation_on
                state.last_key = 'E (excavation)'
            node.publish_excavation(val)
            state.log_event(f"Excavation {'ON' if val else 'OFF'}")

        elif ch in ('q', 'Q'):
            state.log_event("Quit")
            with state.lock:
                state.running = False
            break

        else:
            # Start a word buffer with this character
            cmd_buffer = ch
            with state.lock:
                state.last_key = f"'{ch}' (word mode)"


# ─── Entry Point ──────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    state = RobotState()
    node  = ManualControlNode(state)

    # Spin ROS in background so timers fire independently of the UI loop
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(_run_curses, node, state)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety guarantee: always zero out velocity before exit
        with state.lock:
            state.running = False
        node.publish_stop()
        time.sleep(0.15)   # allow the zero Twist to be flushed
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
