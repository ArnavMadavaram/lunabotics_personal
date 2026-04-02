# robot_manual_control

Keyboard teleoperation node for manual testing of the Lunabotics robot.
Provides a live curses terminal dashboard while the autonomy stack is under development.

---

## How to build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_manual_control
source install/setup.bash
```

---

## How to launch

### Recommended — run directly (best curses compatibility)
```bash
ros2 run robot_manual_control manual_control
```

### Via launch file
```bash
ros2 launch robot_manual_control manual_control.launch.py
```

> **Note:** The curses UI requires a real TTY attached to the terminal.
> If you see garbled output when using the launch file, fall back to `ros2 run`.

---

## Key bindings

| Key | Action |
|---|---|
| W or ↑ | Forward |
| S or ↓ | Backward |
| A or ← | Turn left |
| D or → | Turn right |
| Space | Stop immediately |
| E | Toggle excavation system |
| Shift+D | Toggle deposition system |
| Q or Ctrl+C | Quit (publishes zero Twist before exit) |

### Word commands (type full word + Enter)
| Command | Action |
|---|---|
| `excavation` | Toggle excavation |
| `deposition` | Toggle deposition |
| `stop` | Stop all motion |
| `quit` | Exit node |

Press **Escape** to clear the word buffer without executing.

---

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Drive commands, published at 10 Hz continuously |
| `/excavation/cmd` | `std_msgs/Bool` | Excavation system on/off toggle |
| `/deposition/cmd` | `std_msgs/Bool` | Deposition system on/off toggle |

---

## ECE team — values to tune

All tunable constants are at the top of `nodes/manual_control.py`:

```python
LINEAR_SPEED  = 0.3   # m/s  — safe max forward/backward speed
ANGULAR_SPEED = 0.8   # rad/s — safe max yaw rate
```

**Procedure:**
1. Start with both values low (e.g. `LINEAR_SPEED = 0.1`, `ANGULAR_SPEED = 0.3`).
2. Verify the robot moves in the correct direction for each key.
   - If forward/backward is reversed, negate `LINEAR_SPEED` or swap your motor wiring.
   - If left/right is reversed, negate `ANGULAR_SPEED`.
3. Gradually increase until you reach a comfortable operating speed.
4. Document the final values here once agreed with the software team.

### Motor interface

The node publishes to placeholder topic names.  Hook your motor driver node up to:

- `/cmd_vel` — standard ROS `geometry_msgs/Twist`:
  - `linear.x`  → forward (+) / backward (−) velocity command
  - `angular.z` → left-turn (+) / right-turn (−) rate command
  - All other fields are zero.

- `/excavation/cmd` — `std_msgs/Bool`, `data=true` means activate, `false` means deactivate.
  **TODO ECE:** subscribe to this topic in your excavation driver node.

- `/deposition/cmd` — `std_msgs/Bool`, `data=true` means activate, `false` means deactivate.
  **TODO ECE:** subscribe to this topic in your deposition driver node.

---

## Assumptions

- **Build system:** ROS2 Python packages use `setup.py` + `ament_python` instead
  of `CMakeLists.txt`.  `setup.py` is the functional equivalent for Python nodes.
- **Key-held behaviour:** Pressing and holding a movement key sets a continuous
  velocity that persists until another key is pressed.  The node does *not* stop
  automatically on key release (no key-release events in terminal input).
  Always press **Space** or **S/W** to change speed.
- **Launch file name:** Named `manual_control.launch.py` following the ROS2
  Python launch convention.  The `.launch` (XML) format is ROS1-style and is
  not used here.
- **Thread safety:** The ROS publisher calls made from the UI thread are safe
  under rclpy/DDS — publishers are thread-safe for concurrent `publish()` calls.
