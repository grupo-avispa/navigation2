# nav2_controller_py + nav2_py_pure_pursuit

Python port of the Nav2 `controller_server` C++ node, with a Pure Pursuit
example controller plugin.

---

## Architecture

```
nav2_controller_py/                  ← drop-in replacement for C++ controller_server
│
├── nav2_controller_py/
│   ├── ControllerServer.py          ← 1:1 port of controller_server.cpp (LifecycleNode)
│   ├── exceptions.py                ← mirrors nav2_core/controller_exceptions.hpp
│   ├── core/
│   │   ├── ControllerBase.py        ← mirrors nav2_core::Controller (ABC)
│   │   ├── GoalCheckerBase.py       ← mirrors nav2_core::GoalChecker (ABC)
│   │   ├── ProgressCheckerBase.py   ← mirrors nav2_core::ProgressChecker (ABC)
│   │   └── PluginProvider.py        ← entry_points loader (same pattern as plansys2)
│   └── plugins/
│       ├── SimpleGoalChecker.py     ← mirrors nav2_controller::SimpleGoalChecker
│       └── SimpleProgressChecker.py ← mirrors nav2_controller::SimpleProgressChecker
│
├── config/controller_server_params.yaml
└── launch/controller_server.launch.py

nav2_py_pure_pursuit/                ← example Python controller plugin
└── nav2_py_pure_pursuit/
    └── PurePursuitController.py     ← implements ControllerBase
```

### Plugin system

Python plugins use **`importlib.metadata` entry_points** — the same mechanism
used by `plansys2_support_py/PluginProvider`.  There is no need for C++
`pluginlib` XML files.

Every package that provides a plugin registers it in `setup.py`:

```python
entry_points={
    'nav2_controller_py_plugins': [
        'my_package/MyController = my_package.my_controller:MyController',
    ],
},
```

And references it in the YAML params:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "my_package/MyController"
```

---

## Interface mapping: C++ → Python

| C++ (nav2_core::Controller)         | Python (ControllerBase)              |
|-------------------------------------|--------------------------------------|
| `configure(node, name, tf, map)`    | `configure(node, name, tf, costmap)` |
| `cleanup()`                         | `cleanup()`                          |
| `activate()`                        | `activate()`                         |
| `deactivate()`                      | `deactivate()`                       |
| `computeVelocityCommands(...)`      | `compute_velocity_commands(...)`     |
| `setSpeedLimit(limit, pct)`         | `set_speed_limit(limit, pct)`        |
| `newPathReceived(path)`             | `new_path_received(path)`            |
| `reset()`                           | `reset()`                            |
| `cancel()` → bool                   | `cancel()` → bool                    |

---

## Installation

### 1 – Clone into your workspace

```bash
cd ~/ros2_ws/src
# (copy or clone both packages here)
```

### 2 – Build

```bash
cd ~/ros2_ws
colcon build --packages-select nav2_controller_py nav2_py_pure_pursuit --symlink-install
source install/setup.bash
```

> **Important:** `--symlink-install` is needed so that `importlib.metadata`
> picks up the `entry_points` from the editable install.

### 3 – Run

```bash
# Launch the Python controller server with default params:
ros2 launch nav2_controller_py controller_server.launch.py

# Or with your own params file:
ros2 launch nav2_controller_py controller_server.launch.py \
    params_file:=/path/to/your_nav2_params.yaml
```

### 4 – Replace the C++ node in your nav2 bringup

In your existing `nav2_bringup` launch file, change:

```python
# Before (C++ node):
Node(package='nav2_controller', executable='controller_server', ...)

# After (Python node):
LifecycleNode(package='nav2_controller_py', executable='controller_server', ...)
```

---

## Writing your own Python controller plugin

1. Create a new ROS2 Python package:

```bash
ros2 pkg create my_controller --build-type ament_python --dependencies nav2_controller_py
```

2. Implement `ControllerBase`:

```python
# my_controller/my_controller/MyController.py
from nav2_controller_py.core.ControllerBase import ControllerBase
from nav2_controller_py.exceptions import NoValidControl
from geometry_msgs.msg import TwistStamped

class MyController(ControllerBase):

    def configure(self, node, name, tf_buffer, costmap_ros):
        self._node = node
        self._name = name
        # declare and read parameters here

    def cleanup(self): pass
    def activate(self): pass
    def deactivate(self): pass
    def set_speed_limit(self, speed_limit, percentage): pass

    def compute_velocity_commands(self, pose, velocity, goal_checker,
                                   transformed_global_plan, goal_pose):
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.2   # your logic here
        return cmd
```

3. Register the entry_point in `setup.py`:

```python
entry_points={
    'nav2_controller_py_plugins': [
        'my_controller/MyController = my_controller.MyController:MyController',
    ],
},
```

4. Reference in your YAML:

```yaml
FollowPath:
  plugin: "my_controller/MyController"
```

---

## Exception hierarchy

| Exception                | Error code               | When to raise                        |
|--------------------------|--------------------------|--------------------------------------|
| `ControllerException`    | `UNKNOWN`                | Catch-all base class                 |
| `InvalidController`      | `INVALID_CONTROLLER`     | Plugin name not found                |
| `ControllerTFError`      | `TF_ERROR`               | TF transform failure                 |
| `NoValidControl`         | `NO_VALID_CONTROL`       | Cannot compute a valid command       |
| `FailedToMakeProgress`   | `FAILED_TO_MAKE_PROGRESS`| Robot stuck (from progress checker)  |
| `PatienceExceeded`       | `PATIENCE_EXCEEDED`      | `failure_tolerance` time exceeded    |
| `InvalidPath`            | `INVALID_PATH`           | Empty or malformed path              |
| `ControllerTimedOut`     | `CONTROLLER_TIMED_OUT`   | Costmap/resource timeout             |

---

## ROS2 Topics & Actions

| Name                        | Type                             | Direction |
|-----------------------------|----------------------------------|-----------|
| `follow_path`               | `nav2_msgs/action/FollowPath`    | Server    |
| `cmd_vel`                   | `geometry_msgs/msg/TwistStamped` | Publisher |
| `transformed_global_plan`   | `nav_msgs/msg/Path`              | Publisher |
| `speed_limit`               | `nav2_msgs/msg/SpeedLimit`       | Subscriber|
| `odom`                      | `nav_msgs/msg/Odometry`          | Subscriber|
