---
sidebar_position: 5
title: Launch Files and Configuration
description: Orchestrating complex robot systems with ROS 2 launch files
keywords: [ROS 2, launch files, Python launch, configuration, parameters]
---

# Launch Files and Configuration

## The Problem: Starting Many Nodes

A real robot system has **dozens of nodes** running simultaneously:

```bash
# Manual startup (painful!)
ros2 run camera_driver camera_node &
ros2 run object_detector yolo_node &
ros2 run path_planner planner_node &
ros2 run motor_controller controller_node &
ros2 run rviz2 rviz2 -d config.rviz &
# ... 20 more nodes ...
```

**Problems**:
- Error-prone (forget a node → system breaks)
- No coordination (which node starts first?)
- Hard to configure (parameters scattered across commands)
- Can't reproduce (no record of how system was started)

**Solution**: **Launch files** start all nodes with one command, manage dependencies, and centralize configuration.

## ROS 2 Launch System

### Launch File Types

| Type | Extension | Use Case |
|------|-----------|----------|
| **Python** | `.launch.py` | **Recommended**: Most flexible, programmatic control |
| **XML** | `.launch.xml` | Simpler syntax, less flexibility |
| **YAML** | `.launch.yaml` | Declarative, limited capabilities |

This course focuses on **Python launch files** for maximum power.

## Basic Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            output='screen',
        ),

        Node(
            package='object_detector',
            executable='yolo_node',
            name='detector',
            output='screen',
        ),

        Node(
            package='path_planner',
            executable='planner_node',
            name='planner',
            output='screen',
        ),
    ])
```

**Run**:
```bash
ros2 launch my_package robot.launch.py
```

All three nodes start simultaneously with one command!

## Node Configuration

### Setting Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            parameters=[{
                'frame_rate': 30,
                'resolution': [1920, 1080],
                'auto_exposure': True,
            }],
            output='screen',
        ),
    ])
```

### Loading from YAML File

**config/camera_params.yaml**:
```yaml
camera:
  ros__parameters:
    frame_rate: 30
    resolution: [1920, 1080]
    auto_exposure: true
    exposure_time: 10.0
```

**Launch file**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_directory('my_package')

    # Load parameters from YAML
    params_file = os.path.join(pkg_path, 'config', 'camera_params.yaml')

    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            parameters=[params_file],
            output='screen',
        ),
    ])
```

## Remapping Topics

**Scenario**: `camera_node` publishes to `/image`, but `detector_node` expects `/camera/image`.

**Solution**: Remap topics in launch file:

```python
Node(
    package='camera_driver',
    executable='camera_node',
    name='camera',
    remappings=[
        ('/image', '/camera/image'),
    ],
    output='screen',
),
```

## Namespaces

Run multiple instances of same node (e.g., two cameras):

```python
Node(
    package='camera_driver',
    executable='camera_node',
    name='left_camera',
    namespace='left',
    parameters=[{'device_id': 0}],
),

Node(
    package='camera_driver',
    executable='camera_node',
    name='right_camera',
    namespace='right',
    parameters=[{'device_id': 1}],
),
```

**Result**:
- Left camera publishes to: `/left/image`
- Right camera publishes to: `/right/image`

## Launch Arguments (Command-Line Parameters)

Make launch files configurable:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device ID'
    )

    # Get argument values
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_device = LaunchConfiguration('camera_device')

    return LaunchDescription([
        use_sim_time_arg,
        camera_device_arg,

        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
            parameters=[{
                'use_sim_time': use_sim_time,
                'device_id': camera_device,
            }],
            output='screen',
        ),
    ])
```

**Usage**:
```bash
# Default values
ros2 launch my_package robot.launch.py

# Override arguments
ros2 launch my_package robot.launch.py use_sim_time:=true camera_device:=1
```

## Including Other Launch Files

**Modular approach**: Break complex system into multiple launch files.

**robot_bringup.launch.py**:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    sensors_pkg = get_package_share_directory('sensors_package')
    perception_pkg = get_package_share_directory('perception_package')
    navigation_pkg = get_package_share_directory('navigation_package')

    return LaunchDescription([
        # Include sensors launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors_pkg, 'launch', 'sensors.launch.py')
            ),
        ),

        # Include perception launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_pkg, 'launch', 'perception.launch.py')
            ),
        ),

        # Include navigation launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_pkg, 'launch', 'navigation.launch.py')
            ),
        ),
    ])
```

### Passing Arguments to Included Launch Files

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(sensors_pkg, 'launch', 'sensors.launch.py')
    ),
    launch_arguments={
        'use_sim_time': 'true',
        'camera_device': '0',
    }.items(),
),
```

## Conditional Execution

Start nodes based on conditions:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz if true'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        use_rviz_arg,

        # Always start these nodes
        Node(
            package='camera_driver',
            executable='camera_node',
            name='camera',
        ),

        # Conditionally start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            condition=IfCondition(use_rviz),
            arguments=['-d', '/path/to/config.rviz'],
        ),
    ])
```

**Usage**:
```bash
# Start with RViz (default)
ros2 launch my_package robot.launch.py

# Start without RViz
ros2 launch my_package robot.launch.py use_rviz:=false
```

## Loading URDF/Robot Description

**Common pattern**: Load robot URDF and publish to `/robot_description` topic.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # Process xacro file to URDF
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Robot state publisher (publishes transforms and robot description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        ),

        # Joint state publisher (publishes joint states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'robot.rviz')],
        ),
    ])
```

## Practical Example: Humanoid Robot Launch

**humanoid_bringup.launch.py** (complete system startup):

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Package paths
    robot_desc_pkg = get_package_share_directory('humanoid_description')
    sensors_pkg = get_package_share_directory('humanoid_sensors')
    control_pkg = get_package_share_directory('humanoid_control')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    # Configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot description (URDF)
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'humanoid.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Parameter files
    controller_params = os.path.join(control_pkg, 'config', 'controllers.yaml')
    sensor_params = os.path.join(sensors_pkg, 'config', 'sensors.yaml')

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_rviz_arg,

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Sensor drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensors_pkg, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': sensor_params,
            }.items(),
        ),

        # Controllers (balance, locomotion)
        Node(
            package='humanoid_control',
            executable='balance_controller',
            name='balance_controller',
            parameters=[controller_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='humanoid_control',
            executable='locomotion_controller',
            name='locomotion_controller',
            parameters=[controller_params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # VLA cognitive layer (AI planning)
        Node(
            package='humanoid_ai',
            executable='vla_planner',
            name='vla_planner',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # RViz (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(robot_desc_pkg, 'rviz', 'humanoid.rviz')],
        ),
    ])
```

**Usage**:
```bash
# Real robot
ros2 launch humanoid_bringup humanoid_bringup.launch.py

# Simulation (with RViz)
ros2 launch humanoid_bringup humanoid_bringup.launch.py use_sim_time:=true

# Simulation (headless, no RViz)
ros2 launch humanoid_bringup humanoid_bringup.launch.py \
  use_sim_time:=true use_rviz:=false
```

## Event Handlers

Execute actions based on events:

### Start Node After Another Node is Ready

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
        package='camera_driver',
        executable='camera_node',
        name='camera',
    )

    # Start detector only after camera is started
    detector_node = Node(
        package='object_detector',
        executable='yolo_node',
        name='detector',
    )

    return LaunchDescription([
        camera_node,

        RegisterEventHandler(
            OnProcessStart(
                target_action=camera_node,
                on_start=[detector_node],
            )
        ),
    ])
```

### Restart Node on Exit

```python
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent
from launch.events import Shutdown

# Shutdown entire system if critical node exits
RegisterEventHandler(
    OnProcessExit(
        target_action=balance_controller,
        on_exit=[EmitEvent(event=Shutdown(reason='Balance controller died'))],
    )
),
```

## Logging Configuration

Control log output levels:

```python
Node(
    package='my_package',
    executable='my_node',
    name='my_node',
    output='screen',
    arguments=['--ros-args', '--log-level', 'debug'],
),
```

**Log levels**: `debug`, `info`, `warn`, `error`, `fatal`

## Best Practices

### 1. Organize by Function

```
launch/
  ├── sensors.launch.py          # All sensor drivers
  ├── perception.launch.py       # Object detection, SLAM
  ├── navigation.launch.py       # Path planning, Nav2
  ├── control.launch.py          # Motor controllers
  └── full_system.launch.py      # Includes all of the above
```

### 2. Use Parameters Files

**Don't**:
```python
Node(
    parameters=[{
        'param1': 10,
        'param2': 20,
        'param3': 'value',
        # 50 more parameters...
    }]
)
```

**Do**:
```python
params_file = os.path.join(pkg_path, 'config', 'node_params.yaml')
Node(parameters=[params_file])
```

### 3. Provide Defaults, Allow Overrides

```python
DeclareLaunchArgument(
    'camera_device',
    default_value='0',  # Good default
    description='Camera device ID'
)
```

### 4. Document Launch Arguments

```python
DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) time instead of system time. '
                'Set to true when running in simulation.'
)
```

## Debugging Launch Files

### Print Launch Configuration

```bash
ros2 launch my_package robot.launch.py --show-args
# Shows all available arguments and defaults
```

### Check What Nodes Will Start

```bash
ros2 launch my_package robot.launch.py --print-description
# Shows nodes that will be launched
```

### Verbose Output

```bash
ros2 launch -v my_package robot.launch.py
# Shows detailed launch process
```

## Next Steps

Congratulations! You've completed Module 1: The Robotic Nervous System (ROS 2). You can now:
- Create ROS 2 nodes in Python
- Communicate via topics, services, and actions
- Define robot morphology with URDF
- Orchestrate complex systems with launch files

Next, dive into Module 2 to learn physics simulation with Gazebo and Unity.

👉 **[Next: Module 2 - The Digital Twin →](../module2-simulation/gazebo-setup)**

---

:::tip Launch File Tips

1. **Start small**: Build launch files incrementally, test each addition
2. **Use IncludeLaunchDescription**: Modularize for reusability
3. **Parameterize everything**: Avoid hardcoded values
4. **Provide good defaults**: Users shouldn't need to specify arguments
5. **Document**: Clear descriptions for all launch arguments

:::

## Lab Exercise

**Create a humanoid sensor launch system**:

1. **sensors.launch.py**:
   - RealSense camera driver
   - LIDAR driver
   - IMU driver
   - Load parameters from `config/sensors.yaml`

2. **perception.launch.py**:
   - YOLO object detector (subscribes to camera)
   - VSLAM node (subscribes to camera + IMU)
   - Publishes to `/detections` and `/odometry`

3. **robot.launch.py** (full system):
   - Include sensors.launch.py
   - Include perception.launch.py
   - Start robot_state_publisher with URDF
   - Conditionally start RViz

4. **Test**:
   - Launch with: `ros2 launch my_package robot.launch.py`
   - Verify all nodes running: `ros2 node list`
   - Check topics: `ros2 topic list`

**Bonus**: Add launch arguments for camera resolution and YOLO model selection.
