---
sidebar_position: 1
title: Gazebo Simulation Setup
description: Setting up Gazebo for humanoid robot simulation
keywords: [Gazebo, simulation, physics, ROS 2, robot simulation]
---

# Module 2: The Digital Twin - Gazebo Setup

## Introduction to Gazebo

**Gazebo** is an open-source 3D robot simulator that provides:
- **Physics simulation**: Gravity, friction, collisions, dynamics
- **Sensor simulation**: Camera, LIDAR, IMU, force sensors
- **ROS 2 integration**: Seamless communication with ROS 2 nodes
- **Realistic environments**: Build worlds with obstacles, terrain, lighting

### Why Gazebo for Humanoid Robotics?

1. **Free and open-source**: No licensing costs
2. **Physics accuracy**: ODE/Bullet/Dart physics engines
3. **Large ecosystem**: Pre-built models, plugins, worlds
4. **ROS 2 native**: Deep integration via `gazebo_ros_pkgs`
5. **Rapid iteration**: Test without physical robot

### Gazebo vs. Other Simulators

| Simulator | Strengths | Weaknesses | Best For |
|-----------|-----------|------------|----------|
| **Gazebo** | Open-source, ROS 2 integration, mature | Older graphics, slower than Isaac | General robotics, education |
| **NVIDIA Isaac Sim** | Photorealistic, fast, AI-focused | Requires RTX GPU, proprietary | Sim-to-real, AI training |
| **Unity Robotics** | Beautiful graphics, game engine tools | Less physics accuracy | Visualization, VR/AR |
| **PyBullet** | Lightweight, Python-native | Basic visuals, limited ecosystem | Quick prototyping, RL |
| **MuJoCo** | Fast, accurate contact dynamics | Steep learning curve | Research, control theory |

## Gazebo Installation

### Gazebo Versions

- **Gazebo Classic (11)**: Older, stable, still widely used with ROS 2 Humble
- **Gazebo (Fortress, Garden, Harmonic)**: New architecture, better performance

**For this course**: We'll use **Gazebo Classic 11** with ROS 2 Humble (best compatibility).

### Installation on Ubuntu 22.04

```bash
# Install Gazebo Classic with ROS 2 integration
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs

# Install Gazebo Classic itself (if not already installed)
sudo apt install gazebo libgazebo-dev

# Verify installation
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x
```

### Test Installation

```bash
# Launch Gazebo (empty world)
gazebo

# Launch Gazebo with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py

# Check Gazebo topics (should see /clock, /gazebo/... topics)
ros2 topic list
```

## Gazebo Architecture

### Gazebo Components

1. **gzserver** (Physics Server):
   - Runs physics simulation
   - Headless (no GUI)
   - Can run faster than real-time

2. **gzclient** (GUI Client):
   - Visualizes simulation
   - User interface for interaction
   - Optional (can run gzserver alone)

3. **Gazebo plugins**:
   - Extend functionality
   - ROS 2 integration
   - Custom sensors, controllers

### Running Headless (No GUI)

For faster simulation or cloud servers:

```bash
# Server only (no visualization)
gzserver worlds/empty.world

# In another terminal, use ROS 2 to interact
ros2 topic echo /gazebo/link_states
```

## Creating a World

### World File Structure

**worlds/simple_room.world**:
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_room">

    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box_obstacle">
      <static>true</static>
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Launch Custom World

```bash
gazebo --verbose worlds/simple_room.world
```

## Spawning a Robot in Gazebo

### Method 1: Include in World File

```xml
<world name="robot_world">
  <!-- ... physics, lighting ... -->

  <include>
    <uri>model://my_humanoid</uri>
    <pose>0 0 1.0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  </include>
</world>
```

### Method 2: Spawn via ROS 2 (Dynamic)

**Launch file**:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_gazebo')

    # Path to world file
    world_file = os.path.join(pkg_path, 'worlds', 'simple_room.world')

    # Robot URDF
    urdf_file = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }]
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'my_humanoid',
                '-x', '0',
                '-y', '0',
                '-z', '1.0'
            ],
            output='screen'
        ),
    ])
```

**Run**:
```bash
ros2 launch humanoid_gazebo spawn_robot.launch.py
```

## Gazebo-ROS 2 Integration

### Key Topics

When robot is spawned, Gazebo publishes:

```bash
/clock                      # Simulation time
/gazebo/link_states         # All link positions/velocities
/gazebo/model_states        # All model poses

# Robot-specific (depends on plugins)
/joint_states               # Joint positions/velocities
/cmd_vel                    # Velocity commands (if diff-drive plugin)
/camera/image_raw           # Camera images
/scan                       # LIDAR data
```

### Using Simulation Time

**Critical**: Tell ROS 2 nodes to use Gazebo's simulated time, not system clock:

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[{'use_sim_time': True}],  # Essential!
)
```

**Why**: Gazebo can run faster/slower than real-time. Nodes need consistent time source.

## Gazebo Plugins for Sensors

Plugins bridge Gazebo sensors to ROS 2 topics.

### Camera Plugin

Add to URDF:
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>

    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

**Result**: Camera publishes to `/camera/image_raw` and `/camera/camera_info`.

### LIDAR Plugin

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.141592</min_angle>
          <max_angle>3.141592</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Plugin

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Gazebo GUI Basics

### Camera Controls

- **Left mouse**: Rotate view
- **Middle mouse**: Pan
- **Scroll**: Zoom in/out
- **Shift + Left**: Pan

### Model Manipulation

- Select model in scene or left panel
- Use translate/rotate/scale tools in toolbar
- Press `T` (translate), `R` (rotate), `S` (scale) hotkeys

### View Options

- **View → Transparent**: See through models
- **View → Wireframe**: Show mesh structure
- **View → Contacts**: Visualize collision contacts
- **View → Joints**: Show joint axes

### World Properties

- **World → Physics**: Adjust gravity, time step, solver
- **World → Scene**: Lighting, shadows, sky

## Performance Tuning

### Reduce Physics Step Size (Faster Simulation)

```xml
<physics type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Default: 0.001 -->
  <real_time_update_rate>100</real_time_update_rate>  <!-- Default: 1000 -->
</physics>
```

**Trade-off**: Larger steps = faster but less accurate.

### Simplify Collision Geometry

Use primitive shapes instead of complex meshes:

```xml
<collision>
  <geometry>
    <box size="0.5 0.5 1.0"/>  <!-- Fast -->
    <!-- NOT: <mesh filename="complex_model.stl"/> -->
  </geometry>
</collision>
```

### Reduce Sensor Update Rates

```xml
<sensor>
  <update_rate>10</update_rate>  <!-- Lower = less CPU -->
</sensor>
```

## Next Steps

You've learned Gazebo setup and basics. Next, dive deeper into physics simulation and environment building.

👉 **[Next: Physics Simulation →](physics-simulation)**

---

:::tip Gazebo Tips

1. **Start simple**: Empty world → ground plane → simple shapes → complex robot
2. **Test incrementally**: Add one sensor/plugin at a time
3. **Use visualization**: Enable contact points, joints to debug
4. **Check ROS 2 integration**: `ros2 topic list` should show Gazebo topics
5. **Monitor performance**: `top` or `htop` to watch CPU usage

:::

## Lab Exercise

**Create a humanoid test environment**:

1. **World file**:
   - Ground plane
   - 3-4 box obstacles at different positions
   - Proper lighting

2. **Spawn humanoid**:
   - Use launch file to spawn URDF robot
   - Position at (0, 0, 1.0) to start standing

3. **Add sensors**:
   - Camera in head
   - LIDAR on torso
   - IMU in torso

4. **Verify**:
   - `ros2 topic list` shows camera, LIDAR, IMU topics
   - `ros2 topic echo /camera/image_raw` receives images
   - `rviz2` can visualize all sensors

**Bonus**: Add stairs or ramps to test bipedal locomotion.
