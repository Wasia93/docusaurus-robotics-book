# Assessment: Gazebo Simulation Implementation

## Overview

This assessment evaluates your ability to create realistic robot simulations using Gazebo. You'll design a custom environment, implement physics-based simulations, and integrate sensors for a mobile manipulator robot.

## Learning Objectives

By completing this assessment, you will demonstrate:

- Proficiency in Gazebo world creation and environment design
- Understanding of URDF/SDF robot modeling
- Ability to simulate sensors (cameras, LiDAR, IMU)
- Knowledge of physics simulation parameters
- Integration of Gazebo with ROS 2

## Project Requirements

### Task: Warehouse Mobile Manipulator Simulation

Create a complete Gazebo simulation of a mobile manipulator operating in a warehouse environment.

#### 1. World Design

##### Warehouse Environment

Create a warehouse world with:

- **Floor**: 20m x 20m area with realistic friction
- **Shelving units**: At least 4 shelf racks with varying heights
- **Objects**: 10-15 objects of different shapes (boxes, cylinders, spheres)
- **Obstacles**: Static obstacles (walls, columns, barriers)
- **Lighting**: Multiple light sources for realistic rendering
- **Textures**: Realistic materials for floor, walls, objects

**warehouse.world**:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="warehouse">
    <!-- Physics settings -->
    <physics name="default_physics" default="0" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Warehouse floor -->
    <model name="warehouse_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>20 20 0.1</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Shelving units (repeat for multiple shelves) -->
    <include>
      <uri>model://shelf</uri>
      <pose>3 0 0 0 0 0</pose>
      <name>shelf_1</name>
    </include>

    <!-- Objects on shelves -->
    <!-- TODO: Add multiple objects with different properties -->

  </world>
</sdf>
```

#### 2. Robot Model

##### Mobile Manipulator URDF

Create a mobile base with:
- **Differential drive**: Two driven wheels + two casters
- **6-DOF arm**: Mounted on top of mobile base
- **2-finger gripper**: End effector
- **Sensors**: Camera, LiDAR, IMU

**Key URDF components**:

```xml
<?xml version="1.0"?>
<robot name="mobile_manipulator" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Mobile base -->
  <link name="base_link">
    <inertial>
      <mass value="50.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheels (differential drive) -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <geometry>
          <cylinder radius="0.15" length="0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.15" length="0.05"/>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect*0.25} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

  <!-- Arm links and joints -->
  <!-- TODO: Define 6-DOF arm structure -->

  <!-- Gripper -->
  <!-- TODO: Define 2-finger gripper -->

  <!-- Sensors -->
  <!-- TODO: Add camera, LiDAR, IMU -->

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="differential_drive_controller"
            filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.3</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

#### 3. Sensor Simulation

##### Required Sensors

**a) RGB-D Camera**
```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.05</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

**b) 2D LiDAR**
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.2</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**c) IMU**
```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 4. Physics Simulation

Configure realistic physics parameters:

```xml
<gazebo>
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
      <namespace>/gazebo</namespace>
    </ros>
    <update_rate>1.0</update_rate>
  </plugin>
</gazebo>
```

**Contact properties**:
```xml
<gazebo reference="gripper_finger_link">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
```

#### 5. ROS 2 Integration

##### Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mobile_manipulator_sim')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'warehouse.world'),
            'verbose': 'true'
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_manipulator',
            '-file', os.path.join(pkg_dir, 'urdf', 'robot.urdf'),
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[os.path.join(pkg_dir, 'urdf', 'robot.urdf')]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_pub
    ])
```

## Testing Requirements

### 1. Simulation Stability
- Run simulation for 10 minutes without crashes
- Verify physics stability (no objects falling through floor)
- Check real-time factor (should be close to 1.0)

### 2. Sensor Validation
- Verify camera publishes images at correct rate
- Check LiDAR scan ranges are reasonable
- Validate IMU data (gravity, accelerations)

### 3. Motion Tests
- Mobile base drives smoothly
- Arm reaches target poses without collisions
- Gripper grasps and holds objects

### 4. Performance Metrics
- Measure simulation FPS
- Monitor CPU and memory usage
- Verify sensor update rates

## Deliverables

### 1. Simulation Package
```
mobile_manipulator_sim/
├── package.xml
├── CMakeLists.txt
├── worlds/
│   └── warehouse.world
├── models/
│   ├── shelves/
│   └── objects/
├── urdf/
│   ├── robot.urdf.xacro
│   └── robot.urdf
├── launch/
│   └── simulation.launch.py
├── config/
│   └── physics_params.yaml
└── README.md
```

### 2. Documentation
- System architecture diagram
- URDF structure explanation
- Sensor specifications
- Physics parameter justification
- Usage instructions

### 3. Demonstration Video
- Show warehouse environment
- Demonstrate mobile base navigation
- Show arm manipulation
- Display sensor data (camera, LiDAR, IMU)
- Demonstrate object grasping

### 4. Performance Report
- Simulation stability metrics
- Sensor performance (update rates, latency)
- Physics realism evaluation
- Computational requirements

## Evaluation Criteria

| Criteria | Points | Description |
|----------|--------|-------------|
| **World Design** | 15 | Realistic warehouse with objects |
| **Robot Model** | 20 | Proper URDF structure, physics |
| **Sensor Simulation** | 20 | Correct sensor configuration |
| **Physics** | 15 | Realistic physics parameters |
| **ROS 2 Integration** | 15 | Proper launch files, topics |
| **Stability** | 10 | Simulation runs without crashes |
| **Documentation** | 5 | Clear documentation |

**Total: 100 points**

## Tips for Success

1. **Start with simple models**: Build complexity gradually
2. **Test sensors independently**: Verify each sensor before integration
3. **Tune physics parameters**: Adjust step size, friction, damping
4. **Use visualization**: Enable contact visualization, sensor rays
5. **Profile performance**: Monitor resource usage

## Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)

## Extension Challenges (Optional)

1. **Dynamic objects**: Objects that can be pushed/moved
2. **Multiple robots**: Coordinate two robots
3. **Advanced sensors**: 3D LiDAR, force/torque sensors
4. **Deformable objects**: Soft grasping simulation

Good luck with your Gazebo simulation!
