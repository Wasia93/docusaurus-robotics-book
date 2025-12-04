---
sidebar_position: 4
title: URDF for Humanoids
description: Understanding Unified Robot Description Format for defining humanoid robot morphology
keywords: [URDF, robot description, humanoid, ROS 2, kinematics, XACRO]
---

# URDF for Humanoids

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format for describing a robot's:
- **Physical structure**: Links (rigid bodies) and joints (connections)
- **Visual appearance**: 3D meshes and colors
- **Collision geometry**: Simplified shapes for physics simulation
- **Inertial properties**: Mass, center of mass, moments of inertia
- **Sensors and actuators**: Cameras, LIDARs, motors

Think of URDF as the **blueprint** of your robot—it defines the robot's body in a way that simulators (Gazebo, Isaac Sim) and ROS 2 can understand.

## Why URDF for Humanoids?

Humanoid robots are complex:
- **30-50+ degrees of freedom** (DoF): Each joint needs definition
- **Kinematic chains**: Arms, legs, torso form interconnected chains
- **Balance constraints**: Center of mass, foot contact critical
- **Sensor placement**: Cameras, IMUs at specific locations

URDF provides a standardized way to represent this complexity.

## URDF Structure: Links and Joints

### Basic Concepts

**Link** = Rigid body (bone, limb segment)
- Has mass, inertia, visual/collision geometry

**Joint** = Connection between links
- Types: revolute (hinge), prismatic (slider), fixed, continuous

**Kinematic Tree**: Links connected by joints forming a tree structure
```
        [Torso]
       /   |   \
   [L_Arm] | [R_Arm]
          [Pelvis]
          /      \
     [L_Leg]  [R_Leg]
```

### Minimal URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base link (required root) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

## Link Elements

### Visual Geometry

Defines how the robot looks:

```xml
<link name="torso">
  <visual>
    <!-- Transform from link origin to visual -->
    <origin xyz="0 0 0.3" rpy="0 0 0"/>

    <!-- Geometry options -->
    <geometry>
      <!-- Option 1: Basic shapes -->
      <box size="0.3 0.2 0.6"/>  <!-- width, depth, height -->
      <!-- OR -->
      <cylinder radius="0.1" length="0.6"/>
      <!-- OR -->
      <sphere radius="0.1"/>
      <!-- OR -->
      <!-- Option 2: 3D mesh -->
      <mesh filename="package://robot_description/meshes/torso.stl" scale="1.0 1.0 1.0"/>
    </geometry>

    <!-- Material (color or texture) -->
    <material name="white">
      <color rgba="1 1 1 1"/>  <!-- RGBA: 0-1 -->
    </material>
  </visual>
</link>
```

### Collision Geometry

Simplified geometry for physics simulation (faster than visual):

```xml
<collision>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <geometry>
    <!-- Use simpler geometry than visual for performance -->
    <box size="0.3 0.2 0.6"/>
  </geometry>
</collision>
```

**Best practice**: Use primitive shapes (box, cylinder, sphere) for collision, detailed meshes for visual.

### Inertial Properties

Physical properties for dynamics simulation:

```xml
<inertial>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Center of mass -->
  <mass value="5.0"/>  <!-- kg -->

  <!-- Inertia tensor (3x3 matrix) -->
  <inertia
    ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.1" iyz="0.0"
                          izz="0.05"/>
</inertial>
```

**Calculating inertia**: For basic shapes, use formulas:
- **Box**: `ixx = (1/12) * m * (h² + d²)`, etc.
- **Cylinder**: `ixx = (1/12) * m * (3r² + h²)`, `izz = (1/2) * m * r²`
- **MeshLab/Blender**: For complex meshes, use CAD tools

## Joint Types

### 1. Revolute (Hinge)

Most common for humanoid joints (elbow, knee, shoulder):

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>

  <!-- Joint location and orientation -->
  <origin xyz="0 0 0.3" rpy="0 0 0"/>

  <!-- Rotation axis (local frame) -->
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->

  <!-- Joint limits -->
  <limit
    lower="-2.0"       <!-- Min angle (rad) -->
    upper="0.0"        <!-- Max angle (rad) -->
    effort="50.0"      <!-- Max torque (Nm) -->
    velocity="2.0"/>   <!-- Max angular velocity (rad/s) -->

  <!-- Dynamics (optional) -->
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

### 2. Continuous (Unlimited Rotation)

For wheels or spinning joints:

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="10.0" velocity="5.0"/>  <!-- No lower/upper -->
</joint>
```

### 3. Prismatic (Slider)

For linear motion (telescoping limbs):

```xml
<joint name="lift_joint" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Move along Z -->
  <limit
    lower="0.0"
    upper="0.5"        <!-- Max extension (meters) -->
    effort="100.0"
    velocity="0.2"/>
</joint>
```

### 4. Fixed

For rigidly attached parts (no motion):

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

## Humanoid URDF Example: Simple Biped

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base: Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.35" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.8 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint (revolute, pitch) -->
  <joint name="neck_pitch" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Right Upper Leg -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.04" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.0075"/>
    </inertial>
  </link>

  <!-- Right hip joint (revolute, pitch) -->
  <joint name="right_hip_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="0 -0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="2.0"/>
  </joint>

  <!-- Right Lower Leg -->
  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.027" ixy="0" ixz="0" iyy="0.027" iyz="0" izz="0.0032"/>
    </inertial>
  </link>

  <!-- Right knee joint (revolute, pitch) -->
  <joint name="right_knee_pitch" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.5" effort="80" velocity="2.0"/>
  </joint>

  <!-- Right Foot -->
  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Right ankle joint (revolute, pitch) -->
  <joint name="right_ankle_pitch" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7" upper="0.7" effort="50" velocity="1.5"/>
  </joint>

  <!-- Left leg: Mirror of right leg (omitted for brevity) -->
  <!-- Arms: Similar structure (omitted for brevity) -->

</robot>
```

## XACRO: Macros for URDF

**Problem**: URDF is verbose and repetitive (left/right arms/legs are nearly identical).

**Solution**: **XACRO** (XML Macros) extends URDF with:
- Variables
- Math expressions
- Macros (functions) for reusable components
- Conditional statements

### XACRO Example: Parametrized Leg

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties (variables) -->
  <xacro:property name="upper_leg_length" value="0.4"/>
  <xacro:property name="lower_leg_length" value="0.4"/>
  <xacro:property name="upper_leg_mass" value="3.0"/>
  <xacro:property name="lower_leg_mass" value="2.0"/>

  <!-- Macro for a complete leg -->
  <xacro:macro name="leg" params="prefix reflect">

    <!-- Upper leg link -->
    <link name="${prefix}_upper_leg">
      <visual>
        <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="${upper_leg_length}"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <inertial>
        <origin xyz="0 0 ${-upper_leg_length/2}" rpy="0 0 0"/>
        <mass value="${upper_leg_mass}"/>
        <inertia
          ixx="${upper_leg_mass * (3*0.05*0.05 + upper_leg_length*upper_leg_length) / 12}"
          ixy="0" ixz="0"
          iyy="${upper_leg_mass * (3*0.05*0.05 + upper_leg_length*upper_leg_length) / 12}"
          iyz="0"
          izz="${upper_leg_mass * 0.05*0.05 / 2}"/>
      </inertial>
    </link>

    <!-- Hip joint -->
    <joint name="${prefix}_hip_pitch" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_leg"/>
      <origin xyz="0 ${reflect * 0.1} -0.3" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-2.0" upper="2.0" effort="100" velocity="2.0"/>
    </joint>

    <!-- Lower leg link -->
    <link name="${prefix}_lower_leg">
      <visual>
        <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="${lower_leg_length}"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <inertial>
        <origin xyz="0 0 ${-lower_leg_length/2}" rpy="0 0 0"/>
        <mass value="${lower_leg_mass}"/>
        <inertia
          ixx="${lower_leg_mass * (3*0.04*0.04 + lower_leg_length*lower_leg_length) / 12}"
          ixy="0" ixz="0"
          iyy="${lower_leg_mass * (3*0.04*0.04 + lower_leg_length*lower_leg_length) / 12}"
          iyz="0"
          izz="${lower_leg_mass * 0.04*0.04 / 2}"/>
      </inertial>
    </link>

    <!-- Knee joint -->
    <joint name="${prefix}_knee_pitch" type="revolute">
      <parent link="${prefix}_upper_leg"/>
      <child link="${prefix}_lower_leg"/>
      <origin xyz="0 0 ${-upper_leg_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0.0" upper="2.5" effort="80" velocity="2.0"/>
    </joint>

  </xacro:macro>

  <!-- Instantiate left and right legs -->
  <xacro:leg prefix="right" reflect="-1"/>
  <xacro:leg prefix="left" reflect="1"/>

</robot>
```

### Converting XACRO to URDF

```bash
# Process xacro file to generate URDF
xacro robot.urdf.xacro > robot.urdf

# Or load directly in launch file (ROS 2)
xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
robot_desc = Command(['xacro ', xacro_file])
```

## Sensors in URDF

### Camera

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera simulation -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

### LIDAR

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="torso"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.35" rpy="0 0 0"/>
</joint>

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
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <topicName>/scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Visualizing and Validating URDF

### check_urdf Tool

```bash
# Validate URDF syntax
check_urdf robot.urdf

# Output shows:
# - Joint and link tree
# - Any errors or warnings
```

### urdf_to_graphviz

```bash
# Generate visual graph of robot structure
urdf_to_graphviz robot.urdf

# Creates PDF showing kinematic tree
```

### RViz Visualization

```bash
# Launch RViz with robot model
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf

# Use joint_state_publisher_gui to move joints
```

## Next Steps

Now that you can define robot morphology with URDF, learn how to orchestrate multiple nodes with launch files.

👉 **[Next: Launch Files and Configuration →](launch-files)**

---

:::tip URDF Best Practices

1. **Start simple**: Build and test incrementally (torso → legs → arms)
2. **Use XACRO**: Avoid repetition, use macros for symmetric parts
3. **Validate often**: Run `check_urdf` after every change
4. **Realistic inertia**: Wrong inertia causes unstable simulation
5. **Collision < Visual**: Use simpler geometry for collision for performance
6. **Sensor placement**: Carefully position cameras/LIDAR for proper field of view

:::

## Lab Exercise

**Create a simple humanoid URDF**:

1. **Basic structure**:
   - Torso (box)
   - Head (sphere) with neck joint
   - 2 arms with shoulder/elbow joints
   - 2 legs with hip/knee/ankle joints

2. **Add sensors**:
   - Camera in head
   - IMU in torso

3. **Validate and visualize**:
   - Run `check_urdf`
   - Visualize in RViz
   - Move joints with `joint_state_publisher_gui`

4. **Convert to XACRO**:
   - Create macro for leg
   - Use parameters for dimensions
   - Instantiate left/right legs

**Bonus**: Add realistic masses and inertias for a 50kg humanoid.
