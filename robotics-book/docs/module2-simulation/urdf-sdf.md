---
sidebar_position: 5
title: URDF and SDF Formats
description: Understanding robot description formats for simulation
keywords: [URDF, SDF, robot description, Gazebo, file formats]
---

# URDF and SDF: Robot Description Formats

## Introduction

**Robot description formats** define the structure, appearance, and physics of robots in simulation and ROS 2. Two main formats:

1. **URDF** (Unified Robot Description Format): ROS standard
2. **SDF** (Simulation Description Format): Gazebo native

### Quick Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Primary Use** | ROS/ROS 2 | Gazebo simulation |
| **Format** | XML | XML |
| **Tree Structure** | Tree only (single robot) | Graph (multiple models, relationships) |
| **Physics** | Basic | Advanced |
| **Sensors** | Limited | Rich plugin ecosystem |
| **Worlds** | No | Yes (environments) |
| **Macros** | XACRO | Direct support |
| **ROS Integration** | Native | Via plugins |

## URDF: ROS Robot Description

### When to Use URDF

- **ROS 2 applications**: Native format
- **Single robot**: Tree hierarchy sufficient
- **Compatibility**: Most ROS packages expect URDF
- **Community**: Large ecosystem, many examples

### URDF Structure

**Key elements**:
```xml
<robot name="my_robot">
  <link name="...">        <!-- Rigid body -->
    <visual>...</visual>    <!-- How it looks -->
    <collision>...</collision>  <!-- Physics shape -->
    <inertial>...</inertial>   <!-- Mass, inertia -->
  </link>

  <joint name="..." type="...">  <!-- Connection -->
    <parent link="..."/>
    <child link="..."/>
    <origin xyz="..." rpy="..."/>
    <axis xyz="..."/>
    <limit lower="..." upper="..."/>
  </joint>
</robot>
```

### URDF Limitations

1. **No closed loops**: Tree structure only (no parallel kinematics)
2. **Single robot**: Can't define multiple robots in one file
3. **No world**: Can't describe environment (ground, obstacles)
4. **Limited sensors**: Must use Gazebo extensions (`<gazebo>` tags)
5. **No materials library**: Colors/textures defined inline

### XACRO: Extending URDF

**XACRO (XML Macros)** adds:
- Variables (`<xacro:property>`)
- Macros (`<xacro:macro>`)
- Math expressions (`${property * 2}`)
- Conditional logic (`<xacro:if>`)
- File inclusion (`<xacro:include>`)

**Example**:
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Properties (variables) -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia
          ixx="${0.5 * wheel_radius * wheel_radius / 2}"
          ixy="0" ixz="0"
          iyy="${0.5 * wheel_radius * wheel_radius / 2}"
          iyz="0"
          izz="${0.5 * wheel_radius * wheel_radius / 2}"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect * 0.2} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <!-- ... -->
  </link>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

**Convert to URDF**:
```bash
xacro robot.urdf.xacro > robot.urdf
```

### Gazebo Extensions in URDF

**Add Gazebo-specific features** with `<gazebo>` tags:

```xml
<robot>
  <!-- URDF links and joints -->
  <link name="base_link">...</link>

  <!-- Gazebo extension -->
  <gazebo reference="base_link">
    <!-- Material (color, texture) -->
    <material>Gazebo/Red</material>

    <!-- Physics properties -->
    <mu1>0.5</mu1>  <!-- Friction coefficient 1 -->
    <mu2>0.5</mu2>  <!-- Friction coefficient 2 -->
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>1.0</kd>  <!-- Contact damping -->
  </gazebo>

  <!-- Gazebo sensor plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros><namespace>/camera</namespace></ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo controller plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

## SDF: Gazebo's Native Format

### When to Use SDF

- **Gazebo-first**: Primary simulator is Gazebo
- **Complex scenes**: Multiple robots, objects, relationships
- **Advanced physics**: Need Gazebo-specific physics features
- **Worlds**: Defining environments (not just robots)

### SDF Structure

**More expressive than URDF**:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Robot model -->
    <model name="my_robot">
      <pose>0 0 1 0 0 0</pose>
      <link name="base_link">
        <pose>0 0 0 0 0 0</pose>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.0</iyy>
            <iyz>0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>

      <!-- Plugins directly in SDF -->
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so"/>
    </model>
  </world>
</sdf>
```

### SDF Advantages Over URDF

1. **Multiple models**: Define multiple robots/objects in one file
2. **World description**: Ground, lighting, physics in same file
3. **Graph structure**: Can represent closed kinematic loops
4. **Native plugins**: Sensors and controllers built-in
5. **Nested models**: Hierarchical model composition
6. **Actors**: Animated humans, scripted motions

### SDF for Worlds

**Defining complete environments**:

```xml
<sdf version="1.7">
  <world name="warehouse">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Obstacles -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Include robot from separate file -->
    <include>
      <uri>model://my_humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Animated human -->
    <actor name="person1">
      <pose>5 0 0 0 0 0</pose>
      <skin>
        <filename>model://person/meshes/person.dae</filename>
      </skin>
      <animation name="walking">
        <filename>model://person/meshes/walk.dae</filename>
      </animation>
      <script>
        <trajectory id="0" type="walking">
          <waypoint>
            <time>0</time>
            <pose>5 0 0 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>5</time>
            <pose>-5 0 0 0 0 0</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>
  </world>
</sdf>
```

## Converting Between Formats

### URDF to SDF

**Automatic** when spawning URDF in Gazebo:
```bash
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot
# Gazebo converts URDF to SDF internally
```

**Manual conversion**:
```bash
# Using gz (Gazebo command-line tool)
gz sdf -p robot.urdf > robot.sdf
```

**Caveats**:
- Some URDF features may not translate perfectly
- Gazebo extensions (`<gazebo>` tags) are preserved

### SDF to URDF

**Not straightforward** (SDF is more expressive):
- No automated tool for full conversion
- Can manually extract single model from SDF world
- Some features (actors, closed loops) can't be represented in URDF

**Workaround**: Keep robot in URDF, world in SDF.

## Best Practices

### Workflow Recommendation

**For ROS 2 + Gazebo Projects**:

1. **Robot**: Define in URDF/XACRO
   - Benefits from ROS 2 ecosystem
   - Works with `robot_state_publisher`, `joint_state_publisher`
   - Compatible with MoveIt, Nav2

2. **World**: Define in SDF
   - Leverage Gazebo's world features
   - Include multiple obstacles, actors
   - Define lighting, physics

3. **Integration**: Launch both
   ```python
   # Launch file
   return LaunchDescription([
       # Load robot URDF
       Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           parameters=[{'robot_description': robot_description}]
       ),

       # Launch Gazebo with SDF world
       ExecuteProcess(
           cmd=['gazebo', '--verbose', world_file],
           output='screen'
       ),

       # Spawn URDF robot into SDF world
       Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=['-topic', '/robot_description', '-entity', 'my_robot']
       ),
   ])
   ```

### File Organization

```
my_robot/
├── urdf/
│   ├── robot.urdf.xacro       # Main robot description (XACRO)
│   ├── robot.urdf              # Generated from XACRO (git-ignored)
│   └── materials.xacro         # Shared materials
├── meshes/
│   ├── base_link.stl
│   ├── arm_link.dae
│   └── ...
├── worlds/
│   ├── warehouse.world         # SDF world
│   ├── office.world
│   └── models/                 # SDF models for obstacles
│       ├── box/
│       │   └── model.sdf
│       └── table/
│           └── model.sdf
└── launch/
    └── simulation.launch.py    # Launches Gazebo + robot
```

### Gazebo Model Path

**For SDF models**:
```bash
export GAZEBO_MODEL_PATH=/path/to/my_robot/worlds/models:$GAZEBO_MODEL_PATH
```

In launch file:
```python
import os
os.environ['GAZEBO_MODEL_PATH'] = '/path/to/models:' + os.environ.get('GAZEBO_MODEL_PATH', '')
```

## Common Pitfalls

### 1. Mixing Formats Incorrectly

**Wrong**:
```xml
<!-- Trying to put SDF world tags in URDF -->
<robot>
  <world>  <!-- ERROR: URDF doesn't support <world> -->
    <light>...</light>
  </world>
</robot>
```

**Right**:
- Keep robot in URDF
- Keep world in SDF
- Spawn robot into world at runtime

### 2. Forgetting XACRO Namespace

**Wrong**:
```xml
<robot name="my_robot">
  <property name="length" value="1.0"/>  <!-- ERROR: not XACRO -->
</robot>
```

**Right**:
```xml
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:property name="length" value="1.0"/>
</robot>
```

### 3. Incorrect File Paths

**Wrong**:
```xml
<mesh filename="my_mesh.stl"/>  <!-- Relative path unclear -->
```

**Right**:
```xml
<mesh filename="package://my_robot_description/meshes/my_mesh.stl"/>
```

## Summary Table

| Use Case | Format | Tool |
|----------|--------|------|
| ROS 2 robot (no sim) | URDF/XACRO | robot_state_publisher |
| Gazebo robot | URDF + `<gazebo>` tags | gazebo_ros |
| Gazebo world | SDF | Gazebo |
| Complex robot (macros) | XACRO → URDF | xacro |
| Multiple robots | SDF world | Gazebo |
| Isaac Sim | USD (convert from URDF) | Isaac URDF importer |

## Next Steps

You've completed Module 2: The Digital Twin! You now understand simulation with Gazebo, Unity, sensors, and robot description formats.

👉 **[Next: Module 3 - The AI-Robot Brain →](../module3-isaac/isaac-sim)**

---

:::tip Format Selection Guide

**Quick decision tree**:
1. Need ROS 2 integration? → URDF/XACRO
2. Single robot only? → URDF/XACRO
3. Complex scene with multiple models? → SDF
4. Just visualization? → Consider Unity (+ ROS bridge)
5. AI training (sim-to-real)? → Isaac Sim (USD)

**Most common**: URDF for robot + SDF for world.

:::

## Lab Exercise

**Convert and test both formats**:

1. **Create XACRO robot**:
   - Define humanoid with macros (left/right legs)
   - Use properties for link dimensions
   - Add Gazebo plugins (camera, LIDAR, IMU)

2. **Create SDF world**:
   - Ground plane
   - 5 box obstacles at random positions
   - Dome light
   - Animated human actor walking

3. **Integration**:
   - Write launch file that:
     - Loads URDF robot
     - Launches SDF world
     - Spawns robot into world
   - Test that all sensors publish to ROS 2
   - Verify actor moves

4. **Conversion test**:
   - Convert XACRO to URDF: `xacro robot.urdf.xacro > robot.urdf`
   - Convert URDF to SDF: `gz sdf -p robot.urdf > robot.sdf`
   - Compare file sizes and structure

**Bonus**: Create second robot and spawn both into the same world.
