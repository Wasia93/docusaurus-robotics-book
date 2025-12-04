---
sidebar_position: 2
title: Physics Simulation
description: Understanding physics engines and simulating humanoid dynamics
keywords: [physics simulation, ODE, dynamics, collision, humanoid balance]
---

# Physics Simulation for Humanoids

## Why Physics Simulation?

Humanoid robots are **dynamic systems** governed by physics:
- **Gravity** pulls them down
- **Friction** determines slipping
- **Collisions** with ground and obstacles
- **Inertia** affects movement and balance
- **Joint dynamics** (motors, damping, friction)

Accurate simulation is critical for:
1. **Testing without hardware**: Develop algorithms before deploying
2. **Safety**: Debug dangerous scenarios (falling, collisions) safely
3. **Sim-to-real transfer**: Train neural networks in simulation, deploy to robot
4. **Rapid iteration**: Test 100x faster than real-time

## Physics Engines

Gazebo supports multiple physics engines:

| Engine | Strengths | Weaknesses | Use Case |
|--------|-----------|------------|----------|
| **ODE** | Fast, stable, default in Gazebo | Less accurate contacts | General robotics |
| **Bullet** | Good collision detection | Can be unstable | Multi-body dynamics |
| **Dart** | Accurate, good for humanoids | Slower | Precise simulation, research |
| **Simbody** | High accuracy | Complex setup | Biomechanics |

**For humanoids**: **DART** is recommended for accuracy, **ODE** for speed.

### Selecting Physics Engine

In world file:
```xml
<world name="humanoid_world">
  <physics type="dart">  <!-- or "ode", "bullet" -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
</world>
```

## Physics Parameters

### Time Stepping

**max_step_size**: How often physics is updated

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms, 1000 Hz -->
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

**Trade-offs**:
- **Smaller step** (0.0001): More accurate, slower
- **Larger step** (0.01): Faster, less accurate, can be unstable

**Humanoid recommendation**: 0.001 (1 kHz) for balance, 0.002 for efficiency.

### Real-Time Factor

**real_time_factor**: Simulation speed relative to real-time

```xml
<real_time_factor>1.0</real_time_factor>  <!-- 1x real-time -->
<real_time_factor>0.5</real_time_factor>  <!-- 2x slower (more accurate) -->
<real_time_factor>2.0</real_time_factor>  <!-- 2x faster (if CPU allows) -->
```

**Note**: If simulation is heavy, actual speed may be slower than target.

### Gravity

```xml
<gravity>0 0 -9.81</gravity>  <!-- Earth gravity (m/s²) -->
```

Custom gravity for testing:
```xml
<gravity>0 0 -3.71</gravity>  <!-- Mars gravity -->
<gravity>0 0 -1.62</gravity>  <!-- Moon gravity -->
```

## Contact Dynamics

### Friction

**Critical for humanoids**: Feet must not slip on ground.

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>    <!-- Friction coefficient (0-inf) -->
      <mu2>1.0</mu2>  <!-- Secondary friction direction -->
    </ode>
  </friction>
</surface>
```

**Friction coefficients**:
- `mu = 0`: Frictionless (ice)
- `mu = 0.5`: Low friction (wet surface)
- `mu = 1.0`: Normal (concrete, rubber)
- `mu = 2.0`: High friction (rubber on rubber)

**Example**: High-friction feet
```xml
<link name="right_foot">
  <collision name="collision">
    <geometry>
      <box size="0.2 0.1 0.05"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.5</mu>   <!-- Prevent slipping -->
          <mu2>1.5</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Contact Stiffness and Damping

Controls "softness" of collisions:

```xml
<surface>
  <contact>
    <ode>
      <kp>1000000.0</kp>  <!-- Contact stiffness (N/m) -->
      <kd>1.0</kd>        <!-- Contact damping (N·s/m) -->
    </ode>
  </contact>
</surface>
```

**Higher kp**: Harder contacts (less penetration, more bounce)
**Lower kp**: Softer contacts (some penetration, less bounce)

### Bounce (Restitution)

How much energy is preserved in collisions:

```xml
<surface>
  <bounce>
    <restitution_coefficient>0.0</restitution_coefficient>  <!-- 0-1 -->
    <threshold>0.01</threshold>  <!-- Min velocity for bounce -->
  </bounce>
</surface>
```

- `restitution = 0.0`: Inelastic (no bounce)
- `restitution = 0.5`: Loses 50% energy
- `restitution = 1.0`: Perfectly elastic (bounce forever)

**Humanoid feet**: Use 0.0 (no bounce, stable standing).

## Joint Dynamics

### Joint Damping

Resists motion (like viscous friction):

```xml
<joint name="knee_pitch" type="revolute">
  <!-- ... -->
  <dynamics>
    <damping>0.7</damping>  <!-- N·m·s/rad -->
  </dynamics>
</joint>
```

**Effect**: Higher damping = slower, more controlled motion.

**Typical values**:
- Small joints (fingers): 0.1-0.3
- Medium joints (elbows, knees): 0.5-1.0
- Large joints (hips, shoulders): 1.0-2.0

### Joint Friction

Static friction at joint:

```xml
<dynamics>
  <friction>0.1</friction>  <!-- N·m -->
</dynamics>
```

**Effect**: Minimum torque to start moving joint.

### Joint Limits

Enforce physical constraints:

```xml
<joint name="knee_pitch" type="revolute">
  <axis xyz="0 1 0"/>
  <limit>
    <lower>0.0</lower>       <!-- Fully extended -->
    <upper>2.5</upper>       <!-- Max flexion (rad) -->
    <effort>100.0</effort>   <!-- Max torque (N·m) -->
    <velocity>5.0</velocity> <!-- Max angular velocity (rad/s) -->
  </limit>
</joint>
```

**Safety**: Prevents unrealistic poses and motor overload.

## Inertial Properties

### Why Inertia Matters

**Incorrect inertia** → Unstable simulation

Mass and inertia determine:
- How much force/torque is needed to move
- Rotational dynamics (spinning, tipping)
- Balance stability

### Calculating Inertia

For basic shapes:

**Box** (dimensions: x, y, z):
```
ixx = (1/12) * m * (y² + z²)
iyy = (1/12) * m * (x² + z²)
izz = (1/12) * m * (x² + y²)
```

**Cylinder** (radius: r, height: h, along z-axis):
```
ixx = (1/12) * m * (3r² + h²)
iyy = (1/12) * m * (3r² + h²)
izz = (1/2) * m * r²
```

**Sphere** (radius: r):
```
ixx = iyy = izz = (2/5) * m * r²
```

### Example: Torso Link

```xml
<link name="torso">
  <inertial>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Center of mass -->
    <mass value="10.0"/>  <!-- 10 kg -->

    <!-- Box: 0.3 x 0.2 x 0.6 (width, depth, height) -->
    <inertia
      ixx="0.35"  <!-- (1/12)*10*(0.2²+0.6²) -->
      ixy="0.0"
      ixz="0.0"
      iyy="0.4"   <!-- (1/12)*10*(0.3²+0.6²) -->
      iyz="0.0"
      izz="0.13"  <!-- (1/12)*10*(0.3²+0.2²) -->
    />
  </inertial>
</link>
```

### Tools for Complex Meshes

For complex shapes (3D models):

**MeshLab**:
1. Import mesh (STL, OBJ)
2. Filters → Quality Measure → Compute Geometric Measures
3. Outputs mass, center of mass, inertia tensor

**SolidWorks/Fusion 360**:
- CAD software automatically calculates inertial properties
- Export URDF with correct inertia

**meshlab-inertia** (Python script):
```bash
pip install trimesh
python compute_inertia.py model.stl --mass 2.0
```

## Simulating Balance and Walking

### Center of Mass (CoM)

**Key for humanoids**: CoM must stay over support polygon (feet).

**Visualize CoM** in Gazebo:
- View → Center of Mass
- Green sphere shows CoM location

### Zero Moment Point (ZMP)

**ZMP**: Point on ground where net moment is zero.

**Stable walking**: ZMP stays inside support polygon.

Gazebo doesn't calculate ZMP directly, but you can:
1. Get link states via `/gazebo/link_states`
2. Calculate CoM from link masses and positions
3. Project CoM onto ground plane

### Ground Contact Forces

**Check if feet are on ground**:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState

class FootContactNode(Node):
    def __init__(self):
        super().__init__('foot_contact')

        self.contact_sub = self.create_subscription(
            ContactsState,
            '/foot_contact',
            self.contact_callback,
            10
        )

    def contact_callback(self, msg):
        if len(msg.states) > 0:
            self.get_logger().info('Foot in contact with ground')
            # Access force: msg.states[0].total_wrench.force
        else:
            self.get_logger().warn('Foot in air!')
```

Add contact sensor to foot:
```xml
<gazebo reference="right_foot">
  <sensor name="foot_contact_sensor" type="contact">
    <contact>
      <collision>right_foot_collision</collision>
    </contact>
    <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/foot</namespace>
      </ros>
      <frame_name>right_foot</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Advanced: Joint Controllers

### Position Control

**PID controller** to reach target joint position:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/</robotNamespace>
  </plugin>
</gazebo>
```

**Controller config** (`controllers.yaml`):
```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

position_controller:
  type: effort_controllers/JointGroupPositionController
  joints:
    - right_hip_pitch
    - right_knee_pitch
    - right_ankle_pitch
    - left_hip_pitch
    - left_knee_pitch
    - left_ankle_pitch
```

### Effort (Torque) Control

**Direct torque control** (for advanced control algorithms):

Publish to `/joint_name/command`:
```python
from std_msgs.msg import Float64

torque_pub = self.create_publisher(Float64, '/right_hip_pitch/command', 10)
torque_msg = Float64()
torque_msg.data = 50.0  # 50 N·m
torque_pub.publish(torque_msg)
```

## Debugging Physics Issues

### Robot Falls Through Ground

**Cause**: Collision geometry missing or wrong.

**Fix**:
1. Ensure `<collision>` elements exist
2. Check `check_urdf` output
3. Verify ground plane in world file

### Robot Vibrates/Explodes

**Cause**: Physics instability (timestep too large, inertia wrong).

**Fix**:
1. Reduce `max_step_size` to 0.0005 or 0.001
2. Check inertial properties (use realistic values)
3. Increase contact stiffness (`kp`)

### Joints Don't Move

**Cause**: Joint limits, missing controller, or high friction.

**Fix**:
1. Check joint limits in URDF
2. Verify controller is loaded
3. Reduce `damping` and `friction` in `<dynamics>`

### Feet Slip

**Cause**: Low friction.

**Fix**:
1. Increase `mu` to 1.0-2.0 in foot collision
2. Increase contact stiffness (`kp`)

## Noise Models

Real sensors are noisy. Add noise for realism:

```xml
<sensor type="ray" name="lidar">
  <ray>
    <!-- ... -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm noise -->
    </noise>
  </ray>
</sensor>
```

**Types**:
- `gaussian`: Normal distribution
- `uniform`: Random within range

## Performance: Real-Time vs. Faster

**Real-time factor**:
- `RTF = 1.0`: 1 sim second = 1 real second
- `RTF < 1.0`: Simulation slower than real-time (complex physics)
- `RTF > 1.0`: Simulation faster (simple scenarios)

**Check RTF**:
```bash
# In Gazebo GUI, bottom panel shows real-time factor
# Or monitor /clock topic rate
ros2 topic hz /clock
```

**Optimize**:
1. Simplify collision geometry
2. Increase `max_step_size` (carefully)
3. Reduce sensor update rates
4. Run headless (`gzserver` only, no GUI)

## Next Steps

You've mastered physics simulation in Gazebo. Next, explore Unity for high-fidelity rendering and human-robot interaction.

👉 **[Next: Unity Rendering →](unity-rendering)**

---

:::tip Physics Tuning Tips

1. **Start with default physics**: Don't over-tune initially
2. **Measure real robot**: Match inertia, friction to actual hardware
3. **Iterate**: Small changes, test, repeat
4. **Use visualization**: View contacts, forces, CoM in Gazebo
5. **Validate**: Compare simulation to real robot behavior (sim-to-real)

:::

## Lab Exercise

**Tune physics for stable humanoid standing**:

1. **Baseline**:
   - Spawn humanoid, observe if it falls immediately
   - Record time until fall

2. **Tune inertia**:
   - Calculate correct inertia for all links
   - Re-run, check stability improvement

3. **Tune friction**:
   - Set foot friction to 1.5
   - Verify no slipping when applying force

4. **Tune joints**:
   - Add damping (0.7) to leg joints
   - Test if robot stands more stably

5. **Measure**:
   - Time to fall should increase significantly
   - Robot should stand for 10+ seconds without active control

**Bonus**: Implement simple balance controller to keep CoM over feet.
