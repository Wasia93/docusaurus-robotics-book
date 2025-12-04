---
sidebar_position: 4
title: Sensor Simulation
description: Simulating LIDAR, cameras, IMUs, and depth sensors
keywords: [sensor simulation, LIDAR, camera, IMU, depth camera, Gazebo plugins]
---

# Sensor Simulation

## Why Simulate Sensors?

**Accurate sensor simulation** is critical for:
- **Algorithm development**: Test perception before hardware
- **Data generation**: Create labeled datasets for ML
- **Edge case testing**: Simulate rare scenarios (fog, darkness, sensor failures)
- **Cost savings**: No need to buy expensive sensors during development

### Simulation vs. Real Sensors

| Aspect | Real Sensor | Simulated Sensor |
|--------|-------------|------------------|
| **Cost** | $100-5,000 | Free |
| **Setup Time** | Hours (mounting, calibration) | Minutes (add plugin) |
| **Noise** | Real-world noise patterns | Approximate (Gaussian) |
| **Edge Cases** | Hard to reproduce | Easy (fog, occlusion, etc.) |
| **Accuracy** | Ground truth | Approximation |
| **Data Volume** | Limited by recording | Unlimited |

**Best approach**: Develop with simulation, validate with real sensors.

## Camera Simulation

### RGB Camera in Gazebo

**Add to URDF**:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <!-- Field of view -->
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->

      <!-- Image settings -->
      <image>
        <width>1920</width>
        <height>1080</height>
        <format>R8G8B8</format>  <!-- RGB -->
      </image>

      <!-- Clipping planes -->
      <clip>
        <near>0.02</near>  <!-- 2cm minimum -->
        <far>300</far>     <!-- 300m maximum -->
      </clip>

      <!-- Lens distortion (optional, for realism) -->
      <distortion>
        <k1>0.0</k1>  <!-- Radial distortion -->
        <k2>0.0</k2>
        <k3>0.0</k3>
        <p1>0.0</p1>  <!-- Tangential distortion -->
        <p2>0.0</p2>
        <center>0.5 0.5</center>
      </distortion>

      <!-- Noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- ~2/255 pixel noise -->
      </noise>
    </camera>

    <!-- ROS 2 plugin -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
      <camera_name>head_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topics published**:
- `/camera/image_raw`: Raw RGB image
- `/camera/camera_info`: Camera calibration (K matrix, distortion)

### Depth Camera (RGB-D)

**Simulates Intel RealSense, Kinect**:

```xml
<gazebo reference="depth_camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30.0</update_rate>
    <camera name="depth_camera">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
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

    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/depth_camera</namespace>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>

      <!-- Publish both RGB and depth -->
      <min_depth>0.05</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topics**:
- `/depth_camera/image_raw`: RGB image
- `/depth_camera/depth/image_raw`: Depth image (16-bit, millimeters)
- `/depth_camera/points`: Point cloud (PointCloud2)

### Stereo Camera

**Two cameras for depth estimation**:

```xml
<!-- Left camera -->
<gazebo reference="left_camera_link">
  <sensor type="camera" name="left_camera">
    <!-- Same as RGB camera config -->
    <plugin name="left_camera_controller" filename="libgazebo_ros_camera.so">
      <ros><namespace>/stereo/left</namespace></ros>
      <camera_name>left</camera_name>
      <frame_name>left_camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>

<!-- Right camera (offset by baseline) -->
<gazebo reference="right_camera_link">
  <sensor type="camera" name="right_camera">
    <plugin name="right_camera_controller" filename="libgazebo_ros_camera.so">
      <ros><namespace>/stereo/right</namespace></ros>
      <camera_name>right</camera_name>
      <frame_name>right_camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>  <!-- 7cm baseline -->
    </plugin>
  </sensor>
</gazebo>
```

**Post-processing**: Use `stereo_image_proc` to compute disparity and depth.

## LIDAR Simulation

### 2D LIDAR (Laser Scanner)

**Simulates RPLIDAR, SICK, Hokuyo**:

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>  <!-- Show rays in Gazebo -->
    <update_rate>10</update_rate>

    <ray>
      <!-- Scan parameters -->
      <scan>
        <horizontal>
          <samples>720</samples>  <!-- 720 points -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
        </horizontal>
      </scan>

      <!-- Range parameters -->
      <range>
        <min>0.10</min>  <!-- 10cm minimum -->
        <max>30.0</max>  <!-- 30m maximum -->
        <resolution>0.01</resolution>  <!-- 1cm resolution -->
      </range>

      <!-- Noise model -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>  <!-- 1cm noise -->
      </noise>
    </ray>

    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topic**: `/lidar/scan` (LaserScan message)

### 3D LIDAR (Point Cloud)

**Simulates Velodyne, Ouster**:

```xml
<gazebo reference="lidar_3d_link">
  <sensor type="ray" name="lidar_3d">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <!-- Horizontal scan -->
        <horizontal>
          <samples>1800</samples>  <!-- 0.2 degree resolution -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>

        <!-- Vertical scan (16 or 32 channels) -->
        <vertical>
          <samples>16</samples>  <!-- 16-channel LIDAR -->
          <resolution>1</resolution>
          <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
          <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
        </vertical>
      </scan>

      <range>
        <min>0.5</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.02</stddev>
      </noise>
    </ray>

    <plugin name="gazebo_ros_3d_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros><namespace>/lidar_3d</namespace></ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>lidar_3d_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topic**: `/lidar_3d/points` (PointCloud2 message)

## IMU Simulation

**Inertial Measurement Unit** (accelerometer + gyroscope):

```xml
<gazebo reference="imu_link">
  <gravity>true</gravity>
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz typical -->

    <imu>
      <!-- Angular velocity (gyroscope) -->
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>  <!-- rad/s -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>

      <!-- Linear acceleration (accelerometer) -->
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- m/s² -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topic**: `/imu/data` (Imu message)

**Fields**:
- `orientation`: Quaternion (roll, pitch, yaw)
- `angular_velocity`: rad/s (x, y, z)
- `linear_acceleration`: m/s² (x, y, z)

## Contact Sensors (Force/Torque)

**Detect collisions and measure forces**:

```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact_sensor" type="contact">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>

    <contact>
      <collision>foot_collision</collision>  <!-- Must match collision name -->
    </contact>

    <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/foot</namespace>
        <remapping>bumper_states:=contact</remapping>
      </ros>
      <frame_name>foot_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**ROS 2 topic**: `/foot/contact` (ContactsState message)

**Use case**: Detect when foot touches ground for bipedal walking.

## GPS Simulation

**For outdoor navigation**:

```xml
<gazebo>
  <plugin name="gazebo_ros_gps" filename="libgazebo_ros_gps_sensor.so">
    <ros>
      <namespace>/gps</namespace>
      <remapping>~/out:=fix</remapping>
    </ros>
    <frame_name>base_link</frame_name>

    <!-- GPS accuracy -->
    <position_sensing>
      <horizontal>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2.0</stddev>  <!-- 2m horizontal error -->
        </noise>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>4.0</stddev>  <!-- 4m vertical error -->
        </noise>
      </vertical>
    </position_sensing>

    <!-- Update rate -->
    <update_rate>1.0</update_rate>  <!-- 1 Hz typical -->
  </plugin>
</gazebo>
```

**ROS 2 topic**: `/gps/fix` (NavSatFix message)

## Noise Models

### Types of Noise

**Gaussian (Normal Distribution)**:
```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- Standard deviation -->
</noise>
```

**Uniform (Random within range)**:
```xml
<noise>
  <type>uniform</type>
  <min>-0.05</min>
  <max>0.05</max>
</noise>
```

### Realistic Noise Parameters

Based on real sensor specs:

| Sensor | Parameter | Noise (stddev) |
|--------|-----------|----------------|
| **Camera** | Pixel intensity | 0.007 (2/255) |
| **Depth Camera** | Depth (RealSense) | 0.01m (1cm) |
| **LIDAR** | Range | 0.01-0.03m |
| **IMU Gyro** | Angular velocity | 0.0002 rad/s |
| **IMU Accel** | Linear accel | 0.017 m/s² |
| **GPS** | Horizontal position | 2.0m |
| **GPS** | Vertical position | 4.0m |

**Source**: Manufacturer datasheets (Intel, Velodyne, etc.)

## Sensor Update Rates

**Match real sensor frequencies**:

| Sensor | Typical Rate | Gazebo Update Rate |
|--------|-------------|-------------------|
| **Camera** | 30-60 Hz | 30 Hz |
| **Depth Camera** | 30-90 Hz | 30 Hz |
| **2D LIDAR** | 5-10 Hz | 10 Hz |
| **3D LIDAR** | 10-20 Hz | 10 Hz |
| **IMU** | 100-1000 Hz | 100 Hz |
| **GPS** | 1-10 Hz | 1 Hz |
| **Contact Sensor** | 100-1000 Hz | 100 Hz |

**Trade-off**: Higher rates = more data but slower simulation.

## Visualizing Sensor Data

### In Gazebo

```
View → Contacts (visualize contact points)
View → Transparent (see through objects)
```

For ray sensors (LIDAR):
```xml
<visualize>true</visualize>  <!-- Show laser rays -->
```

### In RViz

```bash
ros2 run rviz2 rviz2

# Add displays:
# - Camera: Subscribe to /camera/image_raw
# - LaserScan: Subscribe to /lidar/scan
# - PointCloud2: Subscribe to /lidar_3d/points
# - Imu: Subscribe to /imu/data (shows orientation)
```

## Testing Sensors

### Test 1: Camera

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch my_robot gazebo.launch.py

# Terminal 2: View camera stream
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Terminal 3: Check topic info
ros2 topic info /camera/image_raw
ros2 topic hz /camera/image_raw  # Should be ~30 Hz
```

### Test 2: LIDAR

```bash
# Terminal 1: Launch Gazebo
ros2 launch my_robot gazebo.launch.py

# Terminal 2: View LIDAR in RViz
ros2 run rviz2 rviz2
# Add LaserScan display → Topic: /lidar/scan

# Terminal 3: Echo data
ros2 topic echo /lidar/scan --no-arr  # Suppress large arrays
```

### Test 3: IMU

```bash
# Terminal 1: Launch Gazebo
ros2 launch my_robot gazebo.launch.py

# Terminal 2: Echo IMU data
ros2 topic echo /imu/data

# Tilt robot in Gazebo (use mouse to rotate)
# Should see orientation change in IMU data
```

## Multi-Sensor Fusion Example

**Combine camera + LIDAR for better perception**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import CvBridge

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.scan_callback, 10
        )

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.fuse_data()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.fuse_data()

    def fuse_data(self):
        if self.latest_image is None or self.latest_scan is None:
            return

        # Example: Overlay LIDAR data on camera image
        # Project LIDAR points to image plane
        # (Requires camera-LIDAR extrinsic calibration)

        self.get_logger().info('Fusing camera + LIDAR data')
```

## Next Steps

You've learned sensor simulation. Next, understand robot description formats (URDF vs SDF).

👉 **[Next: URDF and SDF →](urdf-sdf)**

---

:::tip Sensor Simulation Tips

1. **Start with defaults**: Don't over-tune noise initially
2. **Match real specs**: Use manufacturer datasheets for realistic noise
3. **Test incrementally**: Add one sensor at a time
4. **Visualize**: Use RViz to debug sensor placement and data
5. **Profile performance**: Too many high-rate sensors slow simulation

:::

## Lab Exercise

**Build a multi-sensor humanoid**:

1. **Add sensors to URDF**:
   - Head: RGB camera (30 Hz, 1920x1080)
   - Head: Depth camera (30 Hz, 640x480)
   - Torso: IMU (100 Hz)
   - Torso: 2D LIDAR (10 Hz, 360°)
   - Feet: Contact sensors (100 Hz)

2. **Configure noise**:
   - Camera: stddev 0.007
   - Depth: stddev 0.01m
   - LIDAR: stddev 0.02m
   - IMU: gyro 0.0002 rad/s, accel 0.017 m/s²

3. **Test in Gazebo**:
   - Spawn robot
   - Verify all topics publishing
   - Check update rates with `ros2 topic hz`

4. **Visualize in RViz**:
   - Camera image
   - LIDAR scan
   - IMU orientation
   - Robot TF tree

**Bonus**: Create sensor fusion node that combines camera + LIDAR.
