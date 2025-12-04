---
sidebar_position: 3
title: VSLAM and Navigation
description: Visual SLAM for localization and autonomous navigation
keywords: [VSLAM, visual SLAM, localization, mapping, navigation, cuVSLAM]
---

# VSLAM and Navigation

## Visual SLAM Overview

**VSLAM (Visual Simultaneous Localization and Mapping)** enables robots to:
- **Build a map** of unknown environment
- **Localize** itself within that map
- **Navigate** autonomously using the map

**Why Visual SLAM?**
- Uses cameras (cheaper than LIDAR)
- Rich visual information (landmarks, textures)
- Works indoors and outdoors
- Scales to large environments

## The SLAM Problem

**Goal**: Estimate robot pose (x, y, θ) AND map simultaneously.

**Challenge**: Chicken-and-egg problem:
- Need map to localize
- Need pose to build map

**Solution**: Solve jointly using Bayesian filtering.

### SLAM Components

```
Camera Images → Feature Extraction → Feature Matching → Pose Estimation
                                                              ↓
                                                        Loop Closure
                                                              ↓
                                                     Bundle Adjustment
                                                              ↓
                                                     Optimized Map + Pose
```

## cuVSLAM: NVIDIA's Visual SLAM

**cuVSLAM** (CUDA Visual SLAM) is NVIDIA's GPU-accelerated SLAM solution.

### Features

- **Real-time**: 60+ FPS on Jetson Orin
- **Multiple sensors**: Stereo, RGB-D, mono+IMU, fisheye
- **Loop closure**: Detects revisited locations
- **Global optimization**: Bundle adjustment on GPU
- **Map persistence**: Save/load maps for localization

### System Architecture

```
┌────────────────────────────────────────┐
│  Sensors (Camera + IMU)                │
└───────────────┬────────────────────────┘
                │
┌───────────────▼────────────────────────┐
│  Visual Odometry (VO)                  │
│  - Feature extraction (GPU)            │
│  - Feature tracking (GPU)              │
│  - Pose estimation                     │
└───────────────┬────────────────────────┘
                │
┌───────────────▼────────────────────────┐
│  Mapping                               │
│  - Keyframe selection                  │
│  - 3D point triangulation              │
│  - Local map optimization              │
└───────────────┬────────────────────────┘
                │
┌───────────────▼────────────────────────┐
│  Loop Closure                          │
│  - Place recognition (GPU)             │
│  - Loop constraint generation          │
└───────────────┬────────────────────────┘
                │
┌───────────────▼────────────────────────┐
│  Global Optimization                   │
│  - Pose graph optimization             │
│  - Bundle adjustment (GPU)             │
└────────────────────────────────────────┘
```

## Running cuVSLAM

### With RealSense D435i

**Launch file**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # RealSense camera with IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py'
            ),
            launch_arguments={
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '1',  # Linear interpolation
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_depth': 'false',  # Not needed for stereo SLAM
            }.items()
        ),

        # cuVSLAM
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'enable_slam_visualization': True,
                'enable_landmarks_view': True,
                'enable_observations_view': True,
                'num_cameras': 2,  # Stereo
                'min_num_images': 2,
                'enable_image_denoising': True,
                'rectified_images': True,
                'enable_imu_fusion': True,
            }],
            remappings=[
                ('/stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                ('/stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                ('/stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                ('/stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
                ('/visual_slam/imu', '/camera/imu'),
            ],
            output='screen'
        ),
    ])
```

**Run**:
```bash
ros2 launch my_robot vslam.launch.py
```

### Published Topics

```bash
/visual_slam/tracking/odometry           # Odometry (nav_msgs/Odometry)
/visual_slam/tracking/vo_pose            # Visual odometry pose
/visual_slam/tracking/slam_path          # Full trajectory (Path)
/visual_slam/tracking/vo_pose_covariance # Pose uncertainty
/visual_slam/vis/observations_cloud      # Observed features (PointCloud2)
/visual_slam/vis/landmarks_cloud         # Map landmarks (PointCloud2)
/visual_slam/vis/loop_closure_cloud      # Loop closure matches
/tf                                      # Transforms (base_link → odom)
```

## Visualizing SLAM in RViz

```bash
# Launch RViz
ros2 run rviz2 rviz2

# Add displays:
# 1. Odometry: /visual_slam/tracking/odometry (show robot pose)
# 2. Path: /visual_slam/tracking/slam_path (show trajectory)
# 3. PointCloud2: /visual_slam/vis/landmarks_cloud (show map)
# 4. PointCloud2: /visual_slam/vis/observations_cloud (show features)
# 5. TF: Show all transforms
# 6. Image: /camera/infra1/image_rect_raw (show camera view)

# Set fixed frame: odom
```

## Mapping Workflow

### Phase 1: Exploration (Mapping)

**Goal**: Build a map by exploring environment.

```bash
# 1. Launch VSLAM in mapping mode (default)
ros2 launch my_robot vslam.launch.py

# 2. Teleoperate robot to explore
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. Move slowly through environment
#    - Cover all areas
#    - Look at textured surfaces
#    - Return to starting point (for loop closure)

# 4. Monitor map quality
ros2 topic echo /visual_slam/tracking/vo_pose_covariance
# Low covariance = high confidence

# 5. Save map
ros2 service call /visual_slam/save_map \
    isaac_ros_visual_slam_interfaces/srv/FilePath \
    "{file_path: '/home/user/maps/my_map.db'}"
```

**Best practices for mapping**:
- Move slowly (< 0.5 m/s)
- Avoid fast rotations
- Look at textured surfaces (not blank walls)
- Revisit starting location (enables loop closure)
- Map in good lighting

### Phase 2: Localization (Using Map)

**Goal**: Localize in pre-built map.

**Modify launch file**:

```python
Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_visual_slam',
    parameters=[{
        'enable_localization_n_mapping': False,  # Localization only
        'load_map_from_file': True,
        'map_file_path': '/home/user/maps/my_map.db',
    }],
    # ... rest of config
)
```

**Run**:
```bash
ros2 launch my_robot vslam_localization.launch.py

# Robot will localize in existing map
# No new landmarks added, only tracking
```

## Integrating VSLAM with Navigation

### Nav2 Integration

**Nav2** (Navigation 2) uses VSLAM odometry for navigation.

**Architecture**:
```
cuVSLAM (/visual_slam/tracking/odometry) → Nav2 (/odom)
                                               ↓
                                          Path Planner
                                               ↓
                                        Controller (/cmd_vel)
                                               ↓
                                          Robot moves
```

### Complete Navigation Stack

**Launch file**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # 1. Camera + VSLAM (as before)
        IncludeLaunchDescription(...),

        # 2. Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # 3. Nvblox (3D mapping for obstacle avoidance)
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            parameters=[{
                'global_frame': 'odom',
            }],
            remappings=[
                ('/depth/image', '/camera/depth/image_rect_raw'),
                ('/color/image', '/camera/color/image_raw'),
            ]
        ),

        # 4. Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': '/path/to/nav2_params.yaml',
            }.items()
        ),
    ])
```

### Sending Navigation Goals

**Command-line**:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Python**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavigationGoalNode(Node):
    def __init__(self):
        super().__init__('nav_goal')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Send goal after 2 seconds
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 5.0
        goal.pose.position.y = 2.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info('Navigation goal sent')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**RViz (Interactive)**:
```
1. Open RViz
2. Add Nav2 panel: Panels → Add New Panel → Nav2
3. Click "2D Goal Pose" button
4. Click on map to set goal
5. Robot navigates autonomously
```

## Troubleshooting VSLAM

### Problem: Tracking Lost

**Symptoms**: `tracking_vo_state` = 0 (lost)

**Causes**:
- Moving too fast
- Insufficient features (blank wall, darkness)
- Camera obstructed
- Sudden lighting change

**Solutions**:
- Move slower
- Point camera at textured surfaces
- Improve lighting
- Enable image denoising: `enable_image_denoising: true`

### Problem: Drift (Map Not Closing)

**Symptoms**: Robot returns to start, but map shows offset.

**Causes**:
- No loop closure
- Insufficient overlap with starting location

**Solutions**:
- Ensure robot revisits starting area (same viewpoint)
- Increase `num_images_for_loop_closure` parameter
- Check loop closure detections: `/visual_slam/vis/loop_closure_cloud`

### Problem: Poor Performance (Low FPS)

**Symptoms**: FPS < 30, high latency

**Causes**:
- CPU/GPU overloaded
- High-resolution images
- Too many features tracked

**Solutions**:
- Reduce image resolution: 640x480 instead of 1920x1080
- Reduce feature count: `max_num_features: 200` (default: 500)
- Run headless (no visualization): `enable_slam_visualization: false`
- Close other applications

## Advanced: Multi-Camera VSLAM

**Using 3+ cameras** for 360° coverage:

```python
Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_visual_slam',
    parameters=[{
        'num_cameras': 4,  # 4 cameras
    }],
    remappings=[
        ('/camera_0/image', '/front_camera/image_raw'),
        ('/camera_1/image', '/left_camera/image_raw'),
        ('/camera_2/image', '/right_camera/image_raw'),
        ('/camera_3/image', '/back_camera/image_raw'),
        # ... camera_info for each
    ]
)
```

**Benefits**:
- No blind spots
- More robust tracking
- Better loop closure

## Performance Tuning

### Parameters for Speed

```yaml
visual_slam:
  ros__parameters:
    # Reduce features for speed
    max_num_features: 150  # Default: 500

    # Disable expensive operations
    enable_slam_visualization: false
    enable_landmarks_view: false
    enable_observations_view: false

    # Lower image resolution (at camera driver level)
    # RealSense: width: 640, height: 480
```

**Result**: 60+ FPS → 90+ FPS

### Parameters for Accuracy

```yaml
visual_slam:
  ros__parameters:
    # Increase features for accuracy
    max_num_features: 800

    # Enable denoising
    enable_image_denoising: true

    # Enable loop closure
    enable_loop_closure: true
    num_images_for_loop_closure: 50

    # Higher frequency IMU
    imu_jitter_time_s: 0.001
```

**Result**: Better accuracy, lower drift, but slower (40-50 FPS).

## Comparison: cuVSLAM vs. ORB-SLAM3

| Feature | cuVSLAM | ORB-SLAM3 |
|---------|---------|-----------|
| **Platform** | GPU (Jetson, NVIDIA GPU) | CPU |
| **Speed** | 60 FPS (Jetson Orin) | 10-20 FPS |
| **Accuracy** | Excellent | Excellent |
| **Map Persistence** | Yes (save/load) | Yes |
| **IMU Fusion** | Yes | Yes |
| **License** | Apache 2.0 | GPLv3 |
| **ROS 2 Integration** | Native (Isaac ROS) | Community packages |

**Recommendation**: Use cuVSLAM if you have NVIDIA hardware (Jetson or RTX GPU).

## Next Steps

You've mastered VSLAM and navigation. Next, apply navigation to bipedal humanoid robots.

👉 **[Next: Nav2 for Bipedal Robots →](nav2-bipedal)**

---

:::tip VSLAM Tips

1. **Good lighting**: VSLAM needs sufficient light and texture
2. **Slow movements**: Moving < 0.5 m/s prevents tracking loss
3. **Loop closure**: Revisit starting area to close the map
4. **Save maps**: Reuse maps for faster startup (localization mode)
5. **Monitor covariance**: Low covariance = high confidence

:::

## Lab Exercise

**Build a complete VSLAM-based navigation system**:

1. **Setup**:
   - RealSense D435i on Jetson Orin Nano
   - Launch cuVSLAM in mapping mode

2. **Create map**:
   - Teleoperate robot through environment
   - Cover all rooms/areas
   - Return to start (trigger loop closure)
   - Save map to file

3. **Test localization**:
   - Restart cuVSLAM in localization mode
   - Place robot at random location
   - Verify it localizes correctly

4. **Add Nav2**:
   - Launch Nvblox (3D mapping)
   - Launch Nav2 navigation stack
   - Send navigation goal via RViz
   - Robot navigates autonomously

5. **Measure performance**:
   - Record FPS: `ros2 topic hz /visual_slam/tracking/odometry`
   - Measure drift: Return to start, compare pose
   - Test in different lighting conditions

**Bonus**: Implement waypoint navigation (visit multiple goals sequentially).
