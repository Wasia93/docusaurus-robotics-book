---
sidebar_position: 4
title: Nav2 for Bipedal Humanoids
description: Path planning and navigation for humanoid robots
keywords: [Nav2, navigation, path planning, bipedal, humanoid, obstacle avoidance]
---

# Nav2 for Bipedal Humanoids

## Introduction to Nav2

**Nav2 (Navigation 2)** is ROS 2's navigation framework:
- Path planning (global and local)
- Obstacle avoidance
- Recovery behaviors
- Waypoint following
- Dynamic reconfiguration

### Why Nav2 for Humanoids?

**Challenge**: Humanoids are not wheeled robots.
- Different kinematics (legs, not wheels)
- Balance constraints (can't rotate in place)
- Footstep planning (discrete foot placements)
- Higher center of mass (stability concerns)

**Solution**: Adapt Nav2 for bipedal constraints.

## Nav2 Architecture

```
┌─────────────────────────────────────────────────┐
│  Behavior Tree Navigator                        │
│  - Task coordination                            │
│  - Recovery behaviors                           │
└────────────┬────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────┐
│  Planner Server                                 │
│  - Global path planning (A*, Dijkstra, etc.)   │
└────────────┬────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────┐
│  Controller Server                              │
│  - Local trajectory generation                  │
│  - DWB, TEB, MPPI controllers                   │
└────────────┬────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────┐
│  Costmap 2D                                     │
│  - Obstacle representation                      │
│  - Inflation layer (safety buffer)             │
└─────────────────────────────────────────────────┘
```

## Nav2 for Wheeled vs. Bipedal

### Wheeled Robot (Differential Drive)

**Kinematic model**: Simple
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω

Commands: (v, ω) - linear and angular velocity
```

**Nav2 default**: Works out-of-the-box

### Bipedal Humanoid

**Kinematic model**: Complex
- Footstep planning (discrete positions)
- Balance constraints (ZMP, CoM)
- Joint limits (hip, knee, ankle ranges)
- Stability margins

**Nav2 adaptation**: Need custom controller

## Simplified Approach: Holonomic Base

**For this course**: Treat humanoid as holonomic base.

**Assumption**:
- Humanoid can move in any direction (forward, backward, strafe)
- Low-level controller handles balance
- Nav2 sends velocity commands: (vx, vy, vtheta)

**Justification**:
- Focus on high-level navigation (path planning)
- Low-level walking controller is separate (Module 4)
- Practical for simulation (Gazebo, Isaac Sim)

### Holonomic Configuration

**Nav2 params**:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

      # Velocity limits (humanoid-specific)
      max_vel_x: 0.5    # Forward: 0.5 m/s
      min_vel_x: -0.3   # Backward: 0.3 m/s (slower)
      max_vel_y: 0.3    # Strafe: 0.3 m/s
      max_vel_theta: 0.5  # Rotation: 0.5 rad/s

      # Acceleration limits (gentler for stability)
      acc_lim_x: 0.5
      acc_lim_y: 0.5
      acc_lim_theta: 1.0

      # DWB-specific
      min_speed_xy: 0.1
      max_speed_xy: 0.5
      min_speed_theta: 0.2
```

## Installing Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Basic Navigation Setup

### 1. Robot Configuration

**robot_params.yaml**:

```yaml
# Robot footprint (humanoid standing)
robot_base_frame: "base_link"
robot_radius: 0.25  # Circle approximation (50cm diameter)
# OR
footprint: "[[0.2, 0.15], [0.2, -0.15], [-0.2, -0.15], [-0.2, 0.15]]"  # Rectangle

# Transform tolerance
transform_tolerance: 0.2
```

### 2. Costmap Configuration

**Costmap**: 2D grid representing obstacle costs.

**costmap_params.yaml**:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: odom
      robot_base_frame: base_link

      resolution: 0.05  # 5cm per cell
      width: 50
      height: 50

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: lidar depth_camera

        lidar:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"

        depth_camera:
          topic: /camera/depth/points
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Safety margin (robot radius + buffer)

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true

      width: 5
      height: 5
      resolution: 0.05

      plugins: ["obstacle_layer", "inflation_layer"]
      # (Same obstacle and inflation config as global)
```

### 3. Planner Configuration

**planner_params.yaml**:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true  # A* (faster than Dijkstra)
      allow_unknown: true
```

### 4. Controller Configuration

**controller_params.yaml** (already shown above)

### 5. Launch Navigation

**nav2_launch.py**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_humanoid_nav')

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Localization (VSLAM from Module 3)
        IncludeLaunchDescription(...),

        # Mapping (Nvblox for costmap)
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            # ...
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
            }.items()
        ),

        # RViz with Nav2 panel
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'nav2.rviz')]
        ),
    ])
```

**Run**:
```bash
ros2 launch my_humanoid_nav nav2_launch.py
```

## Sending Navigation Goals

### Method 1: RViz (Interactive)

```
1. Open RViz
2. Panels → Add New Panel → Nav2
3. Click "2D Goal Pose" button
4. Click on map where you want robot to go
5. Drag to set orientation
6. Robot navigates autonomously
```

### Method 2: Command Line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 5.0, y: 2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### Method 3: Python (Programmatic)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Convert yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance_remaining:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')

def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()

    # Send goal: (5, 2) with 0 degree heading
    nav_client.send_goal(5.0, 2.0, 0.0)

    rclpy.spin(nav_client)
```

## Waypoint Navigation

**Visit multiple locations sequentially**:

```python
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def send_waypoints(self, waypoints):
        goal_msg = FollowWaypoints.Goal()

        for (x, y, yaw) in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y

            from tf_transformations import quaternion_from_euler
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            goal_msg.poses.append(pose)

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent {len(waypoints)} waypoints')

# Usage
navigator = WaypointNavigator()
waypoints = [
    (2.0, 0.0, 0.0),    # First waypoint
    (5.0, 2.0, 1.57),   # Second waypoint (facing 90 degrees)
    (3.0, -1.0, 3.14),  # Third waypoint (facing 180 degrees)
]
navigator.send_waypoints(waypoints)
```

## Humanoid-Specific Challenges

### Challenge 1: Narrow Footprint

**Problem**: Humanoid feet are small, but body is tall.

**Solution**: Define footprint conservatively:
```yaml
# Include shoulders/arms in footprint (top-down view)
footprint: "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"
```

### Challenge 2: Cannot Rotate in Place

**Problem**: Bipedal robots can't spin like wheeled robots.

**Solution**: Configure wider turn radius:
```yaml
# Controller parameters
min_turning_radius: 0.5  # Meters
prefer_forward: true
```

### Challenge 3: Slow Speed

**Problem**: Walking is slower than driving.

**Solution**: Adjust velocity limits:
```yaml
max_vel_x: 0.5  # 0.5 m/s walking speed
max_vel_theta: 0.3  # Slow rotation
```

### Challenge 4: Balance While Turning

**Problem**: Sharp turns can cause loss of balance.

**Solution**: Smooth trajectory with low acceleration:
```yaml
acc_lim_x: 0.3  # Gentle acceleration
acc_lim_theta: 0.5
sim_time: 2.0  # Longer prediction horizon
```

## Advanced: Footstep Planning

**True bipedal navigation** requires footstep planner.

**Concept**:
1. **Global planner**: Generates coarse path (like wheeled robot)
2. **Footstep planner**: Converts path to discrete foot placements
3. **Walking controller**: Executes footsteps while maintaining balance

**Example**: Humanoid Footstep Planner

```python
class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Subscribe to Nav2 path
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )

        # Publish footsteps
        self.footstep_pub = self.create_publisher(
            FootstepArray, '/footsteps', 10
        )

    def path_callback(self, path_msg):
        # Convert path to footsteps
        footsteps = self.generate_footsteps(path_msg)

        # Publish footstep plan
        self.footstep_pub.publish(footsteps)

    def generate_footsteps(self, path):
        footsteps = []
        step_length = 0.3  # 30cm steps
        step_width = 0.2   # 20cm apart (left-right)

        left_foot = True
        for i in range(0, len(path.poses), 2):
            pose = path.poses[i].pose

            if left_foot:
                foot_offset = [0, step_width/2]  # Left foot
            else:
                foot_offset = [0, -step_width/2]  # Right foot

            # Create footstep (simplified)
            footstep = Footstep()
            footstep.position = pose.position
            footstep.position.y += foot_offset[1]
            footstep.is_left_foot = left_foot

            footsteps.append(footstep)
            left_foot = not left_foot

        return footsteps
```

**Note**: Full footstep planning is complex (covered briefly in Module 4).

## Behavior Trees

**Nav2 uses behavior trees** for task coordination.

**Default behaviors**:
- `ComputePathToPose`: Plan global path
- `FollowPath`: Execute path with local controller
- `ClearCostmapExceptRegion`: Recovery behavior
- `Spin`: Rotate in place (recovery)
- `Wait`: Pause (recovery)

**Custom behavior tree** (advanced):

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <ComputePathToPose goal="{goal}" path="{path}"/>
      <FollowPath path="{path}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

## Monitoring Navigation

### Check Status

```bash
# Action status
ros2 action list

# Navigation state
ros2 topic echo /navigation_state

# Costmap visualization
ros2 run rviz2 rviz2
# Add Map display → /global_costmap/costmap
```

### Debugging

**Navigation fails?**

1. **Check costmap**: Is path obstructed?
   ```bash
   ros2 topic echo /global_costmap/costmap --no-arr
   ```

2. **Check transforms**: Is TF tree valid?
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

3. **Check velocity limits**: Are limits too conservative?
   ```yaml
   max_vel_x: 0.5  # Try increasing
   ```

4. **Check localization**: Is robot localized?
   ```bash
   ros2 topic echo /visual_slam/tracking/vo_pose_covariance
   # Low covariance = good localization
   ```

## Next Steps

You've learned navigation for humanoid robots. Next, explore reinforcement learning for autonomous behavior.

👉 **[Next: Reinforcement Learning →](reinforcement-learning)**

---

:::tip Nav2 for Humanoids Tips

1. **Conservative footprint**: Include arms/shoulders in 2D footprint
2. **Slow speeds**: Walking < 0.5 m/s for stability
3. **Gentle acceleration**: Avoid sudden changes (balance)
4. **Wide turns**: min_turning_radius to prevent sharp turns
5. **Test in sim first**: Perfect navigation before deploying to hardware

:::

## Lab Exercise

**Build humanoid navigation system**:

1. **Setup Nav2**:
   - Configure costmap with robot footprint
   - Set velocity limits for walking
   - Launch Nav2 with custom params

2. **Test navigation**:
   - Send single goal via RViz
   - Verify robot reaches goal
   - Check costmap shows obstacles

3. **Waypoint navigation**:
   - Define 4-5 waypoints in a loop
   - Send waypoints sequentially
   - Robot visits all locations

4. **Obstacle avoidance**:
   - Place dynamic obstacle in path
   - Robot should replan and avoid

5. **Measure performance**:
   - Time to reach goal
   - Path length vs. optimal path
   - Number of replanning events

**Bonus**: Implement custom recovery behavior (e.g., sidestep if stuck).
