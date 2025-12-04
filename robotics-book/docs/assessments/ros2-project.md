# Assessment: ROS 2 Package Development

## Overview

This assessment evaluates your ability to design and implement a complete ROS 2 package for robotic control. You'll create a multi-node system that demonstrates core ROS 2 concepts including nodes, topics, services, actions, and launch files.

## Learning Objectives

By completing this assessment, you will demonstrate:

- Proficiency in ROS 2 package structure and build systems
- Ability to create and configure multiple communicating nodes
- Understanding of ROS 2 communication patterns (topics, services, actions)
- Skill in writing launch files for complex systems
- Knowledge of parameter management and configuration

## Project Requirements

### Task: Robotic Arm Controller

Create a ROS 2 package that controls a simulated robotic arm with the following capabilities:

#### 1. Package Structure

```
ros2_arm_controller/
├── package.xml
├── CMakeLists.txt (C++) or setup.py (Python)
├── launch/
│   ├── arm_controller.launch.py
│   └── simulation.launch.py
├── config/
│   ├── arm_params.yaml
│   └── controllers.yaml
├── src/ (or ros2_arm_controller/ for Python)
│   ├── arm_controller_node.cpp/py
│   ├── trajectory_planner_node.cpp/py
│   ├── safety_monitor_node.cpp/py
│   └── gripper_controller_node.cpp/py
├── msg/
│   └── JointCommand.msg
├── srv/
│   └── PlanTrajectory.srv
├── action/
│   └── MoveToTarget.action
└── README.md
```

#### 2. Required Nodes

##### a) Arm Controller Node

**Responsibilities:**
- Subscribe to joint command topics
- Publish joint states
- Execute motion commands
- Enforce joint limits

**Interface:**
```python
# Publishers
/arm/joint_states (sensor_msgs/JointState)

# Subscribers
/arm/joint_commands (your_package/JointCommand)

# Parameters
- joint_limits: [min, max] for each joint
- control_rate: Hz
- max_velocity: rad/s
- max_acceleration: rad/s^2
```

##### b) Trajectory Planner Node

**Responsibilities:**
- Plan smooth trajectories between poses
- Provide trajectory planning service
- Interpolate waypoints

**Interface:**
```python
# Services
/arm/plan_trajectory (your_package/PlanTrajectory)
  Request:
    - geometry_msgs/Pose target_pose
    - float64 max_velocity
    - float64 max_acceleration
  Response:
    - trajectory_msgs/JointTrajectory trajectory
    - bool success
    - string message

# Publishers
/arm/planned_trajectory (trajectory_msgs/JointTrajectory)
```

##### c) Safety Monitor Node

**Responsibilities:**
- Monitor joint states for safety violations
- Emergency stop capability
- Workspace boundary enforcement

**Interface:**
```python
# Subscribers
/arm/joint_states (sensor_msgs/JointState)

# Publishers
/arm/safety_status (std_msgs/Bool)
/arm/safety_alerts (std_msgs/String)

# Services
/arm/emergency_stop (std_srvs/Trigger)

# Parameters
- workspace_bounds: [x_min, x_max, y_min, y_max, z_min, z_max]
- joint_velocity_limits: [max_vel] for each joint
```

##### d) Gripper Controller Node

**Responsibilities:**
- Control gripper open/close
- Monitor grasp force
- Provide action interface for pick/place

**Interface:**
```python
# Action Server
/arm/gripper/move (your_package/MoveToTarget)
  Goal:
    - string command (open/close/grasp)
    - float64 target_position
    - float64 max_effort
  Feedback:
    - float64 current_position
    - float64 current_effort
  Result:
    - bool success
    - string message

# Publishers
/arm/gripper/state (sensor_msgs/JointState)
```

#### 3. Custom Messages

##### JointCommand.msg
```
Header header
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```

##### PlanTrajectory.srv
```
# Request
geometry_msgs/Pose target_pose
float64 max_velocity
float64 max_acceleration
---
# Response
trajectory_msgs/JointTrajectory trajectory
bool success
string message
```

##### MoveToTarget.action
```
# Goal
string command
float64 target_position
float64 max_effort
---
# Result
bool success
string message
---
# Feedback
float64 current_position
float64 current_effort
float64 progress_percentage
```

#### 4. Launch Files

##### arm_controller.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'config_file',
            default_value='arm_params.yaml',
            description='Configuration file'
        ),

        # Arm controller node
        Node(
            package='ros2_arm_controller',
            executable='arm_controller_node',
            name='arm_controller',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),

        # Trajectory planner node
        Node(
            package='ros2_arm_controller',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            output='screen'
        ),

        # Safety monitor node
        Node(
            package='ros2_arm_controller',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen'
        ),

        # Gripper controller node
        Node(
            package='ros2_arm_controller',
            executable='gripper_controller_node',
            name='gripper_controller',
            output='screen'
        ),
    ])
```

#### 5. Configuration Files

##### arm_params.yaml
```yaml
arm_controller:
  ros__parameters:
    joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    joint_limits:
      joint_1: [-3.14, 3.14]
      joint_2: [-1.57, 1.57]
      joint_3: [-2.0, 2.0]
      joint_4: [-1.57, 1.57]
    control_rate: 100.0
    max_velocity: 1.0
    max_acceleration: 2.0

safety_monitor:
  ros__parameters:
    workspace_bounds:
      x: [-1.0, 1.0]
      y: [-1.0, 1.0]
      z: [0.0, 2.0]
    joint_velocity_limits: [2.0, 2.0, 3.0, 3.0]
    enable_collision_checking: true
```

## Testing Requirements

### Unit Tests

Create unit tests for each node:

```python
import unittest
from ros2_arm_controller import ArmController

class TestArmController(unittest.TestCase):
    def test_joint_limits(self):
        """Test that joint limits are enforced"""
        controller = ArmController()
        # Test joint limit enforcement
        pass

    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        pass

    def test_trajectory_execution(self):
        """Test trajectory following"""
        pass
```

### Integration Tests

```python
import rclpy
from rclpy.node import Node

class TestArmIntegration(unittest.TestCase):
    def test_full_motion(self):
        """Test complete motion from start to goal"""
        # 1. Initialize nodes
        # 2. Send target pose
        # 3. Verify trajectory planning
        # 4. Verify execution
        # 5. Check final position
        pass

    def test_safety_violation(self):
        """Test safety monitor catches violations"""
        pass
```

## Deliverables

### 1. Source Code
- Complete ROS 2 package with all required nodes
- Custom messages, services, and actions
- Launch files and configuration files

### 2. Documentation
- README with:
  - Package description
  - Installation instructions
  - Usage examples
  - Architecture diagram
  - API documentation

### 3. Demonstration
- Video or live demo showing:
  - Successful trajectory planning and execution
  - Gripper control (open/close/grasp)
  - Safety monitoring (boundary violation, emergency stop)
  - Multiple nodes communicating

### 4. Test Results
- Unit test coverage report (>80%)
- Integration test results
- Performance metrics (control frequency, latency)

## Evaluation Criteria

| Criteria | Points | Description |
|----------|--------|-------------|
| **Package Structure** | 10 | Proper ROS 2 package layout, build configuration |
| **Node Implementation** | 30 | All required nodes functioning correctly |
| **Communication** | 20 | Proper use of topics, services, actions |
| **Launch Files** | 10 | Correct launch file configuration |
| **Safety** | 10 | Safety monitoring and emergency stop |
| **Code Quality** | 10 | Clean code, documentation, comments |
| **Testing** | 10 | Unit and integration tests |

**Total: 100 points**

## Submission Guidelines

1. **Git Repository**: Submit link to git repository containing:
   - All source code
   - Documentation
   - Test files

2. **Demonstration Video**: 5-10 minute video showing functionality

3. **Report**: PDF document (5-10 pages) including:
   - System architecture
   - Design decisions
   - Challenges encountered
   - Test results

## Tips for Success

1. **Start Simple**: Implement basic node structure first, then add features
2. **Test Early**: Write tests as you develop, not at the end
3. **Use ROS 2 Tools**: Leverage `ros2 topic echo`, `ros2 service call`, etc.
4. **Document**: Comment your code and maintain a good README
5. **Version Control**: Commit frequently with clear messages

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Example ROS 2 Packages](https://github.com/ros2/examples)

## Extension Challenges (Optional)

For extra credit, implement:

1. **Collision Avoidance**: Integrate with obstacle detection
2. **Visual Servoing**: Control arm based on camera feedback
3. **Force Control**: Implement impedance control for compliant motion
4. **Multi-Arm Coordination**: Coordinate two arms for bimanual tasks

Good luck with your ROS 2 project!
