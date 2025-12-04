# Bipedal Locomotion

## Overview

Bipedal locomotion is one of the most challenging aspects of humanoid robotics. Unlike wheeled or quadruped robots, bipedal robots must constantly maintain balance while dynamically shifting weight between two legs.

## Why Bipedal Locomotion is Hard

### Challenges

1. **Underactuation**: Only two contact points with the ground
2. **Dynamic Instability**: Must actively maintain balance
3. **High-dimensional control**: 12+ DOF in legs alone
4. **Energy efficiency**: Biological walking is remarkably efficient; robots lag behind
5. **Robustness**: Must handle uneven terrain, disturbances, obstacles

### Static vs. Dynamic Walking

| Static Walking | Dynamic Walking |
|----------------|-----------------|
| Center of Mass (CoM) always over support polygon | CoM may be outside support polygon |
| Slow and stable | Fast and natural |
| Low energy efficiency | Higher efficiency possible |
| Safe but limited | Enables running, jumping |

## Fundamentals of Balance

### Center of Mass (CoM)

The average position of all mass in the robot:

```python
def compute_com(robot_state):
    """Compute center of mass position"""
    total_mass = 0
    weighted_pos = np.zeros(3)

    for link in robot_state.links:
        mass = link.mass
        position = link.position

        weighted_pos += mass * position
        total_mass += mass

    com = weighted_pos / total_mass
    return com
```

### Zero Moment Point (ZMP)

The point on the ground where the net moment of ground reaction forces is zero.

**ZMP Stability Criterion**: Robot is stable if ZMP is inside the support polygon (footprint area).

```python
def compute_zmp(robot_state, forces):
    """Compute Zero Moment Point"""

    # Simplified ZMP calculation
    # Full calculation involves all forces and torques

    com = compute_com(robot_state)
    com_accel = robot_state.com_acceleration

    # Simplified ZMP (assuming flat ground at z=0)
    zmp_x = com[0] - (com[2] / 9.81) * com_accel[0]
    zmp_y = com[1] - (com[2] / 9.81) * com_accel[1]

    return np.array([zmp_x, zmp_y, 0])
```

### Support Polygon

The convex hull of all contact points:

```python
from scipy.spatial import ConvexHull

def compute_support_polygon(contact_points):
    """Compute support polygon from foot contacts"""

    if len(contact_points) < 3:
        # Line segment for single or double support
        return contact_points

    # Convex hull for multiple contacts
    hull = ConvexHull(contact_points[:, :2])  # 2D projection
    return contact_points[hull.vertices]

def is_stable(zmp, support_polygon):
    """Check if ZMP is inside support polygon"""

    # Use point-in-polygon test
    from matplotlib.path import Path

    polygon_path = Path(support_polygon[:, :2])
    return polygon_path.contains_point(zmp[:2])
```

## Gait Patterns

### Walking Phases

1. **Double Support**: Both feet on ground
2. **Single Support**: One foot on ground, other swinging
3. **Swing Phase**: Moving foot from back to front
4. **Stance Phase**: Foot on ground supporting weight

### Gait Cycle

```
Right Heel Strike → Left Toe Off → Left Heel Strike → Right Toe Off → Right Heel Strike
    (Double)         (Single)        (Double)          (Single)        (Double)
```

### Implementing a Simple Gait

```python
class GaitGenerator:
    def __init__(self):
        self.step_length = 0.2  # meters
        self.step_height = 0.05  # meters
        self.step_duration = 1.0  # seconds

    def generate_foot_trajectory(self, t, is_stance):
        """Generate foot trajectory for swing or stance phase"""

        if is_stance:
            # Stance foot stays on ground
            return np.array([0, 0, 0])

        # Swing foot follows trajectory
        # Normalized time in swing phase (0 to 1)
        phase = (t % self.step_duration) / self.step_duration

        # Forward motion (straight line)
        x = self.step_length * phase

        # Vertical motion (parabola)
        y = 0
        z = 4 * self.step_height * phase * (1 - phase)

        return np.array([x, y, z])

    def generate_com_trajectory(self, t):
        """Generate CoM trajectory for stable walking"""

        # Sinusoidal lateral shift during walking
        phase = (t % self.step_duration) / self.step_duration

        # Shift CoM over stance foot
        lateral_shift = 0.05 * np.sin(2 * np.pi * phase)

        # Vertical oscillation (reduce impact)
        vertical = -0.01 * np.cos(2 * np.pi * phase)

        return np.array([0, lateral_shift, vertical])
```

## Preview Control

### Model Predictive Control (MPC)

Plan CoM trajectory over future time horizon:

```python
class PreviewController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon  # Look ahead N steps
        self.dt = dt

    def compute_com_trajectory(self, current_state, target_zmp):
        """Compute CoM trajectory that achieves target ZMP"""

        # Simplified linear inverted pendulum model
        # x_com = [position, velocity]

        A = np.array([[1, self.dt], [0, 1]])
        B = np.array([[self.dt**2 / 2], [self.dt]])

        # Preview control formulation
        Q = np.eye(2) * 10  # State weight
        R = np.eye(1) * 1   # Control weight

        # Solve optimal control problem
        # Minimize: sum(||x_com - x_desired||^2 + ||u||^2)

        # Simplified implementation
        com_trajectory = []
        state = current_state

        for i in range(self.horizon):
            # Compute control to reach target ZMP
            zmp_error = target_zmp[i] - self.compute_zmp(state)
            control = self.compute_optimal_control(state, zmp_error)

            # Update state
            state = A @ state + B @ control
            com_trajectory.append(state[0])

        return com_trajectory
```

## Capture Point Theory

### Capture Point

The point on the ground where the robot should step to come to a complete stop.

```python
def compute_capture_point(com_pos, com_vel, gravity=9.81):
    """Compute instantaneous capture point"""

    # For linear inverted pendulum model
    omega = np.sqrt(gravity / com_pos[2])  # Natural frequency

    # Capture point in x-y plane
    cp = com_pos[:2] + com_vel[:2] / omega

    return cp

def compute_footstep_location(capture_point, com_pos):
    """Determine next footstep location"""

    # Place foot at or near capture point
    # Add offset for desired walking direction

    footstep = capture_point.copy()

    # Constrain to reachable workspace
    max_step_length = 0.3
    step_vector = footstep - com_pos[:2]

    if np.linalg.norm(step_vector) > max_step_length:
        step_vector = step_vector / np.linalg.norm(step_vector) * max_step_length
        footstep = com_pos[:2] + step_vector

    return footstep
```

## Linear Inverted Pendulum Model (LIPM)

Simplified dynamics for walking:

```
ẍ = ω² (x - p)

where:
x = CoM position
p = ZMP position
ω = √(g/h), h = CoM height
```

```python
class InvertedPendulum:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.h = com_height
        self.g = gravity
        self.omega = np.sqrt(gravity / com_height)

    def dynamics(self, state, zmp):
        """Compute CoM dynamics given ZMP"""

        # state = [x, x_dot]
        x, x_dot = state

        # LIPM dynamics: x_ddot = omega^2 * (x - zmp)
        x_ddot = self.omega**2 * (x - zmp)

        # Return derivatives [x_dot, x_ddot]
        return np.array([x_dot, x_ddot])

    def simulate(self, initial_state, zmp_trajectory, dt=0.01):
        """Simulate LIPM dynamics"""

        state = initial_state
        trajectory = [state[0]]

        for zmp in zmp_trajectory:
            # Euler integration
            derivatives = self.dynamics(state, zmp)
            state = state + derivatives * dt
            trajectory.append(state[0])

        return np.array(trajectory)
```

## Whole-Body Control

### Hierarchical Quadratic Programming (HQP)

Solve multiple objectives with priority:

1. **High Priority**: Contact constraints (feet on ground)
2. **Medium Priority**: CoM trajectory tracking
3. **Low Priority**: Joint limit avoidance, posture

```python
from qpsolvers import solve_qp

class WholeBodyController:
    def __init__(self, robot_model):
        self.robot = robot_model

    def compute_joint_torques(self, desired_com, desired_feet):
        """Compute joint torques for desired motion"""

        # Get robot state
        q = self.robot.get_joint_positions()
        q_dot = self.robot.get_joint_velocities()

        # Compute Jacobians
        J_com = self.robot.compute_com_jacobian(q)
        J_left_foot = self.robot.compute_link_jacobian(q, "left_foot")
        J_right_foot = self.robot.compute_link_jacobian(q, "right_foot")

        # Task: Track CoM trajectory
        com_error = desired_com - self.robot.compute_com(q)
        v_com_desired = self.Kp_com * com_error

        # Task: Keep feet fixed (stance phase)
        v_feet_desired = np.zeros(6)  # Zero velocity for stance

        # Quadratic program formulation
        # Minimize: ||J_com * q_dot - v_com_desired||^2
        # Subject to: J_foot * q_dot = 0 (foot contact constraints)

        # QP matrices
        P = J_com.T @ J_com
        q = -J_com.T @ v_com_desired

        # Constraints
        A_eq = np.vstack([J_left_foot, J_right_foot])
        b_eq = np.zeros(12)  # Both feet fixed

        # Solve QP
        q_dot_desired = solve_qp(P, q, A=A_eq, b=b_eq)

        # Compute joint torques from desired accelerations
        tau = self.robot.inverse_dynamics(q, q_dot, q_dot_desired)

        return tau
```

## Practical Exercises

### Exercise 1: ZMP Computation

Implement ZMP calculation and stability check:

```python
# TODO: Student implementation
# 1. Load humanoid model in simulation
# 2. Compute CoM and ZMP during walking
# 3. Visualize ZMP and support polygon
# 4. Verify stability criterion
```

### Exercise 2: Gait Generation

Create a simple walking gait:

```python
# TODO: Student implementation
# 1. Generate foot swing trajectories
# 2. Generate CoM trajectory
# 3. Use IK to compute joint angles
# 4. Simulate in Gazebo/Isaac Sim
```

### Exercise 3: Push Recovery

Implement balance recovery from external disturbance:

```python
# TODO: Student implementation
# 1. Apply external push to robot
# 2. Compute capture point
# 3. Plan recovery footstep
# 4. Execute stabilizing motion
```

## Key Takeaways

- ZMP must stay inside support polygon for stability
- Gait generation requires coordinating foot and CoM trajectories
- Linear Inverted Pendulum Model simplifies walking dynamics
- Capture Point Theory helps with balance recovery
- Whole-body control coordinates all joints for desired motion
- Bipedal walking is complex but enables human-like navigation

## Resources

- [Introduction to Humanoid Robotics (Kajita et al.)](https://www.springer.com/gp/book/9783642540950)
- [MIT Underactuated Robotics Course](http://underactuated.mit.edu/)
- [ZMP Tutorial](https://scaron.info/teaching/zero-tilting-moment-point.html)

## Next Steps

Continue to [Manipulation and Grasping](./manipulation-grasping.md) to learn about object interaction.
