# Humanoid Robot Kinematics

## Overview

Humanoid kinematics studies the motion of humanoid robots without considering forces. Understanding kinematics is essential for controlling the position and orientation of robot links and end-effectors.

## Humanoid Robot Structure

### Key Components

```
Head
  ├─ Neck (1-3 DOF)
Torso
  ├─ Spine/Waist (1-3 DOF)
  ├─ Left Arm
  │   ├─ Shoulder (3 DOF)
  │   ├─ Elbow (1-2 DOF)
  │   └─ Wrist + Hand (3 DOF + fingers)
  ├─ Right Arm (symmetric)
  ├─ Left Leg
  │   ├─ Hip (3 DOF)
  │   ├─ Knee (1 DOF)
  │   └─ Ankle (2 DOF)
  └─ Right Leg (symmetric)

Total DOF: Typically 25-40+ for full humanoid
```

### Degrees of Freedom (DOF)

- **DOF**: Number of independent parameters defining configuration
- Human body: ~244 DOF (including fingers, spine segments)
- Humanoid robots: 25-40 DOF (trade-off between capability and complexity)

## Forward Kinematics

### Definition

Forward kinematics (FK) computes end-effector pose given joint angles.

```
Joint Angles θ₁, θ₂, ..., θₙ → End-Effector Position & Orientation (x, y, z, roll, pitch, yaw)
```

### Denavit-Hartenberg (DH) Parameters

Standard method for describing robot kinematics:

| Link | θ (angle) | d (offset) | a (length) | α (twist) |
|------|-----------|------------|------------|-----------|
| 1    | θ₁        | d₁         | a₁         | α₁        |
| 2    | θ₂        | d₂         | a₂         | α₂        |
| ...  | ...       | ...        | ...        | ...       |

### Transformation Matrices

Each joint transformation:

```
T_i = Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
```

Full forward kinematics:

```
T_end = T_0 * T_1 * T_2 * ... * T_n
```

### Python Implementation

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Compute transformation matrix from DH parameters"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])
    return T

def forward_kinematics(joint_angles, dh_params):
    """Compute end-effector pose from joint angles"""
    T = np.eye(4)

    for i, theta in enumerate(joint_angles):
        theta_total = theta + dh_params[i]['theta_offset']
        d = dh_params[i]['d']
        a = dh_params[i]['a']
        alpha = dh_params[i]['alpha']

        T = T @ dh_transform(theta_total, d, a, alpha)

    return T

# Example: Simple 3-DOF arm
dh_params = [
    {'theta_offset': 0, 'd': 0.1, 'a': 0.3, 'alpha': 0},
    {'theta_offset': 0, 'd': 0,   'a': 0.25, 'alpha': 0},
    {'theta_offset': 0, 'd': 0,   'a': 0.15, 'alpha': 0}
]

joint_angles = [np.pi/4, np.pi/6, np.pi/3]
T_end = forward_kinematics(joint_angles, dh_params)

position = T_end[:3, 3]
print(f"End-effector position: {position}")
```

## Inverse Kinematics (IK)

### Definition

Inverse kinematics computes joint angles needed to achieve desired end-effector pose.

```
Desired Pose (x, y, z, roll, pitch, yaw) → Joint Angles θ₁, θ₂, ..., θₙ
```

### Challenges

- **Multiple solutions**: Same pose can be reached with different joint configurations
- **No solution**: Pose may be out of reach
- **Singularities**: Some configurations have infinite solutions or no solution
- **Computational complexity**: Non-linear problem, can be slow

### Analytical IK (Closed-Form)

For simple kinematic chains, solve algebraically:

```python
def analytical_ik_2dof_arm(target_x, target_y, L1, L2):
    """Analytical IK for 2-DOF planar arm"""

    # Distance to target
    D = np.sqrt(target_x**2 + target_y**2)

    # Check reachability
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None  # Unreachable

    # Law of cosines for theta2
    cos_theta2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)

    # Two solutions: elbow up, elbow down
    theta2_solutions = [theta2, -theta2]

    solutions = []
    for theta2 in theta2_solutions:
        # Solve for theta1
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(target_y, target_x) - np.arctan2(k2, k1)

        solutions.append([theta1, theta2])

    return solutions
```

### Numerical IK (Iterative)

For complex robots, use numerical optimization:

```python
from scipy.optimize import minimize

def numerical_ik(target_pose, initial_guess, dh_params):
    """Numerical IK using optimization"""

    def objective(joint_angles):
        """Minimize distance between current and target pose"""
        T_current = forward_kinematics(joint_angles, dh_params)
        current_pos = T_current[:3, 3]
        target_pos = target_pose[:3]

        # Position error
        pos_error = np.linalg.norm(current_pos - target_pos)

        # Optionally add orientation error
        return pos_error

    result = minimize(
        objective,
        initial_guess,
        method='SLSQP',
        bounds=[(-np.pi, np.pi)] * len(initial_guess)
    )

    if result.success:
        return result.x
    else:
        return None
```

### Jacobian-Based IK

Use Jacobian matrix for iterative IK:

```python
def jacobian_ik(target_pose, initial_angles, dh_params, max_iter=100, tol=1e-3):
    """IK using Jacobian pseudo-inverse method"""

    joint_angles = np.array(initial_angles)

    for _ in range(max_iter):
        # Current end-effector pose
        T_current = forward_kinematics(joint_angles, dh_params)
        current_pos = T_current[:3, 3]

        # Error
        error = target_pose[:3] - current_pos

        if np.linalg.norm(error) < tol:
            return joint_angles  # Converged

        # Compute Jacobian numerically
        J = compute_jacobian(joint_angles, dh_params)

        # Pseudo-inverse
        J_pinv = np.linalg.pinv(J)

        # Update joint angles
        delta_theta = J_pinv @ error
        joint_angles += 0.1 * delta_theta  # Damped update

    return joint_angles  # May not have converged

def compute_jacobian(joint_angles, dh_params, epsilon=1e-6):
    """Compute Jacobian numerically"""
    J = []

    # Current position
    T_0 = forward_kinematics(joint_angles, dh_params)
    pos_0 = T_0[:3, 3]

    for i in range(len(joint_angles)):
        # Perturb joint i
        angles_perturbed = joint_angles.copy()
        angles_perturbed[i] += epsilon

        # Compute perturbed position
        T_perturbed = forward_kinematics(angles_perturbed, dh_params)
        pos_perturbed = T_perturbed[:3, 3]

        # Finite difference derivative
        derivative = (pos_perturbed - pos_0) / epsilon
        J.append(derivative)

    return np.array(J).T
```

## URDF and Robot Description

### Loading Robot Model

```python
import pybullet as p
import rclpy
from urdf_parser_py.urdf import URDF

def load_humanoid_urdf(urdf_path):
    """Load humanoid robot from URDF"""

    # PyBullet for physics
    robot_id = p.loadURDF(urdf_path, useFixedBase=False)

    # Get joint info
    num_joints = p.getNumJoints(robot_id)
    joint_info = []

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_info.append({
            'name': info[1].decode('utf-8'),
            'type': info[2],
            'lower_limit': info[8],
            'upper_limit': info[9]
        })

    return robot_id, joint_info
```

## Whole-Body IK for Humanoids

### Multiple End-Effectors

Humanoids often need to control multiple parts simultaneously:
- Left hand position
- Right hand position
- Head orientation
- Center of Mass (CoM) position

```python
def whole_body_ik(targets, robot_model):
    """IK for multiple end-effectors"""

    def objective(joint_angles):
        total_error = 0

        # Set robot configuration
        robot_model.set_joint_angles(joint_angles)

        # Error for each end-effector
        for effector, target in targets.items():
            current_pose = robot_model.get_link_pose(effector)
            error = np.linalg.norm(current_pose - target)
            total_error += error

        return total_error

    result = minimize(
        objective,
        initial_guess,
        method='SLSQP',
        bounds=robot_model.joint_limits
    )

    return result.x
```

## Practical Exercises

### Exercise 1: Forward Kinematics

Implement FK for a 3-DOF robot arm:

```python
# TODO: Student implementation
# 1. Define DH parameters
# 2. Implement transformation matrices
# 3. Compute end-effector position
# 4. Visualize the arm configuration
```

### Exercise 2: Analytical IK

Solve IK for a 2-DOF planar arm:

```python
# TODO: Student implementation
# 1. Implement law of cosines solution
# 2. Handle both elbow-up and elbow-down solutions
# 3. Check reachability
# 4. Visualize both solutions
```

### Exercise 3: Numerical IK

Implement iterative IK for a humanoid arm:

```python
# TODO: Student implementation
# 1. Load humanoid URDF
# 2. Implement Jacobian computation
# 3. Implement Jacobian-based IK
# 4. Test with various target poses
```

## Key Takeaways

- Forward kinematics: joint angles → end-effector pose
- Inverse kinematics: desired pose → joint angles
- Analytical IK is fast but limited to simple chains
- Numerical IK handles complex robots but may be slow
- Whole-body IK coordinates multiple end-effectors
- Singularities and multiple solutions are inherent challenges

## Resources

- [Modern Robotics: Kinematics Textbook](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [PyBullet Inverse Kinematics](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit)
- [Introduction to Humanoid Robotics](https://www.springer.com/gp/book/9783642540950)

## Next Steps

Continue to [Bipedal Locomotion](./bipedal-locomotion.md) to learn about walking and balance control.
