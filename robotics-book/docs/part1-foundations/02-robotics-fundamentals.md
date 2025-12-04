---
sidebar_position: 2
title: Robotics Fundamentals
description: Core mathematical and mechanical principles underlying robotic systems
keywords: [kinematics, dynamics, forward kinematics, inverse kinematics, robot mechanics]
---

# Robotics Fundamentals

**Estimated Reading Time:** 25 minutes
**Difficulty:** Beginner to Intermediate
**Prerequisites:** Linear algebra basics, Python programming

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand robot kinematics and dynamics
- Calculate forward and inverse kinematics for simple manipulators
- Apply coordinate transformations using homogeneous matrices
- Implement basic robot motion planning
- Write tests for kinematic calculations

## Robot Coordinate Systems

### Homogeneous Transformations

Robots use homogeneous transformation matrices to represent position and orientation in 3D space.

```python
import numpy as np

def create_transform(x, y, z, roll, pitch, yaw):
    """
    Create a 4x4 homogeneous transformation matrix.

    Args:
        x, y, z: Translation in meters
        roll, pitch, yaw: Rotation in radians

    Returns:
        4x4 numpy array representing the transformation
    """
    # Rotation matrices
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation
    R = Rz @ Ry @ Rx

    # Create homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T
```

**Expected Output:**
```
4x4 transformation matrix combining rotation and translation
```

### Testing Transformations

```python
import pytest
import numpy as np

def test_identity_transform():
    """Test that zero transformation is identity"""
    T = create_transform(0, 0, 0, 0, 0, 0)
    np.testing.assert_array_almost_equal(T, np.eye(4))

def test_pure_translation():
    """Test translation without rotation"""
    T = create_transform(1.0, 2.0, 3.0, 0, 0, 0)
    expected_translation = np.array([1.0, 2.0, 3.0])
    np.testing.assert_array_almost_equal(T[:3, 3], expected_translation)
    np.testing.assert_array_almost_equal(T[:3, :3], np.eye(3))

def test_rotation_preserves_distance():
    """Test that rotation doesn't change point distance from origin"""
    point = np.array([1, 0, 0, 1])
    T = create_transform(0, 0, 0, 0, 0, np.pi/4)
    transformed = T @ point
    original_dist = np.linalg.norm(point[:3])
    transformed_dist = np.linalg.norm(transformed[:3])
    assert np.isclose(original_dist, transformed_dist)
```

## Forward Kinematics

Forward kinematics calculates the end-effector position given joint angles.

### 2-Link Planar Arm Example

```python
class TwoLinkArm:
    """Simple 2-link planar robotic arm"""

    def __init__(self, L1: float, L2: float):
        """
        Initialize arm with link lengths.

        Args:
            L1: Length of first link (meters)
            L2: Length of second link (meters)
        """
        self.L1 = L1
        self.L2 = L2

    def forward_kinematics(self, theta1: float, theta2: float):
        """
        Calculate end-effector position given joint angles.

        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)

        Returns:
            (x, y): End-effector position
        """
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return x, y
```

**Testing Forward Kinematics:**

```python
def test_forward_kinematics_zero_angles():
    """Test FK when all joints at zero"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)
    x, y = arm.forward_kinematics(0, 0)
    assert np.isclose(x, 2.0)
    assert np.isclose(y, 0.0)

def test_forward_kinematics_right_angle():
    """Test FK with 90-degree configuration"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)
    x, y = arm.forward_kinematics(0, np.pi/2)
    assert np.isclose(x, 1.0)
    assert np.isclose(y, 1.0)

def test_workspace_limits():
    """Test that FK respects physical workspace limits"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)
    for _ in range(100):
        theta1 = np.random.uniform(-np.pi, np.pi)
        theta2 = np.random.uniform(-np.pi, np.pi)
        x, y = arm.forward_kinematics(theta1, theta2)
        distance = np.sqrt(x**2 + y**2)
        # Distance must be within reachable workspace
        assert distance <= arm.L1 + arm.L2
        assert distance >= abs(arm.L1 - arm.L2)
```

## Inverse Kinematics

Inverse kinematics (IK) calculates joint angles needed to reach a desired position.

```python
class TwoLinkArm:
    # ... (previous methods)

    def inverse_kinematics(self, x: float, y: float):
        """
        Calculate joint angles to reach target position.

        Args:
            x, y: Target end-effector position

        Returns:
            (theta1, theta2): Joint angles in radians
            Returns None if position unreachable

        Raises:
            ValueError: If target is outside workspace
        """
        distance = np.sqrt(x**2 + y**2)

        # Check if target is reachable
        if distance > (self.L1 + self.L2) or distance < abs(self.L1 - self.L2):
            raise ValueError(f"Target ({x}, {y}) is outside workspace")

        # Use law of cosines
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Handle numerical errors

        # Two solutions (elbow up/down)
        theta2 = np.arccos(cos_theta2)

        # Calculate theta1
        k1 = self.L1 + self.L2 * np.cos(theta2)
        k2 = self.L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return theta1, theta2
```

**Testing Inverse Kinematics:**

```python
def test_inverse_kinematics_round_trip():
    """Test that IK undoes FK"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)

    # Start with known joint angles
    theta1_orig, theta2_orig = 0.5, 0.8

    # Calculate end-effector position
    x, y = arm.forward_kinematics(theta1_orig, theta2_orig)

    # Solve IK to get back to joint angles
    theta1_computed, theta2_computed = arm.inverse_kinematics(x, y)

    # Verify we reach the same position
    x_check, y_check = arm.forward_kinematics(theta1_computed, theta2_computed)

    assert np.isclose(x, x_check, atol=1e-6)
    assert np.isclose(y, y_check, atol=1e-6)

def test_inverse_kinematics_unreachable():
    """Test that IK raises error for unreachable targets"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)

    with pytest.raises(ValueError, match="outside workspace"):
        arm.inverse_kinematics(3.0, 0.0)  # Too far

def test_inverse_kinematics_boundary():
    """Test IK at workspace boundary"""
    arm = TwoLinkArm(L1=1.0, L2=1.0)

    # Test fully extended position
    x, y = 2.0, 0.0
    theta1, theta2 = arm.inverse_kinematics(x, y)

    assert np.isclose(theta1, 0.0, atol=1e-6)
    assert np.isclose(theta2, 0.0, atol=1e-6)
```

## Robot Dynamics

Dynamics describes how forces and torques produce motion.

### Key Concepts

1. **Inertia**: Resistance to changes in motion
2. **Gravity**: Downward force on each link
3. **Coriolis/Centrifugal**: Forces due to rotation
4. **Friction**: Energy dissipation at joints

### Equation of Motion

For a robotic manipulator:

```
τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
```

Where:
- `τ` = joint torques
- `M(q)` = mass/inertia matrix
- `C(q,q̇)` = Coriolis/centrifugal terms
- `G(q)` = gravity vector
- `q, q̇, q̈` = position, velocity, acceleration

## Summary

This chapter covered:
- Coordinate transformations using homogeneous matrices
- Forward kinematics for calculating end-effector position
- Inverse kinematics for solving desired joint angles
- Introduction to robot dynamics
- Test-driven development for kinematic calculations

## Exercises

### Exercise 1: 3-Link Arm

Extend the `TwoLinkArm` class to support a 3-link planar arm. Implement:
- Forward kinematics
- Comprehensive unit tests

### Exercise 2: Workspace Visualization

Write code to visualize the reachable workspace of a 2-link arm:
```python
def plot_workspace(L1, L2):
    # Your code here: generate points and plot
    pass
```

### Exercise 3: IK with Obstacles

Modify the IK solver to avoid a circular obstacle at position `(x_obs, y_obs)` with radius `r`.

## Next Steps

Next, we'll explore **Sensors and Actuators**, covering how robots perceive and act in the physical world.

## Additional Resources

- [Modern Robotics textbook - Lynch & Park](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [Introduction to Robotics - Stanford CS223A](https://cs.stanford.edu/groups/manips/teaching/cs223a/)
- [PyRobotics library documentation](https://pyrobotics.readthedocs.io/)

---

💡 **Pro Tip**: Always validate IK solutions by running them through FK and checking the result matches the target.
