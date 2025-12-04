---
sidebar_position: 1
title: Bipedal Locomotion
description: Walking and balance control for humanoid robots
keywords: [bipedal locomotion, walking, balance, ZMP, gait, humanoid]
---

# Bipedal Locomotion

**Estimated Reading Time:** 25 minutes
**Difficulty:** Advanced
**Prerequisites:** Part 1 complete, especially kinematics and control systems

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the challenges of bipedal walking
- Implement Zero Moment Point (ZMP) control
- Generate walking gaits using trajectory planning
- Apply balance control strategies
- Test locomotion controllers in simulation

## Introduction

Bipedal locomotion is one of the most challenging problems in robotics. Unlike wheeled or quadruped robots, bipedal robots:
- Are inherently unstable
- Have a small support polygon
- Must continuously manage center of mass
- Require coordinated multi-joint control

## Stability Concepts

### Zero Moment Point (ZMP)

The ZMP is the point on the ground where the net moment from ground reaction forces is zero.

**Stability Condition**: Robot is stable if ZMP is inside the support polygon (foot contact area).

```python
import numpy as np
from dataclasses import dataclass

@dataclass
class RobotState:
    """Humanoid robot state"""
    com_position: np.ndarray  # Center of mass [x, y, z]
    com_velocity: np.ndarray
    com_acceleration: np.ndarray
    foot_positions: dict  # {'left': [x,y,z], 'right': [x,y,z]}

class ZMPCalculator:
    """Calculate Zero Moment Point for balance"""

    def __init__(self, mass: float, gravity: float = 9.81):
        """
        Initialize ZMP calculator.

        Args:
            mass: Total robot mass (kg)
            gravity: Gravitational acceleration (m/s²)
        """
        self.mass = mass
        self.g = gravity

    def calculate_zmp(self, state: RobotState) -> tuple[float, float]:
        """
        Calculate ZMP position.

        Args:
            state: Current robot state

        Returns:
            (zmp_x, zmp_y): ZMP coordinates on ground
        """
        # Simplified ZMP calculation (assumes flat ground)
        com_height = state.com_position[2]

        zmp_x = state.com_position[0] - (com_height / self.g) * state.com_acceleration[0]
        zmp_y = state.com_position[1] - (com_height / self.g) * state.com_acceleration[1]

        return zmp_x, zmp_y

    def is_stable(
        self,
        zmp: tuple[float, float],
        support_polygon: list[tuple[float, float]]
    ) -> bool:
        """
        Check if ZMP is inside support polygon.

        Args:
            zmp: (x, y) ZMP position
            support_polygon: List of (x, y) vertices

        Returns:
            True if stable (ZMP inside polygon)
        """
        # Point-in-polygon test
        # TODO: Implement proper point-in-polygon algorithm
        return True  # Placeholder
```

## Gait Generation

### Walking Pattern Generator

```python
class WalkingGaitGenerator:
    """Generate walking trajectories"""

    def __init__(
        self,
        step_length: float,
        step_height: float,
        step_duration: float
    ):
        """
        Initialize gait generator.

        Args:
            step_length: Length of each step (m)
            step_height: Maximum foot height during swing (m)
            step_duration: Time for one step (seconds)
        """
        self.step_length = step_length
        self.step_height = step_height
        self.step_duration = step_duration

    def generate_foot_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        t: float
    ) -> np.ndarray:
        """
        Generate swing foot trajectory.

        Args:
            start_pos: Starting foot position [x, y, z]
            end_pos: Ending foot position [x, y, z]
            t: Time in step cycle (0 to 1)

        Returns:
            Current foot position [x, y, z]
        """
        # Cubic polynomial for smooth trajectory
        s = 3 * t**2 - 2 * t**3  # Smooth interpolation (0 to 1)

        # Horizontal motion
        pos = start_pos + s * (end_pos - start_pos)

        # Vertical motion (parabolic arc)
        if 0 < t < 1:
            pos[2] += self.step_height * 4 * t * (1 - t)

        return pos
```

**Testing Gait Generation:**

```python
def test_foot_trajectory_starts_at_origin():
    """Test that trajectory starts at specified position"""
    generator = WalkingGaitGenerator(
        step_length=0.3,
        step_height=0.05,
        step_duration=0.5
    )

    start = np.array([0.0, 0.0, 0.0])
    end = np.array([0.3, 0.0, 0.0])

    pos = generator.generate_foot_trajectory(start, end, t=0.0)
    np.testing.assert_array_almost_equal(pos, start)

def test_foot_trajectory_ends_at_target():
    """Test that trajectory ends at specified position"""
    generator = WalkingGaitGenerator(
        step_length=0.3,
        step_height=0.05,
        step_duration=0.5
    )

    start = np.array([0.0, 0.0, 0.0])
    end = np.array([0.3, 0.0, 0.0])

    pos = generator.generate_foot_trajectory(start, end, t=1.0)
    np.testing.assert_array_almost_equal(pos, end)

def test_foot_clears_obstacle():
    """Test that foot reaches specified clearance height"""
    generator = WalkingGaitGenerator(
        step_length=0.3,
        step_height=0.05,
        step_duration=0.5
    )

    start = np.array([0.0, 0.0, 0.0])
    end = np.array([0.3, 0.0, 0.0])

    # Check midpoint of swing
    pos = generator.generate_foot_trajectory(start, end, t=0.5)

    # Should be at maximum height
    assert pos[2] >= 0.04  # Near step_height
```

## Balance Control

### COM Trajectory Planning

```python
class COMController:
    """Center of Mass trajectory controller"""

    def __init__(self, com_height: float):
        """
        Initialize COM controller.

        Args:
            com_height: Desired COM height (m)
        """
        self.com_height = com_height

    def generate_com_trajectory(
        self,
        current_com: np.ndarray,
        target_zmp: np.ndarray,
        dt: float
    ) -> np.ndarray:
        """
        Generate COM trajectory to achieve desired ZMP.

        Args:
            current_com: Current COM position [x, y, z]
            target_zmp: Desired ZMP position [x, y]
            dt: Time step

        Returns:
            Desired COM acceleration [ax, ay, az]
        """
        # Simplified preview control
        # Real implementation would use more sophisticated control

        g = 9.81
        omega = np.sqrt(g / self.com_height)

        # Proportional control of COM based on ZMP error
        Kp = omega**2

        com_acceleration = np.zeros(3)
        com_acceleration[0] = Kp * (current_com[0] - target_zmp[0])
        com_acceleration[1] = Kp * (current_com[1] - target_zmp[1])

        return com_acceleration
```

## Summary

This chapter covered:
- ZMP stability criterion for bipedal robots
- Walking gait generation with foot trajectories
- Center of mass control for balance
- Test-driven development for locomotion

## Exercises

### Exercise 1: Support Polygon

Implement the point-in-polygon test for the `is_stable()` method.

### Exercise 2: Dynamic Walking

Extend the gait generator to support variable step length and turning.

### Exercise 3: Push Recovery

Simulate a push disturbance and implement a controller to recover balance.

## Next Steps

Next chapter: **Motion Planning** for collision-free humanoid navigation.

## Additional Resources

- [Introduction to Humanoid Robotics - Kajita](https://link.springer.com/book/10.1007/978-3-642-54536-8)
- [Bipedal Walking - MIT Course](http://underactuated.mit.edu/)

---

💡 **Pro Tip**: Always test walking controllers in simulation with realistic ground contact models before deploying on hardware.
