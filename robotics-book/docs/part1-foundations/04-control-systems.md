---
sidebar_position: 4
title: Control Systems Basics
description: Fundamental control theory for robotic systems
keywords: [PID control, feedback control, state space, control theory, stability]
---

# Control Systems Basics

**Estimated Reading Time:** 20 minutes
**Difficulty:** Intermediate
**Prerequisites:** Basic calculus, Python programming, previous chapters

## Learning Objectives

By the end of this chapter, you will be able to:

- Implement PID controllers for robotic systems
- Tune PID gains using systematic methods
- Understand stability and performance tradeoffs
- Apply state-space control methods
- Write comprehensive tests for controllers

## Introduction to Feedback Control

Feedback control systems measure the output and adjust inputs to achieve desired behavior.

```
        ┌──────────────┐
r(t) ─→ │ Controller   │ ─→ u(t) ─→ │ System │ ─→ y(t)
   ↑    └──────────────┘             └────────┘      │
   │                                                  │
   └──────────────────────────────────────────────────┘
                  Feedback
```

- `r(t)` = reference (setpoint)
- `y(t)` = output (measurement)
- `u(t)` = control input
- `e(t) = r(t) - y(t)` = error

## PID Control

PID (Proportional-Integral-Derivative) is the most common control algorithm.

```python
class PIDController:
    """PID controller implementation with anti-windup"""

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        output_limits: tuple[float, float] = (-float('inf'), float('inf'))
    ):
        """
        Initialize PID controller.

        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            output_limits: (min, max) output saturation limits
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits

        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, setpoint: float, measurement: float, time: float) -> float:
        """
        Calculate control output.

        Args:
            setpoint: Desired value
            measurement: Current measured value
            time: Current time (seconds)

        Returns:
            Control output u(t)
        """
        error = setpoint - measurement

        # Proportional term
        P = self.Kp * error

        # Integral term
        if self.last_time is not None:
            dt = time - self.last_time
            self.integral += error * dt
            I = self.Ki * self.integral
        else:
            I = 0.0

        # Derivative term
        if self.last_error is not None and self.last_time is not None:
            dt = time - self.last_time
            if dt > 0:
                derivative = (error - self.last_error) / dt
                D = self.Kd * derivative
            else:
                D = 0.0
        else:
            D = 0.0

        # Calculate output
        output = P + I + D

        # Apply output limits (saturation)
        output = max(self.output_limits[0], min(output, self.output_limits[1]))

        # Anti-windup: stop integrating if saturated
        if output == self.output_limits[0] or output == self.output_limits[1]:
            # Don't update integral if output is saturated
            if self.last_time is not None:
                self.integral -= error * dt

        self.last_error = error
        self.last_time = time

        return output

    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.last_error = None
        self.last_time = None
```

**Testing PID Controller:**

```python
import numpy as np

def test_pid_proportional_only():
    """Test P-controller (Ki=Kd=0)"""
    controller = PIDController(Kp=1.0, Ki=0.0, Kd=0.0)

    output = controller.update(setpoint=10.0, measurement=5.0, time=0.0)

    # Output should be proportional to error
    expected = 1.0 * (10.0 - 5.0)
    assert np.isclose(output, expected)

def test_pid_integral_accumulation():
    """Test that integral term accumulates error"""
    controller = PIDController(Kp=0.0, Ki=1.0, Kd=0.0)

    # Constant error over time
    outputs = []
    for t in range(5):
        output = controller.update(setpoint=10.0, measurement=5.0, time=float(t))
        outputs.append(output)

    # Integral should accumulate (output increases)
    assert all(outputs[i] < outputs[i+1] for i in range(len(outputs)-1))

def test_pid_derivative_response():
    """Test derivative term responds to rate of change"""
    controller = PIDController(Kp=0.0, Ki=0.0, Kd=1.0)

    # First measurement
    output1 = controller.update(setpoint=10.0, measurement=0.0, time=0.0)

    # Error increasing rapidly
    output2 = controller.update(setpoint=10.0, measurement=5.0, time=1.0)

    # Error increasing slowly
    controller.reset()
    controller.update(setpoint=10.0, measurement=0.0, time=0.0)
    output3 = controller.update(setpoint=10.0, measurement=1.0, time=1.0)

    # Derivative response should be stronger for rapid change
    assert abs(output2) < abs(output3)

def test_pid_output_limits():
    """Test output saturation"""
    controller = PIDController(
        Kp=10.0, Ki=0.0, Kd=0.0,
        output_limits=(-5.0, 5.0)
    )

    # Large error would produce large output
    output = controller.update(setpoint=100.0, measurement=0.0, time=0.0)

    # But output should be clamped
    assert output == 5.0

def test_pid_anti_windup():
    """Test anti-windup prevents integral buildup during saturation"""
    controller = PIDController(
        Kp=1.0, Ki=1.0, Kd=0.0,
        output_limits=(-10.0, 10.0)
    )

    # Saturate the controller
    for t in range(100):
        controller.update(setpoint=100.0, measurement=0.0, time=float(t))

    # Now set a reachable setpoint
    controller.update(setpoint=5.0, measurement=4.0, time=100.0)

    # Without anti-windup, integral would be huge and cause overshoot
    # With anti-windup, response should be reasonable
    assert abs(controller.integral) < 200.0  # Reasonable bound
```

## PID Tuning

### Ziegler-Nichols Method

```python
def tune_pid_ziegler_nichols(Ku: float, Tu: float, control_type: str = 'PID'):
    """
    Calculate PID gains using Ziegler-Nichols method.

    Args:
        Ku: Ultimate gain (gain at which system oscillates)
        Tu: Ultimate period (oscillation period)
        control_type: 'P', 'PI', or 'PID'

    Returns:
        Dictionary with Kp, Ki, Kd values
    """
    if control_type == 'P':
        return {'Kp': 0.5 * Ku, 'Ki': 0.0, 'Kd': 0.0}
    elif control_type == 'PI':
        return {'Kp': 0.45 * Ku, 'Ki': 0.54 * Ku / Tu, 'Kd': 0.0}
    elif control_type == 'PID':
        return {
            'Kp': 0.6 * Ku,
            'Ki': 1.2 * Ku / Tu,
            'Kd': 0.075 * Ku * Tu
        }
    else:
        raise ValueError(f"Unknown control type: {control_type}")
```

## Practical Example: Robot Joint Control

```python
class RobotJoint:
    """Simple robot joint simulation"""

    def __init__(self, inertia: float, damping: float):
        """
        Initialize joint.

        Args:
            inertia: Rotational inertia (kg⋅m²)
            damping: Viscous damping coefficient
        """
        self.J = inertia
        self.b = damping
        self.position = 0.0
        self.velocity = 0.0

    def update(self, torque: float, dt: float):
        """
        Update joint state given applied torque.

        Args:
            torque: Applied torque (Nm)
            dt: Time step (seconds)
        """
        # Equation of motion: J⋅θ̈ + b⋅θ̇ = τ
        acceleration = (torque - self.b * self.velocity) / self.J
        self.velocity += acceleration * dt
        self.position += self.velocity * dt


def simulate_joint_control():
    """Simulate PID control of robot joint"""
    joint = RobotJoint(inertia=0.1, damping=0.05)
    controller = PIDController(Kp=10.0, Ki=2.0, Kd=1.0, output_limits=(-50, 50))

    target_position = 1.57  # 90 degrees
    dt = 0.01
    time = 0.0

    positions = []
    times = []

    for _ in range(500):
        # PID control
        torque = controller.update(target_position, joint.position, time)

        # Update joint
        joint.update(torque, dt)

        positions.append(joint.position)
        times.append(time)
        time += dt

    return times, positions
```

**Testing Closed-Loop System:**

```python
def test_joint_reaches_setpoint():
    """Test that PID control reaches target position"""
    joint = RobotJoint(inertia=0.1, damping=0.05)
    controller = PIDController(Kp=10.0, Ki=2.0, Kd=1.0, output_limits=(-50, 50))

    target = 1.0
    dt = 0.01
    time = 0.0

    # Run simulation
    for _ in range(1000):
        torque = controller.update(target, joint.position, time)
        joint.update(torque, dt)
        time += dt

    # Should reach setpoint within tolerance
    assert np.isclose(joint.position, target, atol=0.01)
    # Should be stable (low velocity)
    assert abs(joint.velocity) < 0.1

def test_joint_control_stability():
    """Test that control system is stable (no divergence)"""
    joint = RobotJoint(inertia=0.1, damping=0.05)
    controller = PIDController(Kp=10.0, Ki=2.0, Kd=1.0, output_limits=(-50, 50))

    target = 1.0
    dt = 0.01
    time = 0.0

    max_position = 0.0

    # Run for extended time
    for _ in range(2000):
        torque = controller.update(target, joint.position, time)
        joint.update(torque, dt)
        max_position = max(max_position, abs(joint.position))
        time += dt

    # Position should remain bounded (stable)
    assert max_position < 2.0  # Shouldn't go wild

def test_joint_control_no_overshoot():
    """Test that system doesn't overshoot significantly"""
    joint = RobotJoint(inertia=0.1, damping=0.05)
    controller = PIDController(Kp=5.0, Ki=1.0, Kd=2.0, output_limits=(-50, 50))

    target = 1.0
    dt = 0.01
    time = 0.0

    max_position = 0.0

    # Run simulation
    for _ in range(500):
        torque = controller.update(target, joint.position, time)
        joint.update(torque, dt)
        max_position = max(max_position, joint.position)
        time += dt

    # Overshoot should be less than 20%
    overshoot = (max_position - target) / target
    assert overshoot < 0.2
```

## Advanced Topics

### State-Space Control

State-space representation:
```
ẋ = Ax + Bu
y = Cx + Du
```

Where:
- `x` = state vector
- `u` = input vector
- `y` = output vector
- `A, B, C, D` = system matrices

### LQR (Linear Quadratic Regulator)

Optimal control that minimizes:
```
J = ∫(x^T Q x + u^T R u)dt
```

Where `Q` and `R` weight state error and control effort.

## Summary

This chapter covered:
- PID control implementation with anti-windup
- PID tuning methods (Ziegler-Nichols)
- Closed-loop system simulation and testing
- Stability and performance analysis
- Introduction to state-space control

## Exercises

### Exercise 1: Tune PID for Different Systems

Implement and tune PID controllers for:
- Fast, lightly-damped system
- Slow, heavily-damped system
Compare performance.

### Exercise 2: Cascade Control

Implement cascade control for a robot joint:
- Inner loop: velocity control
- Outer loop: position control

### Exercise 3: Disturbance Rejection

Add external disturbances to the joint simulation. Test how well the PID controller rejects them.

## Next Steps

This completes **Part 1: Foundations**. You now have the mathematical and practical foundations for Physical AI systems.

In **Part 2: Humanoid Robotics**, we'll apply these concepts to bipedal robots, exploring locomotion, manipulation, and human-robot interaction.

## Additional Resources

- [Control System Design - MIT OpenCourseWare](https://ocw.mit.edu/courses/mechanical-engineering/)
- [Brian Douglas Control Tutorials (YouTube)](https://www.youtube.com/user/ControlLectures)
- [Python Control Systems Library](https://python-control.readthedocs.io/)

---

💡 **Pro Tip**: Start with a pure P controller, then add I to eliminate steady-state error, and finally add D to reduce overshoot. Tune one gain at a time.
