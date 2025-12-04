---
sidebar_position: 3
title: Sensors and Actuators
description: Understanding how robots sense and interact with their environment
keywords: [sensors, actuators, IMU, encoders, motors, servo, sensor fusion]
---

# Sensors and Actuators

**Estimated Reading Time:** 20 minutes
**Difficulty:** Beginner to Intermediate
**Prerequisites:** Basic physics, Python programming

## Learning Objectives

By the end of this chapter, you will be able to:

- Identify common sensors used in robotics
- Understand sensor characteristics (accuracy, precision, noise)
- Implement sensor data processing and filtering
- Select appropriate actuators for robotic applications
- Apply sensor fusion techniques
- Write tests for sensor processing code

## Introduction

Robots interact with the physical world through:
- **Sensors**: Devices that measure physical quantities (perception)
- **Actuators**: Devices that produce motion or force (action)

## Common Robot Sensors

### 1. Encoders (Position/Velocity)

Encoders measure rotational position and velocity of motors and joints.

```python
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class EncoderReading:
    """Encoder measurement data"""
    position: float  # radians
    velocity: float  # radians/second
    timestamp: float  # seconds

class Encoder:
    """Quadrature encoder for measuring joint position"""

    def __init__(self, resolution: int):
        """
        Initialize encoder.

        Args:
            resolution: Counts per revolution (CPR)
        """
        self.resolution = resolution
        self.counts = 0
        self.last_time = None
        self.last_position = None

    def update(self, counts: int, timestamp: float) -> EncoderReading:
        """
        Process encoder counts and calculate position/velocity.

        Args:
            counts: Current encoder count
            timestamp: Measurement time (seconds)

        Returns:
            EncoderReading with position and velocity
        """
        # Convert counts to radians
        position = (counts / self.resolution) * 2 * np.pi

        # Calculate velocity
        if self.last_time is not None:
            dt = timestamp - self.last_time
            if dt > 0:
                velocity = (position - self.last_position) / dt
            else:
                velocity = 0.0
        else:
            velocity = 0.0

        self.last_time = timestamp
        self.last_position = position

        return EncoderReading(position, velocity, timestamp)
```

**Testing Encoder Processing:**

```python
def test_encoder_position():
    """Test encoder position calculation"""
    encoder = Encoder(resolution=1000)  # 1000 CPR

    # One full revolution = 1000 counts = 2π radians
    reading = encoder.update(counts=1000, timestamp=1.0)

    assert np.isclose(reading.position, 2 * np.pi)

def test_encoder_velocity():
    """Test encoder velocity calculation"""
    encoder = Encoder(resolution=1000)

    # Initial reading
    encoder.update(counts=0, timestamp=0.0)

    # After 0.1 seconds, 100 counts (10% of revolution)
    reading = encoder.update(counts=100, timestamp=0.1)

    expected_velocity = (0.1 * 2 * np.pi) / 0.1  # 0.2π rad / 0.1 sec
    assert np.isclose(reading.velocity, expected_velocity)

def test_encoder_handles_wraparound():
    """Test encoder handles count overflow/underflow"""
    encoder = Encoder(resolution=1000)

    # Simulate wraparound (implementation would need to handle this)
    # This test documents the expected behavior
    pass  # TODO: Implement wraparound handling and test
```

### 2. IMU (Inertial Measurement Unit)

IMUs measure acceleration and angular velocity using accelerometers and gyroscopes.

```python
@dataclass
class IMUReading:
    """IMU sensor data"""
    accel_x: float  # m/s²
    accel_y: float
    accel_z: float
    gyro_x: float  # rad/s
    gyro_y: float
    gyro_z: float
    timestamp: float

class IMUFilter:
    """Simple complementary filter for IMU data"""

    def __init__(self, alpha: float = 0.98):
        """
        Initialize IMU filter.

        Args:
            alpha: Complementary filter weight (0-1)
                   Higher = trust gyro more, lower = trust accel more
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.last_time = None

    def update(self, imu_data: IMUReading) -> tuple[float, float]:
        """
        Estimate roll and pitch from IMU data.

        Args:
            imu_data: Raw IMU measurements

        Returns:
            (roll, pitch) in radians
        """
        # Calculate roll and pitch from accelerometer
        accel_roll = np.arctan2(imu_data.accel_y, imu_data.accel_z)
        accel_pitch = np.arctan2(
            -imu_data.accel_x,
            np.sqrt(imu_data.accel_y**2 + imu_data.accel_z**2)
        )

        if self.last_time is not None:
            dt = imu_data.timestamp - self.last_time

            # Integrate gyro measurements
            self.roll += imu_data.gyro_x * dt
            self.pitch += imu_data.gyro_y * dt

            # Complementary filter: combine gyro and accel
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        else:
            # Initialize with accelerometer reading
            self.roll = accel_roll
            self.pitch = accel_pitch

        self.last_time = imu_data.timestamp

        return self.roll, self.pitch
```

**Testing IMU Filter:**

```python
def test_imu_filter_static():
    """Test IMU filter with robot at rest"""
    imu_filter = IMUFilter(alpha=0.98)

    # Simulate robot flat on ground
    for i in range(100):
        imu_data = IMUReading(
            accel_x=0.0, accel_y=0.0, accel_z=9.81,  # Gravity only
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,      # No rotation
            timestamp=i * 0.01
        )
        roll, pitch = imu_filter.update(imu_data)

    # Should remain near zero
    assert abs(roll) < 0.05
    assert abs(pitch) < 0.05

def test_imu_filter_tilted():
    """Test IMU filter with robot tilted"""
    imu_filter = IMUFilter(alpha=0.98)

    # Simulate 45-degree roll
    roll_angle = np.pi / 4
    g = 9.81

    for i in range(100):
        imu_data = IMUReading(
            accel_x=0.0,
            accel_y=g * np.sin(roll_angle),
            accel_z=g * np.cos(roll_angle),
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            timestamp=i * 0.01
        )
        roll, pitch = imu_filter.update(imu_data)

    # Should converge to 45 degrees
    assert np.isclose(roll, roll_angle, atol=0.1)
```

### 3. Lidar (Distance Measurement)

```python
class LidarSensor:
    """2D Lidar sensor simulation"""

    def __init__(self, num_rays: int = 360, max_range: float = 10.0):
        """
        Initialize Lidar sensor.

        Args:
            num_rays: Number of rays (angular resolution)
            max_range: Maximum sensing range (meters)
        """
        self.num_rays = num_rays
        self.max_range = max_range
        self.angle_increment = 2 * np.pi / num_rays

    def scan(self, obstacles: list) -> np.ndarray:
        """
        Simulate lidar scan with obstacles.

        Args:
            obstacles: List of (x, y, radius) tuples

        Returns:
            Array of distances for each ray
        """
        distances = np.full(self.num_rays, self.max_range)

        for i in range(self.num_rays):
            angle = i * self.angle_increment
            # Ray casting logic here
            # (simplified for demonstration)

        return distances

    def filter_outliers(self, scan: np.ndarray, threshold: float = 0.5) -> np.ndarray:
        """
        Remove outlier measurements from scan.

        Args:
            scan: Raw lidar distances
            threshold: Maximum allowed change between adjacent rays

        Returns:
            Filtered scan data
        """
        filtered = scan.copy()

        for i in range(1, len(scan) - 1):
            # Check if measurement differs significantly from neighbors
            diff_prev = abs(scan[i] - scan[i-1])
            diff_next = abs(scan[i] - scan[i+1])

            if diff_prev > threshold and diff_next > threshold:
                # Likely an outlier, replace with median of neighbors
                filtered[i] = np.median([scan[i-1], scan[i+1]])

        return filtered
```

## Actuators

### DC Motors and Servos

```python
class DCMotor:
    """DC motor model with basic physics"""

    def __init__(self, torque_constant: float, resistance: float, inertia: float):
        """
        Initialize DC motor model.

        Args:
            torque_constant: Nm/A
            resistance: Ohms
            inertia: kg⋅m²
        """
        self.Kt = torque_constant
        self.R = resistance
        self.J = inertia
        self.velocity = 0.0
        self.position = 0.0

    def update(self, voltage: float, dt: float, load_torque: float = 0.0):
        """
        Update motor state given applied voltage.

        Args:
            voltage: Applied voltage (V)
            dt: Time step (seconds)
            load_torque: External load torque (Nm)
        """
        # Motor torque from current
        current = voltage / self.R
        motor_torque = self.Kt * current

        # Net torque
        net_torque = motor_torque - load_torque

        # Update velocity (τ = J⋅α)
        acceleration = net_torque / self.J
        self.velocity += acceleration * dt

        # Update position
        self.position += self.velocity * dt
```

**Testing Motor Model:**

```python
def test_motor_no_load():
    """Test motor acceleration with no load"""
    motor = DCMotor(torque_constant=0.1, resistance=1.0, inertia=0.01)

    # Apply constant voltage
    for _ in range(100):
        motor.update(voltage=12.0, dt=0.01, load_torque=0.0)

    # Motor should have accelerated
    assert motor.velocity > 0
    assert motor.position > 0

def test_motor_stall_condition():
    """Test motor behavior when stalled"""
    motor = DCMotor(torque_constant=0.1, resistance=1.0, inertia=0.01)

    voltage = 12.0
    stall_torque = (voltage / motor.R) * motor.Kt

    # Apply load equal to motor torque
    for _ in range(100):
        motor.update(voltage=voltage, dt=0.01, load_torque=stall_torque)

    # Motor should not move
    assert np.isclose(motor.velocity, 0.0, atol=1e-3)
```

## Sensor Fusion

Combining multiple sensors improves accuracy and reliability.

```python
class KalmanFilter1D:
    """Simple 1D Kalman filter for sensor fusion"""

    def __init__(self, process_noise: float, measurement_noise: float):
        """
        Initialize Kalman filter.

        Args:
            process_noise: Process uncertainty
            measurement_noise: Measurement uncertainty
        """
        self.Q = process_noise
        self.R = measurement_noise
        self.x = 0.0  # State estimate
        self.P = 1.0  # Estimation uncertainty

    def predict(self, dt: float):
        """Prediction step"""
        # State doesn't change (constant velocity model would be different)
        # Uncertainty increases
        self.P += self.Q * dt

    def update(self, measurement: float):
        """Update step with new measurement"""
        # Kalman gain
        K = self.P / (self.P + self.R)

        # Update estimate
        self.x = self.x + K * (measurement - self.x)

        # Update uncertainty
        self.P = (1 - K) * self.P

    def get_estimate(self) -> float:
        """Get current state estimate"""
        return self.x
```

## Summary

This chapter covered:
- Common robot sensors (encoders, IMUs, Lidar)
- Sensor data processing and filtering
- Actuator models (DC motors)
- Sensor fusion with Kalman filters
- Test-driven development for sensor/actuator code

## Exercises

### Exercise 1: Sensor Noise Simulation

Add realistic noise to the encoder and IMU sensor models. Test that filters can handle noisy data.

### Exercise 2: Multi-Sensor Fusion

Implement a Kalman filter that fuses encoder and IMU data to estimate robot position.

### Exercise 3: Motor PID Control

Implement a PID controller for the DC motor to reach and maintain a target velocity.

## Next Steps

In the next chapter, we'll explore **Control Systems**, covering PID control, state-space methods, and modern control techniques.

## Additional Resources

- [Kalman Filter Interactive Guide](https://www.kalmanfilter.net/)
- [Sensor Fusion Tutorial - Coursera](https://www.coursera.org/learn/sensor-fusion)
- [IMU Basics - SparkFun](https://learn.sparkfun.com/tutorials/accelerometer-basics)

---

💡 **Pro Tip**: Always characterize your sensors in the actual deployment environment - sensor specifications are often best-case values.
