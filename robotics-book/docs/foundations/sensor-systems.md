---
sidebar_position: 4
title: Sensor Systems for Physical AI
description: Understanding sensors that enable robots to perceive and interact with the physical world
keywords: [sensors, LIDAR, cameras, IMU, force sensors, depth cameras, robotics]
---

# Sensor Systems for Physical AI

## The Robot's Senses

Just as humans rely on five senses to perceive the world, robots use **sensor systems** to:

- **See**: Cameras, LIDAR, depth sensors
- **Feel**: Force/torque sensors, tactile arrays, proprioception
- **Hear**: Microphones for voice commands and audio understanding
- **Balance**: IMUs (Inertial Measurement Units) for orientation and acceleration
- **Localize**: GPS, wheel encoders, visual odometry

This chapter surveys the essential sensors for humanoid robotics and Physical AI.

## Vision Sensors

### RGB Cameras

**What they measure**: Color images (Red, Green, Blue channels)

**Use cases**:
- Object detection and recognition
- Scene understanding
- Visual servoing (using vision to guide motion)
- Human-robot interaction (gesture recognition)

**Specifications**:
- **Resolution**: 720p to 4K (higher = more detail but more computation)
- **Frame rate**: 30-60 FPS for standard, 120+ for high-speed motion
- **Field of view**: Wide (120°) for navigation, narrow (60°) for manipulation

**Common models**:
- Intel RealSense D435i RGB-D camera
- Raspberry Pi Camera Module
- Industrial machine vision cameras

**Challenges**:
- **Lighting sensitivity**: Performance degrades in low light
- **No depth information**: 2D image of 3D world (need depth estimation)
- **Computational load**: Processing HD video in real-time

### Depth Cameras (RGB-D)

**What they measure**: Color image + depth (distance to each pixel)

**Technologies**:
1. **Stereo vision**: Two cameras triangulate depth (like human eyes)
2. **Structured light**: Project pattern, measure distortion (Kinect)
3. **Time-of-flight (ToF)**: Measure light travel time (modern approach)

**Advantages over RGB**:
- Direct 3D reconstruction
- Robust object segmentation
- Accurate distance measurements

**Common models**:
- **Intel RealSense D435i**: Stereo + IMU, outdoor capable
- **Intel RealSense D455**: Longer range (up to 20m)
- **Azure Kinect**: ToF, high accuracy
- **Orbbec Astra**: Budget-friendly alternative

**Use cases**:
- VSLAM (Visual SLAM)
- Obstacle avoidance
- Grasping (estimating object 6D pose)
- Person tracking and gesture recognition

**Limitations**:
- **Range**: Typically 0.3-10m effective
- **Sunlight interference**: IR-based sensors struggle outdoors
- **Reflective/transparent surfaces**: Depth errors on glass, mirrors

### LIDAR (Light Detection and Ranging)

**What it measures**: Distance to objects using laser pulses

**How it works**:
1. Emit laser pulse
2. Measure time for reflection to return
3. Calculate distance: `d = (c × t) / 2` (c = speed of light)
4. Rotate laser to build 2D or 3D point cloud

**Types**:
- **2D LIDAR**: Scans in a plane (like a lighthouse beam)
- **3D LIDAR**: Full 3D point cloud (multiple scan lines)
- **Solid-state LIDAR**: No moving parts, more durable

**Advantages**:
- **Accurate**: mm-level precision
- **Long range**: 20-200m depending on model
- **Lighting independent**: Works in darkness
- **Fast**: High scan rates (10-100 Hz)

**Common models**:
- **Velodyne VLP-16**: 16-channel 3D LIDAR (~$4k)
- **RPLIDAR A1/A2**: Budget 2D LIDAR (~$100-300)
- **Ouster OS1/OS2**: High-resolution 3D LIDAR
- **Livox**: Affordable 3D LIDAR for drones/robots

**Use cases**:
- Mapping and localization (SLAM)
- Collision avoidance
- Autonomous navigation
- Terrain analysis

**Limitations**:
- **Cost**: High-quality 3D LIDAR is expensive ($1k-10k+)
- **No color information**: Point clouds lack texture
- **Glass/transparent**: Can't detect windows
- **Computational**: Processing millions of points per second

## Inertial Measurement Units (IMUs)

**What they measure**: Acceleration and rotational velocity

**Components**:
- **Accelerometer**: Linear acceleration (3 axes: x, y, z)
- **Gyroscope**: Angular velocity (rotation around 3 axes)
- **Magnetometer** (optional): Compass heading

**Use cases**:
- **Balance control**: Essential for bipedal walking
- **Orientation estimation**: Roll, pitch, yaw
- **Dead reckoning**: Estimate motion when GPS unavailable
- **Fall detection**: Sudden acceleration changes

**Common models**:
- **BNO055**: 9-axis IMU with sensor fusion (~$20)
- **MPU6050/MPU9250**: Budget 6/9-axis IMU (~$5-10)
- **LORD MicroStrain**: High-accuracy industrial IMU ($500+)
- **Integrated**: Many depth cameras (RealSense D435i) include IMU

**Key concepts**:
- **Sensor fusion**: Combine accelerometer + gyro + magnetometer
  - Accelerometer: Good for static orientation, noisy during motion
  - Gyroscope: Accurate short-term, drifts over time
  - Fusion: Complementary filter or Kalman filter for best estimate

**Challenges**:
- **Drift**: Integration errors accumulate over time
- **Vibration sensitivity**: Motors and motion create noise
- **Calibration**: Requires regular recalibration for accuracy

## Force and Torque Sensors

**What they measure**: Forces and moments applied to robot

**Types**:
- **6-axis F/T sensors**: 3 force components + 3 torque components
- **Single-axis load cells**: Measure force along one direction
- **Joint torque sensors**: Measure torque at each robot joint

**Use cases**:
- **Compliant control**: Adjust robot stiffness based on interaction forces
- **Collision detection**: Detect unexpected contact
- **Grasping**: Measure grip force to avoid crushing or dropping
- **Force control**: Apply specific forces (polishing, assembly)

**Common models**:
- **ATI Industrial Automation**: Industry standard F/T sensors ($2k-5k)
- **OptoForce**: Compact force sensors for grippers
- **Tactile arrays**: Multi-point force sensing (robotic skin)

**Applications in humanoids**:
- **Foot contact**: Detect ground contact for balance
- **Hand grasping**: Adaptive grip control
- **Collision safety**: Stop if robot hits human/obstacle

## Tactile Sensors

**What they measure**: Contact location, force distribution, texture

**Technologies**:
- **Resistive**: Pressure changes resistance
- **Capacitive**: Pressure changes capacitance
- **Optical**: Pressure deforms material, detected via camera
- **Piezoresistive**: Pressure changes voltage

**State of the art**:
- **GelSight**: High-resolution tactile sensing using camera
- **BioTac**: Fingertip sensor mimicking human touch
- **ReSkin**: Meta's flexible magnetic-based skin

**Use cases**:
- **Texture recognition**: Identify materials by touch
- **Slip detection**: Prevent objects from dropping
- **Grasping**: Fine force control for delicate objects
- **Dexterous manipulation**: Fine motor tasks

**Challenges**:
- **Durability**: Tactile surfaces wear out with use
- **Cost**: High-resolution tactile arrays are expensive
- **Integration**: Adding sensors without bulk
- **Data volume**: Dense sensor arrays generate large data streams

## Proprioceptive Sensors (Internal State)

**What they measure**: Robot's own configuration and state

### Joint Encoders
- **Position encoders**: Measure joint angles (absolute or incremental)
- **Velocity sensors**: Measure joint rotation speed
- **Current sensors**: Measure motor current (proxy for torque)

**Use for**:
- **Forward kinematics**: Calculate end-effector pose from joint angles
- **Feedback control**: Close the loop for position control
- **Safety monitoring**: Detect joint limit violations

### Temperature Sensors
- Monitor motor and battery temperature
- Prevent overheating and damage
- Throttle performance if needed

### Battery Monitoring
- Voltage, current, state of charge
- Critical for mobile robots with limited runtime
- Plan tasks based on remaining energy

## Audio Sensors (Microphones)

**What they measure**: Sound waves and audio signals

**Use cases**:
- **Voice commands**: "Pick up the red cup"
- **Sound localization**: Detect direction of sound source
- **Human detection**: Identify people by voice
- **Environmental awareness**: Detect alarms, machinery

**Types**:
- **Single microphone**: Basic audio capture
- **Microphone arrays**: Beamforming for directional audio
  - **ReSpeaker USB Mic Array**: 4-6 mics, far-field voice (~$70)
  - **MATRIX Voice**: 7-mic array for voice AI

**Processing**:
- **Speech-to-text**: OpenAI Whisper, Google Speech API
- **Speaker identification**: Voice embeddings
- **Noise cancellation**: Filter background sounds

## Sensor Fusion

**Why fusion?**: Each sensor has strengths and weaknesses

### Example: Localization

- **LIDAR**: Accurate geometry, no color, expensive
- **Camera**: Rich semantics, lighting-dependent, 2D
- **IMU**: High-rate orientation, drifts over time
- **Wheel encoders**: Direct motion, accumulates error

**Fused solution**: Visual-Inertial Odometry (VIO)
- Camera + IMU for robust, accurate localization
- Camera provides visual features
- IMU fills in between frames, aids scale estimation

### Sensor Fusion Algorithms

**Kalman Filter**:
- Optimal for linear systems with Gaussian noise
- Predicts state, corrects with sensor measurements
- Widely used for sensor fusion

**Particle Filter**:
- Handles non-linear, non-Gaussian systems
- Represents belief as particles (hypotheses)
- Good for localization in known maps

**Extended/Unscented Kalman Filter**:
- Extend Kalman filter to non-linear systems
- Used in VSLAM and IMU integration

## Sensor Requirements for Humanoid Robots

Based on the course hardware kit:

### Minimum Sensor Suite

| Sensor | Purpose | Example Model |
|--------|---------|---------------|
| **RGB-D Camera** | Vision + depth | Intel RealSense D435i |
| **IMU** | Balance, orientation | BNO055 (or built into camera) |
| **Microphone Array** | Voice commands | ReSpeaker USB Mic Array |
| **Joint Encoders** | Proprioception | Built into motors/servos |
| **Force Sensors** | Foot contact (optional) | Load cells for feet |

### Advanced Sensor Suite (Production Humanoid)

Additional sensors for robustness:
- **3D LIDAR**: Long-range obstacle detection
- **Stereo cameras**: Redundant vision, better depth
- **Tactile arrays**: Fingertips and hands
- **Joint torque sensors**: Force control and compliance
- **GPS** (outdoor): Absolute localization

## Course Coverage

This course emphasizes sensors through practical use:

### Module 1 (ROS 2)
- Publishing sensor data as ROS 2 topics
- Subscribing to sensor streams
- Simulating sensors in URDF

### Module 2 (Gazebo/Unity)
- Simulating LIDAR, cameras, IMUs
- Sensor plugins and configuration
- Noise models for realistic simulation

### Module 3 (NVIDIA Isaac)
- Isaac ROS: Hardware-accelerated sensor processing
- VSLAM using RGB-D cameras
- Sensor fusion for navigation

### Module 4 (VLA)
- Vision: Object detection and pose estimation
- Audio: Speech-to-text with Whisper
- Multimodal fusion for cognitive planning

## Next Steps

With a solid understanding of Physical AI foundations, embodied intelligence, the humanoid landscape, and sensor systems, you're ready to dive into the technical modules.

👉 **[Next: Module 1 - The Robotic Nervous System (ROS 2) →](../module1-ros2/ros2-overview)**

---

:::tip Sensor Selection Tip

When choosing sensors for a robotics project:
1. **Start simple**: RGB camera + IMU covers many use cases
2. **Add as needed**: Depth, LIDAR, tactile for specific requirements
3. **Consider tradeoffs**: Cost, power, computational load, reliability
4. **Redundancy**: Critical systems (e.g., autonomous vehicles) need backup sensors
5. **Simulation first**: Test sensor configurations in simulation before buying hardware

:::

## Further Reading

- **Books**:
  - "Probabilistic Robotics" by Thrun, Burgard, Fox (sensor fusion)
  - "Introduction to Autonomous Mobile Robots" by Siegwart, Nourbakhsh

- **Datasheets**:
  - Intel RealSense: https://www.intelrealsense.com/
  - Velodyne LIDAR: https://velodynelidar.com/
  - BNO055 IMU: Bosch Sensortec documentation

- **ROS 2 Sensor Drivers**:
  - `realsense-ros`: RealSense camera driver
  - `rplidar_ros`: RPLIDAR driver
  - `imu_tools`: IMU processing utilities
