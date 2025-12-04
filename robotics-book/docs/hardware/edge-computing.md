---
sidebar_position: 2
title: Edge Computing Kits
description: NVIDIA Jetson and edge AI hardware for physical robot deployment
keywords: [Jetson, edge computing, NVIDIA Orin, embedded AI, robotics hardware]
---

# Edge Computing Kits for Physical AI

## The "Brain" of Your Robot

While your workstation runs **simulations and training**, the **edge computing kit** is the actual brain deployed on the physical robot.

### Why Edge Computing?

**Edge AI** means running AI models directly on the robot, not in the cloud:

**Advantages**:
- ✅ **Low latency**: No network delay (critical for control loops)
- ✅ **Reliability**: Works without internet connection
- ✅ **Privacy**: Sensor data stays on device
- ✅ **Scalability**: Fleet doesn't overload cloud servers

**Challenges**:
- ⚠️ **Limited compute**: Can't run models as large as cloud/workstation
- ⚠️ **Power constraints**: Battery-powered robots need efficiency
- ⚠️ **Thermal management**: Small form factor = heat issues

## The NVIDIA Jetson Platform

**NVIDIA Jetson** is the industry standard for edge AI in robotics.

### Why Jetson for This Course?

1. **ROS 2 support**: Native Ubuntu 20.04/22.04
2. **Isaac ROS**: Hardware-accelerated perception (VSLAM, object detection)
3. **CUDA cores**: GPU acceleration for neural networks
4. **Tensor Cores**: Optimized for deep learning inference
5. **Ecosystem**: Large community, extensive tutorials

### Jetson Product Line (2025)

| Model | TOPS | VRAM | Power | Price | Use Case |
|-------|------|------|-------|-------|----------|
| **Orin Nano Super** | 40-67 | 8GB | 15W | $249 | **Recommended for course** |
| **Orin NX** | 100 | 16GB | 25W | $599 | Advanced projects, larger models |
| **Orin AGX** | 275 | 64GB | 60W | $1,999 | Production humanoids, research |
| **Xavier NX** | 21 | 8GB | 15W | $399 | Older gen, still capable |
| **Nano** (legacy) | 0.5 | 4GB | 10W | $99 | Too weak for this course |

**TOPS**: Tera Operations Per Second (AI performance metric)

### Recommended: Jetson Orin Nano Super Dev Kit

**Specs**:
- **GPU**: 1024 CUDA cores, 32 Tensor Cores
- **CPU**: 6-core Arm® Cortex®-A78AE
- **AI Performance**: 40 TOPS (INT8), 67 TOPS (sparse)
- **Memory**: 8GB LPDDR5
- **Storage**: microSD card (up to 1TB)
- **Connectivity**: WiFi 6, Bluetooth 5.2, Gigabit Ethernet
- **I/O**: 4x USB 3.2, HDMI 2.1, DisplayPort, GPIO

**Price**: $249 (official MSRP, dropped from ~$499)

**Why this model?**:
- **Newest**: Released 2024, best performance per dollar
- **Sufficient**: Can run VLA models (with optimization)
- **Student-friendly**: Affordable compared to alternatives
- **Supported**: Isaac ROS fully compatible

## The Complete Economy Jetson Student Kit

**Total cost**: ~$700

| Component | Model | Price | Notes |
|-----------|-------|-------|-------|
| **The Brain** | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 | 40 TOPS AI performance |
| **The Eyes** | Intel RealSense D435i | $349 | Includes IMU, essential for SLAM |
| **The Ears** | ReSpeaker USB Mic Array v2.0 | $69 | Far-field microphone for voice |
| **Wi-Fi** | Included in Dev Kit | $0 | Pre-installed in "Super" kit |
| **Power/Misc** | SD Card (128GB) + Jumper Wires | $30 | High-endurance microSD for OS |
| **TOTAL** | | **~$700** | Complete edge AI kit |

### Component Breakdown

#### 1. The Brain: Jetson Orin Nano Super

**What it does**:
- Runs ROS 2 nodes (perception, planning, control)
- Executes neural networks (object detection, VLA models)
- Interfaces with sensors (camera, IMU, LIDAR)
- Sends motor commands to actuators

**Software stack**:
```
┌─────────────────────────────────┐
│   ROS 2 Nodes (Python/C++)      │
├─────────────────────────────────┤
│   Isaac ROS (VSLAM, Nav2)       │
├─────────────────────────────────┤
│   CUDA/TensorRT (Inference)     │
├─────────────────────────────────┤
│   Ubuntu 22.04 (JetPack 6.0)    │
└─────────────────────────────────┘
```

#### 2. The Eyes: Intel RealSense D435i

**Why D435i specifically?**:
- **Depth camera**: Stereo vision for 3D sensing (0.3-10m range)
- **IMU included**: BNO055 9-axis IMU for orientation
- **Outdoor capable**: Infrared-based, works in sunlight (better than Kinect)
- **ROS 2 support**: Official `realsense-ros` package
- **Wide FOV**: 87° × 58° field of view

**Critical**: Do NOT buy the D435 (non-i). The "i" model includes the IMU, which is **essential** for Visual-Inertial Odometry (VIO) in Module 3.

**Alternatives**:
- **Intel RealSense D455**: Longer range (up to 20m), $399
- **Orbbec Astra**: Budget option, ~$150, less capable
- **Oak-D**: Onboard AI processing, $249, experimental ROS 2 support

#### 3. The Ears: ReSpeaker USB Mic Array

**What it does**:
- **Far-field microphone**: Capture voice from 3-5 meters away
- **Beamforming**: Focus on speech direction, reject noise
- **4-6 microphones**: Array for spatial audio
- **USB interface**: Plug-and-play with Jetson

**Use cases**:
- Voice commands: "Pick up the red cup"
- Speaker localization: Turn toward speaking person
- Sound event detection: Alarms, door knocks

**Alternatives**:
- **MATRIX Voice**: 7-mic array, $99, more features
- **Simple USB mic**: Cheaper, but no beamforming ($20-30)

#### 4. Storage: High-Endurance microSD Card

**Why high-endurance?**:
- Jetson boots from microSD (no onboard storage)
- ROS 2 + Isaac ROS + models = 30-50GB
- **Write endurance**: Standard cards fail quickly with OS writes

**Recommended**:
- **Samsung PRO Endurance** 128GB (~$20)
- **SanDisk MAX Endurance** 128GB (~$25)

**Avoid**: Generic/cheap microSD cards (will corrupt)

## Setup and Installation

### 1. Flash JetPack OS

**JetPack** = NVIDIA's Ubuntu-based OS with CUDA, TensorRT, Isaac ROS

**Steps**:
1. Download **NVIDIA SDK Manager** (from NVIDIA developer site)
2. Flash JetPack 6.0 (Ubuntu 22.04) to microSD
3. Boot Jetson from microSD
4. Complete Ubuntu setup wizard

**Alternatively**: Use pre-flashed image:
```bash
# Download JetPack 6.0 image
# Use Balena Etcher to flash microSD card
# Boot Jetson
```

### 2. Install ROS 2 Humble

```bash
# Set up sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Base (no GUI tools to save space)
sudo apt update
sudo apt install ros-humble-base

# Source setup (add to ~/.bashrc)
source /opt/ros/humble/setup.bash
```

### 3. Install RealSense SDK and ROS Wrapper

```bash
# RealSense SDK
sudo apt install librealsense2-utils librealsense2-dev

# ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description

# Test camera
realsense-viewer  # Verify camera works
```

### 4. Install Isaac ROS

```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-image-proc

# Verify installation
ros2 pkg list | grep isaac
```

## Performance Optimization

### Model Quantization

**Problem**: Large models (VLA, LLMs) won't fit in 8GB VRAM

**Solution**: Quantization (reduce precision)
- **FP32** (full precision): 4 bytes per weight
- **FP16** (half precision): 2 bytes per weight
- **INT8** (quantized): 1 byte per weight

**Example**: GPT-2 model
- FP32: 2GB VRAM
- INT8: 500MB VRAM (4x smaller, ~2-5% accuracy loss)

**Tools**:
- **TensorRT**: NVIDIA's inference optimizer
- **ONNX Runtime**: Cross-platform inference engine

### Power Modes

Jetson has configurable power modes:

```bash
# List available modes
sudo nvpmodel -q

# Set to maximum performance (25W)
sudo nvpmodel -m 0

# Set to power-saving mode (10W)
sudo nvpmodel -m 1
```

**Trade-off**: Performance vs. battery life (for mobile robots)

### Thermal Management

**Jetson heats up** under load. Add active cooling:

**Options**:
- **Noctua NF-A4x10 5V** fan (~$15): Quiet, effective
- **Heatsink upgrade**: Better passive cooling
- **Case with fan**: Official Jetson case options

## Course Usage: Simulation vs. Edge Deployment

### Week 1-10: Simulation on Workstation

- Develop and test in Isaac Sim / Gazebo
- Train models on RTX GPU workstation
- No Jetson needed yet

### Week 11-13: Edge Deployment

1. **Export trained model** from workstation
2. **Optimize for Jetson** (quantize with TensorRT)
3. **Deploy to Jetson** (copy model weights)
4. **Test on real hardware** (if available) or continue simulation

**Workflow**:
```
Workstation (Train) → Export → Jetson (Deploy) → Robot (Execute)
```

## Connecting Jetson to Robot Hardware

### Option 1: Desktop Deployment (No Robot)

**Setup**:
- Jetson on desk with power supply
- RealSense camera connected via USB
- ReSpeaker mic connected via USB
- Monitor, keyboard, mouse (via HDMI/USB)

**Use case**: Learn ROS 2, VSLAM, perception without physical robot

### Option 2: Jetson on Quadruped (Proxy)

**Example**: Unitree Go2
- Mount Jetson on robot chassis
- RealSense camera on front
- Jetson sends motor commands via serial/Ethernet

### Option 3: Jetson on Humanoid

**Example**: Unitree G1, custom humanoid
- Jetson as main controller
- Interfaces with motor controllers (CAN bus, serial)
- Real-time control loop (100-1000 Hz)

## Next Steps

With edge computing covered, explore the robot lab options for physical testing.

👉 **[Next: Robot Lab Options →](robot-lab-options)**

---

:::tip Jetson Development Tip

**Develop on workstation first**:
1. Write and test ROS 2 nodes on your powerful workstation
2. Use `docker` or `rsync` to deploy to Jetson
3. Test on Jetson only when ready

This saves time—Jetson compilation is slower than workstation.

:::

:::warning 8GB VRAM Limitation

The Jetson Orin Nano has only 8GB shared between GPU and system. Large models (LLaMA, large VLAs) won't fit without aggressive quantization. For production humanoids, consider Orin NX (16GB) or AGX (64GB).

:::

## Further Reading

- **NVIDIA Jetson Documentation**: https://developer.nvidia.com/embedded/jetson
- **JetPack SDK**: https://developer.nvidia.com/embedded/jetpack
- **Isaac ROS**: https://nvidia-isaac-ros.github.io/
- **RealSense with ROS 2**: https://github.com/IntelRealSense/realsense-ros
- **Jetson Community**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/

## Lab Exercise

**Set up your Jetson kit**:
1. Flash JetPack 6.0 to microSD card
2. Install ROS 2 Humble
3. Connect RealSense D435i and verify with `realsense-viewer`
4. Run Isaac ROS Visual SLAM demo
5. Monitor resource usage: `tegrastats` (Jetson monitoring tool)

**Bonus**: SSH into Jetson from your workstation for headless operation.
