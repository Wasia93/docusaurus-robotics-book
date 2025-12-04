---
sidebar_position: 1
title: NVIDIA Isaac Sim
description: Photorealistic simulation and synthetic data generation
keywords: [Isaac Sim, NVIDIA, Omniverse, photorealistic simulation, synthetic data]
---

# Module 3: The AI-Robot Brain - NVIDIA Isaac Sim

## Introduction to Isaac Sim

**NVIDIA Isaac Sim** is a photorealistic robot simulation platform built on NVIDIA Omniverse:

- **Photorealistic rendering**: RTX ray tracing for realistic visuals
- **Accurate physics**: PhysX 5 engine for dynamics and contacts
- **Synthetic data generation**: Train AI models with simulated data
- **ROS 2 integration**: Isaac ROS bridges for seamless communication
- **Scalable**: Run multiple simulations in parallel for RL training

### Why Isaac Sim for Humanoids?

1. **Sim-to-Real Transfer**: Realistic visuals help models generalize to real world
2. **Domain Randomization**: Vary lighting, textures, physics for robustness
3. **Hardware Acceleration**: GPU-accelerated perception (VSLAM, object detection)
4. **Synthetic Data**: Generate millions of labeled images for training
5. **Production Ready**: Used by Tesla, Boston Dynamics, industry leaders

## Isaac Sim vs. Gazebo

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Graphics** | Basic (OpenGL) | Photorealistic (RTX) |
| **Physics** | ODE/Bullet/DART | PhysX 5 (GPU-accelerated) |
| **AI Integration** | Manual | Built-in (Isaac ROS, Replicator) |
| **Performance** | CPU-bound | GPU-accelerated |
| **Synthetic Data** | Limited | Advanced (Replicator) |
| **Cost** | Free, open-source | Free, but needs RTX GPU |
| **Learning Curve** | Moderate | Steep |

**When to use Isaac Sim**:
- Training vision models (need realistic images)
- Sim-to-real transfer (deploy to physical robot)
- Reinforcement learning (need fast, parallel sims)
- Production humanoids (industry-grade simulation)

**When to use Gazebo**:
- Learning ROS 2 basics
- Testing control algorithms (graphics don't matter)
- Limited hardware (no RTX GPU)
- Open-source requirement

## System Requirements

### Minimum

- **GPU**: NVIDIA RTX 3070 (8GB VRAM)
- **CPU**: Intel i7 or AMD Ryzen 7
- **RAM**: 32GB
- **OS**: Ubuntu 22.04 or Windows 10/11
- **Storage**: 50GB free

### Recommended

- **GPU**: NVIDIA RTX 4080/4090 (16-24GB VRAM)
- **CPU**: Intel i9 or AMD Ryzen 9
- **RAM**: 64GB
- **OS**: Ubuntu 22.04 (best ROS 2 support)

**Critical**: You MUST have an NVIDIA RTX GPU (ray tracing cores required).

## Installation

### Step 1: Install Omniverse Launcher

1. Go to: https://www.nvidia.com/en-us/omniverse/
2. Download Omniverse Launcher for your OS
3. Install and create NVIDIA account (free)

### Step 2: Install Isaac Sim via Launcher

```bash
# In Omniverse Launcher:
# 1. Navigate to "Exchange" tab
# 2. Search for "Isaac Sim"
# 3. Click "Install" (downloads ~20GB)
# 4. Wait for installation to complete
```

**Version**: Install **Isaac Sim 2023.1.1** or latest stable release.

### Step 3: Verify Installation

Launch Isaac Sim:
```bash
# From Omniverse Launcher, click "Launch" on Isaac Sim

# Or via command line (Linux):
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Windows:
C:\Users\<username>\AppData\Local\ov\pkg\isaac_sim-2023.1.1\isaac-sim.bat
```

You should see the Isaac Sim interface with a viewport.

### Step 4: Install Isaac ROS (For ROS 2 Integration)

```bash
# On Ubuntu 22.04 with ROS 2 Humble
sudo apt update
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-image-proc
sudo apt install ros-humble-isaac-ros-nvblox

# Verify installation
ros2 pkg list | grep isaac
```

## Isaac Sim Interface

### Main Components

1. **Viewport**: 3D scene view (rendered with RTX)
2. **Stage**: Hierarchy of objects in scene (like scene graph)
3. **Content Browser**: Assets, materials, robots
4. **Property Panel**: Object properties, transforms, physics
5. **Timeline**: Animation and simulation playback controls

### Navigation

- **Left mouse**: Rotate camera
- **Middle mouse**: Pan
- **Right mouse**: Zoom
- **F**: Focus on selected object
- **Shift + Left**: First-person camera

## Creating a Scene

### Method 1: Using GUI

```
1. File → New Stage
2. Create → Mesh → Ground Plane
3. Create → Mesh → Cube (add obstacles)
4. Create → Light → Dome Light (realistic lighting)
5. Click "Play" button to start simulation
```

### Method 2: Using Python API (Programmatic)

```python
from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim (headless optional)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
import numpy as np

# Create world
world = World()

# Add ground plane
world.scene.add_default_ground_plane()

# Add a cube obstacle
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="obstacle",
        position=np.array([2.0, 0.0, 0.5]),
        size=np.array([1.0, 1.0, 1.0]),
        color=np.array([1.0, 0.0, 0.0]),  # Red
    )
)

# Reset world
world.reset()

# Run simulation loop
for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

## Loading a Humanoid Robot

### Using USD Assets

**USD (Universal Scene Description)**: File format for 3D scenes.

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Create world
world = World()
world.scene.add_default_ground_plane()

# Get Isaac assets path
assets_root_path = get_assets_root_path()
robot_usd_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"

# Load robot
robot = world.scene.add(
    Robot(
        prim_path="/World/Humanoid",
        name="my_humanoid",
        usd_path=robot_usd_path,
        position=[0, 0, 1.0],
    )
)

# Reset and run
world.reset()
for i in range(1000):
    world.step(render=True)
```

### Converting URDF to USD

```bash
# Isaac Sim includes URDF importer
# In Isaac Sim GUI:
# Isaac Utils → URDF Importer
# Select your URDF file → Import

# Or via Python:
from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()
urdf_interface.parse_urdf(
    urdf_path="/path/to/robot.urdf",
    import_config=_urdf.ImportConfig(),
    dest_path="/World/Humanoid"
)
```

## ROS 2 Integration

### ROS 2 Bridge

Isaac Sim has built-in ROS 2 bridge:

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import omni.isaac.ros2_bridge as ros2_bridge

# Create world
world = World()
world.scene.add_default_ground_plane()

# Enable ROS 2 bridge
simulation_app.update()

# Add ROS 2 camera
from omni.isaac.ros2_bridge import Camera
camera_prim = world.stage.GetPrimAtPath("/World/Camera")
camera = Camera(
    prim_path="/World/Camera",
    ros_topic_name="/camera/image_raw",
    frame_id="camera_link",
)

# Simulation loop
world.reset()
while simulation_app.is_running():
    world.step(render=True)
    # ROS 2 messages published automatically

simulation_app.close()
```

**Check ROS 2 topics**:
```bash
# In another terminal
ros2 topic list
# Should see: /camera/image_raw, /clock, etc.

ros2 topic echo /camera/image_raw
```

## Synthetic Data Generation

### Isaac Sim Replicator

**Replicator**: Tool for generating synthetic datasets.

```python
import omni.replicator.core as rep

# Create randomized scene
def create_scene():
    # Random lights
    lights = rep.create.light(
        light_type="Dome",
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        intensity=rep.distribution.uniform(500, 1500),
    )

    # Random objects
    shapes = rep.create.from_usd(
        usd="/path/to/objects.usd",
        semantics=[("class", "object")],
        count=10,
    )

    # Randomize positions
    with shapes:
        rep.modify.pose(
            position=rep.distribution.uniform((-5, -5, 0), (5, 5, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        )

    # Camera
    camera = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
    return camera

# Register scene
rep.randomizer.register(create_scene)

# Capture images
with rep.trigger.on_frame(num_frames=1000):
    rep.randomizer.create_scene()

# Write output
writer = rep.BasicWriter(
    output_dir="/tmp/synthetic_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True,
)

rep.orchestrator.run()
```

**Output**: 1000 images with randomized scenes + labels (bounding boxes, segmentation masks).

## Domain Randomization

**Key for sim-to-real**: Train models on diverse simulated data.

### Randomize Lighting

```python
import omni.replicator.core as rep

# Random dome light
with rep.create.light(light_type="Dome"):
    rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
    rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1)))
```

### Randomize Textures

```python
# Randomize materials on objects
objects = rep.get.prims(path_pattern="/World/Objects/*")
with objects:
    rep.randomizer.materials(
        materials=rep.get.prims(path_pattern="/Materials/*"),
    )
```

### Randomize Physics

```python
# Randomize mass
with objects:
    rep.modify.attribute("physics:mass", rep.distribution.uniform(0.5, 2.0))

# Randomize friction
with objects:
    rep.modify.attribute(
        "physics:dynamicFriction",
        rep.distribution.uniform(0.2, 1.0)
    )
```

## Performance Optimization

### GPU Utilization

```bash
# Monitor GPU usage
nvidia-smi -l 1

# Should see high GPU utilization (80-100%) during rendering
```

### Running Headless (No GUI)

For faster training:

```python
simulation_app = SimulationApp({"headless": True})
# 2-5x faster without rendering GUI
```

### Multi-GPU Support

Distribute simulation across GPUs:

```python
# Coming soon in Isaac Sim (experimental)
# Multi-GPU rendering and physics
```

## Next Steps

You've learned Isaac Sim basics. Next, explore Isaac ROS for hardware-accelerated perception.

👉 **[Next: Isaac ROS →](isaac-ros)**

---

:::tip Isaac Sim Tips

1. **Start with examples**: Isaac Sim includes many example scenes and scripts
2. **Use Python API**: Automate repetitive tasks, generate datasets
3. **Domain randomization**: Essential for sim-to-real transfer
4. **Monitor GPU**: Keep GPU utilization high for best performance
5. **Version control USD files**: Use Git LFS for large assets

:::

## Lab Exercise

**Create a humanoid training environment**:

1. **Scene setup**:
   - Ground plane
   - 5-10 random obstacles
   - Dome light with randomization

2. **Load humanoid**:
   - Import from Isaac assets or convert your URDF

3. **Add sensors**:
   - Camera in head (ROS 2 topic)
   - IMU in torso

4. **Domain randomization**:
   - Randomize lighting (intensity, color)
   - Randomize obstacle positions

5. **Generate data**:
   - Capture 100 images with Replicator
   - Save RGB + depth + segmentation masks

**Bonus**: Train a simple object detector on the synthetic data.
