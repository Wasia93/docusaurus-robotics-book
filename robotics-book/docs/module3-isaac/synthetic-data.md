# Synthetic Data Generation

## Overview

Synthetic data generation is a cornerstone of modern AI robotics, enabling robots to train in diverse, photorealistic environments without the cost and risk of real-world data collection.

## NVIDIA Isaac Sim for Synthetic Data

### What is Synthetic Data?

Synthetic data is artificially generated information that mimics real-world data characteristics. In robotics, this includes:

- **Visual data**: RGB images, depth maps, segmentation masks
- **Sensor data**: LiDAR point clouds, IMU readings, force/torque measurements
- **Annotations**: Bounding boxes, pose estimations, semantic labels

### Why Synthetic Data Matters

1. **Cost-Effective**: No need for expensive real-world data collection
2. **Scalable**: Generate millions of training samples programmatically
3. **Safe**: Train dangerous scenarios without physical risk
4. **Diverse**: Control lighting, weather, object placement, and scenarios
5. **Perfect Labels**: Automatic ground truth annotations

## Isaac Sim Domain Randomization

### Appearance Randomization

```python
import omni.replicator.core as rep

# Randomize lighting
with rep.trigger.on_frame():
    rep.randomizer.color(
        lights=rep.get.light(),
        colors=rep.distribution.uniform((0.5, 0.5, 0.5), (1, 1, 1))
    )

# Randomize textures
with rep.trigger.on_frame():
    rep.randomizer.materials(
        objects=rep.get.prims(semantics=[("class", "object")]),
        materials=rep.distribution.choice([
            "metal", "plastic", "wood", "fabric"
        ])
    )
```

### Physics Randomization

```python
# Randomize object properties
with rep.trigger.on_frame():
    rep.randomizer.physics_material(
        objects=rep.get.prims(semantics=[("class", "object")]),
        friction=rep.distribution.uniform(0.1, 1.0),
        restitution=rep.distribution.uniform(0.0, 0.8)
    )
```

## Data Collection Pipeline

### Setting Up Replicator

```python
import omni.replicator.core as rep

# Initialize camera
camera = rep.create.camera(position=(3, 3, 3), look_at=(0, 0, 0))

# Render products
render_product = rep.create.render_product(camera, (1280, 720))

# Writers for different data types
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(
    output_dir="data/rgb",
    rgb=True
)

depth_writer = rep.WriterRegistry.get("BasicWriter")
depth_writer.initialize(
    output_dir="data/depth",
    distance_to_camera=True
)

semantic_writer = rep.WriterRegistry.get("BasicWriter")
semantic_writer.initialize(
    output_dir="data/semantic",
    semantic_segmentation=True
)
```

### Generating Training Data

```python
# Run data generation
with rep.trigger.on_frame(num_frames=10000):
    # Randomize scene
    with rep.create.group([rep.get.prims(semantics=[("class", "object")])]):
        rep.modify.pose(
            position=rep.distribution.uniform((-2, -2, 0), (2, 2, 2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Capture data
    rgb_writer.attach([render_product])
    depth_writer.attach([render_product])
    semantic_writer.attach([render_product])

# Execute
rep.orchestrator.run()
```

## Sim-to-Real Transfer

### Domain Gap Challenges

The "reality gap" between simulation and real world includes:

- **Rendering differences**: Simulated vs. actual lighting and materials
- **Physics inaccuracies**: Simplified collision and dynamics
- **Sensor noise**: Perfect simulated sensors vs. noisy real sensors
- **Calibration**: Camera parameters and sensor alignment

### Bridging the Gap

#### 1. Diverse Training Data

Generate data across wide variation ranges to improve generalization:

```python
# Wide variation in parameters
lighting_intensity = rep.distribution.uniform(50, 5000)  # lux
camera_noise = rep.distribution.uniform(0, 0.05)  # noise level
```

#### 2. Domain Randomization

Randomize appearance to force the model to learn invariant features:

- Texture randomization
- Color jittering
- Lighting variation
- Background changes

#### 3. Sensor Simulation

Add realistic sensor noise and artifacts:

```python
# Add camera noise
with rep.trigger.on_frame():
    rep.randomizer.texture(
        cameras=camera,
        texture_type="noise",
        amplitude=rep.distribution.uniform(0, 0.02)
    )
```

## Practical Exercises

### Exercise 1: Basic Data Generation

Create a simple scene and generate 1000 RGB-D image pairs:

```python
# TODO: Student implementation
# 1. Create a scene with a table and objects
# 2. Set up RGB and depth cameras
# 3. Add basic randomization
# 4. Generate 1000 frames
```

### Exercise 2: Object Detection Dataset

Generate a dataset for training an object detection model:

```python
# TODO: Student implementation
# 1. Place multiple objects in scene
# 2. Randomize positions, rotations, and lighting
# 3. Capture RGB images and bounding box annotations
# 4. Export in COCO format
```

### Exercise 3: Sim-to-Real Validation

Test your synthetic data pipeline:

```python
# TODO: Student implementation
# 1. Generate synthetic training data
# 2. Train a simple perception model
# 3. Test on real-world images
# 4. Measure and document the domain gap
```

## Key Takeaways

- Synthetic data enables scalable, safe, and cost-effective training
- Domain randomization is essential for sim-to-real transfer
- NVIDIA Isaac Sim provides powerful tools for data generation
- Always validate synthetic data with real-world testing

## Resources

- [NVIDIA Isaac Sim Replicator Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)
- [Domain Randomization Research Papers](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer Best Practices](https://developer.nvidia.com/blog/simulating-robotics-with-isaac-sim/)

## Next Steps

Continue to [Isaac ROS](./isaac-ros.md) to learn about hardware-accelerated perception on edge devices.
