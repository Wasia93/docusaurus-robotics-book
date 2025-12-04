# Assessment: Isaac-Based Perception Pipeline

## Overview

This assessment evaluates your ability to develop a complete perception pipeline using NVIDIA Isaac platform. You'll implement object detection, pose estimation, and scene understanding for robotic manipulation tasks.

## Learning Objectives

By completing this assessment, you will demonstrate:

- Proficiency in Isaac Sim for synthetic data generation
- Understanding of Isaac ROS perception nodes
- Ability to train and deploy vision models
- Knowledge of 3D perception (depth, point clouds, pose estimation)
- Integration of perception with robotic control

## Project Requirements

### Task: Object Detection and Pose Estimation for Bin Picking

Create a complete perception system that:
1. Detects objects in a cluttered bin
2. Estimates 6D pose (position + orientation)
3. Segments individual objects from point cloud
4. Publishes detections to ROS 2 for grasp planning

#### 1. Environment Setup

##### Isaac Sim Scene Creation

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add bin
add_reference_to_stage(
    usd_path="/Isaac/Props/Bins/bin_0.usd",
    prim_path="/World/Bin"
)

# Populate bin with objects
objects = []
for i in range(15):
    obj_type = np.random.choice(['cube', 'sphere', 'cylinder'])

    if obj_type == 'cube':
        obj = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Object_{i}",
                name=f"object_{i}",
                position=[np.random.uniform(-0.2, 0.2),
                         np.random.uniform(-0.2, 0.2),
                         0.5 + i * 0.15],
                size=0.08,
                color=np.random.rand(3)
            )
        )
    elif obj_type == 'sphere':
        obj = world.scene.add(
            DynamicSphere(
                prim_path=f"/World/Object_{i}",
                name=f"object_{i}",
                position=[np.random.uniform(-0.2, 0.2),
                         np.random.uniform(-0.2, 0.2),
                         0.5 + i * 0.15],
                radius=0.04,
                color=np.random.rand(3)
            )
        )
    # Add cylinder similarly

    objects.append(obj)

# Add camera
from omni.isaac.sensor import Camera
camera = world.scene.add(
    Camera(
        prim_path="/World/Camera",
        position=[0, -0.5, 0.8],
        orientation=[0.7071, 0, 0.7071, 0],  # Looking at bin
        resolution=(1280, 720),
        frequency=30
    )
)
```

#### 2. Synthetic Data Generation

##### Domain Randomization

```python
import omni.replicator.core as rep

# Setup camera
camera_rep = rep.create.camera(
    position=(0, -0.5, 0.8),
    look_at=(0, 0, 0.3)
)

render_product = rep.create.render_product(camera_rep, (1280, 720))

# Randomization
with rep.trigger.on_frame(num_frames=1000):
    # Randomize lighting
    with rep.create.light(
        light_type="Sphere",
        intensity=rep.distribution.uniform(1000, 5000),
        position=rep.distribution.uniform((-1, -1, 1), (1, 1, 2)),
        scale=0.1
    ) as light:
        pass

    # Randomize object positions
    with rep.get.prims(semantics=[("class", "object")]) as objects:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (-0.25, -0.25, 0.3),
                (0.25, 0.25, 0.8)
            ),
            rotation=rep.distribution.uniform(
                (0, 0, 0),
                (360, 360, 360)
            )
        )

    # Randomize textures
    with rep.get.prims(semantics=[("class", "object")]) as objects:
        rep.randomizer.color(
            colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1))
        )

# Writers
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

bbox_writer = rep.WriterRegistry.get("BasicWriter")
bbox_writer.initialize(
    output_dir="data/bbox",
    bounding_box_2d_tight=True
)

semantic_writer = rep.WriterRegistry.get("BasicWriter")
semantic_writer.initialize(
    output_dir="data/semantic",
    semantic_segmentation=True
)

# Attach writers
rgb_writer.attach([render_product])
depth_writer.attach([render_product])
bbox_writer.attach([render_product])
semantic_writer.attach([render_product])

# Run generation
rep.orchestrator.run()
```

#### 3. Object Detection Model Training

##### YOLOv8 Training

```python
from ultralytics import YOLO

# Prepare dataset in YOLO format
# data.yaml:
"""
train: data/train/images
val: data/val/images

nc: 3  # number of classes
names: ['cube', 'sphere', 'cylinder']
"""

# Train model
model = YOLO('yolov8n.pt')

results = model.train(
    data='data/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    name='bin_picking_detector'
)

# Validate
metrics = model.val()

# Export to ONNX for inference
model.export(format='onnx')
```

#### 4. ROS 2 Perception Node

##### Detection Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from geometry_msgs.msg import Pose, Point
import cv2
import numpy as np
from cv_bridge import CvBridge
import onnxruntime as ort

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Load model
        self.session = ort.InferenceSession('bin_detector.onnx')

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection3DArray, '/detections_3d', 10
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.latest_depth = None

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        """Store latest depth image"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def rgb_callback(self, msg):
        """Detect objects and estimate poses"""

        # Convert to OpenCV
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run detection
        detections_2d = self.detect_objects(rgb)

        # Estimate 3D poses
        if self.latest_depth is not None and self.camera_matrix is not None:
            detections_3d = self.estimate_3d_poses(
                detections_2d,
                self.latest_depth,
                self.camera_matrix
            )

            # Publish
            detection_msg = self.create_detection_msg(detections_3d)
            self.detection_pub.publish(detection_msg)

    def detect_objects(self, image):
        """Run YOLO detection"""

        # Preprocess
        input_img = cv2.resize(image, (640, 640))
        input_img = input_img.transpose(2, 0, 1)  # HWC to CHW
        input_img = input_img.astype(np.float32) / 255.0
        input_img = np.expand_dims(input_img, axis=0)

        # Inference
        outputs = self.session.run(None, {'images': input_img})

        # Post-process
        detections = self.post_process_yolo(outputs, image.shape[:2])

        return detections

    def estimate_3d_poses(self, detections_2d, depth_image, camera_matrix):
        """Estimate 3D pose from 2D detection and depth"""

        detections_3d = []

        for det in detections_2d:
            # Get center point
            cx = int(det['bbox'][0] + det['bbox'][2] / 2)
            cy = int(det['bbox'][1] + det['bbox'][3] / 2)

            # Get depth
            depth = depth_image[cy, cx]

            if depth > 0:
                # Backproject to 3D
                fx = camera_matrix[0, 0]
                fy = camera_matrix[1, 1]
                cx_cam = camera_matrix[0, 2]
                cy_cam = camera_matrix[1, 2]

                x = (cx - cx_cam) * depth / fx
                y = (cy - cy_cam) * depth / fy
                z = depth

                # Estimate orientation (simplified)
                orientation = self.estimate_orientation(
                    det, depth_image, camera_matrix
                )

                detections_3d.append({
                    'class': det['class'],
                    'confidence': det['confidence'],
                    'position': [x, y, z],
                    'orientation': orientation
                })

        return detections_3d

    def estimate_orientation(self, detection, depth_image, camera_matrix):
        """Estimate object orientation using PCA or similar"""

        # Extract object point cloud from bounding box
        bbox = detection['bbox']
        object_depth = depth_image[
            bbox[1]:bbox[1]+bbox[3],
            bbox[0]:bbox[0]+bbox[2]
        ]

        # Backproject to 3D points
        points_3d = self.depth_to_pointcloud(
            object_depth, bbox, camera_matrix
        )

        # Fit orientation using PCA
        if len(points_3d) > 10:
            # Center points
            centroid = np.mean(points_3d, axis=0)
            centered = points_3d - centroid

            # PCA
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eig(cov)

            # Largest eigenvector is main axis
            main_axis = eigenvectors[:, np.argmax(eigenvalues)]

            # Convert to quaternion
            orientation = self.axis_to_quaternion(main_axis)

            return orientation

        return [0, 0, 0, 1]  # Identity quaternion
```

#### 5. Point Cloud Processing

##### Segmentation and Pose Refinement

```python
import open3d as o3d

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')

        self.pcd_sub = self.create_subscription(
            PointCloud2, '/camera/points', self.pcd_callback, 10
        )

    def pcd_callback(self, msg):
        """Process point cloud for segmentation"""

        # Convert ROS PointCloud2 to Open3D
        pcd = self.ros_to_open3d(msg)

        # Remove outliers
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Plane segmentation (remove table)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )

        object_cloud = pcd.select_by_index(inliers, invert=True)

        # Cluster objects
        labels = np.array(
            object_cloud.cluster_dbscan(eps=0.02, min_points=10)
        )

        # Process each cluster
        for label in set(labels):
            if label == -1:
                continue

            # Extract cluster
            cluster_indices = np.where(labels == label)[0]
            cluster = object_cloud.select_by_index(cluster_indices)

            # Estimate pose using ICP or other methods
            pose = self.estimate_cluster_pose(cluster)

            # Publish detection
            # ...

    def estimate_cluster_pose(self, cluster):
        """Refine pose estimation for cluster"""

        # Compute oriented bounding box
        obb = cluster.get_oriented_bounding_box()

        position = obb.center
        orientation = obb.R  # Rotation matrix

        # Convert rotation matrix to quaternion
        quat = self.rotation_matrix_to_quaternion(orientation)

        return {
            'position': position,
            'orientation': quat
        }
```

## Testing Requirements

### 1. Detection Accuracy
- Measure precision, recall, F1 score
- Test on held-out validation set
- Minimum 90% detection rate for visible objects

### 2. Pose Estimation Accuracy
- Compare estimated vs. ground truth poses
- Position error < 5mm
- Orientation error < 10 degrees

### 3. Runtime Performance
- Detection + pose estimation < 100ms
- Maintain 10 Hz perception loop
- Monitor GPU/CPU usage

### 4. Robustness
- Test under varying lighting conditions
- Evaluate with occlusions (10-30% occluded)
- Test with novel object colors/textures

## Deliverables

### 1. Source Code
```
isaac_perception/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── perception.launch.py
├── config/
│   └── perception_params.yaml
├── scripts/
│   ├── generate_synthetic_data.py
│   ├── train_detector.py
│   └── evaluate_model.py
├── src/
│   ├── detection_node.cpp/py
│   └── pointcloud_processor.cpp/py
├── models/
│   └── bin_detector.onnx
└── README.md
```

### 2. Trained Model
- Trained detection model (ONNX format)
- Training logs and curves
- Model performance metrics

### 3. Dataset
- 1000+ synthetic training images
- Annotations (bounding boxes, segmentation masks)
- Data generation scripts

### 4. Demonstration
- Video showing:
  - Object detection in Isaac Sim
  - 3D pose visualization
  - ROS 2 integration
  - Grasp planning based on detections

### 5. Performance Report
- Detection metrics (precision/recall)
- Pose estimation accuracy
- Runtime benchmarks
- Failure case analysis

## Evaluation Criteria

| Criteria | Points | Description |
|----------|--------|-------------|
| **Synthetic Data** | 15 | Quality and diversity of generated data |
| **Detection Model** | 20 | Accuracy and robustness |
| **3D Pose Estimation** | 25 | Pose accuracy and reliability |
| **Point Cloud Processing** | 15 | Segmentation and refinement |
| **ROS 2 Integration** | 10 | Clean interface, proper topics |
| **Performance** | 10 | Real-time capable |
| **Documentation** | 5 | Clear documentation and code quality |

**Total: 100 points**

## Tips for Success

1. **Generate diverse data**: Vary lighting, object poses, backgrounds
2. **Validate incrementally**: Test 2D detection before 3D pose
3. **Use visualization**: Visualize detections and point clouds
4. **Benchmark performance**: Profile code to find bottlenecks
5. **Handle edge cases**: Occluded objects, poor lighting

## Resources

- [Isaac Sim Replicator](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/)
- [Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [Open3D Tutorial](http://www.open3d.org/docs/)

## Extension Challenges (Optional)

1. **6D pose estimation**: Use PVNet or similar for exact pose
2. **Real-world transfer**: Test on real camera data
3. **Multi-view fusion**: Combine multiple camera views
4. **Online learning**: Adapt model during deployment

Good luck with your perception pipeline!
