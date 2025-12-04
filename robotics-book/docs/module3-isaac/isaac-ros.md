---
sidebar_position: 2
title: Isaac ROS - Hardware-Accelerated Perception
description: GPU-accelerated ROS 2 packages for real-time robotics
keywords: [Isaac ROS, NVIDIA, GPU acceleration, VSLAM, perception, real-time]
---

# Isaac ROS: Hardware-Accelerated Perception

## Introduction to Isaac ROS

**Isaac ROS** is NVIDIA's collection of hardware-accelerated ROS 2 packages for robotics perception and AI:

- **GPU-accelerated**: Leverage CUDA and Tensor Cores
- **Production-ready**: Optimized for real-time performance
- **Sim-to-Real**: Works in Isaac Sim and on physical robots
- **Open-source**: Available on GitHub

### Why Isaac ROS?

**Traditional ROS 2** runs on CPU:
- VSLAM: 10-20 FPS on CPU
- Object detection: 5-10 FPS
- Image processing: Variable, can be slow

**Isaac ROS** runs on GPU:
- VSLAM: 60+ FPS on Jetson Orin
- Object detection: 30+ FPS (TensorRT optimized)
- Image processing: Real-time (30-60 FPS)

**Performance boost**: 3-10x faster than CPU-only solutions.

## Isaac ROS Packages

### Core Packages

| Package | Function | Acceleration |
|---------|----------|--------------|
| **isaac_ros_visual_slam** | Visual SLAM (cuVSLAM) | CUDA |
| **isaac_ros_nvblox** | 3D reconstruction | CUDA |
| **isaac_ros_image_proc** | Image processing | VPI (Vision Programming Interface) |
| **isaac_ros_dnn_inference** | DNN inference | TensorRT |
| **isaac_ros_apriltag** | AprilTag detection | CUDA |
| **isaac_ros_stereo_image_proc** | Stereo depth | VPI |

### Perception Pipeline

```
Camera (RealSense) → Isaac ROS Image Proc → Isaac ROS DNN Inference → Detections
                                        ↓
                                Isaac ROS Visual SLAM → Odometry
                                        ↓
                                Isaac ROS Nvblox → 3D Map
```

## Installation

### Prerequisites

- NVIDIA Jetson (Orin, Xavier) or x86 with NVIDIA GPU
- JetPack 5.1+ (for Jetson) or Ubuntu 22.04 (for x86)
- ROS 2 Humble
- CUDA 11.8+

### Install Isaac ROS

**On Jetson Orin Nano**:

```bash
# Update system
sudo apt update && sudo apt upgrade

# Install dependencies
sudo apt install -y \
    libavformat-dev \
    libavcodec-dev \
    libswscale-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev

# Install Isaac ROS packages
sudo apt install -y \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-nvblox \
    ros-humble-isaac-ros-image-proc \
    ros-humble-isaac-ros-dnn-inference

# Source ROS 2
source /opt/ros/humble/setup.bash
```

**Verify installation**:
```bash
ros2 pkg list | grep isaac_ros
# Should list multiple isaac_ros_* packages
```

## Isaac ROS Visual SLAM (cuVSLAM)

**cuVSLAM**: CUDA-accelerated Visual Simultaneous Localization and Mapping.

### Features

- **Real-time**: 60+ FPS on Jetson Orin
- **Accurate**: Sub-centimeter accuracy
- **Robust**: Handles dynamic scenes, lighting changes
- **Multiple cameras**: Supports stereo, RGB-D, fisheye

### Launch cuVSLAM

**With RealSense D435i**:

```bash
# Terminal 1: RealSense driver
ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=1 \
    enable_infra1:=true \
    enable_infra2:=true

# Terminal 2: Isaac ROS Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Subscribed topics**:
- `/camera/infra1/image_rect_raw`: Left infrared image
- `/camera/infra2/image_rect_raw`: Right infrared image
- `/camera/imu`: IMU data

**Published topics**:
- `/visual_slam/tracking/odometry`: Robot pose estimate (Odometry)
- `/visual_slam/tracking/vo_pose`: Visual odometry pose (PoseStamped)
- `/visual_slam/tracking/slam_path`: Full trajectory (Path)
- `/visual_slam/vis/observations_cloud`: Feature points (PointCloud2)

### Visualize in RViz

```bash
ros2 run rviz2 rviz2

# Add displays:
# - Odometry: /visual_slam/tracking/odometry
# - Path: /visual_slam/tracking/slam_path
# - PointCloud2: /visual_slam/vis/observations_cloud
# - TF
```

### Save and Load Maps

**Save map** (for localization later):
```bash
ros2 service call /visual_slam/save_map \
    isaac_ros_visual_slam_interfaces/srv/FilePath \
    "{file_path: '/tmp/cuvslam_map.db'}"
```

**Load map** and localize:
```bash
# Add to launch file
parameters=[{
    'map_frame': 'map',
    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'enable_localization_n_mapping': False,  # Localization only
    'load_map_from_file': True,
    'map_file_path': '/tmp/cuvslam_map.db'
}]
```

## Isaac ROS Nvblox

**Nvblox**: GPU-accelerated 3D reconstruction and mapping.

### Features

- **Real-time 3D mapping**: Build TSDF (Truncated Signed Distance Field) volumetric maps
- **Mesh generation**: Convert voxel map to mesh
- **Obstacle detection**: Identify free space vs. occupied
- **Dynamic updates**: Update map in real-time as robot moves

### Launch Nvblox

```bash
ros2 launch nvblox_examples_bringup isaac_sim_example.launch.py
```

**Subscribed topics**:
- `/camera/depth/image_raw`: Depth image (from RealSense or Isaac Sim)
- `/camera/color/image_raw`: RGB image
- `/camera/depth/camera_info`: Camera intrinsics

**Published topics**:
- `/nvblox_node/mesh`: 3D mesh (Marker)
- `/nvblox_node/map_slice`: 2D slice of 3D map (OccupancyGrid)
- `/nvblox_node/tsdf_layer`: TSDF voxel grid

### Integration with Nav2

**Use nvblox for navigation**:

```python
# navigation2 uses /map topic
# Nvblox publishes /nvblox_node/map_slice

# Remap in launch file
remappings=[
    ('/map', '/nvblox_node/map_slice'),
]
```

## Isaac ROS DNN Inference

**TensorRT-optimized neural network inference**.

### Supported Models

- **Object detection**: YOLO, RT-DETR, EfficientDet
- **Segmentation**: U-Net, SegFormer
- **Classification**: ResNet, EfficientNet
- **Pose estimation**: OpenPose

### Convert Model to TensorRT

**Example: YOLOv8 → TensorRT**

```bash
# Install TensorRT Python API
pip install tensorrt

# Export YOLOv8 to ONNX
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='onnx')

# Convert ONNX to TensorRT
trtexec --onnx=yolov8n.onnx \
        --saveEngine=yolov8n.trt \
        --fp16  # Use FP16 for Jetson
```

### Run Inference with Isaac ROS

**Create DNN inference node**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import tensorrt as trt
import pycuda.driver as cuda
import numpy as np

class TensorRTInferenceNode(Node):
    def __init__(self):
        super().__init__('tensorrt_inference')

        # Load TensorRT engine
        self.engine = self.load_engine('yolov8n.trt')
        self.context = self.engine.create_execution_context()

        # Allocate GPU memory
        self.inputs, self.outputs, self.bindings = self.allocate_buffers()

        # ROS 2
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )

    def load_engine(self, engine_path):
        with open(engine_path, 'rb') as f, trt.Runtime(trt.Logger(trt.Logger.WARNING)) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def image_callback(self, msg):
        # Convert to numpy
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess (resize, normalize)
        input_image = self.preprocess(cv_image)

        # Inference
        outputs = self.infer(input_image)

        # Postprocess (NMS, convert to Detection2DArray)
        detections = self.postprocess(outputs, cv_image.shape)

        # Publish
        detection_msg = self.to_detection_msg(detections)
        self.detection_pub.publish(detection_msg)

    def infer(self, input_data):
        # Copy input to GPU
        cuda.memcpy_htod(self.inputs[0]['allocation'], input_data)

        # Run inference
        self.context.execute_v2(bindings=self.bindings)

        # Copy output from GPU
        output = np.empty(self.outputs[0]['shape'], dtype=self.outputs[0]['dtype'])
        cuda.memcpy_dtoh(output, self.outputs[0]['allocation'])

        return output

# ... implement allocate_buffers, preprocess, postprocess, to_detection_msg
```

**Performance**: 30-60 FPS on Jetson Orin Nano.

## Isaac ROS Image Processing

**VPI (Vision Programming Interface)**: NVIDIA's hardware-accelerated computer vision library.

### Available Operations

- **Gaussian blur**: Noise reduction
- **Canny edge detection**: Edge extraction
- **Stereo disparity**: Depth from stereo
- **Image format conversion**: RGB ↔ BGR ↔ Grayscale
- **Undistortion/Rectification**: Camera calibration correction

### Example: Gaussian Blur

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Using Isaac ROS image proc
from isaac_ros_image_proc import ImageProcNode

# Or implement manually with VPI
import vpi

class VPIBlurNode(Node):
    def __init__(self):
        super().__init__('vpi_blur')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.image_pub = self.create_publisher(
            Image, '/camera/image_blurred', 10
        )

        # VPI stream (GPU)
        self.stream = vpi.Stream()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert to VPI image
        with vpi.Backend.CUDA:
            vpi_image = vpi.asimage(cv_image)

            # Gaussian blur (GPU-accelerated)
            blurred = vpi_image.gaussian_filter(5, 1.0, stream=self.stream)

            # Convert back to numpy
            result = blurred.cpu()

        # Publish
        blurred_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        blurred_msg.header = msg.header
        self.image_pub.publish(blurred_msg)
```

**Performance**: 10-20x faster than OpenCV on CPU.

## Isaac ROS AprilTag Detection

**AprilTag**: Fiducial markers for localization and calibration.

### Launch AprilTag Detector

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

**Subscribed**: `/camera/image_raw`

**Published**: `/tag_detections` (AprilTagDetectionArray)

### Use Cases

1. **Robot localization**: Place tags at known positions
2. **Object pose estimation**: Tags on objects
3. **Camera calibration**: Tag grid for calibration
4. **Multi-robot coordination**: Tags for robot identification

## Integration Example: Complete Perception Stack

**Humanoid with visual SLAM, object detection, and 3D mapping**:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py'
            ),
            launch_arguments={
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'enable_depth': 'true',
            }.items()
        ),

        # Isaac ROS Visual SLAM (cuVSLAM)
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            parameters=[{
                'enable_image_denoising': True,
                'rectified_images': True,
            }],
            remappings=[
                ('/stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                ('/stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                ('/visual_slam/imu', '/camera/imu'),
            ]
        ),

        # Isaac ROS Nvblox (3D mapping)
        Node(
            package='nvblox_ros',
            executable='nvblox_node',
            parameters=[{
                'global_frame': 'odom',
            }],
            remappings=[
                ('/depth/image', '/camera/depth/image_rect_raw'),
                ('/depth/camera_info', '/camera/depth/camera_info'),
                ('/color/image', '/camera/color/image_raw'),
                ('/color/camera_info', '/camera/color/camera_info'),
            ]
        ),

        # Object detection (TensorRT)
        Node(
            package='my_perception',
            executable='tensorrt_yolo_node',
            parameters=[{
                'model_path': '/path/to/yolov8n.trt',
                'confidence_threshold': 0.5,
            }]
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/path/to/config.rviz']
        ),
    ])
```

**Run**:
```bash
ros2 launch my_robot perception_stack.launch.py
```

**Result**: Real-time visual SLAM, 3D reconstruction, and object detection on Jetson Orin Nano.

## Performance Benchmarks

### cuVSLAM (Isaac ROS Visual SLAM)

| Platform | FPS | Latency | CPU Usage |
|----------|-----|---------|-----------|
| Jetson Orin Nano | 60 FPS | 16ms | 30% |
| Jetson Orin NX | 90 FPS | 11ms | 25% |
| RTX 4080 (Desktop) | 120+ FPS | under 10ms | 15% |

### Nvblox (3D Reconstruction)

| Platform | Voxel Updates/sec | Latency |
|----------|-------------------|---------|
| Jetson Orin Nano | 1M voxels/sec | 30ms |
| Jetson Orin NX | 3M voxels/sec | 20ms |

### TensorRT Inference (YOLOv8n)

| Platform | FPS | Latency |
|----------|-----|---------|
| Jetson Orin Nano (FP16) | 35 FPS | 28ms |
| Jetson Orin NX (FP16) | 60 FPS | 16ms |
| RTX 4080 (FP16) | 120+ FPS | under 10ms |

**Comparison**: CPU-only YOLO runs at 5-10 FPS on same hardware.

## Next Steps

You've learned Isaac ROS for hardware-accelerated perception. Next, explore VSLAM and navigation.

👉 **[Next: VSLAM and Navigation →](vslam-navigation)**

---

:::tip Isaac ROS Tips

1. **Use Jetson**: Isaac ROS is optimized for Jetson (Orin, Xavier)
2. **TensorRT models**: Always convert to TensorRT for best performance
3. **FP16 precision**: Use FP16 on Jetson (minimal accuracy loss, 2x faster)
4. **Monitor resources**: Use `tegrastats` (Jetson) or `nvidia-smi` (desktop)
5. **Synchronize topics**: Use `message_filters` for multi-sensor fusion

:::

## Lab Exercise

**Build accelerated perception pipeline**:

1. **Install Isaac ROS**:
   - On Jetson Orin Nano
   - Install cuVSLAM, Nvblox, image proc

2. **Setup RealSense**:
   - Connect D435i to Jetson
   - Launch with IMU enabled

3. **Run cuVSLAM**:
   - Launch visual SLAM
   - Move robot/camera around
   - Save map to file

4. **Add Nvblox**:
   - Launch 3D mapping
   - Build 3D map as you move
   - Visualize mesh in RViz

5. **Measure performance**:
   - Check FPS: `ros2 topic hz /visual_slam/tracking/odometry`
   - Monitor GPU: `tegrastats`
   - Compare to CPU-only SLAM (ORB-SLAM3)

**Bonus**: Add TensorRT object detection, integrate all three.
