---
sidebar_position: 3
title: Python and ROS 2 Integration
description: Bridging Python AI agents to ROS 2 controllers using rclpy
keywords: [ROS 2, Python, rclpy, AI agents, integration]
---

# Python and ROS 2 Integration

## Bridging AI to Robotics

Modern AI systems are predominantly written in Python (TensorFlow, PyTorch, OpenAI APIs), while robotics requires real-time control. **rclpy** (ROS Client Library for Python) bridges this gap, allowing you to:

- Run AI models (object detection, LLMs) as ROS 2 nodes
- Subscribe to sensor data (camera, LIDAR) for AI processing
- Publish decisions/actions to robot controllers
- Integrate pre-trained models into the robot control loop

## Why Python for ROS 2?

### Advantages
- **AI/ML ecosystem**: NumPy, OpenCV, TensorFlow, PyTorch
- **Rapid prototyping**: Faster development than C++
- **Easy integration**: Call APIs (OpenAI, cloud services) directly
- **Rich libraries**: Extensive robotics packages (scipy, scikit-learn)

### Limitations
- **Performance**: Slower than C++ for compute-intensive tasks
- **GIL (Global Interpreter Lock)**: Limits true multi-threading
- **Memory**: Higher memory usage than C++

### When to Use Each

| Use Case | Language | Reason |
|----------|----------|--------|
| AI inference (object detection, LLMs) | Python | AI libraries are Python-native |
| High-level planning & decision making | Python | Rapid prototyping, flexibility |
| Real-time control loops (50-1000 Hz) | C++ | Low latency, deterministic timing |
| Sensor drivers (camera, LIDAR) | C++ | Performance, vendor SDKs often C++ |
| Visualization & tools | Python | Rapid development |

**Best practice**: Use Python for AI/planning, C++ for control loops, communicate via ROS 2 topics.

## rclpy Fundamentals

### Creating a Node

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

```python
# 1. Initialize ROS 2
rclpy.init(args=args)

# 2. Create node instance
node = MyNode()

# 3. Spin (process callbacks)
rclpy.spin(node)  # Blocks until shutdown

# 4. Cleanup
node.destroy_node()
rclpy.shutdown()
```

## Timers and Periodic Execution

### Creating Timers

```python
class PeriodicNode(Node):
    def __init__(self):
        super().__init__('periodic_node')

        # Execute callback every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback {self.count}')
        self.count += 1
```

### Multiple Timers

```python
class MultiTimerNode(Node):
    def __init__(self):
        super().__init__('multi_timer_node')

        # Fast loop: Sensor reading at 50 Hz
        self.sensor_timer = self.create_timer(0.02, self.read_sensors)

        # Slow loop: Logging at 1 Hz
        self.log_timer = self.create_timer(1.0, self.log_status)

    def read_sensors(self):
        # High-frequency sensor processing
        pass

    def log_status(self):
        # Low-frequency status logging
        pass
```

## Real-World Example: AI Object Detector Node

Integrating a pre-trained YOLO model with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

# YOLOv8 (using ultralytics package)
from ultralytics import YOLO

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        # Load YOLO model
        model_path = self.get_parameter('model_path').value
        self.model = YOLO(model_path)
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        # ROS 2 interfaces
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        self.get_logger().info(f'YOLO Detector started with model: {model_path}')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(cv_image, conf=self.conf_threshold, verbose=False)

        # Convert to ROS 2 Detection2DArray message
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Extract detection info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = result.names[class_id]

                # Create Detection2D message
                detection = Detection2D()

                # Bounding box center and size
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Object hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                detection_msg.detections.append(detection)

        # Publish detections
        self.detection_pub.publish(detection_msg)

        self.get_logger().info(
            f'Detected {len(detection_msg.detections)} objects',
            throttle_duration_sec=1.0  # Log at most once per second
        )

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Detector

```bash
# Terminal 1: Run camera node (or use rosbag)
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Run YOLO detector with custom parameters
ros2 run my_package yolo_detector --ros-args \
  -p model_path:=yolov8m.pt \
  -p confidence_threshold:=0.6

# Terminal 3: Visualize detections
ros2 run rqt_image_view rqt_image_view /detections/visualization
```

## Integrating LLMs for Cognitive Control

### OpenAI GPT Integration Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import os

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')

        # OpenAI API key (use environment variable)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to voice commands (from speech-to-text)
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )

        # Publish robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # System prompt for robot control
        self.system_prompt = """
        You are a robot controller. Convert natural language commands into
        simple motion primitives: forward, backward, turn_left, turn_right, stop.
        Respond ONLY with the command name, no explanation.
        """

        self.get_logger().info('LLM Planner Node started')

    def voice_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f'Received command: {command_text}')

        # Query LLM for action
        action = self.query_llm(command_text)

        # Execute action
        self.execute_action(action)

    def query_llm(self, user_input):
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_input}
                ],
                temperature=0.0,  # Deterministic
                max_tokens=10
            )

            action = response.choices[0].message.content.strip().lower()
            self.get_logger().info(f'LLM action: {action}')
            return action

        except Exception as e:
            self.get_logger().error(f'LLM query failed: {e}')
            return 'stop'

    def execute_action(self, action):
        twist = Twist()

        if action == 'forward':
            twist.linear.x = 0.5
        elif action == 'backward':
            twist.linear.x = -0.5
        elif action == 'turn_left':
            twist.angular.z = 0.5
        elif action == 'turn_right':
            twist.angular.z = -0.5
        elif action == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown action: {action}')
            return

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Executing: {action}')

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Asynchronous Operations

### Using async/await with ROS 2

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
import asyncio

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')

        # Service server
        self.srv = self.create_service(
            Trigger,
            'long_task',
            self.long_task_callback
        )

    async def long_task_callback(self, request, response):
        self.get_logger().info('Starting long task...')

        # Simulate long-running operation
        await asyncio.sleep(5.0)

        self.get_logger().info('Long task completed')
        response.success = True
        response.message = 'Task completed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AsyncNode()

    # Use MultiThreadedExecutor for async operations
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

## Multi-Threaded Nodes

### Parallel Processing Example

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
import threading

class ParallelProcessorNode(Node):
    def __init__(self):
        super().__init__('parallel_processor')

        # Create separate callback groups for parallel execution
        self.camera1_group = MutuallyExclusiveCallbackGroup()
        self.camera2_group = MutuallyExclusiveCallbackGroup()

        # Subscribe to two cameras with separate groups
        self.camera1_sub = self.create_subscription(
            Image,
            '/camera1/image',
            self.camera1_callback,
            10,
            callback_group=self.camera1_group
        )

        self.camera2_sub = self.create_subscription(
            Image,
            '/camera2/image',
            self.camera2_callback,
            10,
            callback_group=self.camera2_group
        )

    def camera1_callback(self, msg):
        # Process camera 1 (can run in parallel with camera2_callback)
        thread_id = threading.get_ident()
        self.get_logger().info(f'Camera 1 processing in thread {thread_id}')
        # Heavy processing here...

    def camera2_callback(self, msg):
        # Process camera 2 (can run in parallel with camera1_callback)
        thread_id = threading.get_ident()
        self.get_logger().info(f'Camera 2 processing in thread {thread_id}')
        # Heavy processing here...

def main(args=None):
    rclpy.init(args=args)
    node = ParallelProcessorNode()

    # Use MultiThreadedExecutor with 4 threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
```

## Best Practices

### 1. Parameter Management

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with defaults
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'default_robot')

        # Get parameter values
        self.update_rate = self.get_parameter('update_rate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.robot_name = self.get_parameter('robot_name').value

        # Register parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate' and param.value > 0:
                self.update_rate = param.value
                # Recreate timer with new rate
                self.get_logger().info(f'Update rate changed to {param.value}')

        return rclpy.parameter.SetParametersResult(successful=True)
```

### 2. Graceful Shutdown

```python
import signal
import sys

class GracefulNode(Node):
    def __init__(self):
        super().__init__('graceful_node')
        self.running = True

        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().info('Shutting down gracefully...')
        self.running = False
        # Cleanup operations
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        # Stop motors, close connections, save state, etc.
        self.get_logger().info('Cleanup completed')

def main(args=None):
    rclpy.init(args=args)
    node = GracefulNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3. Logging Best Practices

```python
class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')

        # Different log levels
        self.get_logger().debug('Debug message')      # Verbose info
        self.get_logger().info('Info message')         # Normal operation
        self.get_logger().warn('Warning message')      # Potential issues
        self.get_logger().error('Error message')       # Errors occurred
        self.get_logger().fatal('Fatal message')       # Critical failures

        # Throttled logging (max once per second)
        self.timer = self.create_timer(0.01, self.fast_callback)
        self.count = 0

    def fast_callback(self):
        self.count += 1
        # Only log once per second, even though callback runs 100 Hz
        self.get_logger().info(
            f'Count: {self.count}',
            throttle_duration_sec=1.0
        )
```

## Next Steps

You've mastered Python and ROS 2 integration. Next, learn how to define robot morphology using URDF for humanoids.

👉 **[Next: URDF for Humanoids →](urdf-humanoids)**

---

:::tip Performance Tip

For AI inference in ROS 2:
1. **Use GPU acceleration**: CUDA/TensorRT for neural networks
2. **Batch processing**: Accumulate messages, process in batches
3. **Async operations**: Don't block callbacks with slow operations
4. **Quality of Service**: Use BEST_EFFORT for sensor data to reduce latency
5. **Multi-threading**: Separate callback groups for parallel processing

:::

## Lab Exercise

**Build an AI-powered robot controller**:

1. **Object detector node**:
   - Subscribe to `/camera/image`
   - Run YOLOv8 detection
   - Publish to `/detections`

2. **Decision node**:
   - Subscribe to `/detections`
   - If "person" detected, send `/cmd_vel` to approach
   - If "stop_sign" detected, send stop command

3. **Test**:
   - Run with webcam or rosbag
   - Verify detection and control logic
   - Measure latency (detection to command)

**Bonus**: Add LLM integration for natural language commands.
