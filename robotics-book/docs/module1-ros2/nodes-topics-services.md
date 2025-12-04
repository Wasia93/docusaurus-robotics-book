---
sidebar_position: 2
title: Nodes, Topics, and Services
description: Master the core communication patterns in ROS 2
keywords: [ROS 2 nodes, topics, services, publish-subscribe, request-response]
---

# Nodes, Topics, and Services

## Building Your First ROS 2 Node

A **node** is a single executable process in your robot system. Let's build a simple publisher node in Python.

### Minimal Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key components**:
1. **Import rclpy**: The ROS 2 Python client library
2. **Inherit from Node**: Base class providing ROS 2 functionality
3. **create_publisher()**: Set up topic publishing
4. **create_timer()**: Periodic callback execution
5. **spin()**: Keep node running and processing callbacks

### Minimal Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the nodes**:
```bash
# Terminal 1
python3 publisher_node.py

# Terminal 2
python3 subscriber_node.py
```

## Topics Deep Dive

### Topic Naming Conventions

**Best practices**:
- Use lowercase with underscores: `camera_image` not `CameraImage`
- Namespace by function: `/robot1/camera/image` not `/robot1_camera_image`
- Be descriptive: `/left_wheel/velocity` not `/lw_vel`

**Common namespaces**:
```
/camera/
    image_raw          # Unprocessed image
    image_rect         # Rectified image
    camera_info        # Calibration data

/lidar/
    scan               # 2D laser scan
    points             # 3D point cloud

/odom                  # Odometry (position estimate)
/cmd_vel               # Velocity commands
/joint_states          # Robot joint positions/velocities
```

### Message Types

**Standard messages** (from `std_msgs`):
```python
from std_msgs.msg import String, Int32, Float64, Bool
```

**Geometry messages** (from `geometry_msgs`):
```python
from geometry_msgs.msg import Twist, Pose, Point, Quaternion

# Velocity command example
twist_msg = Twist()
twist_msg.linear.x = 0.5   # Move forward at 0.5 m/s
twist_msg.angular.z = 0.1  # Turn at 0.1 rad/s
```

**Sensor messages** (from `sensor_msgs`):
```python
from sensor_msgs.msg import Image, LaserScan, Imu, JointState

# Image message
image_msg = Image()
image_msg.height = 480
image_msg.width = 640
image_msg.encoding = "rgb8"
image_msg.data = image_bytes
```

### Creating Custom Messages

Define in `msg/HumanoidState.msg`:
```
# Humanoid robot state message
Header header

# Joint positions (radians)
float64[] joint_positions

# Balance state
geometry_msgs/Point center_of_mass
bool is_balanced

# Battery status
float32 battery_voltage
float32 battery_percentage
```

Build and use:
```python
from my_robot_msgs.msg import HumanoidState

msg = HumanoidState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.joint_positions = [0.0, 0.5, -0.5, ...]
msg.is_balanced = True
msg.battery_percentage = 85.0
```

## Quality of Service (QoS) Profiles

### Predefined Profiles

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Sensor data (fast, lossy OK)
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Commands (reliable, must not lose)
command_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Publisher with QoS
self.publisher = self.create_publisher(
    Image,
    '/camera/image',
    qos_profile=sensor_qos
)
```

### QoS Compatibility

**Publisher and subscriber must have compatible QoS**:

| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| RELIABLE | RELIABLE | ✅ Yes |
| RELIABLE | BEST_EFFORT | ✅ Yes |
| BEST_EFFORT | RELIABLE | ❌ No |
| BEST_EFFORT | BEST_EFFORT | ✅ Yes |

**Debugging QoS issues**:
```bash
ros2 topic info /my_topic --verbose
# Shows QoS settings for all publishers and subscribers
```

## Services

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Creating a Service Client

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

# Usage
client = AddTwoIntsClient()
future = client.send_request(3, 5)
rclpy.spin_until_future_complete(client, future)
result = future.result()
print(f'Result: {result.sum}')
```

### Real-World Service Example

**Robot mode switching**:

Define `srv/SetMode.srv`:
```
string mode  # "manual", "autonomous", "emergency_stop"
---
bool success
string message
string current_mode
```

Server implementation:
```python
from robot_interfaces.srv import SetMode

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.current_mode = "manual"
        self.srv = self.create_service(
            SetMode,
            'set_mode',
            self.set_mode_callback)

    def set_mode_callback(self, request, response):
        valid_modes = ["manual", "autonomous", "emergency_stop"]

        if request.mode not in valid_modes:
            response.success = False
            response.message = f"Invalid mode. Must be one of {valid_modes}"
            response.current_mode = self.current_mode
            return response

        self.current_mode = request.mode
        response.success = True
        response.message = f"Mode changed to {self.current_mode}"
        response.current_mode = self.current_mode
        self.get_logger().info(response.message)
        return response
```

## Practical Example: Camera Image Processor

Combining publishers and subscribers:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Subscriber: Raw images from camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publisher: Processed images
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_processed',
            10)

        self.bridge = CvBridge()
        self.get_logger().info('Image Processor Node Started')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image (example: convert to grayscale and edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image and publish
        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header = msg.header  # Preserve timestamp
        self.publisher.publish(processed_msg)

        self.get_logger().info('Processed and published image')

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

You've learned the fundamental communication patterns in ROS 2. Next, we'll explore how to bridge Python AI agents to ROS 2 controllers using `rclpy`.

👉 **[Next: Python and ROS 2 Integration →](python-ros2)**

---

:::tip Best Practices

1. **One responsibility per node**: Don't create monolithic nodes
2. **Use appropriate QoS**: Sensor data = BEST_EFFORT, Commands = RELIABLE
3. **Log appropriately**: Use `self.get_logger().info/warn/error()`
4. **Handle startup delays**: Wait for services with `wait_for_service()`
5. **Clean shutdown**: Destroy nodes and call `rclpy.shutdown()`

:::

## Lab Exercise

**Build a temperature monitoring system**:

1. **Publisher node**: Simulate temperature sensor
   - Publish to `/temperature` topic (Float64)
   - Random values between 20-30°C
   - Update at 1 Hz

2. **Subscriber node**: Monitor temperature
   - Subscribe to `/temperature`
   - Log warnings if temperature > 28°C
   - Calculate running average

3. **Service**: Reset statistics
   - Service name: `/reset_stats`
   - Clears running average in subscriber node

4. **Test**:
   - Run both nodes
   - Use `ros2 topic echo /temperature`
   - Call service: `ros2 service call /reset_stats std_srvs/srv/Trigger`
