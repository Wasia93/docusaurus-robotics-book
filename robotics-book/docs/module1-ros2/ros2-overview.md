---
sidebar_position: 1
title: ROS 2 Overview
description: Introduction to Robot Operating System 2 - The robotic nervous system
keywords: [ROS 2, Robot Operating System, middleware, robotics software]
---

# Module 1: The Robotic Nervous System (ROS 2)

## Introduction to ROS 2

**ROS 2 (Robot Operating System 2)** is the industry-standard middleware for building robot software. Think of it as the **nervous system** of a robot—connecting sensors (sensory neurons), processing (brain), and actuators (motor neurons).

### Why ROS 2?

1. **Industry Standard**: Used by Boston Dynamics, Tesla, NASA, most robotics companies
2. **Modular Architecture**: Build complex systems from reusable components
3. **Language Agnostic**: Write nodes in Python, C++, or other languages
4. **Rich Ecosystem**: Thousands of packages for navigation, perception, manipulation
5. **Production Ready**: Unlike ROS 1, ROS 2 is designed for real-world deployment

### ROS 2 vs. ROS 1

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | TCPROS (custom) | DDS (industry standard) |
| **Real-time** | No | Yes (with RT OS) |
| **Security** | No | Built-in encryption & authentication |
| **Multi-robot** | Difficult | Native support |
| **Platforms** | Linux only | Linux, Windows, macOS, RTOS |
| **Lifecycle** | None | Managed node lifecycle |
| **QoS** | No | Flexible Quality of Service |

**Why ROS 2 for this course?**
- Modern architecture suitable for production humanoids
- Better performance and reliability
- Industry is actively migrating from ROS 1 to ROS 2
- ROS 1 is approaching end-of-life (2025)

## What is Middleware?

**Middleware** sits between hardware/OS and application software, providing:

- **Communication**: How different parts of the robot talk to each other
- **Abstraction**: Hide hardware complexity behind standard interfaces
- **Tools**: Debugging, visualization, logging, replay

### Without Middleware (Monolithic Approach)

```python
# Everything in one giant program
def main():
    camera = Camera()
    lidar = Lidar()
    motors = Motors()

    while True:
        image = camera.read()
        obstacles = lidar.scan()

        # Process everything in one place
        decision = make_decision(image, obstacles)
        motors.move(decision)
```

**Problems**:
- Hard to test individual components
- Can't reuse code across robots
- Difficult to debug when things go wrong
- No parallelism (everything sequential)

### With ROS 2 (Modular Approach)

```python
# Separate nodes that communicate via topics

# Node 1: Camera Driver
class CameraNode(Node):
    def __init__(self):
        self.publisher = self.create_publisher(Image, '/camera/image')

    def timer_callback(self):
        image = self.camera.read()
        self.publisher.publish(image)

# Node 2: Object Detector
class DetectorNode(Node):
    def __init__(self):
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.callback)
        self.publisher = self.create_publisher(
            DetectionArray, '/detections')

    def callback(self, image_msg):
        detections = self.detect_objects(image_msg)
        self.publisher.publish(detections)

# Node 3: Motion Planner
class PlannerNode(Node):
    def __init__(self):
        self.subscription = self.create_subscription(
            DetectionArray, '/detections', self.callback)
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel')

    def callback(self, detections_msg):
        velocity = self.plan_motion(detections_msg)
        self.publisher.publish(velocity)
```

**Benefits**:
- Each node is independent and testable
- Nodes can run on different machines/cores
- Easy to swap implementations (e.g., different cameras)
- Reusable across projects

## Core ROS 2 Concepts

### 1. Nodes

**Node** = An independent process/program that performs a specific function

Examples:
- `camera_driver_node`: Publishes camera images
- `object_detector_node`: Detects objects in images
- `path_planner_node`: Plans robot trajectory
- `motor_controller_node`: Sends commands to motors

**Key properties**:
- Each node runs in its own process
- Nodes communicate via topics, services, or actions
- Nodes can be started/stopped independently
- A robot typically has 10-100+ nodes running simultaneously

### 2. Topics (Publish-Subscribe)

**Topic** = Named bus for streaming data

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- **Many-to-many**: Multiple publishers and subscribers per topic

**Example**: `/camera/image` topic
- Camera node publishes images
- Object detector node subscribes
- Visualization node also subscribes
- Logger node records for later replay

**When to use topics**:
- Continuous data streams (sensor data, robot state)
- Broadcast information (one publisher, many subscribers)
- Fire-and-forget communication

### 3. Services (Request-Reply)

**Service** = Synchronous request-response communication

- **Client** sends a request and waits for response
- **Server** processes request and sends response
- **One-to-one**: Client waits for specific server

**Example**: `/set_robot_mode` service
- Client: "Switch to autonomous mode"
- Server: "Mode changed, returning current state"

**When to use services**:
- Infrequent operations (mode changes, calibration)
- Need confirmation/response (success/failure)
- Query information ("What is current battery level?")

### 4. Actions (Long-Running Tasks)

**Action** = Asynchronous goal-based communication with feedback

- **Client** sends a goal
- **Server** works on goal and sends periodic feedback
- **Client** can cancel goal at any time
- **Server** eventually returns result (success/failure)

**Example**: `/navigate_to_goal` action
- Goal: "Go to coordinates (10, 5)"
- Feedback: "Currently at (3, 2), 30% complete"
- Result: "Arrived at goal" or "Failed: obstacle blocking"

**When to use actions**:
- Long-running tasks (navigation, manipulation)
- Need progress feedback
- Need ability to cancel/preempt

### 5. Messages

**Message** = Data structure passed between nodes

Defined in `.msg` files:

```
# geometry_msgs/msg/Twist.msg
Vector3 linear   # linear velocity (x, y, z)
Vector3 angular  # angular velocity (roll, pitch, yaw)
```

Common message types:
- `sensor_msgs/Image`: Camera images
- `sensor_msgs/LaserScan`: LIDAR data
- `geometry_msgs/Twist`: Velocity commands
- `nav_msgs/Odometry`: Robot pose and velocity
- `std_msgs/String`: Simple text messages

### 6. Parameters

**Parameters** = Configuration values for nodes

- Set at startup or changed at runtime
- Type-safe (int, float, string, bool, arrays)
- Can be loaded from YAML files

**Example**: Camera node parameters
```yaml
camera_node:
  ros__parameters:
    frame_rate: 30
    resolution: [1920, 1080]
    auto_exposure: true
```

## DDS: The Communication Layer

ROS 2 uses **DDS (Data Distribution Service)** for communication:

### What is DDS?

- **Industry standard** (Object Management Group)
- **Peer-to-peer**: No central broker (unlike ROS 1's roscore)
- **Discovery**: Nodes automatically find each other on the network
- **Quality of Service (QoS)**: Customize reliability, latency, etc.

### Quality of Service (QoS)

QoS policies control communication behavior:

**Reliability**:
- `BEST_EFFORT`: Fast, can lose messages (sensor data)
- `RELIABLE`: Guaranteed delivery, slower (commands)

**Durability**:
- `VOLATILE`: Only send to current subscribers
- `TRANSIENT_LOCAL`: Send last message to late-joining subscribers

**History**:
- `KEEP_LAST`: Store last N messages
- `KEEP_ALL`: Store all messages (until processed)

**Example**:
```python
# Sensor data: Fast, can tolerate losses
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Commands: Must be reliable
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

## ROS 2 Tools

### Command-Line Tools

**Node management**:
```bash
ros2 node list          # List running nodes
ros2 node info /my_node # Show node details (topics, services)
```

**Topic inspection**:
```bash
ros2 topic list               # List all topics
ros2 topic echo /camera/image # Print messages
ros2 topic hz /camera/image   # Measure publish rate
ros2 topic info /camera/image # Show publishers/subscribers
```

**Service calls**:
```bash
ros2 service list          # List all services
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
```

**Parameter management**:
```bash
ros2 param list /my_node         # List node parameters
ros2 param get /my_node frame_rate
ros2 param set /my_node frame_rate 60
```

### Visualization Tools

**RViz2**: 3D visualization of robot, sensors, and environment
- Display camera images, LIDAR scans, robot model
- Visualize planned paths and detected objects
- Essential for debugging perception and navigation

**rqt**: Plugin-based GUI tools
- `rqt_graph`: Visualize node/topic connections
- `rqt_plot`: Real-time plotting of numeric data
- `rqt_image_view`: Display camera images
- `rqt_console`: View log messages

**Foxglove Studio**: Modern web-based visualization
- Cross-platform (replaces RViz for some use cases)
- Better performance with large datasets
- Custom layouts and panels

### Bag Files (Recording and Replay)

**Record sensor data and replay**:
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /lidar/scan

# Replay recorded data
ros2 bag play my_recording.db3
```

**Use cases**:
- Debug issues without robot hardware
- Collect datasets for machine learning
- Regression testing (replay and compare results)

## ROS 2 Installation

### Supported Platforms

- **Ubuntu 22.04** (Humble Hawksbill) - Recommended for this course
- **Ubuntu 24.04** (Jazzy Jalisco) - Latest LTS
- **Windows 10/11** (via binary or WSL2)
- **macOS** (limited support)

### Installation (Ubuntu 22.04)

```bash
# Set up sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(source /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop  # Full installation with RViz, demos
# OR
sudo apt install ros-humble-base     # Minimal (no GUI tools)

# Install development tools
sudo apt install ros-dev-tools

# Source setup script (add to ~/.bashrc)
source /opt/ros/humble/setup.bash
```

### Verify Installation

```bash
ros2 --version
# Should output: ros2 cli version: 0.18.x

ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener
# Should see messages being published and received
```

## Next Steps

Now that you understand ROS 2 fundamentals, let's dive into the core communication patterns: **Nodes, Topics, and Services**.

👉 **[Next: Nodes, Topics, and Services →](nodes-topics-services)**

---

:::tip Getting Started Tip

The best way to learn ROS 2 is by doing:
1. Install ROS 2 on Ubuntu 22.04 (VM or dual-boot if on Windows/Mac)
2. Work through official tutorials: https://docs.ros.org/en/humble/Tutorials.html
3. Run example nodes and inspect with command-line tools
4. Modify examples to understand how things work

:::

## Further Reading

- **Official Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Design**: https://design.ros2.org/
- **Migrating from ROS 1**: https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html
- **ROS 2 Book**: "A Concise Introduction to Robot Programming with ROS2" by Francisco Martín Rico

## Lab Exercise

**Setup your ROS 2 environment**:
1. Install ROS 2 Humble on Ubuntu 22.04
2. Run the `talker` and `listener` demo nodes
3. Use `ros2 topic echo` to see messages
4. Visualize the node graph with `rqt_graph`
5. Record a bag file and replay it

**Challenge**: Modify the `talker` node to publish at a different rate or with custom messages.
