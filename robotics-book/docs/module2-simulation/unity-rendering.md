---
sidebar_position: 3
title: Unity for Robot Visualization
description: High-fidelity rendering and human-robot interaction in Unity
keywords: [Unity, visualization, rendering, human-robot interaction, simulation]
---

# Unity for High-Fidelity Rendering

## Introduction to Unity Robotics

**Unity** is a game engine that excels at:
- **Beautiful graphics**: Real-time rendering, lighting, materials
- **Human-robot interaction**: Simulate humans, social scenarios
- **VR/AR support**: Immersive experiences
- **Asset ecosystem**: Massive library of 3D models, environments

### Unity vs. Gazebo vs. Isaac Sim

| Feature | Gazebo | Unity | Isaac Sim |
|---------|--------|-------|-----------|
| **Graphics Quality** | Basic | Excellent | Photorealistic |
| **Physics Accuracy** | Good | Moderate | Excellent |
| **HRI Simulation** | Limited | Excellent | Good |
| **ROS Integration** | Native | Via Unity Robotics Hub | Native |
| **Learning Curve** | Moderate | Steep (game engine) | Steep |
| **Use Case** | Control algorithms | Visualization, HRI | AI training |

### When to Use Unity

1. **Visualization**: Need beautiful visuals for presentations, demos
2. **Human-Robot Interaction**: Simulate humans interacting with robots
3. **VR/AR**: Immersive teleoperation or training
4. **Rapid prototyping**: Unity's asset store speeds development
5. **Cross-platform**: Deploy to PC, mobile, web

**Not ideal for**: Accurate physics simulation (use Gazebo/Isaac for that).

## Installation

### Prerequisites

- Windows 10/11, macOS, or Linux
- 8GB RAM minimum, 16GB+ recommended
- GPU with DirectX 11/12 support

### Install Unity Hub

1. Download Unity Hub: https://unity.com/download
2. Install Unity Hub
3. Sign up for Unity account (free Personal license)

### Install Unity Editor

```bash
# Via Unity Hub:
# 1. Open Unity Hub
# 2. Installs → Add
# 3. Select Unity 2022.3 LTS (Long-Term Support)
# 4. Add Modules: Linux Build Support (for ROS 2)
```

**Recommended version**: Unity 2022.3 LTS (stable, good ROS support).

## Unity Robotics Hub

**Unity Robotics Hub** provides ROS integration:
- ROS-TCP-Connector: Bridge Unity ↔ ROS 2
- URDF Importer: Import robot models
- Sensor simulation: Publish to ROS topics

### Installation

**In Unity**:
```
Window → Package Manager → Add package from git URL
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

**On Linux (ROS 2 side)**:
```bash
# Install ROS-TCP-Endpoint
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build
source install/setup.bash

# Run endpoint
ros2 run ros_tcp_endpoint default_server_endpoint
# Listening on port 10000
```

## Creating a Robot Simulation

### Step 1: Create New Project

```
Unity Hub → New Project → 3D (URP - Universal Render Pipeline)
Name: HumanoidRobotSim
Location: /path/to/projects
Create
```

### Step 2: Import URDF

**Method 1: URDF Importer (Unity Package)**

```
1. Download URDF Importer from Unity Asset Store
2. Assets → Import URDF → Select your robot.urdf
3. Wait for import (creates GameObject hierarchy)
```

**Method 2: Manual Import**

```
1. Export robot meshes from URDF (STL, OBJ, FBX)
2. Import meshes to Unity (Assets → Import New Asset)
3. Manually recreate joint hierarchy
4. Add ArticulationBody components (Unity's physics joints)
```

### Step 3: Setup ROS Connection

**Create ROSConnectionManager**:

```csharp
// Assets/Scripts/ROSConnection.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        // Connect to ROS-TCP-Endpoint
        ROSConnection.GetOrCreateInstance().Connect();
    }
}
```

**Attach to GameObject**: Create empty GameObject, attach script.

### Step 4: Add Camera Publisher

**Publish camera images to ROS**:

```csharp
// Assets/Scripts/CameraPublisher.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/camera/image_raw";
    public float publishRate = 10f; // Hz

    private float timer;
    private Camera cam;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        cam = GetComponent<Camera>();
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            timer = 0;
            PublishImage();
        }
    }

    void PublishImage()
    {
        // Capture camera render
        RenderTexture rt = cam.targetTexture;
        RenderTexture.active = rt;

        Texture2D image = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        image.Apply();

        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            height = (uint)rt.height,
            width = (uint)rt.width,
            encoding = "rgb8",
            step = (uint)(rt.width * 3),
            data = image.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

**Attach to Camera**: Select camera, add component.

### Step 5: Add Velocity Subscriber

**Subscribe to /cmd_vel**:

```csharp
// Assets/Scripts/RobotController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/cmd_vel";

    private Vector3 linearVelocity;
    private Vector3 angularVelocity;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, UpdateVelocity);
    }

    void UpdateVelocity(TwistMsg msg)
    {
        linearVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.y,
            (float)msg.linear.z
        );

        angularVelocity = new Vector3(
            (float)msg.angular.x,
            (float)msg.angular.y,
            (float)msg.angular.z
        );
    }

    void FixedUpdate()
    {
        // Apply velocity to robot
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.velocity = transform.TransformDirection(linearVelocity);
        rb.angularVelocity = transform.TransformDirection(angularVelocity);
    }
}
```

## High-Fidelity Environments

### Lighting

**Realistic lighting** for visual perception:

```
Window → Rendering → Lighting
- Environment Lighting: Skybox
- Realtime Global Illumination: ON
- Baked Global Illumination: ON (for static objects)
```

**Add lights**:
```
GameObject → Light → Directional Light (Sun)
Intensity: 1.0
Shadows: Soft Shadows
```

### Materials

**Physically Based Rendering (PBR)**:

```csharp
// Create material
Material mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
mat.SetColor("_BaseColor", Color.red);
mat.SetFloat("_Metallic", 0.5f);
mat.SetFloat("_Smoothness", 0.8f);
```

**Asset Store**: Download free materials:
- "Free PBR Materials"
- "AllSky Free - 10 Sky/Environment Pack"

### Environments

**Pre-built environments**:

1. **Asset Store**: Search "environment", "warehouse", "office"
2. **Import**: Assets → Import Package
3. **Place robot**: Drag scene into Hierarchy

**Example environments**:
- "Office Environment" (free)
- "Warehouse Pack" (free)
- "Apartment Kit" ($20)

## Human-Robot Interaction Simulation

### Adding Humans

**Method 1: Unity Asset Store**

Search: "Human", "Character", "Person"
- "Simple Character Pack" (free)
- "Starter Assets - Third Person Character Controller" (free)

**Method 2: Mixamo (Free)**

1. Go to mixamo.com
2. Select character, download FBX
3. Import to Unity
4. Add animations (walking, waving, etc.)

### Simulating Social Scenarios

**Example: Robot approaches person**:

```csharp
// Assets/Scripts/HumanSimulator.cs
using UnityEngine;

public class HumanSimulator : MonoBehaviour
{
    public Transform robot;
    public float detectionRange = 5f;

    void Update()
    {
        float distance = Vector3.Distance(transform.position, robot.position);

        if (distance < detectionRange)
        {
            // Human looks at robot
            transform.LookAt(robot);

            // Wave animation (if available)
            Animator animator = GetComponent<Animator>();
            if (animator != null)
            {
                animator.SetTrigger("Wave");
            }
        }
    }
}
```

## VR/AR Integration

### VR Setup (Optional)

**For immersive teleoperation**:

```
Window → Package Manager → XR Plugin Management
Install → Oculus XR Plugin (for Meta Quest)
```

**Add VR camera rig**:
```
GameObject → XR → XR Origin (VR)
```

**Control robot with VR controllers**: Map button presses to /cmd_vel.

## Performance Optimization

### Reduce Draw Calls

```
- Use static batching for non-moving objects
- Combine meshes where possible
- Use LOD (Level of Detail) for distant objects
```

### Optimize Physics

```csharp
// Reduce physics update rate if needed
Time.fixedDeltaTime = 0.02f; // 50 Hz (default is 0.02)
```

### Culling

```
- Frustum culling (automatic)
- Occlusion culling (bake in Lighting settings)
- Disable shadows for distant objects
```

## Unity + Gazebo Hybrid

**Best of both worlds**: Use Unity for visualization, Gazebo for physics.

**Workflow**:
1. **Gazebo**: Run physics simulation
2. **ROS 2 Bridge**: Publish robot state (/joint_states, /tf)
3. **Unity**: Subscribe to robot state, render visuals
4. **User**: Sees beautiful Unity visuals, accurate Gazebo physics

**Why**: Unity's rendering + Gazebo's physics accuracy.

## Testing Integration

### Test 1: Camera Stream

```bash
# Terminal 1: ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: View images
ros2 run rqt_image_view rqt_image_view

# Unity: Play scene, select /camera/image_raw
```

### Test 2: Robot Control

```bash
# Terminal 1: ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Publish velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"

# Unity: Play scene, robot should move
```

## Next Steps

You've learned Unity for robot visualization. Next, explore simulating diverse sensors.

👉 **[Next: Sensor Simulation →](sensor-simulation)**

---

:::tip Unity Tips

1. **Start simple**: Blank scene → ground → robot → add complexity
2. **Use Asset Store**: Don't build everything from scratch
3. **Profile performance**: Window → Analysis → Profiler
4. **Learn C# basics**: Unity uses C# for scripting
5. **Separate concerns**: Unity for visuals, Gazebo/Isaac for physics

:::

## Lab Exercise

**Create a visually realistic robot demo**:

1. **Setup Unity project**:
   - Import humanoid URDF
   - Add realistic materials
   - Import warehouse environment

2. **Add ROS integration**:
   - Camera publisher (/camera/image_raw)
   - Velocity subscriber (/cmd_vel)
   - Joint state subscriber (/joint_states)

3. **Add humans**:
   - Import 2-3 human characters
   - Add walking animations
   - Simulate social interaction (humans wave when robot approaches)

4. **Test**:
   - Control robot from ROS 2
   - View camera stream in RViz
   - Record demo video

**Bonus**: Add VR teleoperation.
