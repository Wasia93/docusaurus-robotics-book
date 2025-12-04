---
sidebar_position: 1
title: Welcome to Physical AI & Humanoid Robotics
description: Bridging the gap between the digital brain and the physical body
keywords: [physical ai, robotics, humanoid robots, ROS 2, NVIDIA Isaac, embodied intelligence]
---

# Physical AI & Humanoid Robotics Course Book

**Focus and Theme**: AI Systems in the Physical World. Embodied Intelligence.

**Goal**: Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.

## Course Overview

The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces **Physical AI**—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.

## Why Physical AI Matters

Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.

## Who This Book Is For

This course is designed for:
- **AI/ML Students** transitioning from digital AI to physical robotics
- **Robotics Engineers** building embodied intelligence systems
- **Researchers** in humanoid robotics and physical AI
- **Practitioners** deploying real-world autonomous systems

## Prerequisites

- Strong programming skills (Python required)
- AI/ML fundamentals (previous AI coursework)
- Basic linear algebra and calculus
- Familiarity with deep learning concepts
- Access to required hardware (see Hardware Requirements section)

## Course Structure

This 13-week course is organized into four major modules:

### Introduction: Foundations of Physical AI (Weeks 1-2)

Build foundational understanding:
- **Foundations of Physical AI and embodied intelligence**
- **From digital AI to robots that understand physical laws**
- **Overview of humanoid robotics landscape**
- **Sensor systems**: LIDAR, cameras, IMUs, force/torque sensors

### Module 1: The Robotic Nervous System (Weeks 3-5)

**Focus**: Middleware for robot control

- ROS 2 architecture and core concepts
- Nodes, topics, services, and actions
- Bridging Python Agents to ROS controllers using rclpy
- Understanding URDF (Unified Robot Description Format) for humanoids
- Building ROS 2 packages and launch file management

### Module 2: The Digital Twin (Weeks 6-7)

**Focus**: Physics simulation and environment building

- Simulating physics, gravity, and collisions in Gazebo
- High-fidelity rendering and human-robot interaction in Unity
- URDF and SDF robot description formats
- Simulating sensors: LiDAR, Depth Cameras, and IMUs

### Module 3: The AI-Robot Brain (Weeks 8-10)

**Focus**: Advanced perception and training with NVIDIA Isaac

- **NVIDIA Isaac Sim**: Photorealistic simulation and synthetic data generation
- **Isaac ROS**: Hardware-accelerated VSLAM (Visual SLAM) and navigation
- **Nav2**: Path planning for bipedal humanoid movement
- Reinforcement learning for robot control
- Sim-to-real transfer techniques

### Module 4: Vision-Language-Action (Weeks 11-13)

**Focus**: The convergence of LLMs and Robotics

- **Voice-to-Action**: Using OpenAI Whisper for voice commands
- **Cognitive Planning**: Using LLMs to translate natural language into ROS 2 actions
- **Humanoid Robot Development**: Kinematics, dynamics, and bipedal locomotion
- **Manipulation and grasping** with humanoid hands
- **Capstone Project**: The Autonomous Humanoid

## Learning Outcomes

By the end of this course, you will be able to:

- Understand Physical AI principles and embodied intelligence
- Master ROS 2 (Robot Operating System) for robotic control
- Simulate robots with Gazebo and Unity
- Develop with NVIDIA Isaac AI robot platform
- Design humanoid robots for natural interactions
- Integrate GPT models for conversational robotics

## Our Approach: Test-Driven Development

Throughout this book, we emphasize **Test-Driven Development (TDD)**:

1. **Write tests first** - Define expected behavior before implementation
2. **Implement to pass tests** - Write code that satisfies requirements
3. **Refactor** - Improve code while maintaining correctness

Why TDD for robotics?
- **Safety**: Catch dangerous behaviors before deployment
- **Reliability**: Ensure systems work in edge cases
- **Debugging**: Physical systems are hard to debug in real-time
- **Confidence**: Deploy with evidence of correctness

## Code Examples

All code examples in this book are:
- ✅ **Runnable**: Complete, tested implementations
- ✅ **Practical**: Based on real robotic systems
- ✅ **Well-tested**: Include comprehensive unit tests
- ✅ **Documented**: Clear explanations of "why" not just "what"

Example structure you'll see throughout:

```python
# Implementation with docstrings
def forward_kinematics(theta1, theta2):
    """
    Calculate end-effector position.

    Args:
        theta1, theta2: Joint angles (radians)

    Returns:
        (x, y): End-effector position
    """
    # Implementation here
    pass

# Comprehensive tests
def test_forward_kinematics_zero():
    """Test FK when joints at zero"""
    x, y = forward_kinematics(0, 0)
    assert x == 2.0 and y == 0.0
```

## Assessments

The course includes four major assessments:

1. **ROS 2 Package Development Project** - Build a complete ROS 2 package
2. **Gazebo Simulation Implementation** - Create a simulated robot environment
3. **Isaac-Based Perception Pipeline** - Implement computer vision and SLAM
4. **Capstone: Simulated Humanoid with Conversational AI** - Final project integrating all skills

### The Capstone Project: The Autonomous Humanoid

A final project where a simulated robot:
1. Receives a voice command (e.g., "Clean the room")
2. Plans a path using Nav2
3. Navigates obstacles in simulation
4. Identifies an object using computer vision
5. Manipulates it using humanoid hands

## How to Use This Book

### Sequential Learning Path

This course is designed to be taken sequentially over 13 weeks:
1. **Weeks 1-2**: Complete the Introduction section
2. **Weeks 3-5**: Work through Module 1 (ROS 2)
3. **Weeks 6-7**: Complete Module 2 (Simulation)
4. **Weeks 8-10**: Master Module 3 (NVIDIA Isaac)
5. **Weeks 11-13**: Finish Module 4 and Capstone Project

### Hands-On Practice

This course is **highly practical**:
- Run all code examples in your environment
- Complete lab exercises for each module
- Build incrementally toward the capstone project
- Test on simulated and real hardware when available

## Required Tools and Setup

This course uses industry-standard tools:

### Software Stack
- **Ubuntu 22.04 LTS** (ROS 2 native environment)
- **Python 3.10+** with rclpy for ROS 2
- **ROS 2 Humble/Iron** for robot control
- **Gazebo** for physics simulation
- **Unity** for high-fidelity rendering
- **NVIDIA Isaac Sim** for photorealistic simulation
- **NVIDIA Isaac ROS** for hardware-accelerated perception

### Hardware Requirements

This is a **technically demanding course** sitting at the intersection of:
- Physics Simulation (Isaac Sim/Gazebo)
- Visual Perception (SLAM/Computer Vision)
- Generative AI (LLMs/VLA)

See the **Hardware & Infrastructure** section for detailed requirements including:
- High-performance workstations (RTX 4070 Ti+ GPU, 64GB RAM)
- Edge AI kits (NVIDIA Jetson Orin Nano)
- Robot lab options (Unitree Go2/G1 or alternatives)
- Cloud vs. on-premise infrastructure decisions

## Community and Support

- **GitHub Repository** - Code examples and lab exercises
- **Discussion Forum** - Ask questions and collaborate with peers
- **Office Hours** - Live help sessions with instructors
- **Discord Channel** - Real-time community support

## Let's Begin!

Ready to start your journey from digital AI to Physical AI?

👉 **[Introduction to Physical AI →](foundations/intro-physical-ai)**

---

:::tip Success in Physical AI

Physical AI requires mastery across three domains:
1. **Robotics Engineering** - ROS 2, URDF, kinematics, control systems
2. **AI/ML** - Computer vision, SLAM, reinforcement learning, LLMs
3. **Systems Integration** - Simulation, deployment, debugging, safety

This course gives you all three through hands-on projects culminating in a conversational humanoid robot.
:::

:::warning Hardware Investment Required

Unlike purely digital AI courses, Physical AI requires significant computational resources. Budget for either:
- **High-performance workstation** (~$2,500-4,000) with RTX GPU
- **Cloud computing** (~$200-300/quarter) for simulation
- **Edge AI kit** (~$700) for physical deployment

See Hardware Requirements for detailed breakdown and budget options.
:::
