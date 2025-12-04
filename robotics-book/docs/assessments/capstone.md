---
sidebar_position: 4
title: Capstone Project - Autonomous Humanoid
description: Final project integrating all course skills
keywords: [capstone, autonomous humanoid, VLA, final project, assessment]
---

# Capstone Project: The Autonomous Humanoid

## Project Overview

The **Capstone Project** is the culmination of 13 weeks of learning, integrating all skills from Modules 1-4:

**Goal**: Build a simulated humanoid robot that receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

**Example scenario**:
1. User says: "Clean the room - pick up the red cup"
2. Robot transcribes command using Whisper
3. Robot uses LLM to understand task
4. Robot navigates to cup using Nav2
5. Robot detects cup using computer vision
6. Robot grasps cup using manipulation controller
7. Robot reports: "Task completed"

## Learning Objectives

By completing this project, you will demonstrate:

1. **ROS 2 Mastery** (Module 1):
   - Multi-node system architecture
   - Topic/service communication
   - Launch file orchestration

2. **Simulation Proficiency** (Module 2):
   - Gazebo or Isaac Sim environment
   - Physics-accurate humanoid model
   - Sensor simulation

3. **AI Integration** (Module 3):
   - Isaac ROS perception (VSLAM, object detection)
   - Navigation (Nav2)
   - Real-time decision making

4. **VLA Implementation** (Module 4):
   - Voice-to-action pipeline
   - LLM cognitive planning
   - Vision-language-action coordination

## Project Requirements

### Minimum Viable Product (MVP)

**Required features**:

1. **Voice Command Input**:
   - Use OpenAI Whisper for speech-to-text
   - Support at least 3 commands:
     - "Pick up the [color] [object]"
     - "Navigate to [location]"
     - "Stop"

2. **Humanoid Simulation**:
   - Use Gazebo or Isaac Sim
   - Humanoid with arms, legs, head
   - At least 15 DOF (degrees of freedom)
   - Stable standing position

3. **Navigation**:
   - Use Nav2 or custom planner
   - Path planning with obstacle avoidance
   - Reach target within 0.5m accuracy

4. **Object Detection**:
   - Detect objects by color or class
   - Use YOLO, RT-DETR, or similar
   - At least 80% detection accuracy

5. **Manipulation**:
   - Grasp object using gripper
   - Pick up and hold object
   - Basic success: Touch object with gripper

6. **System Integration**:
   - All nodes communicate via ROS 2
   - Launch entire system with one command
   - Graceful error handling

### Extended Features (Optional)

**Choose 2-3 for extra credit**:

1. **Advanced Manipulation**:
   - Place object at target location
   - Two-arm coordination
   - Force-feedback grasping

2. **Dynamic Locomotion**:
   - Bipedal walking (not just sliding base)
   - Balance control while moving
   - Recover from push disturbances

3. **Multi-Object Scenarios**:
   - "Pick up all red objects"
   - Sort objects by color/type
   - Handle occlusions

4. **Natural Language Understanding**:
   - Complex commands: "Go to the kitchen and bring me a drink"
   - Question answering: "What objects do you see?"
   - Clarification questions: "Which cup - red or blue?"

5. **Sim-to-Real Transfer**:
   - Deploy to physical Jetson Orin Nano
   - Test on real camera/sensors
   - Demonstrate on physical robot (if available)

6. **Learning from Demonstration**:
   - Teleoperate robot to demonstrate task
   - Learn policy from demonstrations
   - Reproduce task autonomously

## Project Phases

### Phase 1: Planning (Week 11)

**Deliverables**:
- System architecture diagram
- Node communication graph
- Task breakdown (Gantt chart optional)
- Risk assessment

**Questions to answer**:
- Which simulator (Gazebo or Isaac Sim)?
- Which object detector (YOLO, RT-DETR)?
- How will you handle failures (object not found, path blocked)?
- What is your testing strategy?

### Phase 2: Implementation (Weeks 12-13)

**Week 12**: Core functionality
- Setup simulation environment
- Implement voice command pipeline
- Implement navigation
- Implement object detection

**Week 13**: Integration and polish
- Connect all modules
- Error handling and robustness
- Testing and debugging
- Documentation

### Phase 3: Demonstration (End of Week 13)

**Requirements**:
- Live demo (5-10 minutes)
- Video recording (backup if live demo fails)
- Code submission (GitHub or zip)
- Project report (5-10 pages)

## Technical Specification

### System Architecture

```
┌───────────────────────────────────────────────────────┐
│                   USER INTERFACE                      │
│  - Microphone (voice commands)                        │
│  - RViz (visualization)                               │
└─────────────┬─────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────┐
│              COGNITIVE LAYER                          │
│  - Whisper STT Node                                   │
│  - LLM Planner Node (GPT-4)                          │
│  - Task Scheduler                                     │
└─────────────┬─────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────┐
│             PERCEPTION LAYER                          │
│  - Camera Driver (simulated)                          │
│  - YOLO Object Detector                               │
│  - VSLAM (Visual SLAM)                                │
└─────────────┬─────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────┐
│              PLANNING LAYER                           │
│  - Nav2 Path Planner                                  │
│  - Grasp Planner                                      │
│  - Motion Planner                                     │
└─────────────┬─────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────┐
│              CONTROL LAYER                            │
│  - Locomotion Controller                              │
│  - Arm Controller                                     │
│  - Gripper Controller                                 │
└─────────────┬─────────────────────────────────────────┘
              │
┌─────────────▼─────────────────────────────────────────┐
│            SIMULATION LAYER                           │
│  - Gazebo / Isaac Sim                                 │
│  - Humanoid URDF/USD                                  │
│  - Environment (obstacles, objects)                   │
└───────────────────────────────────────────────────────┘
```

### Key ROS 2 Topics

```python
# Voice and cognitive
/audio/utterance          # Audio file path (String)
/voice_command            # Transcribed text (String)
/robot/task               # Parsed task (RobotTask)

# Perception
/camera/image_raw         # Camera image (Image)
/detections               # Object detections (Detection2DArray)
/odom                     # Robot odometry (Odometry)

# Planning
/cmd_vel                  # Velocity commands (Twist)
/plan                     # Planned path (Path)

# Control
/joint_states             # Joint positions (JointState)
/arm_command              # Arm trajectory (JointTrajectory)
/gripper_command          # Gripper state (Float64)

# Status
/robot/status             # Robot state machine (String)
/task_result              # Task completion status (TaskResult)
```

## Grading Rubric (100 points)

### Core Functionality (60 points)

1. **Voice Command Pipeline (15 points)**:
   - Whisper integration (5 pts)
   - Command parsing (5 pts)
   - LLM planning (5 pts)

2. **Navigation (15 points)**:
   - Path planning (5 pts)
   - Obstacle avoidance (5 pts)
   - Reaches target (5 pts)

3. **Object Detection (15 points)**:
   - Detection accuracy >80% (5 pts)
   - Correct object identification (5 pts)
   - Real-time performance (5 pts)

4. **Manipulation (15 points)**:
   - Reaches for object (5 pts)
   - Grasps object (5 pts)
   - Lifts object (5 pts)

### System Integration (20 points)

5. **ROS 2 Architecture (10 points)**:
   - Clean node design (3 pts)
   - Proper topic usage (3 pts)
   - Launch file completeness (4 pts)

6. **Robustness (10 points)**:
   - Error handling (4 pts)
   - Recovery from failures (3 pts)
   - Logging and debugging (3 pts)

### Documentation & Presentation (20 points)

7. **Code Quality (5 points)**:
   - Comments and docstrings (2 pts)
   - Code organization (2 pts)
   - Follows best practices (1 pt)

8. **Project Report (10 points)**:
   - System architecture (3 pts)
   - Implementation details (3 pts)
   - Results and analysis (2 pts)
   - Challenges and solutions (2 pts)

9. **Demonstration (5 points)**:
   - Live demo or video (3 pts)
   - Clear presentation (2 pts)

### Bonus (up to 15 points)

- Extended features (5 pts each, max 3)
- Exceptional creativity (up to 5 pts)

**Total**: 100 points + 15 bonus = 115 max

## Testing Scenarios

Your system will be tested on these scenarios:

### Scenario 1: Basic Pickup
```
Command: "Pick up the red cup"
Environment: Single red cup on table, no obstacles
Expected: Navigate to table, detect cup, grasp cup
Time limit: 2 minutes
```

### Scenario 2: Navigation with Obstacles
```
Command: "Go to the kitchen"
Environment: Multiple obstacles between start and kitchen
Expected: Plan path avoiding obstacles, reach kitchen
Time limit: 3 minutes
```

### Scenario 3: Multiple Objects
```
Command: "Pick up the blue ball"
Environment: Red cup, blue ball, yellow cube on table
Expected: Identify correct object, grasp blue ball
Time limit: 2 minutes
```

### Scenario 4: Complex Task
```
Command: "Clean the room - pick up all red objects"
Environment: 3 red objects scattered in room with obstacles
Expected: Plan multi-step task, pick up all red objects
Time limit: 5 minutes
```

## Starter Code

### Project Template Structure

```
capstone_project/
├── launch/
│   └── capstone.launch.py         # Main launch file
├── config/
│   ├── navigation.yaml             # Nav2 params
│   └── perception.yaml             # Detector params
├── src/
│   ├── voice_control/
│   │   ├── whisper_node.py
│   │   └── llm_planner.py
│   ├── perception/
│   │   ├── object_detector.py
│   │   └── vslam_node.py
│   ├── planning/
│   │   ├── task_planner.py
│   │   └── grasp_planner.py
│   └── control/
│       ├── locomotion_controller.py
│       └── arm_controller.py
├── urdf/
│   └── humanoid.urdf.xacro         # Robot description
├── worlds/
│   └── capstone_environment.world  # Gazebo world
├── tests/
│   └── test_scenarios.py           # Unit tests
├── README.md
└── package.xml
```

### Example Task Planner

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')

        self.command_sub = self.create_subscription(
            String, '/robot/task', self.task_callback, 10
        )

        # State machine
        self.state = "IDLE"  # IDLE, NAVIGATING, DETECTING, GRASPING, DONE

    def task_callback(self, msg):
        task = json.loads(msg.data)
        action = task['action']

        if action == "pick":
            self.execute_pick_sequence(task)

    def execute_pick_sequence(self, task):
        # 1. Navigate to object location
        self.state = "NAVIGATING"
        self.navigate_to_object(task['object'])

        # 2. Detect object
        self.state = "DETECTING"
        object_pose = self.detect_object(task['object'], task['color'])

        # 3. Grasp object
        self.state = "GRASPING"
        self.grasp_object(object_pose)

        # 4. Done
        self.state = "DONE"
        self.get_logger().info('Task completed!')

    # Implement navigation, detection, grasping methods...
```

## Submission Requirements

### Code Submission

**Submit via GitHub** (preferred) or zip file:

```
your-name-capstone.zip/
├── src/                  # All source code
├── launch/               # Launch files
├── config/               # Configuration files
├── urdf/                 # Robot description
├── worlds/               # Simulation worlds
├── README.md             # Setup instructions
├── DEMO.md               # How to run demo
└── requirements.txt      # Python dependencies
```

### Project Report

**5-10 pages**, include:

1. **Introduction**: Project goal and motivation
2. **System Architecture**: Diagrams, component descriptions
3. **Implementation**: Key algorithms, design decisions
4. **Results**: Test scenarios, success rates, performance metrics
5. **Challenges**: Problems encountered and solutions
6. **Future Work**: What you would improve given more time
7. **Conclusion**: Summary of learning outcomes

### Video Demonstration

**3-5 minutes**, show:

1. System startup (launch file)
2. Voice command input
3. Robot execution (sped up if needed)
4. Successful task completion
5. Brief code walkthrough (optional)

Upload to YouTube (unlisted) or include in submission.

## Resources

### Example Projects

- **Example 1**: Simple pick-and-place (GitHub: robotics-course/example-pick)
- **Example 2**: Voice-controlled navigation (GitHub: robotics-course/example-voice-nav)
- **Example 3**: Full capstone reference (GitHub: robotics-course/reference-capstone)

### Useful Libraries

```python
# Voice
pip install openai-whisper openai

# Vision
pip install ultralytics opencv-python

# Robotics
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-moveit
```

### Testing Tips

1. **Test incrementally**: Test each component before integration
2. **Use rosbags**: Record sensor data for repeatable testing
3. **Simulation first**: Perfect in sim before deploying to hardware
4. **Error injection**: Test failure modes (object not found, path blocked)

## FAQ

**Q: Can I work in a team?**
A: Yes, teams of 2-3 students allowed. Clearly state each member's contributions.

**Q: Must I use Gazebo or Isaac Sim?**
A: You must use a physics simulator. Gazebo or Isaac Sim recommended, but PyBullet/MuJoCo allowed.

**Q: Can I use pre-trained models?**
A: Yes, encouraged (YOLO, Whisper, GPT-4). Focus on integration, not training from scratch.

**Q: What if my robot can't walk?**
A: Sliding base (like a mobile robot) is acceptable. Bipedal walking is optional (bonus points).

**Q: Hardware required?**
A: No. Simulation-only is acceptable. Deploying to Jetson Orin Nano earns bonus points.

**Q: Can I extend the deadline?**
A: Extensions require instructor approval, granted only for extenuating circumstances.

## Conclusion

The Capstone Project is your opportunity to demonstrate mastery of Physical AI and Humanoid Robotics. It's challenging but rewarding—you'll build a system that would have seemed impossible at the start of the course.

**Good luck, and have fun building your autonomous humanoid!**

---

:::tip Success Tips

1. **Start early**: Don't wait until Week 13
2. **Test often**: Catch bugs early
3. **Ask for help**: Use office hours, Discord, forums
4. **Simplify**: MVP first, polish later
5. **Document**: Future you will thank present you

:::

:::warning Common Pitfalls

- **Over-engineering**: Start simple, add complexity only if needed
- **Integration hell**: Test components individually first
- **Ignoring edge cases**: What if object not found? Path blocked?
- **Poor time management**: Leave time for integration and debugging

:::

## Next Steps

You've completed the entire course outline! Now it's time to start building.

👉 **[Back to Introduction →](/intro)**

**Course complete - Good luck with your projects!**
