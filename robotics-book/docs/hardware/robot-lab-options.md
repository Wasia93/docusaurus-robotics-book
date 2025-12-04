---
sidebar_position: 3
title: Robot Lab Options
description: Physical robot platforms for testing and deployment
keywords: [robot hardware, Unitree, humanoid robots, robot lab, physical testing]
---

# Robot Lab Options

## The "Physical" in Physical AI

While simulation (Gazebo, Isaac Sim) is essential for development and training, **Physical AI** ultimately requires **physical robots** to:
- Validate sim-to-real transfer
- Experience real-world challenges (sensor noise, dynamics, wear)
- Demonstrate actual capabilities
- Discover edge cases simulation misses

This chapter surveys robot hardware options for the course, organized by budget and use case.

## Three-Tier Approach

### Tier 1: Desktop Edge Deployment (No Robot) - $700

**What**: Jetson Orin Nano + sensors on a desk
**Purpose**: Learn ROS 2, perception, AI deployment without physical locomotion

**Kit**:
- NVIDIA Jetson Orin Nano Super ($249)
- Intel RealSense D435i ($349)
- ReSpeaker USB Mic Array ($69)
- Accessories ($30)
- **Total**: ~$700

**What you can do**:
- Run Isaac ROS (VSLAM, perception)
- Test VLA models
- Voice-to-action pipeline
- Object detection and tracking
- All Module 3 and 4 labs

**What you cannot do**:
- Physical locomotion (walking, balance)
- Real-world navigation
- Manipulation with actual arms/hands

**Recommendation**: Start here. Most of the course can be completed with simulation + edge kit.

### Tier 2: Proxy Robot (Quadruped) - $2,500-3,500

**What**: Use a quadruped (dog robot) as a proxy for humanoid
**Purpose**: Test locomotion, navigation, sim-to-real without humanoid cost

#### Option A: Unitree Go2 Edu ($2,800)

**Specs**:
- 4-legged robot (quadruped)
- Max speed: 5 m/s
- Runtime: 1-2 hours
- Payload: 5 kg (can carry Jetson + sensors)
- IP67 waterproof

**ROS 2 Support**: Official `unitree_ros2` package

**Why Go2**:
- ✅ Affordable for a legged robot
- ✅ Durable (can withstand falls)
- ✅ Good ROS 2 integration
- ✅ Active community
- ❌ Not a humanoid (bipedal vs. quadrupedal)

**Use cases**:
- VSLAM and navigation
- Terrain traversal
- Multi-robot coordination
- Sim-to-real learning

**Cost breakdown**:
- Go2 EDU: $2,800
- (Sensors included: cameras, LIDAR)
- **Total**: $2,800

#### Option B: Unitree B2 ($15,000+)

- Heavier payload (60 kg)
- Industrial applications
- Overkill for this course

### Tier 3: Humanoid Robots - $12,000-90,000

**What**: Actual bipedal humanoid robots
**Purpose**: Full humanoid development, bipedal locomotion, manipulation

#### Budget Option: Hiwonder TonyPi Pro (~$600)

**Specs**:
- Small table-top humanoid (35 cm tall)
- Raspberry Pi 4 based
- 18 DOF (servos)
- Basic walking capability

**Pros**:
- ✅ Cheap
- ✅ Good for learning kinematics
- ✅ Safe (small, lightweight)

**Cons**:
- ❌ **Cannot run NVIDIA Isaac ROS** (Raspberry Pi, not Jetson)
- ❌ Toy-grade hardware (not robust)
- ❌ Limited payload (can't add heavy sensors)
- ❌ Slow, unstable walking

**Verdict**: Good for learning, not for this course's AI focus.

#### Mid-Range: Robotis OP3 (~$12,000)

**Specs**:
- Research humanoid (45 cm tall)
- Intel NUC based
- 20 DOF (Dynamixel servos)
- Open-source (ROS support)

**Pros**:
- ✅ Stable platform (used in RoboCup)
- ✅ Good documentation
- ✅ Can add Jetson Orin Nano for AI

**Cons**:
- ❌ Expensive for limited capabilities
- ❌ Old design (2016)
- ❌ Limited payload

**Verdict**: Solid research platform, but expensive.

#### Practical Option: Unitree G1 (~$16,000)

**Specs**:
- Full-size humanoid (1.32 m tall, 35 kg)
- 23-43 DOF (depends on config)
- Can walk dynamically
- SDK for custom control

**Pros**:
- ✅ One of the most affordable full-size humanoids
- ✅ Can actually walk (not just demo)
- ✅ Growing ROS 2 community
- ✅ Payload for sensors and compute

**Cons**:
- ❌ $16,000 is still expensive
- ❌ Limited manipulation (basic grippers)
- ❌ Requires expertise to control

**Verdict**: **Best option** if budget allows. Actual humanoid capabilities.

#### Premium: Unitree H1 (~$90,000+)

**Specs**:
- Advanced humanoid
- Dynamic walking, running
- High-torque actuators
- Research/industrial grade

**Pros**:
- ✅ State-of-the-art capabilities
- ✅ Production quality
- ✅ Advanced SDK

**Cons**:
- ❌ Very expensive
- ❌ Requires dedicated lab space
- ❌ Overkill for course

**Verdict**: For well-funded research labs only.

## Recommended Strategy

### For Individual Students

**Phase 1 (Weeks 1-10)**: Simulation + Edge Kit ($700)
- Learn ROS 2, Gazebo, Isaac Sim
- Develop algorithms in simulation
- Deploy perception to Jetson on desk

**Phase 2 (Weeks 11-13)**: Optional physical robot
- **If no budget**: Continue with simulation, deploy to Jetson
- **If $3k budget**: Unitree Go2 for locomotion testing
- **If $16k budget**: Unitree G1 for full humanoid

### For University Labs

**Recommended lab setup** (4-8 students):

1. **Workstations** ($3k each × 4 = $12k):
   - RTX 4080 GPU, 64GB RAM, Ubuntu 22.04
   - For simulation and training

2. **Edge kits** ($700 each × 4 = $2.8k):
   - Jetson Orin Nano + RealSense + mic
   - One per student for deployment

3. **Shared robots**:
   - **Option A**: 1x Unitree Go2 ($2.8k) - locomotion testing
   - **Option B**: 1x Unitree G1 ($16k) - full humanoid

**Total**:
- Option A: $17.6k (with quadruped)
- Option B: $30.8k (with humanoid)

## DIY: Building Your Own Humanoid

### Is it feasible?

**Yes, but challenging**:
- Requires mechanical design (CAD)
- Electronics (motor controllers, power)
- Software (control algorithms)
- Time investment (6-12 months)

### Components

**Actuators** ($3,000-5,000):
- 20-30x Dynamixel servos ($100-300 each)
- OR custom BLDC motors + drivers

**Structure** ($500-1,000):
- 3D printed parts (PLA, ABS, PETG)
- Aluminum extrusions for frame
- Bearings, fasteners

**Electronics** ($500-1,000):
- Jetson Orin Nano ($249)
- Motor controller boards ($50-200 each)
- Battery (LiPo, 5-10 Ah)
- Wiring, connectors, PCBs

**Sensors** ($700):
- RealSense D435i, IMU, microphone (as before)

**Total**: $4,700-7,700 + hundreds of hours

**Challenges**:
- **Mechanical design**: Balance, strength, weight distribution
- **Dynamic walking**: Very hard (years of research)
- **Safety**: Risk of damage during testing

**Verdict**: Not recommended unless you have mechatronics expertise and time.

## Simulation vs. Real Robot Trade-offs

| Aspect | Simulation | Real Robot |
|--------|------------|------------|
| **Cost** | ~$700 (edge kit) | $2,800-16,000+ |
| **Safety** | Can't break anything | Risk of damage |
| **Iteration Speed** | Very fast (instant reset) | Slow (manual reset, charging) |
| **Realism** | Physics approximation | Ground truth |
| **Sim-to-Real Gap** | Models may not transfer | No gap (is reality) |
| **Scalability** | Run 100s in parallel | Limited by hardware |
| **Learning** | Great for algorithms | Essential for deployment |

**Best approach**: **Develop in simulation, validate on real robot**.

## When Do You NEED a Physical Robot?

**You can skip physical hardware if**:
- Focus is on algorithm development
- Deploying to simulation environments
- Budget constrained
- Safety concerns (humanoids can be dangerous)

**You NEED physical hardware if**:
- Validating sim-to-real transfer
- Deploying to production
- Publishing research (reviewers expect real-world validation)
- Learning from real-world edge cases

## Course Recommendation

### Minimum (Simulation Focus)

**$700 total**:
- Workstation with RTX GPU (or cloud: $200/quarter)
- Jetson Orin Nano edge kit
- Complete course with simulation only

### Ideal (Lab with Physical Testing)

**$3,500 total**:
- Workstation + edge kit ($3,700)
- Unitree Go2 quadruped ($2,800)
- Total: $6,500 (shared among students)

### Premium (Full Humanoid Lab)

**$20,000+ total**:
- Workstations + edge kits ($4,400)
- Unitree G1 humanoid ($16,000)
- Total: $20,400

## Next Steps

Now that you understand hardware options, let's compare cloud vs. on-premise infrastructure decisions.

👉 **[Next: Cloud vs. On-Premise →](cloud-vs-onpremise)**

---

:::tip Budget Planning

**80/20 Rule**: You can complete 80% of the course with simulation + edge kit ($700). The remaining 20% (physical locomotion validation) requires $3k-16k robot.

**Start small**: Begin with simulation. Add physical robot only if:
1. You've mastered simulation
2. You have specific need for physical validation
3. Budget and lab space available

:::

## Further Reading

- **Unitree Robotics**: https://www.unitree.com/
- **Robotis OP3**: https://emanual.robotis.com/docs/en/platform/op3/introduction/
- **DIY Humanoids**: "Open Source Biped Robot" by Will Cogley (YouTube)
- **RoboCup Humanoid League**: https://www.robocup.org/leagues/3 (competition robots)
