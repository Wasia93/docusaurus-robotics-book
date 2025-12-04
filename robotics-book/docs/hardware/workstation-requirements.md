---
sidebar_position: 1
title: Workstation Requirements
description: Hardware requirements for Physical AI development
keywords: [hardware, GPU, RTX, NVIDIA, Isaac Sim, workstation]
---

# Workstation Requirements for Physical AI

## The Computational Challenge

This course is **technically demanding**, sitting at the intersection of three heavy computational loads:

1. **Physics Simulation** (Isaac Sim/Gazebo)
2. **Visual Perception** (SLAM/Computer Vision)
3. **Generative AI** (LLMs/VLA)

Unlike purely digital AI courses, Physical AI requires specialized hardware to run physics simulations, train models, and deploy to robots.

## The "Digital Twin" Workstation (Required)

This is the **most critical component**. NVIDIA Isaac Sim is an Omniverse application that requires **RTX (Ray Tracing)** capabilities. Standard laptops (MacBooks or non-RTX Windows machines) **will not work**.

### Minimum Specifications

| Component | Minimum Spec | Recommended | Notes |
|-----------|-------------|-------------|-------|
| **GPU** | NVIDIA RTX 4070 Ti (12GB VRAM) | RTX 4090 (24GB VRAM) | **The bottleneck** |
| **CPU** | Intel Core i7 13th Gen or AMD Ryzen 9 | i9 14th Gen or Ryzen 9 7950X | Physics calculations |
| **RAM** | 32 GB DDR5 | 64 GB DDR5 | 32GB is absolute minimum |
| **Storage** | 512 GB NVMe SSD | 1 TB NVMe SSD | Fast I/O for datasets |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | **Linux required** |

### GPU Requirements Explained

**Why RTX?**
- NVIDIA Isaac Sim requires **RTX** (ray tracing cores)
- Need high VRAM to load USD (Universal Scene Description) assets for robot and environment
- Must run VLA (Vision-Language-Action) models simultaneously

**VRAM Requirements**:
- **12GB**: Can run Isaac Sim with simple scenes
- **16GB**: Comfortable for most course projects
- **24GB** (RTX 3090/4090): Smooth "Sim-to-Real" training, complex scenes

**Incompatible GPUs** (do not have RTX):
- GTX series (GTX 1080, 1660, etc.)
- AMD GPUs (not supported by Isaac Sim)
- Intel Arc (no Isaac Sim support)
- Integrated graphics (Intel UHD, AMD Radeon Vega)

### CPU Requirements Explained

**Why powerful CPU?**
- **Physics calculations**: Rigid body dynamics in Gazebo/Isaac are CPU-intensive
- **Parallel simulations**: Running multiple robot instances for training
- **ROS 2 nodes**: Many nodes running simultaneously

**Recommended**:
- **Intel**: Core i7-13700K or i9-13900K (13th/14th gen)
- **AMD**: Ryzen 9 7900X or 7950X

### RAM Requirements

**Why 64GB?**
- Isaac Sim alone: 8-16 GB
- ROS 2 + Gazebo: 4-8 GB
- VLA models: 8-16 GB
- OS + background: 4-8 GB
- **Total**: 24-48 GB (64GB provides headroom)

**32GB warning**:
- Will work for simple scenes
- **Will crash** during complex scene rendering or multi-robot simulations
- Requires careful resource management

### Operating System

**Ubuntu 22.04 LTS is mandatory**:
- ROS 2 Humble is native to Linux
- NVIDIA Isaac Sim runs on Windows, but ROS 2 integration is friction-free on Linux
- Dual-booting or dedicated Linux machines recommended

**Options**:
1. **Dual-boot Ubuntu + Windows**: Best performance
2. **Dedicated Ubuntu machine**: Ideal for course
3. **WSL2 (Windows Subsystem for Linux)**: Limited, not recommended for GPU work
4. **Virtual Machine**: Too slow for real-time simulation, not recommended

### Recommended Pre-Built Workstations

**Budget Option (~$2,500)**:
- GPU: RTX 4070 Ti (12GB)
- CPU: i7-13700K
- RAM: 32GB DDR5
- Storage: 1TB NVMe SSD

**Ideal Option (~$3,500-4,000)**:
- GPU: RTX 4080/4090 (16-24GB)
- CPU: i9-14900K or Ryzen 9 7950X
- RAM: 64GB DDR5
- Storage: 2TB NVMe SSD

**Example Builds**:
- **System76 Thelio** (ships with Ubuntu pre-installed)
- **Custom build** via PCPartPicker
- **NVIDIA DGX Station** (if budget allows, overkill for this course)

## Cloud Workstations (Alternative)

If you cannot invest in local hardware, use cloud-based GPU instances.

### AWS Option

**Instance Type**: `g5.2xlarge` or `g6e.xlarge`
- GPU: NVIDIA A10G (24GB VRAM)
- CPU: 8 vCPUs
- RAM: 32GB
- NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI)

**Cost Calculation**:
- Instance cost: ~$1.50/hour (spot/on-demand mix)
- Usage: 10 hours/week × 13 weeks = 130 hours
- Storage (EBS volumes): ~$25/quarter
- **Total**: ~$220 per quarter

**Pros**:
- No upfront hardware cost
- Access to powerful GPUs
- Scalable (can upgrade instance size)

**Cons**:
- Ongoing operational cost
- Latency (cloud to local)
- **Cannot control real robot from cloud** (latency issues)
- Internet dependency

### Azure and Google Cloud

Similar offerings:
- **Azure**: NC-series (NVIDIA Tesla/A100)
- **Google Cloud**: Compute Engine with GPU

**Cost**: Similar to AWS (~$1-2/hour for GPU instances)

## The Latency Trap (Hidden Cost)

**Problem**: Simulating in the cloud works well, but **controlling a real robot from a cloud instance is dangerous** due to latency.

**Solution**:
1. **Train in the cloud**: Use cloud GPU for Isaac Sim training
2. **Download the model**: Get trained model weights
3. **Deploy to edge device**: Flash model to local Jetson (see Edge Computing section)

**Workflow**:
```
Cloud (Isaac Sim) → Train VLA model → Export weights
    ↓
Local Jetson Orin → Load weights → Control real robot
```

## Laptop Compatibility

### Can I use a laptop?

**Gaming laptops with RTX mobile GPUs**:
- ✅ RTX 4070 Mobile or higher (8GB+ VRAM)
- ⚠️ Thermal throttling is a concern
- ⚠️ Battery life will be poor (1-2 hours under load)
- ✅ Can work for lighter simulations

**MacBooks**:
- ❌ **No support** for NVIDIA Isaac Sim (requires NVIDIA GPU)
- ⚠️ ROS 2 support on macOS is limited
- Option: Use cloud instances for simulation, Mac for development

**Recommended**: If buying new, invest in a desktop workstation, not a laptop.

## Hybrid Approach (Recommended for Budget)

**Best of both worlds**:
1. **Cloud for simulation** (~$200-300/quarter):
   - AWS g5.2xlarge for Isaac Sim
   - Train models in the cloud
   - No upfront hardware cost

2. **Local edge device for deployment** (~$700 one-time):
   - NVIDIA Jetson Orin Nano (see Edge Computing section)
   - Deploy trained models locally
   - Control real robot hardware

**Total cost**: ~$200-300/quarter + $700 one-time = **$900-1000 for course**

## Summary: Hardware Decision Matrix

| Scenario | Recommendation | Cost |
|----------|---------------|------|
| **Have RTX 4070 Ti+ desktop** | Use existing hardware | $0 |
| **Building new workstation** | RTX 4080/4090 + Ubuntu 22.04 | $3,000-4,000 |
| **Budget-constrained** | Cloud (AWS g5.2xlarge) + Jetson | $900-1,000/quarter |
| **Have Mac/non-RTX laptop** | Must use cloud instances | $200-300/quarter |
| **Have gaming laptop (RTX Mobile)** | Can work, monitor thermals | $0 |

## Verification Checklist

Before starting the course, verify:

- [ ] GPU: NVIDIA RTX 4070 Ti or higher (12GB+ VRAM)
- [ ] OS: Ubuntu 22.04 LTS installed
- [ ] RAM: 32GB minimum (64GB recommended)
- [ ] Storage: 500GB+ free space
- [ ] Internet: Stable connection for downloading models and datasets

**Test Isaac Sim**:
```bash
# Install Omniverse Launcher
# Download Isaac Sim
# Run sample scene to verify GPU compatibility
```

If any of the above are not met, consider the cloud alternative.

## Next Steps

Now that you understand workstation requirements, learn about the edge computing kits for physical robot deployment.

👉 **[Next: Edge Computing Kits →](edge-computing)**

---

:::warning Critical Hardware Requirement

NVIDIA Isaac Sim **requires** an RTX GPU. If you do not have one, you **must** use cloud instances. Standard laptops and non-RTX GPUs will not work for this course.

:::

:::tip Budget Planning

If cost is a concern:
1. **Borrow or rent**: Lab access, university workstations
2. **Cloud credits**: AWS/Azure/GCP often offer student credits ($100-300)
3. **Used hardware**: RTX 3090 (24GB) can be found used for ~$800-1,000

:::

## Further Reading

- **System Builds**:
  - PCPartPicker: https://pcpartpicker.com/
  - r/buildapc: Community advice on PC builds

- **Isaac Sim Requirements**:
  - NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim/

- **Cloud GPU Comparisons**:
  - "Cloud GPU Price Comparison 2024" (search for latest)

- **Linux Installation**:
  - Ubuntu 22.04 installation guide: https://ubuntu.com/tutorials/install-ubuntu-desktop
