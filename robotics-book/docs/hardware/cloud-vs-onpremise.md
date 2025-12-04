---
sidebar_position: 4
title: Cloud vs. On-Premise Infrastructure
description: Comparing cloud and on-premise options for Physical AI development
keywords: [cloud computing, AWS, on-premise, GPU, cost analysis, infrastructure]
---

# Cloud vs. On-Premise Infrastructure

## The Infrastructure Decision

Building Physical AI systems requires significant computational resources. You have two main options:

1. **On-Premise** (High CapEx): Buy hardware upfront
2. **Cloud** (High OpEx): Rent compute by the hour

This chapter helps you choose based on budget, usage patterns, and requirements.

## Cost Comparison

### Scenario: 13-Week Course (1 Student)

#### On-Premise Option

**Initial Investment**:
- Workstation (RTX 4080, 64GB RAM): $3,500
- Jetson Orin Nano kit: $700
- **Total**: $4,200 upfront

**Quarterly Cost**: $0 (after initial purchase)
**Electricity**: ~$20-30/quarter (negligible)

**Total Course Cost**: $4,200

**Amortized over 3 years** (6 quarters):
- $4,200 / 6 = $700/quarter
- Plus hardware retains resale value (~$2,000 after 3 years)

#### Cloud Option

**Simulation** (AWS g5.2xlarge):
- Instance: $1.50/hour
- Usage: 10 hours/week × 13 weeks = 130 hours
- Cost: 130 × $1.50 = $195

**Storage** (EBS):
- 500GB for datasets/models: $50

**Edge Deployment** (Still need Jetson):
- Jetson Orin Nano kit: $700

**Total Course Cost**: $945 ($195 + $50 + $700)

### Breakeven Analysis

**On-premise breakeven**: 4-5 quarters of usage

```
Quarter 1: Cloud cheaper ($945 vs. $4,200)
Quarter 2: Cloud cheaper ($1,890 vs. $4,200)
Quarter 3: Cloud cheaper ($2,835 vs. $4,200)
Quarter 4: Tie ($3,780 vs. $4,200)
Quarter 5+: On-premise cheaper
```

**Conclusion**:
- **Short-term (1-2 quarters)**: Cloud is cheaper
- **Long-term (3+ quarters)**: On-premise is cheaper

## Detailed Comparison

### On-Premise Workstation

#### Advantages

1. **No ongoing costs**: Pay once, use forever
2. **Full control**: No usage limits, no throttling
3. **Low latency**: Local access, no network delay
4. **Privacy**: Data stays on your machine
5. **Offline capable**: No internet dependency
6. **Resale value**: Hardware retains 30-50% value after 3 years

#### Disadvantages

1. **High upfront cost**: $3,500-4,000
2. **Maintenance**: You handle hardware failures
3. **Limited scalability**: Can't easily add more GPUs
4. **Depreciation**: Hardware loses value over time
5. **Space/power**: Requires desk space, reliable power

#### Best For

- Students planning 2+ years of robotics work
- Labs with multiple students (shared resource)
- Developers working on long-term projects
- Those with reliable power and space

### Cloud Computing

#### Advantages

1. **Low barrier to entry**: No upfront cost
2. **Scalability**: Spin up 10 GPUs for parallel training
3. **Flexibility**: Pay only for what you use
4. **No maintenance**: Provider handles hardware
5. **Latest hardware**: Upgrade to new GPUs easily
6. **Global access**: Work from anywhere

#### Disadvantages

1. **Ongoing costs**: $200-300/quarter adds up
2. **Usage limits**: May need to manage quotas
3. **Latency**: Network delay for data transfer
4. **Internet dependency**: Unusable offline
5. **Data egress costs**: Downloading large datasets expensive
6. **No physical robot control**: Can't control real robot from cloud (latency)

#### Best For

- Short-term projects (1-2 quarters)
- Students with budget constraints (can't afford $4k upfront)
- Those needing massive parallelization (RL training)
- Travelers without permanent workspace

## Cloud Provider Comparison

### AWS (Amazon Web Services)

**GPU Instances**:
- **g5.xlarge**: 1x A10G (24GB VRAM), 4 vCPUs, 16GB RAM - $1.01/hour
- **g5.2xlarge**: 1x A10G (24GB VRAM), 8 vCPUs, 32GB RAM - $1.21/hour
- **g5.12xlarge**: 4x A10G (96GB VRAM), 48 vCPUs, 192GB RAM - $5.67/hour

**Storage**:
- **EBS (gp3)**: $0.08/GB/month
- **S3**: $0.023/GB/month (cheaper for datasets)

**Pros**:
- Largest selection of instance types
- Spot instances (up to 90% discount)
- Mature ecosystem (AMIs for Isaac Sim, ROS 2)

**Cons**:
- Complex pricing (data transfer, storage)
- Steeper learning curve

### Google Cloud Platform (GCP)

**GPU Instances**:
- **n1-standard-4 + T4**: 1x T4 (16GB VRAM), 4 vCPUs, 15GB RAM - $0.63/hour
- **n1-standard-8 + A100**: 1x A100 (40GB VRAM), 8 vCPUs, 30GB RAM - $3.67/hour

**Pros**:
- Simpler pricing
- Good for TensorFlow/JAX (Google tools)
- Preemptible VMs (80% discount)

**Cons**:
- Fewer robotics-specific resources
- Less Isaac Sim support

### Microsoft Azure

**GPU Instances**:
- **NC6s_v3**: 1x V100 (16GB VRAM), 6 vCPUs, 112GB RAM - $3.06/hour
- **NCads_A100_v4**: 1x A100 (80GB VRAM), 24 vCPUs, 220GB RAM - $3.67/hour

**Pros**:
- Good Windows support (if needed)
- Azure Kinect integration
- Student discounts (Azure for Students)

**Cons**:
- More expensive than AWS/GCP for equivalent specs

### Cost Optimization Tips

#### Use Spot/Preemptible Instances

**Spot instances** (AWS) / **Preemptible VMs** (GCP):
- 60-90% cheaper than on-demand
- Can be terminated anytime (when demand is high)
- Good for training (can checkpoint and resume)
- **Not good for** interactive development (annoying to be kicked off)

**Example**:
- AWS g5.2xlarge on-demand: $1.21/hour
- AWS g5.2xlarge spot: $0.36/hour (70% savings)

#### Stop Instances When Not Using

**Mistake**: Leave instance running 24/7
- Cost: $1.21 × 24 × 30 = $871/month

**Correct**: Stop instance when done
- Usage: 10 hours/week = 40 hours/month
- Cost: $1.21 × 40 = $48/month

**Savings**: $823/month (94%)

#### Use S3/Cloud Storage for Datasets

Don't keep large datasets on expensive EBS:
- **EBS**: $0.08/GB/month
- **S3**: $0.023/GB/month

**1TB dataset**:
- EBS: $80/month
- S3: $23/month
- **Savings**: $57/month

#### Right-Size Your Instance

Don't use overkill GPU for simple tasks:

| Task | Instance Needed | Cost |
|------|----------------|------|
| Debugging ROS 2 nodes | CPU-only (t3.medium) | $0.04/hour |
| Training simple CNN | g5.xlarge (1x A10G) | $1.01/hour |
| Training large VLA model | g5.12xlarge (4x A10G) | $5.67/hour |

**Mistake**: Using g5.12xlarge for all tasks → $5.67/hour
**Correct**: Use appropriate instance per task → $0.50/hour average

## Hybrid Approach (Recommended)

**Best of both worlds**:

### Phase 1: Development (Cloud)

- **Use cloud** for initial development and experimentation
- Low commitment, flexibility to try different approaches
- Cost: ~$50-100 for first month of exploration

### Phase 2: Production (On-Premise)

- **Buy workstation** once you're committed to the project
- Amortize cost over remaining course + future work
- Use cloud for occasional heavy parallelization

### Edge Deployment (Always Local)

- **Jetson Orin Nano** is a one-time $249 purchase
- Cannot be replaced by cloud (need local robot control)

## Decision Matrix

| Your Situation | Recommendation | Cost |
|----------------|---------------|------|
| **1 quarter course only** | Cloud + Jetson | $945 |
| **2-3 quarters of work** | Cloud or on-premise (tie) | $1,890-2,835 vs. $4,200 |
| **4+ quarters (degree program)** | On-premise + Jetson | $4,200 (cheaper long-term) |
| **Uncertain commitment** | Start cloud, switch later | Flexible |
| **Lab (5+ students)** | On-premise (shared) | $4,200 / 5 = $840/student |
| **Need massive parallelization** | Cloud (scale up/down) | Variable |
| **Traveling/no fixed workspace** | Cloud | $200-300/quarter |

## Cloud Setup Guide (Quick Start)

### AWS Setup for Isaac Sim

**Step 1**: Create AWS account
- Go to aws.amazon.com
- Sign up (requires credit card)

**Step 2**: Launch g5.2xlarge instance
```bash
# Use NVIDIA Deep Learning AMI (Ubuntu 22.04)
# Instance type: g5.2xlarge
# Storage: 100GB gp3 EBS
# Security group: Allow SSH (port 22)
```

**Step 3**: SSH and install Isaac Sim
```bash
ssh -i your-key.pem ubuntu@<instance-ip>

# Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# Install Isaac Sim via launcher
# (Follow GUI instructions)
```

**Step 4**: Install ROS 2
```bash
# Follow standard ROS 2 Humble installation
# (See Module 1 notes)
```

**Step 5**: Transfer code
```bash
# From local machine
scp -i your-key.pem -r my_robot_package ubuntu@<instance-ip>:~/
```

**Step 6**: Work
```bash
# SSH with X11 forwarding for GUI
ssh -X -i your-key.pem ubuntu@<instance-ip>

# Or use VNC for better performance
# Or run headless (no GUI)
```

**Step 7**: Stop instance when done
```bash
# In AWS Console: Stop instance (not terminate)
# Or via CLI:
aws ec2 stop-instances --instance-ids i-1234567890abcdef0
```

## Monitoring Costs

### AWS Cost Explorer

- AWS Console → Cost Management → Cost Explorer
- View daily/monthly spending
- Set budgets and alerts

**Example alert**:
- Alert if spending > $50/month
- Receive email notification

### Resource Tags

Tag resources for cost tracking:
```
Project: robotics-course
Student: john-doe
Module: isaac-sim
```

## Final Recommendation

### For This Course (13 weeks)

**Option A: Budget ($700)**
- Borrow/use lab workstation for simulation
- Buy Jetson Orin Nano kit ($700)
- Deploy to Jetson for edge AI

**Option B: Cloud ($945)**
- AWS/GCP for simulation (130 hours × $1.50 = $195)
- Jetson Orin Nano kit ($700)
- Storage ($50)

**Option C: On-Premise ($4,200)**
- Buy workstation ($3,500)
- Jetson Orin Nano kit ($700)
- Best long-term value if continuing robotics

### Recommendation

**Start with cloud** ($945):
- Low risk, try before committing
- After 1-2 months, decide if you want to invest in hardware
- If you continue, buy workstation (breakeven by month 5)

## Next Steps

You've completed the Hardware & Infrastructure section! Now explore the course assessments.

👉 **[Next: Assessments →](../assessments/ros2-project)**

---

:::tip Cost Saving Pro Tip

**Student discounts**:
- AWS Educate: $100 free credits
- Azure for Students: $100 free credits
- GCP Education: $50-300 credits
- NVIDIA Inception: Startups get cloud credits

**Total free credits**: $250-500 (covers 150-300 hours of GPU time)

:::

## Further Reading

- **AWS Pricing Calculator**: https://calculator.aws/
- **GCP Pricing Calculator**: https://cloud.google.com/products/calculator
- **Spot Instance Pricing**: https://aws.amazon.com/ec2/spot/pricing/
- **Cost Optimization Guide**: "AWS Cost Optimization" (AWS Well-Architected Framework)
