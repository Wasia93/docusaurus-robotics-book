---
sidebar_position: 5
title: Reinforcement Learning for Robotics
description: Training humanoid robots with RL in Isaac Sim
keywords: [reinforcement learning, RL, Isaac Sim, robot learning, sim-to-real]
---

# Reinforcement Learning for Humanoid Robots

## Introduction to RL for Robotics

**Reinforcement Learning (RL)** enables robots to learn behaviors through trial and error:

- **Exploration**: Try different actions
- **Feedback**: Receive rewards/penalties
- **Learning**: Improve policy to maximize cumulative reward

### Why RL for Humanoids?

**Traditional approach** (hand-coded):
- Program every behavior explicitly
- Fragile (breaks in new situations)
- Time-consuming to develop

**RL approach** (learned):
- Robot learns from experience
- Generalizes to new situations
- Discovers optimal behaviors automatically

**Use cases**:
- **Locomotion**: Learn to walk, run, navigate terrain
- **Manipulation**: Grasp diverse objects
- **Whole-body control**: Coordinate arms, legs, torso

## RL Fundamentals

### Markov Decision Process (MDP)

**Components**:
- **State (s)**: Robot configuration, sensor readings
- **Action (a)**: Motor commands
- **Reward (r)**: Scalar feedback (positive = good, negative = bad)
- **Policy (π)**: Mapping from state to action
- **Value function (V)**: Expected cumulative reward

**Goal**: Learn policy π* that maximizes expected cumulative reward.

### RL Algorithm Categories

| Algorithm | Type | On/Off-Policy | Use Case |
|-----------|------|---------------|----------|
| **PPO** | Policy Gradient | On-policy | General-purpose, stable |
| **SAC** | Actor-Critic | Off-policy | Continuous control |
| **TD3** | Actor-Critic | Off-policy | High-dimensional |
| **DQN** | Value-based | Off-policy | Discrete actions |
| **TRPO** | Policy Gradient | On-policy | Safe, monotonic improvement |

**Most popular for robotics**: **PPO** (Proximal Policy Optimization) - stable and sample-efficient.

## RL in Isaac Sim

**Isaac Sim** is ideal for RL:
- **Fast simulation**: Train 1000x faster than real-time
- **Parallel environments**: Run 100s of robots simultaneously
- **Domain randomization**: Vary physics, visuals for robustness
- **Synthetic data**: No need for real-world data collection

### Isaac Gym

**Isaac Gym**: NVIDIA's RL platform (part of Isaac Sim).

**Features**:
- GPU-accelerated physics (PhysX)
- Tensor-based API (direct PyTorch integration)
- Massive parallelization (4096+ environments)
- Pre-built humanoid tasks

## Example Task: Humanoid Walk

**Goal**: Train humanoid to walk forward.

### State Space

**Observation** (what robot senses):
```python
observation = [
    # Joint positions (30 values for 30 DOF)
    joint_positions,  # rad

    # Joint velocities (30 values)
    joint_velocities,  # rad/s

    # Torso orientation (4 values: quaternion)
    torso_orientation,

    # Torso linear velocity (3 values)
    torso_velocity,

    # Torso angular velocity (3 values)
    torso_angular_velocity,

    # Contact forces (4 values: both feet)
    left_foot_contact,
    right_foot_contact,
]
# Total: ~70 observations
```

### Action Space

**Action** (what robot controls):
```python
action = [
    # Target joint positions (PD control)
    # OR target joint torques (direct torque control)

    # For 30 DOF humanoid: 30 values
]

# Actions are typically in range [-1, 1] (normalized)
```

### Reward Function

**Reward** guides learning:

```python
def compute_reward(obs, action):
    # Reward forward velocity
    reward_forward = obs['torso_velocity'][0]  # x-axis velocity

    # Penalty for falling
    reward_upright = 1.0 if obs['torso_height'] > 0.8 else -10.0

    # Penalty for high energy (smooth movements)
    reward_energy = -0.01 * np.sum(action ** 2)

    # Penalty for orientation deviation
    reward_orientation = -np.abs(obs['torso_roll']) - np.abs(obs['torso_pitch'])

    # Total reward
    reward = (
        2.0 * reward_forward +
        1.0 * reward_upright +
        0.5 * reward_energy +
        1.0 * reward_orientation
    )

    return reward
```

**Key principles**:
- **Reward progress**: Forward velocity
- **Penalize failure**: Falling, tipping over
- **Encourage efficiency**: Low energy consumption
- **Maintain balance**: Upright orientation

## Training with PPO

### PPO Algorithm

**PPO** (Proximal Policy Optimization):
1. Collect trajectories using current policy
2. Compute advantage estimates (how much better than average)
3. Update policy with clipped objective (prevent large updates)
4. Repeat until convergence

### Isaac Gym PPO Example

```python
from omni.isaac.gym.vec_env import VecEnvBase
import torch
import torch.nn as nn
from torch.distributions import Normal

class HumanoidWalkEnv(VecEnvBase):
    def __init__(self, cfg, sim_device, graphics_device_id, headless):
        self.cfg = cfg
        self.num_envs = cfg["env"]["numEnvs"]
        self.num_obs = 70
        self.num_actions = 30

        super().__init__(cfg, sim_device, graphics_device_id, headless)

        # Buffers
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=self.device)
        self.reward_buf = torch.zeros(self.num_envs, device=self.device)
        self.reset_buf = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

    def step(self, actions):
        # Apply actions to robots
        self.apply_actions(actions)

        # Step simulation
        self.sim.step()

        # Compute observations
        self.compute_observations()

        # Compute rewards
        self.compute_rewards()

        # Check for resets (fell, timeout)
        self.check_termination()

        return self.obs_buf, self.reward_buf, self.reset_buf, {}

    def compute_observations(self):
        # Get joint states from simulation
        joint_pos = self.get_joint_positions()
        joint_vel = self.get_joint_velocities()
        torso_state = self.get_torso_state()

        # Concatenate into observation
        self.obs_buf = torch.cat([joint_pos, joint_vel, torso_state], dim=-1)

    def compute_rewards(self):
        # Get relevant states
        torso_vel = self.obs_buf[:, 60:63]  # Indices for velocity
        torso_height = self.obs_buf[:, 63]

        # Reward forward velocity
        reward_forward = torso_vel[:, 0]  # x-axis

        # Penalty for falling
        reward_upright = torch.where(
            torso_height > 0.8,
            torch.ones_like(torso_height),
            -10.0 * torch.ones_like(torso_height)
        )

        # Total reward
        self.reward_buf = 2.0 * reward_forward + reward_upright

    def check_termination(self):
        # Reset if fallen or timeout
        torso_height = self.obs_buf[:, 63]
        fallen = torso_height < 0.5

        self.reset_buf = torch.where(
            fallen,
            torch.ones_like(self.reset_buf),
            self.reset_buf
        )

    def reset(self):
        # Reset robots to initial state
        self.set_initial_state()
        self.compute_observations()
        return self.obs_buf
```

### PPO Training Script

```python
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

# Create environment
env = HumanoidWalkEnv(cfg, sim_device='cuda:0', graphics_device_id=0, headless=False)

# Create PPO agent
model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    verbose=1,
    device='cuda'
)

# Train
model.learn(total_timesteps=10_000_000)

# Save model
model.save("humanoid_walk_ppo")
```

**Training time**: 1-2 hours on RTX 4080 (10M timesteps).

### Parallelization

**Key to fast training**: Run many environments simultaneously.

```python
# Single environment: 10 FPS, 10M steps = 11 days
# 1024 parallel environments: 10 FPS × 1024 = 10,240 steps/sec
# 10M steps / 10,240 = 16 minutes!
```

**Isaac Gym** can run 4096+ environments on a single GPU.

## Domain Randomization

**Problem**: Policies trained in simulation don't work on real robot (sim-to-real gap).

**Solution**: **Domain Randomization** - vary simulation parameters during training.

### Randomize Physics

```python
# Mass randomization
for link in humanoid.links:
    link.mass *= np.random.uniform(0.8, 1.2)

# Friction randomization
for contact in humanoid.contacts:
    contact.friction = np.random.uniform(0.5, 1.5)

# Joint damping randomization
for joint in humanoid.joints:
    joint.damping *= np.random.uniform(0.8, 1.2)
```

### Randomize Visuals

```python
# Lighting randomization
light.intensity = np.random.uniform(500, 2000)
light.color = np.random.uniform([0.8, 0.8, 0.8], [1.0, 1.0, 1.0])

# Texture randomization
robot.material.base_color = np.random.uniform([0, 0, 0], [1, 1, 1])
robot.material.roughness = np.random.uniform(0.0, 1.0)
```

### Randomize Observations

```python
# Add noise to observations
obs_noisy = obs + np.random.normal(0, 0.01, size=obs.shape)

# Simulate sensor lag
obs_delayed = obs_history[t - lag]
```

**Result**: Robust policy that generalizes to real robot.

## Sim-to-Real Transfer

### Workflow

1. **Train in simulation** (Isaac Sim):
   - Use domain randomization
   - Train for millions of timesteps
   - Evaluate in varied simulated conditions

2. **Export policy** (neural network weights):
   ```python
   model.save("humanoid_walk_policy.pth")
   ```

3. **Deploy to real robot** (Jetson Orin Nano):
   ```python
   model = torch.load("humanoid_walk_policy.pth")
   model.eval()

   # Inference loop
   obs = get_robot_state()
   action = model(obs)
   execute_action(action)
   ```

4. **Fine-tune** (optional):
   - Collect small amount of real-world data
   - Fine-tune policy on real robot

### Challenges

**Sim-to-real gap sources**:
- Physics approximation (contacts, friction)
- Sensor noise (real sensors noisier than simulated)
- Actuator dynamics (delays, backlash)
- Unmodeled effects (cable drag, wear)

**Mitigations**:
- Domain randomization (covers wide range of conditions)
- System identification (match simulation to real robot)
- Residual learning (learn correction on top of sim policy)

## Pre-Trained Humanoid Skills

**Isaac Gym** includes pre-trained policies:
- **Walking**: Forward, backward, turning
- **Balancing**: Maintain upright under pushes
- **Reaching**: Arm control to target positions
- **Grasping**: Hand manipulation

**Use pre-trained policies** as initialization for new tasks (transfer learning).

## Advanced: Hierarchical RL

**For complex tasks**: Combine multiple skills.

```
High-level policy: "Navigate to kitchen"
    ↓
Mid-level skills: [Walk forward, Turn left, Open door]
    ↓
Low-level control: Joint torques
```

**Benefits**:
- Faster learning (reuse low-level skills)
- Better generalization
- Interpretable behavior

## Monitoring Training

### TensorBoard

```python
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter()

for episode in range(num_episodes):
    # Training...
    writer.add_scalar('Reward/Episode', episode_reward, episode)
    writer.add_scalar('Length/Episode', episode_length, episode)

# View in browser
# tensorboard --logdir=runs
```

**Metrics to track**:
- Episode reward (should increase)
- Episode length (should increase)
- Policy loss (should decrease)
- Value loss (should decrease)

### Evaluation

```python
# Evaluate trained policy
obs = env.reset()
total_reward = 0

for step in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _ = env.step(action)
    total_reward += reward

    if done:
        break

print(f"Evaluation reward: {total_reward}")
```

## Sample Efficiency

**Challenge**: RL requires millions of samples.

**Strategies**:
- **Off-policy algorithms** (SAC, TD3): Reuse old data
- **Model-based RL**: Learn world model, plan in imagination
- **Imitation learning**: Bootstrap from demonstrations
- **Curriculum learning**: Start easy, gradually increase difficulty

## Next Steps

You've learned RL for training humanoid behaviors. Next, explore synthetic data generation for perception.

👉 **[Next: Synthetic Data Generation →](synthetic-data)**

---

:::tip RL Tips

1. **Start simple**: Train basic behaviors first (balancing before walking)
2. **Tune reward function**: Most important factor for success
3. **Use domain randomization**: Essential for sim-to-real
4. **Parallelize**: Run 100s of environments for fast training
5. **Monitor training**: Use TensorBoard to debug

:::

## Lab Exercise

**Train humanoid to walk forward**:

1. **Setup Isaac Gym**:
   - Install Isaac Sim
   - Load humanoid environment
   - Configure observation and action spaces

2. **Define reward function**:
   - Reward forward velocity
   - Penalize falling
   - Penalize high energy

3. **Train with PPO**:
   - 1024 parallel environments
   - Train for 1M timesteps
   - Monitor reward in TensorBoard

4. **Evaluate policy**:
   - Test in simulation
   - Measure: average velocity, success rate, energy efficiency

5. **Domain randomization**:
   - Add mass randomization
   - Add friction randomization
   - Retrain and compare robustness

**Bonus**: Deploy trained policy to simulated humanoid in Gazebo.
