# Manipulation and Grasping

## Overview

Manipulation and grasping enable humanoid robots to interact with objects in their environment. This module covers the principles of robot grasping, force control, and dexterous manipulation.

## Grasping Fundamentals

### Types of Grasps

#### 1. Power Grasps
- **Cylindrical**: Wrapping fingers around cylinder (e.g., holding a bottle)
- **Spherical**: Cupping hand around sphere (e.g., holding a ball)
- **Hook**: Fingers hooked (e.g., carrying a bag)

#### 2. Precision Grasps
- **Pinch**: Thumb and fingertip opposition (e.g., picking a coin)
- **Tripod**: Thumb and two fingers (e.g., holding a pen)
- **Lateral**: Thumb against side of finger (e.g., holding a key)

### Grasp Quality Metrics

```python
def grasp_quality_epsilon(contact_points, contact_normals, friction_coeff):
    """
    Compute grasp quality (epsilon metric)
    Measures minimum force needed to resist arbitrary wrench
    """

    # Build grasp matrix G
    G = compute_grasp_matrix(contact_points, contact_normals)

    # Compute minimum singular value
    U, s, Vh = np.linalg.svd(G)

    # Epsilon metric is minimum singular value
    epsilon = np.min(s)

    return epsilon

def compute_grasp_matrix(contact_points, normals):
    """Compute grasp matrix from contacts"""

    G = []
    for point, normal in zip(contact_points, normals):
        # Force wrench from this contact
        # [force_x, force_y, force_z, torque_x, torque_y, torque_z]

        # Simplified for point contacts with friction
        wrench = np.zeros((6, 3))
        wrench[:3, :] = np.eye(3)  # Force
        wrench[3:, :] = skew_symmetric(point)  # Torque = r x F

        G.append(wrench)

    return np.hstack(G)

def skew_symmetric(v):
    """Convert vector to skew-symmetric matrix"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
```

## Grasp Planning

### Grasp Pose Generation

```python
import trimesh
import numpy as np

class GraspPlanner:
    def __init__(self, mesh_path):
        self.mesh = trimesh.load(mesh_path)

    def sample_grasp_poses(self, num_samples=100):
        """Sample candidate grasp poses on object surface"""

        grasps = []

        for _ in range(num_samples):
            # Sample point on surface
            point, face_idx = self.mesh.sample(1, return_index=True)
            point = point[0]

            # Surface normal at point
            normal = self.mesh.face_normals[face_idx[0]]

            # Approach direction (opposite to normal)
            approach = -normal

            # Create grasp pose
            grasp_pose = self.create_grasp_pose(point, approach)
            grasps.append(grasp_pose)

        return grasps

    def create_grasp_pose(self, position, approach):
        """Create 4x4 grasp transformation matrix"""

        # Gripper frame: z-axis along approach direction
        z_axis = approach / np.linalg.norm(approach)

        # Choose arbitrary x-axis perpendicular to z
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross(z_axis, [0, 0, 1])
        else:
            x_axis = np.cross(z_axis, [1, 0, 0])

        x_axis = x_axis / np.linalg.norm(x_axis)

        # y-axis completes right-hand frame
        y_axis = np.cross(z_axis, x_axis)

        # Build transformation matrix
        T = np.eye(4)
        T[:3, 0] = x_axis
        T[:3, 1] = y_axis
        T[:3, 2] = z_axis
        T[:3, 3] = position

        return T

    def evaluate_grasps(self, grasps):
        """Rank grasps by quality metrics"""

        scored_grasps = []

        for grasp in grasps:
            # Simulate contact points
            contacts = self.simulate_contacts(grasp)

            if len(contacts) < 2:
                continue  # Need at least two contacts

            # Compute quality
            quality = self.compute_grasp_quality(contacts)

            # Check other criteria
            if self.is_collision_free(grasp) and self.is_reachable(grasp):
                scored_grasps.append((grasp, quality))

        # Sort by quality
        scored_grasps.sort(key=lambda x: x[1], reverse=True)

        return scored_grasps
```

### Grasp Synthesis with Deep Learning

```python
import torch
import torch.nn as nn

class GraspQualityNet(nn.Module):
    """Neural network to predict grasp quality from depth image"""

    def __init__(self):
        super().__init__()

        self.conv_layers = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2),
            nn.ReLU()
        )

        self.fc_layers = nn.Sequential(
            nn.Linear(128 * 7 * 7, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, 1),
            nn.Sigmoid()  # Quality score between 0 and 1
        )

    def forward(self, depth_image):
        x = self.conv_layers(depth_image)
        x = x.view(x.size(0), -1)
        quality = self.fc_layers(x)
        return quality

# Usage
model = GraspQualityNet()
depth_image = capture_depth_image()
grasp_quality = model(depth_image)
```

## Force Control

### Impedance Control

Control robot compliance (stiffness and damping):

```python
class ImpedanceController:
    def __init__(self, K_stiffness, D_damping):
        self.K = np.diag(K_stiffness)  # Stiffness matrix
        self.D = np.diag(D_damping)     # Damping matrix

    def compute_force(self, x_desired, x_current, v_current):
        """Compute desired force for impedance control"""

        # Position error
        error = x_desired - x_current

        # Impedance control law: F = K * error - D * velocity
        F = self.K @ error - self.D @ v_current

        return F

# Example: Soft contact
# High stiffness, low damping = rigid
stiff_controller = ImpedanceController(
    K_stiffness=[1000, 1000, 1000],
    D_damping=[50, 50, 50]
)

# Low stiffness, high damping = compliant
compliant_controller = ImpedanceController(
    K_stiffness=[100, 100, 100],
    D_damping=[200, 200, 200]
)
```

### Hybrid Force-Position Control

Control position in some directions, force in others:

```python
class HybridController:
    def __init__(self, selection_matrix):
        self.S = selection_matrix  # Diagonal: 1 = force control, 0 = position control

    def compute_control(self, x_desired, F_desired, x_current, F_current):
        """Hybrid force-position control"""

        # Position error
        x_error = x_desired - x_current

        # Force error
        F_error = F_desired - F_current

        # Position control in selected directions
        u_position = self.Kp * (np.eye(3) - self.S) @ x_error

        # Force control in selected directions
        u_force = self.Kf * self.S @ F_error

        # Combined control
        u = u_position + u_force

        return u

# Example: Pushing down (control z-force, free x-y position)
controller = HybridController(
    selection_matrix=np.diag([0, 0, 1])  # Force control in z only
)
```

## Dexterous Manipulation

### Fingertip Control

```python
class DexterousHand:
    def __init__(self, num_fingers=5):
        self.num_fingers = num_fingers

    def fingertip_ik(self, target_positions):
        """Inverse kinematics for fingertips"""

        joint_angles = []

        for finger_idx, target in enumerate(target_positions):
            # IK for each finger
            angles = self.finger_ik(finger_idx, target)
            joint_angles.append(angles)

        return joint_angles

    def compute_grasp_matrix(self, fingertip_positions):
        """Compute grasp matrix for current fingertip configuration"""

        # Similar to earlier grasp matrix computation
        # but for multi-finger hand

        return grasp_matrix
```

### In-Hand Manipulation

Reorienting objects within the hand:

```python
def plan_in_hand_manipulation(object_pose_current, object_pose_desired):
    """
    Plan finger motions to reorient object
    """

    # Compute relative rotation
    rotation_needed = compute_relative_rotation(
        object_pose_current,
        object_pose_desired
    )

    # Plan sequence of finger gaits
    # Similar to walking: some fingers maintain contact, others reposition

    manipulation_plan = []

    # Phase 1: Fingers 1,2,3 hold, fingers 4,5 reposition
    manipulation_plan.append({
        'holding_fingers': [0, 1, 2],
        'moving_fingers': [3, 4],
        'object_rotation': rotation_needed * 0.5
    })

    # Phase 2: Fingers 4,5 hold, fingers 1,2,3 reposition
    manipulation_plan.append({
        'holding_fingers': [3, 4],
        'moving_fingers': [0, 1, 2],
        'object_rotation': rotation_needed * 0.5
    })

    return manipulation_plan
```

## Vision-Based Grasping

### RGB-D Grasp Detection

```python
class VisionGraspPlanner:
    def __init__(self, camera):
        self.camera = camera

    def detect_grasp_from_rgbd(self):
        """Detect graspable objects and plan grasps"""

        # Capture RGB-D image
        rgb = self.camera.get_rgb()
        depth = self.camera.get_depth()

        # Segment objects
        masks = self.segment_objects(rgb)

        grasps = []
        for mask in masks:
            # Extract object point cloud
            points = self.depth_to_pointcloud(depth, mask)

            # Fit primitive shape (cylinder, box, sphere)
            shape, params = self.fit_shape(points)

            # Generate grasp based on shape
            grasp = self.shape_based_grasp(shape, params)
            grasps.append(grasp)

        return grasps

    def segment_objects(self, rgb_image):
        """Segment objects using vision model"""
        # Use Segment Anything, Detectron2, or similar
        pass

    def depth_to_pointcloud(self, depth, mask):
        """Convert depth image to 3D point cloud"""

        points = []
        h, w = depth.shape

        for v in range(h):
            for u in range(w):
                if mask[v, u]:
                    z = depth[v, u]
                    x = (u - self.camera.cx) * z / self.camera.fx
                    y = (v - self.camera.cy) * z / self.camera.fy
                    points.append([x, y, z])

        return np.array(points)
```

## Practical Exercises

### Exercise 1: Grasp Planning

Implement basic grasp planning:

```python
# TODO: Student implementation
# 1. Load object mesh
# 2. Sample grasp candidates
# 3. Evaluate grasp quality
# 4. Visualize top 10 grasps
```

### Exercise 2: Force Control

Implement impedance control for compliant grasping:

```python
# TODO: Student implementation
# 1. Set up robot with gripper
# 2. Implement impedance controller
# 3. Grasp deformable object (sponge)
# 4. Measure contact forces
```

### Exercise 3: Vision-Based Grasping

Implement full vision-to-grasp pipeline:

```python
# TODO: Student implementation
# 1. Capture RGB-D image
# 2. Detect objects
# 3. Plan grasp for each object
# 4. Execute best grasp
```

## Key Takeaways

- Grasp quality depends on contact geometry and friction
- Multiple grasp planning approaches: analytical, sampling, learning
- Force control enables safe interaction with objects
- Dexterous manipulation requires coordinating multiple fingers
- Vision is essential for detecting and grasping unknown objects
- Simulation is crucial for testing grasp strategies safely

## Resources

- [Grasp Planning Tutorial](https://manipulation.csail.mit.edu/pick.html)
- [GraspIt! Simulator](https://graspit-simulator.github.io/)
- [Dex-Net: Deep Learning for Grasp Planning](https://berkeleyautomation.github.io/dex-net/)

## Next Steps

Continue to [Human-Robot Interaction Design](./hri-design.md) to learn about natural interaction patterns.
