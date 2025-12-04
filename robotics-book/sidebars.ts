import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Physical AI & Humanoid Robotics Book Structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: '🌟 Introduction (Weeks 1-2)',
      collapsed: false,
      items: [
        'foundations/intro-physical-ai',
        'foundations/embodied-intelligence',
        'foundations/humanoid-landscape',
        'foundations/sensor-systems',
      ],
    },
    {
      type: 'category',
      label: '🧠 Module 1: The Robotic Nervous System (Weeks 3-5)',
      collapsed: false,
      items: [
        'module1-ros2/ros2-overview',
        'module1-ros2/nodes-topics-services',
        'module1-ros2/python-ros2',
        'module1-ros2/urdf-humanoids',
        'module1-ros2/launch-files',
      ],
    },
    {
      type: 'category',
      label: '🎮 Module 2: The Digital Twin (Weeks 6-7)',
      collapsed: false,
      items: [
        'module2-simulation/gazebo-setup',
        'module2-simulation/physics-simulation',
        'module2-simulation/unity-rendering',
        'module2-simulation/sensor-simulation',
        'module2-simulation/urdf-sdf',
      ],
    },
    {
      type: 'category',
      label: '🚀 Module 3: The AI-Robot Brain (Weeks 8-10)',
      collapsed: false,
      items: [
        'module3-isaac/isaac-sim',
        'module3-isaac/synthetic-data',
        'module3-isaac/isaac-ros',
        'module3-isaac/vslam-navigation',
        'module3-isaac/nav2-bipedal',
        'module3-isaac/reinforcement-learning',
      ],
    },
    {
      type: 'category',
      label: '🗣️ Module 4: Vision-Language-Action (Weeks 11-13)',
      collapsed: false,
      items: [
        'module4-vla/voice-to-action',
        'module4-vla/llm-robotics',
        'module4-vla/cognitive-planning',
        'module4-vla/humanoid-kinematics',
        'module4-vla/bipedal-locomotion',
        'module4-vla/manipulation-grasping',
        'module4-vla/hri-design',
        'module4-vla/capstone-project',
      ],
    },
    {
      type: 'category',
      label: '🛠️ Hardware & Infrastructure',
      collapsed: true,
      items: [
        'hardware/workstation-requirements',
        'hardware/edge-computing',
        'hardware/robot-lab-options',
        'hardware/cloud-vs-onpremise',
      ],
    },
    {
      type: 'category',
      label: '📊 Assessments',
      collapsed: true,
      items: [
        'assessments/ros2-project',
        'assessments/gazebo-implementation',
        'assessments/isaac-perception',
        'assessments/capstone',
      ],
    },
  ],
};

export default sidebars;
