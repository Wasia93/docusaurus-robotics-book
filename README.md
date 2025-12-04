# Physical AI & Humanoid Robotics Course

A comprehensive Docusaurus-based course book covering Physical AI, Humanoid Robotics, ROS 2, Simulation, NVIDIA Isaac, and Vision-Language-Action systems.

## 🤖 Course Overview

This course bridges the gap between digital AI and physical robotics, teaching students to design, simulate, and deploy humanoid robots capable of natural human interactions.

### Modules Covered

- **Module 1: The Robotic Nervous System (ROS 2)** - Middleware, nodes, topics, services, URDF
- **Module 2: The Digital Twin (Gazebo & Unity)** - Physics simulation, sensors, environments
- **Module 3: The AI-Robot Brain (NVIDIA Isaac)** - Isaac Sim, Isaac ROS, VSLAM, synthetic data
- **Module 4: Vision-Language-Action** - Voice control, LLM integration, humanoid control, HRI

## 🚀 Quick Start

### Prerequisites

- Node.js 20.0 or higher
- npm or yarn

### Local Development

```bash
cd robotics-book
npm install
npm start
```

This starts a local development server at `http://localhost:3000`.

### Build

```bash
cd robotics-book
npm run build
```

This generates static content into the `build` directory.

## 📚 Documentation Structure

```
robotics-book/docs/
├── foundations/          # Physical AI fundamentals
├── module1-ros2/         # ROS 2 development
├── module2-simulation/   # Gazebo and Unity
├── module3-isaac/        # NVIDIA Isaac platform
├── module4-vla/          # Vision-Language-Action
├── hardware/             # Hardware requirements
└── assessments/          # Course assessments
```

## 🌐 Live Site

Visit the live course at: [https://wasia93.github.io/docusaurus-robotics-book/](https://wasia93.github.io/docusaurus-robotics-book/)

## 🛠️ GitHub Pages Deployment

The site is automatically deployed to GitHub Pages when changes are pushed to the `main` branch using GitHub Actions.

### Manual Deployment (if needed)

```bash
cd robotics-book
npm run build
GIT_USER=Wasia93 npm run deploy
```

## 📖 Course Content

### Foundations
- Introduction to Physical AI
- Embodied Intelligence
- Humanoid Robot Landscape
- Sensor Systems

### Technical Skills
- ROS 2 package development
- Robot simulation (Gazebo, Unity)
- AI-powered perception (Isaac ROS)
- Synthetic data generation
- Vision-language-action integration
- Bipedal locomotion and manipulation
- Human-robot interaction design

### Assessments
- ROS 2 Package Development Project
- Gazebo Simulation Implementation
- Isaac-Based Perception Pipeline
- Capstone: Autonomous Humanoid Robot

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Course content covers industry-standard tools: ROS 2, Gazebo, NVIDIA Isaac, GPT/Claude integration
- Generated with assistance from Claude Code

## 📧 Contact

For questions or feedback, please open an issue on GitHub.

---

**🤖 Generated with [Claude Code](https://claude.com/claude-code)**

Co-Authored-By: Claude <noreply@anthropic.com>
