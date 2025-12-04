import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '🧠 ROS 2 & Middleware',
    icon: '⚙️',
    description: (
      <>
        Master the Robot Operating System 2 - the industry standard for robot control.
        Learn nodes, topics, services, and build production-ready robotic systems.
      </>
    ),
  },
  {
    title: '🎮 Physics Simulation',
    icon: '🔬',
    description: (
      <>
        Create realistic digital twins with Gazebo and Unity. Test your robots in
        photorealistic environments before deploying to hardware.
      </>
    ),
  },
  {
    title: '🚀 NVIDIA Isaac Platform',
    icon: '🤖',
    description: (
      <>
        Leverage Isaac Sim and Isaac ROS for AI-powered perception, synthetic data
        generation, and hardware-accelerated robotics applications.
      </>
    ),
  },
  {
    title: '🗣️ Voice-Controlled AI',
    icon: '🎤',
    description: (
      <>
        Integrate GPT/Claude with Whisper for natural language robot control. Build
        robots that understand and execute voice commands intelligently.
      </>
    ),
  },
  {
    title: '🦿 Bipedal Locomotion',
    icon: '🚶',
    description: (
      <>
        Implement walking algorithms, balance control, and ZMP stability. Master the
        challenges of humanoid robot locomotion and gait planning.
      </>
    ),
  },
  {
    title: '🤝 Human-Robot Interaction',
    icon: '👥',
    description: (
      <>
        Design natural interaction systems with multimodal communication, social
        navigation, and safe human-aware robotic behaviors.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>{icon}</div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <h2 className={styles.sectionTitle}>What You'll Learn</h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive journey from ROS 2 fundamentals to building autonomous humanoid robots
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
