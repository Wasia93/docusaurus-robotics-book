---
sidebar_position: 1
title: Voice-to-Action Pipeline
description: Using OpenAI Whisper and LLMs for voice-controlled robots
keywords: [voice commands, Whisper, speech-to-text, LLM, natural language, robotics]
---

# Module 4: Vision-Language-Action - Voice-to-Action

## Introduction to VLA

**Vision-Language-Action (VLA)** models represent the convergence of:
- **Vision**: Understanding the physical world through cameras
- **Language**: Natural language instructions from humans
- **Action**: Motor commands to execute tasks

This module focuses on building a **voice-to-action pipeline** where:
1. **Human speaks**: "Pick up the red cup"
2. **Speech-to-Text**: Transcribe audio to text (Whisper)
3. **Language Understanding**: Parse command (LLM/GPT)
4. **Action Planning**: Translate to robot actions (ROS 2 commands)
5. **Execution**: Robot performs task

## The Voice-to-Action Pipeline

```
┌─────────────────────────────────────────────────────────┐
│ Human: "Move forward and pick up the red cup"          │
└──────────────────┬──────────────────────────────────────┘
                   │
    ┌──────────────▼─────────────┐
    │  1. Audio Capture          │  (Microphone)
    │     ReSpeaker Mic Array    │
    └──────────────┬─────────────┘
                   │
    ┌──────────────▼─────────────┐
    │  2. Speech-to-Text         │  (OpenAI Whisper)
    │     Transcribe audio       │
    └──────────────┬─────────────┘
                   │
    ┌──────────────▼─────────────┐
    │  3. Language Understanding │  (GPT-4)
    │     Parse intent & params  │
    └──────────────┬─────────────┘
                   │
    ┌──────────────▼─────────────┐
    │  4. Action Planning        │  (Task Planner)
    │     Sequence of actions    │
    └──────────────┬─────────────┘
                   │
    ┌──────────────▼─────────────┐
    │  5. Robot Execution        │  (ROS 2 Actions)
    │     Navigate, grasp, etc.  │
    └────────────────────────────┘
```

## Step 1: Audio Capture

### Using ReSpeaker Microphone

**Hardware**: ReSpeaker USB Mic Array v2.0 ($69)
- 4-microphone array
- Far-field voice capture (3-5 meters)
- Beamforming for noise reduction

### ROS 2 Audio Capture Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import wave

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture')

        # Audio parameters
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # 16kHz (Whisper's native rate)

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
        )

        # Publisher for audio chunks
        self.audio_pub = self.create_publisher(
            String,
            '/audio/raw',
            10
        )

        # Timer for reading audio
        self.timer = self.create_timer(0.1, self.capture_audio)

        self.get_logger().info('Audio Capture Node started')

    def capture_audio(self):
        # Read audio chunk
        data = self.stream.read(self.CHUNK, exception_on_overflow=False)

        # Convert to numpy array
        audio_data = np.frombuffer(data, dtype=np.int16)

        # Check if speech detected (simple energy threshold)
        energy = np.abs(audio_data).mean()
        if energy > 500:  # Adjust threshold
            self.get_logger().info(f'Speech detected, energy: {energy}')
            # Publish audio (in practice, accumulate and publish full utterance)
            # msg = String()
            # msg.data = data.hex()
            # self.audio_pub.publish(msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()
```

## Step 2: Speech-to-Text with Whisper

### OpenAI Whisper

**Whisper** is a robust speech recognition model:
- State-of-the-art accuracy
- Multilingual (99 languages)
- Handles accents, noise, far-field
- Open-source (MIT license)

### Installation

```bash
pip install openai-whisper
# OR for faster inference
pip install whisper-ctranslate2
```

### Whisper ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import wave
import tempfile

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model
        self.declare_parameter('model_size', 'base')  # tiny, base, small, medium, large
        model_size = self.get_parameter('model_size').value

        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded')

        # Subscriber for audio
        self.audio_sub = self.create_subscription(
            String,
            '/audio/utterance',
            self.audio_callback,
            10
        )

        # Publisher for transcriptions
        self.text_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )

    def audio_callback(self, msg):
        # In practice, msg.data contains audio file path or audio bytes
        audio_path = msg.data  # Path to .wav file

        # Transcribe
        self.get_logger().info('Transcribing audio...')
        result = self.model.transcribe(
            audio_path,
            language='en',  # or auto-detect
            fp16=False,  # Use FP32 (FP16 requires CUDA)
        )

        transcription = result['text'].strip()
        self.get_logger().info(f'Transcription: "{transcription}"')

        # Publish transcription
        text_msg = String()
        text_msg.data = transcription
        self.text_pub.publish(text_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Running Whisper Node

```bash
# Launch Whisper node with base model
ros2 run voice_control whisper_node --ros-args -p model_size:=base

# Test by publishing audio file path
ros2 topic pub /audio/utterance std_msgs/String "data: '/path/to/command.wav'"

# Listen to transcriptions
ros2 topic echo /voice_command
```

**Model sizes and performance**:

| Model | Parameters | English-only | Multilingual | Speed (CPU) | Accuracy |
|-------|------------|--------------|--------------|-------------|----------|
| tiny | 39M | ✅ | ✅ | ~10x real-time | Good |
| base | 74M | ✅ | ✅ | ~7x real-time | Better |
| small | 244M | ✅ | ✅ | ~4x real-time | Great |
| medium | 769M | ✅ | ✅ | ~2x real-time | Excellent |
| large | 1550M | ❌ | ✅ | ~1x real-time | Best |

**Recommendation**: **base** for real-time on Jetson, **small** for better accuracy.

## Step 3: Language Understanding with LLMs

### Using GPT-4 for Command Parsing

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import RobotCommand  # Custom message
import openai
import os
import json

class LLMCommandParserNode(Node):
    def __init__(self):
        super().__init__('llm_command_parser')

        # OpenAI API key
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        # Publish parsed commands
        self.command_pub = self.create_publisher(
            RobotCommand,
            '/robot/command',
            10
        )

        # System prompt
        self.system_prompt = """
        You are a robot command parser. Convert natural language commands
        into structured JSON with these fields:
        - action: "navigate", "pick", "place", "stop", "wait"
        - parameters: dict with action-specific params
        - priority: "low", "medium", "high"

        Examples:
        Input: "Move forward 2 meters"
        Output: {"action": "navigate", "parameters": {"direction": "forward", "distance": 2.0}, "priority": "medium"}

        Input: "Pick up the red cup"
        Output: {"action": "pick", "parameters": {"object": "cup", "color": "red"}, "priority": "high"}

        Respond ONLY with valid JSON, no explanation.
        """

        self.get_logger().info('LLM Command Parser started')

    def command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f'Received command: "{command_text}"')

        # Query LLM
        parsed_command = self.parse_command(command_text)

        if parsed_command:
            # Publish structured command
            cmd_msg = RobotCommand()
            cmd_msg.action = parsed_command['action']
            cmd_msg.parameters = json.dumps(parsed_command['parameters'])
            cmd_msg.priority = parsed_command['priority']
            self.command_pub.publish(cmd_msg)

            self.get_logger().info(f'Published command: {parsed_command["action"]}')

    def parse_command(self, text):
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": text}
                ],
                temperature=0.0,  # Deterministic
                max_tokens=150,
            )

            content = response.choices[0].message.content.strip()

            # Remove markdown code blocks if present
            if content.startswith("```json"):
                content = content.split("```json")[1].split("```")[0].strip()
            elif content.startswith("```"):
                content = content.split("```")[1].split("```")[0].strip()

            # Parse JSON
            parsed = json.loads(content)
            return parsed

        except Exception as e:
            self.get_logger().error(f'LLM parsing failed: {e}')
            return None
```

### Custom Robot Command Message

**robot_interfaces/msg/RobotCommand.msg**:
```
string action
string parameters  # JSON string
string priority
```

## Step 4: Action Planning

### Task Planner Node

```python
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import json

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner')

        # Subscribe to parsed commands
        self.command_sub = self.create_subscription(
            RobotCommand,
            '/robot/command',
            self.command_callback,
            10
        )

        # Publishers for various actions
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('Task Planner started')

    def command_callback(self, msg):
        action = msg.action
        params = json.loads(msg.parameters)

        self.get_logger().info(f'Executing action: {action} with params: {params}')

        if action == "navigate":
            self.execute_navigate(params)
        elif action == "pick":
            self.execute_pick(params)
        elif action == "stop":
            self.execute_stop()
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def execute_navigate(self, params):
        direction = params.get('direction', 'forward')
        distance = params.get('distance', 1.0)

        self.get_logger().info(f'Navigating {direction} for {distance}m')

        # Simple velocity control
        twist = Twist()
        if direction == 'forward':
            twist.linear.x = 0.5
        elif direction == 'backward':
            twist.linear.x = -0.5
        elif direction == 'left':
            twist.angular.z = 0.5
        elif direction == 'right':
            twist.angular.z = -0.5

        # Publish for duration (simplified, use Nav2 in practice)
        duration = distance / 0.5  # seconds
        rate = self.create_rate(10)  # 10 Hz
        for _ in range(int(duration * 10)):
            self.vel_pub.publish(twist)
            rate.sleep()

        # Stop
        self.vel_pub.publish(Twist())
        self.get_logger().info('Navigation complete')

    def execute_pick(self, params):
        obj = params.get('object', 'unknown')
        color = params.get('color', 'any')

        self.get_logger().info(f'Picking {color} {obj}')

        # 1. Use computer vision to detect object
        # 2. Navigate to object
        # 3. Lower arm and grasp
        # 4. Lift object

        # Placeholder for actual implementation
        self.get_logger().warn('Pick action not fully implemented yet')

    def execute_stop(self):
        self.get_logger().info('Stopping robot')
        twist = Twist()
        self.vel_pub.publish(twist)
```

## Step 5: Complete Voice Control System

### Launch File

**voice_control.launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Audio capture
        Node(
            package='voice_control',
            executable='audio_capture_node',
            name='audio_capture',
            output='screen',
        ),

        # Whisper speech-to-text
        Node(
            package='voice_control',
            executable='whisper_node',
            name='whisper',
            parameters=[{'model_size': 'base'}],
            output='screen',
        ),

        # LLM command parser
        Node(
            package='voice_control',
            executable='llm_parser_node',
            name='llm_parser',
            output='screen',
        ),

        # Task planner
        Node(
            package='voice_control',
            executable='task_planner_node',
            name='task_planner',
            output='screen',
        ),
    ])
```

**Run**:
```bash
ros2 launch voice_control voice_control.launch.py
```

## Testing the Pipeline

### Test with Audio File

```bash
# Record test command
arecord -d 5 -f S16_LE -r 16000 test_command.wav

# Publish to audio topic
ros2 topic pub /audio/utterance std_msgs/String "data: '/path/to/test_command.wav'"

# Monitor transcription
ros2 topic echo /voice_command

# Monitor parsed commands
ros2 topic echo /robot/command
```

### Test with Live Microphone

```python
# Modify audio_capture_node to continuously listen
# When energy threshold exceeded, record 3 seconds
# Save to file and publish path to /audio/utterance
```

## Advanced: End-to-End VLA Models

### RT-2 (Robotic Transformer 2)

**Google DeepMind's RT-2**: Vision-Language-Action model
- Input: Image + text command
- Output: Robot actions (joint positions)
- Trained on web data + robot demonstrations

**Architecture**:
```
┌──────────────────────────────────────┐
│  Vision Encoder (ViT)                │  ← Camera image
└───────────────┬──────────────────────┘
                │
┌───────────────▼──────────────────────┐
│  Language Model (PaLM/T5)            │  ← Text command
└───────────────┬──────────────────────┘
                │
┌───────────────▼──────────────────────┐
│  Action Head (Transformer Decoder)   │  → Robot actions
└──────────────────────────────────────┘
```

**Using RT-2** (hypothetical, model not publicly released):
```python
# Pseudocode (RT-2 not open-source yet)
from rt2 import RT2Model

model = RT2Model.load_pretrained("rt2-1x")

# Input
image = capture_camera_image()  # numpy array
command = "pick up the red cup"

# Inference
actions = model.predict(image, command)
# actions: [joint1, joint2, ..., gripper]

# Execute on robot
robot.set_joint_positions(actions)
```

### Open-Source Alternatives

**OpenVLA** (2024):
- Open-source VLA model
- Based on GPT-4V + action tokenization
- GitHub: https://github.com/openvla/openvla

## Next Steps

You've built a voice-to-action pipeline. Next, explore LLM-based cognitive planning for complex tasks.

👉 **[Next: LLM Robotics Integration →](llm-robotics)**

---

:::tip Voice Control Tips

1. **Noise reduction**: Use beamforming microphones (ReSpeaker)
2. **Wake word**: Add "Hey robot" detection to avoid false triggers
3. **Confidence threshold**: Only act on high-confidence transcriptions
4. **Feedback**: Robot should confirm understanding ("Moving forward 2 meters")
5. **Safety**: Add "stop" command that interrupts all actions

:::

## Lab Exercise

**Build a voice-controlled navigator**:

1. **Setup**:
   - Install Whisper (base model)
   - Set up OpenAI API key

2. **Implement nodes**:
   - Audio capture (simple version: record on button press)
   - Whisper transcription
   - LLM parsing
   - Simple navigation (forward, backward, left, right)

3. **Test commands**:
   - "Move forward 2 meters"
   - "Turn left 90 degrees"
   - "Go backward"
   - "Stop"

4. **Verify**:
   - Transcription accuracy (>90%)
   - Command parsing correctness
   - Robot executes commands

**Bonus**: Add object detection + "Go to the red cube" command.
