# LLM Integration in Robotics

## Overview

The convergence of Large Language Models (LLMs) and robotics represents a paradigm shift in how robots understand and execute tasks. This module explores how to integrate conversational AI into robotic systems.

## Why LLMs for Robotics?

### Traditional Approach
- Hardcoded commands: `move_forward(2.5)`, `grasp_object("cup")`
- Limited flexibility and natural interaction
- Requires programming for every new task

### LLM-Powered Approach
- Natural language commands: "Please bring me the red cup from the kitchen table"
- Contextual understanding and reasoning
- Adaptive to new scenarios without reprogramming

## Architecture: From Language to Action

### The VLA Pipeline

```
User Speech Input → Speech-to-Text → LLM Planning → Action Primitives → ROS 2 Execution
```

#### 1. Speech-to-Text (OpenAI Whisper)

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
result = model.transcribe("audio.mp3")
print(result["text"])
# Output: "Pick up the blue box and place it on the shelf"
```

#### 2. LLM Task Planning (GPT-4)

```python
import openai

def natural_language_to_plan(command: str) -> list:
    """Convert natural language to robot action plan"""

    system_prompt = """You are a robot task planner. Given a natural language
    command, break it down into a sequence of robot primitives:
    - navigate_to(location)
    - detect_object(description)
    - grasp_object(object_id)
    - place_object(location)
    - speak(text)

    Return a JSON list of actions."""

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": command}
        ]
    )

    return response.choices[0].message.content
```

Example output:
```json
[
    {"action": "navigate_to", "params": {"location": "kitchen"}},
    {"action": "detect_object", "params": {"description": "blue box"}},
    {"action": "grasp_object", "params": {"object_id": "detected_object_1"}},
    {"action": "navigate_to", "params": {"location": "shelf"}},
    {"action": "place_object", "params": {"location": "shelf_surface"}},
    {"action": "speak", "params": {"text": "Task completed"}}
]
```

#### 3. Action Primitive Execution

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.grasp_pub = self.create_publisher(String, '/grasp_command', 10)

    def execute_plan(self, plan: list):
        for action in plan:
            action_type = action["action"]
            params = action["params"]

            if action_type == "navigate_to":
                self.navigate(params["location"])
            elif action_type == "grasp_object":
                self.grasp(params["object_id"])
            elif action_type == "place_object":
                self.place(params["location"])

    def navigate(self, location: str):
        # Publish navigation goal
        goal = PoseStamped()
        # ... set goal based on location
        self.nav_pub.publish(goal)

    def grasp(self, object_id: str):
        # Send grasp command
        msg = String()
        msg.data = f"grasp {object_id}"
        self.grasp_pub.publish(msg)
```

## Grounding: Connecting Words to Physical World

### The Grounding Problem

LLMs understand language but not physical space. We need to ground abstract concepts:

- "Kitchen" → GPS coordinates or map location
- "Blue box" → Object detected by vision system
- "Gently" → Force control parameters

### Semantic Mapping

```python
class SemanticMap:
    def __init__(self):
        self.locations = {
            "kitchen": {"x": 5.0, "y": 3.0, "z": 0.0},
            "living room": {"x": -2.0, "y": 1.0, "z": 0.0},
            "shelf": {"x": 3.0, "y": -1.0, "z": 0.8}
        }

    def ground_location(self, location_name: str):
        """Convert location name to coordinates"""
        return self.locations.get(location_name.lower())

    def ground_object(self, description: str, vision_detections: list):
        """Match description to detected objects"""
        # Use CLIP or similar vision-language model
        for obj in vision_detections:
            similarity = self.compute_similarity(description, obj.features)
            if similarity > threshold:
                return obj
        return None
```

## Context and Memory

### Maintaining Conversation Context

```python
class ConversationalRobot:
    def __init__(self):
        self.conversation_history = []
        self.scene_context = {}

    def process_command(self, user_input: str):
        # Add to conversation history
        self.conversation_history.append({
            "role": "user",
            "content": user_input
        })

        # Include scene context
        context_prompt = f"""
        Current location: {self.scene_context['location']}
        Visible objects: {self.scene_context['objects']}
        Previous actions: {self.conversation_history[-3:]}
        """

        # Get LLM response with full context
        response = self.llm_plan(context_prompt + user_input)

        return response
```

### Example Contextual Dialogue

```
User: "Is there a cup on the table?"
Robot: [Checks vision] "Yes, I see a red cup on the kitchen table."
User: "Bring it to me."
Robot: [Understands "it" refers to the cup] "Navigating to kitchen table..."
```

## Safety and Constraints

### Constraining LLM Outputs

```python
def validate_action_plan(plan: list) -> bool:
    """Ensure plan is safe and feasible"""

    for action in plan:
        # Check action is in allowed set
        if action["action"] not in ALLOWED_ACTIONS:
            return False

        # Check spatial constraints
        if action["action"] == "navigate_to":
            location = action["params"]["location"]
            if not is_reachable(location):
                return False

        # Check object manipulation constraints
        if action["action"] == "grasp_object":
            if not is_safe_to_grasp(action["params"]["object_id"]):
                return False

    return True
```

## Practical Implementation

### Full VLA System

```python
class VLARobot(Node):
    def __init__(self):
        super().__init__('vla_robot')
        self.whisper_model = whisper.load_model("base")
        self.llm_client = openai.Client()
        self.action_executor = ActionExecutor()
        self.semantic_map = SemanticMap()

    def voice_command_callback(self, audio_path: str):
        # 1. Speech-to-Text
        text = self.whisper_model.transcribe(audio_path)["text"]
        self.get_logger().info(f"Heard: {text}")

        # 2. LLM Planning
        plan = self.llm_plan(text)
        self.get_logger().info(f"Plan: {plan}")

        # 3. Ground plan to physical world
        grounded_plan = self.ground_plan(plan)

        # 4. Validate safety
        if not validate_action_plan(grounded_plan):
            self.speak("I cannot safely execute this command")
            return

        # 5. Execute
        self.action_executor.execute_plan(grounded_plan)
```

## Challenges and Limitations

### Current Challenges

1. **Hallucination**: LLMs may plan infeasible actions
2. **Latency**: API calls can introduce delays
3. **Cost**: Frequent LLM queries can be expensive
4. **Grounding**: Connecting abstract concepts to physical world
5. **Safety**: Ensuring safe execution of generated plans

### Mitigation Strategies

- **Validation layers**: Check plan feasibility before execution
- **Local models**: Use smaller, local LLMs for latency-critical applications
- **Caching**: Store common command patterns
- **Human-in-the-loop**: Request confirmation for uncertain actions

## Exercises

### Exercise 1: Basic Voice Commands

Implement a simple voice command system:

```python
# TODO: Student implementation
# 1. Set up Whisper for speech recognition
# 2. Create LLM prompt for task planning
# 3. Implement basic action primitives (navigate, speak)
# 4. Test with simple commands like "Go to the kitchen"
```

### Exercise 2: Contextual Understanding

Extend the system to handle contextual references:

```python
# TODO: Student implementation
# 1. Maintain conversation history
# 2. Handle pronouns ("it", "there", "that")
# 3. Track mentioned objects and locations
# 4. Test multi-turn conversations
```

### Exercise 3: Safe Action Planning

Add safety constraints:

```python
# TODO: Student implementation
# 1. Define workspace boundaries
# 2. Identify forbidden actions
# 3. Implement action validation
# 4. Test rejection of unsafe commands
```

## Key Takeaways

- LLMs enable natural language interaction with robots
- The VLA pipeline: Speech → Planning → Grounding → Execution
- Grounding connects abstract language to physical reality
- Safety validation is critical for autonomous execution
- Context and memory enable more natural conversations

## Resources

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [LangChain for Robotics](https://python.langchain.com/docs/use_cases/robotics)
- [Vision-Language-Action Models Paper](https://arxiv.org/abs/2307.15818)

## Next Steps

Continue to [Cognitive Planning](./cognitive-planning.md) to learn about advanced reasoning for complex tasks.
