# Cognitive Planning with LLMs

## Overview

Cognitive planning bridges high-level natural language commands with low-level robot actions. This module explores how LLMs can decompose complex tasks, reason about constraints, and generate executable plans.

## From Commands to Actions

### The Planning Hierarchy

```
Natural Language → Task Decomposition → Motion Planning → Motor Commands
```

Example: "Clean the room"

1. **Task Level**: Identify objects, plan sequence
2. **Motion Level**: Path planning, collision avoidance
3. **Control Level**: Joint trajectories, gripper control

## LLM as Task Planner

### Hierarchical Task Network (HTN) Planning

```python
def plan_task(task_description: str) -> dict:
    """Use LLM to decompose task into subtasks"""

    prompt = f"""
    You are a robot task planner. Decompose the following task into
    a hierarchical plan using these primitives:

    Navigation: navigate_to(location), explore_area(region)
    Perception: detect_objects(query), identify_object(id)
    Manipulation: pick_object(id), place_object(location), push_object(id)
    Interaction: open_door(id), close_gripper(), speak(text)

    Task: {task_description}

    Return a JSON hierarchical task network.
    """

    response = call_llm(prompt)
    return json.loads(response)
```

Example output for "Clean the room":

```json
{
    "task": "clean_room",
    "subtasks": [
        {
            "task": "identify_mess",
            "actions": [
                {"action": "navigate_to", "params": {"location": "room_center"}},
                {"action": "explore_area", "params": {"region": "room"}},
                {"action": "detect_objects", "params": {"query": "trash, dirty items"}}
            ]
        },
        {
            "task": "collect_items",
            "repeat_for_each": "detected_object",
            "actions": [
                {"action": "navigate_to", "params": {"location": "object_location"}},
                {"action": "pick_object", "params": {"id": "current_object"}},
                {"action": "navigate_to", "params": {"location": "trash_bin"}},
                {"action": "place_object", "params": {"location": "trash_bin"}}
            ]
        },
        {
            "task": "confirm_completion",
            "actions": [
                {"action": "speak", "params": {"text": "Room cleaning complete"}}
            ]
        }
    ]
}
```

## Reasoning About Constraints

### Spatial Reasoning

```python
class SpatialReasoner:
    def __init__(self, semantic_map):
        self.map = semantic_map

    def reason_about_location(self, query: str) -> dict:
        """Use LLM to reason about spatial relationships"""

        context = f"""
        Known locations and objects:
        - Kitchen: [table, chair, refrigerator, sink]
        - Living room: [couch, TV, bookshelf]
        - Objects on kitchen table: [cup, plate, book]

        Question: {query}

        Provide reasoning and answer as JSON.
        """

        response = call_llm(context)
        return json.loads(response)

# Example
reasoner.reason_about_location("Where should I place the cup after cleaning?")
# Output: {"reasoning": "Cups belong in kitchen, preferably in cupboard or on drying rack",
#          "location": "kitchen_cupboard", "alternative": "kitchen_counter"}
```

### Temporal Reasoning

```python
def plan_with_temporal_constraints(goal: str, constraints: list) -> dict:
    """Plan considering time and ordering constraints"""

    prompt = f"""
    Plan to achieve: {goal}

    Constraints:
    {format_constraints(constraints)}

    Consider:
    - Prerequisites (must happen before other actions)
    - Duration estimates
    - Parallel execution opportunities
    - Resource conflicts

    Return a PDDL-style temporal plan.
    """

    plan = call_llm(prompt)
    return parse_temporal_plan(plan)
```

Example constraints:
```python
constraints = [
    "Must grasp object before placing it",
    "Cannot navigate and manipulate simultaneously",
    "Object detection takes 2-3 seconds",
    "Navigation to kitchen takes ~10 seconds"
]
```

## Chain-of-Thought Reasoning

### Step-by-Step Problem Solving

```python
def reason_step_by_step(problem: str) -> dict:
    """Use chain-of-thought prompting for complex reasoning"""

    prompt = f"""
    Problem: {problem}

    Let's solve this step-by-step:
    1. What do we know? (state the facts)
    2. What is the goal? (clarify the objective)
    3. What are the constraints? (identify limitations)
    4. What are possible approaches? (brainstorm)
    5. What is the best approach? (reason about tradeoffs)
    6. What are the action steps? (concrete plan)

    Think through each step carefully.
    """

    response = call_llm(prompt)
    return parse_reasoning_chain(response)
```

Example problem: "Retrieve the book from the high shelf without a ladder"

Response might include:
```
1. Known: Book is on high shelf (2m), robot max reach is 1.5m, no ladder available
2. Goal: Safely retrieve book
3. Constraints: Cannot reach directly, must not damage book or shelf
4. Approaches:
   a. Use long object to push book off (risky - book might fall)
   b. Stack stable objects to gain height (safer if objects available)
   c. Request human assistance (most reliable)
5. Best approach: Request human assistance - safest and most reliable
6. Action: navigate_to(person), speak("Could you help me get the book from the high shelf?")
```

## ReAct: Reasoning + Acting

### Interleaved Reasoning and Action

```python
class ReActAgent:
    def __init__(self):
        self.action_history = []
        self.observations = []

    def solve(self, goal: str):
        """Interleave reasoning and acting until goal achieved"""

        max_steps = 10
        for step in range(max_steps):
            # Reasoning step
            thought = self.reason(goal)
            self.get_logger().info(f"Thought: {thought}")

            # Decide action
            action = self.decide_action(thought)
            self.get_logger().info(f"Action: {action}")

            # Execute action
            observation = self.execute(action)
            self.observations.append(observation)
            self.get_logger().info(f"Observation: {observation}")

            # Check if goal achieved
            if self.is_goal_achieved(goal, observation):
                break

    def reason(self, goal: str) -> str:
        """LLM reasoning about current state and next action"""

        context = f"""
        Goal: {goal}

        Previous actions: {self.action_history[-3:]}
        Recent observations: {self.observations[-3:]}

        Think: What should I do next to achieve the goal?
        Consider: What went well? What failed? What to try?
        """

        return call_llm(context)
```

Example ReAct trace for "Find and bring the red cup":

```
Thought 1: I need to locate the red cup. I should explore the likely locations.
Action 1: navigate_to("kitchen")
Observation 1: Arrived at kitchen. Vision detects: [blue plate, green cup, red cup]

Thought 2: I found the red cup in the kitchen! Now I need to pick it up.
Action 2: navigate_to(red_cup.location)
Observation 2: Positioned in front of red cup

Thought 3: I'm close enough to grasp the cup.
Action 3: grasp_object("red_cup")
Observation 3: Successfully grasped red cup

Thought 4: I have the cup, now return to user.
Action 4: navigate_to("user_location")
Observation 4: Arrived at user. Task complete.
```

## Error Recovery and Replanning

### Handling Failures

```python
class RobustPlanner:
    def execute_with_recovery(self, plan: list):
        """Execute plan with error handling and replanning"""

        for i, action in enumerate(plan):
            try:
                result = self.execute_action(action)

                if not result.success:
                    # Action failed, attempt recovery
                    self.handle_failure(action, result, plan[i+1:])

            except Exception as e:
                self.get_logger().error(f"Exception during {action}: {e}")
                self.emergency_replan(plan[i:])

    def handle_failure(self, failed_action, result, remaining_plan):
        """Use LLM to reason about failure and generate recovery"""

        prompt = f"""
        Action failed: {failed_action}
        Failure reason: {result.error_message}
        Remaining plan: {remaining_plan}

        Analyze the failure and suggest:
        1. Is retry likely to succeed?
        2. Can we achieve the goal using an alternative approach?
        3. Should we abort and report failure?

        Provide recovery strategy.
        """

        recovery = call_llm(prompt)
        return self.execute_recovery(recovery)
```

### Example Failure Recovery

```
Failed action: grasp_object("cup")
Error: "Grasp failed - object slipped"

LLM recovery reasoning:
"The cup likely slipped due to smooth surface. Alternative approaches:
1. Adjust gripper force and retry
2. Grasp from different angle (e.g., handle if cup has one)
3. Use two-handed grasp for better stability

Recommendation: Try grasping from handle if detected, otherwise increase force and retry once."
```

## Tool Use and External Resources

### Integrating External Tools

```python
class ToolUsingAgent:
    def __init__(self):
        self.tools = {
            "calculator": self.use_calculator,
            "map_search": self.search_map,
            "object_database": self.query_objects,
            "physics_sim": self.simulate_physics
        }

    def solve_with_tools(self, problem: str):
        """LLM decides which tools to use"""

        prompt = f"""
        Available tools: {list(self.tools.keys())}

        Problem: {problem}

        Which tools do you need? How will you use them?
        Provide step-by-step plan including tool usage.
        """

        plan = call_llm(prompt)
        return self.execute_plan_with_tools(plan)
```

## Practical Exercises

### Exercise 1: Task Decomposition

Implement hierarchical task planning:

```python
# TODO: Student implementation
# 1. Create LLM prompt for task decomposition
# 2. Parse hierarchical plan structure
# 3. Implement subtask execution
# 4. Test with: "Set the table for dinner"
```

### Exercise 2: ReAct Agent

Build a ReAct agent for dynamic problem solving:

```python
# TODO: Student implementation
# 1. Implement reasoning step (LLM call)
# 2. Implement action execution
# 3. Store and use observation history
# 4. Test with: "Find the missing tool"
```

### Exercise 3: Error Recovery

Add failure handling and replanning:

```python
# TODO: Student implementation
# 1. Detect action failures
# 2. Use LLM to analyze failure
# 3. Generate recovery strategy
# 4. Test with: Intentionally failing grasp actions
```

## Key Takeaways

- LLMs excel at high-level task planning and reasoning
- Chain-of-thought improves complex problem solving
- ReAct pattern interleaves thinking and acting
- Robust systems require error handling and replanning
- Grounding and validation remain critical

## Resources

- [ReAct: Reasoning and Acting Paper](https://arxiv.org/abs/2210.03629)
- [Chain-of-Thought Prompting](https://arxiv.org/abs/2201.11903)
- [LLM-based Robot Planning](https://robot-help.github.io/)

## Next Steps

Continue to [Humanoid Kinematics](./humanoid-kinematics.md) to learn about controlling humanoid robot motion.
