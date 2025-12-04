# Human-Robot Interaction Design

## Overview

Human-Robot Interaction (HRI) design focuses on creating natural, intuitive, and safe interactions between humans and humanoid robots. This module covers principles of HRI, interaction modalities, and social robot behaviors.

## Principles of HRI

### Key Considerations

1. **Naturalness**: Interactions should feel intuitive
2. **Transparency**: Robot intentions should be clear
3. **Safety**: Physical and psychological safety
4. **Predictability**: Humans should understand robot behavior
5. **Adaptability**: Respond to different users and contexts

### Uncanny Valley

The emotional response to human-like robots:

```
Emotional Response
     ^
     |        /\
     |       /  \
     |      /    \___
     |_____/         \____
     +--------------------> Human Likeness
           ^
      Uncanny Valley
```

**Design Implication**: Avoid "almost human" appearance; either clearly robotic or highly realistic.

## Interaction Modalities

### 1. Speech and Voice

```python
import whisper
import pyttsx3

class VoiceInteraction:
    def __init__(self):
        self.speech_recognizer = whisper.load_model("base")
        self.tts_engine = pyttsx3.init()

    def listen(self, audio_file):
        """Convert speech to text"""
        result = self.speech_recognizer.transcribe(audio_file)
        return result["text"]

    def speak(self, text):
        """Convert text to speech"""
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

    def conversation_loop(self):
        """Interactive conversation"""
        while True:
            # Listen
            user_input = self.listen("audio.wav")
            print(f"User: {user_input}")

            # Process with LLM
            response = self.generate_response(user_input)
            print(f"Robot: {response}")

            # Speak
            self.speak(response)
```

### 2. Gesture Recognition

```python
import mediapipe as mp

class GestureRecognizer:
    def __init__(self):
        self.hands = mp.solutions.hands.Hands()

    def recognize_gesture(self, image):
        """Detect hand gestures from camera image"""

        results = self.hands.process(image)

        if not results.multi_hand_landmarks:
            return None

        # Get hand landmarks
        hand_landmarks = results.multi_hand_landmarks[0]

        # Classify gesture
        gesture = self.classify_hand_pose(hand_landmarks)

        return gesture

    def classify_hand_pose(self, landmarks):
        """Classify hand gesture from landmarks"""

        # Simple gesture recognition
        # Check finger states (extended or bent)

        fingers_extended = []
        for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            extended = self.is_finger_extended(landmarks, finger)
            fingers_extended.append(extended)

        # Map to gestures
        if fingers_extended == [False, True, False, False, False]:
            return "pointing"
        elif fingers_extended == [True, False, False, False, True]:
            return "rock_sign"
        elif sum(fingers_extended) == 5:
            return "open_hand"
        elif sum(fingers_extended) == 0:
            return "fist"
        else:
            return "unknown"
```

### 3. Facial Expression Recognition

```python
from fer import FER

class EmotionRecognizer:
    def __init__(self):
        self.detector = FER(mtcnn=True)

    def detect_emotion(self, image):
        """Detect human emotion from face"""

        emotions = self.detector.detect_emotions(image)

        if not emotions:
            return None

        # Get dominant emotion
        emotion_scores = emotions[0]['emotions']
        dominant_emotion = max(emotion_scores, key=emotion_scores.get)

        return dominant_emotion, emotion_scores

    def respond_to_emotion(self, emotion):
        """Generate appropriate robot response"""

        responses = {
            'happy': "I'm glad to see you're happy!",
            'sad': "You seem sad. Is there anything I can help with?",
            'angry': "I sense frustration. Let's take a moment.",
            'surprise': "Something surprised you!",
            'neutral': "How can I assist you today?"
        }

        return responses.get(emotion, "I'm here to help.")
```

### 4. Gaze and Attention

```python
class GazeController:
    def __init__(self, robot):
        self.robot = robot

    def look_at_person(self, person_position):
        """Orient robot head to look at person"""

        # Compute head orientation
        head_target = self.compute_look_at_orientation(
            self.robot.head_position,
            person_position
        )

        # Send head control command
        self.robot.set_head_orientation(head_target)

    def track_speaker(self, audio_direction):
        """Track sound source"""

        # Turn head toward sound
        self.look_at_person(audio_direction)

    def social_gaze(self, interaction_state):
        """Implement natural gaze patterns"""

        if interaction_state == 'listening':
            # Look at speaker's face
            self.look_at_person(self.get_person_face())

        elif interaction_state == 'speaking':
            # Alternate between looking at person and away
            if random.random() < 0.7:  # 70% eye contact
                self.look_at_person(self.get_person_face())
            else:
                self.look_away_naturally()

        elif interaction_state == 'thinking':
            # Look up or to the side
            self.look_at_thinking_direction()
```

## Social Robot Behaviors

### Proxemics (Personal Space)

```python
class ProxemicsManager:
    def __init__(self):
        # Personal space zones (in meters)
        self.intimate_zone = 0.45
        self.personal_zone = 1.2
        self.social_zone = 3.6

    def compute_appropriate_distance(self, context, relationship):
        """Determine appropriate distance based on social context"""

        if relationship == 'stranger':
            return self.social_zone
        elif relationship == 'acquaintance':
            return self.personal_zone
        elif relationship == 'close':
            return self.intimate_zone

        # Default to personal zone
        return self.personal_zone

    def maintain_distance(self, person_position, desired_distance):
        """Navigate to maintain appropriate social distance"""

        current_distance = np.linalg.norm(
            self.robot.position - person_position
        )

        distance_error = current_distance - desired_distance

        if abs(distance_error) > 0.2:  # 20cm tolerance
            # Move toward or away from person
            direction = (person_position - self.robot.position)
            direction = direction / np.linalg.norm(direction)

            target_position = person_position - direction * desired_distance

            self.robot.navigate_to(target_position)
```

### Turn-Taking and Interruption Handling

```python
class ConversationManager:
    def __init__(self):
        self.is_speaking = False
        self.is_listening = False
        self.interrupt_threshold = 0.5  # seconds

    def manage_turn_taking(self):
        """Implement natural turn-taking"""

        # Detect when user starts speaking
        if self.detect_user_speech():
            if self.is_speaking:
                # User interrupted robot
                self.handle_interruption()
            else:
                # User's turn to speak
                self.start_listening()

        # Detect when user stops speaking
        elif self.is_listening and self.detect_silence(duration=1.0):
            # Robot's turn to respond
            self.start_speaking()

    def handle_interruption(self):
        """Handle being interrupted gracefully"""

        # Stop speaking immediately
        self.stop_speech()

        # Acknowledge interruption
        responses = [
            "Yes?",
            "Go ahead.",
            "What is it?"
        ]
        self.queue_response(random.choice(responses))

        # Listen to user
        self.start_listening()

    def detect_end_of_turn(self, speech_audio):
        """Detect if user has finished speaking"""

        # Check for turn-yielding cues:
        # - Silence duration
        # - Falling intonation
        # - Completion of syntactic phrase

        silence_duration = self.get_silence_duration()
        has_falling_intonation = self.detect_falling_pitch(speech_audio)

        if silence_duration > 1.0 and has_falling_intonation:
            return True

        return False
```

### Expressiveness and Body Language

```python
class ExpressiveBehavior:
    def __init__(self, robot):
        self.robot = robot

    def express_emotion(self, emotion, intensity=0.5):
        """Express emotion through body language"""

        if emotion == 'happy':
            self.robot.set_head_tilt(15)  # Slight upward tilt
            # Could also: wave, slight bounce motion

        elif emotion == 'sad':
            self.robot.set_head_tilt(-10)  # Downward tilt
            self.robot.set_shoulder_slump(20)

        elif emotion == 'confused':
            self.robot.set_head_tilt(20, side='right')  # Head tilt to side
            # Could also: scratch head gesture

        elif emotion == 'confident':
            self.robot.set_posture('upright')
            self.robot.set_chest_out(10)

    def animate_speech(self, text):
        """Add gestures during speech"""

        # Simple rule-based gestures
        if '?' in text:
            self.robot.gesture('questioning_hands')

        if any(word in text.lower() for word in ['yes', 'correct', 'right']):
            self.robot.gesture('nod')

        if any(word in text.lower() for word in ['no', 'wrong', 'incorrect']):
            self.robot.gesture('shake_head')

        # Pointing gestures for spatial references
        if any(word in text.lower() for word in ['there', 'that', 'this']):
            self.robot.gesture('point')
```

## Safety in HRI

### Collision Avoidance

```python
class SafeHRI:
    def __init__(self, robot):
        self.robot = robot
        self.safety_distance = 0.3  # meters

    def safe_motion_planning(self, target_position):
        """Plan motion that avoids colliding with humans"""

        # Detect nearby humans
        humans = self.detect_humans()

        # Compute safe trajectory
        trajectory = self.plan_trajectory(
            start=self.robot.position,
            goal=target_position,
            obstacles=humans,
            safety_margin=self.safety_distance
        )

        return trajectory

    def emergency_stop(self):
        """Immediately stop all motion"""

        # Detect imminent collision
        if self.detect_imminent_collision():
            self.robot.stop_all_motion()
            self.robot.lock_joints()
            self.speak("Stopping for safety")

    def compliant_contact(self):
        """React safely to unexpected contact"""

        if self.detect_contact():
            # Immediately reduce force/torque
            self.robot.set_compliant_mode()

            # Move away from contact
            contact_direction = self.get_contact_direction()
            retreat_direction = -contact_direction

            self.robot.move_slowly(retreat_direction)

            self.speak("Sorry, I didn't mean to bump into you")
```

## Multimodal Interaction

### Combining Modalities

```python
class MultimodalInteraction:
    def __init__(self):
        self.voice = VoiceInteraction()
        self.gesture = GestureRecognizer()
        self.gaze = GazeController()

    def process_multimodal_input(self):
        """Fuse information from multiple input channels"""

        # Voice command
        voice_input = self.voice.listen("audio.wav")

        # Gesture
        gesture = self.gesture.recognize_gesture(self.camera.get_frame())

        # Combine information
        if gesture == 'pointing':
            # User is pointing at something
            point_direction = self.compute_point_direction(gesture)

            if "that" in voice_input or "there" in voice_input:
                # Resolve reference: "Bring me that" + pointing
                object_location = point_direction
                command = voice_input.replace("that", "object")

                return {
                    'command': command,
                    'target': object_location,
                    'confidence': 0.9
                }

        return {
            'command': voice_input,
            'target': None,
            'confidence': 0.7
        }

    def multimodal_feedback(self, message):
        """Provide feedback through multiple channels"""

        # Voice
        self.voice.speak(message)

        # Gaze (look at user)
        self.gaze.look_at_person(self.get_user_position())

        # Gesture (nod)
        self.robot.gesture('nod')

        # Could also: Display on screen, LED indicators, etc.
```

## Practical Exercises

### Exercise 1: Conversational Robot

Implement a conversational interface:

```python
# TODO: Student implementation
# 1. Integrate speech recognition and TTS
# 2. Add LLM for conversation
# 3. Implement turn-taking
# 4. Test with multiple users
```

### Exercise 2: Gesture Control

Create a gesture-based robot control system:

```python
# TODO: Student implementation
# 1. Set up camera for hand tracking
# 2. Implement gesture recognition
# 3. Map gestures to robot actions
# 4. Add visual feedback for recognized gestures
```

### Exercise 3: Social Navigation

Implement social-aware navigation:

```python
# TODO: Student implementation
# 1. Detect people in environment
# 2. Implement proxemics-aware path planning
# 3. Respect personal space while navigating
# 4. Test in crowded environment (simulation)
```

## Key Takeaways

- HRI design requires understanding human psychology and social norms
- Multiple interaction modalities enable natural communication
- Safety is paramount in human-robot interaction
- Expressiveness makes robots more understandable and relatable
- Multimodal interaction is more robust than single-channel
- Social behaviors like gaze and proxemics improve user experience

## Resources

- [Handbook of Human-Robot Interaction](https://link.springer.com/referencework/10.1007/978-3-319-32552-1)
- [Socially Assistive Robotics](http://robotics.usc.edu/interaction/)
- [ROS HRI Stack](http://wiki.ros.org/hri)

## Next Steps

Continue to [Capstone Project](./capstone-project.md) to integrate everything into a complete autonomous humanoid system.
