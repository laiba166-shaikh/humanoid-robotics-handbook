---
id: lesson-2-rclpy-agents
title: "Lesson 3.2: Bridging Python Agents with rclpy"
sidebar_position: 2
sidebar_label: "3.2 Python AI Agents"
description: "Learn to integrate Python AI models and agents with ROS 2 using rclpy patterns for intelligent robot control"
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Use cloud-based Jupyter notebooks to prototype AI models, then examine integration patterns conceptually"
learning_objectives:
  - "Integrate Python AI models with ROS 2 nodes using rclpy"
  - "Design perception-decision-action loops for autonomous behavior"
  - "Implement state management for AI agents in long-running nodes"
  - "Apply callback patterns for real-time sensor data processing"
  - "Build AI-driven control nodes that publish commands based on sensor input"
keywords:
  - "rclpy patterns"
  - "AI integration"
  - "Python agents"
  - "perception action loop"
  - "machine learning ROS 2"
  - "autonomous control"
prerequisites:
  - "Lesson 3.1: Building ROS 2 Packages with Python"
  - "Python intermediate (classes, async concepts)"
  - "Basic machine learning concepts (models, inference)"
chapter: "Chapter 3: Building with ROS 2"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 3.2: Bridging Python Agents with rclpy

**Duration**: 75 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Integrate Python AI models with ROS 2 nodes using rclpy
- Design perception-decision-action loops for autonomous behavior
- Implement state management for AI agents in long-running nodes
- Apply callback patterns for real-time sensor data processing
- Build AI-driven control nodes that publish commands based on sensor input

## Why AI Integration Matters in Robotics

A warehouse robot needs to navigate around obstacles while optimizing its path for efficiency. The navigation algorithm uses a neural network trained on thousands of warehouse layouts. How do you connect this Python AI model to the robot's sensors and motors? This lesson teaches you to bridge the gap between AI code and ROS 2 control systems, enabling intelligent autonomous behavior.

## The Perception-Decision-Action Pattern

The **perception-decision-action loop** is the fundamental pattern for autonomous robots. The robot perceives its environment through sensors, an AI agent makes decisions based on that data, and the robot acts by sending commands to actuators. This loop runs continuously, allowing the robot to respond to changing conditions.

In ROS 2, this pattern maps naturally to the publisher-subscriber model. Sensor nodes publish perception data to topics. Your AI agent node subscribes to these topics, processes the data through a model, and publishes action commands to control topics. The beauty of this architecture is that you can swap AI models without changing the sensor or actuator nodes.

Consider a collision avoidance system. A LIDAR sensor publishes distance measurements to `/scan`. Your AI node subscribes to `/scan`, feeds the data into a neural network that predicts safe velocities, and publishes the result to `/cmd_vel`. The motor controller subscribes to `/cmd_vel` and moves the robot. Each component is independent and testable.

The key challenge is timing. Sensors publish data at fixed rates, but AI inference takes variable time depending on model complexity. You must design your node to handle this asynchrony. ROS 2 callbacks help by processing messages as they arrive, but you need to ensure your AI model runs fast enough to keep up with sensor rates. For real-time control, aim for inference times under 100 milliseconds.

## Structuring an AI Agent Node

An AI agent node combines standard ROS 2 patterns with AI model management. The node initializes the model once during startup, then uses it repeatedly in callbacks. This structure keeps inference fast and memory efficient.

The typical structure includes three components: model initialization, sensor callbacks, and command publishing. During initialization, you load your trained model and set up publishers and subscribers. In callbacks, you preprocess sensor data, run inference, and publish results. This separation of concerns makes your code maintainable and testable.

Here is the basic pattern for an AI agent node:

**What we're building**: An obstacle avoidance node that uses a simple decision model to control robot velocity based on LIDAR data.

```python
# obstacle_avoidance_agent.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidanceAgent(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_agent')

        # Initialize AI model (simple rule-based for demonstration)
        self.min_safe_distance = 0.5  # meters
        self.max_speed = 0.5  # m/s

        # Set up subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Obstacle avoidance agent initialized')

    def scan_callback(self, msg):
        # Perception: Extract relevant sensor data
        front_distances = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
        min_distance = min(front_distances)

        # Decision: Apply AI model (simple threshold logic)
        velocity = self.compute_safe_velocity(min_distance)

        # Action: Publish control command
        cmd = Twist()
        cmd.linear.x = velocity
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f'Distance: {min_distance:.2f}m, Velocity: {velocity:.2f}m/s')

    def compute_safe_velocity(self, distance):
        """AI decision model: scale velocity based on obstacle distance"""
        if distance < self.min_safe_distance:
            return 0.0  # Stop
        elif distance < self.min_safe_distance * 2:
            return self.max_speed * 0.3  # Slow down
        else:
            return self.max_speed  # Full speed

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [obstacle_avoidance_agent]: Obstacle avoidance agent initialized
[INFO] [obstacle_avoidance_agent]: Distance: 2.34m, Velocity: 0.50m/s
[INFO] [obstacle_avoidance_agent]: Distance: 0.87m, Velocity: 0.15m/s
[INFO] [obstacle_avoidance_agent]: Distance: 0.42m, Velocity: 0.00m/s
```

**What's happening**:
- Lines 12-14: Model parameters are initialized once during node construction
- Lines 16-22: Subscriber receives LIDAR data, publisher sends velocity commands
- Lines 25-37: Callback implements the perception-decision-action loop
- Lines 39-46: Decision model is separated into its own method for clarity and testing
- The node processes each LIDAR scan and immediately publishes a velocity command

This structure scales to complex AI models. Replace `compute_safe_velocity()` with a neural network inference call, and the rest of the code remains the same.

## Integrating Machine Learning Models

Real AI agents use trained models like neural networks or decision trees. Python's rich ML ecosystem makes integration straightforward. You load the model during node initialization and call it in callbacks.

The key consideration is model format. PyTorch and TensorFlow models need their respective libraries available in your ROS 2 environment. ONNX provides a framework-agnostic format that works across platforms. For production systems, consider converting models to ONNX for better portability.

:::danger ML Model Safety
Always validate ML model outputs before sending commands to physical robots. Implement velocity limits, emergency stops, and sensor validation. Test ML controllers extensively in simulation before deploying to hardware. Never trust model predictions without bounds checking.
:::

**What we're building**: A node that uses a scikit-learn model to predict optimal robot speed based on sensor features.

```python
# ml_velocity_controller.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from sklearn.ensemble import RandomForestRegressor
import pickle

class MLVelocityController(Node):
    def __init__(self):
        super().__init__('ml_velocity_controller')

        # Load pre-trained model
        try:
            with open('velocity_model.pkl', 'rb') as f:
                self.model = pickle.load(f)
            self.get_logger().info('ML model loaded successfully')
        except FileNotFoundError:
            self.get_logger().error('Model file not found, using default behavior')
            self.model = None

        # Set up ROS 2 communication
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def extract_features(self, scan_msg):
        """Convert LIDAR scan to feature vector for ML model"""
        ranges = np.array(scan_msg.ranges)

        # Create features: min, mean, std of distances in different sectors
        front = ranges[len(ranges)//3:2*len(ranges)//3]
        left = ranges[:len(ranges)//3]
        right = ranges[2*len(ranges)//3:]

        features = np.array([
            np.min(front), np.mean(front), np.std(front),
            np.min(left), np.min(right)
        ])
        return features.reshape(1, -1)

    def scan_callback(self, msg):
        if self.model is None:
            return

        # Extract features from sensor data
        features = self.extract_features(msg)

        # Run ML inference
        predicted_velocity = self.model.predict(features)[0]
        predicted_velocity = np.clip(predicted_velocity, 0.0, 0.5)

        # Publish command
        cmd = Twist()
        cmd.linear.x = float(predicted_velocity)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MLVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [ml_velocity_controller]: ML model loaded successfully
```

**What's happening**:
- Lines 15-20: Model is loaded from disk during initialization, with error handling
- Lines 29-42: Feature extraction converts raw sensor data into model input format
- Lines 49-51: ML inference predicts velocity, with clipping to ensure safe bounds
- Line 55: Predicted velocity is published as a Twist message
- The model runs on every LIDAR scan, providing continuous control

This pattern works for any scikit-learn model. For deep learning, replace pickle with PyTorch's `torch.load()` or TensorFlow's `tf.keras.models.load_model()`.

## Managing Agent State Across Callbacks

Many AI agents need to maintain state between callbacks. A path planning agent remembers the current goal. A learning agent tracks performance metrics. ROS 2 nodes are perfect for this because they persist between callbacks.

Store state as instance variables in your node class. Initialize state in `__init__()`, update it in callbacks, and use it in decision-making. This approach keeps state management explicit and testable.

**What we're building**: A goal-seeking agent that remembers its target and adjusts behavior based on progress.

```python
# goal_seeking_agent.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class GoalSeekingAgent(Node):
    def __init__(self):
        super().__init__('goal_seeking_agent')

        # Agent state
        self.current_pose = None
        self.goal_pose = None
        self.goal_reached = False
        self.distance_threshold = 0.5  # meters

        # ROS 2 communication
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Goal-seeking agent ready')

    def odom_callback(self, msg):
        """Update current position from odometry"""
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """Receive new goal from user or planner"""
        self.goal_pose = msg.pose
        self.goal_reached = False
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def control_loop(self):
        """Main decision loop running at 10 Hz"""
        if self.current_pose is None or self.goal_pose is None:
            return

        if self.goal_reached:
            return

        # Calculate distance to goal
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if goal reached
        if distance < self.distance_threshold:
            self.goal_reached = True
            self.stop_robot()
            self.get_logger().info('Goal reached!')
            return

        # Compute control command
        angle_to_goal = math.atan2(dy, dx)
        cmd = Twist()
        cmd.linear.x = min(0.3, distance * 0.5)  # Proportional control
        cmd.angular.z = angle_to_goal * 0.5
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoalSeekingAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [goal_seeking_agent]: Goal-seeking agent ready
[INFO] [goal_seeking_agent]: New goal received: (5.00, 3.00)
[INFO] [goal_seeking_agent]: Goal reached!
```

**What's happening**:
- Lines 12-16: State variables track current position, goal, and completion status
- Lines 32-40: Callbacks update state when new data arrives
- Lines 42-66: Control loop uses state to make decisions at fixed rate
- Lines 54-59: State determines whether to continue or stop
- The agent maintains context across multiple sensor readings and commands

This pattern enables complex behaviors like finite state machines, learning algorithms, and multi-step plans.

## Performance Considerations for Real-Time Control

AI models must run fast enough for real-time control. A robot moving at 1 meter per second travels 10 centimeters in 100 milliseconds. If your model takes longer than that to compute, the robot reacts too slowly.

Profile your model inference time before deploying. Use Python's `time` module to measure how long callbacks take. For deep learning models, consider using GPU acceleration with CUDA. For simpler models, ensure you are using optimized libraries like NumPy with BLAS support.

If your model is too slow, you have several options. First, simplify the model architecture or reduce input dimensions. Second, run inference at a lower rate than sensor updates, using the most recent prediction until a new one is ready. Third, offload inference to a separate thread or process to avoid blocking callbacks. ROS 2 supports multi-threaded executors for this purpose.

Another consideration is memory usage. Loading large models consumes RAM. If you run multiple AI nodes, memory can become a bottleneck. Consider sharing models across nodes using ROS 2 services, where one node hosts the model and others request predictions. This pattern reduces memory usage and centralizes model updates.

## Key Takeaways

- The perception-decision-action loop maps naturally to ROS 2's publisher-subscriber pattern, with AI agents subscribing to sensors and publishing to actuators.
- AI agent nodes load models during initialization and call them in callbacks, keeping inference fast and memory efficient.
- Machine learning models integrate easily with rclpy using standard Python libraries like scikit-learn, PyTorch, or TensorFlow.
- Agent state persists as instance variables in the node class, enabling complex behaviors that span multiple callbacks.
- Real-time control requires model inference times under 100 milliseconds, achievable through model optimization, GPU acceleration, or asynchronous execution.

## Check Your Understanding

1. Describe the three stages of the perception-decision-action loop and how they map to ROS 2 publishers and subscribers.

2. You have a PyTorch model that takes 200 milliseconds to run inference, but your robot needs control updates every 50 milliseconds. What are three strategies to handle this timing mismatch?

3. Examine this code: `self.model = pickle.load(f)` in `__init__()`. Why is it better to load the model during initialization rather than in the callback function?

4. Your AI agent needs to remember the last 10 sensor readings to detect trends. How would you implement this state management in a ROS 2 node?

5. You want to run the same object detection model in three different nodes (navigation, manipulation, and logging). What ROS 2 pattern would you use to avoid loading the model three times?

## Next Steps

Now that you can integrate AI agents with ROS 2, the next lesson covers launch files and parameters. You will learn to start multiple nodes together, configure them with parameters, and manage complex multi-node systems.

---
**Hardware Tier 1 Note**: Use cloud-based Jupyter notebooks or Google Colab to prototype and test your AI models. Examine the integration patterns in this lesson conceptually, and prepare to implement them when you have access to a local ROS 2 environment. The AI model development can happen entirely in the cloud, with only the ROS 2 integration requiring Tier 2 hardware.
