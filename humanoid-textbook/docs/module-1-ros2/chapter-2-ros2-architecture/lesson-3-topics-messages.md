---
id: lesson-3-topics-messages
title: "Lesson 2.3: Topics and Message Passing"
sidebar_position: 3
sidebar_label: "2.3 Topics & Messages"
description: "Learn the publisher-subscriber pattern, message types, and topic communication in ROS 2"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Use The Construct to publish and subscribe to topics in browser"
learning_objectives:
  - "Explain the publisher-subscriber pattern and its advantages in distributed robotics"
  - "Implement publisher and subscriber nodes using rclpy"
  - "Use standard message types including Twist, LaserScan, and Image"
  - "Apply topic naming conventions for clear system architecture"
  - "Configure Quality of Service profiles for reliable message delivery"
keywords:
  - "ROS 2 topics"
  - "publisher subscriber"
  - "Twist message"
  - "LaserScan"
  - "QoS profiles"
  - "message passing"
prerequisites:
  - "Lesson 2.1: ROS 2 Core Concepts"
  - "Lesson 2.2: Nodes - The Building Blocks"
  - "Python classes and object-oriented programming"
chapter: "Chapter 2: ROS 2 Architecture"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 2.3: Topics and Message Passing

**Duration**: 60 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain the publisher-subscriber pattern and its advantages in distributed robotics
- Implement publisher and subscriber nodes using rclpy
- Use standard message types including Twist, LaserScan, and Image
- Apply topic naming conventions for clear system architecture
- Configure Quality of Service profiles for reliable message delivery

## Why Topics Matter in Robotics

A warehouse robot receives camera images at 30 frames per second while its motors need velocity commands every 10 milliseconds. How do these different components communicate without blocking each other? ROS 2 topics provide the answer through asynchronous message passing that keeps your robot responsive and modular.

## Understanding the Publisher-Subscriber Pattern

The **publisher-subscriber pattern** is a communication model where nodes exchange data without direct connections. A **publisher** sends messages to a named channel called a **topic**, and any **subscriber** listening to that topic receives the messages. This pattern decouples nodes, meaning publishers do not know who is listening, and subscribers do not know who is sending.

Think of a topic like a radio station. The broadcaster transmits on a frequency, and anyone tuned to that frequency hears the broadcast. The broadcaster does not need to know how many listeners exist. Similarly, a camera node publishes images to a topic, and vision processing nodes subscribe to receive those images.

This decoupling provides three key advantages. First, you can add or remove subscribers without modifying the publisher. Second, multiple subscribers can receive the same data simultaneously. Third, nodes can start and stop independently without breaking the system. A navigation node can subscribe to laser scan data whenever it needs to plan a path, then unsubscribe when idle.

ROS 2 topics are **typed**, meaning each topic carries a specific message type. A topic publishing `geometry_msgs/Twist` messages cannot suddenly send `sensor_msgs/Image` messages. This type safety prevents errors and makes the system predictable. When you subscribe to a topic, you know exactly what data structure to expect.

## Message Types in ROS 2

ROS 2 provides standard message types for common robotics data. These messages are defined in packages like `geometry_msgs`, `sensor_msgs`, and `std_msgs`. Using standard types ensures compatibility across different robots and libraries.

The **Twist message** (`geometry_msgs/msg/Twist`) represents velocity commands for mobile robots. It contains two 3D vectors: `linear` for forward/backward/sideways motion and `angular` for rotation. Most mobile robots use only `linear.x` for forward speed and `angular.z` for turning. For example, setting `linear.x = 0.5` and `angular.z = 0.1` makes a robot drive forward at 0.5 meters per second while rotating slowly.

The **LaserScan message** (`sensor_msgs/msg/LaserScan`) carries data from LIDAR sensors. It includes an array of distance measurements, the angular range covered, and timing information. Each element in the `ranges` array represents the distance to the nearest obstacle at a specific angle. This data is essential for obstacle avoidance and mapping. A typical LIDAR might provide 360 measurements covering a full circle around the robot.

The **Image message** (`sensor_msgs/msg/Image`) transports camera data. It includes the pixel data, image dimensions, encoding format, and timestamp. The encoding field specifies whether the image is RGB, grayscale, or depth data. Vision processing nodes subscribe to image topics to perform tasks like object detection or visual servoing. Images are large messages, so you must consider bandwidth when publishing at high frame rates.

## Creating a Publisher Node

Let's build a node that publishes velocity commands to control a robot. This pattern is fundamental for any system that needs to send motion commands.

:::danger Robot Safety
When publishing velocity commands to real robots, always implement emergency stop functionality and velocity limits. Test all motion commands in simulation first. Never run untested velocity commands on physical hardware.
:::

**What we're building**: A velocity publisher that sends movement commands to a mobile robot at regular intervals.

```python
# velocity_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.1  # Rotate at 0.1 rad/s
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [velocity_publisher]: Velocity publisher started
[INFO] [velocity_publisher]: Published: linear=0.5, angular=0.1
[INFO] [velocity_publisher]: Published: linear=0.5, angular=0.1
```

**What's happening**:
- Line 8: `create_publisher(Twist, '/cmd_vel', 10)` creates a publisher on the `/cmd_vel` topic with a queue size of 10 messages
- Line 9: `create_timer(0.5, self.publish_velocity)` calls the publish method every 0.5 seconds
- Line 14-15: We create a Twist message and set the linear and angular velocities
- Line 17: `publish(msg)` sends the message to all subscribers on the `/cmd_vel` topic

The queue size parameter determines how many messages to buffer if subscribers cannot keep up. A queue size of 10 is reasonable for most control applications.

## Creating a Subscriber Node

Now let's build a node that receives laser scan data from a LIDAR sensor. Subscribers process incoming messages through callback functions.

**What we're building**: A laser scan subscriber that monitors obstacle distances and warns when objects are too close.

```python
# laser_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Laser subscriber started')

    def scan_callback(self, msg):
        # Find the minimum distance in the scan
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f} meters')

        if min_distance < 0.5:
            self.get_logger().warn('WARNING: Obstacle too close!')

def main(args=None):
    rclpy.init(args=args)
    node = LaserSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [laser_subscriber]: Laser subscriber started
[INFO] [laser_subscriber]: Closest obstacle: 2.34 meters
[INFO] [laser_subscriber]: Closest obstacle: 1.87 meters
[WARN] [laser_subscriber]: WARNING: Obstacle too close!
```

**What's happening**:
- Line 9-14: `create_subscription()` registers a callback function that runs whenever a message arrives
- Line 17: The `scan_callback` function receives the LaserScan message as a parameter
- Line 19: `msg.ranges` is an array of distance measurements from the LIDAR
- Line 23: We check if any obstacle is closer than 0.5 meters and issue a warning

The callback function executes asynchronously whenever a message arrives. You should keep callbacks fast to avoid blocking other node operations.

## Topic Naming Conventions

Proper topic names make your system architecture clear and prevent naming conflicts. ROS 2 follows specific conventions for topic names.

Topic names use forward slashes to create namespaces. A name like `/robot1/camera/image` indicates the image topic belongs to robot1's camera. The leading slash makes it an **absolute topic name** that is the same regardless of which node uses it. Without the leading slash, the name is **relative** and gets prefixed with the node's namespace.

Use descriptive names that indicate the data type and source. Good examples include `/cmd_vel` for velocity commands, `/scan` for laser data, and `/camera/image_raw` for unprocessed camera images. Avoid generic names like `/data` or `/output` that do not convey meaning.

Standard topic names exist for common robot interfaces. Mobile robots typically use `/cmd_vel` for velocity commands and `/odom` for odometry data. Cameras often publish to `/camera/image_raw` and `/camera/camera_info`. Following these conventions makes your robot compatible with existing tools and libraries. For example, the RViz visualization tool knows to look for `/scan` when displaying LIDAR data.

Namespaces help organize complex systems. A humanoid robot might have `/left_arm/joint_states` and `/right_arm/joint_states` to separate the two arms. You can launch nodes with namespace parameters to automatically prefix all their topics, making it easy to run multiple robots in the same ROS 2 network.

## Quality of Service Profiles for Topics

**Quality of Service (QoS)** profiles control how messages are delivered between publishers and subscribers. Different applications need different reliability guarantees, and QoS lets you configure this behavior.

The two most important QoS settings are **reliability** and **durability**. Reliability determines whether messages must be delivered or can be dropped. The `RELIABLE` policy ensures every message reaches subscribers, while `BEST_EFFORT` allows dropping messages if the network is slow. Use `RELIABLE` for critical commands like motor control and `BEST_EFFORT` for high-frequency sensor data where the latest value matters most.

Durability controls whether late-joining subscribers receive old messages. The `TRANSIENT_LOCAL` policy stores recent messages so new subscribers get the last published value. The `VOLATILE` policy only delivers messages published after subscription. Use `TRANSIENT_LOCAL` for configuration data that changes rarely, and `VOLATILE` for continuous sensor streams.

Here is how to create a publisher with custom QoS settings:

**What we're building**: A reliable publisher for critical robot commands that must not be lost.

```python
# reliable_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

class ReliablePublisher(Node):
    def __init__(self):
        super().__init__('reliable_publisher')

        # Configure QoS for reliable delivery
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.get_logger().info('Reliable publisher created with custom QoS')

def main(args=None):
    rclpy.init(args=args)
    node = ReliablePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [reliable_publisher]: Reliable publisher created with custom QoS
```

**What's happening**:
- Line 12: We create a QoSProfile object with a queue depth of 10
- Line 13: `ReliabilityPolicy.RELIABLE` ensures messages are not dropped
- Line 14: `DurabilityPolicy.TRANSIENT_LOCAL` stores messages for late subscribers
- Line 16: We pass the QoS profile to `create_publisher()` instead of a simple integer

Subscribers must use compatible QoS settings to receive messages. A `RELIABLE` publisher and `BEST_EFFORT` subscriber will not connect. ROS 2 logs warnings when QoS policies are incompatible.

## Key Takeaways

- The publisher-subscriber pattern decouples nodes, allowing them to communicate without direct connections or knowledge of each other.
- Topics are typed channels where publishers send messages and subscribers receive them asynchronously.
- Standard message types like Twist, LaserScan, and Image provide common data structures for robot communication.
- Topic names should be descriptive and follow conventions like `/cmd_vel` for velocity and `/scan` for LIDAR data.
- Quality of Service profiles control message delivery reliability and durability based on application requirements.

## Check Your Understanding

1. What are the three main advantages of the publisher-subscriber pattern compared to direct node-to-node communication?

2. You are building a mobile robot that needs to receive velocity commands and publish odometry data. What topic names and message types would you use, and why?

3. A camera publishes images at 30 Hz, but your vision processing node can only handle 10 Hz. Should you use `RELIABLE` or `BEST_EFFORT` QoS for this topic? Explain your reasoning.

4. Examine this code: `self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)`. What is wrong with the topic name, and how would you fix it?

5. Your subscriber is not receiving messages from a publisher, even though both nodes are running. What are three QoS-related reasons this might happen?

## Next Steps

Now that you understand how nodes communicate through topics, the next lesson covers services and actions for request-response interactions and long-running tasks. You will learn when to use topics versus services versus actions.

---
**Hardware Tier 1 Note**: Use [The Construct](https://www.theconstructsim.com/) browser-based ROS 2 environment to run publisher and subscriber examples. The platform provides pre-configured workspaces with simulated robots that publish sensor data to topics you can subscribe to.
