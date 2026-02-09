---
id: lesson-1-core-concepts
title: "Lesson 2.1: ROS 2 Core Concepts"
sidebar_position: 1
sidebar_label: "2.1 Core Concepts"
description: "Learn ROS 2 architecture, DDS middleware, QoS settings, and the computation graph for distributed robot systems."
duration_minutes: 45
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
tier_1_path: "Use The Construct or Foxglove Studio for browser-based ROS 2 exploration"
learning_objectives:
  - "Explain the ROS 2 distributed architecture and its advantages over centralized systems"
  - "Describe how DDS middleware enables peer-to-peer communication between nodes"
  - "Identify appropriate Quality of Service settings for different robot communication patterns"
  - "Visualize the computation graph to understand node relationships"
keywords:
  - "ROS 2 architecture"
  - "DDS middleware"
  - "Quality of Service"
  - "computation graph"
  - "peer-to-peer communication"
  - "distributed systems"
prerequisites:
  - "Chapter 1: Introduction to Physical AI"
  - "Basic understanding of distributed systems"
  - "Python fundamentals"
chapter: "Chapter 2: ROS 2 Architecture"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 2.1: ROS 2 Core Concepts

**Duration**: 45 minutes
**Hardware Tier**: Tier 1 (Laptop + Cloud)
**Layer**: L1 (Manual Foundation)

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain the ROS 2 distributed architecture and its advantages over centralized systems
- Describe how DDS middleware enables peer-to-peer communication between nodes
- Identify appropriate Quality of Service settings for different robot communication patterns
- Visualize the computation graph to understand node relationships

## Why ROS 2 Architecture Matters

A warehouse robot loses connection to its central server. In ROS 1, the entire system would freeze. In ROS 2, the robot continues operating because nodes communicate directly with each other. This architectural shift from centralized to distributed systems makes modern robots more resilient, scalable, and production-ready.

## What is ROS 2 Architecture

**ROS 2 (Robot Operating System 2)** is a distributed middleware framework that breaks complex robot systems into small, independent programs called **nodes**. Each node performs one specific task, such as reading sensor data, processing images, or controlling motors. Unlike traditional monolithic programs, ROS 2 nodes run as separate processes that communicate through well-defined channels.

The architecture follows three core principles. First, nodes are independent executables that can start, stop, and restart without affecting other parts of the system. Second, communication happens through standardized message types, ensuring different nodes can work together even if written by different teams. Third, the system is language-agnostic, supporting Python, C++, and other languages through client libraries.

ROS 2 Humble Hawksbill is the current Long-Term Support release, maintained until 2027. It runs on Ubuntu 22.04, Windows 10, and macOS, making it accessible across development environments. The Humble release focuses on real-time performance, security improvements, and production deployment features that were missing in ROS 1.

This modular design means you can develop a camera node on your laptop, test a navigation node in simulation, and deploy both to a physical robot without rewriting code. Each node remains a black box to others, exposing only its inputs and outputs through topics, services, and actions.

## DDS Middleware and Peer-to-Peer Communication

**DDS (Data Distribution Service)** is the communication layer that powers ROS 2. Unlike ROS 1, which required a central master node to coordinate all communication, DDS enables **peer-to-peer communication** where nodes discover and connect to each other automatically. This eliminates the single point of failure that plagued ROS 1 systems.

When you start a ROS 2 node, DDS broadcasts a discovery message on the local network. Other nodes receive this message and learn about the new node's publishers, subscribers, services, and actions. Within seconds, nodes establish direct connections without human intervention. This process is called **automatic discovery**.

DDS handles the low-level networking details that would otherwise require hundreds of lines of socket programming. It manages message serialization, network transport, congestion control, and reliability guarantees. As a robotics developer, you work with high-level ROS 2 APIs while DDS handles the complexity underneath.

Multiple DDS implementations exist, including Fast DDS (default in Humble), Cyclone DDS, and RTI Connext. ROS 2 abstracts these differences through a common interface called **rmw (ROS middleware)**. You can switch DDS implementations by changing an environment variable, allowing you to choose the best performance for your hardware.

The peer-to-peer model scales better than centralized architectures. A robot with 50 nodes doesn't bottleneck through a single master. If one node crashes, others continue operating. This resilience is critical for production robots in warehouses, hospitals, and factories where downtime costs money.

## Quality of Service Settings for Reliable Communication

**Quality of Service (QoS)** settings control how messages travel between nodes. ROS 2 provides fine-grained control over reliability, durability, and delivery guarantees through QoS profiles. Choosing the right QoS prevents data loss in critical systems while allowing best-effort delivery for high-frequency sensor data.

The **reliability** setting has two options. **Reliable** delivery guarantees every message arrives, retransmitting lost packets until acknowledged. Use this for command messages like motor control where missing a stop command could damage hardware. **Best-effort** delivery sends messages once without confirmation. Use this for sensor streams like camera images where the next frame makes old data obsolete.

The **durability** setting determines whether late-joining subscribers receive old messages. **Transient local** durability stores recent messages so new subscribers catch up on missed data. This works well for robot state information. **Volatile** durability discards old messages, appropriate for real-time sensor feeds where only current data matters.

The **history** setting controls message queuing. A **keep last** policy with depth 10 stores the 10 most recent messages. If a subscriber falls behind, it skips to the newest messages rather than processing a backlog. A **keep all** policy never discards messages, risking memory exhaustion if subscribers can't keep up.

Here's how different robot systems use QoS profiles:

| System Component | Reliability | Durability | History | Reason |
|-----------------|-------------|------------|---------|---------|
| Motor commands | Reliable | Volatile | Keep last 1 | Must arrive, only latest matters |
| Camera stream | Best-effort | Volatile | Keep last 5 | High frequency, old frames useless |
| Robot state | Reliable | Transient local | Keep last 10 | New nodes need current state |
| Emergency stop | Reliable | Volatile | Keep last 1 | Critical, must not be lost |

Mismatched QoS between publisher and subscriber prevents communication. A reliable publisher won't connect to a best-effort subscriber. ROS 2 enforces these rules to prevent silent data loss that could cause robot failures.

## The Computation Graph Visualizes Node Relationships

The **computation graph** is a visual representation of all active nodes and their communication channels. Nodes appear as circles, topics as arrows connecting them. This graph helps you understand data flow, debug connection issues, and verify your system architecture matches your design.

Every ROS 2 system has an implicit graph structure. When you run `ros2 node list`, you see all active nodes. When you run `ros2 topic list`, you see all communication channels. The `rqt_graph` tool combines this information into an interactive diagram where you can click nodes to inspect their connections.

Consider a simple mobile robot with three nodes. A `camera_node` publishes images to the `/camera/image` topic. A `vision_node` subscribes to that topic and publishes detected objects to `/detected_objects`. A `navigation_node` subscribes to detected objects and publishes velocity commands to `/cmd_vel`. The computation graph shows this pipeline visually, making the data flow obvious.

The graph updates in real-time as nodes start and stop. If the vision node crashes, the graph shows the broken connection between camera and navigation. This immediate feedback helps you diagnose problems faster than reading log files. You can also filter the graph to show only specific nodes or topics, reducing clutter in complex systems.

Understanding the computation graph is essential for debugging. If your robot isn't responding to commands, check whether the command publisher and motor subscriber are connected. If sensor data seems stale, verify the sensor node is publishing at the expected rate. The graph makes these invisible connections visible.

## Creating Your First ROS 2 Node

Let's build a minimal ROS 2 node to see these concepts in action. This node will initialize the ROS 2 runtime, create a node object, and log a message to demonstrate the basic structure every node follows.

**What we're building**: A simple ROS 2 node that demonstrates initialization and logging.

```python
# minimal_node.py
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [minimal_node]: Minimal node has started
```

**What's happening**:
- Line 2: `rclpy` is the Python client library for ROS 2, providing all core functionality
- Line 3: `Node` is the base class that all ROS 2 nodes inherit from
- Line 6: `super().__init__('minimal_node')` registers this node with the name 'minimal_node' in the computation graph
- Line 7: `get_logger().info()` writes to the ROS 2 logging system, which timestamps and categorizes messages
- Line 10: `rclpy.init()` initializes the ROS 2 runtime and connects to DDS middleware
- Line 12: `rclpy.spin()` keeps the node running and processing callbacks until interrupted
- Line 13: `rclpy.shutdown()` cleanly disconnects from DDS and releases resources

This pattern appears in every ROS 2 node you'll write. The initialization, spin, and shutdown sequence ensures proper resource management and clean exits when you press Ctrl+C.

:::tip Try This
Save this code as `minimal_node.py` and run it with `python3 minimal_node.py`. In another terminal, run `ros2 node list` to see your node in the computation graph. Press Ctrl+C to stop the node cleanly.
:::

## Key Takeaways

- ROS 2 uses a distributed architecture where nodes communicate peer-to-peer through DDS middleware, eliminating the single point of failure from ROS 1's centralized master.
- DDS provides automatic node discovery, allowing nodes to find and connect to each other without manual configuration.
- Quality of Service settings control message delivery guarantees, with reliable delivery for critical commands and best-effort for high-frequency sensor data.
- The computation graph visualizes all nodes and their communication channels, making system architecture and data flow visible for debugging.
- Every ROS 2 node follows the same pattern: initialize the runtime, create a node object, spin to process callbacks, and shutdown cleanly.

## Check Your Understanding

1. What advantage does ROS 2's peer-to-peer architecture provide over ROS 1's centralized master node?

2. A robot's emergency stop button publishes to the `/emergency_stop` topic. Should this use reliable or best-effort QoS? Explain your reasoning.

3. You run `ros2 node list` and see five nodes, but `rqt_graph` shows only three nodes connected. What does this tell you about your system?

4. Compare transient local and volatile durability. When would you choose each for a robot's battery status topic?

5. In the minimal node example, what would happen if you removed the `rclpy.spin(node)` line? Why?

## Next Steps

Now that you understand ROS 2's distributed architecture and communication patterns, the next lesson explores nodes in depth. You'll learn about node lifecycle management, parameters, and how to structure nodes for maintainability and testing.

---

**Hardware Tier 1 Note**: Use [The Construct](https://www.theconstructsim.com/) for a browser-based ROS 2 environment with pre-configured Humble installation. Alternatively, [Foxglove Studio](https://foxglove.dev/) provides visualization tools without requiring local ROS 2 installation. Both platforms let you run the minimal node example and explore the computation graph through web interfaces.
