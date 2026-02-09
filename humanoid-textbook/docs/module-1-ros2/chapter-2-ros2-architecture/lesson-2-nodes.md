---
id: lesson-2-nodes
title: "Lesson 2.2: Nodes - The Building Blocks"
sidebar_position: 2
sidebar_label: "2.2 Nodes"
description: "Learn to create, manage, and understand ROS 2 nodes as independent executables with lifecycle management"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Use The Construct ROS 2 online environment to create and run nodes"
learning_objectives:
  - "Define what a ROS 2 node is and explain its role in the computation graph"
  - "Create a basic node using the rclpy Python library"
  - "Explain the four lifecycle states and their transitions"
  - "Implement timer callbacks for periodic node operations"
  - "Execute and inspect running nodes using ROS 2 command-line tools"
keywords:
  - "ROS 2 nodes"
  - "rclpy"
  - "node lifecycle"
  - "timer callbacks"
  - "computation graph"
  - "node management"
prerequisites:
  - "Lesson 2.1: ROS 2 Core Concepts"
  - "Python classes and inheritance"
  - "Basic terminal commands"
chapter: "Chapter 2: ROS 2 Architecture"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 2.2: Nodes - The Building Blocks

**Duration**: 60 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu with ROS 2 Humble)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Define what a ROS 2 node is and explain its role in the computation graph
- Create a basic node using the rclpy Python library
- Explain the four lifecycle states and their transitions
- Implement timer callbacks for periodic node operations
- Execute and inspect running nodes using ROS 2 command-line tools

## Why Nodes Matter in Robotics

Every robot system breaks down into smaller tasks. A camera node captures images. A vision node detects objects. A navigation node plans paths. A motor node controls wheels. Nodes are the independent executables that perform these tasks, communicating through topics and services to create intelligent robot behavior.

## What is a ROS 2 Node

A **node** is an independent executable process that performs a specific computation in a ROS 2 system. Each node runs in its own process, has a unique name, and can communicate with other nodes through topics, services, and actions. Think of nodes as specialized workers in a factory. Each worker has one job and communicates with others to complete the overall task.

Nodes provide modularity and fault isolation. If one node crashes, the others continue running. You can test nodes independently, replace them with improved versions, or run multiple instances with different configurations. A typical robot system runs dozens of nodes simultaneously, each handling a specific responsibility like sensor processing, motion planning, or motor control.

The **computation graph** is the network of all running nodes and their communication channels. ROS 2 uses DDS (Data Distribution Service) middleware to discover nodes automatically. When you start a node, it announces itself to the network. Other nodes can find it and establish connections without manual configuration. This peer-to-peer architecture makes ROS 2 systems scalable and resilient.

Every node inherits from the `Node` base class provided by rclpy (the Python client library). This base class handles initialization, communication setup, and lifecycle management. Your custom node adds specific behavior by overriding methods and creating publishers, subscribers, or timers.

## Creating Your First Node with rclpy

The rclpy library provides the Python API for creating ROS 2 nodes. Every node follows a standard pattern: import rclpy, create a class that inherits from `Node`, initialize the node with a name, and spin the node to process callbacks. Let's build a minimal node that logs a message when it starts.

**What we're building**: A minimal ROS 2 node that initializes and logs a startup message.

```python
# minimal_node.py
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node has started!')

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
[INFO] [1707484523.123456789] [minimal_node]: Minimal node has started!
```

**What's happening**:
- Line 2: Import `rclpy`, the Python client library for ROS 2
- Line 3: Import the `Node` base class that all nodes inherit from
- Line 6: Call the parent constructor with `super().__init__('minimal_node')` to register the node name
- Line 7: Use `self.get_logger().info()` to log messages with timestamps and node name
- Line 10: Initialize the ROS 2 Python client library before creating nodes
- Line 11: Create an instance of our custom node class
- Line 12: `rclpy.spin(node)` blocks and processes callbacks until shutdown
- Line 13: Clean up resources when the node exits

The node name `'minimal_node'` must be unique in your system. If you launch two nodes with the same name, ROS 2 will append a suffix to make them unique. Node names use underscores by convention, not hyphens or spaces.

## Node Class Structure and Initialization

Understanding the node class structure helps you build more complex nodes. The `__init__` method is where you configure your node's behavior by creating publishers, subscribers, services, timers, and parameters. All setup happens in the constructor before the node starts spinning.

**What we're building**: A node with multiple components to demonstrate proper initialization structure.

```python
# structured_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StructuredNode(Node):
    def __init__(self):
        super().__init__('structured_node')

        # Node configuration
        self.declare_parameter('update_rate', 1.0)
        rate = self.get_parameter('update_rate').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'status', 10)

        # Create timer
        self.timer = self.create_timer(rate, self.timer_callback)

        # Initialize state
        self.counter = 0

        self.get_logger().info(f'Node initialized with rate: {rate} Hz')

    def timer_callback(self):
        msg = String()
        msg.data = f'Update {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = StructuredNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [1707484523.123456789] [structured_node]: Node initialized with rate: 1.0 Hz
```

**What's happening**:
- Line 11: Declare a parameter with a default value for runtime configuration
- Line 12: Retrieve the parameter value to use in node logic
- Line 15: Create a publisher on the `status` topic with queue size 10
- Line 18: Create a timer that calls `timer_callback` at the specified rate
- Line 21: Initialize instance variables to track node state
- Line 25-28: Timer callback executes periodically, publishing messages

This structure separates configuration (parameters), communication (publishers), scheduling (timers), and state (instance variables). Following this pattern makes your nodes easier to understand, test, and maintain.

## Node Lifecycle States and Transitions

ROS 2 nodes support an explicit **lifecycle** that provides fine-grained control over node behavior. Lifecycle nodes transition through four primary states: **configuring**, **inactive**, **active**, and **shutting down**. This lifecycle enables predictable startup, graceful shutdown, and runtime reconfiguration without restarting the entire system.

The **configuring** state is where the node allocates resources like memory buffers, opens files, or establishes network connections. The node is not yet processing data. If configuration fails, the node can transition to an error state without affecting other nodes. This state answers the question: "Can this node run with the current configuration?"

The **inactive** state means the node is fully configured but not actively processing. Publishers exist but don't send messages. Subscribers exist but don't process incoming data. Timers are paused. This state is useful for hot-standby scenarios where you want a node ready to activate instantly without reconfiguration overhead.

The **active** state is where the node performs its primary function. Publishers send messages, subscribers process data, timers fire callbacks, and services respond to requests. This is the normal operating state for most nodes. Transitions to active only succeed if the node successfully completed configuration.

The **shutting down** state handles cleanup. The node releases resources, closes connections, and saves state if needed. After shutdown completes, the node transitions to the finalized state and can be safely destroyed. Proper shutdown prevents resource leaks and ensures data integrity.

Standard nodes (non-lifecycle) skip these explicit states and go directly to active when created. Lifecycle nodes require explicit state transitions using lifecycle management commands. Use lifecycle nodes when you need controlled startup sequences, runtime reconfiguration, or coordinated multi-node state changes.

## Running and Managing Nodes

ROS 2 provides command-line tools to run, inspect, and manage nodes. The `ros2 run` command executes a node from an installed package. The `ros2 node` command suite provides information about running nodes. These tools are essential for development, debugging, and system monitoring.

To run a node, use the syntax `ros2 run <package_name> <executable_name>`. For example, if you created a package called `my_robot_pkg` with the `minimal_node.py` executable, run it with:

```bash
ros2 run my_robot_pkg minimal_node
```

The node starts and begins processing. Press Ctrl+C to stop it gracefully. The node receives a shutdown signal, completes its cleanup, and exits.

To list all running nodes, use:

```bash
ros2 node list
```

This shows the unique name of every active node in your system. You'll see system nodes (like parameter servers) and your custom nodes.

To get detailed information about a specific node, use:

```bash
ros2 node info /minimal_node
```

This displays the node's publishers, subscribers, services, actions, and parameters. It's invaluable for understanding how a node connects to the computation graph and debugging communication issues.

You can also remap node names at runtime without changing code:

```bash
ros2 run my_robot_pkg minimal_node --ros-args --remap __node:=custom_name
```

This launches the node with the name `custom_name` instead of `minimal_node`. Remapping is useful when running multiple instances of the same node or integrating with existing systems that expect specific node names.

## Timer Callbacks for Periodic Tasks

Many robot tasks require periodic execution. Sensor nodes publish data at fixed rates. Control loops run at specific frequencies. Status monitors check system health on intervals. ROS 2 **timer callbacks** provide a clean pattern for scheduling periodic work without manual thread management.

A timer executes a callback function at a specified rate. You create timers in the node's `__init__` method using `self.create_timer(period_sec, callback_function)`. The timer runs in the node's executor thread, so callbacks should complete quickly to avoid blocking other node operations.

**What we're building**: A node that publishes heartbeat messages at 2 Hz using a timer callback.

```python
# heartbeat_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')
        self.publisher = self.create_publisher(String, 'heartbeat', 10)
        self.timer = self.create_timer(0.5, self.publish_heartbeat)
        self.get_logger().info('Heartbeat node started at 2 Hz')

    def publish_heartbeat(self):
        msg = String()
        msg.data = f'Heartbeat at {datetime.now().strftime("%H:%M:%S")}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [1707484523.123456789] [heartbeat_node]: Heartbeat node started at 2 Hz
[INFO] [1707484523.623456789] [heartbeat_node]: Published: Heartbeat at 14:35:23
[INFO] [1707484524.123456789] [heartbeat_node]: Published: Heartbeat at 14:35:24
[INFO] [1707484524.623456789] [heartbeat_node]: Published: Heartbeat at 14:35:24
```

**What's happening**:
- Line 11: Create a timer with 0.5 second period (2 Hz frequency)
- Line 11: Pass `self.publish_heartbeat` as the callback function (no parentheses)
- Line 14-18: Timer callback executes every 0.5 seconds automatically
- Line 16: Generate a timestamp to show when each heartbeat occurs
- Line 17: Publish the message to the `heartbeat` topic

Timer callbacks should avoid blocking operations like file I/O or network requests. If your callback needs to perform slow operations, consider using a separate thread or async patterns. Keep callbacks fast to maintain consistent timing and responsive node behavior.

## Key Takeaways

- A ROS 2 node is an independent executable process with a unique name that performs specific computation and communicates through topics, services, and actions.
- Every node inherits from the `Node` base class provided by rclpy and initializes with `super().__init__('node_name')` in the constructor.
- Lifecycle nodes transition through four states: configuring (resource allocation), inactive (ready but not processing), active (normal operation), and shutting down (cleanup).
- The `ros2 run` command executes nodes, while `ros2 node list` and `ros2 node info` provide runtime inspection of the computation graph.
- Timer callbacks enable periodic task execution at specified rates using `self.create_timer(period_sec, callback_function)` without manual thread management.

## Check Your Understanding

1. What happens if you launch two nodes with the same name in a ROS 2 system? How does ROS 2 handle this situation?

2. Explain the difference between the inactive and active lifecycle states. Give an example scenario where you would want a node in the inactive state.

3. You create a timer with `self.create_timer(0.1, self.callback)` but your callback function takes 0.2 seconds to execute. What problem does this create, and how would you fix it?

4. Write the command to get detailed information about a running node named `/camera_driver`, including its publishers and subscribers.

5. In the structured node example, what would happen if you called `self.create_publisher()` before calling `super().__init__()`? Why is the order important?

## Next Steps

Now that you understand how to create and manage nodes, the next lesson covers how nodes communicate using topics and messages. You'll learn the publisher-subscriber pattern that enables data flow between nodes in the computation graph.

---

**Hardware Tier 1 Note**: Use [The Construct ROS 2 online environment](https://www.theconstructsim.com/) to create and run nodes in your browser. The platform provides a pre-configured ROS 2 Humble workspace with terminal access and visualization tools. All code examples in this lesson work identically in the cloud environment.
