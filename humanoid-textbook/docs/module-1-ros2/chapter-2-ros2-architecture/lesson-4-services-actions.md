---
id: lesson-4-services-actions
title: "Lesson 2.4: Services and Actions"
sidebar_position: 4
sidebar_label: "2.4 Services & Actions"
description: "Learn ROS 2 services for request-response and actions for long-running tasks with feedback"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Use The Construct to create service servers and clients in browser"
learning_objectives:
  - "Explain the difference between topics, services, and actions in ROS 2"
  - "Implement a service server and client for synchronous request-response communication"
  - "Build an action server and client with feedback for long-running tasks"
  - "Define custom service and action types using .srv and .action files"
keywords:
  - "ROS 2 services"
  - "ROS 2 actions"
  - "request-response"
  - "action feedback"
  - "service definitions"
  - "action definitions"
prerequisites:
  - "Lesson 2.3: Topics and Message Passing"
  - "Understanding of ROS 2 nodes and publishers/subscribers"
chapter: "Chapter 2: ROS 2 Architecture"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 2.4: Services and Actions

**Duration**: 60 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Explain the difference between topics, services, and actions in ROS 2
- Implement a service server and client for synchronous request-response communication
- Build an action server and client with feedback for long-running tasks
- Define custom service and action types using .srv and .action files

## Why Services and Actions Matter

Topics work well for continuous data streams like sensor readings. But what happens when you need to ask a robot "What is your battery level?" and wait for an answer? Or when you want to send a robot to a location and receive progress updates along the way? These scenarios require different communication patterns. Services handle quick questions with immediate answers. Actions manage long-running tasks that need feedback and can be canceled mid-execution.

## Understanding Request-Response Communication in ROS 2

ROS 2 provides three communication patterns, each designed for specific use cases. **Topics** use a publish-subscribe model where data flows continuously in one direction. Publishers send messages without knowing who receives them, and subscribers receive messages without knowing who sent them. This works well for sensor data that updates constantly.

**Services** introduce a different pattern: synchronous request-response communication. A client sends a request to a server and waits for a response. The client blocks until it receives an answer or times out. This pattern mirrors function calls in traditional programming. You ask a question, the system processes it, and you get an answer.

**Actions** extend the service pattern for tasks that take time to complete. When you tell a robot to navigate across a room, you want progress updates, not silence followed by a final result. Actions provide three communication channels: a goal (what to do), feedback (progress updates), and a result (final outcome). Clients can also cancel actions mid-execution if plans change.

The key difference lies in timing and feedback. Topics never wait. Services wait for one response. Actions wait but receive multiple feedback messages along the way.

## ROS 2 Services for Synchronous Operations

A **service** in ROS 2 consists of two parts: a server that provides functionality and a client that requests it. The server waits for incoming requests, processes them, and sends back responses. The client sends a request and blocks until the response arrives.

Services use a request-response message pair defined in a `.srv` file. The file contains two sections separated by three dashes: the request fields above and the response fields below. For example, a service that adds two integers defines two integer inputs and one integer output.

Common use cases for services include querying robot state, setting parameters, triggering one-time actions, and requesting computations. Services should complete quickly, typically within a few seconds. For longer operations, use actions instead.

**What we're building**: A service client that adds two integers using the built-in AddTwoInts service.

```python
# add_two_ints_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request(5, 3)

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main():
    rclpy.init()
    client = AddTwoIntsClient()
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [add_two_ints_client]: Service not available, waiting...
[INFO] [add_two_ints_client]: Result: 8
```

**What's happening**:
- Line 8: `create_client(service_type, service_name)` creates a client for the AddTwoInts service
- Line 11: `wait_for_service()` blocks until a server is available, preventing errors from calling non-existent services
- Line 17: We create a request object and populate its fields (a and b)
- Line 20: `call_async()` sends the request without blocking the node, allowing other callbacks to run
- Line 21: We register a callback that runs when the response arrives

## Service Definition Files and Custom Service Types

While ROS 2 provides common service types in the `example_interfaces` package, you often need custom services for robot-specific operations. Service definitions live in `.srv` files within a package's `srv/` directory.

A service definition file has two sections. The request section lists the input fields with their types. Three dashes separate the request from the response section, which lists the output fields. Each field follows the format `type name`, similar to message definitions.

Here's an example service that checks if a robot can reach a target position:

```
# CheckReachability.srv
float64 target_x
float64 target_y
float64 target_z
---
bool reachable
string message
```

The request contains three coordinates. The response returns a boolean indicating reachability and a string explaining why or why not. This pattern of returning both a result and a human-readable message helps with debugging.

To use a custom service, you must declare it in your package's `CMakeLists.txt` (for C++) or `package.xml` (for Python), then build the package with `colcon build`. The build process generates Python or C++ classes from your `.srv` file.

**What we're building**: A service server that provides the AddTwoInts functionality.

```python
# add_two_ints_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.service = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.handle_request
        )
        self.get_logger().info('AddTwoInts service ready')

    def handle_request(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main():
    rclpy.init()
    server = AddTwoIntsServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [add_two_ints_server]: AddTwoInts service ready
[INFO] [add_two_ints_server]: Request: 5 + 3 = 8
```

**What's happening**:
- Line 8: `create_service(service_type, service_name, callback)` registers a service with the ROS 2 system
- Line 16: The callback receives a request object and a response object
- Line 17: We populate the response fields based on the request
- Line 21: Returning the response sends it back to the client

## ROS 2 Actions for Long-Running Tasks

**Actions** solve a problem that services cannot: providing feedback during long-running operations. When a robot navigates to a goal, the journey might take 30 seconds or more. Services would leave the client waiting in silence. Actions send periodic feedback updates, allowing the client to display progress or make decisions based on intermediate results.

An action has three components. The **goal** describes what to accomplish, similar to a service request. The **feedback** provides periodic updates during execution. The **result** contains the final outcome when the action completes. Actions also support preemption, allowing clients to cancel goals that are no longer needed.

Actions use a state machine with multiple states: idle, executing, succeeded, aborted, and canceled. The server transitions between states as it processes the goal. Clients can query the current state and receive notifications when states change.

Common action use cases include navigation (move to a position), manipulation (pick up an object), and any task that takes more than a few seconds. If you find yourself writing a service that takes more than 5 seconds, convert it to an action.

**What we're building**: An action client that sends a goal and receives feedback updates.

```python
# fibonacci_action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: order={order}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main():
    rclpy.init()
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [fibonacci_action_client]: Sending goal: order=10
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2, 3]
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

**What's happening**:
- Line 10: `ActionClient(node, action_type, action_name)` creates an action client
- Line 23: `send_goal_async()` sends the goal and registers a feedback callback
- Line 29: The feedback callback runs every time the server publishes feedback
- Line 33: The goal response callback runs once when the server accepts or rejects the goal
- Line 39: We request the final result, which arrives when the action completes

## Action Definition Files and Feedback Mechanisms

Action definitions use `.action` files with three sections separated by three dashes. The first section defines the goal fields. The second section defines the result fields. The third section defines the feedback fields that the server sends periodically during execution.

Here's an example action for navigating to a target position:

```
# NavigateToPosition.action
# Goal
float64 target_x
float64 target_y
float64 target_theta
---
# Result
bool success
float64 final_x
float64 final_y
string message
---
# Feedback
float64 current_x
float64 current_y
float64 distance_remaining
```

The goal specifies where to go. The result indicates success and the final position reached. The feedback provides real-time updates on current position and remaining distance. This allows clients to display progress bars or make decisions based on the robot's location.

Action servers should publish feedback at regular intervals, typically 1-10 Hz depending on the task. Too frequent updates waste bandwidth. Too infrequent updates leave clients without useful information. For navigation, 2-5 Hz provides a good balance.

Actions also support cancellation. Clients can call `cancel_goal()` to stop an action mid-execution. The server should check for cancellation requests regularly and clean up resources when canceled. This prevents robots from continuing obsolete tasks when plans change.

## Choosing Between Topics, Services, and Actions

The choice between topics, services, and actions depends on your communication needs. Use **topics** when you need continuous data streams with no response required. Sensor readings, video feeds, and status updates fit this pattern. Topics support one-to-many communication where multiple subscribers receive the same data.

Use **services** when you need a quick request-response interaction. Getting robot state, setting parameters, and triggering one-time actions work well as services. Services should complete within a few seconds. They support one-to-one communication between a single client and server.

Use **actions** when you need feedback during long-running tasks. Navigation, manipulation, and any operation taking more than 5 seconds benefits from actions. Actions provide progress updates and support cancellation. They also use one-to-one communication but with richer interaction patterns.

Here's a decision tree: Does the operation take more than 5 seconds? Use an action. Does it need a response? Use a service. Otherwise, use a topic. This simple rule covers most cases. Edge cases exist, but starting with this guideline helps you choose correctly.

| Pattern | Timing | Feedback | Cancellation | Example Use Case |
|---------|--------|----------|--------------|------------------|
| **Topic** | Continuous | None | N/A | Camera images, sensor data |
| **Service** | Synchronous | Single response | No | Get battery level, set parameter |
| **Action** | Asynchronous | Multiple updates | Yes | Navigate to goal, pick up object |

## Key Takeaways

- Services provide synchronous request-response communication where clients wait for a single response from servers.
- Actions extend services with feedback for long-running tasks, allowing clients to receive progress updates and cancel operations.
- Service definitions use `.srv` files with request and response sections separated by three dashes.
- Action definitions use `.action` files with goal, result, and feedback sections separated by three dashes.
- Choose topics for continuous data, services for quick requests, and actions for long-running tasks with feedback.

## Check Your Understanding

1. What is the key difference between a service and an action in ROS 2?
2. A robot needs to report its temperature every second. Should you use a topic, service, or action? Why?
3. You're building a system where a client asks "Is position (x, y, z) reachable?" and needs an immediate yes/no answer. Which communication pattern fits best?
4. An action definition file has three sections. What does each section contain?
5. Your navigation action takes 30 seconds to complete. How often should you publish feedback updates, and why?

## Next Steps

You now understand all three ROS 2 communication patterns: topics for streaming data, services for quick requests, and actions for long-running tasks. The next chapter covers building complete ROS 2 systems. You will learn to create packages, integrate AI agents with rclpy, write launch files, and describe robot structure with URDF.

---
**Hardware Tier 1 Note**: Use [The Construct](https://www.theconstructsim.com/) ROS 2 online environment to create and test service servers, clients, and action implementations directly in your browser without local installation.
