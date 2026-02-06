---
id: chapter-2-ros2-architecture
title: "Chapter 2: ROS 2 Architecture"
sidebar_position: 2
sidebar_label: "Ch 2: ROS 2 Architecture"
description: "Master the core concepts of ROS 2 - nodes, topics, services, and actions"
---

# Chapter 2: ROS 2 Architecture

**Duration**: Week 3
**Hardware Tier**: Tier 1-2
**Lessons**: 4

## Chapter Overview

ROS 2 is built on a simple but powerful idea: break complex robot systems into small, independent pieces (nodes) that communicate through well-defined channels (topics, services, actions).

In this chapter, you'll learn the architectural patterns that make ROS 2 the backbone of modern robotics - from warehouse robots to Mars rovers.

## Learning Objectives

By the end of this chapter, you will be able to:

- **Explain** the ROS 2 distributed architecture and DDS middleware
- **Create** and manage ROS 2 nodes as independent executables
- **Implement** publisher/subscriber patterns using topics
- **Build** request/response interactions using services
- **Design** long-running tasks using actions with feedback

## Prerequisites

- Chapter 1: Introduction to Physical AI
- Python basics (functions, classes)
- Terminal/command line familiarity

## Lessons

| # | Lesson | Duration | Tier | Description |
|---|--------|----------|------|-------------|
| 2.1 | [ROS 2 Core Concepts](./lesson-1-core-concepts.md) | 45 min | 1-2 | Architecture overview, DDS, and the computation graph |
| 2.2 | [Nodes - The Building Blocks](./lesson-2-nodes.md) | 60 min | 2 | Creating, running, and managing ROS 2 nodes |
| 2.3 | [Topics and Message Passing](./lesson-3-topics-messages.md) | 60 min | 2 | Publisher/subscriber pattern and message types |
| 2.4 | [Services and Actions](./lesson-4-services-actions.md) | 60 min | 2 | Request/response and long-running operations |

## Key Concepts

### The Computation Graph
```
┌─────────┐     /camera/image     ┌─────────────┐
│ Camera  │ ──────────────────────▶│   Vision    │
│  Node   │                       │  Processor  │
└─────────┘                       └──────┬──────┘
                                         │ /detected_objects
                                         ▼
┌─────────┐     /cmd_vel          ┌─────────────┐
│  Motor  │ ◀─────────────────────│  Navigation │
│ Driver  │                       │    Node     │
└─────────┘                       └─────────────┘
```

### Communication Patterns

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topic** | Continuous data streams | Sensor readings, video feeds |
| **Service** | Quick request/response | Get robot state, set parameter |
| **Action** | Long-running with feedback | Navigate to goal, pick up object |

## Hardware Note

:::tip Tier 1 Alternative
For Tier 1 (cloud) users: Use [The Construct](https://www.theconstructsim.com/) or Foxglove Studio for browser-based ROS 2 development.
:::

:::info Tier 2 Setup
For local development: Ubuntu 22.04 with ROS 2 Humble. Installation guide in Lesson 2.1.
:::

## Chapter Assessment

After completing this chapter:
- [ ] Can draw a ROS 2 computation graph for a simple robot
- [ ] Created a publisher and subscriber node
- [ ] Implemented a service server and client
- [ ] Understand when to use topics vs services vs actions
