---
id: module-1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 2
sidebar_label: "Module 1: ROS 2"
description: "Master ROS 2 middleware for robot control - the foundation of modern robotics"
---

# Module 1: The Robotic Nervous System (ROS 2)

**Focus**: Middleware for robot control
**Duration**: Weeks 1-5 (5 weeks)
**Hardware Tier**: Tier 1-2 (Cloud/RTX GPU)

## Module Overview

The Robot Operating System 2 (ROS 2) is the nervous system of modern robots. Just as your nervous system coordinates signals between your brain and body, ROS 2 coordinates communication between different parts of a robot - sensors, processors, and actuators.

In this module, you'll build a solid foundation in Physical AI concepts and master ROS 2, the industry-standard middleware used by companies like Boston Dynamics, NVIDIA, and NASA.

## Learning Objectives

By the end of this module, you will be able to:

- **Define** Physical AI and distinguish it from digital-only AI systems
- **Explain** the ROS 2 architecture and its distributed communication model
- **Create** ROS 2 nodes that publish and subscribe to topics
- **Implement** services and actions for request-response patterns
- **Build** complete ROS 2 packages using Python and rclpy
- **Design** URDF models for humanoid robot descriptions

## Prerequisites

- Basic Python programming (functions, classes, loops)
- Linux command line familiarity (cd, ls, mkdir)
- Understanding of basic networking concepts (helpful but not required)

## Hardware Requirements

| Tier | Equipment | What You Can Do |
|------|-----------|-----------------|
| **Tier 1** | Laptop + Cloud | Browser-based ROS 2 via Foxglove, conceptual lessons |
| **Tier 2** | RTX GPU + Ubuntu 22.04 | Full local ROS 2 Humble installation, Gazebo simulation |

:::tip Tier 1 Path
Don't have a powerful computer? No problem! All lessons include cloud-based alternatives using Foxglove Studio or The Construct.
:::

## Chapters

### Chapter 1: Introduction to Physical AI
*Weeks 1-2 • 4 Lessons • ~3 hours*

Foundations of embodied intelligence and the humanoid robotics landscape. Understand why robots need to understand physics, not just language.

[Start Chapter 1 →](/docs/module-1-ros2/chapter-1-physical-ai)

### Chapter 2: ROS 2 Architecture
*Week 3 • 4 Lessons • ~4 hours*

Core concepts including nodes, topics, services, and actions. Learn the building blocks that make robots tick.

[Start Chapter 2 →](/docs/module-1-ros2/chapter-2-ros2-architecture)

### Chapter 3: Building with ROS 2
*Weeks 4-5 • 4 Lessons • ~5 hours*

Hands-on package development with Python. Build real ROS 2 applications and understand URDF for describing humanoid robots.

[Start Chapter 3 →](/docs/module-1-ros2/chapter-3-building-ros2)

## What You'll Build

By the end of this module, you will have created:

1. **Talker/Listener Nodes** - Your first ROS 2 communication
2. **Sensor Monitor** - A node that reads and processes sensor data
3. **Service Server** - A node that responds to requests
4. **Complete ROS 2 Package** - A reusable package with launch files
5. **Humanoid URDF** - A robot description file for a simple humanoid

## Assessment

- **Project**: Build a ROS 2 package that monitors simulated robot sensors and publishes health status
- **Quiz**: 20 questions covering ROS 2 concepts and architecture
