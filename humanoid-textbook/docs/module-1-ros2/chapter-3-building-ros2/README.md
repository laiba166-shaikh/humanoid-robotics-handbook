---
id: chapter-3-building-ros2
title: "Chapter 3: Building with ROS 2"
sidebar_position: 3
sidebar_label: "Ch 3: Building ROS 2"
description: "Create complete ROS 2 packages with Python, launch files, and URDF robot descriptions"
---

# Chapter 3: Building with ROS 2

**Duration**: Weeks 4-5
**Hardware Tier**: Tier 2
**Lessons**: 4

## Chapter Overview

You've learned the concepts - now it's time to build. In this chapter, you'll create complete ROS 2 packages from scratch, learn to bridge Python AI agents with robot controllers, and describe humanoid robots using URDF.

By the end, you'll have a fully functional ROS 2 package ready for simulation.

## Learning Objectives

By the end of this chapter, you will be able to:

- **Create** ROS 2 packages with proper structure and dependencies
- **Use** rclpy to bridge Python code with ROS 2 controllers
- **Write** launch files to start multiple nodes with configuration
- **Design** URDF models that describe humanoid robot kinematics
- **Build** and test complete ROS 2 applications

## Prerequisites

- Chapter 2: ROS 2 Architecture (completed)
- Python intermediate (classes, decorators, async basics)
- ROS 2 Humble installed (Tier 2) or cloud environment (Tier 1)

## Lessons

| # | Lesson | Duration | Tier | Description |
|---|--------|----------|------|-------------|
| 3.1 | [Building ROS 2 Packages](./lesson-1-packages-python.md) | 75 min | 2 | Package structure, setup.py, dependencies |
| 3.2 | [Bridging Python Agents with rclpy](./lesson-2-rclpy-agents.md) | 75 min | 2 | Connecting AI/ML code to ROS 2 |
| 3.3 | [Launch Files and Parameters](./lesson-3-launch-params.md) | 60 min | 2 | Multi-node launch, configuration, namespaces |
| 3.4 | [Understanding URDF for Humanoids](./lesson-4-urdf-humanoids.md) | 90 min | 2 | Robot description format, joints, links |

## Key Concepts

### ROS 2 Package Structure
```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── talker_node.py
│   └── listener_node.py
├── launch/
│   └── robot_launch.py
├── config/
│   └── params.yaml
├── urdf/
│   └── robot.urdf
├── package.xml
├── setup.py
└── setup.cfg
```

### URDF Basics
```xml
<robot name="simple_humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

## Hardware Note

:::caution Tier 2 Required
This chapter requires a local ROS 2 installation for hands-on package development. Cloud environments have limitations for package building.
:::

:::tip Tier 1 Workaround
Tier 1 users can follow along conceptually and use pre-built packages in simulation. Full hands-on practice recommended when you upgrade to Tier 2.
:::

## What You'll Build

### Module 1 Capstone: Robot Health Monitor

A complete ROS 2 package that:
1. Subscribes to simulated sensor topics (battery, temperature, joint states)
2. Processes data and detects anomalies
3. Publishes health status on a custom topic
4. Provides a service to query detailed diagnostics
5. Includes launch file for easy startup

## Chapter Assessment

After completing this chapter:
- [ ] Created a ROS 2 package from scratch
- [ ] Successfully built and ran your package
- [ ] Wrote a launch file that starts multiple nodes
- [ ] Created a basic URDF for a humanoid robot
- [ ] Completed the Robot Health Monitor capstone project
