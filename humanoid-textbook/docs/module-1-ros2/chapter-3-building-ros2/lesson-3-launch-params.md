---
id: lesson-3-launch-params
title: "Lesson 3.3: Launch Files and Parameters"
sidebar_position: 3
sidebar_label: "3.3 Launch & Parameters"
description: "Master ROS 2 launch files to start multiple nodes with configuration, parameters, and namespaces"
duration_minutes: 60
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Study launch file examples and understand the patterns for when you have local ROS 2 access"
learning_objectives:
  - "Create Python launch files using LaunchDescription and Node actions"
  - "Configure node parameters through launch files and YAML files"
  - "Apply namespaces to run multiple instances of the same node"
  - "Use launch arguments for flexible system configuration"
  - "Organize complex multi-node systems with launch file composition"
keywords:
  - "ROS 2 launch files"
  - "LaunchDescription"
  - "node parameters"
  - "namespaces"
  - "launch arguments"
  - "YAML configuration"
prerequisites:
  - "Lesson 3.1: Building ROS 2 Packages with Python"
  - "Lesson 3.2: Bridging Python Agents with rclpy"
  - "Python functions and dictionaries"
chapter: "Chapter 3: Building with ROS 2"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 3.3: Launch Files and Parameters

**Duration**: 60 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Create Python launch files using LaunchDescription and Node actions
- Configure node parameters through launch files and YAML files
- Apply namespaces to run multiple instances of the same node
- Use launch arguments for flexible system configuration
- Organize complex multi-node systems with launch file composition

## Why Launch Files Are Essential

Imagine starting a humanoid robot system. You need to launch 15 nodes: camera drivers, LIDAR processors, joint controllers, navigation planners, and safety monitors. Starting each node manually with `ros2 run` is tedious and error-prone. Launch files solve this problem by starting all nodes together with one command, complete with their configuration and dependencies.

## Understanding Launch Files in ROS 2

A **launch file** is a Python script that describes which nodes to start, how to configure them, and in what order. ROS 2 uses Python for launch files instead of XML, giving you the full power of a programming language to create dynamic, conditional launches.

Launch files answer three questions: what nodes to run, how to configure them, and how they connect. The "what" is the package and executable name. The "how" includes parameters, remappings, and namespaces. The "connect" part handles topic remapping and node dependencies. This declarative approach makes complex systems manageable.

The core of every launch file is the `generate_launch_description()` function. This function returns a `LaunchDescription` object containing a list of actions. The most common action is `Node`, which starts a ROS 2 node. Other actions include loading parameters, setting environment variables, and including other launch files.

Launch files live in the `launch/` directory of your package. You register them in `setup.py` so ROS 2 can find them. Once installed, you run a launch file with `ros2 launch package_name launch_file.py`. This command processes the launch file and starts all specified nodes.

## Creating Your First Launch File

Let's create a simple launch file that starts two nodes: a sensor simulator and a data processor. This demonstrates the basic structure and syntax.

**What we're building**: A launch file that starts a temperature sensor node and a monitoring node together.

```python
# robot_system_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_monitor',
            executable='temperature_sensor',
            name='temp_sensor',
            output='screen'
        ),
        Node(
            package='robot_monitor',
            executable='health_monitor',
            name='health_monitor',
            output='screen'
        ),
    ])
```

**Expected output**:
```bash
$ ros2 launch robot_monitor robot_system_launch.py
[INFO] [temp_sensor]: Temperature sensor started
[INFO] [health_monitor]: Health monitor started
```

**What's happening**:
- Line 2-3: Import the launch system classes
- Line 5: The required function that returns launch configuration
- Lines 7-11: First Node action specifies package, executable, and node name
- Line 11: `output='screen'` prints node logs to the terminal
- Lines 12-17: Second Node action starts another node
- Both nodes start simultaneously when you run the launch file

The `name` parameter sets the node's name in the ROS 2 graph. This overrides the name specified in the node's code. The `output='screen'` parameter is crucial for debugging, as it shows all log messages in your terminal.

## Configuring Nodes with Parameters

**Parameters** are configuration values that nodes read at startup or runtime. Launch files can set parameters, making it easy to configure nodes without modifying code. Parameters support integers, floats, strings, booleans, and lists.

You pass parameters as a list of dictionaries to the `parameters` argument. Each dictionary contains parameter names and values. Nodes access these parameters using `self.declare_parameter()` and `self.get_parameter()`.

**What we're building**: A launch file that configures a sensor node with custom parameters.

```python
# sensor_config_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            parameters=[{
                'update_rate': 10.0,
                'low_voltage_threshold': 11.0,
                'critical_voltage_threshold': 10.0,
                'enable_warnings': True
            }]
        ),
    ])
```

**Expected output**:
```bash
$ ros2 launch robot_monitor sensor_config_launch.py
[INFO] [battery_monitor]: Update rate: 10.0 Hz
[INFO] [battery_monitor]: Low voltage threshold: 11.0V
```

**What's happening**:
- Lines 12-16: Dictionary of parameters passed to the node
- Line 13: `update_rate` parameter sets how often the node publishes
- Lines 14-15: Threshold parameters configure warning levels
- Line 16: Boolean parameter enables or disables features
- The node reads these parameters during initialization

To use these parameters in your node, add this code to `__init__()`:

```python
self.declare_parameter('update_rate', 1.0)  # Default value
self.declare_parameter('low_voltage_threshold', 11.0)

update_rate = self.get_parameter('update_rate').value
threshold = self.get_parameter('low_voltage_threshold').value
```

## Loading Parameters from YAML Files

For complex configurations, storing parameters in YAML files is cleaner than embedding them in launch files. YAML files organize parameters hierarchically and are easy to edit without touching code.

**What we're building**: A YAML configuration file and a launch file that loads it.

First, create the YAML file in `config/robot_params.yaml`:

```yaml
# config/robot_params.yaml
battery_monitor:
  ros__parameters:
    update_rate: 10.0
    low_voltage_threshold: 11.0
    critical_voltage_threshold: 10.0
    enable_warnings: true
    battery_capacity: 5000.0  # mAh

temperature_monitor:
  ros__parameters:
    update_rate: 5.0
    max_temperature: 80.0
    warning_temperature: 70.0
```

Now create a launch file that loads this configuration:

```python
# system_with_config_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to config file
    config_file = os.path.join(
        get_package_share_directory('robot_monitor'),
        'config',
        'robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[config_file]
        ),
        Node(
            package='robot_monitor',
            executable='temperature_monitor',
            name='temperature_monitor',
            parameters=[config_file]
        ),
    ])
```

**Expected output**:
```bash
$ ros2 launch robot_monitor system_with_config_launch.py
[INFO] [battery_monitor]: Loaded 5 parameters from config file
[INFO] [temperature_monitor]: Loaded 3 parameters from config file
```

**What's happening**:
- Lines 9-13: Construct the full path to the YAML file using package share directory
- Line 20: Pass the config file path instead of a dictionary
- Line 26: Multiple nodes can share the same config file
- Each node reads only its own section from the YAML file (matching node name)

The `ros__parameters` key in YAML is required. Parameters under this key become available to the node. The top-level key (like `battery_monitor`) must match the node name.

## Using Namespaces for Multiple Robot Instances

**Namespaces** allow you to run multiple instances of the same node without topic name conflicts. Each node's topics get prefixed with its namespace, creating isolated communication channels.

This is essential for multi-robot systems. If you have two robots, you can run the same nodes for each but in different namespaces like `/robot1` and `/robot2`. Topics become `/robot1/cmd_vel` and `/robot2/cmd_vel`, preventing crosstalk.

**What we're building**: A launch file that runs two instances of the same node in different namespaces.

```python
# multi_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 nodes
        Node(
            package='robot_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            namespace='robot1',
            parameters=[{'robot_id': 1}]
        ),
        Node(
            package='robot_controller',
            executable='velocity_controller',
            name='velocity_controller',
            namespace='robot1'
        ),

        # Robot 2 nodes
        Node(
            package='robot_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            namespace='robot2',
            parameters=[{'robot_id': 2}]
        ),
        Node(
            package='robot_controller',
            executable='velocity_controller',
            name='velocity_controller',
            namespace='robot2'
        ),
    ])
```

**Expected output**:
```bash
$ ros2 launch robot_monitor multi_robot_launch.py
[INFO] [robot1.battery_monitor]: Monitoring robot 1
[INFO] [robot2.battery_monitor]: Monitoring robot 2
$ ros2 topic list
/robot1/battery/voltage
/robot1/cmd_vel
/robot2/battery/voltage
/robot2/cmd_vel
```

**What's happening**:
- Line 12: `namespace='robot1'` prefixes all topics with `/robot1`
- Lines 8-20: First set of nodes for robot 1
- Lines 23-35: Identical nodes for robot 2 with different namespace
- Each robot has isolated topics, preventing interference
- You can run the same code for multiple robots simultaneously

Namespaces also work with parameters. A parameter file can have sections for each namespace, allowing per-robot configuration.

## Launch Arguments for Flexible Configuration

**Launch arguments** make launch files reusable by accepting input values at runtime. You can change behavior without editing the launch file, similar to command-line arguments in programs.

Common use cases include enabling debug mode, selecting robot models, or specifying configuration files. Arguments have default values but can be overridden when launching.

**What we're building**: A launch file with arguments for robot name and simulation mode.

```python
# flexible_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation time'
    )

    # Use arguments in node configuration
    robot_name = LaunchConfiguration('robot_name')
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,

        Node(
            package='robot_controller',
            executable='main_controller',
            name='controller',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': use_sim
            }]
        ),
    ])
```

**Expected output**:
```bash
$ ros2 launch robot_controller flexible_robot_launch.py
[INFO] [robot1.controller]: Controller started for robot1

$ ros2 launch robot_controller flexible_robot_launch.py robot_name:=robot2 use_sim:=true
[INFO] [robot2.controller]: Controller started for robot2 (simulation mode)
```

**What's happening**:
- Lines 9-13: Declare an argument with name, default value, and description
- Lines 22-23: Create LaunchConfiguration objects to reference argument values
- Line 33: Use the argument value as the namespace
- Line 35: Pass argument to node parameter
- Command line: `argument_name:=value` syntax overrides defaults

Launch arguments make your launch files adaptable to different scenarios without code changes.

## Key Takeaways

- Launch files use Python to declaratively specify which nodes to start, how to configure them, and how they connect.
- The LaunchDescription object contains Node actions that define package, executable, name, and configuration for each node.
- Parameters configure nodes at startup through dictionaries in launch files or YAML configuration files.
- Namespaces enable running multiple instances of the same node by prefixing topic names, essential for multi-robot systems.
- Launch arguments provide runtime flexibility, allowing users to customize behavior without editing launch files.

## Check Your Understanding

1. What is the purpose of the `generate_launch_description()` function, and what must it return?

2. You have a node that needs three parameters: `max_speed`, `robot_id`, and `enable_safety`. Write the Node action with these parameters configured.

3. Explain the difference between setting parameters directly in a launch file versus loading them from a YAML file. When would you use each approach?

4. You want to run three instances of a camera node, each publishing to different topics. How would you use namespaces to accomplish this?

5. Your launch file needs to work in both simulation and real robot modes. What launch argument would you add, and how would you use it to change node behavior?

## Next Steps

Now that you can orchestrate multi-node systems with launch files, the next lesson covers URDF for describing humanoid robot structure. You will learn to define links, joints, and kinematics for robot visualization and simulation.

---
**Hardware Tier 1 Note**: Study the launch file patterns and syntax in this lesson. While you cannot run `ros2 launch` commands in a browser environment, understanding launch file structure prepares you for when you have local ROS 2 access. Many cloud platforms provide example launch files you can examine to reinforce these concepts.
