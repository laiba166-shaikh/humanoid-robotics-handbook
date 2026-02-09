---
id: lesson-1-packages-python
title: "Lesson 3.1: Building ROS 2 Packages with Python"
sidebar_position: 1
sidebar_label: "3.1 ROS 2 Packages"
description: "Learn to create, structure, and build ROS 2 packages with Python using setup.py, package.xml, and colcon"
duration_minutes: 75
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Follow along conceptually and examine pre-built package examples in The Construct"
learning_objectives:
  - "Create a ROS 2 Python package with proper directory structure"
  - "Configure setup.py for package installation and dependencies"
  - "Write package.xml with correct metadata and dependency declarations"
  - "Build and install packages using colcon build workflow"
  - "Organize nodes, launch files, and configuration in a package"
keywords:
  - "ROS 2 packages"
  - "setup.py"
  - "package.xml"
  - "colcon build"
  - "ament_python"
  - "package structure"
prerequisites:
  - "Chapter 2: ROS 2 Architecture (completed)"
  - "Python intermediate (modules, imports, setuptools basics)"
  - "Terminal commands and file system navigation"
chapter: "Chapter 3: Building with ROS 2"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 3.1: Building ROS 2 Packages with Python

**Duration**: 75 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Create a ROS 2 Python package with proper directory structure
- Configure setup.py for package installation and dependencies
- Write package.xml with correct metadata and dependency declarations
- Build and install packages using colcon build workflow
- Organize nodes, launch files, and configuration in a package

## Why Package Structure Matters

You have written individual ROS 2 nodes, but how do you share them with teammates or deploy them to a robot? ROS 2 packages provide the answer. A **package** is a self-contained unit that bundles your code, dependencies, and configuration into a distributable format. Proper package structure makes your code reusable, testable, and ready for production robotics systems.

## Understanding ROS 2 Package Organization

A **ROS 2 package** is a directory containing source code, metadata files, and build instructions. Packages are the fundamental unit of organization in ROS 2. Every node you create belongs to a package, and packages declare their dependencies on other packages.

Think of a package like a Python library you install with pip. The package includes the code, specifies what other libraries it needs, and provides installation instructions. ROS 2 packages work the same way but are designed specifically for robotics applications. They can contain nodes, message definitions, launch files, and configuration files all in one organized structure.

ROS 2 supports two package types: **ament_cmake** for C++ packages and **ament_python** for Python packages. This lesson focuses on Python packages, which are simpler to create and perfect for integrating AI and machine learning code. Python packages use familiar tools like setuptools, making them accessible to developers coming from the Python ecosystem.

The key advantage of packages is dependency management. When you declare that your package depends on `geometry_msgs`, ROS 2 ensures that package is available before building yours. This prevents runtime errors and makes your system more reliable. Packages also enable workspace overlays, where you can develop custom packages that extend or override system packages.

## Anatomy of a Python ROS 2 Package

A Python ROS 2 package follows a specific directory structure. Understanding this structure is essential for creating packages that build correctly.

The top-level directory contains your package name. Inside, you have a Python module directory with the same name, plus several configuration files. Here is the standard structure:

```
my_robot_package/
├── my_robot_package/          # Python module (same name as package)
│   ├── __init__.py            # Makes this a Python module
│   ├── talker_node.py         # Your node implementations
│   └── listener_node.py
├── launch/                    # Launch files (optional)
│   └── robot_launch.py
├── config/                    # Configuration files (optional)
│   └── params.yaml
├── test/                      # Unit tests (optional)
│   └── test_talker.py
├── resource/                  # Package marker (required)
│   └── my_robot_package
├── package.xml                # Package metadata (required)
├── setup.py                   # Installation script (required)
└── setup.cfg                  # Setup configuration (required)
```

The **package.xml** file declares metadata like the package name, version, maintainer, license, and dependencies. ROS 2 reads this file to understand what your package needs. The **setup.py** file tells Python how to install your package and where to find executables. The **setup.cfg** file configures how setuptools processes your package.

The **resource** directory contains a marker file with your package name. This file is empty but must exist for ROS 2 to recognize your package. The Python module directory contains your actual code. Each node is typically a separate Python file within this module.

Optional directories include **launch** for launch files, **config** for parameter files, and **test** for unit tests. Organizing these files properly makes your package maintainable as it grows. A well-structured package is easy to navigate and follows conventions that other ROS 2 developers expect.

## Creating a Package with ros2 pkg create

ROS 2 provides a command-line tool to generate package scaffolding automatically. This tool creates all required files with correct structure, saving you from manual setup.

**What we're building**: A new ROS 2 Python package called `robot_monitor` that will contain health monitoring nodes.

```bash
# Navigate to your workspace src directory
cd ~/ros2_ws/src

# Create a Python package with dependencies
ros2 pkg create robot_monitor \
  --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs

# Verify the package was created
ls robot_monitor/
```

**Expected output**:
```
config  launch  package.xml  resource  robot_monitor  setup.cfg  setup.py  test
```

**What's happening**:
- `ros2 pkg create` generates a complete package structure
- `--build-type ament_python` specifies a Python package (not C++)
- `--dependencies` automatically adds rclpy, std_msgs, and sensor_msgs to package.xml
- The command creates all required files and directories

The generated package is ready to build immediately, even though it contains no nodes yet. This scaffolding approach ensures you start with correct structure and avoid common mistakes.

## Configuring package.xml for Dependencies

The **package.xml** file is the heart of your package metadata. It declares what your package is, who maintains it, and what it depends on.

**What we're building**: A complete package.xml file with proper metadata and dependency declarations.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_monitor</name>
  <version>0.1.0</version>
  <description>Health monitoring system for robots</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Expected output**:
```
(No output - this is a configuration file)
```

**What's happening**:
- Line 4: Package name must match the directory name
- Line 5: Version follows semantic versioning (major.minor.patch)
- Line 11: `buildtool_depend` specifies ament_python as the build system
- Lines 14-17: `depend` tags declare runtime dependencies on other packages
- Lines 20-23: `test_depend` tags specify packages needed for testing
- Line 26: Export section declares this is an ament_python package

Dependencies are critical. If you use `geometry_msgs/Twist` in your code but forget to declare the dependency, your package will build but fail at runtime. Always add dependencies for every message type, service type, or package you import.

## Writing setup.py for Node Installation

The **setup.py** file tells Python how to install your package and register your nodes as executable commands. This file uses standard Python setuptools with ROS 2-specific extensions.

**What we're building**: A setup.py that installs our package and registers node entry points.

```python
# setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'robot_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Health monitoring system for robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = robot_monitor.battery_monitor:main',
            'temperature_monitor = robot_monitor.temperature_monitor:main',
        ],
    },
)
```

**Expected output**:
```
(No output - this is a configuration file)
```

**What's happening**:
- Line 6: Package name must match package.xml and directory name
- Lines 12-19: `data_files` specifies non-Python files to install (launch files, configs)
- Lines 28-31: `entry_points` registers executable commands that run your nodes
- Line 29: Creates a `battery_monitor` command that calls the `main()` function in `battery_monitor.py`

The entry points section is crucial. Each entry creates a command you can run with `ros2 run robot_monitor battery_monitor`. The format is `command_name = package.module:function`. This mechanism allows ROS 2 to discover and execute your nodes.

## Building and Installing with colcon

**colcon** is the build tool for ROS 2. It compiles packages, resolves dependencies, and installs them into your workspace. Understanding the colcon workflow is essential for package development.

The build process has three steps: configure, build, and install. For Python packages, colcon primarily handles installation since Python does not require compilation. However, colcon still validates your package structure and sets up the environment correctly.

**What we're building**: A complete build and installation of our robot_monitor package.

```bash
# Navigate to workspace root (not src/)
cd ~/ros2_ws

# Build all packages in the workspace
colcon build

# Build only the robot_monitor package
colcon build --packages-select robot_monitor

# Build with verbose output for debugging
colcon build --packages-select robot_monitor --event-handlers console_direct+

# Source the workspace to use the package
source install/setup.bash

# Verify the package is available
ros2 pkg list | grep robot_monitor
```

**Expected output**:
```
Starting >>> robot_monitor
Finished <<< robot_monitor [0.45s]

Summary: 1 package finished [0.52s]

robot_monitor
```

**What's happening**:
- `colcon build` processes all packages in the src/ directory
- `--packages-select` builds only specified packages (faster for development)
- `--event-handlers console_direct+` shows detailed build output
- `source install/setup.bash` adds the package to your ROS 2 environment
- The package is now available for `ros2 run` and `ros2 launch` commands

After building, colcon creates three directories: **build** (temporary build files), **install** (installed packages), and **log** (build logs). The install directory contains your package in a form ROS 2 can use. You must source `install/setup.bash` in every terminal where you want to use your package.

## Adding a Node to Your Package

Now let's add an actual node to our package. This demonstrates the complete workflow from code to executable.

**What we're building**: A battery monitoring node that publishes battery status messages.

```python
# robot_monitor/battery_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher = self.create_publisher(Float32, '/battery/voltage', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        self.voltage = 12.6  # Simulated battery voltage
        self.get_logger().info('Battery monitor started')

    def publish_battery_status(self):
        msg = Float32()
        msg.data = self.voltage
        self.publisher.publish(msg)

        # Simulate battery drain
        self.voltage -= 0.01
        if self.voltage < 10.0:
            self.get_logger().warn(f'Low battery: {self.voltage:.2f}V')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected output**:
```
[INFO] [battery_monitor]: Battery monitor started
[WARN] [battery_monitor]: Low battery: 9.99V
[WARN] [battery_monitor]: Low battery: 9.98V
```

**What's happening**:
- Lines 6-12: Standard ROS 2 node initialization with publisher
- Lines 14-21: Timer callback publishes voltage and simulates battery drain
- Lines 23-28: Main function follows the standard rclpy pattern
- The node is now part of the robot_monitor package

After adding this file, rebuild the package with `colcon build --packages-select robot_monitor`, source the workspace, and run with `ros2 run robot_monitor battery_monitor`. The entry point in setup.py connects the command to this code.

## Key Takeaways

- ROS 2 packages are self-contained units that bundle code, dependencies, and configuration for distribution and deployment.
- Python packages use ament_python build type with setup.py, package.xml, and setup.cfg as required files.
- The package.xml file declares metadata and dependencies, ensuring all required packages are available at build and runtime.
- The setup.py file registers node entry points that create executable commands for running your nodes.
- colcon build compiles and installs packages, creating the install directory that must be sourced before using the package.

## Check Your Understanding

1. What are the three required files in every ROS 2 Python package, and what is the purpose of each?

2. You added `from geometry_msgs.msg import Twist` to your node code, but when you run the node, you get an import error. What did you likely forget to do?

3. Examine this entry point: `'my_node = my_package.node_file:start'`. What command would you use to run this node, and what function does it call?

4. After building your package with colcon, you open a new terminal and try to run your node, but ROS 2 says the package is not found. What step did you forget?

5. Your package depends on both `sensor_msgs` and `geometry_msgs`. Where must you declare these dependencies, and what happens if you forget one?

## Next Steps

Now that you can create and build ROS 2 packages, the next lesson covers bridging Python AI agents with ROS 2 using rclpy patterns. You will learn to integrate machine learning models and AI decision-making into your robot control systems.

---
**Hardware Tier 1 Note**: Use [The Construct](https://www.theconstructsim.com/) to examine pre-built ROS 2 packages and understand their structure. While you cannot run the full colcon build workflow in a browser, you can study package organization and prepare for hands-on practice when you upgrade to Tier 2.
