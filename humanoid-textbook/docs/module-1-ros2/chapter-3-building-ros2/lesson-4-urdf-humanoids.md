---
id: lesson-4-urdf-humanoids
title: "Lesson 3.4: Understanding URDF for Humanoids"
sidebar_position: 4
sidebar_label: "3.4 URDF for Humanoids"
description: "Learn to describe humanoid robot structure using URDF with links, joints, and kinematics for simulation and visualization"
duration_minutes: 90
proficiency_level: "B1"
layer: "L2"
hardware_tier: 2
tier_1_path: "Use online URDF visualizers and examine example humanoid URDFs from open-source projects"
learning_objectives:
  - "Define robot structure using URDF links and joints"
  - "Specify joint types and limits for humanoid articulation"
  - "Create visual and collision geometries for robot parts"
  - "Build a kinematic chain for a humanoid upper body"
  - "Visualize URDF models in RViz for validation"
keywords:
  - "URDF"
  - "robot description"
  - "links and joints"
  - "kinematics"
  - "humanoid structure"
  - "RViz visualization"
prerequisites:
  - "Chapter 3 lessons 3.1-3.3 completed"
  - "Basic XML syntax understanding"
  - "3D coordinate systems and transformations"
chapter: "Chapter 3: Building with ROS 2"
module: "Module 1: The Robotic Nervous System — ROS 2"
---

# Lesson 3.4: Understanding URDF for Humanoids

**Duration**: 90 minutes
**Hardware Tier**: Tier 2 (RTX GPU + Ubuntu)
**Layer**: L2 (AI Collaboration)

## Learning Objectives

By the end of this lesson, you will be able to:
- Define robot structure using URDF links and joints
- Specify joint types and limits for humanoid articulation
- Create visual and collision geometries for robot parts
- Build a kinematic chain for a humanoid upper body
- Visualize URDF models in RViz for validation

## Why URDF Matters for Humanoid Robotics

Before you can simulate a humanoid robot or plan its movements, you need to describe its physical structure. How long is the upper arm? What angle can the shoulder rotate? Where is the camera mounted relative to the head? The **Unified Robot Description Format (URDF)** answers these questions in a standardized XML format that ROS 2 tools understand. Every simulation, visualization, and motion planning system starts with a URDF file.

## What is URDF and Why XML

**URDF** is an XML-based format that describes a robot's kinematic and dynamic properties. It defines the robot as a tree of **links** (rigid bodies) connected by **joints** (movable connections). This hierarchical structure mirrors how real robots are built, with a base link and child links attached through joints.

Think of URDF like a blueprint for a robot. Just as architectural blueprints specify dimensions, materials, and connections for a building, URDF specifies sizes, shapes, and movement constraints for a robot. The XML format makes URDF both human-readable and machine-parseable. You can edit it in a text editor, and ROS 2 tools can load it automatically.

URDF serves three primary purposes. First, it enables **visualization** in tools like RViz, where you can see your robot model and verify its structure. Second, it powers **simulation** in Gazebo, where physics engines use URDF to compute realistic motion and collisions. Third, it supports **motion planning** with MoveIt, which uses URDF to calculate inverse kinematics and collision-free paths.

The format has limitations. URDF only supports tree structures, not closed kinematic chains like parallel mechanisms. It uses a simplified physics model that may not capture all real-world dynamics. For complex humanoids, you often supplement URDF with additional configuration files. Despite these limitations, URDF remains the standard for robot description in ROS 2.

## Anatomy of a URDF File: Links and Joints

A URDF file has two main elements: **links** and **joints**. Links represent rigid bodies like the torso, upper arm, or foot. Joints represent connections between links, defining how they move relative to each other.

Every URDF starts with a root link, typically called `base_link`. This is the reference point for the entire robot. All other links connect to the root through a chain of joints. For a humanoid, `base_link` might be the pelvis, with the torso, legs, and arms branching from there.

Links contain three types of information: **visual** (how it looks), **collision** (its shape for collision detection), and **inertial** (mass and inertia for physics simulation). Visual geometry can be simple shapes like boxes and cylinders, or complex meshes loaded from STL or DAE files. Collision geometry is usually simplified for computational efficiency.

Joints define the relationship between a parent link and a child link. The joint specifies the axis of rotation or translation, the limits of motion, and the effort (torque or force) the joint can exert. ROS 2 supports six joint types: fixed, revolute, continuous, prismatic, planar, and floating. Humanoids primarily use revolute joints for rotational movement.

## Creating Your First URDF: A Simple Robot Arm

Let's build a basic robot arm to understand URDF syntax. This arm has a base, an upper arm, and a forearm connected by revolute joints.

**What we're building**: A two-link robot arm with shoulder and elbow joints.

```xml
<?xml version="1.0"?>
<!-- simple_arm.urdf -->
<robot name="simple_arm">

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Shoulder joint (connects base to upper arm) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Elbow joint (connects upper arm to forearm) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="5.0" velocity="1.0"/>
  </joint>

</robot>
```

**Expected output**:
```bash
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_arm.urdf)"
$ rviz2
# In RViz, add RobotModel display and set Fixed Frame to "base_link"
# You will see a gray base with blue upper arm and green forearm
```

**What's happening**:
- Lines 5-19: `base_link` defined with box geometry for visual and collision
- Lines 11-13: Material defines color (RGBA: red, green, blue, alpha)
- Lines 22-37: `upper_arm` link with cylinder geometry offset by 0.2m in z-axis
- Lines 40-46: `shoulder_joint` connects base to upper arm as revolute joint
- Line 44: `axis xyz="0 1 0"` means rotation around y-axis
- Line 45: Joint limits in radians (approximately -90 to +90 degrees)
- Lines 60-66: `elbow_joint` connects upper arm to forearm
- The `origin` in joints specifies where the child link attaches to the parent

This URDF creates a kinematic chain: base_link → shoulder_joint → upper_arm → elbow_joint → forearm.

## Joint Types and Their Applications in Humanoids

Understanding joint types is crucial for modeling humanoid robots accurately. Each joint type represents a different kind of motion.

**Revolute joints** rotate around an axis with defined limits. Most humanoid joints are revolute: shoulders, elbows, hips, knees, and ankles. The `limit` element specifies the range of motion in radians. For example, a human knee bends from 0 to about 2.4 radians (140 degrees). Setting accurate limits prevents the simulation from creating impossible poses.

**Continuous joints** are revolute joints without limits. They can rotate infinitely, like a wheel. Humanoids rarely use continuous joints, but they appear in mobile bases or rotating sensors. The syntax is identical to revolute joints but without the `limit` element.

**Prismatic joints** slide along an axis, like a telescope extending. Humanoids might use prismatic joints for adjustable height mechanisms or gripper fingers. The `limit` element specifies the minimum and maximum extension distance in meters.

**Fixed joints** do not move. They rigidly attach one link to another. Use fixed joints for sensors mounted on the robot, like cameras on the head. Fixed joints simplify the kinematic tree by treating multiple rigid parts as a single unit.

**Planar joints** allow motion in a plane (two translations and one rotation). **Floating joints** allow unconstrained motion in 3D space (three translations and three rotations). These are rare in humanoid descriptions but useful for modeling the robot's base relative to the world.

Here is an example showing different joint types:

**What we're building**: A demonstration of various joint types in one URDF.

```xml
<!-- Joint type examples -->

<!-- Revolute: Shoulder with 180-degree range -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.2 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50.0" velocity="2.0"/>
</joint>

<!-- Continuous: Rotating camera mount -->
<joint name="camera_rotation" type="continuous">
  <parent link="head"/>
  <child link="camera_mount"/>
  <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic: Gripper finger -->
<joint name="gripper_finger" type="prismatic">
  <parent link="hand"/>
  <child link="finger"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.05" effort="10.0" velocity="0.1"/>
</joint>

<!-- Fixed: Camera rigidly mounted -->
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.08 0 0.05" rpy="0 0 0"/>
</joint>
```

**Expected output**:
```
(No direct output - these are URDF definitions)
```

**What's happening**:
- Lines 4-10: Revolute joint with effort (torque) and velocity limits
- Lines 13-17: Continuous joint has no position limits
- Lines 20-25: Prismatic joint limits linear motion to 5cm
- Lines 28-32: Fixed joint has no axis or limits, just a static transform

Choose joint types based on the actual mechanism. Accurate joint modeling ensures realistic simulation and correct motion planning.

## Building a Humanoid Upper Body

Now let's create a more realistic humanoid upper body with torso, shoulders, and arms. This demonstrates how to build a branching kinematic tree.

**What we're building**: A humanoid torso with two arms, each having shoulder and elbow joints.

```xml
<?xml version="1.0"?>
<!-- humanoid_upper_body.urdf -->
<robot name="humanoid_upper_body">

  <!-- Torso (root link) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Left shoulder link -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="2.0"/>
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <!-- Left elbow joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="20.0" velocity="2.0"/>
  </joint>

  <!-- Right arm (mirror of left) -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <sphere radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="0 -0.15 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30.0" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="20.0" velocity="2.0"/>
  </joint>

</robot>
```

**Expected output**:
```bash
$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat humanoid_upper_body.urdf)"
$ ros2 run joint_state_publisher_gui joint_state_publisher_gui
# GUI appears with sliders for each joint
# Move sliders to see the arms move in RViz
```

**What's happening**:
- Lines 6-15: Torso is the root link (base_link)
- Lines 18-36: Left shoulder and joint attached to torso at y=0.15 (right side)
- Lines 39-58: Left upper arm attached to shoulder with elbow joint
- Lines 61-99: Right arm mirrors the left arm structure at y=-0.15
- The kinematic tree branches: base_link splits into left and right arms
- Each arm is an independent chain from the torso

This structure demonstrates how humanoids have multiple kinematic chains branching from a central torso. Real humanoids add legs, head, and hands following the same pattern.

## Visualizing and Testing URDF in RViz

**RViz** is ROS 2's visualization tool. It displays your URDF model in 3D, allowing you to verify geometry, joint ranges, and coordinate frames. Testing in RViz catches errors before simulation.

To visualize a URDF, you need two nodes: `robot_state_publisher` (publishes the robot's transform tree) and `joint_state_publisher` (publishes joint positions). The GUI version of joint_state_publisher provides sliders to move joints interactively.

**What we're building**: A complete workflow to visualize and test a URDF model.

```bash
# Terminal 1: Start robot state publisher with your URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat humanoid_upper_body.urdf)"

# Terminal 2: Start joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Start RViz
rviz2

# In RViz:
# 1. Set Fixed Frame to "base_link"
# 2. Add → RobotModel
# 3. Add → TF to see coordinate frames
# 4. Use joint_state_publisher_gui sliders to move joints
```

**Expected output**:
```
RViz window shows:
- White torso box in center
- Red shoulder spheres on left and right
- Blue arm cylinders extending downward
- Coordinate frame axes at each link
- Sliders in GUI control arm positions
```

**What's happening**:
- `robot_state_publisher` reads the URDF and publishes TF transforms
- `joint_state_publisher_gui` publishes joint positions from slider values
- RViz subscribes to these topics and renders the 3D model
- Moving sliders updates joint positions in real-time
- TF display shows the coordinate frame at each link

Common issues to check in RViz: links appearing in wrong positions (check `origin` in joints), missing links (check parent-child relationships), and incorrect joint axes (verify `axis xyz` values). RViz makes these problems immediately visible.

## Best Practices for Humanoid URDF Design

Creating maintainable URDF files for complex humanoids requires following best practices. These guidelines prevent common errors and make your models easier to modify.

**Use consistent naming conventions**. Name links descriptively: `left_upper_arm`, `right_knee`, `head_link`. Name joints to indicate what they connect: `left_shoulder_pitch`, `right_hip_roll`. Consistent naming makes the URDF readable and helps when debugging.

**Define materials once and reuse them**. Instead of repeating color definitions, define materials at the top of the file and reference them by name. This makes color changes easier and keeps the file organized.

**Separate visual and collision geometry**. Visual geometry can be detailed meshes for realistic appearance. Collision geometry should be simplified shapes (boxes, cylinders, spheres) for fast collision detection. Using the same complex mesh for both slows simulation significantly.

**Set realistic joint limits**. Research actual humanoid joint ranges. A human shoulder has different limits than a robot shoulder. Incorrect limits cause motion planning failures or unrealistic poses. Document where you got the values.

**Include inertial properties for simulation**. Gazebo needs mass and inertia tensors to simulate physics correctly. Even rough estimates are better than omitting inertial properties. Use online calculators for common shapes.

**Validate with check_urdf tool**. ROS 2 provides `check_urdf` to verify your file is valid XML and follows URDF rules. Run this before testing in RViz or Gazebo.

```bash
# Validate URDF syntax and structure
check_urdf humanoid_upper_body.urdf

# View the kinematic tree
urdf_to_graphiz humanoid_upper_body.urdf
# Opens a PDF showing the link-joint tree
```

These practices scale from simple arms to full humanoid robots with 30+ joints.

## Key Takeaways

- URDF is an XML format that describes robot structure as a tree of links connected by joints, used for visualization, simulation, and motion planning.
- Links define rigid bodies with visual, collision, and inertial properties, while joints define how links move relative to each other.
- Revolute joints are most common in humanoids, with limits specified in radians for rotation range and effort for torque capacity.
- Humanoid URDF files branch from a root link (torso or pelvis) into multiple kinematic chains for arms, legs, and head.
- RViz visualization with robot_state_publisher and joint_state_publisher_gui enables interactive testing and validation of URDF models.

## Check Your Understanding

1. What are the three types of information a URDF link can contain, and what is each used for?

2. You are modeling a humanoid knee joint that bends from fully straight to 140 degrees. Write the joint definition with appropriate type, axis, and limits.

3. Explain the difference between the `origin` element in a link's visual geometry versus the `origin` element in a joint. What does each specify?

4. Your URDF has a torso with two arms. Draw the kinematic tree showing links and joints. Which link should be the root, and why?

5. After loading your URDF in RViz, the right arm appears in the wrong location. What are three URDF elements you should check to diagnose the problem?

## Next Steps

Congratulations on completing Module 1! You have learned Physical AI concepts, ROS 2 architecture, and how to build complete ROS 2 systems. Module 2 will cover simulation environments, where you will use your URDF models in Gazebo to test robot behaviors before deploying to hardware. You will also learn sensor simulation, world building, and physics-based testing.

---
**Hardware Tier 1 Note**: Use online URDF visualizers like [urdf-viz-web](https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/) to view URDF files in your browser. Examine open-source humanoid URDFs from projects like NASA Valkyrie or Boston Dynamics Atlas (available on GitHub) to see production-quality robot descriptions. While you cannot run RViz without local ROS 2, studying URDF structure prepares you for hands-on work when you upgrade to Tier 2.
