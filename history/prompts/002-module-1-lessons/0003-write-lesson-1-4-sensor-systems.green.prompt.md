---
id: 0003
title: Write Lesson 1.4 Sensor Systems
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: user
command: Write Lesson 1.4 Sensor Systems
labels: ["lesson-writing", "module-1", "sensors", "physical-ai", "conceptual"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-4-sensor-systems.md
tests:
  - Quality checklist validation (all items passed)
---

## Prompt

Write Lesson 1.4: Sensor Systems

**Context:**
- Chapter README: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/README.md
- Research content: specs/002-module-1-lessons/research.md
- Constitution: .specify/memory/constitution.md
- Output path: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-4-sensor-systems.md

**Requirements:**
1. Read all 5 lesson-generator skill reference files from .claude/skills/lesson-generator/references/ before writing
2. Read the constitution at .specify/memory/constitution.md for glossary and standards
3. Read the chapter README for context and learning objectives
4. Read research.md for sensor technologies: LIDAR principles, depth cameras (Intel RealSense), IMU mechanics, force/torque sensors

**Frontmatter values:**
- id: lesson-4-sensor-systems
- title: "Lesson 1.4: Sensor Systems"
- sidebar_position: 4
- sidebar_label: "1.4 Sensor Systems"
- duration_minutes: 60
- proficiency_level: "A2"
- layer: "L1"
- hardware_tier: 1
- tier_1_path: "Fully conceptual - no hardware needed"
- chapter: "Chapter 1: Introduction to Physical AI"
- module: "Module 1: The Robotic Nervous System — ROS 2"

**Content focus:**
- LIDAR (Light Detection and Ranging) principles
- Depth cameras (Intel RealSense) for 3D vision
- IMUs (Inertial Measurement Units) for balance and orientation
- Force/torque sensors for manipulation
- How sensors enable the perception-action loop

**No code examples** - this is a conceptual lesson.

Validate against quality-checklist.md before saving.

## Response snapshot

Successfully created Lesson 1.4: Sensor Systems with the following structure:

**Frontmatter**: All 14 required fields completed with 5 learning objectives using Bloom's verbs (Define, Explain, Describe, Compare, Identify), 8 keywords, and complete metadata.

**Content sections (6 H2 headers)**:
1. Why Robots Need Sensors (hook - connects to ChatGPT's limitations vs physical robots)
2. LIDAR: Measuring Distance with Light (time-of-flight principles, 360-degree mapping, point clouds)
3. Depth Cameras for 3D Vision (Intel RealSense, stereo/ToF/structured light, depth maps)
4. IMUs: The Robot's Inner Ear (accelerometer + gyroscope, balance control, drift and sensor fusion)
5. Force and Torque Sensors for Touch (strain gauges, manipulation, compliance)
6. How Sensors Enable the Perception-Action Loop (integration of all sensor types, timing requirements)

**Key Takeaways**: 5 factual bullet points covering each sensor type and sensor fusion concept.

**Check Your Understanding**: 5 questions mixing recall (40%) and application (60%) - glass detection problem, egg manipulation scenario, LIDAR vs depth camera comparison, IMU drift compensation, perception-action loop analysis.

**Quality validation results**:
- ✅ All frontmatter fields complete
- ✅ Learning objectives use Bloom's verbs
- ✅ Sentences under 30 words, paragraphs 3-5 sentences
- ✅ Technical terms bolded on first use (LIDAR, depth cameras, Intel RealSense, IMU, force sensors, torque sensors, perception-action loop)
- ✅ No banned phrases
- ✅ Active voice throughout
- ✅ H2 sections 300-500 tokens each, self-contained for RAG
- ✅ Descriptive, keyword-rich headers
- ✅ Hardware Tier 1 note included

## Outcome

- ✅ Impact: Completed Lesson 1.4 for Chapter 1, covering all four major sensor types used in humanoid robotics
- 🧪 Tests: Quality checklist validation passed (all items)
- 📁 Files: 1 new lesson file created (119 lines, ~2800 words)
- 🔁 Next prompts: Chapter 1 is now complete (4/4 lessons). Next: Begin Chapter 2 (ROS 2 Architecture) or create chapter assessment
- 🧠 Reflection: Conceptual lesson with no code examples as specified. Successfully integrated research content on LIDAR, depth cameras, IMUs, and force sensors. Strong focus on perception-action loop as the unifying concept. All sensor types connected to real humanoid robot use cases.

## Evaluation notes (flywheel)

- Failure modes observed: None - all reference files read successfully, quality standards met
- Graders run and results (PASS/FAIL): Quality checklist manual validation - PASS (all 65 items)
- Prompt variant (if applicable): Standard lesson-generator workflow with all 5 reference files
- Next experiment (smallest change to try): Consider adding a sensor comparison table in future sensor lessons for quick reference
