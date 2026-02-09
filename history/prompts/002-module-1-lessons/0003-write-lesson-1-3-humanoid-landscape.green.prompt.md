---
id: 0003
title: Write Lesson 1.3 The Humanoid Landscape
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: user
command: Write Lesson 1.3 The Humanoid Landscape
labels: ["lesson-writing", "module-1", "physical-ai", "humanoid-robotics"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-3-humanoid-landscape.md
tests:
  - Manual validation against quality-checklist.md (PASS)
---

## Prompt

Write Lesson 1.3: The Humanoid Landscape

**Context:**
- Chapter README: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/README.md
- Research content: specs/002-module-1-lessons/research.md
- Constitution: .specify/memory/constitution.md
- Output path: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-3-humanoid-landscape.md

**Requirements:**
1. Read all 5 lesson-generator skill reference files from .claude/skills/lesson-generator/references/ before writing
2. Read the constitution at .specify/memory/constitution.md for glossary and standards
3. Read the chapter README for context and learning objectives
4. Read research.md for humanoid robotics market data, key players (Tesla Optimus, Figure 01/02, Unitree G1/H1, Boston Dynamics Atlas)

**Frontmatter values:**
- id: lesson-3-humanoid-landscape
- title: "Lesson 1.3: The Humanoid Landscape"
- sidebar_position: 3
- sidebar_label: "1.3 Humanoid Landscape"
- duration_minutes: 45
- proficiency_level: "A2"
- layer: "L1"
- hardware_tier: 1
- tier_1_path: "Fully conceptual - no hardware needed"
- chapter: "Chapter 1: Introduction to Physical AI"
- module: "Module 1: The Robotic Nervous System — ROS 2"

**Content focus:**
- Key players: Tesla Bot, Figure, Unitree, Boston Dynamics
- Market projections ($154B by 2035 per Goldman Sachs)
- Form factor advantages of humanoid robots
- Current status of each major platform

**No code examples** - this is a conceptual lesson.

Validate against quality-checklist.md before saving.

## Response snapshot

Successfully created Lesson 1.3: The Humanoid Landscape following all lesson-generator skill standards.

**Process:**
1. Read all 5 reference files from .claude/skills/lesson-generator/references/
2. Read constitution, chapter README, and research.md for context
3. Wrote lesson with complete YAML frontmatter (14 fields)
4. Structured content with 6 core H2 sections (300-500 tokens each):
   - Why Humanoid Form Factors Are Advantageous
   - The Market Opportunity for Humanoid Robotics
   - Tesla Optimus: Industrial Automation at Scale
   - Figure AI: Human-Robot Collaboration Focus
   - Unitree Robotics: Accessible Humanoid Platforms
   - Boston Dynamics Atlas: The Athletic Pioneer
5. Included hook, learning objectives, key takeaways, check your understanding, next steps
6. Validated against quality checklist

**Content highlights:**
- Explained infrastructure and tool compatibility advantages
- Covered $154B market projection with economic drivers
- Profiled all 4 major platforms with current capabilities and strategic positioning
- No code examples (conceptual lesson as specified)
- All sections self-contained for RAG retrieval
- Descriptive, keyword-rich headers

**Quality validation:**
- ✅ All 14 frontmatter fields present
- ✅ Learning objectives use Bloom's verbs (Identify, Explain, Describe, Summarize)
- ✅ 6 keywords (within 4-8 range)
- ✅ Description under 160 characters
- ✅ No banned phrases
- ✅ Active voice throughout
- ✅ 6 Key Takeaways
- ✅ 5 Check Your Understanding questions (mix recall and application)
- ✅ Hardware Tier 1 Note at bottom

## Outcome

- ✅ Impact: Created production-ready lesson file covering the humanoid robotics landscape with market data and key player analysis
- 🧪 Tests: Manual validation against quality-checklist.md - all items passed
- 📁 Files: 1 file created (lesson-3-humanoid-landscape.md)
- 🔁 Next prompts: Write Lesson 1.4 Sensor Systems (LIDAR, depth cameras, IMUs, force/torque sensors)
- 🧠 Reflection: Lesson successfully balances market/business context with technical platform details. RAG-optimized structure with self-contained sections for each major platform.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Quality checklist manual validation - PASS
- Prompt variant (if applicable): Standard lesson-generator workflow
- Next experiment (smallest change to try): Continue with Lesson 1.4 on sensor systems, which will be more technical and may benefit from diagrams
