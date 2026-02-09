---
id: 0005
title: Write Lesson 2.3 Topics and Message Passing
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: user
command: Write Lesson 2.3 Topics and Message Passing
labels: ["lesson-writing", "ros2", "topics", "publisher-subscriber", "message-passing"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-3-topics-messages.md
tests:
  - N/A (content writing)
---

## Prompt

Write Lesson 2.3: Topics and Message Passing

**Context:**
- Chapter README: humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/README.md
- Research content: specs/002-module-1-lessons/research.md
- Constitution: .specify/memory/constitution.md
- Output path: humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-3-topics-messages.md

**Requirements:**
1. Read all 5 lesson-generator skill reference files from .claude/skills/lesson-generator/references/ before writing
2. Read the constitution at .specify/memory/constitution.md for glossary and standards
3. Read the chapter README for context and learning objectives
4. Read research.md for topic/publisher/subscriber patterns with Twist, LaserScan, Image messages

**Frontmatter values:**
- id: lesson-3-topics-messages
- title: "Lesson 2.3: Topics and Message Passing"
- sidebar_position: 3
- sidebar_label: "2.3 Topics & Messages"
- duration_minutes: 60
- proficiency_level: "B1"
- layer: "L2"
- hardware_tier: 2
- tier_1_path: "Use The Construct to publish and subscribe to topics in browser"
- chapter: "Chapter 2: ROS 2 Architecture"
- module: "Module 1: The Robotic Nervous System — ROS 2"

**Content focus:**
- Publisher/subscriber pattern
- Message types (Twist, LaserScan, Image)
- Topic naming conventions
- QoS profiles for topics
- Code examples: publisher node, subscriber node, Twist message for robot control

**Code examples**: Include illustrative snippets following four-part pattern (context + code + output + explanation).

Validate against quality-checklist.md before saving.

## Response snapshot

Successfully created Lesson 2.3: Topics and Message Passing with complete frontmatter, structured content, and validated code examples.

**Lesson structure:**
1. Read all 8 reference files (quality checklist, writing standards, frontmatter schema, constitution, chapter README, code patterns, research, lesson template)
2. Created complete YAML frontmatter with all 14 required fields
3. Wrote 6 core H2 sections (300-500 tokens each):
   - Understanding the Publisher-Subscriber Pattern
   - Message Types in ROS 2
   - Creating a Publisher Node
   - Creating a Subscriber Node
   - Topic Naming Conventions
   - Quality of Service Profiles for Topics
4. Included 3 complete code examples following four-part pattern:
   - Velocity publisher with Twist messages
   - Laser scan subscriber with LaserScan messages
   - Reliable publisher with custom QoS settings
5. Added learning objectives with Bloom's verbs
6. Included hook connecting to real-world warehouse robot scenario
7. Provided 5 key takeaways for RAG retrieval
8. Created 5 check-your-understanding questions (mix of recall and application)
9. Added next steps pointing to Lesson 2.4
10. Included Tier 1 hardware note with cloud alternative

**Quality validation:**
- All sentences under 30 words
- Paragraphs 3-5 sentences
- Active voice throughout
- No banned phrases ("simply", "just", "obviously")
- All technical terms defined on first use with bold
- Descriptive H2 headers with keywords
- Code blocks max 40 lines with all imports shown
- Expected output and line-by-line explanations included

## Outcome

- ✅ Impact: Created production-ready lesson file for Chapter 2, Lesson 3 covering ROS 2 topics and message passing
- 🧪 Tests: N/A (content writing, no code tests required)
- 📁 Files: 1 new lesson file created at humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-3-topics-messages.md
- 🔁 Next prompts: Write Lesson 2.4 (Services and Actions), or review/revise Lesson 2.3 based on feedback
- 🧠 Reflection: Successfully followed all lesson-generator skill standards. Code examples are practical and demonstrate real robotics patterns. QoS section provides important production knowledge often missing from beginner tutorials.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation against quality-checklist.md - PASS (all 65 checklist items verified)
- Prompt variant (if applicable): Standard lesson-generator workflow
- Next experiment (smallest change to try): Consider adding a diagram showing topic communication flow for visual learners
