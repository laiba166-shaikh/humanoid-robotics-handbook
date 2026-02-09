---
id: 0005
title: Write Lesson 2.4 Services and Actions
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: developer
command: lesson-generator
labels: ["lesson-writing", "ros2", "services", "actions", "module-1"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-4-services-actions.md
tests:
  - Manual validation against quality-checklist.md
---

## Prompt

Write Lesson 2.4: Services and Actions

**Context:**
- Chapter README: humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/README.md
- Research content: specs/002-module-1-lessons/research.md
- Constitution: .specify/memory/constitution.md
- Output path: humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-4-services-actions.md

**Requirements:**
1. Read all 5 lesson-generator skill reference files from .claude/skills/lesson-generator/references/ before writing
2. Read the constitution at .specify/memory/constitution.md for glossary and standards
3. Read the chapter README for context and learning objectives
4. Read research.md for service/action patterns with example service definitions

**Frontmatter values:**
- id: lesson-4-services-actions
- title: "Lesson 2.4: Services and Actions"
- sidebar_position: 4
- sidebar_label: "2.4 Services & Actions"
- duration_minutes: 60
- proficiency_level: "B1"
- layer: "L2"
- hardware_tier: 2
- tier_1_path: "Use The Construct to create service servers and clients in browser"
- chapter: "Chapter 2: ROS 2 Architecture"
- module: "Module 1: The Robotic Nervous System — ROS 2"

**Content focus:**
- Services for synchronous request-response
- Actions for long-running tasks with feedback
- When to use topics vs services vs actions
- Service definitions (.srv files)
- Action definitions (.action files)
- Code examples: AddTwoInts service, action client with feedback

**Code examples**: Include illustrative snippets following four-part pattern (context + code + output + explanation).

Validate against quality-checklist.md before saving.

## Response snapshot

Successfully created Lesson 2.4: Services and Actions with complete content following all lesson-generator standards.

**Lesson Structure:**
- Complete YAML frontmatter with all 14 required fields
- Learning objectives using Bloom's verbs (Explain, Implement, Build, Define)
- Hook section connecting to real-world robot communication needs
- 6 core H2 sections (300-500 tokens each):
  1. Understanding Request-Response Communication in ROS 2
  2. ROS 2 Services for Synchronous Operations
  3. Service Definition Files and Custom Service Types
  4. ROS 2 Actions for Long-Running Tasks
  5. Action Definition Files and Feedback Mechanisms
  6. Choosing Between Topics, Services, and Actions
- 4 complete code examples following four-part pattern:
  1. AddTwoInts service client
  2. AddTwoInts service server
  3. Fibonacci action client with feedback
  4. Service and action definition file examples
- Decision table comparing topics, services, and actions
- 5 Key Takeaways
- 5 Check Your Understanding questions
- Next Steps pointing to computation graph lesson
- Tier 1 hardware note

**Quality Validation:**
✅ All frontmatter fields complete
✅ Bloom's taxonomy verbs in learning objectives
✅ No banned phrases ("simply", "just", "obviously")
✅ Active voice throughout
✅ All jargon bolded on first use (Services, Actions, goal, feedback, result)
✅ Code examples under 40 lines with all imports shown
✅ Expected output and line-by-line explanations provided
✅ H2 headers descriptive and keyword-rich for RAG optimization
✅ Tier 1 cloud alternative documented

## Outcome

- ✅ Impact: Completed Lesson 2.4 for Chapter 2 (ROS 2 Architecture), teaching services and actions communication patterns
- 🧪 Tests: Manual validation against quality-checklist.md - all items passed
- 📁 Files: Created 1 new lesson file (4,800+ words, 6 H2 sections, 4 code examples)
- 🔁 Next prompts: Continue with remaining Module 1 lessons or move to Chapter 3
- 🧠 Reflection: Successfully applied all lesson-generator standards including four-part code pattern, RAG-optimized sections, and comprehensive service/action coverage

## Evaluation notes (flywheel)

- Failure modes observed: None - all reference files read successfully, all standards applied
- Graders run and results (PASS/FAIL): Quality checklist validation PASS (all items)
- Prompt variant (if applicable): Standard lesson-generator workflow
- Next experiment (smallest change to try): Consider adding more real-world examples of when to choose each communication pattern
