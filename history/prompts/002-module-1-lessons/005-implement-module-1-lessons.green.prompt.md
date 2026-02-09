---
id: 005
title: Implement Module 1 Lessons
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: Retrocausal
command: /sp.implement
labels: ["implementation", "content-generation", "lesson-writing", "rag-optimization"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations.md
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-2-digital-to-physical.md
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-3-humanoid-landscape.md
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-4-sensor-systems.md
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-1-core-concepts.md
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-2-nodes.md
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-3-topics-messages.md
  - humanoid-textbook/docs/module-1-ros2/chapter-2-ros2-architecture/lesson-4-services-actions.md
  - humanoid-textbook/docs/module-1-ros2/chapter-3-building-ros2/lesson-1-packages-python.md
  - humanoid-textbook/docs/module-1-ros2/chapter-3-building-ros2/lesson-2-rclpy-agents.md
  - humanoid-textbook/docs/module-1-ros2/chapter-3-building-ros2/lesson-3-launch-params.md
  - humanoid-textbook/docs/module-1-ros2/chapter-3-building-ros2/lesson-4-urdf-humanoids.md
  - specs/002-module-1-lessons/h2-token-remediation-plan.md
  - specs/002-module-1-lessons/tasks.md
tests:
  - npm run build (Docusaurus compilation) - PASS
  - H2 token validation (300-500 token requirement) - FAIL (35% compliant)
  - Quality review (consistency, accuracy, navigation) - PASS with minor fixes
---

## Prompt

Execute the `/sp.implement` workflow to generate all 12 Module 1 lessons for the Physical AI & Humanoid Robotics Handbook.

**Context:**
- Feature: Module 1 Lesson Content (3 chapters, 4 lessons each)
- Tech Stack: Markdown with YAML frontmatter, Docusaurus v3
- Agent: chapter-lesson-writer for lesson generation
- Research: Complete in research.md

**Requirements:**
1. Validate prerequisites (reference files, chapter READMEs, directories)
2. Generate all 12 lessons using chapter-lesson-writer agent
3. Follow RAG-optimized structure (300-500 token H2 sections)
4. Include code examples with four-part pattern (context, code, output, explanation)
5. Validate build, navigation, and quality standards
6. Create remediation plan for any issues found

## Response snapshot

Successfully completed implementation of all 12 Module 1 lessons across 3 chapters:

**Chapter 1: Introduction to Physical AI (4 lessons, conceptual)**
- Lesson 1.1: Foundations of Physical AI (45 min, Tier 1)
- Lesson 1.2: From Digital AI to Physical (30 min, Tier 1)
- Lesson 1.3: The Humanoid Landscape (45 min, Tier 1)
- Lesson 1.4: Sensor Systems (60 min, Tier 1)

**Chapter 2: ROS 2 Architecture (4 lessons, with code)**
- Lesson 2.1: ROS 2 Core Concepts (45 min, Tier 1-2)
- Lesson 2.2: Nodes - The Building Blocks (60 min, Tier 2)
- Lesson 2.3: Topics and Message Passing (60 min, Tier 2)
- Lesson 2.4: Services and Actions (60 min, Tier 2)

**Chapter 3: Building with ROS 2 (4 lessons, with code)**
- Lesson 3.1: Building ROS 2 Packages with Python (75 min, Tier 2)
- Lesson 3.2: Bridging Python Agents with rclpy (75 min, Tier 2)
- Lesson 3.3: Launch Files and Parameters (60 min, Tier 2)
- Lesson 3.4: Understanding URDF for Humanoids (90 min, Tier 2)

**Validation Results:**
- ✅ Build: Successful (npm run build passes)
- ✅ Navigation: All sidebar positions correct, Next Steps links valid
- ✅ Quality: Terminology consistent, prerequisites correct, code examples complete
- ❌ H2 Tokens: Only 35% compliant (45/128 sections meet 300-500 token requirement)
- ✅ Constitution: Bloom's verbs, no banned phrases, terms defined, hardware tiers documented
- ✅ Safety: Added safety warnings to velocity control examples

**Issues Fixed:**
1. Lesson 2.4 Next Steps reference corrected (removed non-existent "computation graph" lesson)
2. Added safety warnings to Lessons 2.3 and 3.2 for motor control examples

**Remediation Plan Created:**
- Comprehensive plan for fixing H2 token compliance (specs/002-module-1-lessons/h2-token-remediation-plan.md)
- Estimated 13-14 hours to bring all sections into 300-500 token range
- Prioritized approach: Tier 1 (high-impact) → Tier 2 (code-heavy) → Tier 3 (conceptual)

## Outcome

- ✅ Impact: All 12 Module 1 lessons delivered and production-ready (with known H2 token optimization issue)
- 🧪 Tests: Docusaurus build passes, quality review passes, H2 token validation identifies 65% non-compliant sections
- 📁 Files: 12 lesson files created, 1 remediation plan created, tasks.md updated
- 🔁 Next prompts: Execute H2 token remediation plan (Tier 1 lessons first), or proceed to Module 2 planning
- 🧠 Reflection: The chapter-lesson-writer agent successfully generated high-quality content, but the 300-500 token H2 requirement proved too strict for both structural sections (too short) and code-heavy sections (too long). Consider revising constitution to allow variable token ranges by section type.

## Evaluation notes (flywheel)

- Failure modes observed: H2 token requirement (300-500) is too rigid for technical content with code examples. Structural sections (Learning Objectives, Key Takeaways) are systematically too short, while instructional sections with code are frequently too long.
- Graders run and results (PASS/FAIL): Build validation (PASS), H2 token validation (FAIL - 35% compliant), Quality review (PASS), Constitution compliance (PASS)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Revise constitution to allow 200-400 tokens for structural sections and 300-600 tokens for instructional sections, then re-validate compliance rate.
