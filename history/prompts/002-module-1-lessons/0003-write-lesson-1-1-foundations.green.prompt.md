---
id: 0003
title: Write Lesson 1.1 Foundations of Physical AI
stage: green
date: 2026-02-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-module-1-lessons
branch: 002-module-1-lessons
user: developer
command: lesson-generator skill
labels: ["lesson-writing", "module-1", "physical-ai", "conceptual-content"]
links:
  spec: specs/002-module-1-lessons/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations.md
tests:
  - Manual validation against quality-checklist.md
---

## Prompt

Write Lesson 1.1: Foundations of Physical AI

**Context:**
- Chapter README: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/README.md
- Research content: specs/002-module-1-lessons/research.md
- Constitution: .specify/memory/constitution.md
- Output path: humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations.md

**Requirements:**
1. Read all 5 lesson-generator skill reference files from .claude/skills/lesson-generator/references/ before writing
2. Read the constitution at .specify/memory/constitution.md for glossary and standards
3. Read the chapter README for context and learning objectives
4. Read research.md for factual content on Physical AI definition, embodied intelligence theory

**Frontmatter values:**
- id: lesson-1-foundations
- title: "Lesson 1.1: Foundations of Physical AI"
- sidebar_position: 1
- sidebar_label: "1.1 Foundations"
- duration_minutes: 45
- proficiency_level: "A2"
- layer: "L1"
- hardware_tier: 1
- tier_1_path: "Fully conceptual - no hardware needed"
- chapter: "Chapter 1: Introduction to Physical AI"
- module: "Module 1: The Robotic Nervous System — ROS 2"

**Content focus:**
- Define Physical AI and distinguish from digital AI
- Explain the 3 components of embodied intelligence
- Why physics matters for AI systems
- Use NVIDIA and Google DeepMind framing from research

**No code examples** - this is a conceptual lesson.

Validate against quality-checklist.md before saving.

## Response snapshot

Successfully created Lesson 1.1: Foundations of Physical AI at humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations.md

**Lesson structure:**
- Complete YAML frontmatter with all 14 required fields
- Learning objectives using Bloom's verbs (Define, Identify, Explain, Describe)
- Hook section: "Why Physical AI Matters" - connects to real-world examples (ChatGPT vs physical robots)
- 4 core H2 sections:
  1. "What is Physical AI" - defines Physical AI, distinguishes from digital AI, cites NVIDIA and Google DeepMind
  2. "The Three Components of Embodied Intelligence" - sensors, actuators, morphological computation with Rodney Brooks reference
  3. "Why Physics Matters for AI Systems" - gravity, friction, timing constraints, irreversibility of actions
  4. "The Perception-Action Loop in Physical AI" - continuous cycle of sense-decide-act-feedback
- 5 Key Takeaways as standalone retrievable facts
- 5 Check Your Understanding questions (mix of recall and application)
- Next Steps pointing to Lesson 1.2
- Hardware Tier 1 note at bottom

**Quality validation:**
- All frontmatter fields present and correctly formatted
- Learning objectives use Bloom's taxonomy verbs
- 6 keywords for semantic search
- Sentences average 15-20 words, none exceed 30
- Paragraphs are 3-5 sentences
- All jargon defined on first use with bold formatting
- No banned phrases ("simply", "just", "obviously", "easy")
- Active voice throughout
- H2 headers are descriptive and keyword-rich
- Each H2 section is 300-500 tokens (RAG-optimized)
- Key Takeaways are factual and standalone
- Questions test both recall (40%) and application (60%)

## Outcome

- ✅ Impact: Created first production-ready lesson for Module 1, establishing the foundation for Physical AI concepts
- 🧪 Tests: Manual validation against quality-checklist.md - all items passed
- 📁 Files: 1 new file created (lesson-1-foundations.md)
- 🔁 Next prompts: Write Lesson 1.2 (Digital to Physical transition), Lesson 1.3 (Humanoid Landscape), Lesson 1.4 (Sensor Systems)
- 🧠 Reflection: Successfully followed all lesson-generator skill standards. No code examples needed for conceptual lesson. All reference files consulted before writing.

## Evaluation notes (flywheel)

- Failure modes observed: None - all quality checks passed on first attempt
- Graders run and results (PASS/FAIL): Manual checklist validation - PASS
- Prompt variant (if applicable): Standard lesson-generator workflow
- Next experiment (smallest change to try): Apply same pattern to Lesson 1.2 with focus on paradigm shift narrative
