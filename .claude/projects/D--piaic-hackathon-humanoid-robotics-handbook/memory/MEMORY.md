# Project Memory: Physical AI & Humanoid Robotics Handbook

## Active Feature: 002-module-1-lessons
- **Branch**: `002-module-1-lessons`
- **Spec**: `specs/002-module-1-lessons/spec.md` (complete, clarified)
- **Plan**: `specs/002-module-1-lessons/plan.md` (complete)
- **Research**: `specs/002-module-1-lessons/research.md` (PENDING — agent lost, needs recreation)
- **Tasks**: Not yet created (next: `/sp.tasks`)
- **Status**: Plan complete, research needs to be written, then `/sp.tasks`

## Key Decisions
- **Code examples**: Illustrative snippets (not fully runnable) — clarified in spec
- **Agent strategy**: Use `chapter-lesson-writer` agent for each lesson via Task tool
- **Skill reference files**: Agent reads 5 files from `.claude/skills/lesson-generator/references/`
- **Batch order**: Ch1 (parallel) → Ch2 (parallel) → Ch3 (parallel) — sequential by chapter
- **Proficiency**: A2 for Ch1, B1 for Ch2-Ch3

## Lesson Inventory (12 lessons)
See `specs/002-module-1-lessons/plan.md` for full table with tiers, layers, durations.

| Lesson | File | Chapter Dir |
|--------|------|-------------|
| 1.1 | lesson-1-foundations.md | chapter-1-physical-ai |
| 1.2 | lesson-2-digital-to-physical.md | chapter-1-physical-ai |
| 1.3 | lesson-3-humanoid-landscape.md | chapter-1-physical-ai |
| 1.4 | lesson-4-sensor-systems.md | chapter-1-physical-ai |
| 2.1 | lesson-1-core-concepts.md | chapter-2-ros2-architecture |
| 2.2 | lesson-2-nodes.md | chapter-2-ros2-architecture |
| 2.3 | lesson-3-topics-messages.md | chapter-2-ros2-architecture |
| 2.4 | lesson-4-services-actions.md | chapter-2-ros2-architecture |
| 3.1 | lesson-1-packages-python.md | chapter-3-building-ros2 |
| 3.2 | lesson-2-rclpy-agents.md | chapter-3-building-ros2 |
| 3.3 | lesson-3-launch-params.md | chapter-3-building-ros2 |
| 3.4 | lesson-4-urdf-humanoids.md | chapter-3-building-ros2 |

## Agent/Skill Setup (already wired)
- `.claude/agents/chapter-lesson-writer.md` — updated to reference skill files
- `.claude/skills/lesson-generator/SKILL.md` — portable, not tied to constitution
- 5 reference files in `.claude/skills/lesson-generator/references/`

## Patterns Learned
- `create-new-feature.ps1` fails with positional param error but still creates branch+file
- `onBrokenLinks: 'warn'` in docusaurus.config.ts — keeps build passing with future links
- Infima CSS overrides require `!important` or high-specificity selectors
- Windows: `init_skill.py` fails with Unicode (emoji) on cp1252 — create dirs manually
- PHR IDs: check existing files in `history/prompts/<feature>/` to avoid collision

## PHR Index for 002-module-1-lessons
- 001: specify-module-1-lessons (spec stage)
- 002: clarify-module-1-lessons (spec stage)
- 003: plan-module-1-lessons (plan stage) — PENDING creation

## Files Modified This Session
- `specs/002-module-1-lessons/spec.md` — created + clarified
- `specs/002-module-1-lessons/plan.md` — created
- `specs/002-module-1-lessons/checklists/requirements.md` — created
- `.claude/agents/chapter-lesson-writer.md` — updated (wired to skill)
- `history/prompts/002-module-1-lessons/001-*.prompt.md` — created
- `history/prompts/002-module-1-lessons/002-*.prompt.md` — created
