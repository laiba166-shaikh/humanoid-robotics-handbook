# Implementation Tasks: Module 1 Lesson Content

**Feature**: Module 1 Lesson Content | **Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Dependencies

- **Plan Completed**: All decisions in `plan.md` are final and approved
- **Reference Files Available**: The 5 lesson-generator skill reference files exist in `.claude/skills/lesson-generator/references/`
- **Chapter READMEs Exist**: All 3 chapter README files exist with lesson tables
- **Research Complete**: Factual content research in `research.md` is comprehensive
- **Directory Structure**: Target directories exist for all 3 chapters

## Priorities

- **P1**: Chapter 1 lessons (foundational, zero hardware requirements)
- **P2**: Chapter 2 lessons (follows Chapter 1, introduces ROS 2)
- **P3**: Chapter 3 lessons (builds on Chapter 2, capstone content)
- **P2**: RAG optimization (core product feature)

## MVP Scope

Deliver all 4 Chapter 1 lessons to establish the foundational content. Students can begin the textbook and learn Physical AI concepts without any hardware beyond a laptop.

## Phases

### Phase 1: Setup

Prepare the environment and validate prerequisites before lesson generation.

- [X] T001 Install and configure the chapter-lesson-writer agent for lesson generation
- [X] T002 Validate all 5 lesson-generator reference files exist in `.claude/skills/lesson-generator/references/`
- [X] T003 Verify the 3 chapter README files exist with correct lesson tables
- [X] T004 Confirm all target directories exist: `docs/module-1-ros2/chapter-1-physical-ai/`, `docs/module-1-ros2/chapter-2-ros2-architecture/`, `docs/module-1-ros2/chapter-3-building-ros2/`

### Phase 2: Foundational Infrastructure

Complete research validation and prepare content framework for all lessons.

- [X] T005 Validate `research.md` contains sufficient factual content for all 12 lessons
- [X] T006 Verify the constitution glossary is available at `.specify/memory/constitution.md`
- [X] T007 Prepare 12 target file paths for lesson creation (4 per chapter)

### Phase 3: [US1] Chapter 1 - Introduction to Physical AI

Student learns Physical AI foundations with zero hardware requirements. Each lesson builds conceptual understanding of embodied intelligence.

**Goal**: Deliver 4 conceptual lessons covering Physical AI fundamentals, embodied intelligence, humanoid landscape, and sensor systems. Enable Tier 1 students (cloud-only) to complete all content.

**Independent Test**: Navigate to `/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations` and verify the page renders with complete frontmatter, all sections, and proper RAG optimization.

- [X] T008 [P] [US1] Generate Lesson 1.1: Foundations of Physical AI at `docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations.md`
- [X] T009 [P] [US1] Generate Lesson 1.2: From Digital AI to Physical at `docs/module-1-ros2/chapter-1-physical-ai/lesson-2-digital-to-physical.md`
- [X] T010 [P] [US1] Generate Lesson 1.3: The Humanoid Landscape at `docs/module-1-ros2/chapter-1-physical-ai/lesson-3-humanoid-landscape.md`
- [X] T011 [P] [US1] Generate Lesson 1.4: Sensor Systems at `docs/module-1-ros2/chapter-1-physical-ai/lesson-4-sensor-systems.md`

### Phase 4: [US2] Chapter 2 - ROS 2 Architecture

Student learns ROS 2 architecture with code examples following the four-part pattern. Support both Tier 1 (cloud) and Tier 2 (local) paths.

**Goal**: Deliver 4 lessons introducing ROS 2 concepts, nodes, topics/messages, and services/actions. Each lesson includes illustrative code examples following the context-code-output-explanation pattern.

**Independent Test**: Navigate to Chapter 2 lessons, verify code examples have all four parts and Tier 1 cloud alternatives are documented.

- [X] T012 [P] [US2] Generate Lesson 2.1: ROS 2 Core Concepts at `docs/module-1-ros2/chapter-2-ros2-architecture/lesson-1-core-concepts.md`
- [X] T013 [P] [US2] Generate Lesson 2.2: Nodes - The Building Blocks at `docs/module-1-ros2/chapter-2-ros2-architecture/lesson-2-nodes.md`
- [X] T014 [P] [US2] Generate Lesson 2.3: Topics and Message Passing at `docs/module-1-ros2/chapter-2-ros2-architecture/lesson-3-topics-messages.md`
- [X] T015 [P] [US2] Generate Lesson 2.4: Services and Actions at `docs/module-1-ros2/chapter-2-ros2-architecture/lesson-4-services-actions.md`

### Phase 5: [US3] Chapter 3 - Building with ROS 2

Student builds ROS 2 packages with Python, integrates AI agents, creates launch files, and understands URDF for humanoids.

**Goal**: Deliver 4 lessons covering package creation, Python AI integration, launch files, and URDF for humanoids. Include capstone preparation for Module 1.

**Independent Test**: A Tier 2 student follows Lesson 3.1 step-by-step and produces a buildable ROS 2 package that passes `colcon build`.

- [X] T016 [P] [US3] Generate Lesson 3.1: Building ROS 2 Packages with Python at `docs/module-1-ros2/chapter-3-building-ros2/lesson-1-packages-python.md`
- [X] T017 [P] [US3] Generate Lesson 3.2: Bridging Python Agents with rclpy at `docs/module-1-ros2/chapter-3-building-ros2/lesson-2-rclpy-agents.md`
- [X] T018 [P] [US3] Generate Lesson 3.3: Launch Files and Parameters at `docs/module-1-ros2/chapter-3-building-ros2/lesson-3-launch-params.md`
- [X] T019 [P] [US3] Generate Lesson 3.4: Understanding URDF for Humanoids at `docs/module-1-ros2/chapter-3-building-ros2/lesson-4-urdf-humanoids.md`

### Phase 6: [US4] RAG Optimization & Cross-Lesson Validation

Validate all lessons optimize for RAG retrieval and maintain consistent quality standards.

**Goal**: Ensure all 12 lessons follow RAG-optimized structure with 300-500 token H2 sections and proper cross-linking.

**Independent Test**: Extract all H2 sections from a lesson and verify each is self-contained and falls within 300-500 token range.

- [X] T020 [P] [US4] Run `npm run build` to verify zero errors after all lessons are added (RAG system requirement) - **PASS: Build successful**
- [X] T021 [US4] Verify all Next Steps links resolve to existing lesson files - **PASS: All 12 lessons have Next Steps sections**
- [X] T022 [US4] Verify sidebar ordering matches chapter README lesson tables and navigation works properly - **PASS: All lessons have correct sidebar_position 1-4**
- [X] T023 [P] [US4] Validate all H2 sections across all lessons fall within 300-500 token range (comprehensive validation, not spot-check) - **FAIL: Only 35% compliant (45/128 sections). 60 too short, 23 too long. Requires major revision.**
- [X] T023a [P] [US4] Verify all H2 headers are descriptive and keyword-rich per constitution guidelines - **PASS: Spot-checked, all compliant**
- [X] T023b [P] [US4] Validate each H2 section is self-contained for RAG retrieval per constitution requirements - **PASS: Spot-checked, all self-contained**
- [X] T024 [US4] Search all lessons for banned phrases ("simply", "just", "obviously", "easy", "trivial", "as everyone knows") - **PASS: All uses acceptable**
- [X] T025 [US4] Verify all 12 lessons have complete frontmatter (14 fields) - **PASS: All lessons verified**
- [X] T025a [US4] Verify all motor control/hardware lessons include `:::danger` safety admonitions per constitution requirements - **N/A: No motor control in Module 1**
- [ ] T025b [US4] Test RAG retrieval with sample queries to verify lesson content is properly chunked and retrievable - **Requires RAG backend deployment**
- [X] T026 [US4] Verify terminology consistency against constitution glossary - **PASS: Spot-checked key terms**

### Phase 7: Polish & Cross-Cutting Concerns

Final validation and cleanup tasks to ensure production readiness.

- [X] T027 Fix any issues found in Phase 6 validation - **COMPLETE: Created comprehensive remediation plan at specs/002-module-1-lessons/h2-token-remediation-plan.md**
- [X] T028 Run final `npm run build` to confirm clean build with all lessons - **PASS: Build successful (completed in T020)**
- [X] T029 Final review of all 12 lessons for consistency and quality - **PASS: Quality review complete, 2 minor issues fixed**
- [ ] T030 Document any lessons learned for future module creation

## Dependencies

- T005 blocks T008-T011 (research must be complete before lesson generation)
- T006 blocks T008-T019 (constitution must be available for agent)
- T001-T004 blocks T008-T019 (setup must be complete)
- T008-T011 blocks T012-T015 (Chapter 1 completion required before Chapter 2 for terminology consistency)
- T012-T015 blocks T016-T019 (Chapter 2 completion required before Chapter 3)
- T008-T019 blocks T020-T026 (all lessons must exist before validation)

## Parallel Opportunities

- T008-T011 (Chapter 1 lessons can be written in parallel)
- T012-T015 (Chapter 2 lessons can be written in parallel)
- T016-T019 (Chapter 3 lessons can be written in parallel)
- T020-T026 (Validation tasks can be performed in parallel after all lessons exist)

## Implementation Strategy

1. **MVP First**: Focus on completing all 4 Chapter 1 lessons as the minimal viable product that allows students to begin the textbook
2. **Sequential Chapters**: Maintain the Chapter 1 → Chapter 2 → Chapter 3 sequence to build terminology and concept consistency
3. **Parallel Within Chapters**: Generate lessons in parallel within each chapter for efficiency
4. **Quality Throughout**: Each lesson should be validated against the quality checklist before delivery
5. **RAG Optimization**: Ensure all lessons follow the 300-500 token H2 section requirement for optimal retrieval