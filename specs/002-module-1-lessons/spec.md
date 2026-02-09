# Feature Specification: Module 1 Lesson Content

**Feature Branch**: `002-module-1-lessons`
**Created**: 2026-02-07
**Status**: Draft
**Input**: User description: "Help me write all lessons for the Module 1 chapters"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learns Physical AI Foundations (Priority: P1)

A beginner student (Tier 1, cloud-only setup) opens Chapter 1 and works through all four lessons sequentially. Each lesson loads with complete frontmatter, renders correctly in Docusaurus sidebar, and follows the mandatory lesson skeleton (title, objectives, hook, core sections, takeaways, questions, next steps). The student completes all four conceptual lessons without needing any hardware beyond a laptop and browser.

**Why this priority**: Chapter 1 is the entry point — every student starts here. If these lessons are missing or broken, no one can begin the textbook. Zero hardware requirements make it the widest-reach content.

**Independent Test**: Navigate to `/docs/module-1-ros2/chapter-1-physical-ai/lesson-1-foundations` and verify the page renders with all sections, frontmatter parses correctly in sidebar, and internal links resolve.

**Acceptance Scenarios**:

1. **Given** a student visits Chapter 1, **When** they click Lesson 1.1, **Then** they see a complete lesson page with title, duration, tier, learning objectives, hook, 3+ core sections, key takeaways, self-assessment questions, and next steps link to Lesson 1.2
2. **Given** a student completes Lesson 1.1, **When** they follow the "Next Steps" link, **Then** they arrive at Lesson 1.2 which builds on Lesson 1.1 concepts
3. **Given** all four Chapter 1 lessons are published, **When** a student completes them in order, **Then** they can answer the Chapter 1 assessment checklist items from the chapter README

---

### User Story 2 - Student Learns ROS 2 Architecture (Priority: P2)

A student who completed Chapter 1 opens Chapter 2 and works through four hands-on ROS 2 lessons. Lessons include code examples following the four-part pattern (context, code, output, explanation). Tier 1 users have cloud-based alternatives documented. Tier 2 users follow along with local ROS 2 Humble installation.

**Why this priority**: Chapter 2 introduces the core technical skill (ROS 2) that the entire textbook builds upon. Without these lessons, students cannot progress to building packages in Chapter 3.

**Independent Test**: Navigate to Chapter 2 lessons, verify code examples have all four parts (context sentence, code block with imports, expected output, line-by-line explanation), and verify Tier 1 cloud alternatives are documented.

**Acceptance Scenarios**:

1. **Given** a student with ROS 2 installed, **When** they follow Lesson 2.2 code examples, **Then** they can create and run a ROS 2 node that prints output matching the "Expected output" block
2. **Given** a Tier 1 student without local ROS 2, **When** they read any Chapter 2 lesson, **Then** each lesson includes a Tier 1 cloud-based path so they can follow along
3. **Given** all four Chapter 2 lessons are complete, **When** a student finishes them, **Then** they understand when to use topics vs services vs actions (testable via Check Your Understanding questions)

---

### User Story 3 - Student Builds ROS 2 Packages (Priority: P3)

A Tier 2 student works through Chapter 3's four hands-on lessons to create a complete ROS 2 package from scratch, integrate Python AI agents, write launch files, and create a humanoid URDF. By the end, they complete the Module 1 capstone (Robot Health Monitor).

**Why this priority**: Chapter 3 is the culmination of Module 1. It turns conceptual knowledge into a buildable project. However, it depends on Chapters 1 and 2 being complete first.

**Independent Test**: A Tier 2 student follows Lesson 3.1 step-by-step and produces a buildable ROS 2 package that passes `colcon build` without errors.

**Acceptance Scenarios**:

1. **Given** a Tier 2 student with ROS 2 Humble, **When** they follow Lesson 3.1 code examples, **Then** they create a ROS 2 package that builds successfully with `colcon build`
2. **Given** a student completes all Chapter 3 lessons, **When** they attempt the capstone project (Robot Health Monitor), **Then** they have all the building blocks from prior lessons to complete it
3. **Given** a Tier 1 student, **When** they read Chapter 3 lessons, **Then** each lesson explains the Tier 1 workaround so they can follow conceptually

---

### User Story 4 - RAG Chatbot Retrieves Lesson Content (Priority: P2)

A student asks the embedded RAG chatbot a question like "What is Physical AI?" The chatbot retrieves the relevant H2 section from the lesson content and provides an accurate, self-contained answer.

**Why this priority**: RAG retrieval is a core product feature. Every lesson's structure (300-500 token H2 sections, keyword-rich headers, standalone Key Takeaways) directly serves chatbot accuracy.

**Independent Test**: Extract all H2 sections from a lesson and verify each is self-contained (makes sense without surrounding context) and falls within 300-500 token range.

**Acceptance Scenarios**:

1. **Given** lesson content with H2 sections sized 300-500 tokens, **When** a RAG system chunks by H2 boundaries, **Then** each chunk is self-contained and answers a specific question
2. **Given** Key Takeaways written as standalone facts, **When** a student asks a factual question covered by the lesson, **Then** the takeaway text provides a direct answer without needing surrounding context

---

### Edge Cases

- What happens when a student skips directly to Chapter 3 without completing Chapters 1-2? Each lesson lists prerequisites in frontmatter; the chapter README links back to required prior chapters.
- What happens when a Tier 1 student encounters a Tier 2 code example? Every lesson includes a `tier_1_path` in frontmatter and an inline hardware tier note at the bottom with a cloud/browser alternative.
- What happens when a linked lesson file is not yet published? Docusaurus is configured with `onBrokenLinks: 'warn'` so builds succeed with warnings for future content. Existing chapter README links to lesson files will resolve once content is created.
- What happens when a student cannot run a code example? Every code block shows expected output so students can verify correctness even in read-only mode.

## Clarifications

### Session 2026-02-07

- Q: Should code examples in Chapters 2-3 be fully runnable end-to-end code or illustrative snippets? → A: Illustrative snippets — code blocks explain concepts but may omit boilerplate; students don't produce runnable output every lesson.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Each lesson MUST contain complete YAML frontmatter with all 14 mandatory fields (id, title, sidebar_position, sidebar_label, description, duration_minutes, proficiency_level, layer, hardware_tier, tier_1_path, learning_objectives, keywords, prerequisites, chapter, module)
- **FR-002**: Each lesson MUST follow the mandatory body skeleton: title + metadata block, learning objectives, hook, core H2 sections, key takeaways, check your understanding, next steps, hardware tier note
- **FR-003**: Every H2 core section MUST be 300-500 tokens and self-contained for RAG chunking
- **FR-004**: Every H2 header MUST be descriptive and keyword-rich (not clever or vague)
- **FR-005**: Every code block MUST follow the four-part pattern: context sentence, illustrative code snippet (max 40 lines, may omit boilerplate but must show key imports), expected output, line-by-line explanation. Code is concept-focused, not required to be copy-paste-runnable.
- **FR-006**: Every lesson MUST include a non-empty `tier_1_path` in frontmatter describing the cloud/browser fallback
- **FR-007**: Learning objectives MUST use Bloom's taxonomy verbs (Define, Explain, Build, Compare, Design, etc.)
- **FR-008**: Writing MUST follow the writing standards: 15-20 word sentences (max 30), 3-5 sentence paragraphs, active voice, no banned phrases
- **FR-009**: Technical terms MUST be bolded and defined on first use within each lesson
- **FR-010**: Each lesson MUST include 3-5 Key Takeaways written as standalone retrievable facts
- **FR-011**: Each lesson MUST include 3-5 Check Your Understanding questions (40% recall, 60% application)
- **FR-012**: Next Steps section MUST link to the following lesson or chapter with context
- **FR-013**: Lesson files MUST use the naming convention `lesson-{n}-{slug}.md` matching the existing chapter README link targets
- **FR-014**: All 12 lessons MUST be created (4 per chapter, 3 chapters)
- **FR-015**: Code examples involving motor control or hardware MUST include `:::danger` safety admonitions
- **FR-016**: The Docusaurus site MUST build successfully (`npm run build`) with no errors after all lessons are added

### Lesson Inventory

#### Chapter 1: Introduction to Physical AI (4 lessons)

| # | File | Title | Duration | Tier | Layer |
|---|------|-------|----------|------|-------|
| 1.1 | `lesson-1-foundations.md` | Foundations of Physical AI | 45 min | 1 | L1 |
| 1.2 | `lesson-2-digital-to-physical.md` | From Digital AI to Physical | 30 min | 1 | L1 |
| 1.3 | `lesson-3-humanoid-landscape.md` | The Humanoid Landscape | 45 min | 1 | L1 |
| 1.4 | `lesson-4-sensor-systems.md` | Sensor Systems | 60 min | 1 | L1 |

#### Chapter 2: ROS 2 Architecture (4 lessons)

| # | File | Title | Duration | Tier | Layer |
|---|------|-------|----------|------|-------|
| 2.1 | `lesson-1-core-concepts.md` | ROS 2 Core Concepts | 45 min | 1-2 | L1 |
| 2.2 | `lesson-2-nodes.md` | Nodes - The Building Blocks | 60 min | 2 | L2 |
| 2.3 | `lesson-3-topics-messages.md` | Topics and Message Passing | 60 min | 2 | L2 |
| 2.4 | `lesson-4-services-actions.md` | Services and Actions | 60 min | 2 | L2 |

#### Chapter 3: Building with ROS 2 (4 lessons)

| # | File | Title | Duration | Tier | Layer |
|---|------|-------|----------|------|-------|
| 3.1 | `lesson-1-packages-python.md` | Building ROS 2 Packages with Python | 75 min | 2 | L2 |
| 3.2 | `lesson-2-rclpy-agents.md` | Bridging Python Agents with rclpy | 75 min | 2 | L2-L3 |
| 3.3 | `lesson-3-launch-params.md` | Launch Files and Parameters | 60 min | 2 | L2 |
| 3.4 | `lesson-4-urdf-humanoids.md` | Understanding URDF for Humanoids | 90 min | 2 | L2 |

### Key Entities

- **Lesson**: A single instructional unit with frontmatter, body content, and self-assessment. Belongs to one chapter and one module. Identified by `lesson-{n}-{slug}` convention.
- **Chapter**: A group of 4 related lessons covering one major topic area. Has a README overview page. Three chapters in Module 1.
- **Module**: The top-level curriculum unit (Module 1: ROS 2). Contains 3 chapters and 12 lessons spanning 5 weeks.
- **Hardware Tier**: Equipment level (1-4) that determines which activities a student can perform. Every lesson must support Tier 1 with a fallback path.
- **Teaching Layer**: Pedagogical progression (L1-L4) from manual foundation to spec-driven AI collaboration.

## Assumptions

- The existing chapter README files define the canonical lesson filenames, titles, durations, and tier requirements. Lessons must match these exactly.
- ROS 2 Humble is the target distribution for all code examples (current LTS as of the textbook's context).
- Python 3 is the primary language for code examples. rclpy is the ROS 2 Python client library.
- Foxglove Studio and The Construct are the recommended Tier 1 cloud alternatives.
- Lesson proficiency starts at A2 (beginner) for Chapter 1 and progresses to B1 (intermediate) for Chapters 2-3.
- The `onBrokenLinks: 'warn'` configuration in docusaurus.config.ts remains in place so future module links don't break the build.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 12 lesson files exist at the correct paths and render in the Docusaurus sidebar with proper titles and ordering
- **SC-002**: Every lesson passes all items in the quality checklist (frontmatter completeness, clarity rules, structure, code patterns, accessibility, RAG optimization)
- **SC-003**: `npm run build` succeeds with zero errors after all 12 lessons are added
- **SC-004**: Every H2 section across all lessons falls within 300-500 tokens when measured
- **SC-005**: Every code example in Chapters 2-3 follows the four-part pattern with key imports shown, expected output documented, and concept-focused illustrative snippets (not required to be fully runnable)
- **SC-006**: A student with only a laptop and browser (Tier 1) can complete all Chapter 1 lessons and follow the Tier 1 path for Chapters 2-3
- **SC-007**: All lesson cross-references (Next Steps links, prerequisite references) resolve to existing pages
- **SC-008**: No lesson contains banned phrases ("simply", "just", "obviously", "easy", "trivial", "as everyone knows")
