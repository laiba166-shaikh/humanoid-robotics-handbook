# Agent Memory: Chapter & Lesson Writer

## Lesson-Generator Skill Workflow

### Successful Pattern (Validated 2026-02-09)
1. **Read all 5 reference files first** from `.claude/skills/lesson-generator/references/`:
   - frontmatter-schema.md
   - lesson-template.md
   - writing-standards.md
   - code-patterns.md
   - quality-checklist.md
2. **Read project context**: constitution.md, chapter README, research materials
3. **Write lesson** following mandatory skeleton
4. **Validate** against quality-checklist.md before delivery
5. **Create PHR** in `history/prompts/{feature-name}/`

### Content Patterns Discovered

**Conceptual Lessons (No Code)**
- Still require same structural rigor as code-heavy lessons
- Market/business content benefits from platform-specific H2 sections
- Each company/platform = one self-contained RAG chunk
- Use bold for key terms: **Infrastructure compatibility**, **Strategic advantages**

**RAG Optimization for Survey Lessons**
- When covering multiple entities (companies, platforms, technologies):
  - Give each entity its own H2 section
  - Use descriptive headers: "Tesla Optimus: Industrial Automation at Scale"
  - Keep sections 300-500 tokens for optimal retrieval
  - Include current status, capabilities, and strategic positioning in each

### Quality Checklist Effectiveness
- Following reference files closely = zero validation issues
- Bloom's verbs in learning objectives: Identify, Explain, Describe, Summarize, Compare, Analyze
- Banned phrases to avoid: "simply", "just", "obviously", "easy", "trivial", "as everyone knows"
- Sentence target: 15-20 words (max 30)
- Paragraph target: 3-5 sentences

### File Paths
- Lessons: `humanoid-textbook/docs/{module-folder}/{chapter-folder}/lesson-{n}-{slug}.md`
- PHRs: `history/prompts/{feature-name}/{ID}-{slug}.{stage}.prompt.md`
- Reference files: `.claude/skills/lesson-generator/references/`

## Project-Specific Conventions

### Module 1 Structure
- Module: "Module 1: The Robotic Nervous System — ROS 2"
- Chapter 1: "Chapter 1: Introduction to Physical AI"
- Lessons use format: "Lesson X.Y: Title"
- All Chapter 1 lessons are Tier 1 (conceptual, no hardware)

### Terminology (from constitution.md)
- Physical AI (not "Embodied AI" in most contexts)
- ROS 2 (not "ROS2" or "ros2" in prose)
- Humanoid (not "android" when being specific)
- Hardware Tier (not "hardware level")

### Frontmatter Requirements
All 14 fields mandatory:
- id, title, sidebar_position, sidebar_label, description
- duration_minutes, proficiency_level, layer, hardware_tier, tier_1_path
- learning_objectives (2-5 items), keywords (4-8 items), prerequisites
- chapter, module

## Lessons Completed

### Chapter 1: Introduction to Physical AI (4/4 complete)
- ✅ Lesson 1.1: Foundations of Physical AI
- ✅ Lesson 1.2: From Digital to Physical AI
- ✅ Lesson 1.3: The Humanoid Robotics Landscape
- ✅ Lesson 1.4: Sensor Systems

### Chapter 2: ROS 2 Architecture (4/4 complete)
- ✅ Lesson 2.1: ROS 2 Core Concepts (45 min, Tier 1-2)
- ✅ Lesson 2.2: Nodes - The Building Blocks (60 min, Tier 2)
- ✅ Lesson 2.3: Topics and Message Passing (60 min, Tier 2)
- ✅ Lesson 2.4: Services and Actions (60 min, Tier 2)

### Chapter 3: Building with ROS 2 (4/4 complete)
- ✅ Lesson 3.1: Building ROS 2 Packages with Python (75 min, Tier 2)
- ✅ Lesson 3.2: Bridging Python Agents with rclpy (75 min, Tier 2)
- ✅ Lesson 3.3: Launch Files and Parameters (60 min, Tier 2)
- ✅ Lesson 3.4: Understanding URDF for Humanoids (90 min, Tier 2)

**Module 1 Status: COMPLETE (12/12 lessons)**

## Key Patterns from Module 1 Completion

**Chapter 2 (Architecture) Patterns**
- QoS comparison tables for decision-making guidance
- Publisher/subscriber code examples with four-part pattern
- Real-world hooks (warehouse robots, sensor timing)
- Topic naming conventions section essential for system design

**Chapter 3 (Building) Patterns**
- Package structure lessons need directory tree visualizations
- Launch file examples should show both inline params and YAML configs
- AI integration lessons benefit from perception-decision-action loop framing
- URDF lessons need progressive complexity (simple arm → humanoid upper body)

**Code Example Insights**
- Keep examples under 40 lines (validated across 8 code-heavy lessons)
- Always show all imports (never assume reader knows them)
- Expected output builds confidence that code works
- Line-by-line explanations for key concepts only (not every line)

**Multi-Lesson Coherence**
- Each lesson's "Next Steps" should explicitly reference the following lesson
- Prerequisites should reference specific prior lessons, not just chapters
- Terminology must be consistent across all lessons (check constitution.md)
- Code patterns should build progressively (simple → complex)
