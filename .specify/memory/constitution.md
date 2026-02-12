# Project Constitution: Physical AI & Humanoid Robotics Textbook

## Project Identity

**Name**: Physical AI & Humanoid Robotics Handbook
**Purpose**: An AI-native technical textbook teaching Physical AI and Humanoid Robotics with integrated RAG chatbot
**Target Audience**: Students and professionals learning robotics, from beginners to advanced practitioners

---

## Core Principles

### 1. Educational Clarity
- Use Bloom's taxonomy verbs in all learning objectives
- One concept per page (max 2000 words)
- Progressive complexity: L1 → L2 → L3 → L4 teaching layers
- Clear prerequisites for every lesson

### 2. Hardware-Aware Content
Every lesson MUST specify hardware requirements:
| Tier | Equipment | Description |
|------|-----------|-------------|
| Tier 1 | Laptop + Cloud | No GPU required, browser-based |
| Tier 2 | RTX GPU + Ubuntu | Local ROS 2, Gazebo |
| Tier 3 | Jetson Edge | Real sensors, deployment |
| Tier 4 | Physical Robot | Unitree Go2/G1 |

**Rule**: Every lesson MUST work for Tier 1 with cloud fallback.

### 3. RAG-Optimized Content
- Complete YAML frontmatter on every page (15+ fields)
- H2-level section headers for chunk boundaries
- **Variable token ranges by section type** (optimal for retrieval):
  - **Structural sections** (Learning Objectives, Key Takeaways, Check Your Understanding, Next Steps): 200-400 tokens
  - **Instructional sections** (concept explanations, theory): 300-600 tokens
  - **Code-heavy sections** (examples with explanations): 400-700 tokens
  - **Soft limit**: Authors should target 800 tokens max per section
  - **Ingestion safety net**: Sections exceeding 700 tokens are automatically split with overlap by the ingestion pipeline. Existing content (e.g., Module 1 code-heavy sections up to 950 tokens) is handled gracefully.
- Consistent terminology across all lessons
- Keywords embedded naturally for semantic search
- See `docs/rag-implementation-guide.md` for chunking strategy

### 4. Safety-First for Physical AI
Motor control and robot lessons MUST include:
- Emergency stop patterns
- Joint limits enforcement
- Velocity constraints
- Sensor validation before action
- Simulation-first approach (Gazebo before physical)

### 5. 4-Layer Teaching Framework
| Layer | When to Use | AI Role |
|-------|-------------|---------|
| L1 (Manual Foundation) | New concepts, safety-critical | Minimal |
| L2 (AI Collaboration) | After foundation built | Active partner |
| L3 (Intelligence Design) | Recurring patterns | Co-designer |
| L4 (Spec-Driven) | Capstone projects | Full orchestrator |

**Key Rule**: Students master manual basics BEFORE AI collaboration.

---

## Technical Standards

### Docusaurus Conventions
| Do | Don't |
|----|-------|
| Use `.md` extension | `.mdx` (unless React needed) |
| Name folders `README.md` | `index.md` |
| Use `"less than"` in prose | `<` characters outside code |
| ASCII diagrams | Mermaid (not configured) |
| `proficiency_level` | `cefr_level` |
| `duration_minutes` | `duration` |

### Frontmatter Schema (Required Fields)
```yaml
---
id: unique-lesson-id
title: "Lesson Title"
sidebar_position: 1
sidebar_label: "Short Label"
description: "SEO description"
duration_minutes: 45
proficiency_level: "A2"  # A2/B1/C2
layer: "L1"              # L1/L2/L3/L4
hardware_tier: 1         # 1-4
tier_1_path: "Cloud alternative description"
learning_objectives:
  - "Bloom's verb + measurable outcome"
keywords:
  - "keyword1"
  - "keyword2"
prerequisites:
  - "Previous lesson or skill"
chapter: "Chapter Name"
module: "Module Name"
---
```

### Code Standards
- All code examples must be tested and runnable
- Include expected output in comments
- Safety warnings in callout boxes for hardware code
- Python 3.10+ syntax
- ROS 2 Humble/Iron compatible

---

## Quality Gates

### Before Merging Content
1. ✅ Frontmatter complete (all 15 fields)
2. ✅ Learning objectives use Bloom's verbs
3. ✅ Hardware tier specified
4. ✅ Tier 1 path documented
5. ✅ Code examples tested
6. ✅ Safety warnings present (if motor control)
7. ✅ Section lengths within type-specific ranges (200-800 tokens, see RAG guidelines)

### Before Deployment
1. ✅ `npm run build` succeeds
2. ✅ RAG ingestion completes
3. ✅ 5+ test queries return relevant results
4. ✅ GitHub Pages accessible
5. ✅ Railway backend healthy

---

## Brand Identity

### Color Palette (MUST USE CONSISTENTLY)

**CRITICAL**: Use ONLY these colors throughout the entire project. Do not introduce new colors.

| Name | Hex | CSS Variable | Usage |
|------|-----|--------------|-------|
| Azure Mist | `#e5f9fa` | `--color-azure-mist` | Light backgrounds, light mode text on dark |
| Pacific Blue | `#12b2c2` | `--color-pacific-blue` | Primary accent, buttons, CTAs |
| Jet Black | `#212a2f` | `--color-jet-black` | Dark backgrounds, dark mode base |
| Stormy Teal | `#24727c` | `--color-stormy-teal` | Secondary accent, hover states |
| Pacific Cyan | `#0e8b9f` | `--color-pacific-cyan` | Links, interactive elements |

**Color Usage Rules**:
1. **Dark Mode**: Jet Black background, Azure Mist text, Pacific Blue accents
2. **Light Mode**: White/Azure Mist background, Jet Black text, Pacific Cyan accents
3. **Links**: Pacific Blue (dark) / Pacific Cyan (light), hover with Stormy Teal
4. **Buttons**: Pacific Blue primary, Stormy Teal secondary
5. **Borders/Dividers**: Stormy Teal or Pacific Blue at 20% opacity
6. **Code blocks**: Slightly lighter/darker than background (use surface color)
7. **Alerts/Callouts**: Use palette colors with appropriate opacity
8. **Charts/Diagrams**: Use only palette colors for data visualization

**DO NOT**:
- Use random hex colors not in the palette
- Use pure black (#000000) or pure white (#ffffff) for text
- Introduce new accent colors
- Use colors with different opacity without design justification

### Voice & Tone
- Technical but accessible
- Encouraging without being patronizing
- Safety-conscious without being alarmist
- Practical over theoretical

---

## Content Writing Standards

*These standards govern every lesson, chapter overview, and module page. They ensure stakeholders — students, career-switchers, bootcamp instructors, and industry reviewers — receive consistent, high-quality technical content.*

### Stakeholder Alignment

| Stakeholder | What They Need | How We Serve Them |
|-------------|----------------|-------------------|
| CS students (beginner) | Clear foundations, no assumed robotics knowledge | Concept-first lessons, analogies to software they know |
| Software engineers (career switch) | Fast path from existing skills to robotics | Python-first examples, map to patterns they know |
| Bootcamp participants | Structured, time-boxed learning | Duration estimates, clear prerequisites, self-assessment |
| ESL readers | Parseable English, no idioms | Short sentences, defined terms, consistent vocabulary |
| Instructors/mentors | Teachable structure, assignment-ready | Learning objectives, check-your-understanding, hardware tiers |
| Industry reviewers (hackathon judges) | Technical accuracy, professional quality | Verified code, cited sources, current ecosystem data |

### Core Content Principles

#### 1. Clarity Is Non-Negotiable

Every sentence must be understood on the first read. If a reader has to re-read a sentence, the sentence has failed.

| Rule | Rationale |
|------|-----------|
| One idea per sentence | Reduces cognitive load |
| One concept per paragraph | Allows scanning and retrieval |
| Define before you use | No forward-referencing jargon |
| Simplest correct word | "Send" not "transmit", "use" not "leverage" |

```
BAD:  "The node leverages the pub-sub paradigm to facilitate
       inter-process communication via DDS middleware."

GOOD: "Nodes send messages to each other using publishers and
       subscribers. Under the hood, ROS 2 uses DDS (Data
       Distribution Service) to deliver these messages."
```

#### 2. Concrete Before Abstract

Always lead with something tangible — a code snippet, a real-world analogy, or a diagram — before explaining the theory.

```
BAD:  "A ROS 2 topic is a named bus over which nodes exchange
       typed messages using an anonymous publish-subscribe model."

GOOD: "Imagine a radio channel. Anyone can broadcast on it, and
       anyone tuned in will hear the message. A ROS 2 topic works
       the same way — it's a named channel where nodes publish
       and subscribe to messages."
```

**Pattern**: Example/Analogy → Definition → Details → "Why It Matters"

#### 3. Respect the Reader's Time

- Get to the point in the first 2 sentences of every section
- Use bullet points and tables for scannable content
- **Bold** key terms on first use
- Provide duration estimates so readers can plan their study sessions
- Never pad content — if a section can be said in 3 sentences, don't write 6

#### 4. Progressive Disclosure

Teach the 80% case first. Edge cases, optimizations, and advanced patterns come later or link to separate sections.

| Stage | What to Include | What to Defer |
|-------|----------------|---------------|
| First mention | Core behavior, basic usage | Configuration options, internals |
| Code example | Working minimum | Error handling, logging |
| Explanation | "What it does" and "why" | "How it works internally" |

#### 5. Every Concept Earns Its Place

If a concept doesn't connect to something the reader will build, question whether it belongs. Within 2 paragraphs of introducing any concept, answer: **"Why does this matter to you?"**

### Language Rules

#### Sentence Structure
- **Target**: 15-20 words per sentence
- **Maximum**: 30 words — break longer sentences
- **Paragraphs**: 3-5 sentences maximum
- **Active voice always**: "The publisher sends a message" not "A message is sent by the publisher"
- **Transition words**: "First," "However," "As a result," "For example," to guide the reader

#### Banned Phrases

These phrases undermine the reader's confidence. Remove them entirely.

| Never Write | Why | Write Instead |
|-------------|-----|---------------|
| "Simply do X" | Nothing is simple to a beginner | "Do X" |
| "Obviously" | If it were obvious, you wouldn't explain it | Remove or explain why |
| "Just" | Minimizes complexity | Remove entirely |
| "Easy" / "Trivial" | Shames readers who struggle | "Straightforward" or remove |
| "As everyone knows" | Excludes those who don't know | State the fact directly |
| "He/she" | Not inclusive | "They" or "the developer" |

#### Technical Terminology Protocol
1. **First use**: Write the full term with definition in bold — **Node**: an independent ROS 2 executable that performs computation
2. **Subsequent uses**: Use the short form consistently — "node"
3. **Acronyms**: Expand on first use — "Robot Operating System 2 (ROS 2)" then "ROS 2"
4. **Cross-module terms**: Must match the Glossary section exactly — no synonyms

### Lesson Body Structure

Every lesson MUST follow this skeleton for consistency and RAG optimization:

```markdown
# Lesson X.Y: [Title]

**Duration**: XX minutes
**Hardware Tier**: Tier X (Description)
**Layer**: LX (Layer Name)

## Learning Objectives
By the end of this lesson, you will be able to:
- [Bloom's verb] + [measurable outcome]

## [The Hook — Why This Matters]
[2-3 sentences connecting to a real-world application or problem.
 This section answers: "Why should I care about this?"]

## [Core Section 1 — H2 Header]
[300-500 tokens. Self-contained for RAG retrieval.
 Descriptive header, not clever — "How Nodes Communicate"
 not "Talking Shop"]

## [Core Section 2 — H2 Header]
[Continue pattern. Each H2 = one RAG chunk boundary.]

## Key Takeaways
[3-5 bullet points. These are the facts the RAG chatbot
 should retrieve when asked about this lesson's topic.]

## Check Your Understanding
[3-5 self-assessment questions. Mix factual recall and
 application questions.]

## Next Steps
[1-2 sentences pointing to the next lesson with context.]

---
**Hardware Tier X Note**: [Tier-specific alternative or guidance]
```

### Code Example Pattern

Every code block MUST follow the four-part pattern:

```markdown
**What we're building**: [1 sentence — the problem this code solves]

\`\`\`python
# [filename hint: e.g., velocity_publisher.py]
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
\`\`\`

**Expected output**:
\`\`\`
[INFO] Created publisher on topic /cmd_vel with queue size 10
\`\`\`

**What's happening**:
- Line 6: We inherit from `Node`, the base class for all ROS 2 executables
- Line 8: `create_publisher(msg_type, topic, queue_size)` registers on the topic
```

| Code Rule | Rationale |
|-----------|-----------|
| Max 40 lines per block | Longer blocks lose attention |
| Show all imports | Never assume the reader knows them |
| Comments explain "why", not "what" | Code shows what; comments explain reasoning |
| Meaningful names | `velocity_publisher` not `vp` or `pub1` |
| Filename hint in first comment | Reader knows where to save it |

### Section Header Rules (Critical for RAG)

Section headers directly impact RAG retrieval quality. They must be:

| Rule | Example |
|------|---------|
| Descriptive, not clever | "How ROS 2 Nodes Communicate" not "The Art of Robot Talk" |
| Self-contained meaning | "Installing ROS 2 on Ubuntu" not "Getting Started" |
| Include key terms | "Publisher-Subscriber Pattern in ROS 2" not "The Pattern" |
| No questions as H2 | Use statements: "Why Humanoids Need Balance" not "Why Do Humanoids Need Balance?" |

### Engagement Patterns

Use Docusaurus admonitions consistently:

| Admonition | When to Use | Example |
|------------|-------------|---------|
| `:::tip` | Efficiency gains, shortcuts for experienced readers | "Use `ros2 topic echo` to debug messages in real-time" |
| `:::note` | Important context that's not blocking | "ROS 2 Iron is the latest release but Humble has longer support" |
| `:::warning` | Common mistakes | "Forgetting to source your workspace is the #1 beginner error" |
| `:::danger` | Physical safety (motors, hardware) | "Always set joint velocity limits before running on hardware" |

**Interactive prompts** within lessons:
- **Try This** (5 min): Mini-exercise within the lesson flow
- **Challenge** (optional): Harder exercise for advanced learners
- **Real-World Connection**: Industry application of the concept

### Content Quality Checklist

Before any lesson is considered complete:

**Clarity**
- [ ] Every sentence understood on first read (read aloud test)
- [ ] No sentence exceeds 30 words
- [ ] No paragraph exceeds 5 sentences
- [ ] All jargon defined on first use
- [ ] No banned phrases ("simply", "just", "obviously", "easy")

**Accuracy**
- [ ] All code examples tested and produce expected output
- [ ] Version numbers and CLI commands verified against current docs
- [ ] Technical claims cite official documentation or authoritative source
- [ ] No invented APIs or fabricated data

**Structure**
- [ ] Learning objectives use Bloom's verbs and are measurable
- [ ] Every H2 section is 300-500 tokens (RAG-optimized)
- [ ] H2 headers are descriptive and keyword-rich (not clever)
- [ ] Key Takeaways present and factually accurate
- [ ] Check Your Understanding questions test application, not just recall

**Accessibility**
- [ ] Alt text on all images describes content, not decoration
- [ ] Tables have clear headers and don't rely on color alone
- [ ] Code blocks have sufficient surrounding context
- [ ] No information conveyed by color alone

---

## Success Metrics

| Metric | Target |
|--------|--------|
| RAG search latency | < 500ms (95th percentile) |
| Hardware tier filtering accuracy | 100% |
| Frontmatter compliance | 100% |
| Content coverage | 4 modules, 10 chapters, 40 lessons |
| Module 1 implementation | 12 full lessons |

---

## Tech Stack (Mandatory)

| Component | Technology | Version/Tier |
|-----------|------------|--------------|
| Frontend | Docusaurus | v3.x + TypeScript |
| Styling | Infima + Custom CSS | (No Tailwind) |
| Backend | FastAPI | Python 3.10+ |
| Vector DB | Qdrant Cloud | Free tier |
| Embeddings | Cohere | embed-english-v3.0 |
| Database | Neon Postgres | Serverless |
| Chat UI | OpenAI ChatKit SDK | Latest |
| Auth | Better-Auth | (Stretch goal) |
| Book Hosting | GitHub Pages | - |
| API Hosting | Railway | - |
| Workflow | Spec-Kit Plus | - |

**DO NOT** introduce alternative technologies without updating this constitution.

---

## Project Structure

```
humanoid-robotics-handbook/
├── humanoid-textbook/           # Docusaurus frontend
│   ├── docs/
│   │   ├── intro.md             # Landing page (sidebar_position: 1)
│   │   ├── module-1-ros2/
│   │   │   ├── README.md        # Module overview
│   │   │   ├── chapter-1-*/
│   │   │   │   ├── README.md    # Chapter overview
│   │   │   │   └── lesson-*.md  # Individual lessons
│   │   │   └── ...
│   │   └── module-2-*/...
│   ├── src/
│   │   ├── components/          # React components
│   │   └── css/custom.css       # Theme (color palette)
│   └── static/
│       └── img/                 # Images and assets
├── rag-backend/                 # FastAPI backend
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py              # FastAPI app, CORS, lifespan
│   │   ├── config.py            # Settings via pydantic-settings
│   │   ├── models/              # Pydantic domain, request, response models
│   │   ├── api/                 # Route handlers (health, search, chat)
│   │   ├── services/            # Business logic (embeddings, vectorstore, chat, reranker)
│   │   ├── ingestion/           # Parser, chunker, pipeline
│   │   └── ingest.py            # CLI entry: python -m app.ingest
│   ├── tests/
│   ├── .env.example
│   ├── requirements.txt
│   └── Procfile
├── .claude/skills/              # Reusable Claude Code skills
├── .specify/memory/             # Spec-Kit Plus (this file)
├── specs/                       # Feature specifications
└── plan.md                      # Execution plan
```

### Naming Conventions
| Type | Convention | Example |
|------|------------|---------|
| Module folders | `module-{n}-{slug}` | `module-1-ros2` |
| Chapter folders | `chapter-{n}-{slug}` | `chapter-1-physical-ai` |
| Lesson files | `lesson-{n}-{slug}.md` | `lesson-1-foundations.md` |
| Component folders | PascalCase | `ChatWidget/` |
| CSS files | kebab-case | `custom.css` |
| Python files | snake_case | `embeddings.py` |
| Skills | kebab-case | `lesson-writer.md` |

---

## Backend API Standards

### Endpoint Conventions
```
GET  /api/health          → Health check
POST /api/search          → Semantic search
POST /api/chat            → RAG chat
```
### Ingestion runs via CLI: python -m app.ingest (no API endpoint)

### Response Format
```json
{
  "success": true,
  "data": { ... },
  "error": null,
  "meta": {
    "latency_ms": 123,
    "source_count": 5
  }
}
```

### Error Handling
- Always return JSON, never HTML error pages
- Include error codes: `VALIDATION_ERROR`, `NOT_FOUND`, `RATE_LIMITED`, `INTERNAL_ERROR`, `SERVICE_UNAVAILABLE`
- Log errors with context (query, user_tier, timestamp)

### CORS
- Allow only: `https://{username}.github.io` (production)
- Allow `http://localhost:3000` (development)

---

## RAG Chatbot Behavior

### Persona
```
You are a helpful teaching assistant for the Physical AI & Humanoid Robotics textbook.
You answer questions based ONLY on the textbook content.
You cite sources as [Module X, Chapter Y, Lesson Z].
You adapt explanations to the user's hardware tier when known.
```

### Response Rules
1. **Only answer from retrieved content** - Never make up information
2. **Always cite sources** - Format: `[Module 1, Chapter 2, Lesson 3]`
3. **Respect hardware tiers** - Don't suggest Tier 3 solutions to Tier 1 users
4. **Admit uncertainty** - Say "I don't have information on that in the textbook"
5. **Suggest next steps** - Point to relevant lessons for deeper learning
6. **Code safety** - Always include safety warnings when discussing motor control

### Citation Format
```
Based on [Module 1, Chapter 2, Lesson 3 - ROS 2 Nodes]:
"A node is an independent executable that performs computation..."
```

### Selected Text Feature
When user selects text and asks about it:
1. Use the selected text as primary context
2. Find related chunks for additional context
3. Explain the selection, don't just repeat it

---

## Image & Asset Guidelines

### Image Requirements
| Type | Format | Max Size | Dimensions |
|------|--------|----------|------------|
| Screenshots | PNG | 500KB | 1200px wide max |
| Diagrams | SVG preferred, PNG fallback | 200KB | - |
| Photos | JPG | 300KB | 1200px wide max |
| Icons | SVG | 10KB | 24x24 or 48x48 |

### Naming Convention
```
{module}-{chapter}-{description}.{ext}
Example: m1-c2-ros2-node-diagram.svg
```

### Alt Text
**REQUIRED** on all images for accessibility:
```markdown
![ROS 2 node communication diagram showing publisher-subscriber pattern](./img/m1-c2-ros2-node-diagram.svg)
```

### Storage Location
- All images in `humanoid-textbook/static/img/`
- Reference as `/img/filename.ext` in markdown

---

## Git Conventions

### Commit Message Format
```
<type>(<scope>): <short description>

<optional body>

Co-Authored-By: Claude <noreply@anthropic.com>
```

### Types
| Type | Usage |
|------|-------|
| `feat` | New lesson, chapter, feature |
| `fix` | Bug fix, typo correction |
| `docs` | Documentation only |
| `style` | Formatting, no code change |
| `refactor` | Code restructure |
| `test` | Adding tests |
| `chore` | Build, config changes |

### Examples
```
feat(module-1): add lesson on ROS 2 nodes
fix(rag): correct chunk size calculation
docs(readme): update deployment instructions
```

### Branch Strategy
- `main` - Production, deployed to GitHub Pages
- `feat/*` - Feature branches (optional for hackathon)

---

## Environment Variables

### Required for Backend
```bash

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://...

# Cohere API Key
COHERE_API_KEY=...

# Optional
GITHUB_PAGES_URL=https://username.github.io/humanoid-robotics-handbook
```

### Never Commit
- `.env` files
- API keys
- Database credentials

---

## Glossary (Key Terms for RAG Consistency)

Use these exact terms consistently throughout:

| Term | Definition | Do NOT Use |
|------|------------|------------|
| Physical AI | AI systems operating in the physical world | Embodied AI (use sparingly) |
| ROS 2 | Robot Operating System 2 | ROS2, ros2 (in prose) |
| Node | Independent ROS 2 executable | Process (when referring to ROS) |
| Topic | Named bus for message passing | Channel, stream |
| Gazebo | Physics simulation environment | Gazebo Sim, gz |
| Isaac Sim | NVIDIA's robot simulation | Omniverse (too broad) |
| URDF | Unified Robot Description Format | Robot description file |
| Hardware Tier | User's equipment level (1-4) | Hardware level, tier |
| Humanoid | Human-shaped robot | Android, robot (when specific) |

---

## Spec-Kit Plus Workflow

### When Starting a New Feature
```
1. /sp.specify  → Define what to build
2. /sp.plan     → Design the approach
3. /sp.tasks    → Break into checklist
4. /sp.implement → Execute with Claude Code
5. Validate against spec
```

### Specification Template
```markdown
# Feature: [Name]

## Intent
What problem does this solve?

## Success Criteria
- [ ] Criterion 1
- [ ] Criterion 2

## Non-Goals
What this feature does NOT do

## Technical Approach
How to implement
```

---

## Claude Code Skills Guidelines

### Skill File Structure
```markdown
# Skill: [name]

## Purpose
One-line description

## Persona
How Claude should think when using this skill

## Questions to Ask
- Question 1?
- Question 2?

## Principles
- Principle 1
- Principle 2

## Output Format
Expected output structure
```

### Skill Location
All skills in `.claude/skills/` with `.md` extension

### Required Skills for This Project
1. `lesson-writer.md` - Generate RAG-optimized lessons
2. `rag-content-validator.md` - Validate content structure
3. `rag-backend-builder.md` - Create FastAPI components

---

## Accessibility Basics

### Required
- Alt text on all images
- Sufficient color contrast (4.5:1 minimum)
- Semantic HTML headings (h1 → h2 → h3, no skipping)
- Keyboard navigable components
- No flashing content

### Color Contrast Check
| Combination | Ratio | Status |
|-------------|-------|--------|
| Pacific Blue on Jet Black | 5.2:1 | ✅ Pass |
| Azure Mist on Jet Black | 12.1:1 | ✅ Pass |
| Pacific Cyan on White | 4.6:1 | ✅ Pass |

---

*This constitution governs all content creation and technical decisions for the Physical AI & Humanoid Robotics Handbook project.*
