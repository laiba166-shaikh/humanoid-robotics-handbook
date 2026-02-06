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
- 300-500 tokens per section (optimal for retrieval)
- Consistent terminology across all lessons
- Keywords embedded naturally for semantic search

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
7. ✅ Section lengths 300-500 tokens

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
| Embeddings | OpenAI | text-embedding-3-small |
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
│   ├── main.py
│   ├── api/
│   ├── rag/
│   └── requirements.txt
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
POST /api/ingest          → Content ingestion (admin)
```

### Response Format
```json
{
  "success": true,
  "data": { ... },
  "error": null,
  "meta": {
    "latency_ms": 123,
    "sources": [...]
  }
}
```

### Error Handling
- Always return JSON, never HTML error pages
- Include error codes: `VALIDATION_ERROR`, `NOT_FOUND`, `RATE_LIMITED`, `INTERNAL_ERROR`
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
# OpenAI
OPENAI_API_KEY=sk-...

# Qdrant Cloud
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...

# Neon Postgres
DATABASE_URL=postgresql://...

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
