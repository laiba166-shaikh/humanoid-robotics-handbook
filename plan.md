# Physical AI & Humanoid Robotics Textbook - 24-Hour Execution Plan

## Executive Summary

**Starting Point**: Empty project (only requirements.md and takeaways files)
**Target Score**: 200/200 points (100 base + 50 skills + 50 auth)
**Deadline**: Friday, Feb 06, 2026 at 10:00 PM
**Realistic Target**: 150-175 points in 24 hours

### Your Current Setup
- Neon Postgres account
- GitHub repo created
- **Need to create**: OpenAI API account, Qdrant Cloud account

### Priority Decision
**More content, basic chat** - Focus on completing Module 1-2 content thoroughly with simpler ChatKit integration. Auth/personalization as stretch goal.

---

## Point Strategy

| Component | Points | Priority | Risk Level |
|-----------|--------|----------|------------|
| Docusaurus Textbook + GitHub Pages | 40 | CRITICAL | Low |
| RAG Chatbot (Qdrant + FastAPI + ChatKit) | 60 | CRITICAL | Medium |
| Reusable Claude Code Skills (3) | 50 | HIGH | Low |
| Better-Auth + Hardware Survey | 50 | MEDIUM | Medium |

---

## Spec-Kit Plus Workflow

Integrated throughout the project phases:

```
/sp.constitution → Define project principles (Hour 0)
       ↓
/sp.specify → Define feature/module specs (Before each phase)
       ↓
/sp.plan → Create implementation approach
       ↓
/sp.tasks → Generate task checklist
       ↓
/sp.implement → Execute with Claude Code
       ↓
/sp.validate → Check against spec
```

**Constitution Principles** (to define in Hour 0):
- Educational clarity: Bloom's taxonomy verbs in objectives
- Hardware-aware: Every lesson specifies tier requirements
- RAG-optimized: Proper frontmatter, H2 chunking, keywords
- Safety-first: Motor control lessons include emergency stops
- 4-Layer Teaching Framework (L1-L4)

---

## Complete Book Outline

**Implementation Scope**: Full outline for all 4 modules, implement Module 1 only (3 chapters, 12 lessons)

### Module 1: The Robotic Nervous System (ROS 2) — Weeks 1-5 ✅ IMPLEMENT

```
Module 1: The Robotic Nervous System (ROS 2)
├── README.md (Module overview page)
├── chapter-1-physical-ai/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-foundations.md         ✅ Implement
│   ├── lesson-2-digital-to-physical.md ✅ Implement
│   ├── lesson-3-humanoid-landscape.md  ✅ Implement
│   └── lesson-4-sensor-systems.md      ✅ Implement
├── chapter-2-ros2-architecture/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-core-concepts.md       ✅ Implement
│   ├── lesson-2-nodes.md               ✅ Implement
│   ├── lesson-3-topics-messages.md     ✅ Implement
│   └── lesson-4-services-actions.md    ✅ Implement
└── chapter-3-building-ros2/
    ├── README.md (Chapter overview)
    ├── lesson-1-packages-python.md     ✅ Implement
    ├── lesson-2-rclpy-agents.md        ✅ Implement
    ├── lesson-3-launch-params.md       ✅ Implement
    └── lesson-4-urdf-humanoids.md      ✅ Implement
```

### Module 2: The Digital Twin (Gazebo & Unity) — Weeks 6-7 📝 OUTLINE ONLY

```
Module 2: The Digital Twin (Gazebo & Unity)
├── README.md (Module overview page)
├── chapter-1-gazebo/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-environment-setup.md   📝 Outline
│   ├── lesson-2-urdf-sdf.md            📝 Outline
│   ├── lesson-3-physics-simulation.md  📝 Outline
│   └── lesson-4-sensor-simulation.md   📝 Outline
└── chapter-2-unity/
    ├── README.md (Chapter overview)
    ├── lesson-1-unity-intro.md         📝 Outline
    ├── lesson-2-rendering.md           📝 Outline
    ├── lesson-3-hri-unity.md           📝 Outline
    └── lesson-4-sensor-sim-unity.md    📝 Outline
```

### Module 3: The AI-Robot Brain (NVIDIA Isaac) — Weeks 8-10 📝 OUTLINE ONLY

```
Module 3: The AI-Robot Brain (NVIDIA Isaac)
├── README.md (Module overview page)
├── chapter-1-isaac-sim/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-isaac-sdk.md           📝 Outline
│   ├── lesson-2-photorealistic-sim.md  📝 Outline
│   └── lesson-3-synthetic-data.md      📝 Outline
├── chapter-2-isaac-ros/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-vslam.md               📝 Outline
│   ├── lesson-2-perception.md          📝 Outline
│   └── lesson-3-rl-control.md          📝 Outline
└── chapter-3-deployment/
    ├── README.md (Chapter overview)
    ├── lesson-1-nav2-bipedal.md        📝 Outline
    ├── lesson-2-sim-to-real.md         📝 Outline
    └── lesson-3-jetson-deploy.md       📝 Outline
```

### Module 4: Vision-Language-Action (VLA) — Weeks 11-13 📝 OUTLINE ONLY

```
Module 4: Vision-Language-Action (VLA)
├── README.md (Module overview page)
├── chapter-1-humanoid-dev/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-kinematics.md          📝 Outline
│   ├── lesson-2-locomotion.md          📝 Outline
│   ├── lesson-3-manipulation.md        📝 Outline
│   └── lesson-4-hri-design.md          📝 Outline
├── chapter-2-conversational/
│   ├── README.md (Chapter overview)
│   ├── lesson-1-whisper-voice.md       📝 Outline
│   ├── lesson-2-speech-nlu.md          📝 Outline
│   ├── lesson-3-multimodal.md          📝 Outline
│   └── lesson-4-cognitive-planning.md  📝 Outline
└── chapter-3-capstone/
    ├── README.md (Chapter overview)
    ├── lesson-1-specification.md       📝 Outline
    ├── lesson-2-implementation.md      📝 Outline
    └── lesson-3-evaluation.md          📝 Outline
```

---

## Page Templates

### Module Page Template (README.md)
```yaml
---
id: module-1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

**Focus**: Middleware for robot control
**Duration**: Weeks 1-5 (5 weeks)
**Hardware Tier**: Tier 1-2 (Cloud/RTX GPU)

## Learning Objectives
- Understand Physical AI principles and embodied intelligence
- Master ROS 2 architecture and core concepts
- Build ROS 2 packages with Python using rclpy
- Understand URDF for humanoid robot descriptions

## Prerequisites
- Basic Python programming
- Linux command line familiarity
- Understanding of basic networking concepts

## Hardware Requirements
| Tier | Equipment | Notes |
|------|-----------|-------|
| Tier 1 | Laptop + Cloud | Use Foxglove for visualization |
| Tier 2 | RTX GPU + Ubuntu | Local ROS 2 installation |

## Chapters

### Chapter 1: Introduction to Physical AI
Foundations of embodied intelligence and the humanoid robotics landscape.

### Chapter 2: ROS 2 Architecture
Core concepts including nodes, topics, services, and actions.

### Chapter 3: Building with ROS 2
Hands-on package development with Python and URDF.
```

### Chapter Page Template (README.md)
```yaml
---
id: chapter-1-physical-ai
title: "Chapter 1: Introduction to Physical AI"
sidebar_position: 1
---

# Chapter 1: Introduction to Physical AI

**Duration**: Weeks 1-2
**Hardware Tier**: Tier 1 (Cloud compatible)

## Learning Objectives
- Define Physical AI and distinguish from digital-only AI
- Identify the key sensor systems used in humanoid robotics
- Understand the humanoid robotics industry landscape

## Prerequisites
- None (this is the first chapter)

## Lessons

| Lesson | Title | Duration | Tier |
|--------|-------|----------|------|
| 1.1 | Foundations of Physical AI | 45 min | 1 |
| 1.2 | From Digital AI to Physical | 30 min | 1 |
| 1.3 | The Humanoid Landscape | 45 min | 1 |
| 1.4 | Sensor Systems | 60 min | 1 |
```

---

## Project Structure

```
D:\piaic-hackathon\humanoid-robotics-handbook\
├── humanoid-textbook/           # Docusaurus frontend
│   ├── docs/
│   │   ├── intro.md             # Landing page
│   │   ├── module-1-ros2/       # Full content (12 lessons)
│   │   ├── module-2-digital-twin/ # Outline only (8 lessons)
│   │   ├── module-3-isaac/      # Outline only (9 lessons)
│   │   └── module-4-vla/        # Outline only (11 lessons)
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatWidget/
│   │   │   └── HardwareSurvey/
│   │   └── css/custom.css
│   └── docusaurus.config.ts
├── rag-backend/                 # FastAPI Backend → Railway
│   ├── main.py                 # FastAPI app entry point
│   ├── requirements.txt        # Python dependencies
│   ├── api/
│   │   ├── health.py           # GET /api/health
│   │   ├── search.py           # POST /api/search
│   │   └── chat.py             # POST /api/chat
│   ├── rag/
│   │   ├── embeddings.py
│   │   ├── ingestion.py
│   │   └── retrieval.py
│   └── db/
│       └── models.py
├── .claude/
│   └── skills/                  # Reusable intelligence
│       ├── lesson-writer.md
│       ├── rag-content-validator.md
│       └── rag-backend-builder.md
├── specs/                       # Spec-Kit-Plus artifacts
└── .specify/memory/
```

---

## Hour-by-Hour Timeline

### PHASE 1: Foundation (Hours 0-2)

**Hour 0-1: Account Setup + Spec-Kit Plus Init**
- [ ] **Create OpenAI API account** at platform.openai.com (for embeddings + chat)
- [ ] **Create Qdrant Cloud account** at cloud.qdrant.io (free tier: 1GB)
- [ ] Initialize Docusaurus: `npx create-docusaurus@latest humanoid-textbook classic --typescript`
- [ ] **Run `/sp.constitution`** - Define project principles:
  - Educational clarity (Bloom's taxonomy)
  - Hardware-aware content (Tier 1-4)
  - RAG-optimized (frontmatter, H2 chunking)
  - Safety-first for motor control
  - 4-Layer Teaching Framework
- [ ] Configure dark mode theme in `docusaurus.config.ts`
- [ ] Create complete folder structure (4 modules, 10 chapters)
- [ ] Set up `.claude/skills/` directory

**Hour 1-2: Create Reusable Skills (50 Bonus Points)**
- [ ] **Skill 1: lesson-writer** - Generates RAG-optimized lessons with frontmatter
- [ ] **Skill 2: rag-content-validator** - Validates content structure for RAG
- [ ] **Skill 3: rag-backend-builder** - Creates FastAPI + Qdrant backend components

**MILESTONE 1**: Project structure ready, constitution defined, 3 skills created

---

### PHASE 2A: Content Stream (Hours 2-14)

**Hours 2-4: Landing Page + Module/Chapter Structure**
- [ ] **Run `/sp.specify`** for landing page feature
- [ ] Hero section: "From ChatGPT to Walking Robots"
- [ ] User journey visualization (Cloud → Miniature → Premium tiers)
- [ ] Hardware decision guide with 4 tiers
- [ ] Module overview cards (all 4 modules)
- [ ] "What You'll Build" showcase
- [ ] FAQ section (RAG-optimized)
- [ ] Create all Module README.md pages (4 modules)
- [ ] Create all Chapter README.md pages (10 chapters)

**Landing Page Sections**:
1. Hero with tagline and CTA
2. "Choose Your Path" - Interactive hardware tier selector
3. Course Stats: 13 weeks, 4 modules, 40 lessons
4. Module Preview Cards
5. "What You'll Build" - Screenshots of Gazebo, ROS nodes, capstone
6. Hardware Requirements by Tier
7. FAQ (SEO + RAG friendly)

**Hours 4-8: Module 1, Chapter 1 - Introduction to Physical AI (4 lessons)**

| Lesson | Title | Layer | Tier | Time |
|--------|-------|-------|------|------|
| 1.1 | Foundations of Physical AI | L1 | 1 | 45m |
| 1.2 | From Digital AI to Physical | L1 | 1 | 30m |
| 1.3 | The Humanoid Landscape | L1 | 1 | 45m |
| 1.4 | Sensor Systems (LIDAR, IMU, Cameras) | L1 | 1 | 60m |

**Hours 8-11: Module 1, Chapter 2 - ROS 2 Architecture (4 lessons)**

| Lesson | Title | Layer | Tier | Time |
|--------|-------|-------|------|------|
| 2.1 | ROS 2 Core Concepts | L1 | 1-2 | 45m |
| 2.2 | Nodes - The Building Blocks | L2 | 2 | 60m |
| 2.3 | Topics and Message Passing | L2 | 2 | 60m |
| 2.4 | Services and Actions | L2 | 2 | 60m |

**Hours 11-14: Module 1, Chapter 3 - Building with ROS 2 (4 lessons)**

| Lesson | Title | Layer | Tier | Time |
|--------|-------|-------|------|------|
| 3.1 | Building ROS 2 Packages with Python | L2 | 2 | 75m |
| 3.2 | Bridging Python Agents with rclpy | L2 | 2 | 75m |
| 3.3 | Launch Files and Parameters | L2 | 2 | 60m |
| 3.4 | Understanding URDF for Humanoids | L2 | 2 | 90m |

**Lesson Frontmatter Template**:
```yaml
---
id: m1-c1-lesson-1-foundations
title: "Lesson 1.1: Foundations of Physical AI"
sidebar_position: 1
sidebar_label: "1.1 Foundations"
description: "Understand embodied intelligence and the paradigm shift from digital to physical AI"
duration_minutes: 45
proficiency_level: "A2"
layer: "L1"
hardware_tier: 1
tier_1_path: "No hardware required - conceptual lesson"
learning_objectives:
  - "Define Physical AI and distinguish from digital-only AI"
  - "Identify the three components of embodied intelligence"
  - "Explain why humanoids excel in human environments"
keywords: ["Physical AI", "embodied intelligence", "humanoid robotics", "ROS 2"]
prerequisites: []
chapter: "Introduction to Physical AI"
module: "The Robotic Nervous System"
---
```

**MILESTONE 2**: Module 1 complete (12 lessons) + all outlines (Hour 14)

---

### PHASE 2B: Backend Stream (Hours 2-12) [PARALLEL]

**Hours 2-4: FastAPI + Railway Setup**
- [ ] Create `rag-backend/` folder with FastAPI structure
- [ ] Install Railway CLI: `npm install -g @railway/cli`
- [ ] Initialize Railway project: `railway login && railway init`
- [ ] Configure Qdrant Cloud cluster (free tier)
- [ ] Connect existing Neon Postgres database
- [ ] Configure OpenAI API for embeddings
- [ ] Set up CORS for GitHub Pages domain

**Hours 4-8: RAG Pipeline**
- [ ] Implement markdown parser with frontmatter extraction
- [ ] Build H2-level chunker (300-500 tokens per chunk)
- [ ] Create embedding pipeline with text-embedding-3-small
- [ ] Implement Qdrant upsert with metadata

**Chunk Schema**:
```json
{
  "chunk_id": "module-1/lesson-1-1#section-2",
  "content": "A node is an independent executable...",
  "title": "Lesson 1.1: From ChatGPT to Walking Robots",
  "section": "What is Physical AI?",
  "module": 1,
  "lesson": "1.1",
  "hardware_tier": 1,
  "layer": "L1",
  "keywords": ["Physical AI", "embodied intelligence"]
}
```

**Hours 8-12: FastAPI Endpoints**

Standard FastAPI application structure:

```python
# rag-backend/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://yourusername.github.io"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/api/health")
async def health():
    return {"status": "ok"}

@app.post("/api/search")
async def search(query: str, hardware_tier: int = 4):
    # Qdrant search with tier filtering
    ...

@app.post("/api/chat")
async def chat(query: str, session_id: str = None):
    # RAG retrieval + OpenAI response
    ...
```

- [ ] `GET /api/health` - Health check endpoint
- [ ] `POST /api/search` - Semantic search with hardware tier filtering
- [ ] `POST /api/chat` - RAG-augmented chat with sources
- [ ] `POST /api/ingest` - Trigger content ingestion (admin only)

**requirements.txt**:
```
fastapi>=0.109.0
uvicorn[standard]>=0.27.0
qdrant-client>=1.7.0
openai>=1.12.0
asyncpg>=0.29.0
python-dotenv>=1.0.0
```

**MILESTONE 3**: Backend fully functional (Hour 12)

---

### PHASE 3: Module 2-4 Outlines (Hours 14-16)

Create outline pages for Modules 2, 3, and 4 (content stubs with learning objectives and prerequisites).

**Module 2: The Digital Twin (Gazebo & Unity)** - Outline only
```
Chapter 1: Gazebo Simulation (Week 6)
  - 1.1 Gazebo Environment Setup
  - 1.2 URDF and SDF Robot Descriptions
  - 1.3 Physics Simulation (Gravity, Collisions)
  - 1.4 Sensor Simulation in Gazebo

Chapter 2: Unity Visualization (Week 7)
  - 2.1 Introduction to Unity for Robotics
  - 2.2 High-Fidelity Rendering
  - 2.3 Human-Robot Interaction in Unity
  - 2.4 Simulating LiDAR, Depth Cameras, IMUs
```

**Module 3: AI-Robot Brain (NVIDIA Isaac)** - Outline only
```
Chapter 1: Isaac Sim Fundamentals (Week 8)
  - 1.1 NVIDIA Isaac SDK Overview
  - 1.2 Photorealistic Simulation
  - 1.3 Synthetic Data Generation

Chapter 2: Isaac ROS & Perception (Week 9)
  - 2.1 Hardware-Accelerated VSLAM
  - 2.2 AI-Powered Perception
  - 2.3 Reinforcement Learning for Control

Chapter 3: Navigation & Deployment (Week 10)
  - 3.1 Nav2 Path Planning for Bipedal Movement
  - 3.2 Sim-to-Real Transfer Techniques
  - 3.3 Deploying to Jetson Edge
```

**Module 4: Vision-Language-Action (VLA)** - Outline only
```
Chapter 1: Humanoid Robot Development (Weeks 11-12)
  - 1.1 Humanoid Kinematics and Dynamics
  - 1.2 Bipedal Locomotion and Balance
  - 1.3 Manipulation and Grasping
  - 1.4 Natural Human-Robot Interaction

Chapter 2: Conversational Robotics (Week 13)
  - 2.1 Voice-to-Action with OpenAI Whisper
  - 2.2 Speech Recognition and NLU
  - 2.3 Multi-Modal Interaction
  - 2.4 Cognitive Planning with LLMs

Chapter 3: Capstone Project
  - 3.1 The Autonomous Humanoid Specification
  - 3.2 Implementation Guide
  - 3.3 Evaluation and Next Steps
```

**Outline Page Format** (for lessons not fully implemented):
```yaml
---
id: m2-c1-lesson-1-gazebo-setup
title: "Lesson 1.1: Gazebo Environment Setup"
sidebar_position: 1
description: "Learn to set up Gazebo simulation environment for robot testing"
duration_minutes: 90
hardware_tier: 2
learning_objectives:
  - "Install and configure Gazebo on Ubuntu"
  - "Load robot models into Gazebo"
  - "Navigate the Gazebo interface"
keywords: ["Gazebo", "simulation", "robot testing"]
prerequisites: ["Module 1 complete"]
status: "outline"
---

# Lesson 1.1: Gazebo Environment Setup

> **Coming Soon**: This lesson is currently in outline form.
> Full content will be available in a future update.

## What You'll Learn
- Installing Gazebo on Ubuntu 22.04
- Understanding the Gazebo interface
- Loading your first robot model

## Prerequisites
- Complete Module 1: The Robotic Nervous System
- Ubuntu 22.04 with ROS 2 Humble installed

## Hardware Requirements
- **Tier 2**: RTX GPU recommended for smooth simulation
- **Tier 1 Alternative**: Use cloud-based Gazebo via AWS RoboMaker
```

---

### PHASE 4: Integration (Hours 16-20)

**Hours 16-18: ChatKit Widget**
- [ ] Create ChatWidget React component
- [ ] Implement chat API connection
- [ ] Add streaming response support
- [ ] Display source citations
- [ ] **Selected text feature**: Ask about highlighted text

**Hours 18-20: Better-Auth + Hardware Survey (STRETCH GOAL - 50 Bonus Points)**

*Given your priority of content depth, this is optional. Only attempt if ahead of schedule at Hour 18.*

- [ ] Configure Better-Auth with Neon Postgres
- [ ] Create signup flow with email/password
- [ ] Build Hardware Survey component:
  - Hardware tier (1-4)
  - ROS experience level
  - Python proficiency
- [ ] Store survey responses in database
- [ ] Filter RAG results by user's hardware tier

**MILESTONE 4**: Chat working (auth if time permits) (Hour 20)

---

### PHASE 5: Polish & Deploy (Hours 20-23)

**Hour 20: Content Ingestion**
- [ ] Run ingestion pipeline on all 12 Module 1 lessons + outlines
- [ ] Verify ~80-100 chunks in Qdrant
- [ ] Test hardware tier filtering

**Hour 21: Testing**
- [ ] Test RAG queries:
  - "What is Physical AI?"
  - "How do ROS 2 nodes communicate?"
  - "What are topics and services?"
  - "I only have a laptop, can I learn?"
- [ ] Test auth flow end-to-end (if implemented)
- [ ] Test chat widget across modules

**Hour 21-22: Deployment**
- [ ] Configure GitHub Actions for Docusaurus
- [ ] Deploy Docusaurus to GitHub Pages
- [ ] Deploy FastAPI to Railway:
  ```bash
  cd rag-backend
  railway up
  ```
- [ ] Configure environment variables in Railway dashboard:
  - `OPENAI_API_KEY`
  - `QDRANT_URL` and `QDRANT_API_KEY`
  - `DATABASE_URL` (Neon connection string)
- [ ] Update ChatWidget with Railway API URL
- [ ] Test CORS between GitHub Pages and Railway

**Hour 22-23: Demo Video (90 seconds)**
```
[0-10s]  Hook: "What if you could teach a robot to walk..."
[10-25s] Book Tour: Landing page, hardware tiers
[25-40s] Content: Module 1 chapter/lesson structure
[40-55s] RAG Demo: Ask question, show sources, select text
[55-70s] Auth + Survey (if implemented): Signup, personalization
[70-85s] Architecture: Docusaurus + FastAPI + Qdrant + Railway
[85-90s] CTA: "Start your Physical AI journey"
```

---

### PHASE 6: Submission (Hours 23-24)

**Final Checklist**:
- [ ] GitHub repo is public
- [ ] All code committed
- [ ] GitHub Pages accessible
- [ ] Backend deployed and responding
- [ ] RAG chatbot functional
- [ ] Auth working (if implemented)
- [ ] Demo video under 90 seconds
- [ ] Skills documented in `.claude/skills/`

**Submit**:
1. GitHub Repo URL
2. Live Book URL
3. Demo Video Link
4. WhatsApp number

---

## Reusable Skills (Bonus Points)

### Skill 1: lesson-writer
```yaml
Purpose: Generate RAG-optimized lessons with proper frontmatter
Persona: Physical AI curriculum designer
Questions:
  - Topic and learning objectives?
  - Hardware tier required (1-4)?
  - Teaching layer (L1-L4)?
  - Prerequisites?
Principles:
  - Complete YAML frontmatter always
  - H2 sections for chunk boundaries
  - Safety warnings for motor control
  - Tier 1 cloud path always available
```

### Skill 2: rag-content-validator
```yaml
Purpose: Validate content structure for RAG optimization
Checks:
  - Frontmatter completeness (15 fields)
  - Section length (300-500 tokens per H2)
  - Consistent terminology
  - Keywords for embeddings
```

### Skill 3: rag-backend-builder
```yaml
Purpose: Create FastAPI + Qdrant backend components
Generates:
  - FastAPI routers with proper typing
  - Qdrant search with metadata filtering
  - RAG chat endpoint with streaming
  - Ingestion pipeline for markdown content
Principles:
  - Standard FastAPI patterns (no serverless hacks)
  - Async/await for all I/O operations
  - Proper CORS configuration
  - Environment variable management
```

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Qdrant rate limits | Pre-batch embeddings, cache queries |
| ChatKit integration issues | Fallback to simple fetch + textarea |
| Running behind schedule | Skip Better-Auth (lose 50 points, save 2 hours) |
| Build failures | Test builds every 2 hours |
| Demo video too long | Script and practice first |

**Emergency Fallback (Hour 16 checkpoint)**:
If behind schedule:
1. Skip Better-Auth (-50 points)
2. Minimal ChatKit styling
3. Deploy with 6 lessons instead of 9
4. Focus on working demo over polish

---

## Color Palette & Theme

**Brand Colors**:
| Name | Hex | Usage |
|------|-----|-------|
| Azure Mist | `#e5f9fa` | Light mode background, cards |
| Pacific Blue | `#12b2c2` | Primary accent, buttons, links |
| Jet Black | `#212a2f` | Dark mode background, text |
| Stormy Teal | `#24727c` | Secondary accent, hover states |
| Pacific Cyan | `#0e8b9f` | Links, interactive elements |

### Dark Mode (Default)
```css
/* src/css/custom.css */
[data-theme='dark'] {
  --ifm-color-primary: #12b2c2;           /* Pacific Blue */
  --ifm-color-primary-dark: #0e8b9f;      /* Pacific Cyan */
  --ifm-color-primary-darker: #24727c;    /* Stormy Teal */
  --ifm-color-primary-darkest: #24727c;
  --ifm-color-primary-light: #12b2c2;
  --ifm-color-primary-lighter: #e5f9fa;   /* Azure Mist */
  --ifm-color-primary-lightest: #e5f9fa;

  --ifm-background-color: #212a2f;        /* Jet Black */
  --ifm-background-surface-color: #2a363d;
  --ifm-navbar-background-color: #212a2f;
  --ifm-footer-background-color: #1a2024;

  --ifm-font-color-base: #e5f9fa;         /* Azure Mist for text */
  --ifm-heading-color: #ffffff;
  --ifm-link-color: #12b2c2;              /* Pacific Blue */
  --ifm-link-hover-color: #0e8b9f;        /* Pacific Cyan */

  --ifm-code-background: #2a363d;
  --ifm-toc-border-color: #24727c;
}
```

### Light Mode
```css
[data-theme='light'] {
  --ifm-color-primary: #0e8b9f;           /* Pacific Cyan */
  --ifm-color-primary-dark: #24727c;      /* Stormy Teal */
  --ifm-color-primary-darker: #24727c;
  --ifm-color-primary-darkest: #212a2f;   /* Jet Black */
  --ifm-color-primary-light: #12b2c2;     /* Pacific Blue */
  --ifm-color-primary-lighter: #12b2c2;
  --ifm-color-primary-lightest: #e5f9fa;  /* Azure Mist */

  --ifm-background-color: #ffffff;
  --ifm-background-surface-color: #e5f9fa; /* Azure Mist */
  --ifm-navbar-background-color: #ffffff;
  --ifm-footer-background-color: #e5f9fa;

  --ifm-font-color-base: #212a2f;         /* Jet Black for text */
  --ifm-heading-color: #212a2f;
  --ifm-link-color: #0e8b9f;              /* Pacific Cyan */
  --ifm-link-hover-color: #24727c;        /* Stormy Teal */

  --ifm-code-background: #e5f9fa;
  --ifm-toc-border-color: #12b2c2;
}
```

### Docusaurus Config Theme Settings
```typescript
// docusaurus.config.ts
themeConfig: {
  colorMode: {
    defaultMode: 'dark',
    disableSwitch: false,
    respectPrefersColorScheme: true,
  },
}
```

---

## Tech Stack Summary

| Component | Technology |
|-----------|------------|
| Frontend | Docusaurus + TypeScript |
| Backend | **FastAPI (Python)** |
| Vector DB | Qdrant Cloud (free tier) |
| Embeddings | OpenAI text-embedding-3-small |
| Database | Neon Serverless Postgres (your existing account) |
| Chat UI | OpenAI ChatKit SDK |
| Auth | Better-Auth (stretch goal) |
| Deployment | **GitHub Pages (book) + Railway (API)** |
| Workflow | Spec-Kit Plus |

---

## Critical Files to Modify/Create

**Spec-Kit Plus**:
1. `.specify/memory/constitution.md` - Project principles and standards

**Docusaurus Frontend**:
2. `humanoid-textbook/docusaurus.config.ts` - Core Docusaurus config
3. `humanoid-textbook/docs/intro.md` - Landing page
4. `humanoid-textbook/docs/module-1-ros2/README.md` - Module 1 overview
5. `humanoid-textbook/docs/module-1-ros2/chapter-1-physical-ai/README.md` - Chapter overview
6. `humanoid-textbook/src/components/ChatWidget/` - Chat component
7. `humanoid-textbook/src/css/custom.css` - Theme colors (see Color Palette below)

**FastAPI Backend**:
8. `rag-backend/main.py` - FastAPI app entry point
9. `rag-backend/api/chat.py` - RAG chat endpoint
10. `rag-backend/api/search.py` - Search endpoint
11. `rag-backend/rag/ingestion.py` - Content ingestion pipeline
12. `rag-backend/requirements.txt` - Python dependencies

**Reusable Skills**:
13. `.claude/skills/lesson-writer.md` - Generates RAG-optimized lessons
14. `.claude/skills/rag-content-validator.md` - Validates content structure
15. `.claude/skills/rag-backend-builder.md` - Creates FastAPI components

---

## Verification Plan

1. **Docusaurus**: `npm run build && npm run serve` locally
2. **RAG Search**: Test 5+ queries, verify sources returned
3. **Chat**: Send message, verify streaming response with citations
4. **Auth**: Complete signup, verify survey data in database
5. **Deployment**: Access GitHub Pages URL from incognito browser
6. **Demo**: Record and verify under 90 seconds

---

## Identified Gaps & Enhancements

### Gaps in Original Plan (Now Addressed)

| Gap | Resolution |
|-----|------------|
| No account setup steps | Added OpenAI + Qdrant account creation in Hour 0-1 |
| Backend deployment unclear | Using Railway for simple FastAPI deployment |
| Auth priority unclear | Marked as stretch goal given content priority |
| Complex serverless patterns | Switched to standard FastAPI (no special handlers) |
| No chapter hierarchy | Added Module → Chapter → Lesson structure based on Weekly Breakdown |
| Missing Spec-Kit Plus | Added `/sp.constitution` in Phase 1, `/sp.specify` before features |
| Module/Chapter pages undefined | Added templates with learning objectives, prerequisites, hardware tiers |
| Incomplete course coverage | Full outline for all 4 modules (40 lessons), implement Module 1 (12 lessons) |

### Enhancement Opportunities for Landing Page

1. **Interactive Hardware Quiz** - "Which path is right for you?" decision tree
2. **Animated Robot Silhouette** - CSS-only hero animation
3. **Progress Tracking** - Module completion indicators
4. **Cost Calculator** - Show cloud vs hardware costs by tier
5. **Testimonials/Social Proof** - Placeholder for future students

### Enhancement Opportunities for RAG

1. **Context Window Expansion** - Include prev/next chunks for better answers
2. **Citation Deep Links** - Link directly to specific lesson sections
3. **Conversation Memory** - Store chat history in Neon for context
4. **Suggested Questions** - Show related questions after each answer

### Additional Skills Worth Creating (If Time Permits)

1. **docusaurus-theme-customizer** - Generates consistent theme changes
2. **rag-test-generator** - Creates test queries from lesson content
3. **fastapi-router-generator** - Scaffolds FastAPI routers with Qdrant integration

---

## Success Criteria

- [ ] Book deployed to GitHub Pages with complete structure:
  - 4 Module overview pages
  - 10 Chapter overview pages
  - 12 full lessons (Module 1)
  - 28 outline lessons (Modules 2-4)
- [ ] RAG chatbot answers questions with source citations
- [ ] Selected text feature works
- [ ] Hardware tier filtering functional
- [ ] 3 reusable skills documented in `.claude/skills/`
- [ ] Spec-Kit Plus constitution defined
- [ ] Auth + survey implemented (stretch goal)
- [ ] Demo video under 90 seconds
- [ ] All submitted before 10:00 PM deadline
