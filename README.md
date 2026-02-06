# Physical AI & Humanoid Robotics Handbook

An AI-native technical textbook teaching Physical AI and Humanoid Robotics — from ChatGPT to walking robots. Built with Docusaurus, powered by a RAG chatbot, and developed using Spec-Kit Plus.

## Overview

This project is a unified textbook + chatbot system for a 13-week course on Physical AI and Humanoid Robotics. Readers learn to design, simulate, and deploy humanoid robots using ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models.

**Live Book**: [GitHub Pages](https://yourusername.github.io/humanoid-robotics-handbook/) (coming soon)

## Tech Stack

| Component | Technology |
|-----------|------------|
| Textbook Frontend | Docusaurus v3 + TypeScript |
| Styling | Infima + Custom CSS |
| Backend API | FastAPI (Python 3.10+) |
| Vector Database | Qdrant Cloud |
| Embeddings | OpenAI text-embedding-3-small |
| User Database | Neon Serverless Postgres |
| Chat UI | OpenAI ChatKit SDK |
| Auth | Better-Auth (stretch goal) |
| Book Hosting | GitHub Pages |
| API Hosting | Railway |
| Dev Workflow | Spec-Kit Plus + Claude Code |

## Course Modules

| Module | Title | Status |
|--------|-------|--------|
| 1 | **The Robotic Nervous System (ROS 2)** | Full content (12 lessons) |
| 2 | The Digital Twin (Gazebo & Unity) | Outline |
| 3 | The AI-Robot Brain (NVIDIA Isaac) | Outline |
| 4 | Vision-Language-Action (VLA) | Outline |

### Module 1 Chapters (Implemented)
- **Chapter 1**: Introduction to Physical AI — Foundations, landscape, hardware tiers, sensor systems
- **Chapter 2**: ROS 2 Architecture — Core concepts, nodes, topics, services, actions
- **Chapter 3**: Building with ROS 2 — Python packages, rclpy, launch files, URDF

### Hardware Tiers

Content is tagged by hardware requirements so every reader has a path:

| Tier | Equipment | Cost |
|------|-----------|------|
| Tier 1 | Laptop + Cloud | Free |
| Tier 2 | RTX GPU + Ubuntu | ~$1,500 |
| Tier 3 | Jetson Edge Kit | ~$700 |
| Tier 4 | Physical Robot (Unitree G1) | ~$16,000 |

Every lesson works for Tier 1 with cloud-based alternatives.

## Project Structure

```
humanoid-robotics-handbook/
├── humanoid-textbook/           # Docusaurus frontend
│   ├── docs/                    # All textbook content
│   │   ├── intro.md             # Landing page
│   │   ├── module-1-ros2/       # Full content (3 chapters, 12 lessons)
│   │   ├── module-2-digital-twin/  # Outline
│   │   ├── module-3-isaac/      # Outline
│   │   └── module-4-vla/        # Outline
│   ├── src/
│   │   ├── components/          # React components (ChatWidget, etc.)
│   │   └── css/custom.css       # Theme with brand color palette
│   └── docusaurus.config.ts
├── rag-backend/                 # FastAPI backend (Railway)
│   ├── main.py
│   ├── api/                     # Endpoints (health, search, chat)
│   ├── rag/                     # Embeddings, ingestion, retrieval
│   └── requirements.txt
├── .claude/
│   ├── commands/                # Spec-Kit Plus slash commands
│   └── skills/                  # Reusable Claude Code skills
├── .specify/
│   ├── memory/constitution.md   # Project principles and standards
│   ├── templates/               # SDD templates
│   └── scripts/                 # Automation scripts
├── specs/                       # Feature specifications
├── history/                     # Prompt History Records & ADRs
├── plan.md                      # Execution plan
├── requirements.md              # Hackathon requirements
└── CLAUDE.md                    # Agent instructions
```

## RAG Chatbot

The integrated chatbot answers questions about textbook content using Retrieval-Augmented Generation:

- **Semantic search** across all lessons via Qdrant vector database
- **Source citations** in format `[Module X, Chapter Y, Lesson Z]`
- **Hardware-tier filtering** — answers respect the reader's equipment level
- **Selected text** — highlight text in the book and ask questions about it

### API Endpoints

```
GET  /api/health    → Health check
POST /api/search    → Semantic search with tier filtering
POST /api/chat      → RAG-augmented chat with citations
POST /api/ingest    → Content ingestion (admin)
```

## Claude Code Skills

This project includes reusable skills for Claude Code:

| Skill | Purpose |
|-------|---------|
| **skill-creator** | Guide for creating effective Claude Code skills |
| **skill-validator** | Validate skills against production-level quality criteria |

## Getting Started

### Prerequisites

- Node.js 18+
- Python 3.10+
- Git

### Run the Textbook Locally

```bash
cd humanoid-textbook
npm install
npm run start
```

The book opens at `http://localhost:3000`.

### Run the Backend Locally

```bash
cd rag-backend
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows
pip install -r requirements.txt
uvicorn main:app --reload
```

API available at `http://localhost:8000`.

### Environment Variables

Create a `.env` file in `rag-backend/`:

```bash
OPENAI_API_KEY=sk-...
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
```

## Color Palette

| Name | Hex | Usage |
|------|-----|-------|
| Azure Mist | `#e5f9fa` | Light backgrounds |
| Pacific Blue | `#12b2c2` | Primary accent, buttons |
| Jet Black | `#212a2f` | Dark mode background |
| Stormy Teal | `#24727c` | Secondary accent, hover |
| Pacific Cyan | `#0e8b9f` | Links, interactive elements |

## Development Workflow

This project uses **Spec-Driven Development** with Spec-Kit Plus:

```
/sp.specify  → Define what to build
/sp.plan     → Design the approach
/sp.tasks    → Break into checklist
/sp.implement → Execute with Claude Code
```

Every interaction is recorded as a **Prompt History Record (PHR)** under `history/prompts/`.

## License

This project is created for the PIAIC Hackathon I. All rights reserved.

---

Built with Claude Code and Spec-Kit Plus.
