# Frontmatter Schema

Every lesson requires complete YAML frontmatter for RAG indexing, sidebar rendering, and content filtering.

## Required Fields

```yaml
---
id: lesson-{n}-{slug}              # Unique ID (e.g., lesson-1-foundations)
title: "Lesson X.Y: Full Title"     # Display title with lesson number
sidebar_position: {n}               # Order in sidebar (matches lesson number)
sidebar_label: "X.Y Short Label"    # Abbreviated sidebar text
description: "SEO/search summary"   # 1 sentence, under 160 chars
duration_minutes: 45                 # Estimated completion time
proficiency_level: "A2"             # A2 (beginner), B1 (intermediate), C2 (advanced)
layer: "L1"                         # L1 (manual) / L2 (AI collab) / L3 (co-design) / L4 (spec-driven)
hardware_tier: 1                    # Primary tier required (1-4)
tier_1_path: "Cloud alternative"    # ALWAYS required — how Tier 1 students follow along
learning_objectives:
  - "Bloom's verb + measurable outcome"
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

## Field Rules

| Field | Constraint |
|-------|-----------|
| `id` | Lowercase, hyphenated, unique across project |
| `title` | Include "Lesson X.Y:" prefix for consistency |
| `description` | Under 160 characters, no markdown |
| `duration_minutes` | Integer, realistic estimate |
| `proficiency_level` | One of: A2, B1, C2 |
| `layer` | One of: L1, L2, L3, L4 |
| `hardware_tier` | Integer 1-4 |
| `tier_1_path` | Non-empty string describing cloud/browser fallback |
| `learning_objectives` | 2-5 items, each starting with a Bloom's verb |
| `keywords` | 4-8 terms for semantic search |
| `prerequisites` | List of prior lessons or skills; use "None" for first lessons |

## Bloom's Taxonomy Verbs

Use these verbs to start every learning objective:

| Level | Verbs |
|-------|-------|
| Remember | Define, List, Identify, Name, Recall |
| Understand | Explain, Describe, Summarize, Interpret, Classify |
| Apply | Build, Implement, Execute, Demonstrate, Use |
| Analyze | Compare, Differentiate, Examine, Debug, Test |
| Evaluate | Assess, Justify, Critique, Validate, Recommend |
| Create | Design, Construct, Develop, Compose, Integrate |

## Teaching Layers

| Layer | When to Use | AI Role |
|-------|-------------|---------|
| L1 (Manual Foundation) | New concepts, safety-critical | Minimal |
| L2 (AI Collaboration) | After foundation built | Active partner |
| L3 (Intelligence Design) | Recurring patterns | Co-designer |
| L4 (Spec-Driven) | Capstone projects | Full orchestrator |

**Rule**: Students master manual basics (L1) before AI collaboration (L2+).

## Hardware Tiers

| Tier | Equipment | Description |
|------|-----------|-------------|
| 1 | Laptop + Cloud | No GPU required, browser-based |
| 2 | RTX GPU + Ubuntu | Local development environment |
| 3 | Edge Device + Sensors | Real sensors, edge deployment |
| 4 | Physical Robot | Actual robot hardware |

**Rule**: Every lesson MUST work for Tier 1 with a cloud/browser fallback path.
