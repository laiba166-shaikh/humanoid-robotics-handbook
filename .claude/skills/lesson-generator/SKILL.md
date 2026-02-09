---
name: lesson-generator
description: Generate complete, RAG-optimized lessons for technical textbooks with structured frontmatter, chunked sections, and validated content quality. Use when the user asks to write a lesson, create lesson content, generate a new lesson file, or draft textbook material. Produces production-ready .md files with YAML frontmatter, Bloom's taxonomy objectives, hardware tier paths, code examples, and self-assessment questions. Works with any Docusaurus-based or markdown-based educational project that uses RAG retrieval.
---

# Lesson Generator

Generate production-ready textbook lessons optimized for RAG retrieval, educational clarity, and consistent quality.

## Workflow

### 1. Gather Context

Before writing, determine these inputs. Ask the user if not provided:

- **Module and chapter** placement
- **Lesson number and title**
- **Topic scope** — what the lesson covers
- **Hardware/environment tier** (1-4) — equipment level required
- **Teaching layer** (L1-L4) — manual foundation through spec-driven
- **Proficiency level** — A2 (beginner), B1 (intermediate), C2 (advanced)
- **Duration** in minutes
- **Prerequisites** — what the student needs first

Also read the project's constitution or style guide if one exists (e.g., `.specify/memory/constitution.md`) to pick up project-specific terminology, glossary, color palette, and naming conventions.

### 2. Generate Frontmatter

Read [references/frontmatter-schema.md](references/frontmatter-schema.md) for the full YAML schema.

Key rules:
- All fields are mandatory — no field may be omitted
- `learning_objectives` must use Bloom's taxonomy verbs
- `tier_1_path` is always required — every lesson must have a cloud/browser fallback
- `keywords` must include 4-8 terms for semantic search

### 3. Write Lesson Body

Read [references/lesson-template.md](references/lesson-template.md) for the mandatory skeleton and examples.

Section order:
1. Title + metadata block (duration, tier, layer)
2. Learning Objectives — Bloom's verbs + measurable outcomes
3. The Hook — 2-3 sentences connecting to a real-world problem
4. Core Sections — H2 headers, 300-500 tokens each, self-contained for RAG chunking
5. Key Takeaways — 3-5 bullet points (primary RAG retrieval targets)
6. Check Your Understanding — 3-5 self-assessment questions
7. Next Steps — 1-2 sentences pointing to the next lesson

### 4. Apply Writing Standards

Read [references/writing-standards.md](references/writing-standards.md) for the full language rules.

Non-negotiable rules:
- 15-20 words per sentence (max 30)
- 3-5 sentences per paragraph
- Active voice always
- Define before use — bold term on first use with definition
- Concrete before abstract — example/analogy first, then theory
- No banned phrases: "simply", "just", "obviously", "easy", "trivial", "as everyone knows"

### 5. Write Code Examples

Read [references/code-patterns.md](references/code-patterns.md) for the four-part code block pattern.

Every code block: context sentence, code (max 40 lines with all imports), expected output, line-by-line explanation.

### 6. Validate

Read [references/quality-checklist.md](references/quality-checklist.md) and verify every item passes before delivering.

### 7. Save File

Write to the project's docs directory following the naming convention:
`docs/{module-folder}/{chapter-folder}/lesson-{n}-{slug}.md`

## Quick Reference

| Element | Constraint |
|---------|-----------|
| Sentence length | 15-20 words (max 30) |
| Paragraph length | 3-5 sentences |
| H2 section size | 300-500 tokens |
| Code block length | Max 40 lines |
| Keywords per lesson | 4-8 |
| Learning objectives | Bloom's verbs only |
| Tier 1 fallback | Always required |
