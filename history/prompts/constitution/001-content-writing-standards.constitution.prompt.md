---
id: 001
title: "Add Content Writing Standards to Constitution"
stage: constitution
date: 2026-02-06
surface: agent
model: claude-opus-4-6
feature: none
branch: main
user: retrocausal
command: direct-prompt
labels: ["constitution", "content-standards", "technical-writing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

I reviewed the constitution.md and feel that we should define the criteria for our content core principles. It should be align with the potential stake holders of this book. The lessons should be of high quality and easy to grab. The language should be technical but easy to understand. Think as a senior technical writer and define the core principle and practices should be followed for the content

## Response snapshot

Added a comprehensive **Content Writing Standards** section to `constitution.md` with:

1. **Stakeholder Alignment Table** — Maps 6 stakeholder types (CS students, career-switchers, bootcamp participants, ESL readers, instructors, hackathon judges) to their needs and how content serves them.

2. **5 Core Content Principles**:
   - Clarity Is Non-Negotiable (first-read comprehension, simplest correct word)
   - Concrete Before Abstract (example/analogy before theory)
   - Respect the Reader's Time (no padding, scannable formatting)
   - Progressive Disclosure (80% case first, edge cases deferred)
   - Every Concept Earns Its Place (answer "why does this matter?" within 2 paragraphs)

3. **Language Rules**: Sentence length limits (15-20 words, max 30), banned phrases table ("simply", "just", "obviously", "easy"), technical terminology protocol, active voice mandate.

4. **Lesson Body Structure**: Standardized skeleton — Hook → Objectives → Core Sections → Takeaways → Questions → Next Steps.

5. **Code Example Pattern**: Four-part — Context → Code → Output → Explanation. Max 40 lines, filename hints, meaningful variable names.

6. **Section Header Rules**: Descriptive and keyword-rich for RAG retrieval (no clever titles).

7. **Engagement Patterns**: When to use `:::tip`, `:::note`, `:::warning`, `:::danger`.

8. **Content Quality Checklist**: 17-point checklist across Clarity, Accuracy, Structure, and Accessibility.

## Outcome

- ✅ Impact: Constitution now has actionable, enforceable content writing standards aligned to 6 stakeholder types
- 🧪 Tests: Manual review — no unresolved placeholders, all sections have examples
- 📁 Files: `.specify/memory/constitution.md` (modified — added ~200 lines)
- 🔁 Next prompts: Create Module 1 lesson files following these standards, review plan.md alignment
- 🧠 Reflection: The original Voice & Tone section was too vague (4 bullets). The new standards give concrete rules with bad/good examples that any writer or AI agent can follow consistently.

## Evaluation notes (flywheel)

- Failure modes observed: Previous lessons (now lost) didn't follow a consistent structure. Standards were implicit.
- Graders run and results (PASS/FAIL): PASS — all placeholders resolved, content verified against constitution
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Apply these standards to lesson-1-digital-to-physical.md and validate against the quality checklist
