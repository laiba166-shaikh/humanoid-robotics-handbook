---
id: 006
title: Apply Cross-Artifact Analysis Remediations
stage: tasks
date: 2026-02-06
surface: agent
model: claude-opus-4-6
feature: 001-landing-page
branch: 001-landing-page
user: architect
command: remediation follow-up to /sp.analyze
labels: ["remediation", "analysis", "content-spec", "palette", "consistency"]
links:
  spec: specs/001-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-landing-page/content.md (created)
  - specs/001-landing-page/spec.md (modified)
  - specs/001-landing-page/plan.md (modified)
  - specs/001-landing-page/tasks.md (modified)
  - humanoid-textbook/src/css/custom.css (modified)
tests:
  - none (artifact edits only)
---

## Prompt

yes, suggest remediation edits for the top 6 issues

1. refer the @landing-page.md file and pick each section content from there for the placeholder and create content.md file. it is approved from my side.
2. My Decision -> Replace them with palette-derived alternatives (e.g., all tiers use Pacific Blue/Stormy Teal shading)
4. the footer "Introduction" link point to /docs (the welcome page)

I agree with the 3, 5, 6 remediation

## Response snapshot

Applied all 6 remediations from the cross-artifact analysis:

1. **C3 (CRITICAL)**: Created `specs/001-landing-page/content.md` with all 7 sections of approved copy extracted from `landing-page.md`. This resolves SC-008 verification baseline.
2. **K1 (CRITICAL)**: Replaced non-palette tier badge colors in `custom.css` with palette-derived alternatives (Pacific Blue → Pacific Cyan → Stormy Teal → Jet Black gradient). Added palette enforcement acceptance criterion to T031 and assumption to spec.md.
3. **A1 (HIGH)**: Documented exact reusable CSS classes in spec.md assumptions. Noted missing `.button--outline` class — secondary CTA must use Infima `button--secondary` or CSS Module.
4. **C2 (HIGH)**: Expanded T002 to include footer label rename ('Introduction' → 'Documentation') and navbar auto-resolve verification.
5. **F1 (HIGH)**: Fixed plan.md content source path from `specs/landing-page/landing-page.md` to `specs/001-landing-page/content.md`.
6. **F2 (HIGH)**: Fixed parallel execution IDs in tasks.md (was T017-T020, corrected to T022-T025, T030).

## Outcome

- ✅ Impact: Resolved 2 CRITICAL and 4 HIGH issues; artifacts now consistent and implementation-ready
- 🧪 Tests: N/A (artifact edits, no code logic changes except CSS palette fix)
- 📁 Files: 1 created, 4 modified
- 🔁 Next prompts: /sp.implement to begin Phase 1 configuration tasks
- 🧠 Reflection: Having the content spec as a separate approved file decouples content review from implementation review — good separation of concerns

## Evaluation notes (flywheel)

- Failure modes observed: None — all edits applied cleanly
- Graders run and results (PASS/FAIL): Manual verification — PASS
- Prompt variant (if applicable): remediation follow-up
- Next experiment: Consider automating palette compliance checks as a build-time lint step
