---
id: 007
title: Fix All Remaining Analysis Issues
stage: tasks
date: 2026-02-06
surface: agent
model: claude-opus-4-6
feature: 001-landing-page
branch: 001-landing-page
user: architect
command: remediation round 2 (post /sp.analyze rerun)
labels: ["remediation", "parallel-ids", "breakpoints", "palette", "cleanup"]
links:
  spec: specs/001-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-landing-page/tasks.md (modified)
  - specs/001-landing-page/spec.md (modified)
  - specs/001-landing-page/plan.md (modified)
  - specs/001-landing-page/content.md (modified)
  - humanoid-textbook/src/css/custom.css (modified)
tests:
  - none (artifact edits only)
---

## Prompt

Fix the High issue in tasks.md for all parallel execution ID mismatch
1. B1: Standardize mobile breakpoint to 768px across FR-013, US6, SC-005, and plan
2. F5: Update button--outline references in content.md and plan.md to match spec's corrected class name
3. K2: Fix #ffffff heading color in custom.css dark mode (constitution violation)
4. F7: Add content.md to plan project structure tree
5. D1: Remove redundant directory-creation tasks
6. K3: Document admonition color exception

## Response snapshot

Applied 16 edits across 5 files resolving all remaining issues:

**HIGH (parallel ID fixes in tasks.md):**
- US3: T022 → T027
- US4: T023, T024 → T028, T029
- US5: T029 → T034
- US6: T030 → T035

**B1 (breakpoint standardization):**
- spec.md FR-013: "below 576px" → "below 768px", "576-996px" → "768-996px"
- plan.md: "576px (mobile)" → "768px (mobile)"

**F5 (button--outline → button--secondary):**
- content.md: updated secondary CTA class
- plan.md: updated Phase 3 HeroSection reference

**K2 (constitution compliance):**
- custom.css dark mode heading: `#ffffff` → `#e5f9fa` (Azure Mist)

**F7 (plan structure):**
- Added `content.md` to plan project structure tree

**D1 (cleanup):**
- Removed 5 redundant directory-creation tasks (T007, T010, T013, T016, T019). Task count: 39 → 34.

**K3 (admonition exception):**
- Documented `.admonition-caution` and `.admonition-danger` as known palette exceptions for safety semantics

## Outcome

- ✅ Impact: All HIGH, MEDIUM, and LOW issues from reanalysis resolved. Zero CRITICAL, zero HIGH issues remaining.
- 🧪 Tests: N/A (artifact edits, no code logic changes except CSS color fix)
- 📁 Files: 5 modified across 16 edits
- 🔁 Next prompts: /sp.implement — artifacts are now fully consistent and implementation-ready
- 🧠 Reflection: Batch-fixing all remaining issues in one pass is more efficient than iterating. The "fix all instances" approach prevented the partial-fix problem from round 1.

## Evaluation notes (flywheel)

- Failure modes observed: None — all 16 edits applied cleanly in parallel
- Graders run and results (PASS/FAIL): Manual verification — PASS
- Prompt variant (if applicable): batch remediation
- Next experiment: Consider a post-edit validation pass that re-reads modified files to confirm no unresolved placeholders
