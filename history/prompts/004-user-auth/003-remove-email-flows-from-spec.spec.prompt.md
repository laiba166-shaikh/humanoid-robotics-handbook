---
id: 003
title: Remove email flows from spec
stage: spec
date: 2026-02-13
surface: agent
model: claude-opus-4-6
feature: 004-user-auth
branch: 004-user-auth
user: laiba166-shaikh
command: spec update
labels: ["auth", "spec-update", "email-verification", "password-reset", "scope-reduction"]
links:
  spec: specs/004-user-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-user-auth/spec.md
  - specs/004-user-auth/checklists/requirements.md
tests:
  - Checklist re-validated: all items PASS
---

## Prompt

update the @specs/004-user-auth\ spec files and remove the verify email and reset password from there. Also mention to use better-auth-setup skill to add the auth flow where it usable. update the checklists as well if needed

## Response snapshot

Updated spec.md to remove all email verification and password reset references. Changes: (1) User Story 1 — removed verification email flow and acceptance scenarios 4-5, signup now logs in automatically. (2) Removed User Story 5 (Password Reset) entirely. (3) Removed FR-003 (verification email) and FR-010 (password reset), renumbered FR-004 through FR-010. (4) Removed Verification Token entity. (5) Removed is_verified from User entity. (6) Removed SMTP assumption. (7) Replaced SC-007/SC-008 with refresh token rotation criterion. (8) Added explicit out-of-scope note for email verification and password reset. (9) Added better-auth-setup skill reference in Assumptions. (10) Updated checklist with change notes.

## Outcome

- ✅ Impact: Spec reduced to 4 user stories, 10 FRs, 7 SCs — focused on core auth
- 🧪 Tests: Checklist re-validated, all 16 items PASS
- 📁 Files: spec.md + checklists/requirements.md updated
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Clean removal — FR numbering renumbered sequentially, no orphan references

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Checklist PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
