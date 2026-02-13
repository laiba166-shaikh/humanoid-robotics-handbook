---
id: 002
title: Remove email flows from auth skill
stage: misc
date: 2026-02-13
surface: agent
model: claude-opus-4-6
feature: 004-user-auth
branch: 004-user-auth
user: laiba166-shaikh
command: skill update
labels: ["auth", "skill-update", "email-verification", "password-reset"]
links:
  spec: specs/004-user-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skills/better-auth-setup/SKILL.md
  - .claude/skills/better-auth-setup/references/module-structure.md
  - better-auth-setup.skill
tests:
  - Skill validation passed via package_skill.py
---

## Prompt

update the @better-auth-setup.skill and remove verify email and password reset flow for now.

## Response snapshot

Updated the better-auth-setup skill to remove email verification and password reset flows. Changes: (1) SKILL.md — removed email verification/password reset from description, feature list, file tree, endpoints, env vars (SMTP section), and references. (2) module-structure.md — removed VerificationToken model, removed is_verified field from User model, removed VerifyEmailRequest/ForgotPasswordRequest/ResetPasswordRequest schemas, removed send_verification_email call from signup endpoint, removed email_service.py reference from router. (3) Deleted references/email-flows.md entirely. Repackaged as better-auth-setup.skill (3 files instead of 4).

## Outcome

- ✅ Impact: Skill simplified to core auth only (signup, login, refresh, logout, Google OIDC, /me)
- 🧪 Tests: Skill validation PASS
- 📁 Files: 2 files updated, 1 file deleted, .skill repackaged
- 🔁 Next prompts: /sp.plan or /sp.clarify to continue feature work
- 🧠 Reflection: Clean removal — no orphan references left

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): package_skill.py validation PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
