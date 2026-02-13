---
id: 001
title: Create better-auth-setup skill
stage: general
date: 2026-02-13
surface: agent
model: claude-opus-4-6
feature: none
branch: main
user: laiba166-shaikh
command: skill-creator
labels: ["auth", "skill-creation", "fastapi", "jwt", "oauth"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skills/better-auth-setup/SKILL.md
  - .claude/skills/better-auth-setup/references/module-structure.md
  - .claude/skills/better-auth-setup/references/google-oidc.md
  - .claude/skills/better-auth-setup/references/email-flows.md
  - better-auth-setup.skill
tests:
  - Skill validation passed via package_skill.py
---

## Prompt

create better-auth-setup skill uing skill-creator to have login and signup user option with JWT token and OIDC/O Auth 2.1. There should be validation there for email and password. Ask me any question related to auth integration in the project.

## Response snapshot

Created the `better-auth-setup` skill using the skill-creator workflow. Gathered user requirements via targeted questions (framework: FastAPI, provider: Google only, storage: PostgreSQL, validation: standard, JWT: access+refresh in httpOnly cookies, output: complete module, extra flows: email verification + password reset). Built SKILL.md with a 6-step workflow and 3 reference files covering module structure, Google OIDC/OAuth 2.1, and email flows. Packaged successfully as `better-auth-setup.skill`.

## Outcome

- ✅ Impact: New skill created and packaged — ready for use with "add auth", "setup authentication", etc.
- 🧪 Tests: Skill validation passed (frontmatter, structure, description quality)
- 📁 Files: 4 skill files + 1 .skill package
- 🔁 Next prompts: Use the skill to scaffold auth in a project, iterate on patterns
- 🧠 Reflection: Skill-creator workflow with init_skill.py has a Windows encoding issue (emoji in print) — workaround: PYTHONIOENCODING=utf-8 + python -X utf8

## Evaluation notes (flywheel)

- Failure modes observed: init_skill.py fails on Windows cp1252 console due to emoji chars
- Graders run and results (PASS/FAIL): package_skill.py validation PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test the skill end-to-end on a real FastAPI project
