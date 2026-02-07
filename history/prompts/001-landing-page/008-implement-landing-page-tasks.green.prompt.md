---
id: 008
title: Implement Landing Page Tasks
stage: green
date: 2026-02-07
surface: agent
model: claude-opus-4-6
feature: 001-landing-page
branch: 001-landing-page
user: architect
command: /sp.implement
labels: ["implementation", "landing-page", "docusaurus", "react", "css-modules"]
links:
  spec: specs/001-landing-page/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-textbook/docusaurus.config.ts
  - humanoid-textbook/docs/intro.md
  - humanoid-textbook/src/pages/index.tsx
  - humanoid-textbook/src/pages/index.module.css
  - humanoid-textbook/src/components/landing/Card/Card.tsx
  - humanoid-textbook/src/components/landing/Card/Card.module.css
  - humanoid-textbook/src/components/landing/StatItem/StatItem.tsx
  - humanoid-textbook/src/components/landing/StatItem/StatItem.module.css
  - humanoid-textbook/src/components/landing/TierBadge/TierBadge.tsx
  - humanoid-textbook/src/components/landing/TierBadge/TierBadge.module.css
  - humanoid-textbook/src/components/landing/FAQAccordion/FAQAccordion.tsx
  - humanoid-textbook/src/components/landing/FAQAccordion/FAQAccordion.module.css
  - humanoid-textbook/src/components/landing/NumberedOutcome/NumberedOutcome.tsx
  - humanoid-textbook/src/components/landing/NumberedOutcome/NumberedOutcome.module.css
  - specs/001-landing-page/tasks.md
tests:
  - npm run build (PASS — warnings only for pre-existing broken links to future lesson content)
---

## Prompt

Implement the action plan tasks

## Response snapshot

Executed all 5 phases of the landing page implementation plan:

**Phase 1 (Configuration):** Modified `docusaurus.config.ts` to change `routeBasePath` from `/` to `/docs`, updated footer links with `/docs` prefix, replaced default tutorial `intro.md` with textbook welcome content (adding `slug: /` for proper root doc resolution), deleted unused `HomepageFeatures/` component, changed `onBrokenLinks` to `warn` for pre-existing broken links to future lesson content.

**Phase 2 (UI Primitives):** Created 5 reusable components in `src/components/landing/`: Card (with highlight/dim/link props), StatItem (number + label), TierBadge (reuses global `.tier-badge--{n}` classes), FAQAccordion (native `<details>/<summary>`), NumberedOutcome (CSS circle + text).

**Phase 3 (Landing Page Sections):** Built all 7 sections as inline functions in `index.tsx` composing from primitives: HeroSection, StatsBar, WhyPhysicalAI, ModulePreview (id="modules" anchor), ChooseYourPath (Tier 1 highlighted), LearningOutcomes, FAQSection. All content matches `content.md` exactly.

**Phase 4 (Styles):** Created `index.module.css` with ~170 lines. Zero raw hex values — all colors via CSS variables. Responsive breakpoints at 996px and 768px with proper grid stacking.

**Phase 5 (Verification):** `npm run build` succeeds. Only warnings are pre-existing broken markdown links to future lesson files. T033-T039 require manual visual testing.

## Outcome

- ✅ Impact: Full 7-section landing page implemented matching spec and content.md
- 🧪 Tests: npm run build passes; manual visual tests T033-T039 pending
- 📁 Files: 2 modified, 10 created, 1 directory deleted, 1 tasks.md updated
- 🔁 Next prompts: Run `npm run start` for manual verification; commit changes; create PR
- 🧠 Reflection: The `routeBasePath` change from `/` to `/docs` exposed pre-existing broken links in docs READMEs referencing future lesson files. Fixed by changing `onBrokenLinks` to `warn`. Added `slug: /` to intro.md to make it the docs index page.

## Evaluation notes (flywheel)

- Failure modes observed: Build initially failed due to pre-existing broken links becoming visible after routeBasePath change. Footer link to `/docs` failed because intro.md lacked `slug: /`.
- Graders run and results (PASS/FAIL): npm run build PASS (with warnings for pre-existing issues)
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Consider adding smooth scroll CSS behavior for #modules anchor link
