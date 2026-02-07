# Tasks for Feature: Textbook Landing Page

**Branch**: `001-landing-page` | **Date**: 2026-02-06 | **Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Summary

This document outlines the tasks required to implement the Textbook Landing Page feature according to the specification and plan. Tasks are organized into phases, with user story-specific tasks grouped under their respective stories. Dependencies and parallelization opportunities are identified.

## Dependencies

- **Phase 1: Configuration** must be completed before any other phases.
- **Phase 2: Reusable UI Primitives** must be completed before **Phase 3: Landing Page Sections**.
- User stories are largely independent for parallel execution, especially for UI component creation.

## Parallel Execution Examples

### User Story 1 - First-time visitor evaluates the course (P1)

- Tasks T022, T023, T024, T025, T030 can be executed in parallel (after Phase 2 primitives complete).

### User Story 2 - Visitor explores course structure and modules (P1)

- Task T026 can be executed in parallel with US1 tasks (after Phase 2 primitives complete).

### User Story 3 - Visitor assesses hardware requirements (P2)

- Task T027 can be executed in parallel (after Phase 2 primitives complete).

### User Story 4 - Visitor checks prerequisites and FAQ (P2)

- Tasks T028, T029 can be executed in parallel (after Phase 2 primitives complete).

### User Story 5 - Visitor views page in dark and light modes (P3)

- Task T034 depends on all UI components being built and styled correctly.

### User Story 6 - Visitor views page on mobile device (P3)

- Task T035 depends on all UI components being built and styled correctly.

## Implementation Strategy

The strategy focuses on delivering a Minimum Viable Product (MVP) first, covering the critical P1 user stories. Development will proceed incrementally, completing one user story at a time after foundational setup.

**MVP Scope**: Phase 1 (Configuration), Phase 2 (Reusable UI Primitives), Phase 3 (HeroSection, StatsBar, WhyPhysicalAI, ModulePreview from User Stories 1 & 2), and Phase 4 (initial `index.module.css`).

## Task Checklist

### Phase 1: Configuration (Prerequisite)

- [x] T001 Modify `routeBasePath` to `/docs` in `docusaurus.config.ts` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\docusaurus.config.ts)
- [x] T002 Update footer link `to: '/'` to `to: '/docs'` in `docusaurus.config.ts` (line 85). Update label from 'Introduction' to 'Documentation'. Verify navbar 'Start Learning' (type: docSidebar, lines 62-68) auto-resolves correctly after routeBasePath change (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\docusaurus.config.ts)
- [x] T003 Update footer link `to: '/module-1-ros2'` to `to: '/docs/module-1-ros2'` in `docusaurus.config.ts` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\docusaurus.config.ts)
- [x] T004 Update `docs/intro.md` with textbook welcome content (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\docs\intro.md)
- [x] T005 Delete `src/components/HomepageFeatures/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\HomepageFeatures\index.tsx)
- [x] T006 Delete `src/components/HomepageFeatures/styles.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\HomepageFeatures\styles.module.css)

### Phase 2: Reusable UI Primitives (`src/components/landing/`)

- [x] T008 [P] Create `src/components/landing/Card/Card.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\Card\Card.tsx)
- [x] T009 [P] Create `src/components/landing/Card/Card.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\Card\Card.module.css)
- [x] T011 [P] Create `src/components/landing/StatItem/StatItem.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\StatItem\StatItem.tsx)
- [x] T012 [P] Create `src/components/landing/StatItem/StatItem.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\StatItem\StatItem.module.css)
- [x] T014 [P] Create `src/components/landing/TierBadge/TierBadge.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\TierBadge\TierBadge.tsx)
- [x] T015 [P] Create `src/components/landing/TierBadge/TierBadge.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\TierBadge\TierBadge.module.css)
- [x] T017 [P] Create `src/components/landing/FAQAccordion/FAQAccordion.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\FAQAccordion\FAQAccordion.tsx)
- [x] T018 [P] Create `src/components/landing/FAQAccordion/FAQAccordion.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\FAQAccordion\FAQAccordion.module.css)
- [x] T020 [P] Create `src/components/landing/NumberedOutcome/NumberedOutcome.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\NumberedOutcome\NumberedOutcome.tsx)
- [x] T021 [P] Create `src/components/landing/NumberedOutcome/NumberedOutcome.module.css` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\components\landing\NumberedOutcome\NumberedOutcome.module.css)

### Phase 3: Landing Page Sections (`index.tsx`)

- [x] T022 [P] [US1] Define static data arrays (modules, tiers, outcomes, FAQ, stats) in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T023 [P] [US1] Implement `HeroSection` in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T024 [P] [US1] Implement `StatsBar` composed of `StatItem` components in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T025 [P] [US1] Implement `WhyPhysicalAI` section with `Card` components in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T026 [P] [US2] Implement `ModulePreview` section with `Card` components and `TierBadge` in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T027 [P] [US3] Implement `ChooseYourPath` section with `Card` components and `TierBadge` in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T028 [P] [US4] Implement `LearningOutcomes` section with `NumberedOutcome` components in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T029 [P] [US4] Implement `FAQSection` with `FAQAccordion` components in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)
- [x] T030 [P] [US1] Implement `Home` layout wrapper composing all sections in `src/pages/index.tsx` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.tsx)

### Phase 4: Styles (`index.module.css`)

- [x] T031 [P] Create/update `src/pages/index.module.css` with overall layout and section-specific styles. Acceptance: zero raw hex values; all colors via `var(--ifm-*)` or `var(--color-*)` CSS variables only (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\src\pages\index.module.css)

### Phase 5: Verification

- [x] T032 Run `npm run build` from `humanoid-textbook/` directory to verify no broken links (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\)
- [ ] T033 Run `npm run start` from `humanoid-textbook/` directory and visually verify all 7 sections at `localhost:3000` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T034 Verify dark/light mode toggling (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T035 Verify responsive layouts on mobile (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T036 Verify "Start Learning" CTA navigates to `/docs/module-1-ros2` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T037 Verify "Explore Modules" scrolls smoothly to `#modules` (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T038 Verify Module 1 card navigates to module page (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
- [ ] T039 Verify FAQ accordions open/close with keyboard (D:\piaic-hackathon\humanoid-robotics-handbook\humanoid-textbook\) — MANUAL
