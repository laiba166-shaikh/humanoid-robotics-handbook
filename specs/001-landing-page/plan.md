# Implementation Plan: Textbook Landing Page

**Branch**: `001-landing-page` | **Date**: 2026-02-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-landing-page/spec.md`

## Summary

Replace the default Docusaurus homepage with a professional 7-section landing page for the Physical AI & Humanoid Robotics textbook. The core technical change is reconfiguring the docs route base path from `/` to `/docs` to free the root URL for a custom React page. Reusable UI primitives (Card, TierBadge, FAQAccordion, StatItem, NumberedOutcome) will be extracted into `src/components/landing/` for maintainability and consistency. All seven sections (Hero, Stats Bar, Why Physical AI, Module Preview, Choose Your Path, Learning Outcomes, FAQ) will be built as inline React components in a single page file (`index.tsx`) leveraging these primitives, with CSS Modules for scoped styling. Existing theme classes from `custom.css` are reused for consistency.

## Technical Context

**Language/Version**: TypeScript (Docusaurus v3 / React 18)
**Primary Dependencies**: Docusaurus v3, React 18, Infima CSS framework, existing `custom.css`
**Storage**: N/A (static content, no data persistence)
**Testing**: Manual visual testing + `npm run build` for link validation
**Target Platform**: Web (GitHub Pages — static site)
**Project Type**: Web application (frontend only for this feature)
**Performance Goals**: Static page — instant render, no API calls
**Constraints**: No new external dependencies; reuse existing CSS classes; no Tailwind; palette-only colors
**Scale/Scope**: 1 page, 7 sections, ~400 lines TSX, ~300 lines CSS (including new components)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Color palette compliance | PASS | All colors from constitution palette via CSS variables |
| No new dependencies | PASS | Uses only React (bundled with Docusaurus) and existing CSS |
| Docusaurus conventions | PASS | `.md` for docs, React for pages, CSS Modules for styles |
| Accessibility basics | PASS | Semantic headings, native `<details>/<summary>`, keyboard-navigable, contrast-safe |
| Tech stack compliance | PASS | Docusaurus v3 + TypeScript + Infima — no Tailwind or external CSS |
| Dark/light mode support | PASS | All colors via `var(--ifm-*)` CSS variables |

**Post-design re-check**: All gates still pass. No violations.

## Project Structure

### Documentation (this feature)

```text
specs/001-landing-page/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: research findings
├── quickstart.md        # Phase 1: developer quickstart
├── checklists/
│   └── requirements.md  # Spec quality checklist
├── content.md           # Approved page copy (content spec)
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
humanoid-textbook/
├── docusaurus.config.ts          # MODIFY: routeBasePath '/' → '/docs', footer links
├── docs/
│   └── intro.md                  # MODIFY: replace default tutorial with textbook welcome
├── src/
│   ├── pages/
│   │   ├── index.tsx             # REWRITE: 7 section components leveraging primitives
│   │   └── index.module.css      # REWRITE: ~300 lines scoped styles for index.tsx
│   ├── components/
│   │   ├── HomepageFeatures/     # DELETE: unused after rewrite
│   │   └── landing/              # NEW: Reusable UI primitives for landing page
│   │       ├── Card/
│   │       │   ├── Card.tsx
│   │       │   └── Card.module.css
│   │       ├── StatItem/
│   │       │   ├── StatItem.tsx
│   │       │   └── StatItem.module.css
│   │       ├── TierBadge/
│   │       │   ├── TierBadge.tsx
│   │       │   └── TierBadge.module.css
│   │       ├── FAQAccordion/
│   │       │   ├── FAQAccordion.tsx
│   │       │   └── FAQAccordion.module.css
│   │       └── NumberedOutcome/
│   │           ├── NumberedOutcome.tsx
│   │           └── NumberedOutcome.module.css
│   └── css/
│       └── custom.css            # NO CHANGE: reuse existing classes
```

**Structure Decision**: Frontend-only change within the existing Docusaurus project. New `src/components/landing/` directory created for reusable UI primitives. All other modifications are to existing files or deletion of unused components.

## Design Decisions

### D1: Modular components for reusable UI primitives

**Decision**: Extract reusable UI elements (e.g., Card, StatItem, TierBadge, FAQAccordion, NumberedOutcome) into individual components within `src/components/landing/`. The 7 main landing page sections will be defined as inline functions in `index.tsx`, composed from these primitives.

**Rationale**: This balances the need for code reuse and maintainability for common UI patterns (cards, badges) with the desire for a single `index.tsx` file for overall page composition. It avoids unnecessary file overhead for unique section components while promoting consistency for repeated elements.

**Alternative rejected**: Fully inline all UI elements within `index.tsx` — would lead to duplication and harder maintenance for consistent elements like cards and badges. Separate files for each of the 7 sections was also rejected for its overhead for single-use sections.

### D2: CSS Modules vs. global CSS additions

**Decision**: Use CSS Modules (`index.module.css` and `.module.css` for new primitive components) for all landing page styles.

**Rationale**: Scoped by default — no risk of style leaks into docs pages. Global `custom.css` classes (`.hero`, `.card`, `.tier-badge--*`, `.button--primary`) are applied alongside module classes via `clsx()` where appropriate.

**Alternative rejected**: Adding all styles to `custom.css` — would pollute global scope and make cleanup harder.

### D3: Static data arrays vs. external JSON/config

**Decision**: Define module data, tier data, FAQ data, and outcomes as typed `const` arrays directly in `index.tsx`.

**Rationale**: Content is fixed, small, and tightly coupled to rendering. External files add fetch complexity for zero benefit on a static site.

### D4: Native `<details>/<summary>` for FAQ

**Decision**: Use native HTML elements instead of a React accordion component, encapsulated within the `FAQAccordion` primitive component.

**Rationale**: Zero JavaScript required for basic functionality, accessible by default (keyboard + screen reader), supported in all modern browsers. Aligns with constitution's accessibility requirements. Encapsulating it allows for consistent styling and a custom toggle indicator.

### D5: Route base path change strategy

**Decision**: Change `routeBasePath` from `'/'` to `'/docs'` in `docusaurus.config.ts`.

**Rationale**: This is the documented Docusaurus approach for freeing the root URL. All internal links (footer, navbar) need updating to include the `/docs` prefix. The `build` command will catch any broken links.

## Implementation Approach

### Phase 1: Configuration (prerequisite for all other work)

1. **Modify `docusaurus.config.ts`**:
   - Line 35: `routeBasePath: '/'` → `routeBasePath: '/docs'`
   - Line 85: footer link `to: '/'` → `to: '/docs'`
   - Line 89: footer link `to: '/module-1-ros2'` → `to: '/docs/module-1-ros2'`

2. **Update `docs/intro.md`**:
   - Replace default Docusaurus tutorial with textbook welcome content
   - Keep `sidebar_position: 1` frontmatter

3. **Delete `src/components/HomepageFeatures/`**:
   - Remove `index.tsx` and `styles.module.css`
   - These are imported by the current `index.tsx` — must delete before rewrite

### Phase 2: Reusable UI Primitives (`src/components/landing/`)

Create the following new components, each in its own subdirectory within `src/components/landing/` (e.g., `src/components/landing/Card/Card.tsx` and `src/components/landing/Card/Card.module.css`):

1. **Card/Card.tsx / Card/Card.module.css**: General-purpose card component, reusing global `.card` styles and adding specific padding/flex/shadow/responsive styles.
2. **StatItem/StatItem.tsx / StatItem/StatItem.module.css**: For displaying numbers and labels in the Stats Bar. Responsive styling for 2x2 grid on mobile.
3. **TierBadge/TierBadge.tsx / TierBadge/TierBadge.module.css**: For displaying hardware tier badges. Reuses global `.tier-badge--*` classes.
4. **FAQAccordion/FAQAccordion.tsx / FAQAccordion/FAQAccordion.module.css**: Encapsulates `<details>/<summary>` with custom styling for toggle indicator and content.
5. **NumberedOutcome/NumberedOutcome.tsx / NumberedOutcome/NumberedOutcome.module.css**: For displaying numbered learning outcomes with CSS-only circles.

### Phase 3: Landing Page Sections (`index.tsx`)

Build all 7 sections as functions in a single file, composing them from the new primitives and existing Docusaurus components:

1. **Data definitions** — typed arrays for modules, tiers, outcomes, FAQ, stats
2. **HeroSection** — gradient background, eyebrow text, h1, subtitle, two CTAs (using `button--primary` and `button button--secondary`)
3. **StatsBar** — composed of `StatItem` components
4. **WhyPhysicalAI** — section heading + 3 value cards composed of `Card` components
5. **ModulePreview** — `id="modules"` anchor, 2x2 grid of cards, Module 1 clickable, 2-4 dimmed
6. **ChooseYourPath** — 4 tier cards composed of `Card` components, `TierBadge` usage, Tier 1 highlighted
7. **LearningOutcomes** — 6 items composed of `NumberedOutcome` components
8. **FAQSection** — 6 `FAQAccordion` components, single-column centered
9. **Home** — Layout wrapper composing all sections

### Phase 4: Styles (`index.module.css`)

~300 lines organized by section for `index.tsx`:

- **Section rhythm**: Alternating backgrounds (default / surface-color)
- **Hero**: min-height 80vh, gradient reuse, flex centering
- **Stats**: horizontal flex → 2x2 grid below 768px (using `StatItem.module.css` for item styling)
- **Cards**: 3-column → stacked on mobile, 2x2 → single column on mobile (using `Card.module.css` for item styling)
- **Tiers**: 4-column → 2x2 → single column (using `Card.module.css` for item styling)
- **Outcomes**: 2-column → single column (using `NumberedOutcome.module.css` for item styling)
- **FAQ**: max-width 800px centered (using `FAQAccordion.module.css` for item styling)
- **Responsive breakpoints**: 996px (tablet), 768px (mobile)

### Phase 5: Verification

1. `npm run build` — zero broken links
2. `npm run start` — visual check of all 7 sections at `localhost:3000`
3. Dark/light toggle — all sections adapt
4. Mobile resize — responsive layouts work
5. CTA navigation — "Start Learning" → `/docs/module-1-ros2`
6. Smooth scroll — "Explore Modules" → `#modules`
7. Module 1 card → navigates to module page
8. FAQ — accordions open/close with keyboard

## Content Source

All text content (headings, descriptions, statistics, FAQ answers, module data, tier descriptions, learning outcomes) is defined in the companion content spec at `specs/001-landing-page/content.md`. The implementation must match this content exactly.

## Complexity Tracking

No constitution violations detected. No complexity justifications needed.
