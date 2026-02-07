# Feature Specification: Textbook Landing Page

**Feature Branch**: `001-landing-page`
**Created**: 2026-02-06
**Status**: Draft
**Input**: User description: "Build the landing page for my humanoid physical AI textbook. All sections should follow the design and theme consistently. The landing page has 7 sections: Hero, Stats Bar, Why Physical AI (3 value cards), Module Preview Cards (2x2 grid), Choose Your Path (hardware tiers), Learning Outcomes, and FAQ. Must fix routeBasePath from '/' to '/docs' to free root URL. Reuse existing custom.css classes."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - First-time visitor evaluates the course (Priority: P1)

A prospective learner lands on the homepage for the first time. They see a compelling hero section that communicates the course value proposition — "From ChatGPT to Walking Robots." They scan the stats bar (13 weeks, 4 modules, 40 lessons, 4 hardware tiers) to gauge course scope. They scroll through the "Why Physical AI?" cards to understand industry relevance. Convinced, they click "Start Learning" to begin Module 1.

**Why this priority**: The hero and value proposition are the first impression. If visitors don't understand what the course offers within seconds, they leave. This is the primary conversion path.

**Independent Test**: Can be fully tested by loading the homepage at `/` and verifying that the hero renders with correct heading, subtitle, and CTA buttons. Clicking "Start Learning" navigates to `/docs/module-1-ros2`.

**Acceptance Scenarios**:

1. **Given** a visitor loads the root URL, **When** the page renders, **Then** they see the hero section with the heading "From ChatGPT to Walking Robots", a subtitle describing the 13-week course, and two call-to-action buttons
2. **Given** a visitor is on the hero section, **When** they click "Start Learning", **Then** they navigate to the first module's documentation page
3. **Given** a visitor scrolls past the hero, **When** they reach the stats bar, **Then** they see four statistics (13 Weeks, 4 Modules, 40 Lessons, 4 Hardware Tiers) displayed horizontally

---

### User Story 2 - Visitor explores course structure and modules (Priority: P1)

A visitor wants to understand what the course covers before committing. They click "Explore Modules" in the hero or scroll down to the module preview section. They see four module cards in a grid: Module 1 (available, clickable), Modules 2-4 (coming soon, visually dimmed). Each card shows the module title, focus area, week range, and applicable hardware tiers. They click Module 1 to browse its content.

**Why this priority**: Module visibility is essential for course evaluation. Visitors need to see the full curriculum structure to make an enrollment decision.

**Independent Test**: Can be tested by scrolling to the modules section and verifying all four module cards render with correct titles, tier badges, and status indicators. Module 1 card links to `/docs/module-1-ros2`. Modules 2-4 are visually dimmed and not clickable.

**Acceptance Scenarios**:

1. **Given** a visitor is on the hero section, **When** they click "Explore Modules", **Then** the page scrolls smoothly to the module preview section
2. **Given** a visitor views the module cards, **When** they see Module 1, **Then** it displays "The Robotic Nervous System", "Weeks 1-5", Tier 1-2 badges, and an "Available" status indicator
3. **Given** a visitor views the module cards, **When** they see Modules 2-4, **Then** each displays a "Coming Soon" status indicator and appears visually dimmed
4. **Given** a visitor clicks on the Module 1 card, **When** navigation completes, **Then** they arrive at the Module 1 documentation page

---

### User Story 3 - Visitor assesses hardware requirements (Priority: P2)

A visitor is interested but worried about equipment costs. They scroll to the "Choose Your Path" section and see four hardware tiers. Tier 1 (Cloud Explorer) is visually highlighted as the recommended starting point — free, any laptop with a browser. They feel reassured that they can start immediately without buying hardware. Tiers 2-4 show progressive investment levels for deeper engagement.

**Why this priority**: Hardware anxiety is the top barrier to entry for robotics courses. Clearly communicating that the course works with zero investment removes the primary objection.

**Independent Test**: Can be tested by scrolling to the hardware tiers section and verifying all four tier cards render with correct equipment, cost, and description. Tier 1 has a visual highlight and "Recommended Start" badge.

**Acceptance Scenarios**:

1. **Given** a visitor views the hardware tiers section, **When** they see Tier 1 (Cloud Explorer), **Then** it shows "$0" cost, "Any laptop with a browser" equipment, and a "Recommended Start" badge
2. **Given** a visitor views all four tiers, **When** they compare them, **Then** each tier displays its name, equipment requirements, cost range, and a description of what they can accomplish
3. **Given** a visitor views the tiers section, **When** Tier 1 renders, **Then** it is visually distinguished from the other tiers with a highlighted border

---

### User Story 4 - Visitor checks prerequisites and FAQ (Priority: P2)

A visitor scrolls to the FAQ section to check if the course is right for them. They expand "What programming experience do I need?" and learn that Python knowledge is sufficient. They expand "Can I use macOS or Windows?" and learn about cloud-based alternatives. The accordion interaction is smooth and accessible via keyboard.

**Why this priority**: FAQs reduce support burden and help visitors self-qualify. Accessible accordion interactions demonstrate polish and professionalism.

**Independent Test**: Can be tested by scrolling to the FAQ section, clicking each question summary, and verifying the answer expands/collapses. All six Q&A pairs render with correct content.

**Acceptance Scenarios**:

1. **Given** a visitor views the FAQ section, **When** they click on a question, **Then** the answer expands below the question
2. **Given** a visitor has expanded an FAQ answer, **When** they click the same question again, **Then** the answer collapses
3. **Given** a visitor navigates the FAQ with keyboard, **When** they press Enter on a focused question, **Then** the answer toggles open/closed
4. **Given** a visitor views the FAQ, **When** they count the questions, **Then** there are exactly six Q&A pairs

---

### User Story 5 - Visitor views page in dark and light modes (Priority: P3)

A visitor toggles between dark and light modes using the theme switcher. All seven landing page sections adapt correctly — text remains readable, backgrounds alternate between page and surface colors, and the hero gradient stays consistent. No elements break or become invisible in either mode.

**Why this priority**: Theme consistency is a quality signal. Broken dark/light mode undermines trust in a technical course.

**Independent Test**: Can be tested by toggling the theme switch and visually verifying each section in both modes. All text is readable, all backgrounds use theme variables.

**Acceptance Scenarios**:

1. **Given** a visitor is viewing the landing page in dark mode, **When** they toggle to light mode, **Then** all sections adapt colors correctly with readable text and appropriate backgrounds
2. **Given** a visitor is viewing in either mode, **When** they scroll through all sections, **Then** alternating sections use distinct background colors for visual rhythm

---

### User Story 6 - Visitor views page on mobile device (Priority: P3)

A visitor accesses the landing page on a phone or tablet. The layout adapts: hero text scales down, stats arrange in a 2x2 grid, value cards stack vertically, module cards stack to single column, hardware tier cards stack vertically, learning outcomes stack to single column, and FAQ remains single column. All content is readable and tappable.

**Why this priority**: Mobile visitors represent a significant portion of traffic. Responsive design ensures the course is accessible to everyone.

**Independent Test**: Can be tested by resizing the browser below 768px width and verifying all sections reflow correctly. All text is readable, all interactive elements are tappable.

**Acceptance Scenarios**:

1. **Given** a visitor views the page on a screen narrower than 768px, **When** they see the stats bar, **Then** the four statistics arrange in a 2x2 grid instead of a horizontal row
2. **Given** a visitor views the page on a small screen, **When** they scroll through module cards, **Then** each card occupies the full width and stacks vertically
3. **Given** a visitor views the hero on mobile, **When** the heading renders, **Then** the text size is reduced for readability on small screens

---

### Edge Cases

- What happens when a visitor clicks a "Coming Soon" module card? Nothing — the card is not interactive and is visually dimmed to signal unavailability.
- What happens when JavaScript is disabled? FAQ accordions still work because they use native `<details>/<summary>` elements. All content remains visible and readable.
- What happens when the page is accessed at `/docs` directly? The documentation welcome page loads correctly, independent of the landing page.
- What happens when a visitor uses a screen reader? All sections use semantic headings (h2/h3), the FAQ uses native `<details>/<summary>` for accessibility, and CTA buttons have descriptive text.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The root URL (`/`) MUST display the custom landing page with all seven sections in order: Hero, Stats Bar, Why Physical AI, Module Preview, Choose Your Path, Learning Outcomes, FAQ
- **FR-002**: The documentation content MUST be accessible at `/docs` instead of the root URL, requiring a configuration change to the docs route base path
- **FR-003**: The hero section MUST display the course title "From ChatGPT to Walking Robots", a descriptive subtitle, and two call-to-action buttons ("Start Learning" linking to the first module, "Explore Modules" scrolling to the modules section)
- **FR-004**: The stats bar MUST display four course statistics: 13 Weeks, 4 Modules, 40 Lessons, 4 Hardware Tiers
- **FR-005**: The "Why Physical AI?" section MUST display three value proposition cards with titles and descriptions covering market opportunity, software-hardware convergence, and hands-on learning
- **FR-006**: The module preview section MUST display four module cards showing title, focus area, week range, hardware tier badges, and availability status
- **FR-007**: Module 1 card MUST be clickable and link to its documentation page; Modules 2-4 MUST appear visually dimmed with "Coming Soon" status and MUST NOT be clickable
- **FR-008**: The "Choose Your Path" section MUST display four hardware tier cards with equipment requirements, cost, and description; Tier 1 MUST be visually highlighted as the recommended starting point
- **FR-009**: The learning outcomes section MUST display six numbered outcomes describing skills learners will acquire
- **FR-010**: The FAQ section MUST display six questions with expandable answers using accessible accordion interactions that work without JavaScript
- **FR-011**: All navigation links in the footer MUST use the updated `/docs` path prefix
- **FR-012**: The landing page MUST render correctly in both dark and light theme modes using the established color palette
- **FR-013**: The landing page MUST be responsive, adapting layouts for desktop (above 996px), tablet (768-996px), and mobile (below 768px) viewports
- **FR-014**: The unused default homepage component MUST be removed to prevent confusion
- **FR-015**: The documentation welcome page at `/docs` MUST display textbook-specific content replacing the default tutorial placeholder

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The landing page loads and displays all seven sections at the root URL without errors
- **SC-002**: A first-time visitor can navigate from the landing page to Module 1 content within two clicks (via "Start Learning" CTA or Module 1 card)
- **SC-003**: The "Explore Modules" button scrolls smoothly to the module preview section
- **SC-004**: All seven sections render correctly in both dark and light theme modes with readable text and appropriate contrast
- **SC-005**: All seven sections adapt to mobile viewports (below 768px) without horizontal scrolling or text overflow
- **SC-006**: FAQ accordions open and close via mouse click and keyboard interaction (Enter key)
- **SC-007**: The site builds without broken link errors after the route base path change
- **SC-008**: Page content matches the approved copy from the content specification (all headings, descriptions, statistics, and FAQ answers are accurate)

## Assumptions

- The existing `src/css/custom.css` provides these reusable classes that MUST NOT be modified: `.hero` (gradient background), `.hero__title`, `.hero__subtitle`, `.card` (with hover transform), `.tier-badge` (with `--1` through `--4` modifiers), `.button--primary`. No `.button--outline` class exists; the secondary CTA must use Infima's built-in `button button--secondary` or a CSS Module class
- CSS color enforcement: All new landing page styles MUST use only CSS variables (e.g., `var(--ifm-color-primary)`, `var(--ifm-background-surface-color)`). No raw hex values in new CSS files
- The site framework supports route base path configuration for separating docs from the landing page
- Dark/light mode switching is handled by the framework's built-in theme toggle using CSS variables
- The landing page does not require any external data fetching — all content is static and defined inline
- Module availability status is hardcoded: Module 1 is "Available", Modules 2-4 are "Coming Soon"
- The FAQ uses native HTML `<details>/<summary>` elements, requiring zero additional JavaScript

## Scope

### In Scope

- Seven-section landing page with all specified content
- Route configuration change to separate docs from root URL
- Footer link updates to use new docs path prefix
- Documentation welcome page replacement
- Removal of unused default homepage component
- Responsive design for desktop, tablet, and mobile
- Dark/light mode compatibility

### Out of Scope

- Analytics or tracking integration
- Animation libraries or complex transitions
- Backend API integration
- User authentication or personalization
- A/B testing of landing page variants
- SEO meta tags beyond framework defaults
- Chat widget integration (separate feature)
