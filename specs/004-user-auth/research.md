# Research: User Authentication

**Feature**: 004-user-auth | **Date**: 2026-02-13

## R1: Cross-Origin Token Delivery

**Decision**: Tokens delivered in JSON response body; access token in React state, refresh token in localStorage.

**Rationale**: Frontend (GitHub Pages) and backend (Railway) are on unrelated domains. httpOnly cookies cannot be shared cross-origin without same-site domain setup, which is not feasible with GitHub Pages. Response body tokens with `Authorization: Bearer` header is the standard pattern for SPAs on separate domains.

**Alternatives considered**:
- httpOnly cookies with proxy — Rejected: GitHub Pages doesn't support server-side proxying
- Same parent domain — Rejected: GitHub Pages domain (`*.github.io`) can't share cookies with Railway domain
- BFF (Backend-for-Frontend) — Rejected: Over-engineering for this scope; adds an entire server layer

**Security note**: localStorage is vulnerable to XSS. Mitigated by: (1) Docusaurus renders static content with minimal user-generated input, (2) refresh tokens are rotated on each use, (3) access tokens are short-lived (15 min) and stored only in memory.

---

## R2: Auth Pages in Docusaurus

**Decision**: Custom React pages under `src/pages/auth/` within the existing Docusaurus site.

**Rationale**: Docusaurus v3 supports [custom pages](https://docusaurus.io/docs/creating-pages) as React components in `src/pages/`. This gives full React functionality (state, effects, API calls) while inheriting the site layout, navbar, footer, and theme (dark/light mode). No separate app to deploy.

**Alternatives considered**:
- Separate React app — Rejected: Extra deployment, theme drift, maintenance burden
- Modal/overlay — Rejected: Doesn't support complex flows (Google redirect, form validation) cleanly

---

## R3: Navbar Auth State

**Decision**: Custom navbar component registered via Docusaurus swizzling or `clientModules` config.

**Rationale**: Docusaurus supports custom navbar items via the theme component system. We can either:
1. **Swizzle** `NavbarItem/ComponentTypes` to add a custom type
2. Use a **client module** that injects auth state globally

Option 1 (swizzle) is cleaner and officially supported.

**Alternatives considered**:
- Inject DOM directly — Rejected: Fragile, breaks with Docusaurus updates
- Use `themeConfig.navbar.items` with HTML type — Rejected: Limited, no React state access

---

## R4: Google OAuth 2.1 with PKCE (Cross-Origin)

**Decision**: Full-page redirect flow with backend-initiated PKCE. Backend callback redirects to frontend with tokens in URL fragment.

**Rationale**: With separate domains, the OAuth callback must land on the backend (which holds the client secret). After token exchange, the backend redirects to the frontend with tokens. Using URL fragment (`#access_token=...`) instead of query params prevents tokens from being logged in server access logs.

**Alternatives considered**:
- Popup window flow — Rejected: Blocked by many browsers, poor mobile UX
- Frontend-only OIDC (implicit flow) — Rejected: Deprecated in OAuth 2.1; requires exposing client secret

---

## R5: Database Migration Strategy

**Decision**: Alembic for schema migrations with async SQLAlchemy.

**Rationale**: The project already uses SQLAlchemy models. Alembic is the standard migration tool for SQLAlchemy, supports async engines, and integrates with Neon Postgres. Migrations are version-controlled and reversible.

**Alternatives considered**:
- Raw SQL migrations — Rejected: No version tracking, error-prone
- Prisma — Rejected: Wrong ecosystem (Node.js, not Python)

---

## R6: Rate Limiting Implementation

**Decision**: In-memory per-IP counter with automatic TTL cleanup.

**Rationale**: At the current scale (single backend instance, low traffic), an in-memory dict with timestamps is sufficient. No need for Redis or external rate limiting service. The counter tracks failed login attempts per IP and resets after 60 seconds.

**Alternatives considered**:
- Redis-based rate limiting — Rejected: Over-engineering; adds infrastructure dependency for a simple counter
- `slowapi` library — Viable alternative, but adds a dependency for a single endpoint. Can be introduced later if needed.

---

## R7: Auth UI Theme Compliance

**Decision**: CSS modules using brand CSS variables from constitution palette. Infima base styles extended for forms.

**Rationale**: Constitution mandates using ONLY the 5 brand colors and Infima + Custom CSS (no Tailwind). Auth forms will use:
- `--ifm-color-primary` (mapped to Pacific Blue) for buttons and focus states
- `--color-stormy-teal` for hover states and secondary actions
- `--color-azure-mist` for light mode backgrounds
- `--color-jet-black` for dark mode backgrounds
- `--color-pacific-cyan` for links

Existing component pattern (`.module.css` files) matches other components in `src/components/landing/`.

**Alternatives considered**:
- Tailwind CSS — Rejected: Constitution explicitly forbids it
- Inline styles — Rejected: Not maintainable, no dark mode support
