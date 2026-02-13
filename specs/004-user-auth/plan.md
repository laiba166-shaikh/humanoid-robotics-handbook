# Implementation Plan: User Authentication

**Branch**: `004-user-auth` | **Date**: 2026-02-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/004-user-auth/spec.md`

## Summary

Add user authentication to the Physical AI & Humanoid Robotics Handbook with email/password signup and login, Google OAuth 2.1 social login, and a Login link in the Docusaurus navbar. The backend (FastAPI) issues JWT access + refresh tokens via JSON response bodies. The frontend (Docusaurus custom pages) stores the access token in React state and the refresh token in localStorage, sending the access token via `Authorization: Bearer` header. User data persists in Neon Postgres via async SQLAlchemy. The auth UI follows the existing brand theme (Infima + custom CSS, Pacific Blue/Stormy Teal palette) with full dark/light mode support.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript 5.6 (frontend, Docusaurus 3.9.2, React 19)
**Primary Dependencies**: FastAPI, SQLAlchemy[asyncio], asyncpg, python-jose[cryptography], passlib[bcrypt], pydantic[email-validator], httpx (backend); Docusaurus, React 19, clsx (frontend)
**Storage**: Neon Postgres (serverless) via `asyncpg`
**Testing**: pytest (backend API endpoints)
**Target Platform**: Web — Railway (backend), GitHub Pages (frontend)
**Project Type**: Web (frontend + backend, separate domains)
**Performance Goals**: Login < 5s, Signup < 60s, Navbar state < 1s (from spec SCs)
**Constraints**: Cross-origin (separate domains) — tokens in response body, no httpOnly cookies for auth. Per-IP rate limiting (5 failures/min). MUST use brand color palette only.
**Scale/Scope**: Single-user auth, no admin panel, no user management UI

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Tech Stack compliance | ✅ PASS | FastAPI, Neon Postgres, Docusaurus — all in constitution |
| Styling: Infima + Custom CSS only | ✅ PASS | No Tailwind. CSS modules with brand variables |
| Color palette: 5 colors only | ✅ PASS | Pacific Blue buttons, Stormy Teal hover, Azure Mist backgrounds, Jet Black dark mode, Pacific Cyan links |
| API response format | ✅ PASS | Standard JSON `{success, data, error, meta}` for all auth endpoints |
| Error codes | ✅ PASS | Using VALIDATION_ERROR, RATE_LIMITED, etc. per constitution |
| API prefix `/api/` | ✅ PASS | Auth endpoints under `/api/auth/` matching convention |
| CORS origins | ✅ PASS | Allow github.io (prod) + localhost:3000 (dev) |
| No secrets in code | ✅ PASS | All via `.env` / env vars |
| Auth constitution entry | ⚠️ NOTE | Constitution says "Better-Auth (Stretch goal)" — building custom JWT auth instead. Acceptable: Better-Auth was labeled stretch, and custom auth gives full control for this cross-origin setup |

**Gate Result**: PASS — no blocking violations.

## Project Structure

### Documentation (this feature)

```text
specs/004-user-auth/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── auth-api.md      # OpenAPI-style contract for all auth endpoints
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
rag-backend/
├── app/
│   ├── auth/                    # NEW — Auth module
│   │   ├── __init__.py
│   │   ├── models.py            # User + RefreshToken SQLAlchemy models
│   │   ├── schemas.py           # Pydantic request/response schemas
│   │   ├── validators.py        # Email & password validation
│   │   ├── security.py          # JWT create/verify, password hash/verify
│   │   ├── dependencies.py      # get_current_user FastAPI dependency
│   │   ├── router.py            # /signup, /login, /refresh, /logout, /me
│   │   └── google_oidc.py       # Google OAuth 2.1 authorization code flow
│   ├── database.py              # NEW — Async SQLAlchemy engine, session, Base
│   ├── main.py                  # MODIFY — Add auth router, SessionMiddleware
│   └── config.py                # MODIFY — Add auth env vars
├── alembic/                     # NEW — DB migrations
│   └── versions/
├── alembic.ini                  # NEW — Migration config
└── requirements.txt             # MODIFY — Add new dependencies

humanoid-textbook/
├── src/
│   ├── components/
│   │   └── auth/                # NEW — Auth UI components
│   │       ├── AuthProvider.tsx  # React Context for auth state
│   │       ├── GoogleLoginButton.tsx  # "Continue with Google" button
│   │       ├── NavbarAuthItem.tsx     # Navbar login/user display
│   │       ├── NavbarAuthItem.module.css  # Navbar auth styles
│   │       └── auth.module.css  # Shared auth styles (brand theme)
│   ├── pages/
│   │   └── auth/                # NEW — Auth pages
│   │       ├── login.tsx        # Login page
│   │       ├── signup.tsx       # Signup page
│   │       └── callback.tsx     # Google OAuth callback handler
│   ├── services/
│   │   └── authApi.ts           # NEW — Auth API client (fetch wrapper)
│   └── pages/
│   │   └── auth/                # NEW — Auth pages
│   │       ├── login.tsx        # Login page
│   │       ├── signup.tsx       # Signup page
│   │       └── callback.tsx     # Google OAuth callback handler
└── docusaurus.config.ts         # MODIFY — Add Login navbar item
```

**Structure Decision**: Web application (Option 2) — existing `rag-backend/` and `humanoid-textbook/` directories. Auth module added to backend under `app/auth/`. Frontend auth components under `src/components/auth/`, pages under `src/pages/auth/`.

## Architecture Decisions

### 1. Token Strategy (Cross-Origin)

Since frontend and backend are on separate domains:
- **Access token**: Stored in React state (in-memory). Short-lived (15 min). Sent via `Authorization: Bearer` header.
- **Refresh token**: Stored in `localStorage`. Long-lived (7 days). Sent to `/api/auth/refresh` endpoint only.
- **On page refresh**: `AuthProvider` checks localStorage for refresh token → calls `/api/auth/refresh` → gets new access token → restores auth state.
- **On logout**: Clear both tokens, revoke refresh token server-side.

### 2. Google OAuth Flow (Cross-Origin)

Since the backend can't set cookies on the frontend domain:
1. Frontend opens `{BACKEND_URL}/api/auth/google/login` (full-page redirect or popup)
2. Backend redirects to Google consent screen
3. Google redirects to backend callback (`/api/auth/google/callback`)
4. Backend creates/finds user, generates tokens
5. Backend redirects to `{FRONTEND_URL}/auth/callback#access_token=...&refresh_token=...`
6. Frontend `callback.tsx` page extracts tokens from URL params, stores them, redirects to `/docs/intro`

### 3. Auth State Management

- **React Context** (`AuthProvider`) wraps the entire Docusaurus app via root decorator (Docusaurus `swizzle` or `clientModules`)
- Provides: `user`, `isAuthenticated`, `login()`, `signup()`, `logout()`, `googleLogin()`
- No external state library needed (React 19 Context + useReducer is sufficient)

### 4. Navbar Integration

- Docusaurus supports [custom navbar items](https://docusaurus.io/docs/api/themes/configuration#navbar-custom) — create a `NavbarAuthItem` component
- Register via `docusaurus.config.ts` navbar items with `type: 'custom-NavbarAuthItem'` or swizzle the `NavbarItem/ComponentTypes`
- Shows "Login" link when unauthenticated, user display name + "Logout" when authenticated

### 5. Database Setup

- **Async SQLAlchemy** with `asyncpg` driver connecting to Neon Postgres
- **Alembic** for schema migrations (new `alembic/` directory in rag-backend)
- Tables: `users`, `refresh_tokens`
- Connection string from `DATABASE_URL` env var (already in constitution)

### 6. Auth UI Design (Brand Theme)

Following the user's instruction to make UI user-friendly and follow the theme:

- **Forms**: Clean card layout centered on page, Pacific Blue submit buttons, Stormy Teal secondary actions
- **Input fields**: Infima form styles with brand border colors, focus ring in Pacific Blue
- **Validation errors**: Inline below each field, using constitution's danger color (#dc3545)
- **Google button**: White background with Google logo + "Continue with Google" text (Google brand guidelines)
- **Dark/light mode**: Full support via CSS variables (Jet Black background / Azure Mist background)
- **Password visibility toggle**: Eye icon to show/hide password
- **Loading states**: Pacific Blue spinner on submit buttons
- **Responsive**: Mobile-first, single-column form layout

### 7. Rate Limiting

- Per-IP rate limit on `/api/auth/login` endpoint: max 5 failed attempts per minute
- Implementation: In-memory counter dict with TTL cleanup (no Redis needed at this scale)
- Returns 429 with `RATE_LIMITED` error code when exceeded

## Complexity Tracking

No constitution violations requiring justification. The "Better-Auth → custom JWT" change is acceptable per constitution's "Stretch goal" label.
