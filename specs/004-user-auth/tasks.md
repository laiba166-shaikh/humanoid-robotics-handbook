# Tasks: User Authentication

**Input**: Design documents from `specs/004-user-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/auth-api.md, quickstart.md

**Tests**: Not explicitly requested — test tasks omitted. Manual verification via quickstart.md.

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1–US4)
- Exact file paths included in every task

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Install dependencies, configure environment, and initialize database tooling

- [X] T001 Add auth dependencies to `rag-backend/requirements.txt` (sqlalchemy[asyncio], asyncpg, passlib[bcrypt], python-jose[cryptography], pydantic[email-validator], httpx, python-multipart, alembic)
- [X] T002 Add auth environment variables to `rag-backend/.env.example` (JWT_SECRET_KEY, JWT_ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES, REFRESH_TOKEN_EXPIRE_DAYS, GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET, GOOGLE_REDIRECT_URI, DATABASE_URL, FRONTEND_URL)
- [X] T003 [P] Update `rag-backend/app/config.py` to add auth settings (jwt_secret_key, jwt_algorithm, access_token_expire_minutes, refresh_token_expire_days, google_client_id, google_client_secret, google_redirect_uri, database_url, frontend_url) using pydantic-settings

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Database connection, SQLAlchemy setup, migration framework, and shared auth utilities that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create async SQLAlchemy engine, session factory, and Base in `rag-backend/app/database.py` using DATABASE_URL from config
- [X] T005 Initialize Alembic for async migrations in `rag-backend/` (alembic init, configure alembic.ini and env.py for async SQLAlchemy with Neon Postgres)
- [X] T006 [P] Create User model in `rag-backend/app/auth/models.py` (id UUID PK, email unique, hashed_password nullable, full_name, is_active, auth_provider, google_sub unique nullable, created_at, updated_at) per data-model.md
- [X] T007 Create RefreshToken model in `rag-backend/app/auth/models.py` (id UUID PK, token unique indexed, user_id FK, expires_at, revoked, created_at) with User relationship per data-model.md
- [X] T008 Generate and run Alembic migration for users and refresh_tokens tables (`alembic revision --autogenerate -m "create auth tables"` then `alembic upgrade head`)
- [X] T009 [P] Create password validator in `rag-backend/app/auth/validators.py` (min 8 chars, 1 uppercase, 1 digit, 1 special char) per spec FR-002
- [X] T010 [P] Create JWT security utilities in `rag-backend/app/auth/security.py` (hash_password, verify_password, create_access_token, create_refresh_token, decode_token) per contracts/auth-api.md
- [X] T011 [P] Create Pydantic request/response schemas in `rag-backend/app/auth/schemas.py` (SignupRequest, LoginRequest, RefreshRequest, LogoutRequest, UserResponse, TokenResponse, MessageResponse) wrapping constitution response format {success, data, error, meta}
- [X] T012 Create get_current_user dependency in `rag-backend/app/auth/dependencies.py` (extract Bearer token from Authorization header, decode, look up user)
- [X] T013 Create `rag-backend/app/auth/__init__.py` exporting auth_router
- [X] T014 Register auth router in `rag-backend/app/main.py` under prefix `/api/auth`, add database lifespan init (create tables on startup), add SessionMiddleware for Google OAuth state

**Checkpoint**: Database connected, models migrated, auth utilities ready — user story implementation can begin

---

## Phase 3: User Story 1 — Email Signup (Priority: P1) MVP

**Goal**: Visitors can create an account with email and password, are logged in automatically, and redirected to docs landing page

**Independent Test**: POST `/api/auth/signup` with valid email + strong password → 201 with user + tokens in response body

### Implementation for User Story 1

- [X] T015 [US1] Implement POST `/api/auth/signup` endpoint in `rag-backend/app/auth/router.py` — validate input, check duplicate email (if email exists with `auth_provider=google` and no `hashed_password`, return error guiding user to use Google login or set a password), hash password, create user, generate access + refresh tokens, return constitution response format per contracts/auth-api.md
- [X] T016 [US1] Create signup page component in `humanoid-textbook/src/pages/auth/signup.tsx` — form with email, password, full_name fields, password strength indicator, validation errors inline, Pacific Blue submit button, link to login page, follows brand theme (Infima + CSS modules)
- [X] T017 [US1] Create auth API client in `humanoid-textbook/src/services/authApi.ts` — fetch wrapper with base URL from env, functions: signup(), login(), refresh(), logout(), getMe(), getGoogleLoginUrl(); handles Authorization header injection and token refresh on 401
- [X] T018 [US1] Create AuthProvider context in `humanoid-textbook/src/components/auth/AuthProvider.tsx` — React Context with useReducer; state: user, accessToken, isAuthenticated, isLoading; actions: login, signup, logout, restoreSession; on mount: check localStorage for refresh token → call refresh() → restore auth state
- [X] T019 [US1] Create shared auth styles in `humanoid-textbook/src/components/auth/auth.module.css` — form card layout (centered, max-width 420px), input fields with brand focus ring (Pacific Blue), button styles (Pacific Blue primary, Stormy Teal hover), error text (danger red), dark/light mode via CSS variables, responsive mobile layout
- [X] T020 [US1] Register AuthProvider as Docusaurus client module — add to `humanoid-textbook/docusaurus.config.ts` clientModules or create `humanoid-textbook/src/theme/Root.tsx` wrapper that wraps children with AuthProvider

**Checkpoint**: Email signup works end-to-end. Visitor fills form → user created in Neon Postgres → tokens returned → frontend stores tokens → user is authenticated

---

## Phase 4: User Story 2 — Email Login & Session Management (Priority: P1)

**Goal**: Registered users can log in with email/password, sessions persist across page refresh, logout works

**Independent Test**: POST `/api/auth/login` → 200 with tokens; refresh page → still authenticated; logout → session cleared

### Implementation for User Story 2

- [X] T021 [US2] Implement POST `/api/auth/login` endpoint in `rag-backend/app/auth/router.py` — validate credentials, check is_active, generate tokens, store refresh token in DB, return constitution response format. Add per-IP rate limiting (5 failures/min) with in-memory counter dict per spec FR-011
- [X] T022 [US2] Implement POST `/api/auth/refresh` endpoint in `rag-backend/app/auth/router.py` — validate refresh token from request body, check not revoked/expired, rotate (revoke old, issue new), return new access + refresh tokens
- [X] T023 [US2] Implement POST `/api/auth/logout` endpoint in `rag-backend/app/auth/router.py` — require Authorization header, revoke refresh token from request body, return success message
- [X] T024 [US2] Implement GET `/api/auth/me` endpoint in `rag-backend/app/auth/router.py` — require get_current_user dependency, return user profile in constitution response format
- [X] T025 [US2] Create login page in `humanoid-textbook/src/pages/auth/login.tsx` — form with email + password, "Invalid email or password" generic error display, parse `?error=oauth_failed` query param and display "Google login was cancelled. Try again or use email.", link to signup page, Pacific Blue submit button, brand-themed card layout using auth.module.css
- [X] T026 [US2] Add login(), logout(), and session restore logic to AuthProvider in `humanoid-textbook/src/components/auth/AuthProvider.tsx` — login stores tokens (access in state, refresh in localStorage), logout clears both and calls API, page refresh triggers token refresh from localStorage
- [X] T027 [US2] Add post-login redirect to `/docs/intro` in login and signup pages (`humanoid-textbook/src/pages/auth/login.tsx` and `humanoid-textbook/src/pages/auth/signup.tsx`) using Docusaurus `useHistory` or `window.location`

**Checkpoint**: Full login/logout cycle works. Login → authenticated → refresh page → still authenticated → logout → unauthenticated

---

## Phase 5: User Story 4 — Navbar Authentication UI (Priority: P1)

**Goal**: Navbar shows "Login" link when unauthenticated, user display name + "Logout" when authenticated

**Independent Test**: Load any page unauthenticated → see "Login" link right side; log in → navbar shows name + Logout button

### Implementation for User Story 4

- [X] T028 [US4] Create NavbarAuthItem component in `humanoid-textbook/src/components/auth/NavbarAuthItem.tsx` — reads auth state from AuthProvider context; when unauthenticated: renders "Login" link to `/auth/login`; when authenticated: renders user display name (or email) + "Logout" button; styled with brand theme (Pacific Blue link, Stormy Teal hover)
- [X] T029 [US4] Register NavbarAuthItem in Docusaurus navbar — either swizzle `NavbarItem/ComponentTypes` to add custom type OR add as custom navbar item in `humanoid-textbook/docusaurus.config.ts` items array, position: right (before GitHub link)
- [X] T030 [US4] Create NavbarAuthItem styles in `humanoid-textbook/src/components/auth/NavbarAuthItem.module.css` — user name display, logout button (compact, matches navbar height), dropdown menu if needed, dark/light mode support, mobile responsive (hamburger menu compatible)

**Checkpoint**: Navbar correctly reflects auth state on every page. "Login" when logged out, user info + Logout when logged in.

---

## Phase 6: User Story 3 — Google Social Login (Priority: P2)

**Goal**: Users can sign up or log in via Google OAuth 2.1, accounts are linked when emails match

**Independent Test**: Click "Continue with Google" → complete Google consent → redirected back authenticated with Google profile data

### Implementation for User Story 3

- [X] T031 [US3] Implement GET `/api/auth/google/login` endpoint in `rag-backend/app/auth/google_oidc.py` — generate PKCE verifier + challenge, store in session (SessionMiddleware), redirect to Google authorization URL with scopes openid email profile
- [X] T032 [US3] Implement GET `/api/auth/google/callback` endpoint in `rag-backend/app/auth/google_oidc.py` — validate state, exchange code for Google tokens with PKCE verifier, fetch userinfo, find-or-create user (link if email matches existing), generate JWT tokens, redirect to `{FRONTEND_URL}/auth/callback#access_token=...&refresh_token=...`
- [X] T033 [US3] Create Google OAuth callback page in `humanoid-textbook/src/pages/auth/callback.tsx` — extract tokens from URL fragment (hash params), store in AuthProvider (access in state, refresh in localStorage), redirect to `/docs/intro`; on error: redirect to `/auth/login?error=oauth_failed`
- [X] T034 [US3] Create GoogleLoginButton component in `humanoid-textbook/src/components/auth/GoogleLoginButton.tsx` — white button with Google logo + "Continue with Google" text (Google brand guidelines), onClick navigates to `{BACKEND_URL}/api/auth/google/login`, styled to match auth card layout
- [X] T035 [US3] Add GoogleLoginButton to login and signup pages (`humanoid-textbook/src/pages/auth/login.tsx` and `humanoid-textbook/src/pages/auth/signup.tsx`) — positioned below the email form with an "or" divider

**Checkpoint**: Google login works end-to-end. Click Google button → consent screen → callback → authenticated with Google profile.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, error handling, and UX improvements across all stories

- [X] T036 Add CORS origin for frontend URL in `rag-backend/app/main.py` — ensure FRONTEND_URL is included in allowed_origins alongside existing origins
- [X] T037 [P] Add loading and error states to all auth pages — spinner on form submit (Pacific Blue), error banners for API failures, disabled buttons during requests in `humanoid-textbook/src/pages/auth/login.tsx` and `signup.tsx`
- [X] T038 [P] Add password visibility toggle (show/hide eye icon) to password fields in `humanoid-textbook/src/pages/auth/signup.tsx` and `login.tsx`
- [X] T039 Verify all auth endpoints return constitution response format {success, data, error, meta} with correct error codes (VALIDATION_ERROR, RATE_LIMITED) in `rag-backend/app/auth/router.py`
- [X] T040 Run quickstart.md validation — test all 7 endpoints manually (signup, login, refresh, logout, me, google/login, google/callback) and verify navbar state changes; spot-check latency targets (signup < 60s, login < 5s, navbar < 1s)
- [X] T041 Update `rag-backend/.env.example` with all final auth environment variables and add setup comments
- [X] T042 Update `.specify/memory/constitution.md` Environment Variables section to include auth env vars (JWT_SECRET_KEY, JWT_ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES, REFRESH_TOKEN_EXPIRE_DAYS, GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET, GOOGLE_REDIRECT_URI, FRONTEND_URL) — standard post-implementation sync

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 — BLOCKS all user stories
- **Phase 3 (US1 Signup)**: Depends on Phase 2 — MVP delivery
- **Phase 4 (US2 Login)**: Depends on Phase 2; benefits from US1 components (AuthProvider, authApi.ts, auth.module.css) but can be built independently
- **Phase 5 (US4 Navbar)**: Depends on Phase 2 + AuthProvider from US1 (T018/T020)
- **Phase 6 (US3 Google)**: Depends on Phase 2; benefits from US1 components
- **Phase 7 (Polish)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (Email Signup)**: Phase 2 only — fully independent, creates foundational frontend components (AuthProvider, authApi.ts, auth.module.css)
- **US2 (Email Login)**: Phase 2 only — reuses AuthProvider and authApi.ts from US1, but endpoints are independent
- **US4 (Navbar UI)**: Requires AuthProvider from US1 (T018/T020) to read auth state
- **US3 (Google Login)**: Phase 2 only for backend; reuses login/signup pages from US1/US2 for GoogleLoginButton placement

### Recommended Execution Order

```
Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 5 (US4) → Phase 6 (US3) → Phase 7
```

### Parallel Opportunities

**Phase 2** (after Phase 1):
```
T006 + T007 (models)         — parallel, same file but independent classes
T009 + T010 + T011           — parallel, different files
```

**Phase 3** (US1, after Phase 2):
```
T015 (backend signup)  +  T016 + T017 + T018 + T019 (frontend components) — parallel, different repos
```

**Phase 7** (Polish):
```
T037 + T038                  — parallel, same pages but independent features
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001–T003)
2. Complete Phase 2: Foundational (T004–T014)
3. Complete Phase 3: User Story 1 — Email Signup (T015–T020)
4. **STOP and VALIDATE**: Signup works end-to-end, user in Neon Postgres, tokens returned
5. Deploy/demo if ready

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. US1 (Signup) → Test → Deploy (MVP!)
3. US2 (Login) → Test → Deploy (core auth complete)
4. US4 (Navbar) → Test → Deploy (full UX)
5. US3 (Google) → Test → Deploy (social login)
6. Polish → Final validation → Deploy

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- All backend auth endpoints MUST follow constitution response format: `{success, data, error, meta}`
- All frontend auth UI MUST use brand color palette only (Pacific Blue, Stormy Teal, Azure Mist, Jet Black, Pacific Cyan)
- Use the `better-auth-setup` skill as a reference for backend code patterns but adapt token delivery from cookies to response body (Authorization: Bearer header)
- Commit after each task or logical group
