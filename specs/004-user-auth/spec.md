# Feature Specification: User Authentication

**Feature Branch**: `004-user-auth`
**Created**: 2026-02-13
**Status**: Draft
**Input**: User description: "Add authentication with email login, signup, and google login. The Login link should be available on the Navbar to go to the auth pages. The user data will be saved in Neon postgres SQL. In the future only logged in user will be able to access the chatbot."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Email Signup (Priority: P1)

A new visitor arrives at the handbook site and wants to create an account. They click "Login" in the navbar, are taken to an authentication page, and choose to sign up with their email address and a password. After submitting valid credentials, their account is created and they are logged in automatically.

**Why this priority**: Account creation is the foundational capability — no other auth feature works without it. This is the minimum slice needed to store users in the database.

**Independent Test**: Can be fully tested by visiting the auth page, filling in the signup form, and confirming the user record is created in the database and the user is logged in.

**Acceptance Scenarios**:

1. **Given** a visitor on the auth page, **When** they enter a valid email, a strong password, and submit the signup form, **Then** an account is created, they are logged in automatically, and redirected to the docs landing page (`/docs/intro`).
2. **Given** a visitor entering an email that already exists, **When** they submit the signup form, **Then** they see an error indicating the email is already registered.
3. **Given** a visitor entering a weak password (fewer than 8 characters, missing uppercase, digit, or special character), **When** they submit the form, **Then** they see specific validation errors explaining what's missing.

---

### User Story 2 - Email Login & Session Management (Priority: P1)

A registered user returns to the site and wants to log in. They click "Login" in the navbar, enter their email and password, and gain access to authenticated features. Their session persists across page reloads and expires after a reasonable period of inactivity.

**Why this priority**: Login is inseparable from signup — together they form the core auth loop. Without login, signup has no practical value.

**Independent Test**: Can be tested by logging in with valid credentials and confirming the user sees authenticated state (e.g., their name or avatar in the navbar, access to protected features).

**Acceptance Scenarios**:

1. **Given** a registered user on the auth page, **When** they enter correct email and password, **Then** they are logged in and redirected to the docs landing page (`/docs/intro`) with the navbar showing their authenticated state.
2. **Given** a user entering incorrect credentials, **When** they submit the login form, **Then** they see a generic "Invalid email or password" error (no information leakage about which field is wrong).
3. **Given** a logged-in user, **When** they refresh the page within the session window, **Then** their authenticated state is restored via token refresh.
4. **Given** a logged-in user, **When** they click "Logout" in the navbar, **Then** their session is terminated and the navbar reverts to showing the "Login" link.
5. **Given** a user whose session has expired, **When** they try to access an authenticated feature, **Then** they are prompted to log in again.

---

### User Story 3 - Google Social Login (Priority: P2)

A visitor prefers not to create a new password and wants to sign up or log in using their Google account. They click "Login" in the navbar, then click "Continue with Google" on the auth page. After authorizing via Google's consent screen, they are logged in and their profile information (name, email) is pulled from their Google account.

**Why this priority**: Social login reduces friction for users who don't want to manage yet another password. It's secondary to core email auth but significantly improves conversion.

**Independent Test**: Can be tested by clicking "Continue with Google", completing the Google consent flow, and confirming the user is logged in with their Google profile data.

**Acceptance Scenarios**:

1. **Given** a visitor on the auth page, **When** they click "Continue with Google" and authorize the app, **Then** an account is created (or existing account found) and they are logged in with their Google profile information.
2. **Given** a user who previously signed up with email and now tries Google login with the same email, **When** they complete the Google flow, **Then** their Google identity is linked to their existing account.
3. **Given** a user who denies consent on the Google screen, **When** they are redirected back, **Then** they see a message that Google login was cancelled and can try again or use email instead.

---

### User Story 4 - Navbar Authentication UI (Priority: P1)

The site navbar must reflect the user's authentication state at all times. When logged out, a "Login" link appears that navigates to the auth pages. When logged in, the user sees their name or avatar and has access to logout.

**Why this priority**: The navbar is the primary navigation element and the entry point to all auth flows. Without it, users can't discover or access authentication.

**Independent Test**: Can be tested by verifying the navbar shows "Login" when logged out, and shows user info + logout option when logged in.

**Acceptance Scenarios**:

1. **Given** an unauthenticated visitor, **When** they view any page, **Then** the navbar displays a "Login" link on the right side.
2. **Given** the visitor clicks the "Login" link, **When** the page loads, **Then** they are taken to the authentication page with login and signup options.
3. **Given** a logged-in user, **When** they view any page, **Then** the navbar shows their display name (or email) and a logout option instead of the "Login" link.

---

### Edge Cases

- What happens when a user tries to sign up with a Google-only account's email via email/password? They should be prompted to use Google login or set a password to enable both methods.
- How does the system handle concurrent sessions from multiple devices? Each device maintains its own session; logging out on one device does not affect others.
- What happens if Google's OAuth service is temporarily unavailable? The Google login button should show an appropriate error; email login remains unaffected.
- What happens when a user's email changes on Google's side after account linking? The system uses the Google account ID (not email) as the linking key, so email changes on Google's side don't break the link.
- What happens when a user exceeds the login rate limit? They receive a "Too many attempts" error and must wait before retrying. The lockout is per-IP and does not affect other users or Google login.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow visitors to create an account using email and password.
- **FR-002**: System MUST validate email format and enforce password strength rules (minimum 8 characters, at least 1 uppercase letter, 1 digit, and 1 special character).
- **FR-003**: System MUST allow users to log in with email and password and receive a persistent session.
- **FR-004**: System MUST support login and signup via Google using the OAuth 2.1 authorization code flow.
- **FR-005**: System MUST link Google accounts to existing email accounts when the email addresses match.
- **FR-006**: System MUST display a "Login" link in the navbar when the user is unauthenticated, and the user's name with a logout option when authenticated.
- **FR-007**: System MUST persist all user data in Neon Postgres.
- **FR-008**: System MUST allow users to log out, terminating their current session.
- **FR-009**: System MUST return generic error messages for login failures (no email enumeration).
- **FR-010**: System MUST support token refresh so users can maintain authenticated state across page reloads without re-entering credentials.
- **FR-011**: System MUST rate-limit failed login attempts per IP address (maximum 5 failures per minute) and return a temporary lockout response when exceeded.

### Key Entities

- **User**: Represents a registered account. Key attributes: email (unique), display name, authentication provider (email or Google), active status, creation date.
- **RefreshToken**: Represents an active user session. Key attributes: associated user, expiration time, revocation status.

## Clarifications

### Session 2026-02-13

- Q: How will the frontend and backend be hosted relative to each other? → A: Separate unrelated domains — tokens delivered via response body, frontend stores in memory.
- Q: Where should the auth pages (login/signup forms) live? → A: Custom Docusaurus pages (`src/pages/auth/login.tsx`, `src/pages/auth/signup.tsx`) — React components inside the existing site.
- Q: Where should users land after successful login or signup? → A: Docs landing page (`/docs/intro`) — go straight to the learning content.
- Q: Should the login endpoint have rate limiting for failed attempts? → A: Per-IP rate limit (max 5 failed attempts per minute, temporary lockout).

## Assumptions

- The Docusaurus frontend and FastAPI backend are deployed on separate, unrelated domains. Auth tokens are delivered in API response bodies (not cookies). The frontend stores the access token in memory and sends it via `Authorization: Bearer` header on API requests.
- Neon Postgres is the sole relational database; no additional database services are introduced.
- Chatbot access gating is a **future phase** — this feature only adds authentication infrastructure and does not restrict existing endpoints.
- The auth pages (login, signup) are custom Docusaurus pages built as React components under `src/pages/` within the existing Docusaurus site.
- No admin dashboard or user management UI is in scope for this feature.
- Email verification and password reset are **out of scope** for this phase and may be added in a future iteration.
- The `better-auth-setup` skill (`.claude/skills/better-auth-setup/`) should be used during implementation to scaffold the backend auth module (signup, login, refresh, logout, Google OIDC, JWT tokens).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete email signup (form submission to account created) in under 60 seconds.
- **SC-002**: Users can complete email login (form submission to authenticated state) in under 5 seconds.
- **SC-003**: Users can complete Google login (click to authenticated state) in under 10 seconds (excluding time on Google's consent screen).
- **SC-004**: 95% of signup attempts with valid data succeed on the first try without errors.
- **SC-005**: The navbar correctly reflects authentication state on every page within 1 second of load.
- **SC-006**: All user records are persisted in Neon Postgres with no data loss across server restarts.
- **SC-007**: Refresh tokens are rotated on each use and old tokens cannot be reused.
