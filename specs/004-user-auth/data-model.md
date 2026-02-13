# Data Model: User Authentication

**Feature**: 004-user-auth | **Date**: 2026-02-13

## Entities

### User

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PK, auto-generated | UUIDv4 |
| email | VARCHAR(255) | UNIQUE, NOT NULL, indexed | Canonical identity |
| hashed_password | VARCHAR(255) | NULLABLE | Null for Google-only accounts |
| full_name | VARCHAR(255) | NULLABLE | From signup form or Google profile |
| is_active | BOOLEAN | DEFAULT true | Soft delete / deactivation |
| auth_provider | VARCHAR(50) | DEFAULT "local" | "local" or "google" |
| google_sub | VARCHAR(255) | UNIQUE, NULLABLE | Google account ID for linking |
| created_at | TIMESTAMP | DEFAULT now() | |
| updated_at | TIMESTAMP | DEFAULT now(), ON UPDATE now() | |

**Identity rules**:
- Email is the unique human-facing identifier
- `google_sub` is the unique Google-facing identifier (survives email changes on Google's side)
- A user can have both `hashed_password` AND `google_sub` (linked accounts)

**State transitions**:
- Created (is_active=true) → Deactivated (is_active=false) — admin action only (future)
- No deleted state — soft delete via is_active

### RefreshToken

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PK, auto-generated | UUIDv4 |
| token | TEXT | UNIQUE, NOT NULL, indexed | JWT refresh token string |
| user_id | UUID | FK → users.id, ON DELETE CASCADE | |
| expires_at | TIMESTAMP | NOT NULL | 7 days from creation |
| revoked | BOOLEAN | DEFAULT false | Revoked on rotation or logout |
| created_at | TIMESTAMP | DEFAULT now() | |

**Lifecycle**:
- Created on login/signup/Google auth
- Revoked on use (rotation) or logout
- Old revoked tokens remain for audit; cleanup via scheduled task (future)

## Relationships

```
User (1) ──── (N) RefreshToken
```

- One user can have multiple active refresh tokens (multi-device support)
- Cascade delete: deleting a user removes all their refresh tokens

## Indexes

| Table | Column(s) | Type | Reason |
|-------|-----------|------|--------|
| users | email | UNIQUE B-tree | Login lookup |
| users | google_sub | UNIQUE B-tree (partial, WHERE NOT NULL) | Google account linking |
| refresh_tokens | token | UNIQUE B-tree | Token validation |
| refresh_tokens | user_id | B-tree | User's tokens lookup |

## Validation Rules (from spec FR-002)

| Field | Rule |
|-------|------|
| email | Valid format (Pydantic EmailStr) |
| password | Min 8 chars, 1 uppercase, 1 digit, 1 special char (`!@#$%^&*()_+-=`) |
| full_name | Optional, max 255 chars |
