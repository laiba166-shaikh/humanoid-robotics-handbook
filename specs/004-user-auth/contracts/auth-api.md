# Auth API Contract

**Base URL**: `{BACKEND_URL}/api/auth`
**Auth mechanism**: `Authorization: Bearer {access_token}` header
**Response format**: Constitution standard `{success, data, error, meta}`

---

## POST /api/auth/signup

Create a new user account with email and password.

**Request**:
```json
{
  "email": "user@example.com",
  "password": "StrongP@ss1",
  "full_name": "Jane Doe"
}
```

**Success Response** (201):
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "uuid",
      "email": "user@example.com",
      "full_name": "Jane Doe",
      "auth_provider": "local",
      "is_active": true,
      "created_at": "2026-02-13T10:00:00Z"
    },
    "access_token": "eyJ...",
    "refresh_token": "eyJ..."
  },
  "error": null,
  "meta": { "latency_ms": 150 }
}
```

**Error Responses**:
- `409` — `VALIDATION_ERROR`: "Email already registered."
- `422` — `VALIDATION_ERROR`: Password strength failures (specific message per rule)

---

## POST /api/auth/login

Authenticate with email and password.

**Request**:
```json
{
  "email": "user@example.com",
  "password": "StrongP@ss1"
}
```

**Success Response** (200):
```json
{
  "success": true,
  "data": {
    "user": {
      "id": "uuid",
      "email": "user@example.com",
      "full_name": "Jane Doe",
      "auth_provider": "local",
      "is_active": true,
      "created_at": "2026-02-13T10:00:00Z"
    },
    "access_token": "eyJ...",
    "refresh_token": "eyJ..."
  },
  "error": null,
  "meta": { "latency_ms": 120 }
}
```

**Error Responses**:
- `401` — `VALIDATION_ERROR`: "Invalid email or password." (generic, no enumeration)
- `403` — `VALIDATION_ERROR`: "Account is deactivated."
- `429` — `RATE_LIMITED`: "Too many failed attempts. Try again later."

---

## POST /api/auth/refresh

Exchange a valid refresh token for a new access token. Rotates the refresh token.

**Request**:
```json
{
  "refresh_token": "eyJ..."
}
```

**Success Response** (200):
```json
{
  "success": true,
  "data": {
    "access_token": "eyJ...",
    "refresh_token": "eyJ..."
  },
  "error": null,
  "meta": { "latency_ms": 80 }
}
```

**Error Responses**:
- `401` — `VALIDATION_ERROR`: "Invalid or expired refresh token."

---

## POST /api/auth/logout

Revoke the current refresh token. Requires authentication.

**Headers**: `Authorization: Bearer {access_token}`

**Request**:
```json
{
  "refresh_token": "eyJ..."
}
```

**Success Response** (200):
```json
{
  "success": true,
  "data": { "message": "Logged out." },
  "error": null,
  "meta": { "latency_ms": 50 }
}
```

---

## GET /api/auth/me

Get the current authenticated user's profile.

**Headers**: `Authorization: Bearer {access_token}`

**Success Response** (200):
```json
{
  "success": true,
  "data": {
    "id": "uuid",
    "email": "user@example.com",
    "full_name": "Jane Doe",
    "auth_provider": "local",
    "is_active": true,
    "created_at": "2026-02-13T10:00:00Z"
  },
  "error": null,
  "meta": { "latency_ms": 30 }
}
```

**Error Responses**:
- `401` — `VALIDATION_ERROR`: "Not authenticated."

---

## GET /api/auth/google/login

Initiate Google OAuth 2.1 flow. Redirects the browser to Google's consent screen.

**Response**: `302 Redirect` to Google OAuth URL with PKCE challenge, state parameter, and scopes `openid email profile`.

---

## GET /api/auth/google/callback

Handle Google's authorization code callback. Exchanges code for Google tokens, creates or finds user, generates JWT tokens, redirects to frontend.

**Query Params** (from Google):
- `code`: Authorization code
- `state`: CSRF state parameter

**Response**: `302 Redirect` to `{FRONTEND_URL}/auth/callback#access_token=...&refresh_token=...`

**Error Responses** (redirect with error):
- Redirect to `{FRONTEND_URL}/auth/login?error=oauth_failed` on any failure

---

## Common Error Format

All error responses follow the constitution standard:

```json
{
  "success": false,
  "data": null,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Human-readable error message."
  },
  "meta": { "latency_ms": 10 }
}
```

**Error Codes Used**:
| Code | HTTP Status | When |
|------|-------------|------|
| `VALIDATION_ERROR` | 401, 403, 409, 422 | Invalid input, auth failures |
| `RATE_LIMITED` | 429 | Login rate limit exceeded |
| `INTERNAL_ERROR` | 500 | Unexpected server errors |
