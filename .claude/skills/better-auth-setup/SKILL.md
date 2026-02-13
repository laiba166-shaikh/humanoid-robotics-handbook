---
name: better-auth-setup
description: >
  Scaffold a production-ready FastAPI authentication module with email/password signup and login,
  JWT access + refresh tokens (httpOnly cookies), Google OIDC/OAuth 2.1 social login,
  and PostgreSQL storage.
  Use when the user asks to add authentication, set up login/signup, integrate OAuth/OIDC,
  add JWT auth, or secure a FastAPI application. Triggers on phrases like "add auth",
  "setup authentication", "login signup", "JWT tokens", "OAuth", "OIDC", "Google login",
  "secure my API".
---

# Better Auth Setup

Generate a complete, pluggable FastAPI authentication module with:

- Email/password registration and login with validation
- JWT access tokens (15 min) + refresh tokens (7 days) in httpOnly cookies
- Google OIDC / OAuth 2.1 social login
- PostgreSQL via SQLAlchemy (async)

## Workflow

1. **Discover project** - detect existing FastAPI app structure, DB setup, dependencies
2. **Install dependencies** - add required packages
3. **Generate auth module** - create all files per module structure reference
4. **Wire into app** - register router, add middleware, update DB init
5. **Configure environment** - add required env vars to `.env.example`
6. **Verify** - run the app and confirm endpoints respond

## Step 1: Discover Project

Before generating code, read the project to understand:

- Entry point (usually `main.py` or `app/main.py`)
- Existing database setup (SQLAlchemy session, Base, engine)
- Existing middleware or dependency injection patterns
- Package manager (`pip`, `poetry`, `uv`)

Adapt generated code to match existing patterns. Do not duplicate DB setup.

## Step 2: Install Dependencies

Required packages:

```
fastapi
sqlalchemy[asyncio]
asyncpg
passlib[bcrypt]
python-jose[cryptography]
pydantic[email-validator]
httpx
python-multipart
```

Use the project's package manager to install.

## Step 3: Generate Auth Module

Create files following the structure in [references/module-structure.md](references/module-structure.md).

Target directory: `app/auth/` (adapt to project layout).

```
app/auth/
  __init__.py
  models.py        # User + RefreshToken SQLAlchemy models
  schemas.py       # Pydantic request/response schemas
  validators.py    # Email & password validation
  security.py      # JWT create/verify, password hash/verify
  dependencies.py  # get_current_user FastAPI dependency
  router.py        # /signup, /login, /refresh, /logout endpoints
  google_oidc.py   # Google OAuth 2.1 authorization code flow
```

### Validation Rules (Standard)

- **Email**: valid format via Pydantic `EmailStr`
- **Password**: min 8 chars, at least 1 uppercase, 1 digit, 1 special char (`!@#$%^&*()_+-=`)

### JWT Configuration

| Parameter | Value | Env Var |
|-----------|-------|---------|
| Access token lifetime | 15 minutes | `ACCESS_TOKEN_EXPIRE_MINUTES` |
| Refresh token lifetime | 7 days | `REFRESH_TOKEN_EXPIRE_DAYS` |
| Algorithm | HS256 | `JWT_ALGORITHM` |
| Secret key | random 64-char | `JWT_SECRET_KEY` |

Tokens delivered via **httpOnly, Secure, SameSite=Lax** cookies.

### Google OIDC / OAuth 2.1

See [references/google-oidc.md](references/google-oidc.md) for the full integration pattern.

Key endpoints:
- `GET /auth/google/login` - redirect to Google consent screen
- `GET /auth/google/callback` - handle authorization code, create/find user, set tokens

## Step 4: Wire Into App

In the FastAPI entry point:

```python
from app.auth.router import auth_router

app.include_router(auth_router, prefix="/auth", tags=["Authentication"])
```

Add CORS middleware if not present (allow credentials for cookies).

## Step 5: Configure Environment

Add to `.env.example`:

```env
# === Auth ===
JWT_SECRET_KEY=           # openssl rand -hex 32
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=15
REFRESH_TOKEN_EXPIRE_DAYS=7

# === Google OAuth ===
GOOGLE_CLIENT_ID=
GOOGLE_CLIENT_SECRET=
GOOGLE_REDIRECT_URI=http://localhost:8000/auth/google/callback

# === Database ===
DATABASE_URL=postgresql+asyncpg://user:pass@localhost:5432/dbname
```

## Step 6: Verify

Run the server and confirm these endpoints return proper responses:

```
POST /auth/signup          -> 201 + user created
POST /auth/login           -> 200 + cookies set
POST /auth/refresh         -> 200 + new access token
POST /auth/logout          -> 200 + cookies cleared
GET  /auth/google/login    -> 302 redirect to Google
GET  /auth/google/callback -> 200 + cookies set (after Google flow)
GET  /auth/me              -> 200 + current user (protected)
```

## References

- [Module Structure](references/module-structure.md) - Complete code patterns for every file
- [Google OIDC](references/google-oidc.md) - OAuth 2.1 authorization code flow
