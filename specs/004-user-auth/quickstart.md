# Quickstart: User Authentication

**Feature**: 004-user-auth | **Date**: 2026-02-13

## Prerequisites

- Python 3.10+
- Node.js 18+
- Neon Postgres database (free tier)
- Google Cloud project with OAuth 2.0 credentials

## Backend Setup

### 1. Install new dependencies

```bash
cd rag-backend
pip install sqlalchemy[asyncio] asyncpg passlib[bcrypt] python-jose[cryptography] pydantic[email-validator] httpx python-multipart alembic
```

### 2. Add environment variables

Add to `rag-backend/.env`:

```env
# Auth
JWT_SECRET_KEY=<openssl rand -hex 32>
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=15
REFRESH_TOKEN_EXPIRE_DAYS=7

# Google OAuth
GOOGLE_CLIENT_ID=<from Google Cloud Console>
GOOGLE_CLIENT_SECRET=<from Google Cloud Console>
GOOGLE_REDIRECT_URI=http://localhost:8000/api/auth/google/callback

# Database (Neon Postgres)
DATABASE_URL=postgresql+asyncpg://<user>:<pass>@<host>/<db>?sslmode=require

# Frontend URL (for OAuth redirect)
FRONTEND_URL=http://localhost:3000
```

### 3. Initialize database

```bash
cd rag-backend
alembic upgrade head
```

### 4. Run backend

```bash
uvicorn app.main:app --reload --port 8000
```

### 5. Verify endpoints

```bash
# Signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test@1234"}'

# Login
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test@1234"}'
```

## Frontend Setup

### 1. No new npm dependencies needed

The auth client uses the native `fetch` API and React Context (built-in).

### 2. Start dev server

```bash
cd humanoid-textbook
npm start
```

### 3. Test auth flow

1. Open `http://localhost:3000/auth/login`
2. Click "Sign Up" tab
3. Enter email + password → should redirect to `/docs/intro`
4. Navbar should show user name + Logout button
5. Refresh page → should stay authenticated (token refresh)
6. Click Logout → navbar reverts to "Login" link

## Google OAuth Setup

1. Go to [Google Cloud Console](https://console.cloud.google.com)
2. Create or select a project
3. Enable "Google Identity" API
4. Create OAuth 2.0 credentials (Web application)
5. Add authorized redirect URI: `http://localhost:8000/api/auth/google/callback`
6. Copy Client ID and Client Secret to `.env`

## Key Files

| File | Purpose |
|------|---------|
| `rag-backend/app/auth/router.py` | All auth API endpoints |
| `rag-backend/app/auth/google_oidc.py` | Google OAuth flow |
| `rag-backend/app/database.py` | Async SQLAlchemy engine + session |
| `humanoid-textbook/src/components/auth/AuthProvider.tsx` | React auth context |
| `humanoid-textbook/src/pages/auth/login.tsx` | Login/signup page |
| `humanoid-textbook/src/components/auth/NavbarAuthItem.tsx` | Navbar auth widget |
| `humanoid-textbook/docusaurus.config.ts` | Navbar item registration |
