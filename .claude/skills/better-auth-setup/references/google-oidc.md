# Google OIDC / OAuth 2.1 Integration

Authorization Code flow with PKCE (OAuth 2.1 best practice).

## Prerequisites

1. Create a project at https://console.cloud.google.com
2. Enable "Google Identity" API
3. Create OAuth 2.0 credentials (Web application type)
4. Set authorized redirect URI to `{BASE_URL}/auth/google/callback`
5. Add `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`, `GOOGLE_REDIRECT_URI` to `.env`

## google_oidc.py

```python
import secrets
import hashlib
import base64
from urllib.parse import urlencode

import httpx
from fastapi import APIRouter, Depends, HTTPException, Request, Response, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.config import settings  # adapt import
from app.database import get_session  # adapt import
from app.auth import models, security

google_oidc_router = APIRouter(prefix="/google")

GOOGLE_AUTH_URL = "https://accounts.google.com/o/oauth2/v2/auth"
GOOGLE_TOKEN_URL = "https://oauth2.googleapis.com/token"
GOOGLE_USERINFO_URL = "https://www.googleapis.com/oauth2/v3/userinfo"


def _generate_pkce() -> tuple[str, str]:
    """Generate PKCE code_verifier and code_challenge (S256)."""
    verifier = secrets.token_urlsafe(64)
    digest = hashlib.sha256(verifier.encode()).digest()
    challenge = base64.urlsafe_b64encode(digest).rstrip(b"=").decode()
    return verifier, challenge


@google_oidc_router.get("/login")
async def google_login(request: Request):
    """Redirect user to Google consent screen."""
    verifier, challenge = _generate_pkce()
    state = secrets.token_urlsafe(32)

    # Store verifier and state in session or signed cookie
    request.session["oauth_code_verifier"] = verifier
    request.session["oauth_state"] = state

    params = {
        "client_id": settings.GOOGLE_CLIENT_ID,
        "redirect_uri": settings.GOOGLE_REDIRECT_URI,
        "response_type": "code",
        "scope": "openid email profile",
        "state": state,
        "code_challenge": challenge,
        "code_challenge_method": "S256",
        "access_type": "online",
        "prompt": "select_account",
    }
    url = f"{GOOGLE_AUTH_URL}?{urlencode(params)}"
    return Response(status_code=status.HTTP_302_FOUND, headers={"Location": url})


@google_oidc_router.get("/callback")
async def google_callback(
    request: Request,
    response: Response,
    code: str,
    state: str,
    db: AsyncSession = Depends(get_session),
):
    """Exchange authorization code for tokens and create/find user."""
    stored_state = request.session.pop("oauth_state", None)
    verifier = request.session.pop("oauth_code_verifier", None)

    if not stored_state or state != stored_state:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid OAuth state.")

    # Exchange code for tokens
    async with httpx.AsyncClient() as client:
        token_resp = await client.post(
            GOOGLE_TOKEN_URL,
            data={
                "code": code,
                "client_id": settings.GOOGLE_CLIENT_ID,
                "client_secret": settings.GOOGLE_CLIENT_SECRET,
                "redirect_uri": settings.GOOGLE_REDIRECT_URI,
                "grant_type": "authorization_code",
                "code_verifier": verifier,
            },
        )
        if token_resp.status_code != 200:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Failed to exchange code.")

        tokens = token_resp.json()
        google_access = tokens["access_token"]

        # Fetch user info
        userinfo_resp = await client.get(
            GOOGLE_USERINFO_URL,
            headers={"Authorization": f"Bearer {google_access}"},
        )
        if userinfo_resp.status_code != 200:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Failed to get user info.")

        userinfo = userinfo_resp.json()

    google_sub = userinfo["sub"]
    email = userinfo["email"]
    name = userinfo.get("name")

    # Find or create user
    result = await db.execute(select(models.User).where(models.User.google_sub == google_sub))
    user = result.scalar_one_or_none()

    if not user:
        # Check if email already exists (local account)
        result = await db.execute(select(models.User).where(models.User.email == email))
        user = result.scalar_one_or_none()
        if user:
            # Link Google to existing account
            user.google_sub = google_sub
            user.is_verified = True
        else:
            # Create new user
            user = models.User(
                email=email,
                full_name=name,
                auth_provider="google",
                google_sub=google_sub,
                is_verified=True,
            )
            db.add(user)

    await db.commit()
    await db.refresh(user)

    # Issue JWT tokens
    access = security.create_access_token(user.id, user.email)
    refresh, expires = security.create_refresh_token(user.id)
    db.add(models.RefreshToken(token=refresh, user_id=user.id, expires_at=expires))
    await db.commit()

    response.set_cookie("access_token", access, httponly=True, secure=True, samesite="lax", path="/")
    response.set_cookie("refresh_token", refresh, httponly=True, secure=True, samesite="lax", path="/auth")
    return {"message": "Google login successful.", "email": user.email}
```

## Session Middleware Requirement

PKCE verifier and state are stored in the server-side session. Add `SessionMiddleware`:

```python
from starlette.middleware.sessions import SessionMiddleware

app.add_middleware(SessionMiddleware, secret_key=settings.JWT_SECRET_KEY)
```

If sessions are undesirable, replace with signed cookies or a short-lived cache (Redis).

## Account Linking

The callback handles three cases:
1. **Returning Google user** - found by `google_sub`, issue tokens
2. **Existing local user** - same email exists, link `google_sub` to it, mark verified
3. **New user** - create with `auth_provider="google"`, no password
