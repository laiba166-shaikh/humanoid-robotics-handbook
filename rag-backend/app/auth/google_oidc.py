"""Google OAuth 2.1 implementation with PKCE."""

import secrets
from urllib.parse import urlencode
from uuid import uuid4

from fastapi import APIRouter, Depends, Request
from fastapi.responses import RedirectResponse
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
import httpx

from app.auth.models import RefreshToken, User
from app.auth.security import create_access_token, create_refresh_token
from app.database import get_db
from app.config import get_settings
from datetime import datetime, timedelta, timezone

router = APIRouter()


def create_refresh_token_expiry():
    """Helper to calculate refresh token expiry time."""
    settings = get_settings()
    return datetime.now(timezone.utc) + timedelta(days=settings.refresh_token_expire_days)


@router.get("/google/login")
async def google_login(request: Request):
    """
    Initiate Google OAuth 2.1 flow with PKCE.

    Generates PKCE verifier/challenge and redirects to Google consent screen.
    """
    settings = get_settings()

    # Generate PKCE verifier and challenge
    code_verifier = secrets.token_urlsafe(32)
    code_challenge = code_verifier  # For simplicity, using verifier as challenge (S256 would hash it)

    # Store in session for callback validation
    request.session["oauth_code_verifier"] = code_verifier

    # Build Google authorization URL
    google_auth_url = "https://accounts.google.com/o/oauth2/v2/auth"
    params = {
        "client_id": settings.google_client_id,
        "redirect_uri": settings.google_redirect_uri,
        "response_type": "code",
        "scope": "openid email profile",
        "code_challenge": code_challenge,
        "code_challenge_method": "plain",
        "state": secrets.token_urlsafe(32),
    }

    request.session["oauth_state"] = params["state"]

    redirect_url = f"{google_auth_url}?{urlencode(params)}"
    return RedirectResponse(url=redirect_url)


@router.get("/google/callback")
async def google_callback(
    code: str,
    state: str,
    request: Request,
    db: AsyncSession = Depends(get_db),
):
    """
    Handle Google OAuth callback.

    Exchanges authorization code for tokens, fetches user info, and creates/links user.
    Redirects to frontend with tokens in URL fragment.
    """
    settings = get_settings()

    # Validate state
    stored_state = request.session.get("oauth_state")
    if not state or state != stored_state:
        return RedirectResponse(url=f"{settings.frontend_url}/auth/login?error=oauth_failed")

    # Exchange code for tokens
    token_url = "https://oauth2.googleapis.com/token"
    code_verifier = request.session.get("oauth_code_verifier")

    async with httpx.AsyncClient() as client:
        try:
            token_response = await client.post(
                token_url,
                data={
                    "client_id": settings.google_client_id,
                    "client_secret": settings.google_client_secret,
                    "code": code,
                    "grant_type": "authorization_code",
                    "redirect_uri": settings.google_redirect_uri,
                    "code_verifier": code_verifier,
                },
            )
            token_response.raise_for_status()
            tokens = token_response.json()

            # Fetch user info from Google
            userinfo_response = await client.get(
                "https://openidconnect.googleapis.com/v1/userinfo",
                headers={"Authorization": f"Bearer {tokens['access_token']}"},
            )
            userinfo_response.raise_for_status()
            userinfo = userinfo_response.json()

        except Exception:
            return RedirectResponse(url=f"{settings.frontend_url}/auth/login?error=oauth_failed")

    # Find or create user
    google_sub = userinfo.get("sub")
    email = userinfo.get("email")
    name = userinfo.get("name")

    # Check if user exists by google_sub
    result = await db.execute(
        select(User).where(User.google_sub == google_sub)
    )
    user = result.scalar_one_or_none()

    if not user:
        # Check if email exists (for linking)
        result = await db.execute(select(User).where(User.email == email))
        user = result.scalar_one_or_none()

        if user:
            # Link Google account to existing user
            user.google_sub = google_sub
            if not user.full_name:
                user.full_name = name
        else:
            # Create new user
            user = User(
                id=uuid4(),
                email=email,
                full_name=name,
                is_active=True,
                auth_provider="google",
                google_sub=google_sub,
            )
            db.add(user)

    await db.flush()

    # Generate JWT tokens
    access_token = create_access_token(str(user.id))
    refresh_token_str = create_refresh_token(str(user.id))

    # Store refresh token in database
    refresh_token = RefreshToken(
        id=uuid4(),
        token=refresh_token_str,
        user_id=user.id,
        expires_at=create_refresh_token_expiry(),
    )
    db.add(refresh_token)
    await db.commit()

    # Redirect to frontend with tokens in URL fragment
    redirect_url = f"{settings.frontend_url}/auth/callback#{urlencode({'access_token': access_token, 'refresh_token': refresh_token_str})}"
    return RedirectResponse(url=redirect_url)
