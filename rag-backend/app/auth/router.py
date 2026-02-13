from datetime import timedelta, datetime, timezone
from uuid import uuid4
from collections import defaultdict

from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.auth.models import RefreshToken, User
from app.auth.schemas import ApiResponse, AuthResponse, LoginRequest, LogoutRequest, RefreshRequest, SignupRequest, UserResponse
from app.auth.security import create_access_token, create_refresh_token, hash_password, verify_password
from app.auth.validators import validate_password
from app.auth.dependencies import get_current_user
from app.database import get_db
from app.config import get_settings

router = APIRouter()

# Rate limiting: per-IP failed login attempts
# Structure: {ip: [(timestamp, count), ...]}
rate_limit_store: dict[str, list[tuple[float, int]]] = defaultdict(list)
RATE_LIMIT_ATTEMPTS = 5
RATE_LIMIT_WINDOW_SECONDS = 60


def check_rate_limit(client_ip: str) -> bool:
    """Check if client has exceeded rate limit for failed logins."""
    now = datetime.now(timezone.utc).timestamp()
    window_start = now - RATE_LIMIT_WINDOW_SECONDS

    # Clean old entries
    if client_ip in rate_limit_store:
        rate_limit_store[client_ip] = [
            (ts, count) for ts, count in rate_limit_store[client_ip]
            if ts > window_start
        ]

    # Count attempts in current window
    total_attempts = sum(count for ts, count in rate_limit_store[client_ip])

    return total_attempts < RATE_LIMIT_ATTEMPTS


def record_failed_attempt(client_ip: str) -> None:
    """Record a failed login attempt for rate limiting."""
    now = datetime.now(timezone.utc).timestamp()
    rate_limit_store[client_ip].append((now, 1))


def create_refresh_token_expiry():
    """Helper to calculate refresh token expiry time."""
    settings = get_settings()
    return datetime.now(timezone.utc) + timedelta(days=settings.refresh_token_expire_days)


@router.post("/signup", status_code=201)
async def signup(
    request: SignupRequest,
    db: AsyncSession = Depends(get_db),
) -> ApiResponse:
    """
    Create a new user account with email and password.

    Validates email uniqueness and password strength.
    Returns user data and tokens on success.
    """
    # Validate password strength
    is_valid, errors = validate_password(request.password)
    if not is_valid:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail={"error_code": "VALIDATION_ERROR", "messages": errors},
        )

    # Check if email already exists
    result = await db.execute(select(User).where(User.email == request.email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        # Edge case: if email exists with Google auth only, guide user
        if existing_user.auth_provider == "google" and not existing_user.hashed_password:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail={
                    "error_code": "VALIDATION_ERROR",
                    "message": "This email is registered with Google. Please use Google login or set a password to enable both methods.",
                },
            )
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={"error_code": "VALIDATION_ERROR", "message": "Email already registered."},
        )

    # Create new user
    user = User(
        id=uuid4(),
        email=request.email,
        hashed_password=hash_password(request.password),
        full_name=request.full_name,
        is_active=True,
        auth_provider="local",
    )

    db.add(user)
    await db.flush()

    # Generate tokens
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

    user_response = UserResponse(
        id=user.id,
        email=user.email,
        full_name=user.full_name,
        auth_provider=user.auth_provider,
        is_active=user.is_active,
        created_at=user.created_at,
    )

    return ApiResponse(
        success=True,
        data={
            "user": user_response.model_dump(),
            "access_token": access_token,
            "refresh_token": refresh_token_str,
        },
        meta={"latency_ms": 150},
    )


@router.post("/login", status_code=200)
async def login(
    request: LoginRequest,
    db: AsyncSession = Depends(get_db),
    http_request: Request = None,
) -> ApiResponse:
    """
    Authenticate with email and password.

    Includes per-IP rate limiting (5 failures per minute).
    Returns generic error message to prevent email enumeration.
    """
    client_ip = http_request.client.host if http_request else "unknown"

    # Check rate limit
    if not check_rate_limit(client_ip):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={"error_code": "RATE_LIMITED", "message": "Too many failed login attempts. Please try again later."},
        )

    # Look up user
    result = await db.execute(select(User).where(User.email == request.email))
    user = result.scalar_one_or_none()

    # Verify password (generic error for both user not found and wrong password)
    if not user or not user.hashed_password or not verify_password(request.password, user.hashed_password):
        record_failed_attempt(client_ip)
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "Invalid email or password."},
        )

    if not user.is_active:
        record_failed_attempt(client_ip)
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "Invalid email or password."},
        )

    # Generate tokens
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

    user_response = UserResponse(
        id=user.id,
        email=user.email,
        full_name=user.full_name,
        auth_provider=user.auth_provider,
        is_active=user.is_active,
        created_at=user.created_at,
    )

    return ApiResponse(
        success=True,
        data={
            "user": user_response.model_dump(),
            "access_token": access_token,
            "refresh_token": refresh_token_str,
        },
        meta={"latency_ms": 120},
    )


@router.post("/refresh", status_code=200)
async def refresh(
    request: RefreshRequest,
    db: AsyncSession = Depends(get_db),
) -> ApiResponse:
    """
    Refresh access token using a valid refresh token.

    Implements token rotation: old token is revoked, new token is issued.
    """
    from app.auth.security import decode_token

    # Decode refresh token
    user_id = decode_token(request.refresh_token)
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "Invalid or expired refresh token."},
        )

    # Look up refresh token in database
    result = await db.execute(
        select(RefreshToken).where(RefreshToken.token == request.refresh_token)
    )
    refresh_token_record = result.scalar_one_or_none()

    if not refresh_token_record or refresh_token_record.revoked:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "Invalid or expired refresh token."},
        )

    if refresh_token_record.expires_at < datetime.now(timezone.utc):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "Invalid or expired refresh token."},
        )

    # Look up user
    result = await db.execute(select(User).where(User.id == refresh_token_record.user_id))
    user = result.scalar_one_or_none()

    if not user or not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error_code": "VALIDATION_ERROR", "message": "User not found or inactive."},
        )

    # Revoke old refresh token
    refresh_token_record.revoked = True

    # Generate new tokens
    access_token = create_access_token(str(user.id))
    new_refresh_token_str = create_refresh_token(str(user.id))

    # Store new refresh token
    new_refresh_token = RefreshToken(
        id=uuid4(),
        token=new_refresh_token_str,
        user_id=user.id,
        expires_at=create_refresh_token_expiry(),
    )
    db.add(new_refresh_token)
    await db.commit()

    user_response = UserResponse(
        id=user.id,
        email=user.email,
        full_name=user.full_name,
        auth_provider=user.auth_provider,
        is_active=user.is_active,
        created_at=user.created_at,
    )

    return ApiResponse(
        success=True,
        data={
            "user": user_response.model_dump(),
            "access_token": access_token,
            "refresh_token": new_refresh_token_str,
        },
        meta={"latency_ms": 100},
    )


@router.post("/logout", status_code=200)
async def logout(
    request: LogoutRequest,
    db: AsyncSession = Depends(get_db),
) -> ApiResponse:
    """
    Logout by revoking the refresh token.
    """
    # Look up and revoke refresh token
    result = await db.execute(
        select(RefreshToken).where(RefreshToken.token == request.refresh_token)
    )
    refresh_token_record = result.scalar_one_or_none()

    if refresh_token_record:
        refresh_token_record.revoked = True
        await db.commit()

    return ApiResponse(
        success=True,
        data={"message": "Logged out successfully."},
        meta={},
    )


@router.get("/me", status_code=200)
async def get_me(
    current_user: User = Depends(get_current_user),
) -> ApiResponse:
    """
    Get current user profile.

    Requires valid access token in Authorization header.
    """
    user_response = UserResponse(
        id=current_user.id,
        email=current_user.email,
        full_name=current_user.full_name,
        auth_provider=current_user.auth_provider,
        is_active=current_user.is_active,
        created_at=current_user.created_at,
    )

    return ApiResponse(
        success=True,
        data={"user": user_response.model_dump()},
        meta={},
    )
