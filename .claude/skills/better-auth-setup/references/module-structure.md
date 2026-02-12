# Auth Module Structure

File-by-file code patterns. Adapt imports and DB session to match the existing project.

---

## models.py

```python
import uuid
from datetime import datetime
from sqlalchemy import Column, String, Boolean, DateTime, ForeignKey, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from app.database import Base  # adapt import


class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    hashed_password = Column(String(255), nullable=True)  # null for OAuth-only users
    full_name = Column(String(255), nullable=True)
    is_active = Column(Boolean, default=True)
    auth_provider = Column(String(50), default="local")  # "local" | "google"
    google_sub = Column(String(255), unique=True, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    refresh_tokens = relationship("RefreshToken", back_populates="user", cascade="all, delete-orphan")


class RefreshToken(Base):
    __tablename__ = "refresh_tokens"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    token = Column(Text, unique=True, nullable=False, index=True)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), nullable=False)
    expires_at = Column(DateTime, nullable=False)
    revoked = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)

    user = relationship("User", back_populates="refresh_tokens")
```

---

## schemas.py

```python
from datetime import datetime
from uuid import UUID
from pydantic import BaseModel, EmailStr, field_validator
from app.auth.validators import validate_password


class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    full_name: str | None = None

    @field_validator("password")
    @classmethod
    def password_strength(cls, v: str) -> str:
        return validate_password(v)


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"


class UserResponse(BaseModel):
    id: UUID
    email: str
    full_name: str | None
    is_active: bool
    auth_provider: str
    created_at: datetime

    model_config = {"from_attributes": True}


class MessageResponse(BaseModel):
    message: str
```

---

## validators.py

```python
import re
from fastapi import HTTPException, status


def validate_password(password: str) -> str:
    """Validate password meets strength requirements.

    Rules: min 8 chars, 1 uppercase, 1 digit, 1 special character.
    """
    if len(password) < 8:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Password must be at least 8 characters long.",
        )
    if not re.search(r"[A-Z]", password):
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Password must contain at least one uppercase letter.",
        )
    if not re.search(r"\d", password):
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Password must contain at least one digit.",
        )
    if not re.search(r"[!@#$%^&*()_+\-=]", password):
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Password must contain at least one special character (!@#$%^&*()_+-=).",
        )
    return password
```

---

## security.py

```python
from datetime import datetime, timedelta, timezone
from uuid import UUID

from jose import JWTError, jwt
from passlib.context import CryptContext
from fastapi import HTTPException, status

from app.config import settings  # adapt import

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def hash_password(password: str) -> str:
    return pwd_context.hash(password)


def verify_password(plain: str, hashed: str) -> bool:
    return pwd_context.verify(plain, hashed)


def create_access_token(user_id: UUID, email: str) -> str:
    expire = datetime.now(timezone.utc) + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    payload = {"sub": str(user_id), "email": email, "exp": expire, "type": "access"}
    return jwt.encode(payload, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)


def create_refresh_token(user_id: UUID) -> tuple[str, datetime]:
    expire = datetime.now(timezone.utc) + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
    payload = {"sub": str(user_id), "exp": expire, "type": "refresh"}
    token = jwt.encode(payload, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)
    return token, expire


def decode_token(token: str) -> dict:
    try:
        return jwt.decode(token, settings.JWT_SECRET_KEY, algorithms=[settings.JWT_ALGORITHM])
    except JWTError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid or expired token.")
```

---

## dependencies.py

```python
from uuid import UUID

from fastapi import Depends, HTTPException, Request, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_session  # adapt import
from app.auth.models import User
from app.auth.security import decode_token


async def get_current_user(request: Request, db: AsyncSession = Depends(get_session)) -> User:
    token = request.cookies.get("access_token")
    if not token:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Not authenticated.")

    payload = decode_token(token)
    if payload.get("type") != "access":
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token type.")

    user_id = UUID(payload["sub"])
    result = await db.execute(select(User).where(User.id == user_id, User.is_active == True))
    user = result.scalar_one_or_none()
    if not user:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="User not found or inactive.")
    return user
```

---

## router.py

```python
from datetime import datetime, timezone

from fastapi import APIRouter, Depends, HTTPException, Response, Request, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_session  # adapt import
from app.auth import models, schemas, security
from app.auth.dependencies import get_current_user
from app.auth.google_oidc import google_oidc_router

auth_router = APIRouter()
auth_router.include_router(google_oidc_router)


def _set_tokens(response: Response, access_token: str, refresh_token: str) -> None:
    response.set_cookie("access_token", access_token, httponly=True, secure=True, samesite="lax", path="/")
    response.set_cookie("refresh_token", refresh_token, httponly=True, secure=True, samesite="lax", path="/auth")


@auth_router.post("/signup", response_model=schemas.UserResponse, status_code=status.HTTP_201_CREATED)
async def signup(body: schemas.SignupRequest, db: AsyncSession = Depends(get_session)):
    existing = await db.execute(select(models.User).where(models.User.email == body.email))
    if existing.scalar_one_or_none():
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail="Email already registered.")

    user = models.User(
        email=body.email,
        hashed_password=security.hash_password(body.password),
        full_name=body.full_name,
        auth_provider="local",
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


@auth_router.post("/login", response_model=schemas.TokenResponse)
async def login(body: schemas.LoginRequest, response: Response, db: AsyncSession = Depends(get_session)):
    result = await db.execute(select(models.User).where(models.User.email == body.email))
    user = result.scalar_one_or_none()

    if not user or not user.hashed_password or not security.verify_password(body.password, user.hashed_password):
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid email or password.")
    if not user.is_active:
        raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="Account is deactivated.")

    access = security.create_access_token(user.id, user.email)
    refresh, expires = security.create_refresh_token(user.id)

    token_record = models.RefreshToken(token=refresh, user_id=user.id, expires_at=expires)
    db.add(token_record)
    await db.commit()

    _set_tokens(response, access, refresh)
    return schemas.TokenResponse(access_token=access)


@auth_router.post("/refresh", response_model=schemas.TokenResponse)
async def refresh(request: Request, response: Response, db: AsyncSession = Depends(get_session)):
    old_token = request.cookies.get("refresh_token")
    if not old_token:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="No refresh token.")

    payload = security.decode_token(old_token)
    if payload.get("type") != "refresh":
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token type.")

    result = await db.execute(
        select(models.RefreshToken).where(
            models.RefreshToken.token == old_token,
            models.RefreshToken.revoked == False,
            models.RefreshToken.expires_at > datetime.now(timezone.utc),
        )
    )
    record = result.scalar_one_or_none()
    if not record:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Refresh token revoked or expired.")

    # Rotate refresh token
    record.revoked = True
    user_result = await db.execute(select(models.User).where(models.User.id == record.user_id))
    user = user_result.scalar_one()

    new_access = security.create_access_token(user.id, user.email)
    new_refresh, expires = security.create_refresh_token(user.id)
    db.add(models.RefreshToken(token=new_refresh, user_id=user.id, expires_at=expires))
    await db.commit()

    _set_tokens(response, new_access, new_refresh)
    return schemas.TokenResponse(access_token=new_access)


@auth_router.post("/logout", response_model=schemas.MessageResponse)
async def logout(request: Request, response: Response, db: AsyncSession = Depends(get_session)):
    refresh_token = request.cookies.get("refresh_token")
    if refresh_token:
        result = await db.execute(
            select(models.RefreshToken).where(models.RefreshToken.token == refresh_token)
        )
        record = result.scalar_one_or_none()
        if record:
            record.revoked = True
            await db.commit()

    response.delete_cookie("access_token", path="/")
    response.delete_cookie("refresh_token", path="/auth")
    return schemas.MessageResponse(message="Logged out.")


@auth_router.get("/me", response_model=schemas.UserResponse)
async def me(user: models.User = Depends(get_current_user)):
    return user
```

---

## __init__.py

```python
from app.auth.router import auth_router

__all__ = ["auth_router"]
```
