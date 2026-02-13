from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, EmailStr, Field


# Request schemas
class SignupRequest(BaseModel):
    """Request body for POST /api/auth/signup"""
    email: EmailStr
    password: str
    full_name: Optional[str] = None


class LoginRequest(BaseModel):
    """Request body for POST /api/auth/login"""
    email: EmailStr
    password: str


class RefreshRequest(BaseModel):
    """Request body for POST /api/auth/refresh"""
    refresh_token: str


class LogoutRequest(BaseModel):
    """Request body for POST /api/auth/logout"""
    refresh_token: str


# Response schemas
class UserResponse(BaseModel):
    """User data in responses"""
    id: UUID
    email: str
    full_name: Optional[str]
    auth_provider: str
    is_active: bool
    created_at: datetime

    class Config:
        from_attributes = True


class TokenResponse(BaseModel):
    """Token data in responses"""
    access_token: str
    refresh_token: str
    token_type: str = "bearer"


class AuthResponse(BaseModel):
    """Response for signup/login endpoints"""
    user: UserResponse
    access_token: str
    refresh_token: str


class MessageResponse(BaseModel):
    """Simple message response"""
    message: str


# Constitution response wrapper
class ApiResponse(BaseModel):
    """Standard API response format per constitution"""
    success: bool
    data: Optional[dict] = None
    error: Optional[str] = None
    meta: dict = Field(default_factory=dict)
