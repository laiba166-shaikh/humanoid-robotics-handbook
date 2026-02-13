"""Authentication module for user signup, login, and OAuth."""

from fastapi import APIRouter

from app.auth.router import router as auth_routes
from app.auth.google_oidc import router as google_routes

router = APIRouter(prefix="/auth", tags=["auth"])
router.include_router(auth_routes)
router.include_router(google_routes)

__all__ = ["router"]
