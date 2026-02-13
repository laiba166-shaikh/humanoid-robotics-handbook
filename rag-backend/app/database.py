import re
import ssl as ssl_module

from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, async_sessionmaker
from sqlalchemy.orm import declarative_base
from app.config import get_settings

settings = get_settings()

# Ensure the URL uses the asyncpg driver
_db_url = settings.database_url
if _db_url.startswith("postgresql://"):
    _db_url = _db_url.replace("postgresql://", "postgresql+asyncpg://", 1)

# Detect if SSL was requested before stripping libpq-specific params
_needs_ssl = "sslmode=require" in _db_url or "sslmode=verify" in _db_url

# Remove libpq-specific params not supported by asyncpg
_db_url = re.sub(r"[&?]channel_binding=[^&]*", "", _db_url)
_db_url = re.sub(r"[&?]sslmode=[^&]*", "", _db_url)

# Build connect_args for asyncpg
_connect_args = {}
if _needs_ssl:
    _ssl_ctx = ssl_module.create_default_context()
    _ssl_ctx.check_hostname = False
    _ssl_ctx.verify_mode = ssl_module.CERT_NONE
    _connect_args["ssl"] = _ssl_ctx

# Create async engine for Neon Postgres with asyncpg driver
engine = create_async_engine(
    _db_url,
    echo=False,
    future=True,
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10,
    connect_args=_connect_args,
)

# Create async session factory
async_session = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autoflush=False,
    autocommit=False,
)

# Declarative base for all models
Base = declarative_base()


async def get_db() -> AsyncSession:
    """Dependency for FastAPI to get async database session."""
    async with async_session() as session:
        yield session
