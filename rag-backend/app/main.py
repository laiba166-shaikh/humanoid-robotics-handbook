import logging
from contextlib import asynccontextmanager

import cohere
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.sessions import SessionMiddleware
from fastapi.responses import JSONResponse
from qdrant_client import AsyncQdrantClient

from app.config import get_settings
from app.database import Base, engine
from app.models.responses import ApiErrorResponse, ErrorCode, ResponseMeta
from app.services.chat import CohereChatService
from app.services.embeddings import CohereEmbedService
from app.services.query import QueryExpander
from app.services.reranker import CohereRerankService
from app.services.vectorstore import QdrantService

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    settings = get_settings()

    logging.basicConfig(
        level=getattr(logging, settings.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Initialize database tables
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    cohere_client = cohere.AsyncClientV2(api_key=settings.cohere_api_key)
    qdrant_client = AsyncQdrantClient(
        url=settings.qdrant_url, api_key=settings.qdrant_api_key
    )

    app.state.embed_service = CohereEmbedService(cohere_client)
    app.state.vectorstore = QdrantService(qdrant_client, settings.collection_name)
    app.state.chat_service = CohereChatService(cohere_client)
    app.state.reranker = CohereRerankService(cohere_client, settings.rerank_enabled)
    app.state.query_expander = QueryExpander()

    await app.state.vectorstore.ensure_collection()

    yield

    await qdrant_client.close()
    await engine.dispose()


app = FastAPI(
    title="RAG Backend — Physical AI Textbook",
    version="1.0.0",
    description="RAG-powered search and chat API for the Physical AI & Humanoid Robotics Handbook.",
    lifespan=lifespan,
)

settings = get_settings()

# SessionMiddleware for Google OAuth state storage (must be before CORS)
app.add_middleware(
    SessionMiddleware,
    secret_key=settings.jwt_secret_key,
    session_cookie="session",
    max_age=3600,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.origins_list + [settings.frontend_url],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    logger.error("Unhandled exception: %s", exc, exc_info=True)

    if "cohere" in type(exc).__module__.lower() if hasattr(type(exc), "__module__") else False:
        error_response = ApiErrorResponse(
            error=f"Embedding/chat service error: {exc}",
            error_code=ErrorCode.SERVICE_UNAVAILABLE,
        )
        return JSONResponse(status_code=503, content=error_response.model_dump(mode="json"))

    if "qdrant" in type(exc).__module__.lower() if hasattr(type(exc), "__module__") else False:
        error_response = ApiErrorResponse(
            error=f"Vector store error: {exc}",
            error_code=ErrorCode.SERVICE_UNAVAILABLE,
        )
        return JSONResponse(status_code=503, content=error_response.model_dump(mode="json"))

    error_response = ApiErrorResponse(
        error="An internal error occurred",
        error_code=ErrorCode.INTERNAL_ERROR,
    )
    return JSONResponse(status_code=500, content=error_response.model_dump(mode="json"))


from app.api.health import router as health_router  # noqa: E402
from app.api.search import router as search_router  # noqa: E402
from app.api.chat import router as chat_router  # noqa: E402
from app.auth import router as auth_router  # noqa: E402

app.include_router(health_router)
app.include_router(search_router)
app.include_router(chat_router)
app.include_router(auth_router, prefix="/api")
