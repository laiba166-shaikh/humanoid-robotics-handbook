import logging
import os
from contextlib import asynccontextmanager
from urllib.parse import urlparse

import cohere
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.sessions import SessionMiddleware
from fastapi.responses import JSONResponse, Response, StreamingResponse
from qdrant_client import AsyncQdrantClient

from chatkit.server import StreamingResult

from app.config import get_settings
from app.auth.security import decode_token
from app.database import Base, engine, async_session
from app.models.responses import ApiErrorResponse, ErrorCode, ResponseMeta
from app.services.chat import CohereChatService
from app.services.embeddings import CohereEmbedService
from app.services.query import QueryExpander
from app.services.reranker import CohereRerankService
from app.services.vectorstore import QdrantService
from app.chatkit.store import PostgresStore
from app.chatkit.server import TextbookChatKitServer

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

    # Initialize ChatKit server (requires GEMINI_API_KEY /OPENAI_API_KEY for Agents SDK)
    if settings.gemini_api_key:
        print('gemini')
        os.environ.setdefault("GEMINI_API_KEY", settings.gemini_api_key)

        chatkit_store = PostgresStore(session_factory=async_session)
        app.state.chatkit_server = TextbookChatKitServer(store=chatkit_store)
        logger.info("ChatKit server initialized")
    elif settings.openai_api_key:
        print('openai')
        os.environ.setdefault("OPENAI_API_KEY", settings.openai_api_key)

        chatkit_store = PostgresStore(session_factory=async_session)
        app.state.chatkit_server = TextbookChatKitServer(store=chatkit_store)
        logger.info("ChatKit server initialized")
    else:
        app.state.chatkit_server = None
        logger.warning("API_KEY not set — ChatKit server disabled")

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

# Build CORS origins: include explicit list + frontend_url + its base origin (scheme+host only)
# The browser always sends Origin as scheme+host (no path), so we must include the base origin
# extracted from FRONTEND_URL even when it contains a path like /humanoid-robotics-handbook.
_parsed = urlparse(settings.frontend_url)
_frontend_base_origin = f"{_parsed.scheme}://{_parsed.netloc}" if _parsed.netloc else ""
_allowed_origins = list({
    *settings.origins_list,
    settings.frontend_url,
    _frontend_base_origin,
} - {""})

app.add_middleware(
    CORSMiddleware,
    allow_origins=_allowed_origins,
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


@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    print("====Received request at /chatkit:==== ",request)  # Debug log to confirm endpoint is hit
    """ChatKit protocol endpoint — handles thread management, messages, and streaming."""
    server = request.app.state.chatkit_server
    if not server:
        return JSONResponse(
            status_code=503,
            content={"error": "ChatKit server not available. Check OPENAI_APqI_KEY."},
        )

    # Extract user_id from JWT (optional — anonymous fallback)
    user_id = "anonymous"
    auth_header = request.headers.get("authorization", "")
    if auth_header.startswith("Bearer "):
        token = auth_header[7:]
        decoded_user_id = decode_token(token)
        if decoded_user_id:
            user_id = decoded_user_id

    context = {"user_id": user_id, "app_state": request.app.state}

    result = await server.process(await request.body(), context=context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")


from app.api.health import router as health_router  # noqa: E402
from app.api.search import router as search_router  # noqa: E402
from app.api.chat import router as chat_router  # noqa: E402
from app.auth import router as auth_router  # noqa: E402

app.include_router(health_router)
app.include_router(search_router)
app.include_router(chat_router)
app.include_router(auth_router, prefix="/api")
