import time

from fastapi import APIRouter, Request

from app.models.requests import ChatRequest
from app.models.responses import (
    ApiResponse,
    ApiErrorResponse,
    ChatResponse,
    ErrorCode,
    ResponseMeta,
)

router = APIRouter(tags=["Chat"])


@router.post("/api/chat")
async def chat_with_textbook(
    body: ChatRequest, request: Request
) -> ApiResponse[ChatResponse]:
    start = time.monotonic()

    query_expander = request.app.state.query_expander
    embed_service = request.app.state.embed_service
    vectorstore = request.app.state.vectorstore
    reranker = request.app.state.reranker
    chat_service = request.app.state.chat_service

    try:
        expanded_query, _ = query_expander.expand(body.query)

        vector = await embed_service.embed_query(expanded_query)

        raw_results = await vectorstore.search(
            vector=vector, hardware_tier=body.hardware_tier, top_k=body.top_k * 2
        )

        reranked = await reranker.rerank(
            query=expanded_query, documents=raw_results, top_n=body.top_k
        )

        answer, sources = await chat_service.generate(
            query=body.query, context_chunks=reranked
        )

        data = ChatResponse(answer=answer, sources=sources)
        latency = (time.monotonic() - start) * 1000

        return ApiResponse[ChatResponse](
            success=True,
            data=data,
            meta=ResponseMeta(
                latency_ms=round(latency, 1), source_count=len(sources)
            ),
        )
    except Exception as e:
        latency = (time.monotonic() - start) * 1000
        from fastapi.responses import JSONResponse

        error_response = ApiErrorResponse(
            error=f"Service unavailable: {e}",
            error_code=ErrorCode.SERVICE_UNAVAILABLE,
            meta=ResponseMeta(latency_ms=round(latency, 1)),
        )
        return JSONResponse(
            status_code=503, content=error_response.model_dump(mode="json")
        )
