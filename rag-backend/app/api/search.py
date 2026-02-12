import time

from fastapi import APIRouter, Request

from app.models.requests import SearchRequest
from app.models.responses import (
    ApiResponse,
    ResponseMeta,
    SearchResponse,
    SearchResultItem,
)

router = APIRouter(tags=["Search"])


@router.post("/api/search")
async def search_content(
    body: SearchRequest, request: Request
) -> ApiResponse[SearchResponse]:
    start = time.monotonic()

    query_expander = request.app.state.query_expander
    embed_service = request.app.state.embed_service
    vectorstore = request.app.state.vectorstore
    reranker = request.app.state.reranker

    expanded_query, was_expanded = query_expander.expand(body.query)

    vector = await embed_service.embed_query(expanded_query)

    raw_results = await vectorstore.search(
        vector=vector, hardware_tier=body.hardware_tier, top_k=body.top_k * 2
    )

    reranked = await reranker.rerank(
        query=expanded_query, documents=raw_results, top_n=body.top_k
    )

    results = [
        SearchResultItem(
            content=r.get("content", "")[:500],
            score=round(r.get("score", 0), 4),
            lesson_id=r.get("lesson_id", ""),
            lesson_title=r.get("lesson_title", ""),
            section_title=r.get("section_title", ""),
            section_type=r.get("section_type", "instructional"),
            module=r.get("module", ""),
            chapter=r.get("chapter", ""),
            hardware_tier=r.get("hardware_tier", 1),
        )
        for r in reranked
    ]

    if not results:
        data = SearchResponse(
            query=expanded_query,
            query_expanded=was_expanded,
            results=[],
        )
    else:
        data = SearchResponse(
            query=expanded_query,
            query_expanded=was_expanded,
            results=results,
        )

    latency = (time.monotonic() - start) * 1000
    return ApiResponse[SearchResponse](
        success=True,
        data=data,
        meta=ResponseMeta(
            latency_ms=round(latency, 1), source_count=len(results)
        ),
    )
