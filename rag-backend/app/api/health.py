import asyncio
import time

from fastapi import APIRouter, Request

from app.models.responses import ApiResponse, HealthResponse, ResponseMeta

router = APIRouter(tags=["Health"])


@router.get("/api/health")
async def health_check(request: Request) -> ApiResponse[HealthResponse]:
    start = time.monotonic()
    embed_service = request.app.state.embed_service
    vectorstore = request.app.state.vectorstore

    cohere_health, qdrant_health = await asyncio.gather(
        embed_service.health_check(),
        vectorstore.health_check(),
    )

    components = {"cohere": cohere_health, "qdrant": qdrant_health}

    all_healthy = all(c.status == "healthy" for c in components.values())
    any_healthy = any(c.status == "healthy" for c in components.values())

    if all_healthy:
        status = "healthy"
    elif any_healthy:
        status = "degraded"
    else:
        status = "unhealthy"

    latency = (time.monotonic() - start) * 1000
    data = HealthResponse(status=status, components=components)

    response = ApiResponse[HealthResponse](
        success=status != "unhealthy",
        data=data,
        meta=ResponseMeta(latency_ms=round(latency, 1)),
    )

    if status != "healthy":
        from fastapi.responses import JSONResponse

        return JSONResponse(
            status_code=503, content=response.model_dump(mode="json")
        )

    return response
