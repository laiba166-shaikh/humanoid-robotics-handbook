import logging
import time

from cohere import AsyncClientV2

from app.models.responses import ComponentHealth

logger = logging.getLogger(__name__)

EMBED_MODEL = "embed-english-v3.0"
BATCH_SIZE = 96


class CohereEmbedService:
    def __init__(self, client: AsyncClientV2) -> None:
        self._client = client

    async def embed_query(self, text: str) -> list[float]:
        response = await self._client.embed(
            texts=[text],
            model=EMBED_MODEL,
            input_type="search_query",
            embedding_types=["float"],
        )
        return response.embeddings.float_[0]

    async def embed_documents(self, texts: list[str]) -> list[list[float]]:
        all_embeddings: list[list[float]] = []
        api_calls = 0
        for i in range(0, len(texts), BATCH_SIZE):
            batch = texts[i : i + BATCH_SIZE]
            response = await self._client.embed(
                texts=batch,
                model=EMBED_MODEL,
                input_type="search_document",
                embedding_types=["float"],
            )
            all_embeddings.extend(response.embeddings.float_)
            api_calls += 1
        logger.info("Embedded %d texts in %d API calls", len(texts), api_calls)
        return all_embeddings

    async def health_check(self) -> ComponentHealth:
        start = time.monotonic()
        try:
            await self._client.embed(
                texts=["health check"],
                model=EMBED_MODEL,
                input_type="search_query",
                embedding_types=["float"],
            )
            latency = (time.monotonic() - start) * 1000
            return ComponentHealth(status="healthy", latency_ms=round(latency, 1))
        except Exception as e:
            latency = (time.monotonic() - start) * 1000
            return ComponentHealth(
                status="unhealthy",
                latency_ms=round(latency, 1),
                error=str(e),
            )
