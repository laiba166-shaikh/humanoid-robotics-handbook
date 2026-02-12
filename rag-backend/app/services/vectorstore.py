import logging
import time
import uuid

from qdrant_client import AsyncQdrantClient
from qdrant_client.models import (
    Distance,
    FieldCondition,
    Filter,
    FilterSelector,
    MatchValue,
    PointStruct,
    Range,
    VectorParams,
)

from app.models.responses import ComponentHealth

logger = logging.getLogger(__name__)

VECTOR_DIM = 1024
NAMESPACE_DNS = uuid.NAMESPACE_DNS


def deterministic_id(lesson_id: str, section_title: str, chunk_index: int) -> str:
    return str(uuid.uuid5(NAMESPACE_DNS, f"{lesson_id}:{section_title}:{chunk_index}"))


class QdrantService:
    def __init__(self, client: AsyncQdrantClient, collection_name: str) -> None:
        self._client = client
        self._collection_name = collection_name

    async def ensure_collection(self) -> None:
        from qdrant_client.models import PayloadSchemaType

        collections = await self._client.get_collections()
        names = [c.name for c in collections.collections]
        if self._collection_name not in names:
            await self._client.create_collection(
                collection_name=self._collection_name,
                vectors_config=VectorParams(size=VECTOR_DIM, distance=Distance.COSINE),
            )
            logger.info("Created collection: %s", self._collection_name)
        else:
            logger.info("Collection already exists: %s", self._collection_name)

        indexed_fields = {
            "lesson_id": PayloadSchemaType.KEYWORD,
            "section_type": PayloadSchemaType.KEYWORD,
            "module": PayloadSchemaType.KEYWORD,
            "chapter": PayloadSchemaType.KEYWORD,
            "hardware_tier": PayloadSchemaType.INTEGER,
        }
        for field_name, field_type in indexed_fields.items():
            try:
                await self._client.create_payload_index(
                    collection_name=self._collection_name,
                    field_name=field_name,
                    field_schema=field_type,
                )
            except Exception:
                pass

    async def upsert_chunks(
        self, chunks_with_vectors: list[tuple[dict, list[float]]]
    ) -> None:
        points = []
        for payload, vector in chunks_with_vectors:
            point_id = deterministic_id(
                payload["lesson_id"],
                payload["section_title"],
                payload.get("chunk_index", 0),
            )
            points.append(
                PointStruct(id=point_id, vector=vector, payload=payload)
            )
        if points:
            await self._client.upsert(
                collection_name=self._collection_name, points=points
            )
            logger.info("Upserted %d points", len(points))

    async def delete_by_lesson(self, lesson_id: str) -> None:
        await self._client.delete(
            collection_name=self._collection_name,
            points_selector=FilterSelector(
                filter=Filter(
                    must=[
                        FieldCondition(
                            key="lesson_id", match=MatchValue(value=lesson_id)
                        )
                    ]
                )
            ),
        )
        logger.info("Deleted points for lesson: %s", lesson_id)

    async def search(
        self,
        vector: list[float],
        hardware_tier: int = 4,
        top_k: int = 5,
    ) -> list[dict]:
        results = await self._client.query_points(
            collection_name=self._collection_name,
            query=vector,
            query_filter=Filter(
                must=[
                    FieldCondition(
                        key="hardware_tier",
                        range=Range(lte=hardware_tier),
                    )
                ]
            ),
            limit=top_k,
            with_payload=True,
        )
        return [
            {**point.payload, "score": point.score}
            for point in results.points
        ]

    async def health_check(self) -> ComponentHealth:
        start = time.monotonic()
        try:
            await self._client.get_collections()
            latency = (time.monotonic() - start) * 1000
            return ComponentHealth(status="healthy", latency_ms=round(latency, 1))
        except Exception as e:
            latency = (time.monotonic() - start) * 1000
            return ComponentHealth(
                status="unhealthy",
                latency_ms=round(latency, 1),
                error=str(e),
            )
