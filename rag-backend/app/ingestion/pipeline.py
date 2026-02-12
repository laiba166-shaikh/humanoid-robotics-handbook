import hashlib
import json
import logging
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from app.ingestion.chunker import AdaptiveChunker
from app.ingestion.parser import parse_lesson_file
from app.models.domain import IngestionReport, SkippedFile

logger = logging.getLogger(__name__)

NAMESPACE_DNS = uuid.NAMESPACE_DNS
BATCH_SIZE = 96
MANIFEST_FILENAME = ".ingestion-manifest.json"


class IngestionPipeline:
    """Orchestrates the ingestion of lesson markdown files into Qdrant.

    Accepts raw Cohere and Qdrant clients (sync or async) via duck typing.
    The pipeline calls .embed() and .upsert() on the clients directly.
    """

    def __init__(self, cohere_client, qdrant_client, collection_name: str) -> None:
        self._cohere = cohere_client
        self._qdrant = qdrant_client
        self._collection_name = collection_name
        self._chunker = AdaptiveChunker()

    def run(self, docs_path: str, selective: bool = False) -> IngestionReport:
        start = time.monotonic()
        report = IngestionReport()

        docs_dir = Path(docs_path)
        if not docs_dir.exists():
            report.errors.append(f"Docs path does not exist: {docs_path}")
            return report

        md_files = sorted(
            f
            for f in docs_dir.rglob("*.md")
            if f.name.lower() != "readme.md"
        )
        report.total_files = len(md_files)

        manifest = _load_manifest(docs_dir) if selective else {}
        new_manifest: dict[str, dict] = {}

        all_chunks = []

        for md_file in md_files:
            rel_path = str(md_file.relative_to(docs_dir))
            file_hash = _hash_file(md_file)

            if selective and rel_path in manifest:
                if manifest[rel_path].get("content_hash") == file_hash:
                    logger.info("Skipping unchanged file: %s", rel_path)
                    new_manifest[rel_path] = manifest[rel_path]
                    continue

            result = parse_lesson_file(md_file)
            if result is None:
                report.skipped_files.append(
                    SkippedFile(file_path=rel_path, reason="invalid or missing frontmatter")
                )
                continue

            frontmatter, sections = result
            chunks = self._chunker.create_chunks(sections, frontmatter)

            self._delete_by_lesson(frontmatter.id)

            for chunk in chunks:
                chunk_type = chunk.section_type.value
                report.chunks_by_type[chunk_type] = (
                    report.chunks_by_type.get(chunk_type, 0) + 1
                )
                if chunk.enrichment_strategy == "context_injection":
                    report.enriched_count += 1
                elif chunk.enrichment_strategy == "overlapping_windows":
                    report.split_count += 1

            all_chunks.extend(chunks)
            report.processed_files += 1

            new_manifest[rel_path] = {
                "content_hash": file_hash,
                "last_ingested_at": datetime.now(timezone.utc).isoformat(),
                "chunk_count": len(chunks),
            }

        if all_chunks:
            embed_calls = self._embed_and_upsert(all_chunks)
            report.embed_api_calls = embed_calls

        report.total_chunks = len(all_chunks)
        report.duration_seconds = round(time.monotonic() - start, 2)

        if selective or new_manifest:
            _save_manifest(docs_dir, new_manifest)

        return report

    def _delete_by_lesson(self, lesson_id: str) -> None:
        from qdrant_client.models import (
            FieldCondition,
            Filter,
            FilterSelector,
            MatchValue,
        )

        self._qdrant.delete(
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
        logger.info("Deleted old chunks for lesson: %s", lesson_id)

    def _embed_and_upsert(self, chunks) -> int:
        from qdrant_client.models import PointStruct

        texts = [c.content for c in chunks]
        all_vectors: list[list[float]] = []
        api_calls = 0

        for i in range(0, len(texts), BATCH_SIZE):
            batch = texts[i : i + BATCH_SIZE]
            response = self._cohere.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document",
                embedding_types=["float"],
            )
            all_vectors.extend(response.embeddings.float_)
            api_calls += 1

        points = []
        for chunk, vector in zip(chunks, all_vectors):
            point_id = str(
                uuid.uuid5(
                    NAMESPACE_DNS,
                    f"{chunk.lesson_id}:{chunk.section_title}:{chunk.chunk_index}",
                )
            )
            payload = chunk.model_dump()
            payload.pop("content", None)
            payload["content"] = chunk.content
            payload["section_type"] = chunk.section_type.value

            points.append(PointStruct(id=point_id, vector=vector, payload=payload))

        if points:
            batch_size = 100
            for i in range(0, len(points), batch_size):
                self._qdrant.upsert(
                    collection_name=self._collection_name,
                    points=points[i : i + batch_size],
                )
            logger.info("Upserted %d points to Qdrant", len(points))

        return api_calls


def _hash_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_manifest(docs_dir: Path) -> dict:
    manifest_path = docs_dir / MANIFEST_FILENAME
    if manifest_path.exists():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            return data.get("files", {})
        except (json.JSONDecodeError, KeyError):
            return {}
    return {}


def _save_manifest(docs_dir: Path, files: dict) -> None:
    manifest_path = docs_dir / MANIFEST_FILENAME
    data = {
        "files": files,
        "last_run": datetime.now(timezone.utc).isoformat(),
    }
    manifest_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    logger.info("Saved ingestion manifest: %s", manifest_path)
