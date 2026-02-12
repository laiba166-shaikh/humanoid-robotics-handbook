"""CLI entry point for the ingestion pipeline.

Usage:
    python -m app.ingest --docs-path ../humanoid-textbook/docs/module-1-ros2
    python -m app.ingest --docs-path ../humanoid-textbook/docs/module-1-ros2 --selective
"""

import argparse
import logging
import sys

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient

from app.config import get_settings
from app.ingestion.pipeline import IngestionPipeline


def main() -> None:
    parser = argparse.ArgumentParser(description="Ingest textbook content into Qdrant")
    parser.add_argument(
        "--docs-path",
        required=True,
        help="Path to the docs directory containing lesson .md files",
    )
    parser.add_argument(
        "--selective",
        action="store_true",
        help="Only process new or modified files (uses SHA-256 manifest)",
    )
    args = parser.parse_args()

    load_dotenv()
    settings = get_settings()

    logging.basicConfig(
        level=getattr(logging, settings.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cohere_client = cohere.ClientV2(api_key=settings.cohere_api_key)
    qdrant_client = QdrantClient(
        url=settings.qdrant_url, api_key=settings.qdrant_api_key
    )

    _ensure_collection(qdrant_client, settings.collection_name)

    pipeline = IngestionPipeline(
        cohere_client=cohere_client,
        qdrant_client=qdrant_client,
        collection_name=settings.collection_name,
    )

    report = pipeline.run(docs_path=args.docs_path, selective=args.selective)

    print("\n=== Ingestion Report ===")
    print(f"Total files:     {report.total_files}")
    print(f"Processed:       {report.processed_files}")
    print(f"Skipped:         {len(report.skipped_files)}")
    for sf in report.skipped_files:
        print(f"  - {sf.file_path}: {sf.reason}")
    print(f"Total chunks:    {report.total_chunks}")
    print(f"Chunks by type:  {report.chunks_by_type}")
    print(f"Enriched:        {report.enriched_count}")
    print(f"Split:           {report.split_count}")
    print(f"Embed API calls: {report.embed_api_calls}")
    print(f"Duration:        {report.duration_seconds}s")
    if report.errors:
        print(f"Errors:          {len(report.errors)}")
        for err in report.errors:
            print(f"  - {err}")
        sys.exit(1)
    else:
        print("Status:          SUCCESS")


def _ensure_collection(client: QdrantClient, name: str) -> None:
    from qdrant_client.models import Distance, PayloadSchemaType, VectorParams

    collections = client.get_collections()
    names = [c.name for c in collections.collections]
    if name not in names:
        client.create_collection(
            collection_name=name,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
        )
        print(f"Created Qdrant collection: {name}")
    else:
        print(f"Qdrant collection exists: {name}")

    indexed_fields = {
        "lesson_id": PayloadSchemaType.KEYWORD,
        "section_type": PayloadSchemaType.KEYWORD,
        "module": PayloadSchemaType.KEYWORD,
        "chapter": PayloadSchemaType.KEYWORD,
        "hardware_tier": PayloadSchemaType.INTEGER,
    }
    for field_name, field_type in indexed_fields.items():
        try:
            client.create_payload_index(
                collection_name=name,
                field_name=field_name,
                field_schema=field_type,
            )
        except Exception:
            pass


if __name__ == "__main__":
    main()
