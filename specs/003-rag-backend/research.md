# Research: RAG Backend

**Feature**: 003-rag-backend | **Date**: 2026-02-11

## Decision 1: Embedding Provider — Cohere `embed-english-v3.0`

**Decision**: Use Cohere `embed-english-v3.0` (1024 dimensions) with `input_type` differentiation.

**Rationale**: Single provider for embed + chat + rerank. Free tier available. 1024-dim vectors provide highest English retrieval quality on Cohere's lineup.

**Key Technical Details**:
- Endpoint: `co.embed(texts=[], model="embed-english-v3.0", input_type="search_document"|"search_query")`
- Max 96 texts per batch request
- Must use `input_type="search_document"` for ingestion, `input_type="search_query"` for queries
- Output: 1024-dim float vectors
- Python SDK: `cohere.ClientV2(api_key=...)` for latest API

**Alternatives Considered**:
- OpenAI `text-embedding-3-small` (1536 dim) — higher cost, separate provider from chat
- Cohere `embed-english-light-v3.0` (384 dim) — smaller vectors but lower quality

## Decision 2: Chat Generation — Cohere Command-R

**Decision**: Use Cohere `command-r-08-2024` with native `documents` parameter for grounded RAG.

**Rationale**: Cohere's chat endpoint natively supports document-grounded generation with built-in citation objects. No need to build custom citation extraction — the API returns `citations` with `start`, `end`, `text`, and `sources` fields.

**Key Technical Details**:
- Model: `command-r-08-2024` (free tier)
- Documents parameter: list of `{"title": "...", "snippet": "..."}` objects
- Citations: returned as structured objects with character offsets and source references
- Python SDK: `co.chat(model="command-r-08-2024", messages=[...], documents=[...])`
- Streaming: `co.chat_stream(...)` for real-time responses

**Alternatives Considered**:
- Cohere `command-r-plus` — higher quality but same free tier limits
- Google Gemini Flash — free but no native document-grounded citations

## Decision 3: Reranking — Cohere Rerank v3.5

**Decision**: Use Cohere `rerank-v3.5` to improve retrieval precision after initial vector search.

**Rationale**: Available on free tier. Reranking is critical for variable-length chunks where a code-heavy 900-token section and a structural 150-token section compete for the same query. Rerank scores semantic relevance more precisely than cosine similarity alone.

**Key Technical Details**:
- Model: `rerank-v3.5`
- Input: query + list of document strings
- Output: reranked results with relevance scores
- Python SDK: `co.rerank(model="rerank-v3.5", query=..., documents=..., top_n=5)`

**Alternatives Considered**:
- No reranking (vector similarity only) — simpler but lower precision for mixed-length chunks
- Cross-encoder `ms-marco-MiniLM` — requires separate model download, not on Cohere

## Decision 4: Vector Database — Qdrant Cloud

**Decision**: Use Qdrant Cloud (free tier) with async client, deterministic UUIDs, and payload-based filtering.

**Rationale**: Free tier provides 1GB storage (sufficient for ~500-800 chunks). Native payload filtering supports hardware tier range queries. Deterministic UUIDs (uuid5) enable idempotent re-ingestion.

**Key Technical Details**:
- Client: `AsyncQdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)`
- Collection: 1024-dim cosine vectors
- Point IDs: `uuid5(NAMESPACE_DNS, f"{lesson_id}:{section_title}")` for deterministic dedup
- Delete by lesson: `FilterSelector(filter=Filter(must=[FieldCondition(key="lesson_id", match=MatchValue(value=id))]))`
- Tier filtering: `FieldCondition(key="hardware_tier", range=Range(lte=user_tier))`
- Health: `client.get_collections()` as connectivity check

**Alternatives Considered**:
- Pinecone — free tier more limited, no self-host option
- ChromaDB — local only, no cloud managed service

## Decision 5: Free Tier Rate Limits — Batching Strategy

**Decision**: Batch all operations aggressively. Cohere free tier allows only 1,000 API calls/month total across all endpoints.

**Rationale**: Each user query costs 2 API calls (1 embed + 1 chat) + optionally 1 rerank = 2-3 calls. At 1,000/month, this supports ~330-500 user queries. Ingestion must batch all chunks into minimal embed calls (96 texts/call).

**Key Constraints**:
- Embed: 5 calls/min, 96 texts/batch → 12 lessons × ~10 chunks = ~120 chunks = 2 embed calls for full Module 1 ingestion
- Chat: 20 calls/min
- Rerank: shared 1,000/month pool
- **Production recommendation**: Upgrade to production API key (1,000 calls/min, unlimited monthly) before demo

**Mitigation**:
- Batch embed calls during ingestion (max 96 texts per call)
- Skip rerank for high-confidence results (cosine score > 0.85)
- Cache frequently asked queries (in-memory LRU)

## Decision 6: Token Counting — tiktoken for Consistency

**Decision**: Use `tiktoken` with `cl100k_base` encoding for token counting during chunking.

**Rationale**: The RAG implementation guide already uses tiktoken. Consistent tokenizer ensures chunk size thresholds (200, 700, 800) are measured identically during ingestion and validation.

**Alternatives Considered**:
- Cohere tokenizer — would be more accurate for Cohere models but no public Python API for token counting
- Simple word count — too imprecise for threshold decisions

## Decision 7: Hardware Tier Filtering Semantics

**Decision**: Use `lte` (less than or equal) filtering: a Tier 2 user sees Tier 1 + Tier 2 content.

**Rationale**: Hardware tiers are cumulative — a user with Tier 2 equipment can run everything Tier 1 can do plus Tier 2 specific content. Filtering with `hardware_tier <= user_tier` matches this semantic correctly.

## Decision 8: Async Architecture

**Decision**: Use async throughout — AsyncQdrantClient + async Cohere calls + FastAPI async endpoints.

**Rationale**: FastAPI is async-native. Qdrant and Cohere both provide async clients. A user query involves 3 sequential I/O operations (embed → search → chat), so async prevents thread blocking and supports concurrent users (SC-006: 10 concurrent).
