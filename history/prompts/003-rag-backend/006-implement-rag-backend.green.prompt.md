---
id: 006
title: Implement RAG Backend
stage: green
date: 2026-02-12
surface: agent
model: claude-opus-4-6
feature: 003-rag-backend
branch: 003-rag-backend
user: laiba166-shaikh
command: /sp.implement
labels: ["implementation", "rag", "fastapi", "qdrant", "cohere"]
links:
  spec: specs/003-rag-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - rag-backend/app/__init__.py
  - rag-backend/app/config.py
  - rag-backend/app/main.py
  - rag-backend/app/models/domain.py
  - rag-backend/app/models/requests.py
  - rag-backend/app/models/responses.py
  - rag-backend/app/services/embeddings.py
  - rag-backend/app/services/vectorstore.py
  - rag-backend/app/services/chat.py
  - rag-backend/app/services/reranker.py
  - rag-backend/app/services/query.py
  - rag-backend/app/api/health.py
  - rag-backend/app/api/search.py
  - rag-backend/app/api/chat.py
  - rag-backend/app/ingestion/parser.py
  - rag-backend/app/ingestion/chunker.py
  - rag-backend/app/ingestion/pipeline.py
  - rag-backend/app/ingest.py
  - rag-backend/requirements.txt
  - rag-backend/requirements-dev.txt
  - rag-backend/Procfile
  - rag-backend/.env.example
  - .gitignore
tests:
  - "Ingestion: 12 files, 136 chunks, 0 errors (live Qdrant + Cohere)"
  - "Search: POST /api/search — top result score 0.963 for 'What is Physical AI?'"
  - "Chat: POST /api/chat — coherent answer with 2 citations from correct lessons"
  - "Health: GET /api/health — both components healthy"
---

## Prompt

/sp.implement

## Response snapshot

Executed all 31 tasks across 7 phases for the RAG backend implementation:

**Phase 1 (Setup)**: Created rag-backend directory structure, requirements.txt, .env.example, Procfile.

**Phase 2 (Foundation)**: Implemented Settings (pydantic-settings), 3 model files (domain, requests, responses), CohereEmbedService, QdrantService, FastAPI app with lifespan context manager.

**Phase 3 (Ingestion)**: Built adaptive chunking pipeline — markdown parser (YAML frontmatter + H2 splitting), AdaptiveChunker (classify → enrich/preserve/split), IngestionPipeline (orchestrate with SHA-256 manifest), CLI entry point. Ingested Module 1: 12 files → 136 chunks (48 structural, 52 instructional, 36 code_heavy) in 2 API calls.

**Phase 4 (Search)**: QueryExpander (structural synonym expansion), CohereRerankService (skip if cosine > 0.85), search endpoint with hardware tier filtering. Verified: 1.27s latency, correct lesson ranking.

**Phase 5 (Chat)**: CohereChatService using Command-R with V2 documents API format, citation extraction from Cohere citation objects, fallback to context-based sources. Fixed V2 document format (requires `data` field).

**Phase 6 (Health)**: Health endpoint with concurrent Cohere + Qdrant checks, 503 for degraded/unhealthy.

**Phase 7 (Polish)**: Global exception handler (Cohere → SERVICE_UNAVAILABLE, Qdrant → SERVICE_UNAVAILABLE, other → INTERNAL_ERROR), dev dependencies, CORS verified.

## Outcome

- ✅ Impact: Full RAG backend operational — all 3 API endpoints + CLI ingestion working against live Cohere and Qdrant services
- 🧪 Tests: 4 live integration tests passed (ingestion, search, chat, health)
- 📁 Files: 22 files created, 1 modified (.gitignore)
- 🔁 Next prompts: Deploy to Railway, connect frontend chat widget, add unit tests
- 🧠 Reflection: Cohere V2 API has different document format (requires `data` field) vs V1. Qdrant requires payload indexes before filtering. Both caught and fixed during live testing.

## Evaluation notes (flywheel)

- Failure modes observed: Qdrant 400 on filter without payload index; Cohere V2 chat requires documents[].data field
- Graders run and results (PASS/FAIL): All 4 endpoint tests PASS
- Prompt variant (if applicable): null
- Next experiment: Add pytest unit tests with mocked clients for CI pipeline
