# Tasks: RAG Backend for Textbook Chatbot

**Input**: Design documents from `specs/003-rag-backend/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/openapi.yaml, quickstart.md

**Tests**: Not explicitly requested — test tasks omitted. Tests can be added via `/sp.tasks` with TDD flag.

**Organization**: Tasks grouped by user story. Ingestion (US3) is implemented before Search (US2) and Chat (US1) because search and chat require ingested data in Qdrant.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4)
- Exact file paths from plan.md project structure

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create rag-backend directory structure, dependencies, and configuration files

- [x] T001 Create rag-backend directory structure with all `__init__.py` files per plan.md: `rag-backend/app/`, `app/models/`, `app/api/`, `app/services/`, `app/ingestion/`, `tests/`
- [x] T002 [P] Create `rag-backend/requirements.txt` with pinned dependencies: fastapi>=0.115.0, uvicorn[standard]>=0.32.0, cohere>=5.0.0, qdrant-client>=1.12.0, python-dotenv>=1.0.0, pyyaml>=6.0.0, tiktoken>=0.7.0, pydantic>=2.0.0, pydantic-settings>=2.0.0
- [x] T003 [P] Create `rag-backend/.env.example` with all required environment variables: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, ALLOWED_ORIGINS, COLLECTION_NAME, LOG_LEVEL
- [x] T004 [P] Create `rag-backend/Procfile` for Railway deployment: `web: uvicorn app.main:app --host 0.0.0.0 --port $PORT`

---

## Phase 2: Foundation (Blocking Prerequisites)

**Purpose**: Core Pydantic models, service clients, and FastAPI app skeleton that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Implement Settings class using pydantic-settings in `rag-backend/app/config.py` — load COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, ALLOWED_ORIGINS (comma-split to list), COLLECTION_NAME (default: handbook_lessons), LOG_LEVEL (default: INFO); use `@lru_cache` for singleton
- [ ] T006 [P] Implement domain models in `rag-backend/app/models/domain.py` — SectionType enum (structural, instructional, code_heavy), LessonFrontmatter (15 required fields from data-model.md), Chunk (content, metadata fields, token_count, section_type, chunk_index, total_chunks, enrichment_strategy), IngestionReport (total_files, processed_files, skipped_files, total_chunks, chunks_by_type, enriched_count, split_count, errors, duration_seconds, embed_api_calls)
- [ ] T007 [P] Implement request models in `rag-backend/app/models/requests.py` — SearchRequest (query: str min 1 max 500, hardware_tier: int 1-4 default 4, top_k: int 1-20 default 5), ChatRequest (query: str min 1 max 500, hardware_tier: int 1-4 default 4, top_k: int 1-10 default 5) per contracts/openapi.yaml
- [ ] T008 [P] Implement response models in `rag-backend/app/models/responses.py` — ResponseMeta (latency_ms, source_count), ApiResponse[T] generic (success, data, error, meta), ApiErrorResponse (success=False, error, error_code enum), SearchResultItem (all fields from data-model.md SearchResult), SearchResponse (query, query_expanded, results list), ChatSource (lesson_id, lesson_title, section_title, module, chapter, citation_label), ChatResponse (answer, sources list), ComponentHealth (status, latency_ms, error), HealthResponse (status enum healthy/degraded/unhealthy, components dict)
- [ ] T009 Implement CohereEmbedService in `rag-backend/app/services/embeddings.py` — async class wrapping cohere.AsyncClientV2; methods: `embed_query(text) -> list[float]` using input_type="search_query", `embed_documents(texts) -> list[list[float]]` using input_type="search_document" with batch chunking at 96 texts per call; model="embed-english-v3.0"
- [ ] T010 Implement QdrantService in `rag-backend/app/services/vectorstore.py` — async class wrapping AsyncQdrantClient; methods: `ensure_collection()` create if not exists (1024 dim, cosine), `upsert_chunks(chunks_with_vectors)` using deterministic uuid5 IDs from lesson_id+section_title+chunk_index, `delete_by_lesson(lesson_id)` using FilterSelector, `search(vector, hardware_tier, top_k)` using Range(lte=tier) filter returning payload; `health_check() -> ComponentHealth`
- [ ] T011 Implement FastAPI app skeleton in `rag-backend/app/main.py` — create app with lifespan context manager that initializes cohere.AsyncClientV2, AsyncQdrantClient, and all services on app.state (CohereEmbedService, QdrantService, CohereChatService, CohereRerankService, QueryExpander); configure CORS from settings.allowed_origins; mount API routers from app.api; shutdown closes qdrant client

**Checkpoint**: Foundation ready — Pydantic models validated, Cohere + Qdrant clients initialized via lifespan, FastAPI app starts with `uvicorn app.main:app --reload`

---

## Phase 3: User Story 3 — Ingest Textbook Content (Priority: P2) 🎯 First Implementation

**Goal**: Build the adaptive chunking ingestion pipeline that parses lesson markdown files, classifies H2 sections by type, applies type-appropriate chunking (enrich/preserve/split), embeds via Cohere, and upserts to Qdrant with deterministic IDs

**Independent Test**: Run `python -m app.ingest --docs-path ../humanoid-textbook/docs/module-1-ros2` and verify ~130-150 chunks appear in Qdrant Cloud dashboard with complete metadata

**Why first**: Search and Chat require data in Qdrant. Ingestion produces that data.

### Implementation for User Story 3

- [ ] T012 [P] [US3] Implement markdown parser in `rag-backend/app/ingestion/parser.py` — `parse_lesson_file(path) -> tuple[LessonFrontmatter, list[dict]]`: extract YAML frontmatter via yaml.safe_load (validate all 15 required fields, raise warning and skip if missing), split body at `## ` H2 boundaries into sections with title and content; strip frontmatter block before splitting; handle files with no H2 headers (entire body = single section)
- [ ] T013 [P] [US3] Implement adaptive chunker in `rag-backend/app/ingestion/chunker.py` — `AdaptiveChunker` class with: `classify_section(title, content) -> SectionType` (title keyword match for structural, 2+ code blocks for code_heavy, default instructional); `enrich_short_section(section, frontmatter) -> Chunk` for <200 tokens (prepend lesson title, module, chapter context); `preserve_section(section, frontmatter) -> Chunk` for 200-700 tokens; `split_long_section(section, frontmatter) -> list[Chunk]` for >700 tokens (split at paragraph/code-block boundaries, 100-token overlap, keep code blocks intact, carry section title forward, set chunk_index and total_chunks); `create_chunks(sections, frontmatter) -> list[Chunk]` orchestrator; token counting via tiktoken cl100k_base
- [ ] T014 [US3] Implement ingestion pipeline orchestrator in `rag-backend/app/ingestion/pipeline.py` — `IngestionPipeline` class taking a Cohere client and QdrantClient (sync or async); methods: `embed_batch(texts) -> list[list[float]]` wraps client.embed internally;`upsert_batch(chunks_with_vectors)` wraps client.upsert internally; `run(docs_path, selective=False) -> IngestionReport`: discover all .md lesson files recursively (skip README.md), for each file: parse → chunk → delete old chunks by lesson_id → batch embed (96/call) with input_type="search_document" → upsert with deterministic UUIDs; collect stats into IngestionReport (files processed, chunks by type, enriched count, split count, embed API calls used, errors)
- [ ] T015 [US3] Implement CLI entry point in `rag-backend/app/ingest.py` — `python -m app.ingest --docs-path <path>` using argparse; loads .env via dotenv, creates sync `cohere.ClientV2` and sync `QdrantClient`, passes them to IngestionPipeline (pipeline accepts both sync and async clients via duck typing on .embed/.upsert), runs ingestion, prints IngestionReport summary to stdout; exit code 0 on success, 1 on errors
- [ ] T016 [US3] Run ingestion pipeline on Module 1 content at `humanoid-textbook/docs/module-1-ros2/` — verify chunks appear in Qdrant Cloud dashboard, validate metadata completeness (lesson_id, section_type, hardware_tier present on all points), confirm no duplicate entries, log the IngestionReport

**Checkpoint**: Module 1 content (~130-150 chunks) is in Qdrant with complete metadata. Ingestion can be re-run idempotently.

---

## Phase 4: User Story 2 — Semantic Search (Priority: P2)

**Goal**: Implement the search endpoint that accepts a query, expands it for structural content, embeds via Cohere, searches Qdrant with hardware tier filtering, optionally reranks, and returns ranked results with metadata

**Independent Test**: `POST /api/search {"query": "publisher subscriber pattern", "top_k": 3}` returns results from Chapter 2 ranked by relevance with section titles and lesson references

### Implementation for User Story 2

- [ ] T017 [P] [US2] Implement QueryExpander in `rag-backend/app/services/query.py` — `expand(query) -> tuple[str, bool]` returns (expanded_query, was_expanded); detect structural keywords (learning objectives, key takeaways, prerequisites, next steps, check your understanding, summary) and append semantic synonyms per FR-018; return original query unchanged if no structural keywords detected
- [ ] T018 [P] [US2] Implement CohereRerankService in `rag-backend/app/services/reranker.py` — async class wrapping cohere.AsyncClientV2; `rerank(query, documents, top_n) -> list[dict]` using model="rerank-v3.5"; skip rerank if top cosine score > 0.85 or if rerank is disabled in config; return results with relevance_score added
- [ ] T019 [US2] Implement search endpoint in `rag-backend/app/api/search.py` — `POST /api/search` router; validate SearchRequest; call QueryExpander.expand → CohereEmbedService.embed_query → QdrantService.search (with hardware_tier Range lte filter) → CohereRerankService.rerank (conditional); map Qdrant results to SearchResultItem list; wrap in ApiResponse[SearchResponse] with ResponseMeta (latency_ms, source_count); handle empty results with FR-016 message; handle validation errors with ApiErrorResponse
- [ ] T020 [US2] Wire search router into FastAPI app in `rag-backend/app/main.py` — import and include search router; verify endpoint responds at `POST /api/search`

**Checkpoint**: Search endpoint returns ranked results with hardware tier filtering. Test with: `curl -X POST http://localhost:8000/api/search -H "Content-Type: application/json" -d '{"query": "What is Physical AI?", "top_k": 3}'`

---

## Phase 5: User Story 1 — RAG Chat with Citations (Priority: P1) 🎯 Core Feature

**Goal**: Implement the chat endpoint that retrieves relevant chunks via search, passes them as documents to Cohere Command-R, and returns a generated answer with structured citations in `[Module X, Chapter Y, Lesson Z]` format

**Independent Test**: `POST /api/chat {"query": "How do ROS 2 nodes communicate?"}` returns a coherent answer with citations referencing Chapter 2 lessons

### Implementation for User Story 1

- [ ] T021 [US1] Implement CohereChatService in `rag-backend/app/services/chat.py` — async class wrapping cohere.AsyncClientV2; `generate(query, context_chunks) -> tuple[str, list[ChatSource]]`: format chunks as Cohere documents list `[{"id": lesson_id, "title": section_title, "snippet": content}]`; call `co.chat(model="command-r-08-2024", messages=[system_prompt, user_query], documents=docs)`; system prompt from constitution RAG persona ("You are a helpful teaching assistant..."); map Cohere citation objects (start, end, sources) back to chunk metadata → ChatSource with citation_label `[Module X, Chapter Y, Lesson Z]`; handle no-results case with "I don't have information on that in the textbook" response
- [ ] T022 [US1] Implement chat endpoint in `rag-backend/app/api/chat.py` — `POST /api/chat` router; validate ChatRequest; call QueryExpander.expand → CohereEmbedService.embed_query → QdrantService.search → CohereRerankService.rerank → CohereChatService.generate; wrap in ApiResponse[ChatResponse] with ResponseMeta; handle dependency unavailability with 503 ApiErrorResponse; handle empty retrieval with helpful fallback message per FR-016
- [ ] T023 [US1] Wire chat router into FastAPI app in `rag-backend/app/main.py` — import and include chat router; verify endpoint responds at `POST /api/chat`

**Checkpoint**: Chat endpoint returns generated answers with citations. Test with: `curl -X POST http://localhost:8000/api/chat -H "Content-Type: application/json" -d '{"query": "What is Physical AI?"}'` — verify answer references Module 1, Chapter 1, Lesson 1

---

## Phase 6: User Story 4 — Health Monitoring (Priority: P3)

**Goal**: Implement health endpoint that checks Qdrant connectivity and Cohere API availability, returning component-level health status

**Independent Test**: `GET /api/health` returns `{"status": "healthy", "components": {"qdrant": {"status": "healthy"}, "cohere": {"status": "healthy"}}}`

### Implementation for User Story 4

- [ ] T024 [P] [US4] Implement Cohere health check method in `rag-backend/app/services/embeddings.py` — add `health_check() -> ComponentHealth` to CohereEmbedService; embed a single test text "health check" and measure latency; return ComponentHealth with status and latency_ms; catch exceptions → unhealthy with error message
- [ ] T025 [US4] Implement health endpoint in `rag-backend/app/api/health.py` — `GET /api/health` router wrap in ApiResponse[HealthResponse]; call vectorstore.health_check() and embed_service.health_check() concurrently (asyncio.gather); aggregate into HealthResponse: status="healthy" if all pass, "degraded" if partial, "unhealthy" if none; return 200 for healthy, 503 for degraded/unhealthy
- [ ] T026 [US4] Wire health router into FastAPI app in `rag-backend/app/main.py` — import and include health router; verify endpoint responds at `GET /api/health`

**Checkpoint**: Health endpoint reports accurate status for all dependencies

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, deployment readiness, documentation updates

- [ ] T027 [P] Add global exception handler middleware in `rag-backend/app/main.py` — catch unhandled exceptions and return ApiErrorResponse with INTERNAL_ERROR code; catch Cohere API errors and return SERVICE_UNAVAILABLE; catch Qdrant connection errors similarly; log all errors with context
- [ ] T028 [P] Add `rag-backend/requirements-dev.txt` with test dependencies: pytest, httpx, pytest-asyncio
- [ ] T029 Verify CORS configuration end-to-end — start server, send preflight OPTIONS request from allowed origin (localhost:3000), verify Access-Control-Allow-Origin header; test with disallowed origin and confirm rejection
- [ ] T030 Update constitution Tech Stack in `.specify/memory/constitution.md` — replace OpenAI entries with Cohere (embed-english-v3.0 for embeddings, Command-R for chat generation); update environment variables section to use COHERE_API_KEY instead of OPENAI_API_KEY
- [ ] T031 Run quickstart.md validation — follow all steps in `specs/003-rag-backend/quickstart.md` on a clean setup; verify Cohere connection, Qdrant connection, server starts, ingestion completes, all 3 endpoints respond correctly

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup ──────────────────► Phase 2: Foundation ──► Phase 3: US3 (Ingest) ──► Phase 4: US2 (Search) ──► Phase 5: US1 (Chat) ──► Phase 7: Polish
                                                                                                               ├──► Phase 6: US4 (Health)──┘
```

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundation)**: Depends on Phase 1 — BLOCKS all user stories
- **Phase 3 (US3 Ingest)**: Depends on Phase 2 — produces data needed by US1 and US2
- **Phase 4 (US2 Search)**: Depends on Phase 3 — requires ingested data for testing
- **Phase 5 (US1 Chat)**: Depends on Phase 4 — builds on search retrieval
- **Phase 6 (US4 Health)**: Depends on Phase 2 only — can run in parallel with US3/US2/US1
- **Phase 7 (Polish)**: Depends on all user stories complete

### User Story Dependencies

- **US3 (Ingestion)**: Foundation only — independently testable via CLI
- **US2 (Search)**: Foundation + ingested data — independently testable via curl
- **US1 (Chat)**: Foundation + search working — independently testable via curl
- **US4 (Health)**: Foundation only — independently testable via curl

### Parallel Opportunities Within Phases

- **Phase 1**: T002, T003, T004 all parallel (different files)
- **Phase 2**: T006, T007, T008 parallel (different model files); T009, T010 parallel after models
- **Phase 3**: T012, T013 parallel (parser + chunker are independent)
- **Phase 4**: T017, T018 parallel (query expander + reranker are independent)
- **Phase 6**: T024 parallel with any Phase 3-5 task (health check is independent)
- **Phase 7**: T027, T028 parallel (different files)

---

## Parallel Example: Phase 2 Foundation

```
# Launch model files in parallel (different files, no dependencies):
T006: Domain models in app/models/domain.py
T007: Request models in app/models/requests.py
T008: Response models in app/models/responses.py

# Then launch services in parallel (depend on models):
T009: CohereEmbedService in app/services/embeddings.py
T010: QdrantService in app/services/vectorstore.py

# Then app skeleton (depends on services):
T011: FastAPI main.py with lifespan
```

## Parallel Example: Phase 3 Ingestion

```
# Launch parser and chunker in parallel (independent modules):
T012: Markdown parser in app/ingestion/parser.py
T013: Adaptive chunker in app/ingestion/chunker.py

# Then pipeline (depends on both):
T014: Pipeline orchestrator in app/ingestion/pipeline.py
T015: CLI entry point in app/ingest.py
T016: Run ingestion on Module 1
```

---

## Implementation Strategy

### MVP First (Ingestion + Search)

1. Complete Phase 1: Setup (4 tasks)
2. Complete Phase 2: Foundation (7 tasks)
3. Complete Phase 3: US3 Ingestion (5 tasks)
4. Complete Phase 4: US2 Search (4 tasks)
5. **STOP and VALIDATE**: Search returns relevant results for test queries
6. Demo search-only if needed

### Full Feature (Add Chat + Health)

7. Complete Phase 5: US1 Chat (3 tasks)
8. Complete Phase 6: US4 Health (3 tasks)
9. **STOP and VALIDATE**: All endpoints functional
10. Complete Phase 7: Polish (5 tasks)
11. Deploy to Railway

### Suggested MVP Scope

**Minimum viable demo**: Phase 1 + 2 + 3 + 4 = **20 tasks** → Working search endpoint with ingested Module 1 content

**Full feature**: All phases = **31 tasks** → Search + Chat with citations + Health monitoring + Polish

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- US3 (Ingest) implemented first despite being P2 because it produces data required by US1 and US2
- Ingestion uses sync clients for simplicity (CLI script); API uses async clients
- Cohere free tier budget: ingestion costs ~2 embed calls; each user query costs 2-3 calls (embed + chat + optional rerank)
- Commit after each task or logical group
