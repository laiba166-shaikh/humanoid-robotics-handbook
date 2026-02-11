# Feature Specification: RAG Backend for Textbook Chatbot

**Feature Branch**: `003-rag-backend`
**Created**: 2026-02-11
**Status**: Draft
**Input**: User description: "Integrate RAG backend for the textbook with the handling of variable length chunk sizes to educate and answer user queries about the book. Make it architecture ready for the autonomous ingestion pipeline for future updates in the book content of existing and new chapters. Create with FastAPI with proper type handling with pydantic models, Qdrant, Cohere"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask a Question About Textbook Content (Priority: P1)

A student reading the Physical AI & Humanoid Robotics textbook has a question about a concept covered in the lessons. They send a natural language question to the chatbot and receive an accurate, cited answer drawn from the textbook content. The answer includes references to the specific module, chapter, and lesson where the information was found, so the student can navigate directly to the source material.

**Why this priority**: This is the core value proposition — answering student questions from textbook content is the fundamental purpose of the RAG backend. Without this, no other features matter.

**Independent Test**: Can be fully tested by sending a question like "What is Physical AI?" and verifying that the response contains accurate content from Lesson 1.1 with a citation in the format `[Module 1, Chapter 1, Lesson 1]`.

**Acceptance Scenarios**:

1. **Given** the textbook content has been ingested into the vector store, **When** a student asks "What is Physical AI?", **Then** the system returns an answer sourced from Module 1, Chapter 1, Lesson 1 with a properly formatted citation.
2. **Given** the textbook content has been ingested, **When** a student asks a question not covered in the textbook (e.g., "What is quantum computing?"), **Then** the system responds with "I don't have information on that in the textbook" and suggests relevant lessons the student might find useful.
3. **Given** the textbook content has been ingested, **When** a student asks "How do ROS 2 nodes communicate?", **Then** the system retrieves content from multiple relevant sections (topics, publishers, subscribers) and synthesizes a coherent answer with multiple citations.

---

### User Story 2 - Search Textbook Content by Topic (Priority: P2)

A student wants to find all relevant sections about a specific topic (e.g., "URDF") across the entire textbook. They submit a semantic search query and receive a ranked list of matching content chunks with their source locations, relevance scores, and preview snippets. This helps students discover related content they may not have encountered yet.

**Why this priority**: Semantic search provides direct value for navigating the textbook and is the retrieval foundation that powers the chat feature. It can be delivered independently and is simpler than the full chat pipeline.

**Independent Test**: Can be tested by searching for "sensor systems" and verifying that results include content from Lesson 1.4 (Sensor Systems) ranked highest, with additional results from related lessons.

**Acceptance Scenarios**:

1. **Given** textbook content is ingested, **When** a student searches for "publisher subscriber pattern", **Then** the system returns ranked results with the most relevant chunks from Chapter 2 (ROS 2 Architecture) at the top.
2. **Given** textbook content is ingested, **When** a student searches for "URDF" and filters by hardware tier 1, **Then** the system returns only chunks from content accessible at hardware tier 1.
3. **Given** textbook content is ingested, **When** a student searches with an empty query, **Then** the system returns a validation error with a clear message.

---

### User Story 3 - Ingest New or Updated Textbook Content (Priority: P2)

A content author adds a new chapter or updates an existing lesson in the textbook. The ingestion system processes the new or updated markdown files, extracts structured metadata from YAML frontmatter, and applies an adaptive chunking strategy based on the actual content distribution observed in the textbook. Module 1 analysis shows 128 H2 sections with significant variance: 38% are short structural sections (79-241 tokens), 35% are optimal instructional sections (300-500 tokens), 18% are long code-heavy sections (600-950 tokens), and 9% are mixed. The pipeline must handle each type appropriately rather than forcing uniform chunk sizes. Existing chunks from updated lessons are replaced rather than duplicated. This pipeline is designed to run autonomously when new content is added.

**Why this priority**: Without ingestion, the chatbot has no knowledge. This is architecturally critical and must be designed for autonomous operation to support ongoing textbook updates without manual intervention.

**Independent Test**: Can be tested by running the ingestion pipeline on a single lesson file and verifying that the correct number of chunks appear in the vector store with complete metadata, and that short/optimal/long sections each received appropriate treatment.

**Acceptance Scenarios**:

1. **Given** a new lesson markdown file exists in the docs directory, **When** the ingestion pipeline runs, **Then** the file is parsed, chunked at H2 boundaries, embedded, and stored with complete metadata (module, chapter, lesson, hardware tier, layer, keywords).
2. **Given** an existing lesson has been previously ingested, **When** the lesson content is updated and the pipeline runs again, **Then** the old chunks for that lesson are replaced with new chunks (no duplicates).
3. **Given** a lesson contains a short structural section like "Learning Objectives" (under 200 tokens), **When** the pipeline processes it, **Then** the chunk is enriched with lesson context (title, module, chapter) to improve its retrievability without altering the original content.
4. **Given** a lesson contains an optimal-length instructional section (200-700 tokens), **When** the pipeline processes it, **Then** the section is stored as a single chunk preserving its natural H2 boundary, with no splitting or padding.
5. **Given** a lesson contains a long code-heavy section (over 700 tokens), **When** the pipeline processes it, **Then** the section is split into overlapping sub-chunks at semantic boundaries (paragraph/code-block edges), keeping code blocks intact and preserving the section title context in each sub-chunk.
6. **Given** a markdown file with invalid or missing frontmatter, **When** the pipeline processes it, **Then** the system logs a warning with the file path and specific missing fields, and skips the file without crashing.
7. **Given** a batch of new lesson files for a newly added chapter, **When** the pipeline runs targeting those files, **Then** only the new files are processed without re-processing unchanged existing content.

---

### User Story 4 - Monitor Backend Health (Priority: P3)

An operator or monitoring system checks whether the RAG backend is running and all its dependencies (vector store, embedding service) are accessible. This supports deployment validation and uptime monitoring.

**Why this priority**: Health checks are essential for production readiness but are low complexity and can be added quickly after core features are working.

**Independent Test**: Can be tested by calling the health endpoint and verifying it returns status information for each dependency.

**Acceptance Scenarios**:

1. **Given** the backend is running and all dependencies are healthy, **When** the health endpoint is called, **Then** it returns a success status with component health details.
2. **Given** the vector store is unreachable, **When** the health endpoint is called, **Then** it returns a degraded status indicating which component is unhealthy.

---

### Edge Cases

- What happens when the vector store returns zero results for a query? The system responds with "I couldn't find relevant content for your question" and suggests browsing the table of contents.
- What happens when the embedding service is temporarily unavailable? The system returns a service-unavailable error with a retry-after suggestion rather than an empty or incorrect response.
- What happens when a lesson file has no H2 headers? The entire content body (excluding frontmatter) is treated as a single chunk.
- What happens when a chunk exceeds 800 tokens? The system logs a warning but still processes and stores the oversized chunk, flagging it for manual review.
- What happens when multiple users query simultaneously? The system handles concurrent requests without blocking, up to the capacity of the hosting environment.
- What happens when a search query is extremely long (over 500 characters)? The system truncates the query to the embedding model's maximum input length and proceeds with the search.
- What happens when a code block spans an entire H2 section with no surrounding prose? The section is classified as code-heavy and kept intact (not split mid-code-block), with the section title prepended for retrieval context.
- What happens when a student asks about structural content like "What are the learning objectives for ROS 2 nodes?" The query is expanded with synonyms (e.g., "What will I learn? What skills will I gain?") to improve retrieval of short enriched structural chunks.
- What happens when the same content appears in overlapping sub-chunks from a split long section? Each sub-chunk carries a chunk index and total chunk count in metadata so downstream consumers can deduplicate or merge context as needed.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept natural language questions and return answers generated from textbook content with source citations in the format `[Module X, Chapter Y, Lesson Z]`.
- **FR-002**: System MUST perform semantic search across all ingested textbook content and return ranked results with relevance scores.
- **FR-003**: System MUST support filtering search and chat results by hardware tier (1-4) so students only see content relevant to their equipment level.
- **FR-004**: System MUST parse markdown lesson files, extract all 15 YAML frontmatter fields, and split content into chunks at H2 section boundaries.
- **FR-005**: System MUST classify each H2 section by type — structural (Learning Objectives, Key Takeaways, Check Your Understanding, Next Steps), instructional (concept explanations, theory), or code-heavy (sections with 2+ code blocks) — and apply type-appropriate chunking.
- **FR-006**: System MUST enrich short sections (under 200 tokens) with lesson context (title, module, chapter) to improve retrievability, while preserving the original content verbatim within the enriched chunk.
- **FR-007**: System MUST preserve optimal-length sections (200-700 tokens) as single chunks without splitting or padding, maintaining natural H2 boundaries.
- **FR-008**: System MUST split long sections (over 700 tokens) into overlapping sub-chunks at semantic boundaries (paragraph and code-block edges), keeping code blocks intact and carrying the section title forward into each sub-chunk.
- **FR-009**: System MUST replace existing chunks when a lesson is re-ingested (keyed by lesson ID), preventing duplicate content in the vector store.
- **FR-010**: System MUST generate vector embeddings for each chunk and store them alongside structured metadata (module, chapter, lesson, hardware tier, layer, keywords, section title, section type, chunk index for split sections).
- **FR-011**: System MUST validate all API request and response payloads using strictly typed data models, returning clear validation error messages for malformed requests.
- **FR-012**: System MUST expose a health check endpoint that reports the status of each backend dependency (vector store connectivity, embedding service availability).
- **FR-013**: System MUST return consistent JSON response envelopes for all endpoints, including success status, data payload, error details, and response metadata (latency, source count).
- **FR-014**: System MUST support CORS for the textbook's frontend domain in production and localhost in development.
- **FR-015**: System MUST log ingestion results including file count, chunk count per section type (structural, instructional, code-heavy), skipped files, and any errors encountered.
- **FR-016**: System MUST gracefully handle cases where no relevant content is found, responding with a helpful message rather than an empty result.
- **FR-017**: System MUST include the section title, lesson title, section type, and a content preview snippet in each search result so students can evaluate relevance before navigating.
- **FR-018**: System MUST expand queries targeting structural content (learning objectives, key takeaways, prerequisites, next steps) with semantic synonyms to improve retrieval of short enriched chunks.
- **FR-019**: System MUST be architecturally structured so that the ingestion pipeline can be triggered independently via CLI script to support autonomous updates when new content is added. No public API endpoint for ingestion. **Note**: GitHub webhook integration is a future enhancement — the CLI architecture supports it without restructuring (webhook handler calls the same pipeline module).
- **FR-020**: System MUST support selective ingestion — processing only new or modified files — to avoid re-embedding unchanged content during incremental updates.

### Key Entities

- **Chunk**: A unit of textbook content derived from an H2 section within a lesson. Contains the text content, section title, section type (structural/instructional/code-heavy), and a reference back to its source lesson. Variable length depending on type and processing strategy. For split sections, also carries a chunk index and total chunk count. The atomic unit for search and retrieval.
- **Section Type**: A classification applied to each H2 section that determines its chunking strategy. Three types: **structural** (Learning Objectives, Key Takeaways, etc. — typically 79-241 tokens, enriched with context), **instructional** (concept explanations — typically 300-600 tokens, preserved as-is), and **code-heavy** (sections with multiple code blocks — typically 400-950 tokens, split with overlap if over 700 tokens).
- **Lesson Metadata**: Structured information extracted from YAML frontmatter including lesson ID, title, module, chapter, hardware tier, teaching layer, proficiency level, keywords, and prerequisites. Used for filtering, citation generation, and context enrichment of short chunks.
- **Search Result**: A ranked match from the vector store containing the chunk content, relevance score, source metadata (including section type), and a preview snippet. Returned as part of search and used internally for RAG context assembly.
- **Chat Message**: A user question paired with a generated answer. The answer includes the response text and a list of source citations referencing specific lessons.
- **Ingestion Report**: A summary of a pipeline run including counts of files processed, chunks created by section type, enrichment/split statistics, files skipped (with reasons), and any errors. Used for monitoring and debugging.

### Content Distribution Reference

Based on analysis of Module 1 (128 H2 sections across 12 lessons):

| Section Type | Token Range | Count | Percentage | Chunking Strategy |
|--------------|-------------|-------|------------|-------------------|
| Structural | 79-241 | 48 | 38% | Enrich with lesson context |
| Instructional (Optimal) | 300-500 | 45 | 35% | Preserve as-is |
| Code-heavy | 600-950 | 23 | 18% | Split with overlap if >700 |
| Mixed/Other | 241-600 | 12 | 9% | Preserve as-is |

This distribution informs the adaptive chunking thresholds and is expected to remain consistent as new modules follow the same constitution standards. See `docs/rag-implementation-guide.md` for the full reference implementation.

### Non-Goals (Deferred to Next Phase)

- **User authentication and authorization** — Better-Auth (JS frontend) + FastAPI token validation will be implemented in a subsequent feature. For this phase, chat and search endpoints are open.
- **User-specific personalization** — Hardware tier preferences tied to user accounts (requires auth).
- **Conversation history persistence** — Storing chat sessions per user in Neon Postgres (requires auth + user identity).
- **Rate limiting per user** — Throttling based on authenticated user identity (requires auth). Basic global rate limiting may be added if needed.

### Assumptions

- The textbook content follows the established lesson structure with YAML frontmatter (15 fields) and H2-delimited sections as defined in the project constitution.
- Cohere's free tier provides sufficient embedding throughput for the current textbook size (~100-200 chunks for Module 1, scaling to ~500-800 for all 4 modules).
- The ingestion pipeline will be triggered via CLI script or GitHub webhook, with autonomous scheduling as a future enhancement.
- The chat feature uses Cohere Command-R (free tier) to synthesize answers from retrieved chunks rather than returning raw chunk text.
- The frontend chat widget already exists as a component in the Docusaurus project and will be connected to these backend endpoints.
- Chat and search endpoints are initially open (no auth). The architecture MUST support adding token-based auth middleware in a future phase without restructuring the endpoint logic.

### Technical Constraints

The user has specified the following technology requirements:
- **API Framework**: FastAPI (Python) with Pydantic models for all request/response types
- **Vector Database**: Qdrant for vector storage and similarity search
- **Embeddings**: Cohere `embed-english-v3.0` (1024 dimensions, free tier) for vectorizing content and queries
- **Chat Generation**: Cohere Command-R (free tier) for synthesizing answers from retrieved chunks
- **Deployment Target**: Railway (as defined in the project constitution)

**Note**: The project constitution currently lists OpenAI for embeddings. This specification uses Cohere per the user's explicit request. The constitution should be updated to reflect this change.

## Clarifications

### Session 2026-02-11

- Q: Which generative model synthesizes chat answers from retrieved chunks? → A: Cohere Command-R (free tier) — single provider for both embeddings and generation.
- Q: Which Cohere embedding model variant and vector dimension? → A: `embed-english-v3.0` (1024 dimensions) — highest quality English embeddings on free tier.
- Q: How is the ingestion pipeline triggered and protected? → A: CLI script or GitHub webhook only — no public API endpoint for ingestion.
- Q: What auth approach for chatbot access? → A: Better-Auth (JS) on frontend + FastAPI validates tokens. **Out of scope for this feature** — auth is a separate phase after RAG system is complete. Endpoints initially open.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students receive accurate, cited answers to textbook questions within 3 seconds of submitting a query (measured end-to-end from request to complete response).
- **SC-002**: Semantic search returns the correct lesson as the top result for at least 80% of test queries derived from lesson content (tested against a set of 10+ representative queries).
- **SC-003**: The ingestion pipeline successfully processes all 12 Module 1 lesson files and produces chunks with complete metadata, with zero data loss or duplicate entries.
- **SC-004**: Hardware tier filtering correctly excludes content above the specified tier in 100% of filtered queries.
- **SC-005**: All API endpoints return properly structured JSON responses with appropriate HTTP status codes, including validation errors for malformed requests.
- **SC-006**: The system handles 10 concurrent users querying simultaneously without errors, maintaining p95 response latency under 5 seconds for search and under 8 seconds for chat.
- **SC-007**: Re-ingesting an updated lesson replaces all previous chunks for that lesson with zero duplicates remaining in the vector store.
- **SC-008**: The health endpoint accurately reports the status of all backend dependencies, correctly identifying unreachable services within 5 seconds.
