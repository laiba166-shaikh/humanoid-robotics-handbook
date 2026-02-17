# Implementation Plan: RAG Chatbot Conversational Interface

**Branch**: `005-rag-chatbot-ui` | **Date**: 2026-02-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-rag-chatbot-ui/spec.md`

## Summary

Add a conversational chat interface to the Docusaurus textbook using OpenAI ChatKit JS (frontend widget) and ChatKit Python SDK (backend server), backed by OpenAI Agents SDK orchestrating the existing Cohere RAG services. The ChatKit server endpoint is added to the existing FastAPI app. Conversation threads and messages persist in the existing Neon PostgreSQL database. The chat widget renders only on `/docs/` routes using Docusaurus route detection.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript (frontend, Docusaurus)
**Primary Dependencies**:
- Backend: `openai-chatkit` (Python SDK), `openai-agents` (Agents SDK), FastAPI, SQLAlchemy async, asyncpg
- Frontend: `@openai/chatkit-react`, Docusaurus v3.9.2
**Storage**: Neon PostgreSQL (existing, shared with auth — new tables for threads/items)
**Testing**: pytest (backend), manual browser testing (frontend widget)
**Target Platform**: Web — Railway (backend), GitHub Pages (frontend)
**Project Type**: Web application (existing backend + existing frontend)
**Performance Goals**: First token < 2s, thread list load < 1s
**Constraints**: Free-tier Cohere for embeddings/chat, Qdrant Cloud free tier, docs-only visibility
**Scale/Scope**: Single-user concurrent, ~100 conversation threads per user

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Tech stack matches constitution | PASS | ChatKit SDK listed in constitution (`Chat UI: OpenAI ChatKit SDK`), FastAPI, Neon Postgres all match |
| Color palette compliance | PASS | ChatKit theming will use constitution palette (Pacific Blue, Jet Black, Azure Mist) |
| No alternative tech without constitution update | PASS | All technologies are already in constitution |
| Backend API standards (JSON envelope, error codes) | PASS | ChatKit uses its own protocol on `/chatkit` endpoint; existing `/api/*` endpoints unchanged |
| Naming conventions (Python snake_case, Component PascalCase) | PASS | Will follow existing conventions |
| CORS rules | PASS | Already configured for frontend domain |
| Environment variables documented | PASS | New vars: `OPENAI_API_KEY` (for Agents SDK) |

**New env var needed**: `OPENAI_API_KEY` — required for the OpenAI Agents SDK to run the agent. This is a new dependency. The agent orchestrates tool calls but the actual RAG work still uses Cohere services.

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-chatbot-ui/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
rag-backend/
├── app/
│   ├── chatkit/                    # NEW: ChatKit server module
│   │   ├── __init__.py
│   │   ├── server.py               # ChatKitServer subclass with respond()
│   │   ├── store.py                # PostgreSQL Store implementation
│   │   ├── agent.py                # OpenAI Agent with RAG tools
│   │   └── models.py               # DB models for threads/items
│   ├── main.py                     # MODIFIED: add /chatkit endpoint + lifespan init
│   └── config.py                   # MODIFIED: add openai_api_key setting
├── .env.example                    # MODIFIED: add OPENAI_API_KEY

humanoid-textbook/
├── src/
│   ├── components/
│   │   └── ChatWidget/             # NEW: ChatKit React wrapper
│   │       ├── index.tsx            # Main component with docs-only logic
│   │       └── ChatWidget.module.css # Theming overrides
│   └── theme/
│       └── DocPage/                # NEW: Docusaurus swizzle for docs-only rendering
│           └── Layout.tsx           # Wraps doc pages with ChatWidget
├── package.json                    # MODIFIED: add @openai/chatkit-react
```

**Structure Decision**: Extends existing web application layout. Backend adds a `chatkit/` module alongside existing `api/`, `services/`, `auth/` modules. Frontend adds a `ChatWidget` component rendered only in the DocPage layout wrapper.

## Architecture

### Request Flow

```
Student on /docs/ page
  ↓
ChatKit React widget (useChatKit hook)
  ↓ POST /chatkit (SSE stream)
FastAPI endpoint → ChatKitServer.process()
  ↓
ChatKitServer.respond()
  ↓ Load thread history from PostgresStore
  ↓ Convert to agent input
  ↓
OpenAI Agent (Agents SDK)
  ↓ Calls search_textbook tool
  ↓   → QueryExpander.expand()
  ↓   → CohereEmbedService.embed_query()
  ↓   → QdrantService.search()
  ↓   → CohereRerankService.rerank()
  ↓ Returns context chunks
  ↓
Agent generates response with citations
  ↓ stream_agent_response() → SSE events
  ↓
ChatKit widget renders streaming tokens + citations
```

### Key Design Decisions

1. **ChatKit Server on same FastAPI app** — No separate service. The `/chatkit` endpoint coexists with `/api/chat`, `/api/search`, etc. Shares the same Cohere/Qdrant clients via `app.state`.

2. **OpenAI Agents SDK wraps existing services** — The agent uses a `@function_tool` that internally calls the existing `QueryExpander`, `CohereEmbedService`, `QdrantService`, and `CohereRerankService`. No Cohere services are replaced.

3. **PostgreSQL Store reuses existing engine** — The ChatKit Store implementation uses the same SQLAlchemy engine and async session factory from `app.database`. New tables: `chatkit_threads` and `chatkit_thread_items`.

4. **Docs-only via DocPage layout swizzle** — Docusaurus's `@theme/DocPage/Layout` is swizzled to include the ChatWidget. Non-doc pages (landing, blog, auth) never render it.

5. **Auth integration via request headers** — The frontend passes the JWT access token as a header in ChatKit API config. The backend extracts `user_id` from it for thread ownership. Unauthenticated users get anonymous threads (no persistence).

## Complexity Tracking

No constitution violations. No complexity justification needed.
