# Research: RAG Chatbot UI

## R1: ChatKit Python SDK + FastAPI Integration

**Decision**: Use `ChatKitServer` subclass with single `/chatkit` POST endpoint on existing FastAPI app.

**Rationale**: ChatKit Python SDK is designed for FastAPI. The `server.process()` method accepts raw request bytes and returns either a `StreamingResult` (SSE) or JSON `Result`. This fits naturally as a single endpoint alongside existing routes.

**Alternatives considered**:
- Separate microservice for ChatKit — rejected (unnecessary complexity, would need separate deployment)
- WebSocket-based custom chat — rejected (ChatKit handles streaming via SSE natively)

**Key findings**:
- Install: `pip install openai-chatkit`
- `ChatKitServer[TContext]` is generic over a request context type
- `respond()` is an async generator yielding `ThreadStreamEvent` objects
- `StreamingResult` wraps the generator for SSE transport
- The store interface has 13 methods (thread CRUD, item CRUD, attachment metadata)

## R2: OpenAI Agents SDK with Existing Cohere Services

**Decision**: Create a single `Agent` with one `@function_tool` called `search_textbook` that wraps the existing RAG pipeline (expand → embed → search → rerank).

**Rationale**: The Agents SDK provides the conversation management, tool orchestration, and streaming that ChatKit expects via `stream_agent_response()`. The existing Cohere services remain the actual RAG engine.

**Alternatives considered**:
- Direct Cohere chat (no agent) — rejected (no built-in tool calling, would need manual streaming event conversion)
- LangChain agent — rejected (not compatible with ChatKit's `stream_agent_response()` helper)
- Replace Cohere with OpenAI for generation — rejected (would require OpenAI API credits, Cohere is free tier)

**Key findings**:
- Install: `pip install openai-agents`
- `Agent(name, instructions, tools, model)` — model is for the agent's reasoning (uses OpenAI API)
- `Runner.run_streamed()` returns a result that `stream_agent_response()` converts to ChatKit events
- `AgentContext` carries thread/store references for tools
- `@function_tool` with `RunContextWrapper[AgentContext]` gives tools access to the store
- **Important**: The agent model (e.g., `gpt-4o-mini`) uses OpenAI API, requiring `OPENAI_API_KEY`. But the actual RAG retrieval and answer generation still use Cohere. The agent decides WHEN to call the tool and formats the final response.

## R3: PostgreSQL Store Implementation

**Decision**: Implement ChatKit `Store` using the existing SQLAlchemy async engine from `app.database`, with two new tables (`chatkit_threads`, `chatkit_thread_items`).

**Rationale**: Reusing the existing engine and session factory means no new database connection, no new configuration. Tables auto-create via `Base.metadata.create_all()` which already runs in the app lifespan.

**Alternatives considered**:
- MemoryStore (built-in) — rejected (no persistence across restarts)
- Separate database — rejected (unnecessary, existing Neon Postgres has capacity)
- Redis for threads — rejected (not needed for this scale, adds dependency)

**Key findings**:
- Store items should be serialized as JSON (forward-compatible with SDK updates)
- Use cursor-based pagination (not OFFSET)
- `sort_key` integer column ensures stable ordering within threads
- CASCADE deletes from threads to items
- Thread ownership via `user_id` column, filtered in `load_threads()`

## R4: Docs-Only Widget Rendering

**Decision**: Swizzle `@theme/DocPage/Layout` to inject the ChatWidget component. The widget only renders within this layout, so non-doc pages never see it.

**Rationale**: Docusaurus has a clear separation between doc pages (using DocPage layout) and other pages (landing, blog, custom). Swizzling DocPage/Layout is the official pattern for adding components to all doc pages.

**Alternatives considered**:
- Route detection in Root.tsx — rejected (fragile, requires path matching)
- Custom Docusaurus plugin — rejected (overkill for a single component injection)
- CSS display:none on non-docs — rejected (still loads the component, wastes resources)

**Key findings**:
- `npx docusaurus swizzle @docusaurus/theme-classic DocPage/Layout -- --wrap` creates a wrapper
- The wrapper component receives `children` (the doc content) and can add siblings
- ChatKit React uses `BrowserOnly` for SSR safety in Docusaurus
- `useLocation()` from `@docusaurus/router` available as a safety check

## R5: Auth Integration with ChatKit

**Decision**: Pass JWT access token via custom headers in ChatKit API config. Backend extracts `user_id` in the `/chatkit` endpoint handler and passes it as request context.

**Rationale**: ChatKit's `api` config supports custom headers. The existing auth system stores tokens in localStorage. The backend already has JWT verification logic.

**Alternatives considered**:
- Cookie-based auth — rejected (ChatKit uses fetch, not browser navigation; httpOnly cookies work but tokens are already in localStorage)
- No auth (anonymous only) — rejected (spec requires conversation persistence per user)

**Key findings**:
- ChatKit `useChatKit({ api: { url, headers } })` supports dynamic headers
- Backend: extract token from `Authorization` header, verify JWT, get `user_id`
- Anonymous fallback: if no token, use session-based anonymous ID (no persistence across sessions)
- Thread isolation: `load_threads()` filters by `user_id` in the store
