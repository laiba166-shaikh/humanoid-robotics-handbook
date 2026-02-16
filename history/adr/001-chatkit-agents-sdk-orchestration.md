# ADR-001: ChatKit and Agents SDK as Chat Orchestration Layer

> **Scope**: Covers the choice of OpenAI ChatKit (JS + Python SDK) for the chat UI and server protocol, OpenAI Agents SDK for agent reasoning and tool orchestration, and the retention of Cohere as the underlying RAG engine.

- **Status:** Accepted
- **Date:** 2026-02-16
- **Feature:** 005-rag-chatbot-ui
- **Context:** The project needs a conversational chat interface for the textbook that streams responses, supports multi-turn conversations, and integrates with the existing Cohere-based RAG pipeline (embed → Qdrant vector search → rerank → generate). The constitution specifies "OpenAI ChatKit SDK" for Chat UI. We need to decide how to wire the chat frontend to the existing backend services and whether to introduce new orchestration layers.

## Decision

- **Chat UI**: OpenAI ChatKit JS (`@openai/chatkit-react`) — pre-built React widget with streaming, theming, thread management
- **Chat Server**: OpenAI ChatKit Python SDK (`openai-chatkit`) — `ChatKitServer` subclass on FastAPI handling the ChatKit protocol via `/chatkit` endpoint
- **Agent Orchestration**: OpenAI Agents SDK (`openai-agents`) — `Agent` with `@function_tool` wrapping existing RAG services
- **RAG Engine**: Cohere (unchanged) — embed-english-v3.0, Command-R, rerank-v3.5 via existing services
- **Agent Model**: OpenAI `gpt-4o-mini` for agent reasoning (requires `OPENAI_API_KEY`)

The agent acts as an orchestration layer: it receives the user's message, decides to call the `search_textbook` tool (which internally uses the existing `QueryExpander → CohereEmbedService → QdrantService → CohereRerankService` pipeline), and generates a response with citations. The actual knowledge retrieval and context remain Cohere-powered.

## Consequences

### Positive

- **Production-ready UI**: ChatKit provides streaming, theming, thread management, progress indicators out of the box — no custom chat component needed
- **Clean separation**: Agent orchestration (OpenAI) is separate from RAG retrieval (Cohere) — each can evolve independently
- **Multi-turn for free**: ChatKit manages thread history and the Agents SDK passes conversation context to the agent automatically
- **Tool streaming**: `ProgressUpdateEvent` gives users real-time feedback during retrieval without custom SSE implementation
- **Constitution alignment**: ChatKit SDK is already in the tech stack constitution

### Negative

- **Dual-provider dependency**: Both OpenAI (agent reasoning) and Cohere (RAG) APIs must be available — two external failure points
- **Cost**: OpenAI API calls for agent reasoning (gpt-4o-mini) add cost on top of Cohere free tier. Each user message = 1 OpenAI completion + 1 Cohere embed + 1 Cohere rerank + 1 Qdrant search
- **SDK coupling**: ChatKit uses its own binary protocol on `/chatkit`, not the project's standard REST JSON envelope. This endpoint doesn't follow the existing `/api/*` conventions
- **New env var**: `OPENAI_API_KEY` is a new secret that must be managed in deployment

## Alternatives Considered

**Alternative A: Direct Cohere Streaming (no ChatKit, no Agents SDK)**
- Use Cohere `chat_stream()` directly, build custom React chat component, manage threads manually
- Why rejected: Massive frontend effort to build streaming chat UI, thread management, progress indicators from scratch. Cohere's streaming doesn't produce ChatKit-compatible events.

**Alternative B: Cohere-only with Custom SSE**
- Keep Cohere for everything, implement custom SSE streaming endpoint, build custom React chat
- Why rejected: Would need to build the entire chat UI from scratch (streaming, theming, thread sidebar, error handling). The constitution already specifies ChatKit.

**Alternative C: Replace Cohere with OpenAI for RAG**
- Use OpenAI embeddings + GPT-4o-mini for both agent and generation, drop Cohere entirely
- Why rejected: OpenAI embeddings and generation require paid API credits. Cohere free tier is already working and proven. Would invalidate all existing ingestion (different embedding dimensions).

## References

- Feature Spec: [specs/005-rag-chatbot-ui/spec.md](../../specs/005-rag-chatbot-ui/spec.md)
- Implementation Plan: [specs/005-rag-chatbot-ui/plan.md](../../specs/005-rag-chatbot-ui/plan.md)
- Research: [specs/005-rag-chatbot-ui/research.md](../../specs/005-rag-chatbot-ui/research.md) (R1, R2)
- Related ADRs: None (first ADR)
- Evaluator Evidence: PHR 002 (plan phase)
