---
name: chatkit-integration
description: >
  Build production-ready conversational AI chat interfaces using OpenAI ChatKit (JS + Python SDK)
  with custom FastAPI backends, AI agent integration, conversation history persistence, and RAG system
  connectivity. Use when the user asks to: add a chat UI, build a chatbot interface, integrate ChatKit,
  create a conversational AI widget, add AI chat to their app, set up chat with streaming, build a
  support chat, implement conversation history, connect chat to RAG/vector search, or wire up AI agents
  with a chat frontend. Triggers on phrases like "add chat", "chat UI", "chatkit", "conversational interface",
  "AI chatbot", "chat widget", "streaming chat", "chat with RAG", "conversation history".
---

# ChatKit Integration

Build conversational AI interfaces with OpenAI ChatKit JS frontend + ChatKit Python SDK backend on FastAPI.

## Architecture Overview

```
[React/Next.js Frontend]     [FastAPI Backend]          [AI/Data Layer]
  ChatKit React component --> /chatkit POST endpoint --> Agent + Tools
  useChatKit hook             ChatKitServer subclass     RAG retrieval
  Widgets + Theming           Store (PostgreSQL)         Vector DB (Qdrant)
  Streaming SSE <------------ StreamingResponse          LLM provider
```

## Quick Start Workflow

1. **Scaffold backend** -- FastAPI + ChatKit Python SDK server
2. **Implement Store** -- PostgreSQL persistence for threads/messages
3. **Wire AI agent** -- OpenAI Agents SDK or custom LLM integration
4. **Add RAG tools** -- Vector search tool for context retrieval
5. **Scaffold frontend** -- React component with `useChatKit` hook
6. **Configure theming** -- Brand colors, dark/light mode
7. **Add production features** -- Auth, error handling, file uploads

## Step 1: Backend Setup

### Install dependencies

```bash
pip install openai-chatkit fastapi uvicorn sqlalchemy asyncpg
```

### Minimal FastAPI server

```python
# server/main.py
from fastapi import FastAPI, Request
from fastapi.responses import Response, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from chatkit.server import ChatKitServer, StreamingResult

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# See Step 2 for store, Step 3 for server implementation
server = MyChatKitServer(store=my_store)

@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    result = await server.process(await request.body(), context={})
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")
```

## Step 2: Conversation Persistence

Implement the `Store` abstract class. See [references/store-postgresql.md](references/store-postgresql.md) for complete PostgreSQL implementation.

### Store interface (required methods)

```python
from chatkit.store import Store, Page
from chatkit.types import Thread, ThreadItem

class PostgresStore(Store[MyRequestContext]):
    # Thread CRUD
    async def generate_thread_id(self, context) -> str
    async def load_thread(self, thread_id, context) -> Thread
    async def save_thread(self, thread, context) -> None
    async def delete_thread(self, thread_id, context) -> None
    async def load_threads(self, limit, after, order, context) -> Page[Thread]

    # Item CRUD
    async def generate_item_id(self, item_type, thread, context) -> str
    async def add_thread_item(self, thread_id, item, context) -> None
    async def save_item(self, thread_id, item, context) -> None
    async def load_item(self, thread_id, item_id, context) -> None
    async def delete_thread_item(self, thread_id, item_id, context) -> None
    async def load_thread_items(self, thread_id, limit, after, order, context) -> Page[ThreadItem]

    # Attachment metadata
    async def load_attachment(self, attachment_id, context) -> Attachment
    async def save_attachment(self, attachment, context) -> None
    async def delete_attachment(self, attachment_id, context) -> None
```

**Key patterns:**
- Serialize thread items as JSON (forward-compatible as SDK evolves)
- Use cursor-based pagination (not OFFSET)
- Index `thread_id` and `created_at` columns
- Default ID format: `thr_{uuid4_hex[:8]}`, `msg_{uuid4_hex[:8]}`, etc.

## Step 3: AI Agent Integration

### ChatKitServer with respond method

```python
from chatkit.server import ChatKitServer
from chatkit.types import (
    ThreadMetadata, UserMessageItem, AssistantMessageItem,
    ThreadStreamEvent, ThreadItemDoneEvent
)
from collections.abc import AsyncIterator

class MyChatKitServer(ChatKitServer[MyRequestContext]):
    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: MyRequestContext,
    ) -> AsyncIterator[ThreadStreamEvent]:
        # Load recent history
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=20, order="desc", context=context
        )
        items = list(reversed(items_page.data))

        # Option A: OpenAI Agents SDK
        from agents import Agent, Runner
        from chatkit.agents import AgentContext, simple_to_agent_input, stream_agent_response

        input_items = await simple_to_agent_input(items)
        agent_context = AgentContext(
            thread=thread, store=self.store, request_context=context
        )
        result = Runner.run_streamed(agent, input_items, context=agent_context)
        async for event in stream_agent_response(agent_context, result):
            yield event

        # Option B: Custom LLM (any provider)
        # See references/custom-llm-integration.md
```

### Agent with RAG tools

```python
from agents import Agent, function_tool

@function_tool()
async def search_knowledge_base(ctx, query: str) -> str:
    """Search the knowledge base for relevant information."""
    # Vector search against Qdrant/Pinecone/etc.
    results = await vector_store.search(query, limit=5)
    return "\n\n".join([r.payload["text"] for r in results])

agent = Agent(
    name="assistant",
    instructions="You are a helpful assistant. Use search_knowledge_base for domain questions.",
    tools=[search_knowledge_base],
    model="gpt-4o-mini",
)
```

See [references/rag-integration.md](references/rag-integration.md) for complete RAG patterns with Qdrant, Cohere, and custom providers.

## Step 4: Frontend Setup

### Install

```bash
npm install @openai/chatkit-react
```

### React component

```tsx
import { ChatKit, useChatKit } from "@openai/chatkit-react";

export function AIChatWidget() {
  const { control } = useChatKit({
    api: {
      url: "http://localhost:8000/chatkit",
      domainKey: "local-dev",
    },
  });

  return (
    <ChatKit
      control={control}
      className="h-[600px] w-full max-w-[400px]"
    />
  );
}
```

### Vanilla JS (framework-agnostic)

```html
<script src="https://cdn.platform.openai.com/deployments/chatkit/chatkit.js"></script>
<script type="module">
  import "@openai/chatkit";
  const chatkit = document.createElement("openai-chatkit");
  chatkit.setOptions({
    api: { url: "http://localhost:8000/chatkit", domainKey: "local-dev" },
  });
  document.getElementById("chat-root")?.append(chatkit);
</script>
```

### Theming

```tsx
<ChatKit
  control={control}
  theme={{
    colorScheme: "dark",       // "light" | "dark" | "auto"
    accentColor: "#6366f1",    // brand color
  }}
/>
```

See [references/frontend-patterns.md](references/frontend-patterns.md) for widgets, actions, theming deep-dive, and layout patterns.

## Step 5: Production Checklist

- [ ] Replace `MemoryStore` with PostgreSQL store
- [ ] Add user authentication to `/chatkit` endpoint (extract user_id from JWT)
- [ ] Set CORS origins to production domains only
- [ ] Add rate limiting
- [ ] Configure `domainKey` per environment
- [ ] Add error handling with `ErrorEvent` yields
- [ ] Set up health check endpoint
- [ ] Enable attachment store if file uploads needed
- [ ] Add input/output guardrails for safety

## Error Handling Pattern

```python
from chatkit.types import ErrorEvent
from agents.exceptions import InputGuardrailTripwireTriggered, OutputGuardrailTripwireTriggered

try:
    async for event in stream_agent_response(agent_context, result):
        yield event
except InputGuardrailTripwireTriggered:
    yield ErrorEvent(message="Message blocked for safety.")
except OutputGuardrailTripwireTriggered:
    yield ErrorEvent(message="Response blocked.", allow_retry=False)
except Exception as e:
    yield ErrorEvent(message="Something went wrong. Please try again.")
```

## Tool Streaming (progress updates)

```python
from agents import function_tool, RunContextWrapper
from chatkit.agents import AgentContext
from chatkit.types import ProgressUpdateEvent

@function_tool()
async def search_documents(ctx: RunContextWrapper[AgentContext], query: str):
    await ctx.context.stream(ProgressUpdateEvent(icon="search", text="Searching..."))
    results = await do_search(query)
    await ctx.context.stream(ProgressUpdateEvent(icon="check", text=f"Found {len(results)} results"))
    return format_results(results)
```

## Reference Files

- [references/store-postgresql.md](references/store-postgresql.md) -- Complete PostgreSQL store implementation with SQLAlchemy
- [references/rag-integration.md](references/rag-integration.md) -- RAG patterns with Qdrant, Cohere, and custom vector stores
- [references/custom-llm-integration.md](references/custom-llm-integration.md) -- Using non-OpenAI LLMs (Cohere, Anthropic, local models)
- [references/frontend-patterns.md](references/frontend-patterns.md) -- Widgets, actions, theming, layout, and advanced UI patterns
