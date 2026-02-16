# RAG System Integration

Patterns for connecting ChatKit agents to vector databases and retrieval-augmented generation pipelines.

## Table of Contents

1. [Architecture](#architecture)
2. [Qdrant Integration](#qdrant-integration)
3. [Cohere Reranking](#cohere-reranking)
4. [Custom RAG Tool](#custom-rag-tool)
5. [Context Window Management](#context-window-management)
6. [Multi-Source RAG](#multi-source-rag)

## Architecture

```
User message --> ChatKitServer.respond()
                   |
                   v
              Agent with RAG tool
                   |
                   v
              search_knowledge_base(query)
                   |
                   +---> Embed query (OpenAI/Cohere)
                   +---> Vector search (Qdrant/Pinecone)
                   +---> Rerank results (Cohere/cross-encoder)
                   +---> Format context
                   |
                   v
              Agent generates response with retrieved context
```

## Qdrant Integration

```python
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

qdrant = AsyncQdrantClient(url="http://localhost:6333")

async def search_qdrant(
    query_embedding: list[float],
    collection: str = "documents",
    limit: int = 5,
    filters: dict | None = None,
) -> list[dict]:
    """Search Qdrant for similar documents."""
    qdrant_filter = None
    if filters:
        conditions = [
            FieldCondition(key=k, match=MatchValue(value=v))
            for k, v in filters.items()
        ]
        qdrant_filter = Filter(must=conditions)

    results = await qdrant.search(
        collection_name=collection,
        query_vector=query_embedding,
        query_filter=qdrant_filter,
        limit=limit,
        with_payload=True,
    )
    return [
        {
            "id": str(hit.id),
            "score": hit.score,
            "text": hit.payload.get("text", ""),
            "metadata": {k: v for k, v in hit.payload.items() if k != "text"},
        }
        for hit in results
    ]
```

### Embedding with OpenAI

```python
from openai import AsyncOpenAI

openai_client = AsyncOpenAI()

async def embed_query(text: str) -> list[float]:
    response = await openai_client.embeddings.create(
        model="text-embedding-3-small",
        input=text,
    )
    return response.data[0].embedding
```

### Embedding with Cohere

```python
from cohere import AsyncClientV2

cohere_client = AsyncClientV2()

async def embed_query_cohere(text: str) -> list[float]:
    response = await cohere_client.embed(
        texts=[text],
        model="embed-v4.0",
        input_type="search_query",
        embedding_types=["float"],
    )
    return response.embeddings.float_[0]
```

## Cohere Reranking

```python
async def rerank_results(query: str, documents: list[dict], top_n: int = 3) -> list[dict]:
    """Rerank search results using Cohere for better relevance."""
    if not documents:
        return []
    response = await cohere_client.rerank(
        model="rerank-v3.5",
        query=query,
        documents=[d["text"] for d in documents],
        top_n=top_n,
    )
    return [documents[r.index] for r in response.results]
```

## Custom RAG Tool

Complete tool for ChatKit agent integration:

```python
from agents import function_tool, RunContextWrapper
from chatkit.agents import AgentContext
from chatkit.types import ProgressUpdateEvent

@function_tool()
async def search_knowledge_base(
    ctx: RunContextWrapper[AgentContext],
    query: str,
    topic: str | None = None,
) -> str:
    """Search the knowledge base for relevant information.

    Args:
        query: Natural language search query
        topic: Optional topic filter (e.g., "ros2", "kinematics", "sensors")
    """
    # Stream progress to client
    await ctx.context.stream(
        ProgressUpdateEvent(icon="search", text="Searching knowledge base...")
    )

    # Embed
    embedding = await embed_query(query)

    # Search
    filters = {"topic": topic} if topic else None
    results = await search_qdrant(embedding, collection="documents", limit=10, filters=filters)

    # Rerank
    reranked = await rerank_results(query, results, top_n=5)

    await ctx.context.stream(
        ProgressUpdateEvent(icon="check", text=f"Found {len(reranked)} relevant sources")
    )

    # Format for LLM context
    if not reranked:
        return "No relevant information found in the knowledge base."

    chunks = []
    for i, doc in enumerate(reranked, 1):
        source = doc["metadata"].get("source", "unknown")
        chunks.append(f"[Source {i}: {source}]\n{doc['text']}")

    return "\n\n---\n\n".join(chunks)
```

### Wire into agent

```python
agent = Agent(
    name="assistant",
    instructions="""You are a knowledgeable assistant.
When users ask domain-specific questions, use search_knowledge_base to find relevant information.
Always cite your sources when using retrieved information.
If the knowledge base doesn't have relevant results, say so honestly.""",
    tools=[search_knowledge_base],
    model="gpt-4o-mini",
)
```

## Context Window Management

### Thread history + RAG context budgeting

```python
class MyChatKitServer(ChatKitServer[MyRequestContext]):
    async def respond(self, thread, input_user_message, context):
        # Load limited history to leave room for RAG context
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=10, order="desc", context=context
        )
        items = list(reversed(items_page.data))

        input_items = await simple_to_agent_input(items)
        agent_context = AgentContext(
            thread=thread, store=self.store, request_context=context
        )

        result = Runner.run_streamed(agent, input_items, context=agent_context)
        async for event in stream_agent_response(agent_context, result):
            yield event
```

### Hidden context injection

Pass page-level or user-level context that the agent sees but the user doesn't:

```python
@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    # Extract context from headers or session
    user_id = request.headers.get("X-User-Id", "anonymous")
    page_context = request.headers.get("X-Page-Context", "")

    context = MyRequestContext(
        user_id=user_id,
        page_context=page_context,
    )
    result = await server.process(await request.body(), context=context)
    ...
```

Then inject into agent instructions dynamically:

```python
agent = Agent(
    name="assistant",
    instructions=lambda ctx: f"""You are a helpful assistant.
<USER_CONTEXT>
User is viewing: {ctx.context.request_context.page_context}
</USER_CONTEXT>
Answer questions using search_knowledge_base when needed.""",
    tools=[search_knowledge_base],
)
```

## Multi-Source RAG

For projects with multiple knowledge sources:

```python
@function_tool()
async def search_textbook(ctx: RunContextWrapper[AgentContext], query: str) -> str:
    """Search the textbook content."""
    embedding = await embed_query(query)
    results = await search_qdrant(embedding, collection="textbook-chapters", limit=5)
    return format_results(results)

@function_tool()
async def search_api_docs(ctx: RunContextWrapper[AgentContext], query: str) -> str:
    """Search API documentation."""
    embedding = await embed_query(query)
    results = await search_qdrant(embedding, collection="api-docs", limit=5)
    return format_results(results)

agent = Agent(
    name="assistant",
    instructions="Use search_textbook for learning content, search_api_docs for API questions.",
    tools=[search_textbook, search_api_docs],
)
```
