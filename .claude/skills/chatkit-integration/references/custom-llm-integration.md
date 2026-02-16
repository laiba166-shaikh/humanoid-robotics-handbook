# Custom LLM Integration

Use non-OpenAI LLM providers (Cohere, Anthropic, local models) with ChatKit's self-hosted backend.

## Table of Contents

1. [Manual Response Streaming](#manual-response-streaming)
2. [Cohere Integration](#cohere-integration)
3. [Anthropic Claude Integration](#anthropic-claude-integration)
4. [Local Model Integration](#local-model-integration)
5. [Provider-Agnostic Pattern](#provider-agnostic-pattern)

## Manual Response Streaming

When not using OpenAI Agents SDK, yield ChatKit events directly:

```python
from chatkit.server import ChatKitServer
from chatkit.types import (
    ThreadMetadata, UserMessageItem, AssistantMessageItem,
    ThreadStreamEvent, ThreadItemCreatedEvent, ThreadItemDoneEvent,
    TextDeltaEvent,
)
from collections.abc import AsyncIterator

class CustomLLMServer(ChatKitServer[MyRequestContext]):
    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: MyRequestContext,
    ) -> AsyncIterator[ThreadStreamEvent]:
        # Load history
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=20, order="desc", context=context
        )
        messages = self._convert_to_messages(list(reversed(items_page.data)))

        # Create assistant message placeholder
        msg_id = await self.store.generate_item_id("message", thread, context)
        yield ThreadItemCreatedEvent(
            item=AssistantMessageItem(id=msg_id, content="")
        )

        # Stream from your LLM
        full_text = ""
        async for chunk in self._stream_llm(messages):
            full_text += chunk
            yield TextDeltaEvent(item_id=msg_id, delta=chunk)

        # Finalize
        yield ThreadItemDoneEvent(
            item=AssistantMessageItem(id=msg_id, content=full_text)
        )
```

## Cohere Integration

```python
from cohere import AsyncClientV2

cohere = AsyncClientV2()

class CohereChatKitServer(ChatKitServer[MyRequestContext]):
    def _items_to_cohere_messages(self, items):
        """Convert ChatKit thread items to Cohere chat format."""
        messages = []
        for item in items:
            if hasattr(item, "content") and item.type == "user_message":
                messages.append({"role": "user", "content": item.content})
            elif hasattr(item, "content") and item.type == "assistant_message":
                messages.append({"role": "assistant", "content": item.content})
        return messages

    async def respond(self, thread, input_user_message, context):
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=20, order="desc", context=context
        )
        messages = self._items_to_cohere_messages(list(reversed(items_page.data)))

        # Add system preamble
        messages.insert(0, {"role": "system", "content": "You are a helpful assistant."})

        msg_id = await self.store.generate_item_id("message", thread, context)
        yield ThreadItemCreatedEvent(
            item=AssistantMessageItem(id=msg_id, content="")
        )

        full_text = ""
        response = await cohere.chat_stream(
            model="command-a-08-2025",
            messages=messages,
        )
        async for event in response:
            if event.type == "content-delta":
                delta = event.delta.message.content.text
                full_text += delta
                yield TextDeltaEvent(item_id=msg_id, delta=delta)

        yield ThreadItemDoneEvent(
            item=AssistantMessageItem(id=msg_id, content=full_text)
        )
```

### Cohere with RAG documents

```python
# When using Cohere's built-in RAG with documents parameter
response = await cohere.chat_stream(
    model="command-a-08-2025",
    messages=messages,
    documents=[
        {"id": doc["id"], "data": {"text": doc["text"], "source": doc["source"]}}
        for doc in retrieved_docs
    ],
)
```

## Anthropic Claude Integration

```python
from anthropic import AsyncAnthropic

anthropic = AsyncAnthropic()

class ClaudeChatKitServer(ChatKitServer[MyRequestContext]):
    async def respond(self, thread, input_user_message, context):
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=20, order="desc", context=context
        )
        messages = self._items_to_anthropic_messages(list(reversed(items_page.data)))

        msg_id = await self.store.generate_item_id("message", thread, context)
        yield ThreadItemCreatedEvent(
            item=AssistantMessageItem(id=msg_id, content="")
        )

        full_text = ""
        async with anthropic.messages.stream(
            model="claude-sonnet-4-5-20250929",
            max_tokens=4096,
            system="You are a helpful assistant.",
            messages=messages,
        ) as stream:
            async for text in stream.text_stream:
                full_text += text
                yield TextDeltaEvent(item_id=msg_id, delta=text)

        yield ThreadItemDoneEvent(
            item=AssistantMessageItem(id=msg_id, content=full_text)
        )
```

## Local Model Integration

Using any OpenAI-compatible local server (Ollama, vLLM, llama.cpp):

```python
from openai import AsyncOpenAI

local_client = AsyncOpenAI(
    base_url="http://localhost:11434/v1",  # Ollama
    api_key="ollama",  # Required but unused
)

class LocalModelServer(ChatKitServer[MyRequestContext]):
    async def respond(self, thread, input_user_message, context):
        items_page = await self.store.load_thread_items(
            thread.id, after=None, limit=20, order="desc", context=context
        )
        messages = self._items_to_openai_messages(list(reversed(items_page.data)))

        msg_id = await self.store.generate_item_id("message", thread, context)
        yield ThreadItemCreatedEvent(
            item=AssistantMessageItem(id=msg_id, content="")
        )

        full_text = ""
        stream = await local_client.chat.completions.create(
            model="llama3.2",
            messages=messages,
            stream=True,
        )
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                delta = chunk.choices[0].delta.content
                full_text += delta
                yield TextDeltaEvent(item_id=msg_id, delta=delta)

        yield ThreadItemDoneEvent(
            item=AssistantMessageItem(id=msg_id, content=full_text)
        )
```

## Provider-Agnostic Pattern

Abstract the LLM provider behind an interface:

```python
from abc import ABC, abstractmethod
from collections.abc import AsyncIterator

class LLMProvider(ABC):
    @abstractmethod
    async def stream_chat(
        self, messages: list[dict], system: str = ""
    ) -> AsyncIterator[str]:
        """Yield text deltas from the LLM."""
        ...

class OpenAIProvider(LLMProvider):
    async def stream_chat(self, messages, system=""):
        if system:
            messages = [{"role": "system", "content": system}] + messages
        stream = await self.client.chat.completions.create(
            model=self.model, messages=messages, stream=True
        )
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content

class CohereProvider(LLMProvider):
    async def stream_chat(self, messages, system=""):
        if system:
            messages = [{"role": "system", "content": system}] + messages
        response = await self.client.chat_stream(model=self.model, messages=messages)
        async for event in response:
            if event.type == "content-delta":
                yield event.delta.message.content.text

# Usage in server
class FlexibleChatKitServer(ChatKitServer[MyRequestContext]):
    def __init__(self, store, provider: LLMProvider):
        super().__init__(store=store)
        self.provider = provider

    async def respond(self, thread, input_user_message, context):
        # ... load history, create msg_id ...
        async for delta in self.provider.stream_chat(messages, system="You are helpful."):
            full_text += delta
            yield TextDeltaEvent(item_id=msg_id, delta=delta)
        # ... finalize ...
```
