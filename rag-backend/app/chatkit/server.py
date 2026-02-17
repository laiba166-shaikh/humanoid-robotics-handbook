import logging
from collections.abc import AsyncIterator

from agents import Runner
from chatkit.server import ChatKitServer
from chatkit.agents import AgentContext, simple_to_agent_input, stream_agent_response
from chatkit.types import (
    ThreadMetadata,
    UserMessageItem,
    ThreadStreamEvent,
    ErrorEvent,
)

from app.chatkit.agent import create_agent

logger = logging.getLogger(__name__)

MAX_INPUT_LENGTH = 4000


class TextbookChatKitServer(ChatKitServer[dict]):
    """ChatKit server that wraps the existing RAG pipeline via OpenAI Agents SDK."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.agent = create_agent()

    async def respond(
        self,
        thread: ThreadMetadata,
        input_user_message: UserMessageItem | None,
        context: dict,
    ) -> AsyncIterator[ThreadStreamEvent]:
        try:
            # Input length validation
            if input_user_message and hasattr(input_user_message, "content"):
                content = input_user_message.content
                if isinstance(content, str) and len(content) > MAX_INPUT_LENGTH:
                    logger.warning(
                        "Truncating user message from %d to %d chars",
                        len(content),
                        MAX_INPUT_LENGTH,
                    )

            # Load recent thread history for multi-turn context
            items_page = await self.store.load_thread_items(
                thread.id, after=None, limit=20, order="desc", context=context
            )
            items = list(reversed(items_page.data))

            # Convert to agent input format
            input_items = await simple_to_agent_input(items)

            # Create agent context with store and request context
            agent_context = AgentContext(
                thread=thread,
                store=self.store,
                request_context=context,
            )

            # Run the agent with streaming
            result = Runner.run_streamed(
                self.agent,
                input_items,
                context=agent_context,
            )

            # Stream events back to client
            async for event in stream_agent_response(agent_context, result):
                yield event

            # Auto-generate thread title from first user message
            if input_user_message and hasattr(input_user_message, "content"):
                await self._maybe_set_thread_title(thread, input_user_message, context)

        except Exception as e:
            error_msg = "The chatbot is temporarily unavailable. Please try again laaaater."

            # Rate limit handling
            err_str = str(e).lower()
            if "429" in err_str or "rate" in err_str:
                error_msg = "Too many requests, please wait a moment."
            elif "timeout" in err_str:
                error_msg = "The request timed out. Please try again."

            logger.error("ChatKit respond error: %s", e, exc_info=True)
            yield ErrorEvent(message=error_msg, allow_retry=True)

    async def _maybe_set_thread_title(
        self,
        thread: ThreadMetadata,
        user_message: UserMessageItem,
        context: dict,
    ) -> None:
        """Set thread title from the first user message if not already set."""
        try:
            existing = await self.store.load_thread(thread.id, context)
            metadata = existing.metadata or {}
            if metadata.get("title"):
                return

            content = user_message.content if hasattr(user_message, "content") else ""
            if isinstance(content, str) and content.strip():
                title = content.strip()[:80]
                if len(content.strip()) > 80:
                    title += "..."
                metadata["title"] = title
                existing.metadata = metadata
                await self.store.save_thread(existing, context)
        except Exception:
            logger.debug("Failed to set thread title", exc_info=True)
