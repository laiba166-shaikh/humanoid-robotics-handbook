import logging

from openai import OpenAI
from agents import Agent, function_tool, RunContextWrapper, OpenAIChatCompletionsModel
from chatkit.agents import AgentContext
from chatkit.types import ProgressUpdateEvent
from app.config import get_settings

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = (
    "You are a helpful teaching assistant for the Physical AI & Humanoid Robotics textbook. "
    "You answer questions based ONLY on the textbook content retrieved via the search_textbook tool. "
    "You cite sources as [Module X, Chapter Y, Lesson Z]. "
    "You adapt explanations to the student's level. "
    "If the information is not in the textbook, say: "
    '"I don\'t have information on that in the textbook." '
    "and suggest relevant lessons if possible. "
    "Always call search_textbook before answering domain questions."
)


@function_tool()
async def search_textbook(
    ctx: RunContextWrapper[AgentContext],
    query: str,
) -> str:
    """Search the Physical AI & Humanoid Robotics textbook for relevant information.

    Args:
        query: Natural language search query about textbook content.
    """
    app_state = ctx.context.request_context.get("app_state")
    if not app_state:
        return "Search service is not available."

    await ctx.context.stream(
        ProgressUpdateEvent(icon="search", text="Searching knowledge base...")
    )

    query_expander = app_state.query_expander
    embed_service = app_state.embed_service
    vectorstore = app_state.vectorstore
    reranker = app_state.reranker

    expanded_query, _ = query_expander.expand(query)
    vector = await embed_service.embed_query(expanded_query)
    raw_results = await vectorstore.search(vector=vector, top_k=10)
    reranked = await reranker.rerank(query=expanded_query, documents=raw_results, top_n=5)

    await ctx.context.stream(
        ProgressUpdateEvent(icon="check", text=f"Found {len(reranked)} relevant sources")
    )

    if not reranked:
        return "No relevant information found in the textbook."

    chunks = []
    for i, doc in enumerate(reranked, 1):
        module = doc.get("module", "")
        chapter = doc.get("chapter", "")
        lesson = doc.get("lesson_title", "")
        section = doc.get("section_title", "")
        content = doc.get("content", "")
        source_label = f"[{module}, {chapter}, {lesson}]" if module else f"[Source {i}]"
        chunks.append(f"[Source {i}: {source_label}]\nSection: {section}\n{content}")

    return "\n\n---\n\n".join(chunks)


def create_agent() -> Agent:
    settings = get_settings()
    print('api_key: ', settings.gemini_api_key or settings.openai_api_key)
    # client = OpenAI(api_key=settings.gemini_api_key, base_url=settings.chatbot_base_url)
    # model=OpenAIChatCompletionsModel(model="gemini-2.0-flash",openai_client=client)
    return Agent(
        name="textbook-assistant",
        instructions=SYSTEM_PROMPT,
        tools=[search_textbook],
        model='gpt-4o-mini',
    )
