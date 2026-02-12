import logging

from cohere import AsyncClientV2

from app.models.responses import ChatSource

logger = logging.getLogger(__name__)

CHAT_MODEL = "command-r-08-2024"

SYSTEM_PROMPT = (
    "You are a helpful teaching assistant for the Physical AI & Humanoid Robotics "
    "Handbook. Answer questions based ONLY on the provided textbook content. "
    "If the information is not in the provided documents, say so clearly. "
    "Always cite the specific module, chapter, and lesson where information was found."
)


class CohereChatService:
    def __init__(self, client: AsyncClientV2) -> None:
        self._client = client

    async def generate(
        self, query: str, context_chunks: list[dict]
    ) -> tuple[str, list[ChatSource]]:
        if not context_chunks:
            return (
                "I don't have information on that in the textbook. "
                "Try browsing the table of contents for related topics.",
                [],
            )

        documents = [
            {
                "id": f"{chunk.get('lesson_id', '')}_{i}",
                "data": {
                    "title": chunk.get("section_title", ""),
                    "snippet": chunk.get("content", ""),
                },
            }
            for i, chunk in enumerate(context_chunks)
        ]

        response = await self._client.chat(
            model=CHAT_MODEL,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": query},
            ],
            documents=documents,
        )

        answer = response.message.content[0].text if response.message.content else ""

        sources: list[ChatSource] = []
        seen_lessons: set[str] = set()

        if response.message.citations:
            for citation in response.message.citations:
                for source in citation.sources:
                    doc_id = source.id if hasattr(source, "id") else ""
                    idx_str = doc_id.rsplit("_", 1)[-1] if "_" in doc_id else ""
                    idx = int(idx_str) if idx_str.isdigit() else -1
                    chunk_meta = context_chunks[idx] if 0 <= idx < len(context_chunks) else None
                    lesson_id = chunk_meta.get("lesson_id", "") if chunk_meta else doc_id
                    if lesson_id and lesson_id not in seen_lessons:
                        seen_lessons.add(lesson_id)
                        if chunk_meta:
                            sources.append(
                                ChatSource(
                                    lesson_id=lesson_id,
                                    lesson_title=chunk_meta.get("lesson_title", ""),
                                    section_title=chunk_meta.get("section_title", ""),
                                    module=chunk_meta.get("module", ""),
                                    chapter=chunk_meta.get("chapter", ""),
                                    citation_label=f"[{chunk_meta.get('module', '')}, {chunk_meta.get('chapter', '')}, {chunk_meta.get('lesson_title', '')}]",
                                )
                            )

        if not sources and context_chunks:
            for chunk in context_chunks[:3]:
                lid = chunk.get("lesson_id", "")
                if lid not in seen_lessons:
                    seen_lessons.add(lid)
                    sources.append(
                        ChatSource(
                            lesson_id=lid,
                            lesson_title=chunk.get("lesson_title", ""),
                            section_title=chunk.get("section_title", ""),
                            module=chunk.get("module", ""),
                            chapter=chunk.get("chapter", ""),
                            citation_label=f"[{chunk.get('module', '')}, {chunk.get('chapter', '')}, {chunk.get('lesson_title', '')}]",
                        )
                    )

        return answer, sources
