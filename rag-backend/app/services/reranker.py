import logging

from cohere import AsyncClientV2

logger = logging.getLogger(__name__)

RERANK_MODEL = "rerank-v3.5"


class CohereRerankService:
    def __init__(self, client: AsyncClientV2, enabled: bool = True) -> None:
        self._client = client
        self._enabled = enabled

    async def rerank(
        self, query: str, documents: list[dict], top_n: int = 5
    ) -> list[dict]:
        if not self._enabled or not documents:
            return documents[:top_n]

        top_score = max((d.get("score", 0) for d in documents), default=0)
        if top_score > 0.85:
            logger.info("Skipping rerank — top cosine score %.3f > 0.85", top_score)
            return documents[:top_n]

        doc_texts = [d.get("content", "") for d in documents]

        response = await self._client.rerank(
            model=RERANK_MODEL,
            query=query,
            documents=doc_texts,
            top_n=top_n,
        )

        reranked = []
        for result in response.results:
            original = documents[result.index]
            reranked.append({**original, "score": result.relevance_score})

        return reranked
