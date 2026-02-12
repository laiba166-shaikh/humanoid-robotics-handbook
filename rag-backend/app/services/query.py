import logging
import re

logger = logging.getLogger(__name__)

STRUCTURAL_KEYWORDS = {
    "learning objectives": "goals, what will I learn, skills gained, outcomes",
    "key takeaways": "main points, summary, important concepts, highlights",
    "prerequisites": "requirements, what do I need, prior knowledge, before starting",
    "next steps": "what's next, continue learning, follow-up, further reading",
    "check your understanding": "quiz, review questions, self-assessment, test knowledge",
    "summary": "overview, recap, main points, key concepts",
}


class QueryExpander:
    def expand(self, query: str) -> tuple[str, bool]:
        query_lower = query.lower()
        for keyword, synonyms in STRUCTURAL_KEYWORDS.items():
            if re.search(rf"\b{re.escape(keyword)}\b", query_lower):
                expanded = f"{query} ({synonyms})"
                logger.info("Expanded query: %s → %s", query, expanded)
                return expanded, True
        return query, False
