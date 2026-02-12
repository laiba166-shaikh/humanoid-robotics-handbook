import logging
import re

import tiktoken

from app.models.domain import Chunk, LessonFrontmatter, SectionType

logger = logging.getLogger(__name__)

TOKENIZER = tiktoken.get_encoding("cl100k_base")

SHORT_THRESHOLD = 200
OPTIMAL_MAX = 700
OVERLAP_TOKENS = 100

STRUCTURAL_TITLES = {
    "learning objectives",
    "key takeaways",
    "check your understanding",
    "next steps",
    "prerequisites",
    "summary",
}


def _count_tokens(text: str) -> int:
    return len(TOKENIZER.encode(text))


def _count_code_blocks(content: str) -> int:
    return len(re.findall(r"```", content)) // 2


class AdaptiveChunker:
    def classify_section(self, title: str, content: str) -> SectionType:
        title_lower = title.lower().strip()
        if title_lower in STRUCTURAL_TITLES:
            return SectionType.structural
        if _count_code_blocks(content) >= 2:
            return SectionType.code_heavy
        return SectionType.instructional

    def enrich_short_section(
        self, title: str, content: str, frontmatter: LessonFrontmatter
    ) -> str:
        prefix = (
            f"Context: {frontmatter.module} > {frontmatter.chapter} > "
            f"{frontmatter.title}\n"
            f"Section: {title}\n\n"
        )
        return prefix + content

    def split_long_section(
        self, title: str, content: str, frontmatter: LessonFrontmatter
    ) -> list[dict]:
        """Split content >700 tokens at paragraph/code-block boundaries with overlap."""
        paragraphs = _split_at_boundaries(content)

        sub_chunks: list[dict] = []
        current_parts: list[str] = []
        current_tokens = 0

        for para in paragraphs:
            para_tokens = _count_tokens(para)

            if current_tokens + para_tokens > OPTIMAL_MAX and current_parts:
                chunk_text = "\n\n".join(current_parts)
                sub_chunks.append({"text": chunk_text, "tokens": _count_tokens(chunk_text)})

                overlap_parts: list[str] = []
                overlap_tokens = 0
                for p in reversed(current_parts):
                    pt = _count_tokens(p)
                    if overlap_tokens + pt > OVERLAP_TOKENS:
                        break
                    overlap_parts.insert(0, p)
                    overlap_tokens += pt

                current_parts = overlap_parts + [para]
                current_tokens = overlap_tokens + para_tokens
            else:
                current_parts.append(para)
                current_tokens += para_tokens

        if current_parts:
            chunk_text = "\n\n".join(current_parts)
            sub_chunks.append({"text": chunk_text, "tokens": _count_tokens(chunk_text)})

        total = len(sub_chunks)
        result = []
        for i, sc in enumerate(sub_chunks):
            text_with_title = f"## {title}\n\n{sc['text']}"
            result.append({
                "text": text_with_title,
                "tokens": _count_tokens(text_with_title),
                "chunk_index": i,
                "total_chunks": total,
            })

        return result

    def create_chunks(
        self,
        sections: list[dict],
        frontmatter: LessonFrontmatter,
    ) -> list[Chunk]:
        chunks: list[Chunk] = []

        for section in sections:
            title = section["title"]
            content = section["content"]
            section_type = self.classify_section(title, content)
            token_count = _count_tokens(content)

            if section_type == SectionType.structural and token_count < SHORT_THRESHOLD:
                enriched = self.enrich_short_section(title, content, frontmatter)
                chunks.append(
                    _make_chunk(
                        content=enriched,
                        title=title,
                        section_type=section_type,
                        frontmatter=frontmatter,
                        token_count=_count_tokens(enriched),
                        enrichment_strategy="context_injection",
                    )
                )
            elif token_count > OPTIMAL_MAX:
                sub_chunks = self.split_long_section(title, content, frontmatter)
                for sc in sub_chunks:
                    chunks.append(
                        _make_chunk(
                            content=sc["text"],
                            title=title,
                            section_type=section_type,
                            frontmatter=frontmatter,
                            token_count=sc["tokens"],
                            chunk_index=sc["chunk_index"],
                            total_chunks=sc["total_chunks"],
                            enrichment_strategy="overlapping_windows",
                        )
                    )
                    if token_count > 800:
                        logger.warning(
                            "Oversized section '%s' in %s: %d tokens (split into %d chunks)",
                            title,
                            frontmatter.id,
                            token_count,
                            sc["total_chunks"],
                        )
            else:
                chunks.append(
                    _make_chunk(
                        content=content,
                        title=title,
                        section_type=section_type,
                        frontmatter=frontmatter,
                        token_count=token_count,
                    )
                )

        return chunks


def _make_chunk(
    content: str,
    title: str,
    section_type: SectionType,
    frontmatter: LessonFrontmatter,
    token_count: int,
    chunk_index: int = 0,
    total_chunks: int = 1,
    enrichment_strategy: str | None = None,
) -> Chunk:
    return Chunk(
        content=content,
        lesson_id=frontmatter.id,
        lesson_title=frontmatter.title,
        section_title=title,
        section_type=section_type,
        module=frontmatter.module,
        chapter=frontmatter.chapter,
        hardware_tier=frontmatter.hardware_tier,
        layer=frontmatter.layer,
        proficiency_level=frontmatter.proficiency_level,
        keywords=frontmatter.keywords,
        chunk_index=chunk_index,
        total_chunks=total_chunks,
        token_count=token_count,
        enrichment_strategy=enrichment_strategy,
    )


def _split_at_boundaries(content: str) -> list[str]:
    """Split content at paragraph and code-block boundaries, keeping code blocks intact."""
    parts: list[str] = []
    code_block_pattern = re.compile(r"(```[\s\S]*?```)", re.MULTILINE)

    last_end = 0
    for match in code_block_pattern.finditer(content):
        before = content[last_end:match.start()].strip()
        if before:
            for para in before.split("\n\n"):
                stripped = para.strip()
                if stripped:
                    parts.append(stripped)
        parts.append(match.group(0))
        last_end = match.end()

    remaining = content[last_end:].strip()
    if remaining:
        for para in remaining.split("\n\n"):
            stripped = para.strip()
            if stripped:
                parts.append(stripped)

    return parts if parts else [content.strip()]
