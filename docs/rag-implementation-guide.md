# RAG Implementation Guide: Variable-Length Content Handling

**Version**: 1.0
**Date**: 2026-02-09
**Purpose**: Reference guide for implementing RAG retrieval with variable-length H2 sections

---

## Overview

This guide documents the RAG chunking and retrieval strategy for the Physical AI & Humanoid Robotics Handbook. The content uses variable-length H2 sections optimized for pedagogical quality rather than uniform chunk sizes.

### Content Token Distribution

Based on Module 1 analysis (128 H2 sections):

| Section Type | Token Range | Count | Percentage | Examples |
|--------------|-------------|-------|------------|----------|
| Structural | 79-241 | 48 | 38% | Learning Objectives, Key Takeaways |
| Optimal | 300-500 | 45 | 35% | Concept explanations, theory |
| Code-heavy | 600-950 | 23 | 18% | Code examples with explanations |
| Other | 241-600 | 12 | 9% | Mixed content |

### Constitution Requirements (Updated)

- **Structural sections**: 200-400 tokens
- **Instructional sections**: 300-600 tokens
- **Code-heavy sections**: 400-700 tokens
- **Hard limit**: 800 tokens maximum

---

## Chunking Strategy

### 1. Base Chunking: H2 Section Boundaries

**Rationale**: H2 sections are natural semantic boundaries in the content. Each section is self-contained and covers a single concept.

```python
import re
from typing import List, Dict, Any

def parse_h2_sections(markdown_content: str) -> List[Dict[str, Any]]:
    """
    Parse markdown content into H2 sections.

    Returns:
        List of dicts with 'title', 'content', 'start_line', 'end_line'
    """
    sections = []
    lines = markdown_content.split('\n')
    current_section = None

    for i, line in enumerate(lines):
        if line.startswith('## ') and not line.startswith('###'):
            # Save previous section
            if current_section:
                sections.append(current_section)

            # Start new section
            current_section = {
                'title': line[3:].strip(),
                'content': '',
                'start_line': i,
                'end_line': i
            }
        elif current_section:
            current_section['content'] += line + '\n'
            current_section['end_line'] = i

    # Add last section
    if current_section:
        sections.append(current_section)

    return sections
```

### 2. Token Counting

Use tiktoken (OpenAI's tokenizer) for consistent token counting:

```python
import tiktoken

def count_tokens(text: str, model: str = "cl100k_base") -> int:
    """
    Count tokens using tiktoken.

    Args:
        text: Text to count tokens for
        model: Tokenizer model (cl100k_base for GPT-4, text-embedding-ada-002)

    Returns:
        Token count
    """
    encoding = tiktoken.get_encoding(model)
    return len(encoding.encode(text))
```

### 3. Adaptive Chunking by Section Type

```python
from enum import Enum
from dataclasses import dataclass

class SectionType(Enum):
    STRUCTURAL = "structural"      # Learning Objectives, Key Takeaways, etc.
    INSTRUCTIONAL = "instructional"  # Concept explanations
    CODE_HEAVY = "code_heavy"      # Code examples with explanations
    UNKNOWN = "unknown"

@dataclass
class Chunk:
    content: str
    metadata: Dict[str, Any]
    token_count: int
    section_type: SectionType

def classify_section(title: str, content: str) -> SectionType:
    """
    Classify section type based on title and content.
    """
    title_lower = title.lower()

    # Structural sections
    structural_keywords = [
        'learning objectives', 'key takeaways',
        'check your understanding', 'next steps',
        'prerequisites', 'summary'
    ]
    if any(kw in title_lower for kw in structural_keywords):
        return SectionType.STRUCTURAL

    # Code-heavy sections (contains code blocks)
    if '```' in content:
        code_block_count = content.count('```') // 2
        if code_block_count >= 2:  # Multiple code blocks
            return SectionType.CODE_HEAVY

    # Default to instructional
    return SectionType.INSTRUCTIONAL

def create_chunks(h2_sections: List[Dict[str, Any]],
                  lesson_metadata: Dict[str, Any]) -> List[Chunk]:
    """
    Create chunks from H2 sections with adaptive strategy.

    Args:
        h2_sections: Parsed H2 sections from markdown
        lesson_metadata: Frontmatter metadata (chapter, module, tier, etc.)

    Returns:
        List of Chunk objects ready for embedding
    """
    chunks = []

    for section in h2_sections:
        title = section['title']
        content = section['content']
        token_count = count_tokens(content)
        section_type = classify_section(title, content)

        # Base metadata for all chunks
        base_metadata = {
            **lesson_metadata,
            'section_title': title,
            'section_type': section_type.value,
            'original_token_count': token_count,
        }

        # Strategy 1: Short sections (<200 tokens) - Add context
        if token_count < 200:
            chunks.extend(handle_short_section(
                section, lesson_metadata, base_metadata
            ))

        # Strategy 2: Long sections (>700 tokens) - Split with overlap
        elif token_count > 700:
            chunks.extend(handle_long_section(
                section, base_metadata
            ))

        # Strategy 3: Optimal sections (200-700 tokens) - Use as-is
        else:
            chunks.append(Chunk(
                content=f"## {title}\n\n{content}",
                metadata=base_metadata,
                token_count=token_count,
                section_type=section_type
            ))

    return chunks
```

### 4. Handling Short Sections (<200 tokens)

**Strategy**: Add contextual information from the lesson to make the chunk more retrievable.

```python
def handle_short_section(section: Dict[str, Any],
                         lesson_metadata: Dict[str, Any],
                         base_metadata: Dict[str, Any]) -> List[Chunk]:
    """
    Handle short sections by adding lesson context.

    For structural sections like "Learning Objectives", the section alone
    may not provide enough context for semantic search. We add:
    - Lesson title and description
    - Chapter context
    - Module context
    """
    title = section['title']
    content = section['content']

    # Build enriched content
    enriched_content = f"""
# {lesson_metadata.get('lesson_title', 'Lesson')}

**Module**: {lesson_metadata.get('module', 'Unknown')}
**Chapter**: {lesson_metadata.get('chapter', 'Unknown')}

## {title}

{content}

---
*This section is part of {lesson_metadata.get('lesson_title', 'this lesson')}
in {lesson_metadata.get('chapter', 'this chapter')}.*
""".strip()

    token_count = count_tokens(enriched_content)

    return [Chunk(
        content=enriched_content,
        metadata={
            **base_metadata,
            'enrichment_strategy': 'context_injection',
            'original_token_count': count_tokens(content),
            'enriched_token_count': token_count
        },
        token_count=token_count,
        section_type=classify_section(title, content)
    )]
```

### 5. Handling Long Sections (>700 tokens)

**Strategy**: Split into overlapping sub-chunks while maintaining semantic coherence.

```python
def handle_long_section(section: Dict[str, Any],
                        base_metadata: Dict[str, Any]) -> List[Chunk]:
    """
    Handle long sections by splitting into overlapping sub-chunks.

    For code-heavy sections, we want to:
    - Keep code blocks intact (don't split mid-code)
    - Maintain context with overlapping windows
    - Preserve the section title in each chunk
    """
    title = section['title']
    content = section['content']

    # Target chunk size: 400-500 tokens
    # Overlap: 100 tokens
    TARGET_SIZE = 450
    OVERLAP = 100

    chunks = []

    # Split by paragraphs and code blocks
    segments = split_by_semantic_boundaries(content)

    current_chunk = f"## {title}\n\n"
    current_tokens = count_tokens(current_chunk)
    chunk_index = 0

    for segment in segments:
        segment_tokens = count_tokens(segment)

        # If adding this segment exceeds target, save current chunk
        if current_tokens + segment_tokens > TARGET_SIZE and current_tokens > 200:
            chunks.append(Chunk(
                content=current_chunk.strip(),
                metadata={
                    **base_metadata,
                    'split_strategy': 'overlapping_windows',
                    'chunk_index': chunk_index,
                    'total_chunks': None  # Will be set after loop
                },
                token_count=current_tokens,
                section_type=SectionType.CODE_HEAVY
            ))

            # Start new chunk with overlap
            overlap_content = get_overlap_content(current_chunk, OVERLAP)
            current_chunk = f"## {title} (continued)\n\n{overlap_content}\n\n{segment}"
            current_tokens = count_tokens(current_chunk)
            chunk_index += 1
        else:
            current_chunk += segment + "\n\n"
            current_tokens += segment_tokens

    # Add final chunk
    if current_tokens > 0:
        chunks.append(Chunk(
            content=current_chunk.strip(),
            metadata={
                **base_metadata,
                'split_strategy': 'overlapping_windows',
                'chunk_index': chunk_index,
                'total_chunks': chunk_index + 1
            },
            token_count=current_tokens,
            section_type=SectionType.CODE_HEAVY
        ))

    # Update total_chunks in all chunks
    for chunk in chunks:
        chunk.metadata['total_chunks'] = len(chunks)

    return chunks

def split_by_semantic_boundaries(content: str) -> List[str]:
    """
    Split content by semantic boundaries (paragraphs, code blocks).

    Ensures code blocks are never split in the middle.
    """
    segments = []
    current_segment = ""
    in_code_block = False

    for line in content.split('\n'):
        if line.strip().startswith('```'):
            in_code_block = not in_code_block
            current_segment += line + '\n'
        elif line.strip() == '' and not in_code_block:
            # Empty line - end of paragraph
            if current_segment.strip():
                segments.append(current_segment.strip())
                current_segment = ""
        else:
            current_segment += line + '\n'

    # Add final segment
    if current_segment.strip():
        segments.append(current_segment.strip())

    return segments

def get_overlap_content(chunk: str, overlap_tokens: int) -> str:
    """
    Extract the last N tokens from a chunk for overlap.
    """
    encoding = tiktoken.get_encoding("cl100k_base")
    tokens = encoding.encode(chunk)

    if len(tokens) <= overlap_tokens:
        return chunk

    overlap_token_ids = tokens[-overlap_tokens:]
    return encoding.decode(overlap_token_ids)
```

---

## Metadata Enrichment

### Frontmatter Extraction

```python
import yaml

def extract_frontmatter(markdown_content: str) -> Dict[str, Any]:
    """
    Extract YAML frontmatter from markdown file.

    Returns:
        Dictionary with frontmatter fields
    """
    if not markdown_content.startswith('---'):
        return {}

    # Find end of frontmatter
    end_index = markdown_content.find('---', 3)
    if end_index == -1:
        return {}

    frontmatter_yaml = markdown_content[3:end_index].strip()
    return yaml.safe_load(frontmatter_yaml)

def enrich_metadata(frontmatter: Dict[str, Any]) -> Dict[str, Any]:
    """
    Enrich metadata with additional fields for filtering.
    """
    return {
        # Core identifiers
        'lesson_id': frontmatter.get('id'),
        'lesson_title': frontmatter.get('title'),
        'chapter': frontmatter.get('chapter'),
        'module': frontmatter.get('module'),

        # Filtering fields
        'hardware_tier': frontmatter.get('hardware_tier'),
        'proficiency_level': frontmatter.get('proficiency_level'),
        'layer': frontmatter.get('layer'),
        'duration_minutes': frontmatter.get('duration_minutes'),

        # Search optimization
        'keywords': frontmatter.get('keywords', []),
        'prerequisites': frontmatter.get('prerequisites', []),
        'learning_objectives': frontmatter.get('learning_objectives', []),

        # Content type
        'content_type': 'lesson',
        'source': 'humanoid-robotics-handbook'
    }
```

---

## Embedding and Storage

### Vector Database Schema

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

def setup_qdrant_collection(client: QdrantClient,
                            collection_name: str = "handbook_lessons"):
    """
    Create Qdrant collection with appropriate schema.
    """
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=1536,  # OpenAI text-embedding-ada-002 dimension
            distance=Distance.COSINE
        )
    )

def store_chunks(client: QdrantClient,
                 chunks: List[Chunk],
                 embeddings: List[List[float]],
                 collection_name: str = "handbook_lessons"):
    """
    Store chunks with embeddings in Qdrant.
    """
    points = []

    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        points.append(PointStruct(
            id=i,
            vector=embedding,
            payload={
                'content': chunk.content,
                'token_count': chunk.token_count,
                'section_type': chunk.section_type.value,
                **chunk.metadata
            }
        ))

    client.upsert(
        collection_name=collection_name,
        points=points
    )
```

### Embedding Generation

```python
from openai import OpenAI

def generate_embeddings(chunks: List[Chunk],
                       model: str = "text-embedding-ada-002") -> List[List[float]]:
    """
    Generate embeddings for chunks using OpenAI API.
    """
    client = OpenAI()

    # Batch process for efficiency
    texts = [chunk.content for chunk in chunks]

    response = client.embeddings.create(
        input=texts,
        model=model
    )

    return [item.embedding for item in response.data]
```

---

## Retrieval Strategy

### Hybrid Search: Semantic + Keyword

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

def hybrid_search(client: QdrantClient,
                  query: str,
                  collection_name: str = "handbook_lessons",
                  hardware_tier: int = None,
                  chapter: str = None,
                  top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Perform hybrid search combining semantic and metadata filtering.

    Args:
        query: User's question
        hardware_tier: Filter by hardware tier (1-4)
        chapter: Filter by chapter name
        top_k: Number of results to return

    Returns:
        List of retrieved chunks with scores
    """
    # Generate query embedding
    client_openai = OpenAI()
    query_embedding = client_openai.embeddings.create(
        input=[query],
        model="text-embedding-ada-002"
    ).data[0].embedding

    # Build metadata filters
    filters = []
    if hardware_tier:
        filters.append(FieldCondition(
            key="hardware_tier",
            match=MatchValue(value=hardware_tier)
        ))
    if chapter:
        filters.append(FieldCondition(
            key="chapter",
            match=MatchValue(value=chapter)
        ))

    # Semantic search with filters
    search_result = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        query_filter=Filter(must=filters) if filters else None,
        limit=top_k,
        with_payload=True
    )

    return [
        {
            'content': hit.payload['content'],
            'score': hit.score,
            'metadata': {k: v for k, v in hit.payload.items() if k != 'content'}
        }
        for hit in search_result
    ]
```

### Query Expansion for Short Sections

```python
def expand_query_for_structural_sections(query: str) -> str:
    """
    Expand queries to better match short structural sections.

    Example:
        "What are the learning objectives?"
        -> "What are the learning objectives for this lesson?
            What will I learn? What skills will I gain?"
    """
    structural_keywords = {
        'learning objectives': 'What will I learn? What skills will I gain? What are the goals?',
        'key takeaways': 'What are the main points? What should I remember? Summary?',
        'prerequisites': 'What do I need to know first? Required knowledge? Background?',
        'next steps': 'What comes next? Where do I go from here? Follow-up lessons?'
    }

    query_lower = query.lower()
    for keyword, expansion in structural_keywords.items():
        if keyword in query_lower:
            return f"{query} {expansion}"

    return query
```

### Re-ranking for Long Sections

```python
def rerank_results(query: str,
                   results: List[Dict[str, Any]],
                   top_k: int = 3) -> List[Dict[str, Any]]:
    """
    Re-rank results to prioritize most relevant content.

    For long sections that were split, this helps identify
    which sub-chunk is most relevant to the query.
    """
    from sentence_transformers import CrossEncoder

    # Use cross-encoder for re-ranking
    model = CrossEncoder('cross-encoder/ms-marco-MiniLM-L-6-v2')

    # Score each result
    pairs = [(query, result['content']) for result in results]
    scores = model.predict(pairs)

    # Sort by score
    ranked_results = [
        {**result, 'rerank_score': score}
        for result, score in zip(results, scores)
    ]
    ranked_results.sort(key=lambda x: x['rerank_score'], reverse=True)

    return ranked_results[:top_k]
```

---

## Testing Strategy

### Test Queries by Section Type

```python
TEST_QUERIES = {
    'structural': [
        "What are the learning objectives for ROS 2 nodes?",
        "What are the key takeaways from the Physical AI lesson?",
        "What prerequisites do I need for Chapter 2?",
    ],
    'instructional': [
        "What is Physical AI?",
        "How does DDS middleware work in ROS 2?",
        "Explain the perception-action loop",
    ],
    'code_heavy': [
        "Show me code for a ROS 2 publisher",
        "How do I create a service client in Python?",
        "Give me an example of a URDF file for a humanoid",
    ],
    'cross_lesson': [
        "How do topics differ from services?",
        "What's the difference between Tier 1 and Tier 2 hardware?",
        "Compare Physical AI to digital AI",
    ]
}

def run_retrieval_tests(client: QdrantClient):
    """
    Run test queries and evaluate retrieval quality.
    """
    results = {}

    for category, queries in TEST_QUERIES.items():
        category_results = []

        for query in queries:
            # Expand query if needed
            expanded_query = expand_query_for_structural_sections(query)

            # Retrieve
            retrieved = hybrid_search(client, expanded_query, top_k=5)

            # Re-rank
            reranked = rerank_results(query, retrieved, top_k=3)

            category_results.append({
                'query': query,
                'expanded_query': expanded_query,
                'top_result': reranked[0] if reranked else None,
                'all_results': reranked
            })

        results[category] = category_results

    return results

def evaluate_retrieval_quality(results: Dict[str, List[Dict]]):
    """
    Evaluate retrieval quality metrics.

    Metrics:
    - Precision@1: Is the top result relevant?
    - Coverage: Are all section types retrievable?
    - Latency: How fast is retrieval?
    """
    print("=== Retrieval Quality Report ===\n")

    for category, queries in results.items():
        print(f"Category: {category}")
        print(f"Queries tested: {len(queries)}")

        for query_result in queries:
            print(f"\nQuery: {query_result['query']}")
            top = query_result['top_result']
            if top:
                print(f"  Top result: {top['metadata']['section_title']}")
                print(f"  Score: {top['score']:.3f}")
                print(f"  Section type: {top['metadata']['section_type']}")
            else:
                print("  No results found!")

        print("\n" + "="*50 + "\n")
```

---

## Production Deployment Checklist

### 1. Ingestion Pipeline

- [ ] Parse all lesson markdown files
- [ ] Extract frontmatter metadata
- [ ] Chunk content using adaptive strategy
- [ ] Generate embeddings (batch process for efficiency)
- [ ] Store in Qdrant with metadata
- [ ] Verify all 12 Module 1 lessons ingested

### 2. API Endpoints

```python
from fastapi import FastAPI, Query
from pydantic import BaseModel

app = FastAPI()

class SearchRequest(BaseModel):
    query: str
    hardware_tier: int = None
    chapter: str = None
    top_k: int = 3

class SearchResponse(BaseModel):
    results: List[Dict[str, Any]]
    query_expansion: str = None

@app.post("/search", response_model=SearchResponse)
async def search_handbook(request: SearchRequest):
    """
    Search the handbook with RAG retrieval.
    """
    # Expand query if needed
    expanded_query = expand_query_for_structural_sections(request.query)

    # Retrieve
    results = hybrid_search(
        client=qdrant_client,
        query=expanded_query,
        hardware_tier=request.hardware_tier,
        chapter=request.chapter,
        top_k=request.top_k * 2  # Retrieve more for re-ranking
    )

    # Re-rank
    reranked = rerank_results(request.query, results, top_k=request.top_k)

    return SearchResponse(
        results=reranked,
        query_expansion=expanded_query if expanded_query != request.query else None
    )
```

### 3. Monitoring

```python
import logging
from datetime import datetime

def log_search_query(query: str, results: List[Dict], user_feedback: str = None):
    """
    Log search queries for monitoring and improvement.
    """
    log_entry = {
        'timestamp': datetime.utcnow().isoformat(),
        'query': query,
        'num_results': len(results),
        'top_result_score': results[0]['score'] if results else 0,
        'user_feedback': user_feedback
    }

    logging.info(f"Search query: {log_entry}")

    # Store in database for analytics
    # TODO: Implement analytics storage
```

---

## Performance Optimization

### Caching Strategy

```python
from functools import lru_cache
import hashlib

@lru_cache(maxsize=1000)
def cached_embedding(text: str) -> List[float]:
    """
    Cache embeddings for frequently queried text.
    """
    client = OpenAI()
    response = client.embeddings.create(
        input=[text],
        model="text-embedding-ada-002"
    )
    return response.data[0].embedding

def get_query_cache_key(query: str, filters: Dict) -> str:
    """
    Generate cache key for query + filters.
    """
    cache_input = f"{query}:{sorted(filters.items())}"
    return hashlib.md5(cache_input.encode()).hexdigest()
```

### Batch Processing

```python
def ingest_all_lessons(lesson_dir: str, batch_size: int = 10):
    """
    Ingest all lessons in batches for efficiency.
    """
    import glob

    lesson_files = glob.glob(f"{lesson_dir}/**/*.md", recursive=True)

    for i in range(0, len(lesson_files), batch_size):
        batch = lesson_files[i:i+batch_size]

        # Process batch
        all_chunks = []
        for lesson_file in batch:
            with open(lesson_file, 'r', encoding='utf-8') as f:
                content = f.read()

            frontmatter = extract_frontmatter(content)
            metadata = enrich_metadata(frontmatter)
            h2_sections = parse_h2_sections(content)
            chunks = create_chunks(h2_sections, metadata)
            all_chunks.extend(chunks)

        # Generate embeddings for batch
        embeddings = generate_embeddings(all_chunks)

        # Store batch
        store_chunks(qdrant_client, all_chunks, embeddings)

        print(f"Processed batch {i//batch_size + 1}: {len(all_chunks)} chunks")
```

---

## Troubleshooting

### Issue: Low retrieval scores for structural sections

**Solution**: Ensure query expansion is enabled and context injection is working.

```python
# Debug: Check if short sections have context
def debug_short_sections(client: QdrantClient):
    results = client.scroll(
        collection_name="handbook_lessons",
        scroll_filter=Filter(
            must=[FieldCondition(
                key="section_type",
                match=MatchValue(value="structural")
            )]
        ),
        limit=10
    )

    for point in results[0]:
        print(f"Section: {point.payload['section_title']}")
        print(f"Token count: {point.payload['token_count']}")
        print(f"Has enrichment: {'enrichment_strategy' in point.payload}")
        print("---")
```

### Issue: Code examples not retrieving well

**Solution**: Verify code blocks are preserved intact and not split mid-code.

```python
# Debug: Check code block integrity
def verify_code_blocks(chunks: List[Chunk]):
    for chunk in chunks:
        if '```' in chunk.content:
            code_block_count = chunk.content.count('```')
            if code_block_count % 2 != 0:
                print(f"WARNING: Incomplete code block in chunk")
                print(f"Section: {chunk.metadata['section_title']}")
```

---

## Next Steps

1. **Implement ingestion pipeline** using the chunking strategy above
2. **Set up Qdrant** with the schema defined
3. **Run test queries** to validate retrieval quality
4. **Monitor performance** and adjust parameters as needed
5. **Collect user feedback** to improve retrieval over time

---

**References**:
- Constitution: `.specify/memory/constitution.md`
- Module 1 Lessons: `humanoid-textbook/docs/module-1-ros2/`
- H2 Token Analysis: `specs/002-module-1-lessons/h2-token-remediation-plan.md`
