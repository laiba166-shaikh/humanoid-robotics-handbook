# Data Model: RAG Backend

**Feature**: 003-rag-backend | **Date**: 2026-02-11

## Entities

### 1. Chunk (Qdrant Point)

The atomic unit stored in the vector database. Each point represents one H2 section (or sub-chunk for split sections).

| Field | Type | Storage | Description |
|-------|------|---------|-------------|
| `id` | `str` (UUID5) | Qdrant point ID | Deterministic: `uuid5(NAMESPACE_DNS, f"{lesson_id}:{section_title}:{chunk_index}")` |
| `vector` | `list[float]` (1024) | Qdrant vector | Cohere `embed-english-v3.0` embedding |
| `content` | `str` | Payload | Full text of the chunk (enriched for structural, original for others) |
| `lesson_id` | `str` | Payload (indexed) | YAML frontmatter `id` field, e.g. `"lesson-1-foundations"` |
| `lesson_title` | `str` | Payload | Full lesson title from frontmatter |
| `section_title` | `str` | Payload | H2 header text |
| `section_type` | `str` | Payload (indexed) | `"structural"` \| `"instructional"` \| `"code_heavy"` |
| `module` | `str` | Payload (indexed) | Module name from frontmatter |
| `chapter` | `str` | Payload (indexed) | Chapter name from frontmatter |
| `hardware_tier` | `int` | Payload (indexed) | 1-4, from frontmatter |
| `layer` | `str` | Payload | `"L1"` \| `"L2"` \| `"L3"` \| `"L4"` |
| `proficiency_level` | `str` | Payload | `"A2"` \| `"B1"` \| `"C2"` |
| `keywords` | `list[str]` | Payload | From frontmatter |
| `chunk_index` | `int` | Payload | 0 for unsplit chunks; 0, 1, 2... for split sections |
| `total_chunks` | `int` | Payload | 1 for unsplit; N for split sections |
| `token_count` | `int` | Payload | Token count of content (tiktoken cl100k_base) |
| `enrichment_strategy` | `str \| None` | Payload | `"context_injection"` for short sections, `"overlapping_windows"` for split, `None` for optimal |

**Identity & Uniqueness**: Point ID is deterministic from `lesson_id + section_title + chunk_index`. Re-ingesting the same content produces the same IDs → Qdrant upsert overwrites in place.

**Indexed Payload Fields** (for filtering): `lesson_id`, `section_type`, `module`, `chapter`, `hardware_tier`

### 2. LessonFrontmatter (Parsed from YAML)

Extracted during ingestion. Not stored separately — embedded into each chunk's payload.

| Field | Type | Required | Source |
|-------|------|----------|--------|
| `id` | `str` | Yes | `id` |
| `title` | `str` | Yes | `title` |
| `sidebar_position` | `int` | Yes | `sidebar_position` |
| `sidebar_label` | `str` | Yes | `sidebar_label` |
| `description` | `str` | Yes | `description` |
| `duration_minutes` | `int` | Yes | `duration_minutes` |
| `proficiency_level` | `str` | Yes | `proficiency_level` |
| `layer` | `str` | Yes | `layer` |
| `hardware_tier` | `int` | Yes | `hardware_tier` |
| `tier_1_path` | `str` | Yes | `tier_1_path` |
| `learning_objectives` | `list[str]` | Yes | `learning_objectives` |
| `keywords` | `list[str]` | Yes | `keywords` |
| `prerequisites` | `list[str]` | Yes | `prerequisites` |
| `chapter` | `str` | Yes | `chapter` |
| `module` | `str` | Yes | `module` |

**Validation**: All 15 fields required. Files missing any field are skipped with a warning logged.

### 3. SectionType (Enum)

Classification of H2 sections that determines chunking strategy.

| Value | Title Keywords | Content Signal | Token Range | Strategy |
|-------|---------------|----------------|-------------|----------|
| `structural` | Learning Objectives, Key Takeaways, Check Your Understanding, Next Steps, Prerequisites, Summary | — | 79-241 | Enrich with context |
| `instructional` | (default) | — | 200-700 | Preserve as-is |
| `code_heavy` | — | 2+ code blocks (``` pairs) | 400-950 | Split if >700 tokens |

**Classification priority**: Title match first → code block count → default to instructional.

### 4. IngestionReport (Returned by CLI pipeline)

| Field | Type | Description |
|-------|------|-------------|
| `total_files` | `int` | Files discovered |
| `processed_files` | `int` | Files successfully ingested |
| `skipped_files` | `list[SkippedFile]` | Files skipped with reasons (see SkippedFile below) |
| `total_chunks` | `int` | Total chunks created |
| `chunks_by_type` | `dict[str, int]` | Count per section type |
| `enriched_count` | `int` | Short sections that were enriched |
| `split_count` | `int` | Long sections that were split |
| `errors` | `list[str]` | Error messages |
| `duration_seconds` | `float` | Pipeline run time |
| `embed_api_calls` | `int` | Number of Cohere embed API calls used |

**SkippedFile** (inline value object):

| Field | Type | Description |
|-------|------|-------------|
| `file_path` | `str` | Absolute or relative path to the skipped file |
| `reason` | `str` | Why it was skipped (e.g., "missing frontmatter field: hardware_tier", "no content after frontmatter") |

### 5. SearchResult (API Response Item)

| Field | Type | Description |
|-------|------|-------------|
| `content` | `str` | Chunk text (truncated for preview) |
| `score` | `float` | Cosine similarity score (0-1) |
| `lesson_id` | `str` | Source lesson identifier |
| `lesson_title` | `str` | Human-readable lesson title |
| `section_title` | `str` | H2 section heading |
| `section_type` | `str` | structural/instructional/code_heavy |
| `module` | `str` | Module name |
| `chapter` | `str` | Chapter name |
| `hardware_tier` | `int` | Content hardware tier |

### 6. ChatSource (Citation in Chat Response)

| Field | Type | Description |
|-------|------|-------------|
| `lesson_id` | `str` | Source lesson |
| `lesson_title` | `str` | Human-readable title |
| `section_title` | `str` | Specific section cited |
| `module` | `str` | Module for citation format |
| `chapter` | `str` | Chapter for citation format |
| `citation_label` | `str` | Formatted: `[Module 1, Chapter 2, Lesson 3]` |

### 7. IngestionManifest (Local JSON File)

Tracks which files have been ingested and their content hashes to support selective ingestion (FR-020). Stored at `rag-backend/.ingestion-manifest.json`. Gitignored (environment-specific).

| Field | Type | Description |
|-------|------|-------------|
| `files` | `dict[str, FileEntry]` | Map of relative file path to ingestion entry |
| `last_run` | `str` (ISO datetime) | Timestamp of the most recent pipeline run |

**FileEntry** (inline value object):

| Field | Type | Description |
|-------|------|-------------|
| `content_hash` | `str` | SHA-256 hex digest of file content |
| `last_ingested_at` | `str` (ISO datetime) | When this file was last ingested |
| `chunk_count` | `int` | Number of chunks produced from this file |

**Selective ingestion logic**: When `selective=True`, the pipeline computes SHA-256 of each `.md` file and compares against the manifest. Files with matching hashes are skipped. After successful ingestion, the manifest is updated. When `selective=False` (default), all files are processed and the manifest is rebuilt from scratch.

## Relationships

```
LessonFile (.md)
  └── has 15-field YAML frontmatter → LessonFrontmatter
  └── contains N H2 sections
        └── each classified as → SectionType
        └── each produces 1+ → Chunk (in Qdrant)
              └── carries LessonFrontmatter fields as payload
              └── referenced by → SearchResult (on query)
              └── referenced by → ChatSource (on chat)

IngestionReport
  └── summarizes one pipeline run across M files → N chunks
  └── contains list[SkippedFile] for skipped files

IngestionManifest (.ingestion-manifest.json)
  └── maps file_path → FileEntry (content_hash, last_ingested_at, chunk_count)
  └── used by pipeline when selective=True to skip unchanged files
```

## State Transitions

### Chunk Lifecycle

```
[File on Disk] → parse → [Sections Extracted] → classify → [Typed Sections]
    → chunk (enrich/preserve/split) → [Chunks] → embed → [Vectors]
    → upsert to Qdrant → [Stored Points]

Re-ingestion:
    [Updated File] → delete old points by lesson_id → re-parse → re-embed → upsert
```

### Collection Lifecycle

```
[Empty] → create_collection(1024, cosine) → [Ready]
    → ingest Module 1 → [~130-150 points]
    → ingest Module 2-4 → [~500-800 points]
    → re-ingest updated lesson → [same count, updated content]
```
