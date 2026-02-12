from enum import Enum

from pydantic import BaseModel, Field


class SectionType(str, Enum):
    structural = "structural"
    instructional = "instructional"
    code_heavy = "code_heavy"


class LessonFrontmatter(BaseModel):
    id: str
    title: str
    sidebar_position: int
    sidebar_label: str
    description: str
    duration_minutes: int
    proficiency_level: str
    layer: str
    hardware_tier: int = Field(ge=1, le=4)
    tier_1_path: str
    learning_objectives: list[str]
    keywords: list[str]
    prerequisites: list[str]
    chapter: str
    module: str


class Chunk(BaseModel):
    content: str
    lesson_id: str
    lesson_title: str
    section_title: str
    section_type: SectionType
    module: str
    chapter: str
    hardware_tier: int = Field(ge=1, le=4)
    layer: str
    proficiency_level: str
    keywords: list[str]
    chunk_index: int = 0
    total_chunks: int = 1
    token_count: int = 0
    enrichment_strategy: str | None = None


class SkippedFile(BaseModel):
    file_path: str
    reason: str


class IngestionReport(BaseModel):
    total_files: int = 0
    processed_files: int = 0
    skipped_files: list[SkippedFile] = Field(default_factory=list)
    total_chunks: int = 0
    chunks_by_type: dict[str, int] = Field(default_factory=dict)
    enriched_count: int = 0
    split_count: int = 0
    errors: list[str] = Field(default_factory=list)
    duration_seconds: float = 0.0
    embed_api_calls: int = 0
