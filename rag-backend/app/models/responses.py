from enum import Enum
from typing import Generic, TypeVar

from pydantic import BaseModel, Field

T = TypeVar("T")


class ResponseMeta(BaseModel):
    latency_ms: float | None = None
    source_count: int | None = None


class ApiResponse(BaseModel, Generic[T]):
    success: bool
    data: T | None = None
    error: str | None = None
    meta: ResponseMeta = Field(default_factory=ResponseMeta)


class ErrorCode(str, Enum):
    VALIDATION_ERROR = "VALIDATION_ERROR"
    NOT_FOUND = "NOT_FOUND"
    RATE_LIMITED = "RATE_LIMITED"
    SERVICE_UNAVAILABLE = "SERVICE_UNAVAILABLE"
    INTERNAL_ERROR = "INTERNAL_ERROR"


class ApiErrorResponse(BaseModel):
    success: bool = False
    error: str
    error_code: ErrorCode | None = None
    data: None = None
    meta: ResponseMeta = Field(default_factory=ResponseMeta)


class SearchResultItem(BaseModel):
    content: str
    score: float
    lesson_id: str
    lesson_title: str
    section_title: str
    section_type: str
    module: str
    chapter: str
    hardware_tier: int


class SearchResponse(BaseModel):
    query: str
    query_expanded: bool = False
    results: list[SearchResultItem] = Field(default_factory=list)


class ChatSource(BaseModel):
    lesson_id: str
    lesson_title: str
    section_title: str
    module: str
    chapter: str
    citation_label: str


class ChatResponse(BaseModel):
    answer: str
    sources: list[ChatSource] = Field(default_factory=list)


class ComponentHealth(BaseModel):
    status: str  # "healthy" | "unhealthy"
    latency_ms: float | None = None
    error: str | None = None


class HealthResponse(BaseModel):
    status: str  # "healthy" | "degraded" | "unhealthy"
    components: dict[str, ComponentHealth] = Field(default_factory=dict)
