from pydantic import BaseModel, Field


class SearchRequest(BaseModel):
    query: str = Field(min_length=1, max_length=500)
    hardware_tier: int = Field(default=4, ge=1, le=4)
    top_k: int = Field(default=5, ge=1, le=20)


class ChatRequest(BaseModel):
    query: str = Field(min_length=1, max_length=500)
    hardware_tier: int = Field(default=4, ge=1, le=4)
    top_k: int = Field(default=5, ge=1, le=10)
