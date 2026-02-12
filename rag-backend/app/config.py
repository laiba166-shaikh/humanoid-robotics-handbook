from functools import lru_cache

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    allowed_origins: str = "http://localhost:3000"
    collection_name: str = "humanoid-textbook"
    log_level: str = "INFO"
    rerank_enabled: bool = True

    @property
    def origins_list(self) -> list[str]:
        return [o.strip() for o in self.allowed_origins.split(",") if o.strip()]

    model_config = {"env_file": ".env", "env_file_encoding": "utf-8"}


@lru_cache
def get_settings() -> Settings:
    return Settings()
