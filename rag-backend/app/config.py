from functools import lru_cache

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # Cohere & Qdrant
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str

    # Database
    database_url: str

    # Authentication - JWT
    jwt_secret_key: str
    jwt_algorithm: str = "HS256"
    access_token_expire_minutes: int = 15
    refresh_token_expire_days: int = 7

    # Authentication - Google OAuth 2.1
    google_client_id: str
    google_client_secret: str
    google_redirect_uri: str

    # OpenAI (for Agents SDK orchestration)
    gemini_api_key: str = ""
    openai_api_key: str = ""
    chatbot_base_url: str = "https://generativelanguage.googleapis.com/v1beta/openai/"

    # Frontend URL (for OAuth redirects and CORS)
    frontend_url: str

    # Application
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
