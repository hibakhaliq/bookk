from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Database settings
    DATABASE_URL: str = "postgresql+asyncpg://user:password@localhost/dbname"

    # Qdrant settings
    QDRANT_URL: str = "http://localhost:6333"  # Use local Qdrant for development
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "docusaurus-book"

    # OpenRouter settings
    OPENROUTER_API_KEY: str = ""
    OPENROUTER_MODEL: str = "google/gemini-pro"
    EMBEDDING_MODEL: str = "nomic-ai/nomic-embed-text-v1.5"

    # Qwen settings
    QWEN_EMBEDDING_API_KEY: str = ""
    QWEN_EMBEDDING_BASE_URL: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"

    # Gemini settings
    GEMINI_API_KEY: str = ""
    GEMINI_MODEL: str = "gemini-1.5-flash"

    # Application settings
    APP_NAME: str = "RAG Chatbot API"
    API_V1_STR: str = "/api/v1"

    class Config:
        env_file = ".env"
        extra = "ignore"  # Allow extra fields in .env file


settings = Settings()