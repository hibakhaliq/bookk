from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from ..config import settings
import os

# Use SQLite for development if PostgreSQL is not available
DATABASE_URL = settings.DATABASE_URL
if "postgresql" in DATABASE_URL.lower() or "postgres" in DATABASE_URL.lower():
    # Check if we can import asyncpg (for PostgreSQL)
    try:
        import asyncpg
    except ImportError:
        # If asyncpg is not available, use SQLite for development
        DATABASE_URL = "sqlite+aiosqlite:///./rag_chatbot.db"
        print("PostgreSQL driver not available, using SQLite for development")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=True,  # Set to False in production
    pool_pre_ping=True,
    pool_size=5,
    max_overflow=10
)

# Create async session maker
AsyncSessionLocal = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)


async def get_db():
    """Dependency for getting database session"""
    async with AsyncSessionLocal() as session:
        yield session