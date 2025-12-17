from sqlalchemy import Column, String, Text, DateTime, UUID, JSON
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid


Base = declarative_base()


class BookContentChunk(Base):
    __tablename__ = "book_content_chunks"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    book_id = Column(String, nullable=False)
    chapter_id = Column(String, nullable=True)
    content = Column(Text, nullable=False)
    metadata_ = Column("metadata", JSON, nullable=True)  # Using metadata_ to avoid conflict with metadata method
    hash = Column(String, nullable=False)
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now())

    def __repr__(self):
        return f"<BookContentChunk(id={self.id}, book_id={self.book_id}, chapter_id={self.chapter_id})>"