from sqlalchemy import Column, String, Text, DateTime, UUID, JSON, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from sqlalchemy import create_engine
from sqlalchemy.orm import relationship
import uuid


Base = declarative_base()


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=True)
    session_token = Column(String, nullable=False, unique=True)
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now())
    metadata_ = Column("metadata", JSON, nullable=True)

    # Relationship to messages
    messages = relationship("Message", back_populates="session", cascade="all, delete-orphan")
    user_queries = relationship("UserQuery", back_populates="session", cascade="all, delete-orphan")


class Message(Base):
    __tablename__ = "messages"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.id", ondelete="CASCADE"), nullable=False)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, server_default=func.now())
    metadata_ = Column("metadata", JSON, nullable=True)

    # Relationship
    session = relationship("ChatSession", back_populates="messages")

    def __repr__(self):
        return f"<Message(id={self.id}, role={self.role}, content={self.content[:50]}...)>"


class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_sessions.id", ondelete="CASCADE"), nullable=False)
    query_text = Column(Text, nullable=False)
    query_type = Column(String(20), nullable=False)  # 'full_book' or 'selected_text'
    selected_text = Column(Text, nullable=True)
    timestamp = Column(DateTime, server_default=func.now())

    # Relationship
    session = relationship("ChatSession", back_populates="user_queries")

    def __repr__(self):
        return f"<UserQuery(id={self.id}, query_type={self.query_type}, query={self.query_text[:50]}...)>"