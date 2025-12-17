from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from ..models.chunk import BookContentChunk  # We'll reuse this base
from typing import List, Dict, Any
from datetime import datetime
import uuid
import json


class ChatHistoryService:
    def __init__(self):
        pass

    async def create_session(self, db: AsyncSession, user_id: str = None, metadata: Dict[str, Any] = None) -> str:
        """
        Create a new chat session
        """
        from sqlalchemy import text

        session_token = str(uuid.uuid4())

        # Create session in database
        query = text("""
            INSERT INTO chat_sessions (user_id, session_token, metadata)
            VALUES (:user_id, :session_token, :metadata)
            RETURNING id
        """)

        result = await db.execute(query, {
            "user_id": user_id,
            "session_token": session_token,
            "metadata": json.dumps(metadata) if metadata else None
        })

        session_id = result.fetchone()[0]
        await db.commit()

        return session_token

    async def add_message(self, db: AsyncSession, session_token: str, role: str, content: str, metadata: Dict[str, Any] = None):
        """
        Add a message to a chat session
        """
        from sqlalchemy import text

        # First get the session_id from session_token
        session_query = text("SELECT id FROM chat_sessions WHERE session_token = :session_token")
        session_result = await db.execute(session_query, {"session_token": session_token})
        session_row = session_result.fetchone()

        if not session_row:
            raise ValueError(f"Session with token {session_token} not found")

        session_id = session_row[0]

        # Insert the message
        message_query = text("""
            INSERT INTO messages (session_id, role, content, metadata)
            VALUES (:session_id, :role, :content, :metadata)
        """)

        await db.execute(message_query, {
            "session_id": session_id,
            "role": role,
            "content": content,
            "metadata": json.dumps(metadata) if metadata else None
        })

        await db.commit()

    async def get_session_history(self, db: AsyncSession, session_token: str) -> List[Dict[str, Any]]:
        """
        Get chat history for a session
        """
        from sqlalchemy import text

        query = text("""
            SELECT m.role, m.content, m.timestamp, m.metadata
            FROM messages m
            JOIN chat_sessions cs ON m.session_id = cs.id
            WHERE cs.session_token = :session_token
            ORDER BY m.timestamp ASC
        """)

        result = await db.execute(query, {"session_token": session_token})
        rows = result.fetchall()

        history = []
        for row in rows:
            history.append({
                "role": row[0],
                "content": row[1],
                "timestamp": row[2],
                "metadata": json.loads(row[3]) if row[3] else {}
            })

        return history

    async def get_user_queries(self, db: AsyncSession, session_token: str) -> List[Dict[str, Any]]:
        """
        Get all user queries for a session
        """
        from sqlalchemy import text

        query = text("""
            SELECT uq.query_text, uq.query_type, uq.selected_text, uq.timestamp
            FROM user_queries uq
            JOIN chat_sessions cs ON uq.session_id = cs.id
            WHERE cs.session_token = :session_token AND uq.role = 'user'
            ORDER BY uq.timestamp ASC
        """)

        result = await db.execute(query, {"session_token": session_token})
        rows = result.fetchall()

        queries = []
        for row in rows:
            queries.append({
                "query_text": row[0],
                "query_type": row[1],
                "selected_text": row[2],
                "timestamp": row[3]
            })

        return queries