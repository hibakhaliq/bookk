from typing import List, Dict, Any
from ..services.rag_service import RAGService
from ..services.chat_history_service import ChatHistoryService
from ..services.cache_service import cache_service
from ..utils.metrics import metrics_collector
from ..database.connection import AsyncSession
import time


class MainRAGService:
    def __init__(self):
        self.rag_service = RAGService()
        self.chat_history_service = ChatHistoryService()

    async def process_query(self,
                           db: AsyncSession,
                           message: str,
                           session_token: str,
                           query_type: str = "full_book",
                           selected_text: str = None,
                           book_id: str = None) -> Dict[str, Any]:
        """
        Process a user query through the complete RAG pipeline
        """
        start_time = time.time()

        try:
            print(f"DEBUG: Processing query: {message[:50]}...")
            print(f"DEBUG: Session token: {session_token}")
            print(f"DEBUG: Query type: {query_type}")

            # Get chat history for context (try cache first)
            chat_history = await cache_service.get_chat_history(session_token)
            if chat_history is None:
                print("DEBUG: Chat history not in cache, fetching from DB")
                chat_history = await self.chat_history_service.get_session_history(db, session_token)
                # Cache the history (with a shorter TTL since it gets updated frequently)
                await cache_service.set_chat_history(session_token, chat_history)
                metrics_collector.record_cache_miss('chat_history')
            else:
                print(f"DEBUG: Chat history retrieved from cache, length: {len(chat_history)}")
                metrics_collector.record_cache_hit('chat_history')

            print(f"DEBUG: Retrieved {len(chat_history)} chat history items")

            # Retrieve relevant context based on query type
            context_start = time.time()
            print(f"DEBUG: Getting relevant context for query: {message[:30]}...")
            context = await self.rag_service.get_relevant_context(
                query=message,
                query_type=query_type,
                book_id=book_id,
                selected_text=selected_text
            )
            context_duration = time.time() - context_start
            print(f"DEBUG: Retrieved {len(context)} context items in {context_duration:.2f}s")

            # Generate response using RAG with context awareness
            response_start = time.time()
            print(f"DEBUG: Generating response with {len(context)} context items...")
            response = await self.rag_service.generate_response_with_context_awareness(
                query=message,
                context=context,
                chat_history=chat_history
            )
            response_duration = time.time() - response_start
            print(f"DEBUG: Generated response in {response_duration:.2f}s, length: {len(response)}")

            # Store the user query and response in history
            await self.chat_history_service.add_message(
                db,
                session_token,
                "user",
                message,
                {"query_type": query_type, "selected_text": selected_text}
            )

            await self.chat_history_service.add_message(
                db,
                session_token,
                "assistant",
                response,
                {"sources": [c["id"] for c in context]}
            )

            # Update the cached chat history
            updated_history = await self.chat_history_service.get_session_history(db, session_token)
            await cache_service.set_chat_history(session_token, updated_history)

            # Prepare sources for response
            sources = []
            for item in context:
                sources.append({
                    "chunk_id": item.get("id", ""),
                    "similarity_score": item.get("score", 0.0),
                    "content_preview": item.get("content", "")[:200] + "..." if len(item.get("content", "")) > 200 else item.get("content", ""),
                    "book_id": item.get("book_id", ""),
                    "chapter_id": item.get("chapter_id", "")
                })

            total_duration = time.time() - start_time

            # Record metrics
            metrics_collector.record_qdrant_search(context_duration, len(context))
            metrics_collector.record_llm_call(response_duration, len(response.split()))
            metrics_collector.registry.observe_histogram('total_query_processing_seconds', total_duration)

            return {
                "response": response,
                "session_token": session_token,
                "sources": sources,
                "query_type": query_type
            }
        except Exception as e:
            total_duration = time.time() - start_time
            metrics_collector.registry.observe_histogram('total_query_processing_seconds', total_duration, {'status': 'error'})
            raise

    async def start_new_session(self, db: AsyncSession, user_id: str = None, metadata: Dict[str, Any] = None) -> str:
        """
        Start a new chat session
        """
        session_token = await self.chat_history_service.create_session(db, user_id, metadata)

        # Initialize an empty chat history in cache
        await cache_service.set_chat_history(session_token, [])

        return session_token