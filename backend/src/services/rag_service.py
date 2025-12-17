from typing import List, Dict, Any
from ..database.qdrant_client import VectorStoreClient
from ..services.openrouter_client import OpenRouterClient
from ..services.llm_service import LLMService
from ..services.context_service import ContextService
from ..services.cache_service import cache_service
from ..utils.metrics import metrics_collector
from ..database.connection import AsyncSessionLocal
from ..models.chat_models import ChatSession, UserQuery
from sqlalchemy import text
import json
import time


class RAGService:
    def __init__(self):
        self.vector_store = VectorStoreClient()
        self.embedding_client = OpenRouterClient()
        self.llm_service = LLMService()
        self.context_service = ContextService()

    async def retrieve_context_full_book(self, query: str, book_id: str = None, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve context from the full book using vector search
        """
        print(f"DEBUG: Retrieving full book context for query: {query[:30]}...")
        print(f"DEBUG: Book ID: {book_id}, Limit: {limit}")

        # Check cache for existing results
        cached_results = await cache_service.get_qdrant_results(query, book_id or "default")
        if cached_results is not None:
            print(f"DEBUG: Context retrieved from cache: {len(cached_results)} items")
            metrics_collector.record_cache_hit('qdrant_results')
            return cached_results

        print("DEBUG: Context not in cache, generating embeddings...")
        # Generate embedding for the query
        embedding_start_time = time.time()
        query_embedding = await self.embedding_client.get_embeddings([query])
        embedding_duration = time.time() - embedding_start_time
        if not query_embedding or len(query_embedding) == 0:
            print("DEBUG: No embeddings returned")
            metrics_collector.record_embedding_generation(embedding_duration, success=False)
            return []

        metrics_collector.record_embedding_generation(embedding_duration, success=True)
        query_vector = query_embedding[0]
        print(f"DEBUG: Generated embedding vector of length: {len(query_vector)}")

        # Search in vector store
        search_start_time = time.time()
        try:
            print("DEBUG: Searching in vector store...")
            results = self.vector_store.search_similar(
                query_vector=query_vector,
                book_id=book_id,
                limit=limit
            )
            search_duration = time.time() - search_start_time
            print(f"DEBUG: Found {len(results)} results from vector store in {search_duration:.2f}s")
            metrics_collector.record_qdrant_search(search_duration, len(results))
        except Exception as e:
            search_duration = time.time() - search_start_time
            print(f"ERROR: Vector store search failed: {e}")
            metrics_collector.record_qdrant_search(search_duration, 0, success=False)
            raise

        # Cache the results
        await cache_service.set_qdrant_results(query, book_id or "default", results)
        metrics_collector.record_cache_miss('qdrant_results')

        return results

    async def retrieve_context_selected_text(self, selected_text: str, query: str, book_id: str = None) -> List[Dict[str, Any]]:
        """
        Retrieve context based only on the selected text (zero-shot RAG)
        """
        # Create a cache key for selected text queries
        cache_key = f"selected_text:{hash(selected_text + query + (book_id or ''))}"

        # Check cache for existing results
        cached_results = await cache_service.get_qdrant_results(query, cache_key)
        if cached_results is not None:
            metrics_collector.record_cache_hit('qdrant_results_selected_text')
            return cached_results

        # For selected text mode, we focus on the provided text
        # Generate embedding for the combined query and selected text
        combined_text = f"Context: {selected_text}\nQuestion: {query}"
        embedding_start_time = time.time()
        query_embedding = await self.embedding_client.get_embeddings([combined_text])
        embedding_duration = time.time() - embedding_start_time
        if not query_embedding or len(query_embedding) == 0:
            metrics_collector.record_embedding_generation(embedding_duration, success=False)
            return []

        metrics_collector.record_embedding_generation(embedding_duration, success=True)
        query_vector = query_embedding[0]

        # Search in vector store but prioritize chunks related to the selected text context
        search_start_time = time.time()
        try:
            results = self.vector_store.search_similar(
                query_vector=query_vector,
                book_id=book_id,
                limit=3  # Fewer results for selected text mode
            )
            search_duration = time.time() - search_start_time
            metrics_collector.record_qdrant_search(search_duration, len(results))
        except Exception as e:
            search_duration = time.time() - search_start_time
            metrics_collector.record_qdrant_search(search_duration, 0, success=False)
            raise

        # Cache the results
        await cache_service.set_qdrant_results(query, cache_key, results)
        metrics_collector.record_cache_miss('qdrant_results_selected_text')

        return results

    async def get_relevant_context(self, query: str, query_type: str, book_id: str = None, selected_text: str = None) -> List[Dict[str, Any]]:
        """
        Get relevant context based on query type
        """
        if query_type == "selected_text" and selected_text:
            return await self.retrieve_context_selected_text(selected_text, query, book_id)
        else:
            # Default to full book query
            return await self.retrieve_context_full_book(query, book_id)

    async def generate_response(self, query: str, context: List[Dict[str, Any]], chat_history: List[Dict[str, Any]] = None) -> str:
        """
        Generate response using the retrieved context and LLM
        """
        # Format the context for the LLM
        context_str = "\n\n".join([f"Source: {item['chapter_id']}\nContent: {item['content']}" for item in context])

        # Use the LLM service to generate the response
        response = await self.llm_service.generate_response_with_context(
            context=context_str,
            question=query,
            chat_history=chat_history
        )

        return response

    async def generate_response_with_context_awareness(self, query: str, context: List[Dict[str, Any]], chat_history: List[Dict[str, Any]] = None) -> str:
        """
        Generate response with awareness of conversation context
        """
        print(f"DEBUG: Generating response with context awareness")
        print(f"DEBUG: Query length: {len(query)}, Context items: {len(context)}")
        print(f"DEBUG: Chat history items: {len(chat_history) if chat_history else 0}")

        # If no context is found, provide a helpful response
        if not context or len(context) == 0:
            print("WARNING: No context found for the query, returning informative message")
            return "I couldn't find any relevant information in the book to answer your question. Please make sure the book content has been properly ingested into the system."

        llm_start_time = time.time()
        try:
            if chat_history:
                print("DEBUG: Using chat history context")
                # Use context service to get relevant context from chat history
                relevant_history = self.context_service.get_relevant_context(chat_history, query)
                formatted_history = self.context_service.format_context_for_llm(relevant_history)

                # Format the context for the LLM with conversation history
                context_str = "\n\n".join([f"Source: {item['chapter_id']}\nContent: {item['content']}" for item in context])
                print(f"DEBUG: Context string length: {len(context_str)}")

                # Use the LLM service to generate the response with full context
                response = await self.llm_service.generate_response_with_context(
                    context=context_str,
                    question=query,
                    chat_history=relevant_history
                )
            else:
                print("DEBUG: No chat history, using basic generation")
                # No history, use basic generation
                response = await self.generate_response(query, context)

            llm_duration = time.time() - llm_start_time
            print(f"DEBUG: LLM call completed in {llm_duration:.2f}s, response length: {len(response)}")
            metrics_collector.record_llm_call(llm_duration, len(response.split()))
            return response
        except Exception as e:
            llm_duration = time.time() - llm_start_time
            print(f"ERROR: LLM call failed: {e}")
            metrics_collector.record_llm_call(llm_duration, 0, success=False)
            raise