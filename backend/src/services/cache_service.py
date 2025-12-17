"""
Cache service for frequently accessed embeddings and other data
"""
import asyncio
import time
from typing import Any, Optional, Dict
from functools import wraps


class LRUCache:
    """
    Simple LRU Cache implementation for storing embeddings and other frequently accessed data
    """
    def __init__(self, max_size: int = 1000, ttl: int = 3600):
        self.max_size = max_size
        self.ttl = ttl  # Time to live in seconds
        self.cache: Dict[str, tuple] = {}  # key -> (value, timestamp)
        self.access_order = {}  # key -> access timestamp
        self._lock = asyncio.Lock()

    def _is_expired(self, timestamp: float) -> bool:
        return time.time() - timestamp > self.ttl

    def _cleanup_expired(self):
        """Remove expired entries from cache"""
        current_time = time.time()
        expired_keys = [
            key for key, (_, timestamp) in self.cache.items()
            if current_time - timestamp > self.ttl
        ]
        for key in expired_keys:
            self.cache.pop(key, None)
            self.access_order.pop(key, None)

    async def get(self, key: str) -> Optional[Any]:
        """Get value from cache, return None if not found or expired"""
        async with self._lock:
            self._cleanup_expired()

            if key not in self.cache:
                return None

            value, timestamp = self.cache[key]
            if self._is_expired(timestamp):
                del self.cache[key]
                del self.access_order[key]
                return None

            # Update access time
            self.access_order[key] = time.time()
            return value

    async def set(self, key: str, value: Any):
        """Set value in cache, evict oldest if max_size exceeded"""
        async with self._lock:
            current_time = time.time()
            self._cleanup_expired()

            # If key already exists, just update it
            if key in self.cache:
                self.cache[key] = (value, current_time)
                self.access_order[key] = current_time
                return

            # Check if we need to evict
            if len(self.cache) >= self.max_size:
                # Find the least recently accessed item
                oldest_key = min(self.access_order.keys(),
                               key=lambda k: self.access_order[k])
                del self.cache[oldest_key]
                del self.access_order[oldest_key]

            # Add new item
            self.cache[key] = (value, current_time)
            self.access_order[key] = current_time

    async def delete(self, key: str):
        """Delete key from cache"""
        async with self._lock:
            self.cache.pop(key, None)
            self.access_order.pop(key, None)

    async def clear(self):
        """Clear all cache"""
        async with self._lock:
            self.cache.clear()
            self.access_order.clear()

    def size(self) -> int:
        """Get current cache size"""
        return len(self.cache)


class EmbeddingCache:
    """
    Specialized cache for embeddings with specific methods for embedding operations
    """
    def __init__(self, max_size: int = 1000, ttl: int = 7200):  # 2 hours TTL for embeddings
        self.cache = LRUCache(max_size=max_size, ttl=ttl)

    async def get_embedding(self, text: str) -> Optional[list]:
        """Get embedding from cache"""
        key = f"embedding:{hash(text)}"
        return await self.cache.get(key)

    async def set_embedding(self, text: str, embedding: list):
        """Set embedding in cache"""
        key = f"embedding:{hash(text)}"
        await self.cache.set(key, embedding)

    async def get_chunk_context(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """Get chunk context from cache"""
        key = f"chunk_context:{chunk_id}"
        return await self.cache.get(key)

    async def set_chunk_context(self, chunk_id: str, context: Dict[str, Any]):
        """Set chunk context in cache"""
        key = f"chunk_context:{chunk_id}"
        await self.cache.set(key, context)


class CacheService:
    """
    Main cache service that manages different types of cached data
    """
    def __init__(self):
        # Separate caches for different data types
        self.embedding_cache = EmbeddingCache()
        self.qdrant_results_cache = LRUCache(max_size=500, ttl=1800)  # 30 min TTL
        self.chat_history_cache = LRUCache(max_size=200, ttl=3600)    # 1 hour TTL
        self.book_metadata_cache = LRUCache(max_size=100, ttl=3600)   # 1 hour TTL

    async def get_embedding(self, text: str) -> Optional[list]:
        """Get embedding from cache"""
        return await self.embedding_cache.get_embedding(text)

    async def set_embedding(self, text: str, embedding: list):
        """Set embedding in cache"""
        await self.embedding_cache.set_embedding(text, embedding)

    async def get_qdrant_results(self, query: str, book_id: str) -> Optional[list]:
        """Get Qdrant search results from cache"""
        key = f"qdrant:{book_id}:{hash(query)}"
        return await self.qdrant_results_cache.get(key)

    async def set_qdrant_results(self, query: str, book_id: str, results: list):
        """Set Qdrant search results in cache"""
        key = f"qdrant:{book_id}:{hash(query)}"
        await self.qdrant_results_cache.set(key, results)

    async def get_chat_history(self, session_token: str) -> Optional[list]:
        """Get chat history from cache"""
        return await self.chat_history_cache.get(session_token)

    async def set_chat_history(self, session_token: str, history: list):
        """Set chat history in cache"""
        await self.chat_history_cache.set(session_token, history)

    async def get_book_metadata(self, book_id: str) -> Optional[Dict[str, Any]]:
        """Get book metadata from cache"""
        return await self.book_metadata_cache.get(book_id)

    async def set_book_metadata(self, book_id: str, metadata: Dict[str, Any]):
        """Set book metadata in cache"""
        await self.book_metadata_cache.set(book_id, metadata)

    async def clear_session_cache(self, session_token: str):
        """Clear cache entries related to a specific session"""
        await self.chat_history_cache.delete(session_token)

    async def clear_book_cache(self, book_id: str):
        """Clear cache entries related to a specific book"""
        # Remove book metadata
        await self.book_metadata_cache.delete(book_id)

        # Remove related qdrant results (this is a simplification;
        # in practice, you'd need to iterate and find related keys)
        # For now, we'll just clear the entire qdrant cache
        await self.qdrant_results_cache.clear()


# Global cache service instance
cache_service = CacheService()


def cache_result(ttl: int = 300):
    """
    Decorator to cache function results
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # Create cache key from function name and arguments
            cache_key = f"{func.__name__}:{hash(str(args) + str(kwargs))}"

            # Try to get from cache first
            cached_result = await cache_service.cache.get(cache_key)
            if cached_result is not None:
                return cached_result

            # Execute function and cache result
            result = await func(*args, **kwargs)
            await cache_service.cache.set(cache_key, result)

            return result
        return wrapper
    return decorator


# Initialize the cache service
async def init_cache_service():
    """
    Initialize the cache service (if needed)
    """
    # Currently no initialization needed, but kept for future use
    pass