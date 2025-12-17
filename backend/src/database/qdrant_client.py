from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
from uuid import uuid4
from ..config import settings
import logging

logger = logging.getLogger(__name__)


class VectorStoreClient:
    def __init__(self):
        self.collection_name = "book_content_chunks"

        # Try to connect to the configured Qdrant instance
        try:
            # Check if we're using a cloud instance (contains ".cloud.qdrant.io")
            if "cloud.qdrant.io" in settings.QDRANT_URL:
                # For cloud instances, we don't need the port in the URL
                url = settings.QDRANT_URL
                self.client = QdrantClient(
                    url=url,
                    api_key=settings.QDRANT_API_KEY,
                    https=True,  # Cloud instances use HTTPS
                    timeout=5.0  # Add timeout to prevent hanging
                )
            else:
                # For local instances or other URLs
                self.client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY,
                    prefer_grpc=False,
                    timeout=5.0  # Add timeout to prevent hanging
                )

            # Test connection and create collection
            self._create_collection_if_not_exists()
            logger.info(f"Successfully connected to Qdrant at {settings.QDRANT_URL}")

        except Exception as e:
            logger.warning(f"Could not connect to Qdrant at {settings.QDRANT_URL}: {e}")
            logger.info("Falling back to local Qdrant instance for development")

            # Fallback to local instance
            try:
                self.client = QdrantClient(
                    host="localhost",
                    port=6333,
                    timeout=5.0
                )
                # Test connection to local instance
                self.client.get_collections()
                self._create_collection_if_not_exists()
                logger.info("Successfully connected to local Qdrant instance")
            except Exception as local_e:
                logger.warning(f"Could not connect to local Qdrant either: {local_e}")
                # As a last resort, try in-memory mode for development
                self.client = QdrantClient(":memory:")
                self._create_collection_if_not_exists()
                logger.info("Using in-memory Qdrant for development")

    def _create_collection_if_not_exists(self):
        """
        Create the collection if it doesn't exist
        """
        try:
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Assuming Qwen embedding dimension
                    distance=models.Distance.COSINE
                )
            )

            # Create payload index for book_id
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="book_id",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

    async def store_chunks(self, chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Store chunks with their embeddings in Qdrant
        """
        import asyncio
        from concurrent.futures import ThreadPoolExecutor

        points = []
        chunk_ids = []

        for chunk in chunks:
            # Use the existing chunk ID instead of generating a new one
            chunk_id = chunk['id']
            chunk_ids.append(chunk_id)

            point = models.PointStruct(
                id=chunk_id,
                vector=chunk['embedding'],
                payload={
                    "book_id": chunk['book_id'],
                    "chapter_id": chunk.get('chapter_id', ''),
                    "chunk_hash": chunk.get('hash', ''),
                    "content": chunk['content'],
                    "metadata": chunk.get('metadata', {})
                }
            )
            points.append(point)

        # Run the synchronous upsert operation in a thread pool to avoid blocking the event loop
        def sync_upsert():
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                # Return the chunk IDs after successful upsert
                return chunk_ids
            except Exception as e:
                print(f"ERROR in Qdrant upsert: {e}")
                raise

        loop = asyncio.get_event_loop()
        with ThreadPoolExecutor() as executor:
            # Execute the sync operation in the thread pool and return the result
            result = await loop.run_in_executor(executor, sync_upsert)
            return result

    def search_similar(self, query_embedding: List[float], book_id: str = None, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in the vector store
        """
        print(f"DEBUG: Searching for similar chunks, query embedding length: {len(query_embedding) if query_embedding else 0}")
        print(f"DEBUG: Book ID: {book_id}, Limit: {limit}")

        conditions = []
        if book_id:
            conditions.append(models.FieldCondition(
                key="book_id",
                match=models.MatchValue(value=book_id)
            ))

        search_filter = None
        if conditions:
            search_filter = models.Filter(
                must=conditions
            )

        print(f"DEBUG: About to perform Qdrant search with filter: {search_filter}")

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=limit
        )

        print(f"DEBUG: Qdrant search returned {len(results)} results")

        result_list = [
            {
                "id": result.id,
                "content": result.payload.get("content", ""),
                "book_id": result.payload.get("book_id", ""),
                "chapter_id": result.payload.get("chapter_id", ""),
                "score": result.score,
                "metadata": result.payload.get("metadata", {})
            }
            for result in results
        ]

        print(f"DEBUG: Converted {len(result_list)} results to dict format")
        return result_list