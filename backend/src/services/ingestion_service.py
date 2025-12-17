from typing import List, Dict, Any
from ..database.connection import AsyncSessionLocal
from ..models.chunk import BookContentChunk
from ..database.qdrant_client import VectorStoreClient
from ..services.openrouter_client import OpenRouterClient
from ..services.cache_service import cache_service
from ..utils.metrics import metrics_collector
from sqlalchemy import select
import hashlib
import uuid
import time


class SimpleTextSplitter:
    """Simple text splitter to avoid heavy dependencies"""
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200, separators=None):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.separators = separators or ["\n\n", "\n", " ", ""]

    def split_text(self, text: str) -> List[str]:
        """Split text into chunks"""
        chunks = []
        start = 0

        while start < len(text):
            # Determine the end position
            end = start + self.chunk_size

            # If we're at the end, just take the remaining text
            if end >= len(text):
                chunks.append(text[start:])
                break

            # Try to find a good separator to break at
            chunk_text = text[start:end]
            found_separator = False

            for separator in self.separators:
                # Look for the separator in the current chunk
                last_sep = chunk_text.rfind(separator)
                if last_sep != -1:
                    # Adjust the end position to include the separator
                    actual_end = start + last_sep + len(separator)
                    chunks.append(text[start:actual_end])
                    # Move start position forward by overlap amount
                    start = actual_end - self.chunk_overlap
                    found_separator = True
                    break

            # If no separator found, just take the chunk as is
            if not found_separator:
                chunks.append(text[start:end])
                start = end - self.chunk_overlap

        # Filter out any empty chunks
        chunks = [chunk for chunk in chunks if chunk.strip()]

        return chunks


class IngestionService:
    def __init__(self):
        self.text_splitter = SimpleTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            separators=["\n\n", "\n", " ", ""]
        )
        self.vector_store = VectorStoreClient()
        self.embedding_client = OpenRouterClient()

    def chunk_text(self, text: str, book_id: str, chapter_id: str = None) -> List[Dict[str, Any]]:
        """
        Split text into chunks appropriate for book content
        """
        chunks = self.text_splitter.split_text(text)

        chunk_objects = []
        for chunk_text in chunks:
            # Create a hash of the content to detect changes later
            content_hash = hashlib.sha256(chunk_text.encode()).hexdigest()

            chunk_obj = {
                "id": str(uuid.uuid4()),
                "book_id": book_id,
                "chapter_id": chapter_id,
                "content": chunk_text,
                "hash": content_hash,
                "metadata": {
                    "source_type": "book",
                    "chapter_id": chapter_id,
                    "original_length": len(chunk_text)
                }
            }
            chunk_objects.append(chunk_obj)

        return chunk_objects

    async def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for all chunks, using cache when possible
        """
        # Check cache for existing embeddings
        chunks_with_embeddings = []
        chunks_to_generate = []

        for chunk in chunks:
            cached_embedding = await cache_service.get_embedding(chunk["content"])
            if cached_embedding is not None:
                chunk["embedding"] = cached_embedding
                chunks_with_embeddings.append(chunk)
                metrics_collector.record_cache_hit('embedding')
            else:
                chunks_to_generate.append(chunk)
                metrics_collector.record_cache_miss('embedding')

        # Generate embeddings for chunks not in cache
        if chunks_to_generate:
            texts = [chunk["content"] for chunk in chunks_to_generate]
            embedding_start_time = time.time()
            try:
                embeddings = await self.embedding_client.get_embeddings_batch(texts)
                embedding_duration = time.time() - embedding_start_time

                # Add embeddings to chunk objects and cache them
                for i, chunk in enumerate(chunks_to_generate):
                    chunk["embedding"] = embeddings[i]
                    # Cache the embedding
                    await cache_service.set_embedding(chunk["content"], embeddings[i])
                    metrics_collector.record_embedding_generation(embedding_duration / len(chunks_to_generate), success=True)
                    chunks_with_embeddings.append(chunk)
            except Exception as e:
                embedding_duration = time.time() - embedding_start_time
                metrics_collector.record_embedding_generation(embedding_duration, success=False)
                raise
        else:
            # All chunks were found in cache
            pass

        return chunks_with_embeddings

    async def store_in_postgres(self, chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Store chunk metadata in Neon Postgres
        """
        async with AsyncSessionLocal() as session:
            chunk_ids = []
            for chunk_data in chunks:
                db_chunk = BookContentChunk(
                    id=uuid.UUID(chunk_data["id"]),
                    book_id=chunk_data["book_id"],
                    chapter_id=chunk_data["chapter_id"],
                    content=chunk_data["content"],
                    metadata_=chunk_data["metadata"],
                    hash=chunk_data["hash"]
                )
                session.add(db_chunk)
                chunk_ids.append(str(db_chunk.id))

            await session.commit()
            return chunk_ids

    async def store_in_vector_store(self, chunks: List[Dict[str, Any]]) -> List[str]:
        """
        Store chunk vectors in Qdrant
        """
        return await self.vector_store.store_chunks(chunks)

    async def ingest_book(self, book_id: str, title: str, content: str, metadata: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Main ingestion method that orchestrates the entire process
        """
        start_time = time.time()
        if metadata is None:
            metadata = {}

        try:
            # Step 1: Chunk the text
            print(f"DEBUG: Starting to chunk content for book '{title}', length: {len(content)}")
            chunks = self.chunk_text(content, book_id, metadata.get("chapter_id"))
            print(f"DEBUG: Created {len(chunks)} chunks for book '{title}'")

            # Step 2: Generate embeddings
            print(f"DEBUG: Starting to generate embeddings for {len(chunks)} chunks")
            chunks_with_embeddings = await self.generate_embeddings(chunks)
            print(f"DEBUG: Generated embeddings for {len(chunks_with_embeddings)} chunks")

            # Step 3: Store in Postgres
            print(f"DEBUG: Storing {len(chunks_with_embeddings)} chunks in Postgres")
            postgres_ids = await self.store_in_postgres(chunks_with_embeddings)
            print(f"DEBUG: Stored {len(postgres_ids)} chunks in Postgres")

            # Step 4: Store in Qdrant
            print(f"DEBUG: Storing {len(chunks_with_embeddings)} chunks in Qdrant")
            qdrant_ids = await self.store_in_vector_store(chunks_with_embeddings)
            print(f"DEBUG: Stored {len(qdrant_ids)} chunks in Qdrant")

            total_duration = time.time() - start_time

            # Record metrics
            metrics_collector.record_ingestion(total_duration, len(chunks))
            # Temporarily comment out cache size recording to identify the source of the error
            # metrics_collector.record_cache_size('embedding', cache_service.embedding_cache.cache.size())

            print(f"DEBUG: Successfully ingested book '{title}' with {len(chunks)} chunks, {len(postgres_ids)} postgres IDs, {len(qdrant_ids)} qdrant IDs")
            return {
                "status": "success",
                "chunks_processed": len(chunks),
                "message": f"Book '{title}' successfully ingested and indexed",
                "postgres_ids": postgres_ids,
                "qdrant_ids": qdrant_ids
            }
        except Exception as e:
            import traceback
            total_duration = time.time() - start_time
            metrics_collector.record_ingestion(total_duration, 0, success=False)
            print(f"ERROR: Failed to ingest book '{title}': {str(e)}")
            print(f"Full traceback: {traceback.format_exc()}")
            raise