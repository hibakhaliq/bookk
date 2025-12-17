import httpx
import asyncio
from typing import List
from ..config import settings


class OpenRouterClient:
    def __init__(self):
        # Check if Qwen embedding settings are available, otherwise fall back to OpenRouter
        if settings.QWEN_EMBEDDING_API_KEY:
            self.api_key = settings.QWEN_EMBEDDING_API_KEY
            self.base_url = settings.QWEN_EMBEDDING_BASE_URL
            self.headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            self.is_qwen = True
        else:
            self.api_key = settings.OPENROUTER_API_KEY
            self.base_url = "https://openrouter.ai/api/v1"
            self.headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            self.is_qwen = False

    async def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Get embeddings for a list of texts using OpenRouter API or Qwen via DashScope
        """
        if not texts:
            return []

        print(f"DEBUG: Getting embeddings for {len(texts)} texts")
        print(f"DEBUG: API Key set: {bool(self.api_key)}")
        print(f"DEBUG: Using Qwen: {self.is_qwen}")
        if self.is_qwen:
            print(f"DEBUG: Using Qwen embedding service at: {self.base_url}")
        else:
            print(f"DEBUG: Model: {settings.EMBEDDING_MODEL}")

        try:
            # Using httpx for async HTTP requests
            async with httpx.AsyncClient(timeout=30.0) as client:
                if self.is_qwen:
                    # Qwen/DashScope embedding API format
                    response = await client.post(
                        f"{self.base_url}/embeddings",
                        headers=self.headers,
                        json={
                            "model": "text-embedding-v1",  # Qwen embedding model
                            "input": {
                                "texts": texts
                            }
                        }
                    )
                else:
                    # OpenRouter embedding API format
                    response = await client.post(
                        f"{self.base_url}/embeddings",
                        headers=self.headers,
                        json={
                            "model": settings.EMBEDDING_MODEL,  # e.g., "nomic-ai/nomic-embed-text-v1.5"
                            "input": texts
                        }
                    )

                print(f"DEBUG: Embeddings API response status: {response.status_code}")
                print(f"DEBUG: Embeddings API response text: {response.text[:200]}...")  # First 200 chars

                if response.status_code != 200:
                    raise Exception(f"API request failed with status {response.status_code}: {response.text}")

                data = response.json()

                # Extract embeddings differently based on service
                if self.is_qwen:
                    embeddings = [item['embedding'] for item in data['data']['embeddings']]
                else:
                    embeddings = [item['embedding'] for item in data['data']]

                print(f"DEBUG: Successfully received {len(embeddings)} embeddings")
                return embeddings
        except Exception as e:
            # Log the full error details before raising
            print(f"ERROR: Error calling embeddings API: {e}")
            print(f"ERROR: API Key: {'SET' if self.api_key else 'NOT SET'}")
            print(f"ERROR: Base URL: {self.base_url}")
            print(f"ERROR: Using Qwen: {self.is_qwen}")
            if not self.is_qwen:
                print(f"ERROR: Model: {settings.EMBEDDING_MODEL}")
            print(f"ERROR: Input texts preview: {texts[0][:100] if texts else 'None'}...")
            raise  # Re-raise the exception instead of returning mock embeddings

    async def get_embeddings_batch(self, texts: List[str], batch_size: int = 10) -> List[List[float]]:
        """
        Get embeddings in batches to respect API limits
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = await self.get_embeddings(batch)
            all_embeddings.extend(batch_embeddings)

            # Add a small delay to respect rate limits
            await asyncio.sleep(0.1)

        return all_embeddings