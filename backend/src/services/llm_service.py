import httpx
import asyncio
from typing import List, Dict, Any
from ..config import settings


class LLMService:
    def __init__(self):
        # Check if Gemini API key is available, otherwise fall back to OpenRouter
        if settings.GEMINI_API_KEY:
            self.api_key = settings.GEMINI_API_KEY
            self.base_url = f"https://generativelanguage.googleapis.com/v1beta/models/{settings.GEMINI_MODEL}:generateContent?key={self.api_key}"
            self.is_gemini = True
        else:
            self.api_key = settings.OPENROUTER_API_KEY
            self.base_url = "https://openrouter.ai/api/v1"
            self.is_gemini = False

        if self.is_gemini:
            self.headers = {
                "Content-Type": "application/json"
            }
        else:
            self.headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

    async def generate_text(self, prompt: str, model: str = "openai/gpt-3.5-turbo", max_tokens: int = 500) -> str:
        """
        Generate text using either Gemini or OpenRouter API
        """
        print(f"DEBUG: Calling LLM API, using Gemini: {self.is_gemini}")
        print(f"DEBUG: API Key set: {bool(self.api_key)}")
        print(f"DEBUG: Prompt length: {len(prompt)}")

        try:
            # Using httpx for async HTTP requests
            async with httpx.AsyncClient(timeout=30.0) as client:
                if self.is_gemini:
                    # Gemini API format
                    response = await client.post(
                        f"https://generativelanguage.googleapis.com/v1beta/models/{settings.GEMINI_MODEL}:generateContent?key={self.api_key}",
                        headers=self.headers,
                        json={
                            "contents": [{
                                "parts": [{
                                    "text": prompt
                                }]
                            }],
                            "generationConfig": {
                                "maxOutputTokens": max_tokens
                            }
                        }
                    )
                else:
                    # OpenRouter API format
                    response = await client.post(
                        f"{self.base_url}/chat/completions",
                        headers=self.headers,
                        json={
                            "model": model,
                            "messages": [
                                {"role": "user", "content": prompt}
                            ],
                            "max_tokens": max_tokens
                        }
                    )

                print(f"DEBUG: API response status: {response.status_code}")
                print(f"DEBUG: API response text: {response.text[:200]}...")  # First 200 chars

                if response.status_code != 200:
                    raise Exception(f"API request failed with status {response.status_code}: {response.text}")

                data = response.json()

                if self.is_gemini:
                    # Extract content from Gemini response
                    if 'candidates' not in data or len(data['candidates']) == 0:
                        raise Exception(f"Invalid Gemini API response format: {data}")
                    content = data['candidates'][0]['content']['parts'][0]['text']
                else:
                    # Extract content from OpenRouter response
                    if 'choices' not in data or len(data['choices']) == 0:
                        raise Exception(f"Invalid OpenRouter API response format: {data}")
                    content = data['choices'][0]['message']['content']

                print(f"DEBUG: Successfully received response from LLM, content length: {len(content)}")
                return content
        except Exception as e:
            # Log the full error details before raising
            print(f"ERROR: Error calling LLM API: {e}")
            print(f"ERROR: API Key: {'SET' if self.api_key else 'NOT SET'}")
            print(f"ERROR: Using Gemini: {self.is_gemini}")
            print(f"ERROR: Base URL: {self.base_url}")
            print(f"ERROR: Prompt preview: {prompt[:100]}...")
            raise  # Re-raise the exception instead of returning mock response

    async def generate_response_with_context(self, context: str, question: str, chat_history: List[Dict[str, Any]] = None) -> str:
        """
        Generate a response using the context and chat history
        """
        # Format the prompt with context
        if chat_history:
            history_str = "\n".join([f"{msg['role']}: {msg['content']}" for msg in chat_history])
            prompt = f"""
            Context information:
            {context}

            Previous conversation:
            {history_str}

            Current question: {question}

            Please provide a helpful answer based on the context information and maintain the conversation flow.
            """
        else:
            prompt = f"""
            Context information:
            {context}

            Question: {question}

            Please provide a helpful answer based on the context information.
            """

        # Call the LLM
        response = await self.generate_text(prompt)
        return response