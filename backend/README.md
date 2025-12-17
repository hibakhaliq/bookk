# RAG Chatbot Backend

This is the backend service for the RAG (Retrieval-Augmented Generation) chatbot that enables users to ask questions about book content.

## Features

- Document ingestion API for processing book content
- Vector storage using Qdrant for semantic search
- Integration with OpenRouter for embeddings and LLM calls
- PostgreSQL database for metadata storage

## Security Notice

⚠️ **IMPORTANT**: The example configuration contains actual API keys and connection strings.
For security reasons, please:
1. Never commit actual API keys to version control
2. Always use environment variables for sensitive data
3. Regenerate any exposed API keys immediately
4. Use proper secret management in production

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Create a `.env` file based on `.env.example`:
```bash
cp .env.example .env
# Edit .env with your actual configuration
# IMPORTANT: Use your own API keys and do not use the example values
```

3. Run the application:
```bash
python -m src.main
```

## API Endpoints

- `POST /api/v1/ingest` - Ingest book content for RAG
- `GET /` - Health check
- `GET /health` - Health status

## Example Usage

```bash
curl -X POST http://localhost:8000/api/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my-book-123",
    "title": "My Book Title",
    "content": "Full text content of the book..."
  }'
```