# Quickstart Guide for RAG Chatbot

This guide will help you set up and run the RAG Chatbot system quickly.

## Prerequisites

- Python 3.9 or higher
- pip package manager
- Git
- Access to OpenRouter API (for Qwen embeddings and LLM)
- Qdrant Cloud account (free tier available)
- Neon Serverless Postgres account (free tier available)

## Environment Setup

1. Clone the repository:
```bash
git clone <your-repo-url>
cd rag-chatbot-book
```

2. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

4. Create environment file:
```bash
cp .env.example .env
```

5. Update the `.env` file with your API keys and connection strings:
```env
# OpenRouter API
OPENROUTER_API_KEY=your_openrouter_api_key_here

# Qdrant Cloud
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here

# Neon Postgres
NEON_DATABASE_URL=your_neon_database_url_here

# App settings
APP_NAME=RAG Chatbot
API_V1_STR=/api/v1
```

## Running the Application

### Backend (FastAPI)

1. Navigate to the backend directory:
```bash
cd backend
```

2. Run the application:
```bash
python -m src.main
```

The API will be available at `http://localhost:8000`

### Frontend (React)

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

3. Run the development server:
```bash
npm start
```

The frontend will be available at `http://localhost:3000`

## API Usage

### 1. Ingest a Book

```bash
curl -X POST "http://localhost:8000/api/v1/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my-book-1",
    "title": "My Book Title",
    "content": "Full content of the book goes here...",
    "metadata": {}
  }'
```

### 2. Start a New Session

```bash
curl -X POST "http://localhost:8000/api/v1/start_session" \
  -H "Content-Type: application/json"
```

### 3. Chat with the Bot

```bash
curl -X POST "http://localhost:8000/api/v1/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is this book about?",
    "session_token": "session-token-from-step-2",
    "query_type": "full_book",
    "book_id": "my-book-1"
  }'
```

### 4. Chat with Selected Text

```bash
curl -X POST "http://localhost:8000/api/v1/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does this text mean?",
    "session_token": "session-token-from-step-2",
    "query_type": "selected_text",
    "selected_text": "The specific text that was selected by the user",
    "book_id": "my-book-1"
  }'
```

### 5. Get Application Metrics

```bash
curl "http://localhost:8000/metrics"
```

## Frontend Integration

To embed the chatbot in your Speckit-published book:

1. Include the embed script in your HTML:
```html
<script src="/path/to/chatbot-embed.js"></script>
<div id="rag-chatbot-container"></div>
```

2. Initialize the chatbot:
```javascript
window.RAGChatbot.init({
  backendUrl: 'http://localhost:8000',
  containerId: 'rag-chatbot-container'
});
```

## Validation Steps

To validate the deployment:

1. **Verify API is running**:
   - Visit `http://localhost:8000/` - should return API status
   - Visit `http://localhost:8000/health` - should return healthy status
   - Visit `http://localhost:8000/metrics` - should return metrics data

2. **Test ingestion**:
   - Send a test book content to `/api/v1/ingest`
   - Verify the response indicates successful ingestion

3. **Test chat functionality**:
   - Start a session via `/api/v1/start_session`
   - Send a test query to `/api/v1/chat`
   - Verify you receive a meaningful response

4. **Check metrics**:
   - After making API calls, check `/metrics` endpoint
   - Verify metrics are being collected and updated

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify your OpenRouter, Qdrant, and Neon credentials in the `.env` file.

2. **Database Connection Issues**: Check that your Neon Postgres connection string is correct and the database is accessible.

3. **Vector Store Issues**: Ensure your Qdrant Cloud instance is properly configured and accessible.

4. **Embedding Generation Failures**: Verify your OpenRouter API key has access to the Qwen model.

### Logs

Check the application logs for any errors or warnings during operation.

## Architecture Overview

The RAG Chatbot system consists of:

- **Backend**: FastAPI application handling ingestion and chat endpoints
- **Vector Store**: Qdrant Cloud for semantic search
- **Metadata Store**: Neon Serverless Postgres for chat history and book metadata
- **Embeddings**: Qwen model via OpenRouter API
- **LLM**: OpenRouter API for response generation
- **Frontend**: React application with embeddable chat interface
- **Caching**: LRU cache for frequently accessed embeddings and query results
- **Metrics**: Built-in monitoring and metrics collection