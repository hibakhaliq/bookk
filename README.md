# RAG Chatbot for Book Content

A Retrieval-Augmented Generation (RAG) chatbot system that allows users to ask questions about book content with support for full-book queries and selected-text queries.

## Architecture

- **Backend**: FastAPI application handling document ingestion and chat endpoints
- **Vector Store**: Qdrant Cloud for semantic search
- **Metadata Store**: Neon Serverless Postgres for chat history and book metadata
- **Embeddings**: Qwen model via OpenRouter API
- **LLM**: OpenRouter API for response generation
- **Frontend**: React application with embeddable chat interface

## Prerequisites

- Python 3.9+ (for backend)
- Node.js 16+ (for frontend)
- OpenRouter API key (for Qwen embeddings and LLM)
- Qdrant Cloud account (free tier available)
- Neon Serverless Postgres account (free tier available)

## Setup Instructions

### 1. Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Create and activate a virtual environment:
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

6. Start the backend server:
```bash
python -m src.main
```

The backend will be available at `http://localhost:8000`

### 2. Frontend Setup

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The frontend will be available at `http://localhost:3000`

## API Endpoints

### Backend API (running on http://localhost:8000)

- `GET /` - API root endpoint
- `GET /health` - Health check
- `GET /metrics` - Application metrics
- `GET /metrics/prometheus` - Prometheus metrics format
- `POST /api/v1/ingest` - Ingest book content
- `POST /api/v1/start_session` - Start new chat session
- `POST /api/v1/chat` - Chat endpoint

### Example API Usage

#### 1. Ingest a Book
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

#### 2. Start a New Session
```bash
curl -X POST "http://localhost:8000/api/v1/start_session" \
  -H "Content-Type: application/json"
```

#### 3. Chat with the Bot
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

#### 4. Chat with Selected Text
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

## Frontend Integration

To embed the chatbot in your Speckit-published book:

### Method 1: Direct Script Inclusion
Add this to your HTML template or individual pages:

```html
<!-- Configure the API URL -->
<script>
  window.CHATBOT_API_URL = 'http://localhost:8000/api/v1'; // Change to your deployed URL
</script>

<!-- Include the chatbot embed script -->
<script src="/path/to/chatbot-embed.js"></script>
```

### Method 2: Manual Integration
If you prefer to include the script directly in your HTML:

```html
<script>
  // Configure the chatbot API URL
  window.CHATBOT_API_URL = 'http://localhost:8000/api/v1';
</script>

<script>
  // Paste the entire embed script here
  // (The content from frontend/src/embed/ChatbotEmbed.js)
</script>
```

### Embedding Guide
For detailed embedding instructions, see [EMBEDDING_GUIDE.md](EMBEDDING_GUIDE.md).

### Testing the Embedding
A sample HTML file with embedded chatbot is available at [sample_embedded_book.html](sample_embedded_book.html) to demonstrate the integration.

### Chatbot Features
- **Floating Widget**: Appears as a floating button on your book pages
- **Two Query Modes**:
  - Full Book: Answers questions based on entire book content
  - Selected Text: Answers questions based only on user-selected text
- **Session Management**: Maintains conversation context
- **Responsive Design**: Works on desktop and mobile devices

## Configuration Details

### Required API Keys and Services

1. **OpenRouter API Key**:
   - Required for Qwen embeddings and LLM calls
   - Get from: https://openrouter.ai/keys
   - Environment variable: `OPENROUTER_API_KEY`

2. **Qdrant Cloud**:
   - Required for vector storage and semantic search
   - Get from: https://cloud.qdrant.io/
   - Environment variables: `QDRANT_URL` and `QDRANT_API_KEY`

3. **Neon Serverless Postgres**:
   - Required for metadata and chat history storage
   - Get from: https://neon.tech/
   - Environment variable: `NEON_DATABASE_URL`

### Configuration File Structure

```
backend/
├── .env                    # Environment variables
├── requirements.txt        # Python dependencies
└── src/
    ├── main.py            # Main FastAPI application
    ├── config.py          # Configuration settings
    ├── api/               # API routes
    ├── services/          # Business logic
    ├── database/          # Database connections
    ├── models/            # Data models
    ├── middleware/        # Middleware
    └── utils/             # Utility functions

frontend/
├── package.json           # Node.js dependencies
├── public/                # Static files
└── src/
    ├── components/        # React components
    ├── services/          # API services
    ├── hooks/             # React hooks
    ├── contexts/          # React contexts
    ├── embed/             # Embeddable script
    ├── styles/            # CSS styles
    └── types/             # TypeScript types
```

## Embeddable Script

The frontend includes an embeddable script at `frontend/src/embed/ChatbotEmbed.js` that can be built and used to integrate the chatbot into any webpage, including Speckit-published books.

To build the embeddable script:
```bash
cd frontend
npm run build
```

The built script will be available in the `build/` directory.

## Running in Production

### Backend
```bash
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
```

### Frontend
```bash
cd frontend
npm run build
# Serve the build directory using your preferred web server
```

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify your OpenRouter, Qdrant, and Neon credentials in the `.env` file.

2. **Database Connection Issues**: Check that your Neon Postgres connection string is correct and the database is accessible.

3. **Vector Store Issues**: Ensure your Qdrant Cloud instance is properly configured and accessible.

4. **Embedding Generation Failures**: Verify your OpenRouter API key has access to the Qwen model.

### Metrics and Monitoring

- Access application metrics at `http://localhost:8000/metrics`
- Prometheus metrics available at `http://localhost:8000/metrics/prometheus`
- Metrics include request counts, response times, cache performance, and service health

## Development

### Backend Development
```bash
cd backend
# Install development dependencies if needed
pip install -r requirements-dev.txt
# Run with auto-reload
uvicorn src.main:app --reload
```

### Frontend Development
```bash
cd frontend
npm start
# This will start the development server with hot reloading
```

## Security Considerations

- Never commit API keys to version control
- Use environment variables for sensitive configuration
- Implement proper authentication for production deployments
- Use HTTPS in production environments
- Validate and sanitize all user inputs 
