# Quickstart Guide: RAG Chatbot for Book Content

## Overview
This guide provides instructions to quickly set up and run the RAG chatbot for book content.

## Prerequisites
- Python 3.11+
- Node.js 18+
- Docker (optional, for local development)
- OpenRouter API key
- Qdrant Cloud account
- Neon Serverless Postgres account

## Environment Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd rag-chatbot-book
```

### 2. Backend Setup
```bash
cd backend
pip install -r requirements.txt
cp .env.example .env
```

Edit `.env` with your configuration:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
```

### 3. Frontend Setup
```bash
cd frontend
npm install
cp .env.example .env
```

Edit `.env` with your backend API URL:
```env
REACT_APP_API_URL=http://localhost:8000
```

## Running Locally

### 1. Start the backend
```bash
cd backend
python -m src.main
```

The backend will start on `http://localhost:8000`

### 2. Start the frontend
```bash
cd frontend
npm start
```

The frontend will start on `http://localhost:3000`

## API Usage

### 1. Ingest Book Content
```bash
curl -X POST http://localhost:8000/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "my-book-123",
    "title": "My Book Title",
    "content": "Full text content of the book..."
  }'
```

### 2. Chat with the Bot
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is this book about?",
    "session_token": "session-123"
  }'
```

## Integration with Speckit Book

### 1. Build the frontend
```bash
cd frontend
npm run build
```

### 2. Copy the embed script
The build process generates `chatbot-embed.js` which can be included in Speckit-generated HTML files.

### 3. Add to book HTML
```html
<!-- In your Speckit book HTML -->
<script src="chatbot-embed.js"></script>
<div id="chatbot-container"></div>
```

## Testing

### Backend Tests
```bash
cd backend
pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Deployment

### Backend (example with Render)
1. Create a new Web Service on Render
2. Connect to your GitHub repository
3. Set the build command to `pip install -r requirements.txt`
4. Set the start command to `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables as needed

### Frontend (example with Vercel)
1. Create a new project on Vercel
2. Connect to your GitHub repository
3. Set the framework to "Create React App"
4. Add environment variables as needed