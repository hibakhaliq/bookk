# Research Summary: RAG Chatbot for Book Content

## Overview
This document captures research findings for the RAG chatbot implementation, addressing key technical decisions and unknowns identified during planning.

## 1. Data Ingestion Pipeline Research

### Decision: Speckit Book Format Parsing
**Rationale**: The system needs to parse Speckit-generated book content to extract text for embedding.
**Approach**:
- Speckit generates static HTML/JS, so we'll implement a content extraction service that can parse HTML documents and extract text content
- Use BeautifulSoup4 for HTML parsing and text extraction
- Handle various content types: chapters, sections, paragraphs, code blocks, etc.

**Alternatives considered**:
- Direct Markdown parsing: Not applicable as Speckit outputs HTML
- PDF parsing: Not needed as we're working with web-based book output

### Decision: Text Chunking Strategy
**Rationale**: Need to break book content into manageable chunks for embedding and retrieval.
**Approach**:
- Use semantic chunking based on document structure (sections, paragraphs)
- Target chunk size: 500-1000 tokens to balance context and retrieval accuracy
- Overlap chunks by 20% to preserve context across boundaries

**Alternatives considered**:
- Fixed-size character chunking: Could break semantic meaning
- Sentence-level chunking: Might result in too small chunks

### Decision: Qwen Embedding Integration
**Rationale**: Need to generate embeddings using Qwen model via OpenRouter API.
**Approach**:
- Use OpenRouter Python client to access Qwen embeddings
- Batch embedding requests to optimize API usage
- Cache embeddings to avoid reprocessing identical content

**Alternatives considered**:
- Self-hosted embedding models: Higher infrastructure complexity
- Alternative embedding providers: Qwen specified in requirements

## 2. FastAPI Backend Architecture

### Decision: API Endpoint Design
**Rationale**: Need to define clear API contracts for ingestion and chat functionality.
**Approach**:
- `/ingest` endpoint: Accept book content and process for RAG
- `/chat` endpoint: Handle chat queries with full-book and selected-text modes
- Use Pydantic models for request/response validation
- Implement proper error handling and response codes

**Alternatives considered**:
- GraphQL: More complex than needed for this use case
- WebSocket for chat: HTTP polling sufficient for initial implementation

### Decision: OpenRouter API Integration
**Rationale**: Need to integrate with OpenRouter for Qwen embeddings and LLM calls.
**Approach**:
- Use OpenRouter Python SDK
- Implement API key management via environment variables
- Handle rate limiting and retries gracefully

## 3. RAG Logic Implementation

### Decision: Qdrant Vector Store Configuration
**Rationale**: Need to configure Qdrant for efficient similarity search.
**Approach**:
- Use Qdrant Cloud with free tier
- Configure collection with appropriate vector dimensions for Qwen embeddings
- Implement hybrid search if needed for better results

**Alternatives considered**:
- Other vector stores: Qdrant specified in requirements

### Decision: Chat History Management
**Rationale**: Need to store and retrieve conversation history.
**Approach**:
- Use Neon Serverless Postgres to store chat sessions and messages
- Design schema with ChatSession and Message entities
- Implement session-based context management

## 4. Frontend Integration

### Decision: ChatKit UI Integration
**Rationale**: Need to embed ChatKit UI in the book frontend.
**Approach**:
- Create a JavaScript embed script that injects ChatKit UI into book pages
- Implement communication between frontend and backend via REST API
- Add text selection functionality for selected-text query mode

**Alternatives considered**:
- Custom UI: ChatKit specified in requirements
- iFrame embedding: Would limit integration capabilities

## 5. Architecture and Deployment

### Decision: Deployment Strategy
**Rationale**: Need to determine how to deploy and integrate with existing book.
**Approach**:
- Deploy backend as a separate service (e.g., on Render, Railway, or similar)
- Embed frontend component in Speckit-generated pages
- Use environment variables for configuration

### Decision: Security Considerations
**Rationale**: Need to secure API endpoints and protect user data.
**Approach**:
- Implement API rate limiting
- Validate and sanitize all inputs
- Use HTTPS for all communications
- Secure API keys with environment variables

## Open Questions & Assumptions

1. **Book content format**: Assuming Speckit outputs standard HTML that can be parsed with BeautifulSoup
2. **API rate limits**: Need to monitor OpenRouter rate limits during implementation
3. **Qwen embedding dimensions**: Will determine based on OpenRouter documentation
4. **CORS configuration**: Need to configure properly for frontend-backend communication