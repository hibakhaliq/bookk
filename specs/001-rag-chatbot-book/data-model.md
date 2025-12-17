# Data Model: RAG Chatbot for Book Content

## Overview
This document defines the data models for the RAG chatbot system, including entities, relationships, and validation rules based on the feature specification.

## Entity Models

### 1. ChatSession
**Description**: Represents a conversation between a user and the chatbot, containing the history of messages and context.

**Fields**:
- `id` (UUID): Unique identifier for the session
- `user_id` (String, optional): Identifier for the user (if authenticated)
- `session_token` (String): Unique token to identify the session
- `created_at` (DateTime): Timestamp when session was created
- `updated_at` (DateTime): Timestamp when session was last updated
- `metadata` (JSON): Additional session metadata (book_id, chapter, etc.)

**Validation Rules**:
- `id` must be a valid UUID
- `session_token` must be unique
- `created_at` and `updated_at` are auto-generated
- `metadata` must be valid JSON

**State Transitions**:
- Active (new session) → Inactive (after 24 hours of inactivity)

### 2. Message
**Description**: An individual message in a chat session, including the user query and system response.

**Fields**:
- `id` (UUID): Unique identifier for the message
- `session_id` (UUID): Reference to the parent ChatSession
- `role` (String): Message role ('user' or 'assistant')
- `content` (Text): The actual message content
- `timestamp` (DateTime): When the message was created
- `metadata` (JSON): Additional message metadata (query_type: 'full_book' or 'selected_text', selected_text, etc.)

**Validation Rules**:
- `id` must be a valid UUID
- `session_id` must reference an existing ChatSession
- `role` must be either 'user' or 'assistant'
- `content` cannot be empty
- `timestamp` is auto-generated
- `metadata` must be valid JSON

**Relationships**:
- Belongs to one ChatSession
- Multiple messages per ChatSession

### 3. BookContentChunk
**Description**: A segment of the book content that has been processed and stored in the vector database for retrieval.

**Fields**:
- `id` (UUID): Unique identifier for the chunk
- `book_id` (String): Identifier for the book
- `chapter_id` (String): Identifier for the chapter/section
- `content` (Text): The actual content of the chunk
- `embedding_vector` (Vector): The embedding vector for similarity search
- `metadata` (JSON): Additional metadata (page number, section, etc.)
- `hash` (String): Hash of content to detect changes

**Validation Rules**:
- `id` must be a valid UUID
- `content` cannot be empty
- `embedding_vector` must match expected dimensions for Qwen model
- `hash` is auto-computed from content
- `metadata` must be valid JSON

**Relationships**:
- Multiple chunks per book

### 4. UserQuery
**Description**: The input from the user that triggers the RAG process to find relevant information from the book.

**Fields**:
- `id` (UUID): Unique identifier for the query
- `session_id` (UUID): Reference to the ChatSession
- `query_text` (Text): The original user query
- `query_type` (String): Type of query ('full_book' or 'selected_text')
- `selected_text` (Text, optional): The text selected by user (for selected_text queries)
- `timestamp` (DateTime): When the query was made

**Validation Rules**:
- `id` must be a valid UUID
- `session_id` must reference an existing ChatSession
- `query_type` must be either 'full_book' or 'selected_text'
- For 'selected_text' queries, `selected_text` cannot be empty
- `timestamp` is auto-generated

**Relationships**:
- Belongs to one ChatSession
- Multiple queries per ChatSession

### 5. RetrievedContext
**Description**: The relevant book content retrieved from the vector store based on the user's query.

**Fields**:
- `id` (UUID): Unique identifier for the retrieval
- `query_id` (UUID): Reference to the UserQuery
- `chunk_id` (UUID): Reference to the BookContentChunk
- `similarity_score` (Float): Similarity score from vector search
- `retrieved_content` (Text): The content that was retrieved
- `retrieval_rank` (Integer): Rank in the retrieval results

**Validation Rules**:
- `id` must be a valid UUID
- `query_id` must reference an existing UserQuery
- `chunk_id` must reference an existing BookContentChunk
- `similarity_score` must be between 0 and 1
- `retrieval_rank` must be a positive integer

**Relationships**:
- Belongs to one UserQuery
- References one BookContentChunk
- Multiple retrieved contexts per query

## Database Schema

### Postgres Tables (Neon Serverless)

```sql
-- Chat sessions table
CREATE TABLE chat_sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id VARCHAR(255),
  session_token VARCHAR(255) UNIQUE NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  metadata JSONB
);

-- Messages table
CREATE TABLE messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
  role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
  content TEXT NOT NULL,
  timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  metadata JSONB
);

-- User queries table
CREATE TABLE user_queries (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
  query_text TEXT NOT NULL,
  query_type VARCHAR(20) NOT NULL CHECK (query_type IN ('full_book', 'selected_text')),
  selected_text TEXT,
  timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Retrieved contexts table
CREATE TABLE retrieved_contexts (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  query_id UUID REFERENCES user_queries(id) ON DELETE CASCADE,
  chunk_id VARCHAR(255) NOT NULL,  -- References Qdrant ID
  similarity_score DECIMAL(5,4) NOT NULL,
  retrieved_content TEXT NOT NULL,
  retrieval_rank INTEGER NOT NULL
);
```

### Qdrant Collection Schema

```json
{
  "name": "book_content_chunks",
  "vector_size": 768,  // Assuming Qwen embedding dimension
  "distance": "Cosine",
  "payload_schema": {
    "book_id": {"type": "keyword"},
    "chapter_id": {"type": "keyword"},
    "chunk_hash": {"type": "keyword"},
    "metadata": {"type": "keyword"}
  }
}
```

## Validation Rules Summary

1. **ChatSession**: Must have unique session_token, valid UUIDs, auto-generated timestamps
2. **Message**: Must belong to valid session, valid role, non-empty content
3. **BookContentChunk**: Must have valid embedding dimensions, non-empty content, unique hash
4. **UserQuery**: Must have valid query_type, selected_text for selected_text queries
5. **RetrievedContext**: Must reference valid query and chunk, valid similarity score range