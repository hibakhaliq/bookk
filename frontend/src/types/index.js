// Type definitions for the RAG Chatbot frontend

// Message types
export const MessageRole = {
  USER: 'user',
  ASSISTANT: 'assistant'
};

// Query types
export const QueryType = {
  FULL_BOOK: 'full_book',
  SELECTED_TEXT: 'selected_text'
};

// Message interface
export interface Message {
  id?: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: Array<{
    chunk_id: string;
    similarity_score: number;
    content_preview: string;
  }>;
}

// Chat session interface
export interface ChatSession {
  sessionToken: string;
  messages: Message[];
  queryType: 'full_book' | 'selected_text';
  selectedText?: string;
}

// API response interfaces
export interface ChatResponse {
  response: string;
  session_token: string;
  sources: Array<{
    chunk_id: string;
    similarity_score: number;
    content_preview: string;
  }>;
  query_type: string;
}

export interface IngestResponse {
  status: string;
  chunks_processed: number;
  message: string;
  postgres_ids: string[];
  qdrant_ids: string[];
}

// Context state interface
export interface ChatState {
  sessionToken: string | null;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  queryType: 'full_book' | 'selected_text';
  selectedText: string;
  isSelecting: boolean;
}

// Context actions interface
export interface ChatActions {
  setSessionToken: (token: string) => void;
  addMessage: (message: Message) => void;
  setLoading: (isLoading: boolean) => void;
  setError: (error: string) => void;
  setQueryType: (queryType: 'full_book' | 'selected_text') => void;
  setSelectedText: (text: string) => void;
  setIsSelecting: (isSelecting: boolean) => void;
  clearMessages: () => void;
  reset: () => void;
}