import axios from 'axios';

// Get API base URL from environment or default to backend
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

// Create axios instance with default config
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: 30000, // 30 second timeout
  headers: {
    'Content-Type': 'application/json',
  }
});

class ChatService {
  // Start a new chat session
  async startNewSession(userId = null, metadata = {}) {
    try {
      const response = await apiClient.post('/start_session', {
        user_id: userId,
        metadata: metadata
      });
      return response.data.session_token;
    } catch (error) {
      console.error('Error starting new session:', error);
      throw error;
    }
  }

  // Send a message to the chatbot
  async sendMessage(message, sessionToken, queryType = 'full_book', selectedText = null, bookId = null) {
    try {
      const response = await apiClient.post('/chat', {
        message: message,
        session_token: sessionToken,
        query_type: queryType,
        selected_text: selectedText,
        book_id: bookId
      });
      return response.data;
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  // Get conversation history
  async getConversationHistory(sessionToken) {
    // Note: This would need a dedicated endpoint on the backend
    // For now, we'll return an empty array as history is managed server-side
    // and returned with each response
    return [];
  }

  // Ingest a book
  async ingestBook(bookId, title, content, metadata = {}) {
    try {
      const response = await apiClient.post('/ingest', {
        book_id: bookId,
        title: title,
        content: content,
        metadata: metadata
      });
      return response.data;
    } catch (error) {
      console.error('Error ingesting book:', error);
      throw error;
    }
  }
}

export const chatService = new ChatService();