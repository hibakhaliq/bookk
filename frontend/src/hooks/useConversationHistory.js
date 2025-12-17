import { useState, useEffect } from 'react';
import { chatService } from '../services/chatService';

export const useConversationHistory = (sessionToken) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Load conversation history when session token changes
  useEffect(() => {
    if (sessionToken) {
      loadHistory();
    }
  }, [sessionToken]);

  const loadHistory = async () => {
    try {
      setIsLoading(true);
      const history = await chatService.getConversationHistory(sessionToken);
      setMessages(history);
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const addMessage = (message) => {
    setMessages(prev => [...prev, message]);
  };

  const clearHistory = () => {
    setMessages([]);
  };

  return {
    messages,
    isLoading,
    error,
    addMessage,
    clearHistory,
    reload: loadHistory
  };
};