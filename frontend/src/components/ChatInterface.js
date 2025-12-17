import React, { useState, useEffect } from 'react';
import { useConversationHistory } from '../hooks/useConversationHistory';
import { useTextSelection } from '../hooks/useTextSelection';
import { chatService } from '../services/chatService';
import TextSelection from './TextSelection';
import ChatHistory from './ChatHistory';

const ChatInterface = ({ initialSessionToken = null }) => {
  const [sessionToken, setSessionToken] = useState(initialSessionToken);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [queryType, setQueryType] = useState('full_book'); // 'full_book' or 'selected_text'
  const [selectedText, setSelectedText] = useState('');
  const [error, setError] = useState(null);

  const { messages, addMessage } = useConversationHistory(sessionToken);
  const { isSelecting } = useTextSelection();

  // Initialize session if we don't have one
  useEffect(() => {
    if (!sessionToken) {
      initializeSession();
    }
  }, []);

  const initializeSession = async () => {
    try {
      const newToken = await chatService.startNewSession();
      setSessionToken(newToken);
    } catch (err) {
      setError('Failed to initialize chat session');
      console.error(err);
    }
  };

  const handleTextSelected = (text, isSelecting) => {
    if (isSelecting && text) {
      setSelectedText(text);
      setQueryType('selected_text');
    } else if (!isSelecting) {
      // If no text is selected, switch back to full book mode
      setQueryType('full_book');
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputMessage.trim() || !sessionToken || isLoading) return;

    try {
      setIsLoading(true);
      setError(null);

      // Add user message to UI immediately
      addMessage({
        role: 'user',
        content: inputMessage,
        timestamp: new Date().toISOString()
      });

      // Send message to backend
      const response = await chatService.sendMessage(
        inputMessage,
        sessionToken,
        queryType,
        queryType === 'selected_text' ? selectedText : null
      );

      // Add assistant response to UI
      addMessage({
        role: 'assistant',
        content: response.response,
        sources: response.sources,
        timestamp: new Date().toISOString()
      });

      // Clear input
      setInputMessage('');

      // If using selected text mode and response is given, clear the selection
      if (queryType === 'selected_text') {
        setQueryType('full_book');
        setSelectedText('');
      }
    } catch (err) {
      setError('Failed to send message. Please try again.');
      console.error(err);
    } finally {
      setIsLoading(false);
    }
  };

  if (!sessionToken) {
    return <div className="chat-interface">Initializing chat session...</div>;
  }

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>RAG Chatbot</h3>
        <div className="query-type-selector">
          <button
            className={queryType === 'full_book' ? 'active' : ''}
            onClick={() => {
              setQueryType('full_book');
              setSelectedText('');
            }}
          >
            Full Book Query
          </button>
          <button
            className={queryType === 'selected_text' ? 'active' : ''}
            onClick={() => setQueryType('selected_text')}
          >
            Selected Text Query
          </button>
        </div>
      </div>

      <TextSelection onTextSelected={handleTextSelected} />

      {selectedText && queryType === 'selected_text' && (
        <div className="selected-text-context">
          <strong>Using selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
        </div>
      )}

      <ChatHistory messages={messages} isLoading={false} />

      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          placeholder={queryType === 'selected_text' && selectedText
            ? "Ask a question about the selected text..."
            : "Ask a question about the book..."}
          disabled={isLoading}
        />
        <button type="submit" disabled={isLoading || !inputMessage.trim()}>
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;