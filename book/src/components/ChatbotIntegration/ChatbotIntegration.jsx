import React, { useState, useEffect, useRef } from 'react';

const ChatbotIntegration = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionToken, setSessionToken] = useState(null);
  const messagesEndRef = useRef(null);

  // Initialize chatbot session when component mounts
  useEffect(() => {
    const initializeSession = async () => {
      try {
        const response = await fetch('http://127.0.0.1:9008/api/v1/start_session', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({}),
        });

        const data = await response.json();
        setSessionToken(data.session_token);

        // Add initial greeting message
        setMessages([
          { id: Date.now(), text: "Hello! I'm your book assistant. Ask me anything about this book.", sender: 'assistant' }
        ]);
      } catch (error) {
        console.error('Error initializing chat session:', error);
        setMessages([
          { id: Date.now(), text: "⚠️ Connection issue: Backend server may not be running. Please start the backend server.", sender: 'assistant' }
        ]);
      }
    };

    initializeSession();
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async () => {
    console.log('Send button clicked!'); // DEBUG LOG
    if (!inputValue.trim() || isLoading) return; // Removed !sessionToken condition

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user'
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch('http://127.0.0.1:9008/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          session_token: sessionToken || 'fallback-session-token',
          query_type: 'full_book'
        }),
      });

      const data = await response.json();
      console.log('Response data:', data); // DEBUG LOG

      // Add assistant response to chat using the correct field name
      const assistantMessage = {
        id: Date.now() + 1,
        text: data.response || data.answer || data.message || "I'm sorry, I couldn't process that request.",
        sender: 'assistant'
      };

      console.log('Adding assistant message:', assistantMessage); // DEBUG LOG
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error); // DEBUG LOG
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error. Please make sure the backend server is running on http://127.0.0.1:9008",
        sender: 'assistant'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <>
      {/* Chatbot toggle button */}
      <button
        onClick={toggleChat}
        style={{
          position: 'fixed',
          bottom: isOpen ? '420px' : '20px',
          right: '20px',
          zIndex: 10000,
          background: '#4f46e5',
          color: 'white',
          border: 'none',
          borderRadius: '50%',
          width: '60px',
          height: '60px',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
        }}
      >
        💬
      </button>

      {/* Chatbot panel */}
      {isOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            width: '400px',
            height: '400px',
            zIndex: 9999,
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            borderRadius: '8px',
            overflow: 'hidden',
            fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
            display: 'flex',
            flexDirection: 'column',
            backgroundColor: 'white',
          }}
        >
          {/* Header */}
          <div
            style={{
              background: '#4f46e5',
              color: 'white',
              padding: '15px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center',
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>Book Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '20px',
                cursor: 'pointer',
                padding: 0,
                margin: 0,
              }}
            >
              &times;
            </button>
          </div>

          {/* Messages container */}
          <div
            style={{
              flex: 1,
              overflowY: 'auto',
              padding: '15px',
              backgroundColor: '#f9fafb',
            }}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  marginBottom: '15px',
                  padding: '10px',
                  borderRadius: '8px',
                  maxWidth: '80%',
                  backgroundColor: message.sender === 'user' ? '#dbeafe' : 'white',
                  marginLeft: message.sender === 'user' ? 'auto' : '0',
                  marginRight: message.sender === 'user' ? '0' : 'auto',
                  textAlign: message.sender === 'user' ? 'right' : 'left',
                }}
              >
                {message.text}
              </div>
            ))}
            {isLoading && (
              <div
                style={{
                  marginBottom: '15px',
                  padding: '10px',
                  borderRadius: '8px',
                  maxWidth: '80%',
                  backgroundColor: 'white',
                  marginLeft: '0',
                  marginRight: 'auto',
                  textAlign: 'left',
                }}
              >
                Typing...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input area - CRITICAL UI ELEMENTS */}
          <div
            style={{
              padding: '15px',
              background: 'white',
              borderTop: '1px solid #e5e7eb',
            }}
          >
            <div style={{ display: 'flex', gap: '10px' }}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question about this book..."
                style={{
                  flex: 1,
                  padding: '10px',
                  border: '1px solid #d1d5db',
                  borderRadius: '4px',
                  resize: 'none',
                  minHeight: '60px',
                  maxHeight: '100px',
                  fontFamily: 'inherit',
                }}
                disabled={isLoading}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || !inputValue.trim()}
                style={{
                  padding: '10px 15px',
                  background: isLoading || !inputValue.trim() ? '#94a3b8' : '#4f46e5',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  cursor: isLoading || !inputValue.trim() ? 'not-allowed' : 'pointer',
                  alignSelf: 'flex-start',
                }}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatbotIntegration;