import React from 'react';

const ChatHistory = ({ messages = [], isLoading = false }) => {
  if (isLoading) {
    return <div className="chat-history loading">Loading conversation...</div>;
  }

  if (!messages || messages.length === 0) {
    return <div className="chat-history empty">No messages yet. Start a conversation!</div>;
  }

  return (
    <div className="chat-history">
      {messages.map((message, index) => (
        <div
          key={index}
          className={`message ${message.role || 'user'}`}
        >
          <div className="message-content">
            {message.content || message.text}
          </div>
          {message.timestamp && (
            <div className="message-timestamp">
              {new Date(message.timestamp).toLocaleTimeString()}
            </div>
          )}
        </div>
      ))}
    </div>
  );
};

export default ChatHistory;