// Chatbot Embed Script for Speckit Integration
// This script can be included in Speckit-generated HTML pages to embed the chatbot

(function() {
  // Configuration - Use the window variable set by the client, with fallback
  const API_BASE_URL = window.CHATBOT_API_URL || 'http://127.0.0.1:8080/api/v1';
  const CONTAINER_ID = 'rag-chatbot-container';

  // Create the chatbot container and UI
  function createChatbotUI() {
    // Create main container
    const container = document.createElement('div');
    container.id = CONTAINER_ID;
    container.style.cssText = `
      position: fixed;
      bottom: 20px;
      right: 20px;
      width: 400px;
      height: 600px;
      z-index: 10000;
      box-shadow: 0 4px 12px rgba(0,0,0,0.15);
      border-radius: 8px;
      overflow: hidden;
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      display: none;
    `;

    // Create toggle button
    const toggleBtn = document.createElement('button');
    toggleBtn.textContent = '💬 Ask Book';
    toggleBtn.style.cssText = `
      position: absolute;
      bottom: 0;
      right: 0;
      z-index: 10001;
      background: #4f46e5;
      color: white;
      border: none;
      border-radius: 50%;
      width: 60px;
      height: 60px;
      font-size: 24px;
      cursor: pointer;
      box-shadow: 0 4px 12px rgba(0,0,0,0.15);
      display: flex;
      align-items: center;
      justify-content: center;
    `;
    toggleBtn.onclick = toggleChatbot;

    document.body.appendChild(toggleBtn);
    document.body.appendChild(container);

    // Initialize chatbot UI inside container
    initializeChatInterface(container);
  }

  function initializeChatInterface(container) {
    container.innerHTML = `
      <div id="chatbot-header" style="
        background: #4f46e5;
        color: white;
        padding: 15px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      ">
        <h3 style="margin: 0; font-size: 16px;">Book Assistant</h3>
        <button id="close-chatbot" style="
          background: none;
          border: none;
          color: white;
          font-size: 20px;
          cursor: pointer;
        ">&times;</button>
      </div>
      <div id="chatbot-messages" style="
        height: calc(100% - 120px);
        overflow-y: auto;
        padding: 15px;
        background: #f9fafb;
      "></div>
      <div id="chatbot-input-area" style="
        padding: 15px;
        background: white;
        border-top: 1px solid #e5e7eb;
        display: block;
      ">
        <div style="display: flex; margin-bottom: 10px;">
          <button id="full-book-mode" style="
            flex: 1;
            padding: 8px;
            background: #e5e7eb;
            border: 1px solid #d1d5db;
            border-radius: 4px 0 0 4px;
            cursor: pointer;
          ">Full Book</button>
          <button id="selected-text-mode" style="
            flex: 1;
            padding: 8px;
            background: #e5e7eb;
            border: 1px solid #d1d5db;
            border-left: none;
            border-radius: 0 4px 4px 0;
            cursor: pointer;
          ">Selected Text</button>
        </div>
        <textarea
          id="chatbot-input"
          placeholder="Ask a question about this book..."
          style="
            width: 100%;
            padding: 10px;
            border: 1px solid #d1d5db;
            border-radius: 4px;
            resize: none;
            height: 60px;
            font-family: inherit;
          "></textarea>
        <button id="send-message" style="
          margin-top: 10px;
          width: 100%;
          padding: 10px;
          background: #4f46e5;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
        ">Send</button>
      </div>
    `;

    // Add event listeners
    document.getElementById('close-chatbot').onclick = () => {
      container.style.display = 'none';
    };

    document.getElementById('send-message').onclick = sendMessage;
    document.getElementById('chatbot-input').addEventListener('keypress', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        sendMessage();
      }
    });

    document.getElementById('full-book-mode').onclick = () => setActiveMode('full_book');
    document.getElementById('selected-text-mode').onclick = () => setActiveMode('selected_text');

    // Initialize with input area hidden
    initializeChatbotSession();
  }

  let currentSessionToken = null;
  let currentMode = 'full_book';

  async function initializeChatbotSession() {
    try {
      const response = await fetch(`${API_BASE_URL}/start_session`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({})
      });

      const data = await response.json();
      currentSessionToken = data.session_token;

      addMessageToUI('Hello! I\'m your book assistant. Ask me anything about this book.', 'assistant');
      // Show input area after successful initialization
      document.getElementById('chatbot-input-area').style.display = 'block';
    } catch (error) {
      console.error('Error initializing chatbot session:', error);
      addMessageToUI('⚠️ Connection issue: Backend server may not be running. Please start the backend server to use the chatbot.', 'assistant');
      // Still make sure input area is available even if connection fails
      ensureInputAreaVisible();
      // Also ensure input area is visible immediately when there's an error
      setTimeout(() => {
        const inputArea = document.getElementById('chatbot-input-area');
        if (inputArea) {
          inputArea.style.display = 'block';
        }
      }, 100);
    }
  }

  function ensureInputAreaVisible() {
    // This function ensures the input area is available for user interaction
    // even when there are connection issues
    const inputArea = document.getElementById('chatbot-input-area');
    inputArea.style.display = 'block';
    console.log('Input area is available for use');
  }

  async function sendMessage() {
    const inputElement = document.getElementById('chatbot-input');
    const message = inputElement.value.trim();

    if (!message || !currentSessionToken) return;

    // Add user message to UI
    addMessageToUI(message, 'user');
    inputElement.value = '';

    try {
      // Get any selected text if in selected text mode
      let selectedText = null;
      if (currentMode === 'selected_text') {
        const selection = window.getSelection();
        selectedText = selection.toString().trim();
        if (selectedText && selectedText.length > 2000) {
          selectedText = selectedText.substring(0, 2000);
        }
      }

      // Send message to backend
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: message,
          session_token: currentSessionToken,
          query_type: currentMode,
          selected_text: selectedText || null
        })
      });

      const data = await response.json();

      // Add assistant response to UI
      addMessageToUI(data.response, 'assistant');
    } catch (error) {
      console.error('Error sending message:', error);
      addMessageToUI('Sorry, I encountered an error. Please try again.', 'assistant');
    }
  }

  function addMessageToUI(content, role) {
    const messagesContainer = document.getElementById('chatbot-messages');
    const messageElement = document.createElement('div');
    messageElement.style.cssText = `
      margin-bottom: 15px;
      padding: 10px;
      border-radius: 8px;
      max-width: 80%;
      ${role === 'user' ? 'margin-left: auto; background: #dbeafe; text-align: right;' : 'margin-right: auto; background: white;'}
    `;
    messageElement.innerHTML = content;
    messagesContainer.appendChild(messageElement);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  function setActiveMode(mode) {
    currentMode = mode;

    // Update button styles
    const fullBookBtn = document.getElementById('full-book-mode');
    const selectedTextBtn = document.getElementById('selected-text-mode');

    if (mode === 'full_book') {
      fullBookBtn.style.background = '#4f46e5';
      fullBookBtn.style.color = 'white';
      selectedTextBtn.style.background = '#e5e7eb';
      selectedTextBtn.style.color = 'inherit';
    } else {
      selectedTextBtn.style.background = '#4f46e5';
      selectedTextBtn.style.color = 'white';
      fullBookBtn.style.background = '#e5e7eb';
      fullBookBtn.style.color = 'inherit';
    }
  }

  function toggleChatbot() {
    const container = document.getElementById(CONTAINER_ID);
    const inputArea = document.getElementById('chatbot-input-area');

    if (container.style.display === 'none' || !container.style.display || container.style.display === '') {
      container.style.display = 'block';
      // Ensure input area is visible when opening
      setTimeout(() => {
        inputArea.style.display = 'block';
      }, 100); // Small delay to ensure container is rendered first
    } else {
      container.style.display = 'none';
    }
  }

  // Initialize when DOM is loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', createChatbotUI);
  } else {
    createChatbotUI();
  }

})();