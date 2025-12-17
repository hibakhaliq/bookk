// This file is loaded on every page by Docusaurus
// It sets up the chatbot configuration and loads the script

// Only run in browser environment
if (typeof window !== 'undefined' && typeof document !== 'undefined') {
  // Set the API URL to point to our backend
  window.CHATBOT_API_URL = 'http://127.0.0.1:9008/api/v1';

  // Load the chatbot script after the DOM is ready
  document.addEventListener('DOMContentLoaded', function() {
    // Check if the chatbot script is already loaded to prevent duplicates
    if (window.ChatbotLoaded) {
      return;
    }

    // Create script element for the chatbot
    const script = document.createElement('script');
    // Add timestamp to prevent caching
    script.src = '/js/chatbot-embed.js?v=' + Date.now(); // Served from the static directory
    script.async = true;

    script.onload = () => {
      console.log('Chatbot script loaded successfully');
      window.ChatbotLoaded = true;
    };

    script.onerror = () => {
      console.error('Failed to load chatbot script');
    };

    document.head.appendChild(script);
  });
}