# Embedding Guide for RAG Chatbot in Speckit-Published Books

This guide provides detailed instructions for embedding the RAG Chatbot in your Speckit-published books.

## Table of Contents
1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Embedding Methods](#embedding-methods)
4. [Configuration Options](#configuration-options)
5. [Testing the Embedding](#testing-the-embedding)
6. [Troubleshooting](#troubleshooting)

## Overview

The RAG Chatbot provides two main query modes:
- **Full Book Mode**: Answers questions based on the entire book content
- **Selected Text Mode**: Answers questions based only on user-selected text

The chatbot is designed to be embedded as a floating widget that appears on your book pages.

## Prerequisites

Before embedding, ensure:
1. Backend API is running and accessible (typically at http://localhost:8000 or your deployed URL)
2. Book content has been ingested into the system
3. Required API keys are configured in the backend

## Embedding Methods

### Method 1: Direct Script Inclusion (Recommended)

Add this script to your Speckit book HTML template or individual pages:

```html
<!-- Add this to the <head> section or before </body> -->
<script>
  // Configure the API URL for your deployed backend
  window.CHATBOT_API_URL = 'http://localhost:8000/api/v1'; // Change to your deployed URL
</script>
<script src="path/to/your/chatbot-embed.js"></script>
```

### Method 2: Using a CDN or Static Host

If you're hosting the embed script on a CDN or static file server:

```html
<script>
  window.CHATBOT_API_URL = 'https://your-deployed-backend.com/api/v1';
</script>
<script src="https://your-cdn.com/chatbot-embed.js"></script>
```

### Method 3: Dynamic Loading

For more control, you can dynamically load the script:

```javascript
// Dynamically load the chatbot
function loadRAGChatbot() {
  // Set API URL
  window.CHATBOT_API_URL = 'http://localhost:8000/api/v1';

  // Create and load script
  const script = document.createElement('script');
  script.src = 'path/to/your/chatbot-embed.js';
  script.async = true;
  document.head.appendChild(script);
}

// Load when page is ready
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', loadRAGChatbot);
} else {
  loadRAGChatbot();
}
```

## Complete HTML Example

Here's a complete example of how to embed the chatbot in an HTML page:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Your Book Page</title>

    <!-- Your book styles -->
    <style>
        body {
            font-family: Georgia, serif;
            line-height: 1.6;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
        }
    </style>
</head>
<body>
    <h1>Your Book Title</h1>
    <h2>Chapter 1: Introduction</h2>

    <p>This is the content of your book. Readers can select text and ask questions about it using the chatbot.</p>

    <p>More book content goes here...</p>

    <!-- Chatbot Configuration -->
    <script>
        // Configure the chatbot
        window.CHATBOT_API_URL = 'http://localhost:8000/api/v1'; // Replace with your backend URL
    </script>

    <!-- Chatbot Embed Script -->
    <script src="chatbot-embed.js"></script>
</body>
</html>
```

## Configuration Options

### API URL Configuration
```javascript
// Set the backend API URL
window.CHATBOT_API_URL = 'http://localhost:8000/api/v1'; // Development
// OR
window.CHATBOT_API_URL = 'https://yourdomain.com/api/v1'; // Production
```

### Custom Styling
The chatbot uses inline styles by default, but you can override them by adding CSS after the script:

```html
<style>
/* Customize the chatbot appearance */
#rag-chatbot-container {
    width: 450px !important;
    height: 650px !important;
}

#rag-chatbot-container button {
    border-radius: 12px !important;
}
</style>
```

## Integration with Speckit

### For Speckit Static Site Generation

1. **Place the script file** in your Speckit project's static assets folder (e.g., `static/js/chatbot-embed.js`)

2. **Add to layout template** in your Speckit configuration:
   - Add the script to your main HTML template
   - Configure the API URL appropriately

3. **Ensure CORS is configured** in your backend to allow requests from your book domain

### Example Speckit Integration

If using Speckit with custom HTML templates:

```html
<!-- In your Speckit template -->
{{#content}}
    <!-- Your book content rendered here -->
{{/content}}

<script>
    // Configure for your deployment
    window.CHATBOT_API_URL = '{{chatbot_api_url}}'; // Set via Speckit config
</script>
<script src="{{static_url}}/js/chatbot-embed.js"></script>
```

## Testing the Embedding

### 1. Basic Functionality Test
1. Load a page with the embedded chatbot
2. Click the "💬 Ask Book" button
3. Verify the chat interface appears
4. Test both "Full Book" and "Selected Text" modes

### 2. API Connectivity Test
1. Open browser developer tools (F12)
2. Go to Network tab
3. Send a test message
4. Verify API calls to your backend are successful

### 3. Selected Text Mode Test
1. Select some text on the page
2. Switch to "Selected Text" mode
3. Ask a question about the selected text
4. Verify the response is contextually relevant to the selection

## Troubleshooting

### Common Issues and Solutions

#### Issue: Chatbot doesn't appear
**Solution:**
- Verify the script is properly loaded (check browser console)
- Ensure `chatbot-embed.js` file exists at the specified path
- Check for JavaScript errors that might prevent initialization

#### Issue: API calls failing
**Solution:**
- Verify the `CHATBOT_API_URL` is correctly set
- Check that your backend is running and accessible
- Verify CORS settings allow requests from your domain
- Check browser console for network errors

#### Issue: Selected text mode not working
**Solution:**
- Ensure text selection is working on your page
- Verify the selection is not too long (limited to 2000 characters)
- Check that the selected text is being sent in API requests

#### Issue: Styling conflicts
**Solution:**
- The embed script uses inline styles that may conflict with your CSS
- Add custom CSS overrides after the script if needed
- Consider modifying the embed script for better style compatibility

### Debugging Steps

1. **Check browser console** for JavaScript errors
2. **Check network tab** for failed API requests
3. **Verify backend logs** for processing errors
4. **Test API endpoints directly** using curl or Postman

### Environment-Specific Considerations

#### Local Development
- Backend typically runs on `http://localhost:8000`
- May require CORS configuration for local file access

#### Production Deployment
- Update API URL to your deployed backend
- Ensure HTTPS if your book is served over HTTPS
- Consider API rate limiting and scaling

## Advanced Configuration

### Custom Initialization
You can customize the initialization process:

```javascript
// Wait for specific conditions before initializing
document.addEventListener('DOMContentLoaded', function() {
    // Custom initialization logic
    window.CHATBOT_API_URL = 'https://your-api.com/api/v1';

    // Load the embed script
    const script = document.createElement('script');
    script.src = '/path/to/chatbot-embed.js';
    document.head.appendChild(script);
});
```

### Event Handling
The embed script automatically initializes, but you can add custom event handling:

```javascript
// Listen for custom events (if added to the embed script)
window.addEventListener('chatbot-loaded', function() {
    console.log('Chatbot has been loaded');
});

// Or modify behavior after DOM is ready
document.addEventListener('DOMContentLoaded', function() {
    // Add custom behavior here
});
```

## Deployment Checklist

- [ ] Backend API is deployed and accessible
- [ ] Frontend embed script is available at the specified path
- [ ] API URL is configured correctly in the embedding context
- [ ] CORS is configured to allow requests from your book domain
- [ ] Book content has been ingested into the RAG system
- [ ] Embed script is included on all desired pages
- [ ] Both query modes have been tested
- [ ] Mobile responsiveness verified
- [ ] Error handling verified

## Support

For issues with embedding:
1. Check the browser console for errors
2. Verify network connectivity to your backend
3. Ensure all configuration values are correct
4. Test API endpoints independently
5. Review the backend logs for processing errors