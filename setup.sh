#!/bin/bash
# Setup script for RAG Chatbot

echo "==========================================="
echo "RAG Chatbot Setup Script"
echo "==========================================="

# Check if running in the correct directory
if [ ! -f "README.md" ] || [ ! -d "backend" ] || [ ! -d "frontend" ]; then
    echo "Error: This script must be run from the project root directory."
    echo "Make sure you're in the directory containing 'backend' and 'frontend' folders."
    exit 1
fi

echo "Project directory structure verified."

# Function to setup backend
setup_backend() {
    echo "Setting up backend..."
    cd backend

    # Check if virtual environment exists, create if not
    if [ ! -d "venv" ]; then
        echo "Creating virtual environment..."
        python -m venv venv
    fi

    # Activate virtual environment
    echo "Activating virtual environment..."
    source venv/bin/activate

    # Install dependencies
    echo "Installing backend dependencies..."
    pip install -r requirements.txt

    # Create .env file if it doesn't exist
    if [ ! -f ".env" ]; then
        echo "Creating .env file..."
        cp .env.example .env
        echo "Please update the .env file with your API keys and connection strings."
    fi

    echo "Backend setup completed."
    cd ..
}

# Function to setup frontend
setup_frontend() {
    echo "Setting up frontend..."
    cd frontend

    # Install dependencies
    echo "Installing frontend dependencies..."
    npm install

    echo "Frontend setup completed."
    cd ..
}

# Function to start backend
start_backend() {
    echo "Starting backend server..."
    cd backend
    source venv/bin/activate
    python -m src.main &
    BACKEND_PID=$!
    cd ..

    echo "Backend server started with PID: $BACKEND_PID"
    echo "Backend API is available at: http://localhost:8000"
}

# Function to start frontend
start_frontend() {
    echo "Starting frontend server..."
    cd frontend
    npm start &
    FRONTEND_PID=$!
    cd ..

    echo "Frontend server started with PID: $FRONTEND_PID"
    echo "Frontend is available at: http://localhost:3000"
}

# Main menu
while true; do
    echo ""
    echo "Choose an option:"
    echo "1) Setup both backend and frontend"
    echo "2) Setup only backend"
    echo "3) Setup only frontend"
    echo "4) Start both servers"
    echo "5) Start only backend"
    echo "6) Start only frontend"
    echo "7) Show API endpoints and usage"
    echo "8) Exit"
    echo -n "Enter your choice [1-8]: "
    read choice

    case $choice in
        1)
            setup_backend
            setup_frontend
            ;;
        2)
            setup_backend
            ;;
        3)
            setup_frontend
            ;;
        4)
            start_backend
            start_frontend
            echo "Both servers started. Press Ctrl+C to stop."
            wait
            ;;
        5)
            start_backend
            echo "Backend started. Press Ctrl+C to stop."
            wait
            ;;
        6)
            start_frontend
            echo "Frontend started. Press Ctrl+C to stop."
            wait
            ;;
        7)
            echo ""
            echo "==========================================="
            echo "API ENDPOINTS AND USAGE"
            echo "==========================================="
            echo ""
            echo "BACKEND API (running on http://localhost:8000)"
            echo ""
            echo "- GET / - API root endpoint"
            echo "- GET /health - Health check"
            echo "- GET /metrics - Application metrics"
            echo "- GET /metrics/prometheus - Prometheus metrics format"
            echo "- POST /api/v1/ingest - Ingest book content"
            echo "- POST /api/v1/start_session - Start new chat session"
            echo "- POST /api/v1/chat - Chat endpoint"
            echo ""
            echo "EXAMPLE USAGE:"
            echo ""
            echo "1. Ingest a Book:"
            echo "curl -X POST \"http://localhost:8000/api/v1/ingest\" \\"
            echo "  -H \"Content-Type: application/json\" \\"
            echo "  -d '{"
            echo "    \"book_id\": \"my-book-1\","
            echo "    \"title\": \"My Book Title\","
            echo "    \"content\": \"Full content of the book goes here...\","
            echo "    \"metadata\": {}"
            echo "  }'"
            echo ""
            echo "2. Start a New Session:"
            echo "curl -X POST \"http://localhost:8000/api/v1/start_session\" \\"
            echo "  -H \"Content-Type: application/json\""
            echo ""
            echo "3. Chat with the Bot:"
            echo "curl -X POST \"http://localhost:8000/api/v1/chat\" \\"
            echo "  -H \"Content-Type: application/json\" \\"
            echo "  -d '{"
            echo "    \"message\": \"What is this book about?\","
            echo "    \"session_token\": \"session-token-from-step-2\","
            echo "    \"query_type\": \"full_book\","
            echo "    \"book_id\": \"my-book-1\""
            echo "  }'"
            echo ""
            echo "4. Chat with Selected Text:"
            echo "curl -X POST \"http://localhost:8000/api/v1/chat\" \\"
            echo "  -H \"Content-Type: application/json\" \\"
            echo "  -d '{"
            echo "    \"message\": \"What does this text mean?\","
            echo "    \"session_token\": \"session-token-from-step-2\","
            echo "    \"query_type\": \"selected_text\","
            echo "    \"selected_text\": \"The specific text that was selected by the user\","
            echo "    \"book_id\": \"my-book-1\""
            echo "  }'"
            echo ""
            echo "FRONTEND INTEGRATION:"
            echo ""
            echo "To embed the chatbot in your Speckit-published book:"
            echo ""
            echo "1. Include the embed script in your HTML:"
            echo "<script src=\"/path/to/chatbot-embed.js\"></script>"
            echo "<div id=\"rag-chatbot-container\"></div>"
            echo ""
            echo "2. Initialize the chatbot:"
            echo "window.RAGChatbot.init({"
            echo "  backendUrl: 'http://localhost:8000',"
            echo "  containerId: 'rag-chatbot-container'"
            echo "});"
            echo ""
            ;;
        8)
            echo "Exiting setup script."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done