@echo off
REM Setup batch file for RAG Chatbot on Windows

echo ===========================================
echo RAG Chatbot Setup Script (Windows)
echo ===========================================

REM Check if running in the correct directory
if not exist "README.md" (
    echo Error: This script must be run from the project root directory.
    echo Make sure you're in the directory containing 'backend' and 'frontend' folders.
    pause
    exit /b 1
)

if not exist "backend" (
    echo Error: Backend directory not found.
    pause
    exit /b 1
)

if not exist "frontend" (
    echo Error: Frontend directory not found.
    pause
    exit /b 1
)

echo Project directory structure verified.

REM Function to setup backend
:setup_backend
echo Setting up backend...
cd backend

REM Check if virtual environment exists, create if not
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
echo Activating virtual environment...
call venv\Scripts\activate.bat

REM Install dependencies
echo Installing backend dependencies...
pip install -r requirements.txt

REM Create .env file if it doesn't exist
if not exist ".env" (
    echo Creating .env file...
    if exist ".env.example" (
        copy .env.example .env
        echo Please update the .env file with your API keys and connection strings.
    ) else (
        echo Warning: .env.example not found. Please create .env file manually.
    )
)

echo Backend setup completed.
cd ..

goto :menu

REM Function to setup frontend
:setup_frontend
echo Setting up frontend...
cd frontend

REM Install dependencies
echo Installing frontend dependencies...
npm install

echo Frontend setup completed.
cd ..

goto :menu

REM Function to show API info
:show_api_info
echo.
echo ===========================================
echo API ENDPOINTS AND USAGE
echo ===========================================
echo.
echo BACKEND API (running on http://localhost:8000)
echo.
echo - GET / - API root endpoint
echo - GET /health - Health check
echo - GET /metrics - Application metrics
echo - GET /metrics/prometheus - Prometheus metrics format
echo - POST /api/v1/ingest - Ingest book content
echo - POST /api/v1/start_session - Start new chat session
echo - POST /api/v1/chat - Chat endpoint
echo.
echo EXAMPLE USAGE:
echo.
echo 1. Ingest a Book:
echo curl -X POST "http://localhost:8000/api/v1/ingest" ^
  -H "Content-Type: application/json" ^
  -d "{^
    \"book_id\": \"my-book-1\",^
    \"title\": \"My Book Title\",^
    \"content\": \"Full content of the book goes here...\",^
    \"metadata\": {}^
  }"
echo.
echo 2. Start a New Session:
echo curl -X POST "http://localhost:8000/api/v1/start_session" ^
  -H "Content-Type: application/json"
echo.
echo 3. Chat with the Bot:
echo curl -X POST "http://localhost:8000/api/v1/chat" ^
  -H "Content-Type: application/json" ^
  -d "{^
    \"message\": \"What is this book about?\",^
    \"session_token\": \"session-token-from-step-2\",^
    \"query_type\": \"full_book\",^
    \"book_id\": \"my-book-1\"^
  }"
echo.
echo 4. Chat with Selected Text:
echo curl -X POST "http://localhost:8000/api/v1/chat" ^
  -H "Content-Type: application/json" ^
  -d "{^
    \"message\": \"What does this text mean?\",^
    \"session_token\": \"session-token-from-step-2\",^
    \"query_type\": \"selected_text\",^
    \"selected_text\": \"The specific text that was selected by the user\",^
    \"book_id\": \"my-book-1\"^
  }"
echo.
echo FRONTEND INTEGRATION:
echo.
echo To embed the chatbot in your Speckit-published book:
echo.
echo 1. Include the embed script in your HTML:
echo ^<script src="/path/to/chatbot-embed.js"^>^</script^>
echo ^<div id="rag-chatbot-container"^>^</div^>
echo.
echo 2. Initialize the chatbot:
echo window.RAGChatbot.init({^
  backendUrl: 'http://localhost:8000',^
  containerId: 'rag-chatbot-container'^
});
echo.
goto :menu

:menu
echo.
echo Choose an option:
echo 1) Setup both backend and frontend
echo 2) Setup only backend
echo 3) Setup only frontend
echo 4) Show API endpoints and usage
echo 5) Exit
set /p choice="Enter your choice [1-5]: "

if "%choice%"=="1" (
    call :setup_backend
    call :setup_frontend
    goto :menu
) else if "%choice%"=="2" (
    call :setup_backend
    goto :menu
) else if "%choice%"=="3" (
    call :setup_frontend
    goto :menu
) else if "%choice%"=="4" (
    call :show_api_info
    goto :menu
) else if "%choice%"=="5" (
    echo Exiting setup script.
    pause
    exit /b 0
) else (
    echo Invalid option. Please try again.
    goto :menu
)