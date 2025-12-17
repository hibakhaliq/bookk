#!/usr/bin/env python3
"""
Script to run the RAG Chatbot backend server with proper configuration
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend/src'))

from backend.src.main import app
import uvicorn

if __name__ == "__main__":
    print("Starting RAG Chatbot API server on port 9008...")
    print("API endpoints will be available at /api/v1/")
    uvicorn.run(
        "backend.src.main:app",
        host="0.0.0.0",
        port=9008,
        reload=True,
        reload_dirs=["backend/src"]
    )