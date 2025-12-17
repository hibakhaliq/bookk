from fastapi.testclient import TestClient
from src.main import app
import pytest


client = TestClient(app)


def test_ingestion_and_chat_integration():
    """
    Test the full integration flow: ingest a book, then ask questions about it
    """
    # Test data
    test_book = {
        "book_id": "integration-test-book",
        "title": "Integration Test Book",
        "content": "This is a test book about artificial intelligence. AI is a wonderful field that combines computer science and cognitive science. Machine learning is a subset of AI that focuses on algorithms that can learn from data."
    }

    # Step 1: Ingest the book
    ingest_response = client.post("/api/v1/ingest", json=test_book)
    assert ingest_response.status_code == 200

    ingest_data = ingest_response.json()
    assert ingest_data["status"] == "success"
    assert ingest_data["chunks_processed"] > 0

    # Step 2: Start a new session
    session_response = client.post("/api/v1/start_session")
    assert session_response.status_code == 200

    session_data = session_response.json()
    session_token = session_data["session_token"]
    assert session_token

    # Step 3: Ask a question about the book
    chat_request = {
        "message": "What is this book about?",
        "session_token": session_token,
        "query_type": "full_book"
    }

    chat_response = client.post("/api/v1/chat", json=chat_request)
    assert chat_response.status_code == 200

    chat_data = chat_response.json()
    assert "response" in chat_data
    assert len(chat_data["response"]) > 0
    assert chat_data["session_token"] == session_token
    assert chat_data["query_type"] == "full_book"
    assert isinstance(chat_data["sources"], list)

    # Step 4: Ask a follow-up question to test conversation history
    followup_request = {
        "message": "Can you tell me more about machine learning?",
        "session_token": session_token,
        "query_type": "full_book"
    }

    followup_response = client.post("/api/v1/chat", json=followup_request)
    assert followup_response.status_code == 200

    followup_data = followup_response.json()
    assert "response" in followup_data
    assert len(followup_data["response"]) > 0

    print("Integration test passed successfully!")


def test_selected_text_query():
    """
    Test the selected text query functionality
    """
    # Test data
    test_book = {
        "book_id": "selected-text-test-book",
        "title": "Selected Text Test Book",
        "content": "This is a test book about artificial intelligence. AI is a wonderful field that combines computer science and cognitive science. Machine learning is a subset of AI that focuses on algorithms that can learn from data."
    }

    # Ingest the book
    ingest_response = client.post("/api/v1/ingest", json=test_book)
    assert ingest_response.status_code == 200

    # Start a new session
    session_response = client.post("/api/v1/start_session")
    assert session_response.status_code == 200

    session_data = session_response.json()
    session_token = session_data["session_token"]

    # Ask a question with selected text
    selected_text = "Machine learning is a subset of AI that focuses on algorithms that can learn from data."
    chat_request = {
        "message": "What does this text say about machine learning?",
        "session_token": session_token,
        "query_type": "selected_text",
        "selected_text": selected_text
    }

    chat_response = client.post("/api/v1/chat", json=chat_request)
    assert chat_response.status_code == 200

    chat_data = chat_response.json()
    assert "response" in chat_data
    assert len(chat_data["response"]) > 0
    assert chat_data["query_type"] == "selected_text"

    print("Selected text query test passed successfully!")


if __name__ == "__main__":
    test_ingestion_and_chat_integration()
    test_selected_text_query()
    print("All integration tests passed!")