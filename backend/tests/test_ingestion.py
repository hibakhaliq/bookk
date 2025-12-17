from fastapi.testclient import TestClient
from src.main import app
import pytest


client = TestClient(app)


def test_ingest_endpoint():
    # Test the ingest endpoint
    test_data = {
        "book_id": "test-book-123",
        "title": "Test Book",
        "content": "This is a test book content. It contains multiple sentences. This helps test the chunking functionality."
    }

    response = client.post("/api/v1/ingest", json=test_data)
    assert response.status_code == 200

    data = response.json()
    assert data["status"] == "success"
    assert data["chunks_processed"] > 0
    assert "Book 'Test Book' successfully ingested and indexed" in data["message"]
    assert len(data["postgres_ids"]) == data["chunks_processed"]
    assert len(data["qdrant_ids"]) == data["chunks_processed"]


def test_health_endpoint():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"