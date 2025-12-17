from fastapi import Request, HTTPException
from typing import Dict, Any
import re


def sanitize_input(text: str, max_length: int = 1000000) -> str:
    """
    Sanitize user input to prevent injection attacks
    For book content, be more permissive while still blocking actual dangerous content
    Increased default max_length to accommodate book content
    """
    if not text:
        return text

    # Limit length
    if len(text) > max_length:
        raise HTTPException(status_code=400, detail=f"Input exceeds maximum length of {max_length} characters")

    # Remove potentially dangerous characters/sequences (basic protection)
    # In a real application, use a proper HTML sanitizer for rich text
    sanitized = text

    # Check for potential SQL injection patterns (basic check)
    # Updated to be more specific and less likely to flag markdown content
    sql_patterns = [
        r"(?i)(union\s+select\b)",
        r"(?i)(drop\s+(table|database|index)\s+\w+)",
        r"(?i)(delete\s+from\s+\w+)",
        r"(?i)(insert\s+into\s+\w+)",
        r"(?i)(update\s+\w+\s+set\b)",
        r"(?i)(exec\s*\()",
        r"(?i)(<script\b)",  # Only block script tags, not script as a word
        r"(?i)(javascript:)",
        r"(?i)(vbscript:)",
        r"(?i)(on\w+\s*=)"  # Event handlers like onclick, onload, etc.
    ]

    for pattern in sql_patterns:
        if re.search(pattern, sanitized, re.IGNORECASE):
            raise HTTPException(status_code=400, detail="Input contains potentially dangerous content")

    return sanitized


def validate_book_id(book_id: str) -> str:
    """
    Validate book ID format
    """
    if not book_id:
        raise HTTPException(status_code=400, detail="Book ID is required")

    # Allow alphanumeric characters, hyphens, and underscores
    if not re.match(r'^[a-zA-Z0-9_-]+$', book_id):
        raise HTTPException(status_code=400, detail="Invalid book ID format")

    return book_id


def validate_session_token(session_token: str) -> str:
    """
    Validate session token format
    """
    if not session_token:
        raise HTTPException(status_code=400, detail="Session token is required")

    # Basic UUID format validation (simplified)
    uuid_pattern = r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$'
    if not re.match(uuid_pattern, session_token, re.IGNORECASE):
        raise HTTPException(status_code=400, detail="Invalid session token format")

    return session_token


def validate_query_type(query_type: str) -> str:
    """
    Validate query type
    """
    valid_types = ["full_book", "selected_text"]
    if query_type not in valid_types:
        raise HTTPException(status_code=400, detail=f"Invalid query type. Must be one of {valid_types}")

    return query_type


def validate_selected_text(selected_text: str) -> str:
    """
    Validate selected text length
    """
    if selected_text and len(selected_text) > 2000:  # As per requirements
        raise HTTPException(status_code=400, detail="Selected text exceeds maximum length of 2000 characters")

    return selected_text


def validate_ingestion_request(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate ingestion request data
    """
    required_fields = ["book_id", "title", "content"]

    for field in required_fields:
        if field not in data or not data[field]:
            raise HTTPException(status_code=400, detail=f"Missing required field: {field}")

    # Validate and sanitize each field
    data["book_id"] = validate_book_id(data["book_id"])
    data["title"] = sanitize_input(data["title"], max_length=500)  # Keep conservative limit for titles
    data["content"] = sanitize_input(data["content"], max_length=1000000)  # 1M max for book content to handle large markdown files

    # Validate metadata if present
    if "metadata" in data and data["metadata"]:
        if not isinstance(data["metadata"], dict):
            raise HTTPException(status_code=400, detail="Metadata must be a valid JSON object")

    return data


def validate_chat_request(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate chat request data
    """
    required_fields = ["message", "session_token"]

    for field in required_fields:
        if field not in data or not data[field]:
            raise HTTPException(status_code=400, detail=f"Missing required field: {field}")

    # Validate and sanitize each field
    data["message"] = sanitize_input(data["message"], max_length=2000)  # Keep conservative limit for chat messages
    data["session_token"] = validate_session_token(data["session_token"])

    # Validate optional fields
    if "query_type" in data and data["query_type"]:
        data["query_type"] = validate_query_type(data["query_type"])
    else:
        data["query_type"] = "full_book"  # Default value

    if "selected_text" in data and data["selected_text"]:
        data["selected_text"] = validate_selected_text(data["selected_text"])

    if "book_id" in data and data["book_id"]:
        data["book_id"] = validate_book_id(data["book_id"])

    return data