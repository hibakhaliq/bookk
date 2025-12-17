from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any, List
from pydantic import BaseModel
from ..database.connection import get_db
from ..services.main_rag_service import MainRAGService
from ..middleware.validation import validate_chat_request
import json


class ChatRequest(BaseModel):
    message: str
    session_token: str
    query_type: str = "full_book"  # "full_book" or "selected_text"
    selected_text: str = None
    book_id: str = None  # Optional: specify which book to query


class ChatResponse(BaseModel):
    response: str
    session_token: str
    sources: List[Dict[str, Any]]
    query_type: str


router = APIRouter()
main_rag_service = MainRAGService()


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Chat with the RAG system
    Send a message to the chatbot and receive a response based on book content
    """
    try:
        # Validate request data
        validated_data = {
            "message": request.message,
            "session_token": request.session_token,
            "query_type": request.query_type,
            "selected_text": request.selected_text,
            "book_id": request.book_id
        }
        validated_data = validate_chat_request(validated_data)

        # Process the query through the main RAG service
        result = await main_rag_service.process_query(
            db=db,
            message=validated_data["message"],
            session_token=validated_data["session_token"],
            query_type=validated_data["query_type"],
            selected_text=validated_data.get("selected_text"),
            book_id=validated_data.get("book_id")
        )

        return ChatResponse(
            response=result["response"],
            session_token=result["session_token"],
            sources=result["sources"],
            query_type=result["query_type"]
        )
    except HTTPException:
        # Re-raise HTTP exceptions (validation errors)
        raise
    except Exception as e:
        # Log the full error for debugging
        import traceback
        print(f"ERROR: Full traceback for chat request: {traceback.format_exc()}")
        print(f"ERROR: Exception in chat processing: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@router.post("/start_session")
async def start_session(
    user_id: str = None,
    metadata: Dict[str, Any] = {},
    db: AsyncSession = Depends(get_db)
):
    """
    Start a new chat session
    """
    try:
        session_token = await main_rag_service.start_new_session(db, user_id, metadata)
        return {"session_token": session_token, "status": "created"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating session: {str(e)}")