from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any
from ..database.connection import get_db
from ..services.ingestion_service import IngestionService
from ..middleware.validation import validate_ingestion_request
from pydantic import BaseModel


class IngestRequest(BaseModel):
    book_id: str
    title: str
    content: str
    metadata: Dict[str, Any] = {}


class IngestResponse(BaseModel):
    status: str
    chunks_processed: int
    message: str
    postgres_ids: list
    qdrant_ids: list


router = APIRouter()
ingestion_service = IngestionService()


@router.post("/ingest", response_model=IngestResponse)
async def ingest_book(
    request: IngestRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Ingest book content for RAG
    Process and store book content for retrieval-augmented generation
    """
    try:
        # Validate request data
        validated_data = validate_ingestion_request(request.dict())

        result = await ingestion_service.ingest_book(
            book_id=validated_data["book_id"],
            title=validated_data["title"],
            content=validated_data["content"],
            metadata=validated_data.get("metadata", {})
        )
        return result
    except HTTPException:
        # Re-raise HTTP exceptions (validation errors)
        raise
    except Exception as e:
        # Log the full exception details for debugging
        import traceback
        error_details = traceback.format_exc()
        print(f"ERROR in ingest_book: {str(e)}")
        print(f"Full traceback: {error_details}")

        # Re-raise with the original error details to provide more information
        raise HTTPException(status_code=500, detail=f"Error ingesting book: {str(e)}")