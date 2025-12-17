from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.ingestion_router import router as ingestion_router
from .api.chat_router import router as chat_router
from .config import settings
from .database.connection import engine
from .models.chunk import Base as ChunkBase
from .models.chat_models import Base as ChatBase
from .utils.metrics import metrics_collector
import asyncio


# Create tables in the database
async def create_tables():
    # Combine all model bases
    from sqlalchemy import MetaData
    metadata = MetaData()

    # Create all tables from all models
    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(ChunkBase.metadata.create_all)
        await conn.run_sync(ChatBase.metadata.create_all)


# Initialize the FastAPI app
app = FastAPI(
    title=settings.APP_NAME,
    version="1.0.0",
    description="RAG Chatbot API for book content"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # In production, specify exact origins instead of ["*"]
)

# Include the routers
app.include_router(
    ingestion_router,
    prefix=settings.API_V1_STR,
    tags=["ingestion"]
)

app.include_router(
    chat_router,
    prefix=settings.API_V1_STR,
    tags=["chat"]
)


@app.on_event("startup")
async def startup_event():
    await create_tables()


@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API", "status": "running"}


@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "RAG Chatbot API"}


@app.get("/metrics")
def get_metrics():
    """Get application metrics"""
    return metrics_collector.get_metrics_summary()


@app.get("/metrics/prometheus")
async def get_prometheus_metrics():
    """Get metrics in Prometheus format"""
    return await metrics_collector.export_prometheus_format()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=9008)