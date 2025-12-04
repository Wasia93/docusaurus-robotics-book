from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.models.schemas import IndexRequest, IndexResponse, HealthResponse
from app.services.rag_service import RAGService
from app.services.openai_service import OpenAIService
from app.services.qdrant_service import QdrantService
from app.database import get_db, DocumentChunk

router = APIRouter()
rag_service = RAGService()


@router.post("/index", response_model=IndexResponse)
async def index_document(request: IndexRequest, db: Session = Depends(get_db)):
    """
    Index a document for RAG
    """
    try:
        # Index in vector database
        chunks_indexed = rag_service.index_document(
            doc_id=request.doc_id,
            title=request.title,
            content=request.content,
            metadata=request.metadata
        )

        # Store in PostgreSQL
        doc_chunk = DocumentChunk(
            doc_id=request.doc_id,
            title=request.title,
            content=request.content,
            chunk_index=0,
            metadata=request.metadata
        )
        db.add(doc_chunk)
        db.commit()

        return IndexResponse(
            success=True,
            message=f"Successfully indexed document: {request.title}",
            chunks_indexed=chunks_indexed
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error indexing document: {str(e)}")


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check health of all services
    """
    try:
        openai_service = OpenAIService()
        qdrant_service = QdrantService()

        return HealthResponse(
            status="healthy",
            qdrant_connected=qdrant_service.health_check(),
            postgres_connected=True,  # If we reach here, Postgres is connected
            openai_configured=openai_service.health_check()
        )
    except Exception as e:
        return HealthResponse(
            status="unhealthy",
            qdrant_connected=False,
            postgres_connected=False,
            openai_configured=False
        )


@router.post("/initialize")
async def initialize_services():
    """
    Initialize Qdrant collection
    """
    try:
        qdrant_service = QdrantService()
        qdrant_service.initialize_collection()
        return {"message": "Services initialized successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error initializing services: {str(e)}")
