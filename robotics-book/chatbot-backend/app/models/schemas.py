from pydantic import BaseModel
from typing import Optional, List, Dict
from datetime import datetime


class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: List[Dict[str, str]]
    timestamp: datetime


class HealthResponse(BaseModel):
    status: str
    qdrant_connected: bool
    postgres_connected: bool
    openai_configured: bool


class IndexRequest(BaseModel):
    doc_id: str
    title: str
    content: str
    metadata: Optional[Dict] = {}


class IndexResponse(BaseModel):
    success: bool
    message: str
    chunks_indexed: int
