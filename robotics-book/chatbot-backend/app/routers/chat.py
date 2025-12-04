from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag_service import RAGService
from app.database import get_db, ChatHistory
from datetime import datetime
import uuid

router = APIRouter()
rag_service = RAGService()


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest, db: Session = Depends(get_db)):
    """
    Process a chat message with RAG
    """
    try:
        # Generate or use existing session ID
        session_id = request.session_id or str(uuid.uuid4())

        # Process query with RAG
        response, sources = rag_service.process_query(
            user_message=request.message,
            selected_text=request.selected_text
        )

        # Store in database
        chat_record = ChatHistory(
            session_id=session_id,
            user_message=request.message,
            bot_response=response,
            context_used=sources,
            selected_text=request.selected_text
        )
        db.add(chat_record)
        db.commit()

        return ChatResponse(
            response=response,
            session_id=session_id,
            sources=sources,
            timestamp=datetime.utcnow()
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat: {str(e)}")


@router.get("/chat/history/{session_id}")
async def get_chat_history(session_id: str, db: Session = Depends(get_db)):
    """
    Get chat history for a session
    """
    history = db.query(ChatHistory).filter(
        ChatHistory.session_id == session_id
    ).order_by(ChatHistory.timestamp).all()

    return [
        {
            "user_message": record.user_message,
            "bot_response": record.bot_response,
            "timestamp": record.timestamp,
            "selected_text": record.selected_text
        }
        for record in history
    ]
