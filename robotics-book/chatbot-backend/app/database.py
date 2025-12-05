from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
from app.config import settings

engine = create_engine(settings.database_url)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, index=True)
    user_message = Column(Text)
    bot_response = Column(Text)
    context_used = Column(JSON, nullable=True)
    selected_text = Column(Text, nullable=True)
    timestamp = Column(DateTime, default=datetime.utcnow)


class DocumentChunk(Base):
    __tablename__ = "document_chunks"

    id = Column(Integer, primary_key=True, index=True)
    doc_id = Column(String, index=True)
    title = Column(String)
    content = Column(Text)
    chunk_index = Column(Integer)
    doc_metadata = Column(JSON)
    created_at = Column(DateTime, default=datetime.utcnow)


def init_db():
    Base.metadata.create_all(bind=engine)


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
