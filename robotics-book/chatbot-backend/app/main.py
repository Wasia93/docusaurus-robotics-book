from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import settings
from app.database import init_db
from app.routers import chat, admin

app = FastAPI(
    title="Robotics Book RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics course",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database
init_db()

# Include routers
app.include_router(chat.router, prefix="/api", tags=["Chat"])
app.include_router(admin.router, prefix="/api/admin", tags=["Admin"])


@app.get("/")
async def root():
    return {
        "message": "Robotics Book RAG Chatbot API",
        "docs": "/docs",
        "health": "/api/admin/health"
    }


@app.on_event("startup")
async def startup_event():
    print("Starting Robotics Book RAG Chatbot API...")
    print(f"CORS Origins: {settings.cors_origins_list}")
