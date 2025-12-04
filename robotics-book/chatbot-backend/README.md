# Robotics Book RAG Chatbot Backend

FastAPI backend with RAG (Retrieval-Augmented Generation) capabilities for the Physical AI & Humanoid Robotics course.

## Architecture

- **FastAPI**: REST API framework
- **OpenAI**: Embeddings and chat completions
- **Qdrant Cloud**: Vector database for semantic search
- **Neon Postgres**: Relational database for chat history
- **RAG Pipeline**: Retrieves relevant context before generating responses

## Setup

### 1. Install Dependencies

```bash
cd chatbot-backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
```

Required credentials:
- **OpenAI API Key**: Get from https://platform.openai.com/api-keys
- **Qdrant Cloud**:
  - Create free account at https://cloud.qdrant.io
  - Create a cluster and get URL + API Key
- **Neon Postgres**:
  - Create free account at https://neon.tech
  - Create a project and get connection string

### 3. Initialize Database

```bash
# The database tables will be created automatically on first run
python -m uvicorn app.main:app --reload
```

### 4. Initialize Qdrant Collection

```bash
curl -X POST http://localhost:8000/api/admin/initialize
```

### 5. Index Documentation

```bash
python scripts/index_docs.py
```

## API Endpoints

### Chat Endpoints

**POST /api/chat**
```json
{
  "message": "What is ROS 2?",
  "session_id": "optional-session-id",
  "selected_text": "optional selected text from book"
}
```

**GET /api/chat/history/{session_id}**
- Retrieve chat history for a session

### Admin Endpoints

**POST /api/admin/index**
```json
{
  "doc_id": "module1/lesson1",
  "title": "Introduction to ROS 2",
  "content": "ROS 2 is...",
  "metadata": {}
}
```

**GET /api/admin/health**
- Check health of all services

**POST /api/admin/initialize**
- Initialize Qdrant collection

## Running the API

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Access the API documentation at: http://localhost:8000/docs

## Deployment

### Option 1: Railway (Recommended)

1. Install Railway CLI:
```bash
npm install -g @railway/cli
```

2. Login and deploy:
```bash
railway login
railway init
railway up
```

3. Add environment variables in Railway dashboard

### Option 2: Render

1. Create a new Web Service
2. Connect your GitHub repository
3. Set build command: `pip install -r chatbot-backend/requirements.txt`
4. Set start command: `cd chatbot-backend && uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables

### Option 3: Vercel (with Serverless)

Create `vercel.json`:
```json
{
  "builds": [
    {
      "src": "chatbot-backend/app/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/(.*)",
      "dest": "chatbot-backend/app/main.py"
    }
  ]
}
```

## Testing

```bash
# Test health endpoint
curl http://localhost:8000/api/admin/health

# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}'
```

## Features

✅ RAG-powered responses using course content
✅ Selected text context support
✅ Chat history tracking
✅ Vector similarity search
✅ Automatic document chunking
✅ Multi-language support ready
✅ CORS configured for GitHub Pages
