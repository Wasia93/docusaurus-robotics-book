# RAG Chatbot Setup Guide

Complete guide to set up and deploy the RAG chatbot for the Physical AI & Humanoid Robotics course.

## Overview

This chatbot uses:
- **Backend**: FastAPI + Python
- **Vector DB**: Qdrant Cloud (Free Tier)
- **Database**: Neon Serverless Postgres (Free Tier)
- **AI**: OpenAI GPT-4o-mini + text-embedding-3-small
- **Frontend**: React component embedded in Docusaurus

## Step 1: Get API Keys & Services

### 1.1 OpenAI API Key
1. Go to https://platform.openai.com/api-keys
2. Create an account (if needed)
3. Click "Create new secret key"
4. Copy the key (starts with `sk-...`)
5. Add $5-10 credit to your account

### 1.2 Qdrant Cloud
1. Go to https://cloud.qdrant.io
2. Sign up for free account
3. Create a cluster (Free tier: 1GB storage)
4. Note your **Cluster URL** (looks like: `https://xxx-xxx.aws.cloud.qdrant.io`)
5. Go to "API Keys" → Create new key
6. Copy the **API Key**

### 1.3 Neon Serverless Postgres
1. Go to https://neon.tech
2. Sign up for free account (Free tier: 512MB storage)
3. Create a new project
4. Copy the **Connection String** (starts with `postgresql://...`)

## Step 2: Backend Setup

### 2.1 Install Dependencies

```bash
cd robotics-book/chatbot-backend
pip install -r requirements.txt
```

### 2.2 Configure Environment

Create `.env` file:

```bash
# OpenAI
OPENAI_API_KEY=sk-your-key-here

# Qdrant Cloud
QDRANT_URL=https://xxx-xxx.aws.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-key-here
QDRANT_COLLECTION_NAME=robotics_book

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb

# API Config
CORS_ORIGINS=http://localhost:3000,https://wasia93.github.io
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4o-mini
```

### 2.3 Initialize Services

```bash
# Start the API
uvicorn app.main:app --reload

# In another terminal, initialize Qdrant
curl -X POST http://localhost:8000/api/admin/initialize

# Index the documentation
python scripts/index_docs.py
```

### 2.4 Test the API

```bash
# Check health
curl http://localhost:8000/api/admin/health

# Test chat
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}'
```

## Step 3: Deploy Backend

### Option A: Railway (Recommended - Easiest)

1. Install Railway CLI:
```bash
npm install -g @railway/cli
```

2. Login and create project:
```bash
railway login
cd chatbot-backend
railway init
```

3. Add environment variables:
```bash
railway variables set OPENAI_API_KEY=sk-...
railway variables set QDRANT_URL=https://...
railway variables set QDRANT_API_KEY=...
railway variables set DATABASE_URL=postgresql://...
railway variables set CORS_ORIGINS=https://wasia93.github.io
```

4. Deploy:
```bash
railway up
```

5. Get your URL:
```bash
railway domain
```

### Option B: Render

1. Go to https://render.com
2. Create new Web Service
3. Connect GitHub repository
4. Configure:
   - **Build Command**: `pip install -r chatbot-backend/requirements.txt`
   - **Start Command**: `cd chatbot-backend && uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables in Render dashboard
6. Deploy

### Option C: Docker + Your Server

```bash
cd chatbot-backend

# Build image
docker build -t robotics-chatbot .

# Run container
docker run -d -p 8000:8000 \
  -e OPENAI_API_KEY=sk-... \
  -e QDRANT_URL=https://... \
  -e QDRANT_API_KEY=... \
  -e DATABASE_URL=postgresql://... \
  -e CORS_ORIGINS=https://wasia93.github.io \
  robotics-chatbot
```

## Step 4: Frontend Integration

### 4.1 Update Docusaurus Config

Edit `robotics-book/docusaurus.config.ts`:

```typescript
const config: Config = {
  // ... other config
  customFields: {
    chatbotApiUrl: process.env.CHATBOT_API_URL || 'https://your-api-url.com',
  },
}
```

### 4.2 Update Environment Variables

Create `.env` file in `robotics-book/`:

```bash
CHATBOT_API_URL=https://your-railway-app.up.railway.app
# or
CHATBOT_API_URL=https://your-render-app.onrender.com
```

### 4.3 Update GitHub Actions

Edit `.github/workflows/deploy.yml` to include the environment variable:

```yaml
- name: Build website
  env:
    CHATBOT_API_URL: ${{ secrets.CHATBOT_API_URL }}
  run: npm run build
```

Add `CHATBOT_API_URL` to GitHub Secrets:
1. Go to repository Settings → Secrets and variables → Actions
2. Add new secret: `CHATBOT_API_URL`

## Step 5: Index Your Documentation

After deploying, index all docs:

```bash
# From local machine pointing to deployed API
python scripts/index_docs.py
```

Or use the API endpoint:

```bash
curl -X POST https://your-api-url.com/api/admin/index \
  -H "Content-Type: application/json" \
  -d '{
    "doc_id": "intro",
    "title": "Introduction",
    "content": "Your content here...",
    "metadata": {}
  }'
```

## Step 6: Test Everything

1. Visit your deployed site: https://wasia93.github.io/docusaurus-robotics-book/
2. Click the floating chat button (💬) in bottom-right
3. Ask: "What is ROS 2?"
4. Select text on the page and ask a question about it

## Features

✅ **General Questions**: Ask anything about course content
✅ **Selected Text**: Highlight text and ask specific questions
✅ **Sources**: Shows which documents were used to answer
✅ **Chat History**: Maintains conversation context
✅ **Multi-language Ready**: Works with all translated versions

## Costs (Free Tier Limits)

- **Qdrant Cloud**: Free 1GB (enough for ~100k chunks)
- **Neon Postgres**: Free 512MB storage
- **OpenAI**: Pay-as-you-go (~$0.0001 per question)
- **Railway/Render**: Free tier available

Expected monthly cost with 1000 questions: **$0.10 - $0.50**

## Troubleshooting

### CORS Errors
- Update `CORS_ORIGINS` in backend `.env`
- Redeploy backend

### No Responses
- Check `http://your-api/api/admin/health`
- Verify all API keys are correct
- Check Railway/Render logs

### Selected Text Not Working
- Make sure you're selecting at least 10 characters
- The component listens for `mouseup` events

### Not Finding Relevant Content
- Re-run indexing script
- Check Qdrant collection has data
- Increase `top_k` parameter in RAG service

## Advanced Configuration

### Custom Chunking Strategy

Edit `app/services/rag_service.py`:

```python
chunks = self._chunk_text(content, chunk_size=1000, overlap=100)
```

### Different AI Model

Edit `.env`:

```bash
CHAT_MODEL=gpt-4-turbo
EMBEDDING_MODEL=text-embedding-3-large
```

### Add Authentication

Add to `app/main.py`:

```python
from fastapi.security import HTTPBearer

security = HTTPBearer()

@app.post("/api/chat")
async def chat(request: ChatRequest, token: str = Depends(security)):
    # Verify token
    ...
```

## Support

For issues:
1. Check logs in Railway/Render dashboard
2. Test health endpoint
3. Verify environment variables
4. Check GitHub issues

## Next Steps

- [ ] Add user authentication
- [ ] Add rate limiting
- [ ] Add usage analytics
- [ ] Support file uploads
- [ ] Add voice input/output
- [ ] Multi-modal support (images)
