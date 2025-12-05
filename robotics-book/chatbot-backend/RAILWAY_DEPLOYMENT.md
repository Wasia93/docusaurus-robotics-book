# Railway Deployment Guide

## Backend is Ready for Deployment!

Your chatbot backend is fully configured and tested locally. All services are connected:
- ✅ OpenAI (GPT-4o-mini + text-embedding-3-small)
- ✅ Qdrant Cloud (vector database)
- ✅ Neon Postgres (chat history)

## Deploy to Railway (Web Dashboard - Easiest Method)

### Step 1: Create Railway Account
1. Go to https://railway.app/
2. Click "Start a New Project"
3. Sign in with GitHub

### Step 2: Deploy from GitHub
1. Click "Deploy from GitHub repo"
2. Select your repository: `Wasia93/docusaurus-robotics-book`
3. Railway will auto-detect it's a Python app
4. Click "Add variables" and copy all values from your local `.env` file

**IMPORTANT:** Copy the exact values from `robotics-book/chatbot-backend/.env`

Required environment variables:
- `OPENAI_API_KEY` - Your OpenAI API key
- `QDRANT_URL` - Your Qdrant Cloud URL
- `QDRANT_API_KEY` - Your Qdrant API key
- `QDRANT_COLLECTION_NAME` - Set to `robotics_book`
- `DATABASE_URL` - Your Neon Postgres connection string
- `API_HOST` - Set to `0.0.0.0`
- `API_PORT` - Set to `8000`
- `CORS_ORIGINS` - Set to `http://localhost:3000,https://wasia93.github.io`
- `EMBEDDING_MODEL` - Set to `text-embedding-3-small`
- `CHAT_MODEL` - Set to `gpt-4o-mini`

### Step 3: Configure Root Directory
Railway needs to know the backend is in a subdirectory:

1. Go to Settings tab
2. Under "Build & Deploy"
3. Set **Root Directory** to: `robotics-book/chatbot-backend`
4. Save changes

### Step 4: Deploy
1. Railway will automatically start building
2. Wait ~2-3 minutes for deployment
3. Once deployed, click "Generate Domain" to get your API URL
4. Test it: `https://your-app.up.railway.app/api/admin/health`

### Step 5: Initialize Qdrant Collection
After deployment, run this once:
```bash
curl -X POST https://your-app.up.railway.app/api/admin/initialize
```

## Alternative: Deploy via CLI

If you prefer command line:

```bash
cd robotics-book/chatbot-backend

# Login (opens browser)
railway login

# Create new project
railway init

# Set root directory
railway service settings --root robotics-book/chatbot-backend

# Add environment variables (one by one)
railway variables set OPENAI_API_KEY="your-key"
# ... (repeat for all variables above)

# Deploy
railway up

# Get URL
railway domain
```

## Next Steps

Once deployed:
1. Copy your Railway API URL (e.g., `https://your-app.up.railway.app`)
2. Update the chatbot frontend with this URL
3. Re-enable the chatbot component
4. Test the chatbot on your site!

## Troubleshooting

**Build fails?**
- Check that Root Directory is set to `robotics-book/chatbot-backend`
- Verify all environment variables are set

**Health check fails?**
- Ensure Qdrant URL and API key are correct
- Check Neon database connection string
- Verify OpenAI API key is active

**CORS errors?**
- Add your GitHub Pages URL to CORS_ORIGINS
- Format: `https://wasia93.github.io`
