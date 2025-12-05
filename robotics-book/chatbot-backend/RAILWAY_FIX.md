# Railway Deployment Fix - Use UI Settings

Instead of using config files, configure directly in Railway UI:

## Step 1: Settings Tab

Go to your Railway service → **Settings** tab

### Build Settings:
- **Root Directory:** `robotics-book/chatbot-backend`
- **Builder:** `NIXPACKS` (default)
- **Install Command:** `pip install -r requirements.txt`
- **Build Command:** Leave empty

### Deploy Settings:
- **Start Command:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

### Custom Start Command (Alternative):
If the above doesn't work, try:
```
python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

## Step 2: Environment Variables

Make sure ALL 10 variables from `.env` are set:

1. OPENAI_API_KEY
2. QDRANT_URL
3. QDRANT_API_KEY
4. QDRANT_COLLECTION_NAME
5. DATABASE_URL
6. API_HOST
7. API_PORT
8. CORS_ORIGINS
9. EMBEDDING_MODEL
10. CHAT_MODEL

## Step 3: Redeploy

After saving settings:
1. Go to **Deployments** tab
2. Click **"Redeploy"**

## If Still Failing:

Try creating a **new Railway service**:
1. Create New → Empty Service
2. Connect to GitHub repo: `Wasia93/docusaurus-robotics-book`
3. Set root directory: `robotics-book/chatbot-backend`
4. Add environment variables
5. Set start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
6. Deploy

## Alternative: Use Python Build Pack

In Railway Settings:
- Nixpack Plan: Select **Python**
- Python Version: **3.11**
