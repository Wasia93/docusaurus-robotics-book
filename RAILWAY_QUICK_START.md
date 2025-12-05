# Quick Start: Deploy Backend to Railway

## Step-by-Step Instructions:

### 1. Go to Railway
Visit: https://railway.app/

### 2. Sign In
Click **"Login"** → Sign in with GitHub

### 3. Create New Project
Click **"New Project"** → **"Deploy from GitHub repo"**

### 4. Select Repository
Choose: **`Wasia93/docusaurus-robotics-book`**

### 5. CRITICAL: Configure Root Directory
⚠️ **THIS IS THE MOST IMPORTANT STEP!**

After Railway starts deploying:
1. Click on your service (should see it building)
2. Click **"Settings"** tab (left sidebar)
3. Scroll down to **"Root Directory"**
4. Click **"Edit"** or the pencil icon
5. Enter: `robotics-book/chatbot-backend`
6. Click **"Save"** or checkmark ✓

### 6. Add Environment Variables
Still in Settings:
1. Scroll to **"Variables"** section
2. Click **"New Variable"**
3. Add each variable from your `.env` file:

```
OPENAI_API_KEY = (paste your value)
QDRANT_URL = (paste your value)
QDRANT_API_KEY = (paste your value)
QDRANT_COLLECTION_NAME = robotics_book
DATABASE_URL = (paste your value)
API_HOST = 0.0.0.0
API_PORT = 8000
CORS_ORIGINS = http://localhost:3000,https://wasia93.github.io
EMBEDDING_MODEL = text-embedding-3-small
CHAT_MODEL = gpt-4o-mini
```

### 7. Redeploy
After adding all variables:
1. Go to **"Deployments"** tab
2. Click **"Redeploy"** on the latest deployment

### 8. Generate Public URL
1. Go to **"Settings"** tab
2. Scroll to **"Networking"** section
3. Click **"Generate Domain"**
4. Copy the URL (e.g., `https://your-app.up.railway.app`)

### 9. Test Your Backend
Open terminal and run:
```bash
curl https://your-app.up.railway.app/api/admin/health
```

Should return:
```json
{"status":"healthy","qdrant_connected":true,"postgres_connected":true,"openai_configured":true}
```

### 10. Initialize Qdrant (Run Once)
```bash
curl -X POST https://your-app.up.railway.app/api/admin/initialize
```

---

## ✅ Backend Deployed!

**Your Railway Backend URL:** `https://your-app.up.railway.app`

**Save this URL** - you'll need it for the frontend deployment!

---

## Troubleshooting:

**Build fails?**
- Check that Root Directory is set to `robotics-book/chatbot-backend`
- Verify all 10 environment variables are added

**Health check fails?**
- Check environment variables are correct
- Look at deployment logs for errors

**Can't access the URL?**
- Make sure you clicked "Generate Domain" in Settings → Networking
