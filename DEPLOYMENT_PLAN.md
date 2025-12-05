# Deployment Plan: Backend on Railway, Frontend on Vercel

## Architecture:
- **Backend (FastAPI)** → Railway → `https://your-backend.up.railway.app`
- **Frontend (Docusaurus)** → Vercel → `https://your-frontend.vercel.app`

## Step 1: Deploy Backend to Railway

### 1.1 Create Railway Project
1. Go to https://railway.app/
2. Sign in with GitHub
3. Click **"New Project"**
4. Select **"Deploy from GitHub repo"**
5. Choose: `Wasia93/docusaurus-robotics-book`

### 1.2 Configure Railway Service
**IMPORTANT Settings:**

- **Root Directory:** `robotics-book/chatbot-backend`
- **Start Command:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
- **Build Command:** (leave empty, Railway auto-detects)

### 1.3 Add Environment Variables
Add ALL 10 variables from `robotics-book/chatbot-backend/.env`:

```
OPENAI_API_KEY=<your-value>
QDRANT_URL=<your-value>
QDRANT_API_KEY=<your-value>
QDRANT_COLLECTION_NAME=robotics_book
DATABASE_URL=<your-value>
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000,https://wasia93.github.io
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4o-mini
```

### 1.4 Generate Domain
1. After deployment completes
2. Click **"Settings"** → **"Domains"**
3. Click **"Generate Domain"**
4. Copy the URL (e.g., `https://robotics-chatbot-backend.up.railway.app`)

### 1.5 Test Backend
```bash
curl https://your-backend.up.railway.app/api/admin/health
```

Should return: `{"status":"healthy",...}`

---

## Step 2: Deploy Frontend to Vercel

### 2.1 Update Chatbot with Railway URL
We'll configure the frontend chatbot to connect to your Railway backend.

### 2.2 Create Vercel Project
1. Go to https://vercel.com/
2. Sign in with GitHub
3. Click **"Add New..."** → **"Project"**
4. Import: `Wasia93/docusaurus-robotics-book`

### 2.3 Configure Vercel
**IMPORTANT Settings:**

- **Framework Preset:** Other
- **Root Directory:** `robotics-book`
- **Build Command:** `npm run build`
- **Output Directory:** `build`
- **Install Command:** `npm install`

### 2.4 Add Environment Variable (if needed)
```
BACKEND_URL=https://your-backend.up.railway.app
```

### 2.5 Deploy
Click **"Deploy"** and wait ~2-3 minutes.

---

## Step 3: Final Testing

### Test Full Flow:
1. Visit your Vercel frontend URL
2. Open the chatbot
3. Ask a question about the robotics course
4. Verify it responds correctly

---

## URLs Summary:
- **Backend API:** `https://your-backend.up.railway.app`
- **Frontend Site:** `https://your-frontend.vercel.app`
- **GitHub Pages (alternative):** `https://wasia93.github.io/docusaurus-robotics-book/`

---

## Next Steps After This Guide:
1. Railway backend deployed ✅
2. Frontend configured with Railway URL ✅
3. Vercel frontend deployed ✅
4. Test chatbot end-to-end ✅
