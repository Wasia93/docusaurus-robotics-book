# Vercel Deployment Guide

## ✅ Backend Ready for Vercel!

Your chatbot backend is configured for Vercel serverless deployment.

## Option 1: Deploy via Vercel Dashboard (Easiest)

### Step 1: Create Vercel Account
1. Go to https://vercel.com/
2. Click "Sign Up" and sign in with GitHub

### Step 2: Import Project
1. Click **"Add New..."** → **"Project"**
2. Click **"Import Git Repository"**
3. Select: `Wasia93/docusaurus-robotics-book`
4. Click **"Import"**

### Step 3: Configure Project
**IMPORTANT:** Vercel will try to deploy the entire repo. Configure it for the backend only:

- **Framework Preset:** Other
- **Root Directory:** Click "Edit" → Enter `robotics-book/chatbot-backend`
- **Build Command:** Leave empty
- **Output Directory:** Leave empty
- **Install Command:** `pip install -r requirements.txt`

### Step 4: Add Environment Variables
Click **"Environment Variables"** and add ALL 10 variables from your `.env` file:

```
OPENAI_API_KEY
QDRANT_URL
QDRANT_API_KEY
QDRANT_COLLECTION_NAME
DATABASE_URL
API_HOST
API_PORT
CORS_ORIGINS
EMBEDDING_MODEL
CHAT_MODEL
```

**IMPORTANT:** Copy exact values from `robotics-book/chatbot-backend/.env`

### Step 5: Deploy
1. Click **"Deploy"**
2. Wait 2-3 minutes for deployment
3. Vercel will give you a URL like: `https://your-project.vercel.app`

### Step 6: Test
Test your deployed API:
```bash
curl https://your-project.vercel.app/api/admin/health
```

Should return:
```json
{"status":"healthy","qdrant_connected":true,"postgres_connected":true,"openai_configured":true}
```

### Step 7: Initialize Qdrant
Run once after deployment:
```bash
curl -X POST https://your-project.vercel.app/api/admin/initialize
```

---

## Option 2: Deploy via Vercel CLI

### Install Vercel CLI
```bash
npm install -g vercel
```

### Deploy
```bash
cd robotics-book/chatbot-backend
vercel login
vercel
```

Follow prompts:
- Set up and deploy? **Y**
- Which scope? Select your account
- Link to existing project? **N**
- Project name? **robotics-chatbot-backend**
- Directory? **./robotics-book/chatbot-backend**
- Override settings? **N**

### Add Environment Variables
```bash
vercel env add OPENAI_API_KEY
# Enter your OpenAI API key when prompted

vercel env add QDRANT_URL
# Enter your Qdrant URL when prompted

# Repeat for all 10 variables
```

### Deploy to Production
```bash
vercel --prod
```

---

## Troubleshooting

### Build Fails?
- Verify Root Directory is: `robotics-book/chatbot-backend`
- Check all environment variables are set
- Ensure requirements.txt is in the root directory

### API Returns 404?
- Check URL format: `https://your-project.vercel.app/api/admin/health`
- Routes should include `/api` prefix

### CORS Errors?
Update `CORS_ORIGINS` environment variable:
```
http://localhost:3000,https://wasia93.github.io
```

---

## Next Steps

Once deployed successfully:
1. Copy your Vercel URL (e.g., `https://your-project.vercel.app`)
2. Update chatbot frontend with this URL
3. Enable chatbot on GitHub Pages
4. Test the live chatbot!

## Notes

- Vercel free tier includes:
  - 100 GB bandwidth/month
  - 100 hours serverless execution/month
  - Automatic HTTPS
  - Auto-deploy on git push
