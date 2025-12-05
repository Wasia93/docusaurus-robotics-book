# 🚀 Deployment Checklist

## Architecture:
- **Backend (FastAPI API)** → Railway
- **Frontend (Docusaurus Site)** → Vercel

---

## Phase 1: Deploy Backend to Railway ⏱️ 10 minutes

### ☐ Step 1: Deploy to Railway
📖 **Follow:** `RAILWAY_QUICK_START.md`

Quick steps:
1. Go to https://railway.app/ and sign in with GitHub
2. Create new project from `Wasia93/docusaurus-robotics-book`
3. ⚠️ **CRITICAL:** Set Root Directory to `robotics-book/chatbot-backend`
4. Add all 10 environment variables from `.env` file
5. Generate public domain

### ☐ Step 2: Test Backend
```bash
curl https://YOUR-RAILWAY-URL.up.railway.app/api/admin/health
```

Expected response:
```json
{"status":"healthy","qdrant_connected":true,"postgres_connected":true,"openai_configured":true}
```

### ☐ Step 3: Initialize Qdrant
```bash
curl -X POST https://YOUR-RAILWAY-URL.up.railway.app/api/admin/initialize
```

### ✅ Backend Complete!
**Save your Railway URL:** `_________________________________`

---

## Phase 2: Configure Frontend ⏱️ 2 minutes

### ☐ Step 4: Update Chatbot Config

Edit file: `robotics-book/src/config/chatbot.config.ts`

Replace:
```typescript
BACKEND_URL: 'http://localhost:8000',
```

With your Railway URL:
```typescript
BACKEND_URL: 'https://YOUR-RAILWAY-URL.up.railway.app',
```

### ☐ Step 5: Commit Changes
```bash
git add robotics-book/src/config/chatbot.config.ts
git commit -m "Update chatbot with Railway backend URL"
git push
```

---

## Phase 3: Deploy Frontend to Vercel ⏱️ 5 minutes

### ☐ Step 6: Deploy to Vercel
📖 **Follow:** `VERCEL_FRONTEND_DEPLOY.md`

Quick steps:
1. Go to https://vercel.com/ and sign in with GitHub
2. Import project `Wasia93/docusaurus-robotics-book`
3. ⚠️ **CRITICAL:** Set Root Directory to `robotics-book`
4. Set Build Command: `npm run build`
5. Set Output Directory: `build`
6. Click Deploy

### ☐ Step 7: Wait for Deployment
~3-5 minutes

---

## Phase 4: Enable Chatbot ⏱️ 2 minutes

### ☐ Step 8: Let me know!
Once you have your Railway URL, tell me and I'll:
1. Enable the chatbot component in the frontend
2. Configure it to use your Railway backend
3. Help you test end-to-end

---

## Final Testing

### ☐ Step 9: Test Complete Flow
1. Visit your Vercel URL
2. Look for chatbot icon (bottom right)
3. Ask: "What topics does this course cover?"
4. Verify intelligent response

---

## 🎉 Deployment Complete!

**Your URLs:**
- Frontend: `https://your-frontend.vercel.app`
- Backend: `https://your-backend.up.railway.app`
- Docs: `/docs` for API documentation

---

## Current Status:

- [ ] Backend deployed to Railway
- [ ] Railway URL obtained
- [ ] Chatbot config updated
- [ ] Frontend deployed to Vercel
- [ ] Chatbot enabled
- [ ] End-to-end testing complete

---

## Need Help?

- Railway issues? Check `RAILWAY_QUICK_START.md`
- Vercel issues? Check `VERCEL_FRONTEND_DEPLOY.md`
- Full plan? Check `DEPLOYMENT_PLAN.md`
