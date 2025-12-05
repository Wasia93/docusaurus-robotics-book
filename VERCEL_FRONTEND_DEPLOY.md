# Deploy Frontend to Vercel

## Prerequisites:
✅ Backend deployed to Railway
✅ Railway backend URL obtained (e.g., `https://your-app.up.railway.app`)

---

## Step 1: Update Chatbot Configuration

### 1.1 Open the config file:
`robotics-book/src/config/chatbot.config.ts`

### 1.2 Replace `BACKEND_URL` with your Railway URL:
```typescript
export const chatbotConfig = {
  BACKEND_URL: 'https://your-actual-railway-url.up.railway.app',
  ENABLED: true,
};
```

### 1.3 Save the file

### 1.4 Commit the change:
```bash
git add robotics-book/src/config/chatbot.config.ts
git commit -m "Configure chatbot with Railway backend URL"
git push
```

---

## Step 2: Enable Chatbot Component

The chatbot component is currently disabled. I'll help you enable it once the backend is deployed.

---

## Step 3: Deploy to Vercel

### 3.1 Go to Vercel
Visit: https://vercel.com/

### 3.2 Sign In
Click **"Login"** → Sign in with GitHub

### 3.3 Import Project
1. Click **"Add New..."** → **"Project"**
2. Find `Wasia93/docusaurus-robotics-book`
3. Click **"Import"**

### 3.4 Configure Project Settings

**Framework Preset:** Other

**Root Directory:**
- Click **"Edit"**
- Enter: `robotics-book`
- Click **"Save"**

**Build Settings:**
- **Build Command:** `npm run build`
- **Output Directory:** `build`
- **Install Command:** `npm install` (default)

### 3.5 Environment Variables (Optional)

Add if you want to override config:
```
BACKEND_URL = https://your-railway-backend.up.railway.app
```

### 3.6 Deploy
Click **"Deploy"** and wait ~3-5 minutes

---

## Step 4: Test Your Deployed Site

### 4.1 Visit Your Vercel URL
Vercel will give you a URL like: `https://docusaurus-robotics-book-xxx.vercel.app`

### 4.2 Test the Chatbot
1. Look for the chatbot icon (bottom right)
2. Click to open
3. Ask: "What is this course about?"
4. Verify it responds correctly

---

## ✅ Complete Deployment!

**Frontend URL:** `https://your-frontend.vercel.app`
**Backend URL:** `https://your-backend.up.railway.app`

---

## Alternative: Keep GitHub Pages

If you prefer to stay with GitHub Pages:
1. Update the chatbot config with Railway URL
2. Run: `npm run build`
3. Run: `npm run deploy`
4. Visit: `https://wasia93.github.io/docusaurus-robotics-book/`

---

## Troubleshooting:

**Chatbot not appearing?**
- Check that chatbot is enabled in Root.tsx
- Verify chatbot.config.ts has correct Railway URL

**Chatbot shows "Connection Error"?**
- Test Railway backend directly: `curl https://your-backend.up.railway.app/api/admin/health`
- Check CORS settings include your Vercel URL

**Build fails?**
- Check Root Directory is set to `robotics-book`
- Verify all dependencies installed: `npm install`
