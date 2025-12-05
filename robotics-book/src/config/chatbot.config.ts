// Chatbot Configuration
// Update BACKEND_URL with your Railway deployment URL

export const chatbotConfig = {
  // IMPORTANT: Replace this with your Railway backend URL after deployment
  // Example: 'https://robotics-chatbot-backend.up.railway.app'
  BACKEND_URL: process.env.BACKEND_URL || 'http://localhost:8000',

  // Enable/disable chatbot
  ENABLED: true,
};
