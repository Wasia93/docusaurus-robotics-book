import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';
import { chatbotConfig } from '@site/src/config/chatbot.config';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <>
      {children}
      {chatbotConfig.ENABLED && <RAGChatbot />}
    </>
  );
}
