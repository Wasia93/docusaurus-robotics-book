import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <>
      {children}
      <RAGChatbot apiUrl={process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000'} />
    </>
  );
}
