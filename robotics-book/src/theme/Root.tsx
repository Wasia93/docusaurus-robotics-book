import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={<div />}>
        {() => {
          const RAGChatbot = require('@site/src/components/RAGChatbot').default;
          return <RAGChatbot apiUrl={process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000'} />;
        }}
      </BrowserOnly>
    </>
  );
}
