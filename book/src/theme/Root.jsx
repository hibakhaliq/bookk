import React from 'react';
import ChatbotIntegration from '../components/ChatbotIntegration/ChatbotIntegration';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotIntegration />
    </>
  );
}