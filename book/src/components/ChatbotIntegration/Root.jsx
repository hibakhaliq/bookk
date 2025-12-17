import React from 'react';
import ChatbotIntegration from './ChatbotIntegration';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotIntegration />
    </>
  );
}