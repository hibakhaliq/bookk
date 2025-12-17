import { useState, useEffect } from 'react';

export const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');
  const [isSelecting, setIsSelecting] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text && text.length > 0 && text.length <= 2000) { // 2000 char limit as per requirements
        setSelectedText(text);
        setIsSelecting(true);
      } else {
        setSelectedText('');
        setIsSelecting(false);
      }
    };

    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Cleanup event listeners on unmount
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const clearSelection = () => {
    setSelectedText('');
    setIsSelecting(false);
    window.getSelection().removeAllRanges();
  };

  return {
    selectedText,
    isSelecting,
    clearSelection
  };
};