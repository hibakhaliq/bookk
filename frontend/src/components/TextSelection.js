import React from 'react';
import { useTextSelection } from '../hooks/useTextSelection';

const TextSelection = ({ onTextSelected }) => {
  const { selectedText, isSelecting, clearSelection } = useTextSelection();

  // Call the parent callback when text selection changes
  React.useEffect(() => {
    if (onTextSelected) {
      onTextSelected(selectedText, isSelecting);
    }
  }, [selectedText, isSelecting, onTextSelected]);

  if (!isSelecting) {
    return null;
  }

  return (
    <div className="text-selection-indicator">
      <div className="selected-text-preview">
        <strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
      </div>
      <button onClick={clearSelection} className="clear-selection-btn">
        Clear Selection
      </button>
    </div>
  );
};

export default TextSelection;