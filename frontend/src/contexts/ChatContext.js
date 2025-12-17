import React, { createContext, useContext, useReducer } from 'react';

// Initial state for the chat context
const initialState = {
  sessionToken: null,
  messages: [],
  isLoading: false,
  error: null,
  queryType: 'full_book',
  selectedText: '',
  isSelecting: false
};

// Action types
const actionTypes = {
  SET_SESSION_TOKEN: 'SET_SESSION_TOKEN',
  ADD_MESSAGE: 'ADD_MESSAGE',
  SET_LOADING: 'SET_LOADING',
  SET_ERROR: 'SET_ERROR',
  SET_QUERY_TYPE: 'SET_QUERY_TYPE',
  SET_SELECTED_TEXT: 'SET_SELECTED_TEXT',
  SET_IS_SELECTING: 'SET_IS_SELECTING',
  CLEAR_MESSAGES: 'CLEAR_MESSAGES',
  RESET: 'RESET'
};

// Reducer function
function chatReducer(state, action) {
  switch (action.type) {
    case actionTypes.SET_SESSION_TOKEN:
      return {
        ...state,
        sessionToken: action.payload
      };
    case actionTypes.ADD_MESSAGE:
      return {
        ...state,
        messages: [...state.messages, action.payload]
      };
    case actionTypes.SET_LOADING:
      return {
        ...state,
        isLoading: action.payload
      };
    case actionTypes.SET_ERROR:
      return {
        ...state,
        error: action.payload,
        isLoading: false
      };
    case actionTypes.SET_QUERY_TYPE:
      return {
        ...state,
        queryType: action.payload
      };
    case actionTypes.SET_SELECTED_TEXT:
      return {
        ...state,
        selectedText: action.payload
      };
    case actionTypes.SET_IS_SELECTING:
      return {
        ...state,
        isSelecting: action.payload
      };
    case actionTypes.CLEAR_MESSAGES:
      return {
        ...state,
        messages: []
      };
    case actionTypes.RESET:
      return initialState;
    default:
      return state;
  }
}

// Create context
const ChatContext = createContext();

// Provider component
export function ChatProvider({ children }) {
  const [state, dispatch] = useReducer(chatReducer, initialState);

  // Actions
  const setSessionToken = (token) => {
    dispatch({ type: actionTypes.SET_SESSION_TOKEN, payload: token });
  };

  const addMessage = (message) => {
    dispatch({ type: actionTypes.ADD_MESSAGE, payload: message });
  };

  const setLoading = (isLoading) => {
    dispatch({ type: actionTypes.SET_LOADING, payload: isLoading });
  };

  const setError = (error) => {
    dispatch({ type: actionTypes.SET_ERROR, payload: error });
  };

  const setQueryType = (queryType) => {
    dispatch({ type: actionTypes.SET_QUERY_TYPE, payload: queryType });
  };

  const setSelectedText = (text) => {
    dispatch({ type: actionTypes.SET_SELECTED_TEXT, payload: text });
  };

  const setIsSelecting = (isSelecting) => {
    dispatch({ type: actionTypes.SET_IS_SELECTING, payload: isSelecting });
  };

  const clearMessages = () => {
    dispatch({ type: actionTypes.CLEAR_MESSAGES });
  };

  const reset = () => {
    dispatch({ type: actionTypes.RESET });
  };

  const value = {
    ...state,
    actions: {
      setSessionToken,
      addMessage,
      setLoading,
      setError,
      setQueryType,
      setSelectedText,
      setIsSelecting,
      clearMessages,
      reset
    }
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}

// Custom hook to use the chat context
export function useChat() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
}