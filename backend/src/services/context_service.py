from typing import List, Dict, Any
from ..models.chat_models import Message


class ContextService:
    def __init__(self, max_context_length: int = 10):  # Keep last 10 messages
        self.max_context_length = max_context_length

    def trim_context(self, messages: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Trim the context to the most recent messages within the max context length
        """
        if len(messages) <= self.max_context_length:
            return messages
        else:
            # Return the most recent messages
            return messages[-self.max_context_length:]

    def format_context_for_llm(self, messages: List[Dict[str, Any]]) -> str:
        """
        Format the conversation history for the LLM
        """
        formatted_messages = []
        for msg in messages:
            role = msg.get('role', 'user')
            content = msg.get('content', '')
            formatted_messages.append(f"{role.capitalize()}: {content}")

        return "\n".join(formatted_messages)

    def get_relevant_context(self, messages: List[Dict[str, Any]], query: str, window_size: int = 5) -> List[Dict[str, Any]]:
        """
        Get the most relevant context for the current query
        """
        if len(messages) <= window_size:
            return messages

        # For now, return the most recent messages
        # In a more advanced implementation, we could use semantic similarity
        # to find the most relevant messages for the current query
        return messages[-window_size:]

    def calculate_context_relevance(self, messages: List[Dict[str, Any]], query: str) -> List[Dict[str, Any]]:
        """
        Calculate which messages are most relevant to the current query
        This is a simplified implementation - in practice, you'd use semantic similarity
        """
        # For now, just return the messages with a relevance score based on recency
        relevant_messages = []
        for i, msg in enumerate(messages):
            # More recent messages are more relevant (higher index = higher relevance)
            relevance_score = (i + 1) / len(messages) if messages else 0
            msg_copy = msg.copy()
            msg_copy['relevance_score'] = relevance_score
            relevant_messages.append(msg_copy)

        return relevant_messages