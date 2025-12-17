import logging
from datetime import datetime
import json
from typing import Dict, Any


class CustomFormatter(logging.Formatter):
    """Custom formatter to output logs in a structured format"""

    def format(self, record):
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
        }

        # Add any extra fields
        if hasattr(record, "user_id"):
            log_entry["user_id"] = record.user_id
        if hasattr(record, "session_token"):
            log_entry["session_token"] = record.session_token
        if hasattr(record, "request_id"):
            log_entry["request_id"] = record.request_id

        # Add exception info if present
        if record.exc_info:
            log_entry["exception"] = self.formatException(record.exc_info)

        return json.dumps(log_entry)


def setup_logging():
    """Set up the logging configuration"""
    # Create a custom logger
    logger = logging.getLogger("rag_chatbot")
    logger.setLevel(logging.INFO)

    # Create handlers
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)

    # Create formatters and add them to handlers
    console_handler.setFormatter(CustomFormatter())

    # Add handlers to the logger
    logger.addHandler(console_handler)

    # Also set up the root logger with the same handler for consistency
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    if not root_logger.handlers:
        root_logger.addHandler(console_handler)

    return logger


# Global logger instance
logger = setup_logging()


def log_api_call(endpoint: str, method: str, user_id: str = None, session_token: str = None, request_id: str = None):
    """Decorator or function to log API calls"""
    def log_message():
        extra = {}
        if user_id:
            extra["user_id"] = user_id
        if session_token:
            extra["session_token"] = session_token
        if request_id:
            extra["request_id"] = request_id

        logger.info(f"API call to {method} {endpoint}", extra=extra)

    return log_message


def log_error(message: str, user_id: str = None, session_token: str = None, request_id: str = None, exc_info: bool = False):
    """Log an error with optional context"""
    extra = {}
    if user_id:
        extra["user_id"] = user_id
    if session_token:
        extra["session_token"] = session_token
    if request_id:
        extra["request_id"] = request_id

    logger.error(message, extra=extra, exc_info=exc_info)


def log_info(message: str, user_id: str = None, session_token: str = None, request_id: str = None):
    """Log an info message with optional context"""
    extra = {}
    if user_id:
        extra["user_id"] = user_id
    if session_token:
        extra["session_token"] = session_token
    if request_id:
        extra["request_id"] = request_id

    logger.info(message, extra=extra)


def log_debug(message: str, user_id: str = None, session_token: str = None, request_id: str = None):
    """Log a debug message with optional context"""
    extra = {}
    if user_id:
        extra["user_id"] = user_id
    if session_token:
        extra["session_token"] = session_token
    if request_id:
        extra["request_id"] = request_id

    logger.debug(message, extra=extra)