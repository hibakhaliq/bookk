# Implementation Plan: RAG Chatbot for Book Content

**Branch**: `001-rag-chatbot-book` | **Date**: 2025-12-15 | **Spec**: [RAG Chatbot Spec](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Retrieval-Augmented Generation (RAG) chatbot that enables users to ask questions about book content through both full-book queries and selected-text queries. The system includes a FastAPI backend with Qdrant vector storage and Neon Postgres for metadata/chat history, integrated with the Speckit-published book frontend using ChatKit UI.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/TypeScript (frontend)
**Primary Dependencies**: FastAPI, Qdrant client, OpenAI/ChatKit SDKs, Neon Postgres client, OpenRouter API
**Storage**: Qdrant Cloud (vector storage), Neon Serverless Postgres (metadata/chat history)
**Testing**: pytest (backend), Jest (frontend)
**Target Platform**: Web application (Linux server backend, browser frontend)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: <5s response time for queries, support 500 concurrent users, 85%+ accuracy for relevant responses
**Constraints**: <200ms p95 for internal API calls, 2000 character limit for selected text queries, must work with Speckit-generated HTML/JS
**Scale/Scope**: 1 book with multiple chapters, 500 concurrent users, multi-turn conversations up to 20 exchanges

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Check

| Principle | Status | Justification |
|-----------|--------|---------------|
| Library-First | ✅ COMPLIANT | Backend services will be implemented as reusable libraries with clear APIs |
| CLI Interface | ✅ COMPLIANT | Backend will expose functionality via REST API with JSON I/O protocol |
| Test-First (NON-NEGOTIABLE) | ✅ COMPLIANT | All components will have unit and integration tests written before implementation |
| Integration Testing | ✅ COMPLIANT | Tests will cover API contracts, RAG pipeline, and frontend-backend integration |
| Observability | ✅ COMPLIANT | Structured logging will be implemented for debugging and monitoring |
| Simplicity | ✅ COMPLIANT | Starting with core RAG functionality, adding features incrementally |

### Additional Constraints Compliance

- ✅ Technology stack: Python 3.11+, Node.js 18+ for frontend components - COMPLIANT
- ✅ Open-source dependencies only - COMPLIANT (FastAPI, Qdrant client, etc.)
- ✅ Code reviews mandatory - COMPLIANT (part of workflow)
- ✅ Automated tests required - COMPLIANT (test-first principle)

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/          # Data models (ChatSession, Message, etc.)
│   ├── services/        # Core services (RAG, Embedding, Vector store)
│   ├── api/             # FastAPI endpoints (/chat, /ingest)
│   ├── database/        # Database connection and operations
│   └── utils/           # Utility functions
├── tests/
│   ├── unit/
│   ├── integration/
│   └── contract/
└── requirements.txt

frontend/
├── src/
│   ├── components/      # ChatKit UI components
│   ├── services/        # API client for backend communication
│   ├── hooks/           # React hooks for chat functionality
│   └── types/           # TypeScript type definitions
├── public/
└── package.json

# Integration with existing book structure
book/
├── ...
└── chatbot-embed.js     # Script to embed chatbot in Speckit output
```

**Structure Decision**: Web application structure selected with separate backend (FastAPI) and frontend (React/ChatKit) components to maintain clear separation of concerns. The backend handles RAG logic and API, while the frontend provides the user interface and integrates with the existing Speckit book structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
