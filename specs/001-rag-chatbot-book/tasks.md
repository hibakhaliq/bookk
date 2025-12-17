---
description: "Task list for RAG Chatbot implementation"
---

# Tasks: RAG Chatbot for Book Content

**Input**: Design documents from `/specs/001-rag-chatbot-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume web app structure based on plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create backend project structure in backend/ with src/, tests/, requirements.txt
- [ ] T002 Create frontend project structure in frontend/ with src/, public/, package.json
- [ ] T003 [P] Initialize Python virtual environment and install FastAPI dependencies in backend/
- [ ] T004 [P] Initialize Node.js project and install React dependencies in frontend/
- [ ] T005 Create .env files for both backend and frontend with example configurations

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Setup Neon Serverless Postgres connection in backend/src/database/connection.py
- [ ] T007 Create database schema migrations for chat_sessions, messages, user_queries, retrieved_contexts tables
- [ ] T008 Setup Qdrant Cloud connection in backend/src/database/qdrant_client.py
- [ ] T009 [P] Create configuration management in backend/src/config.py for API keys and service URLs
- [ ] T010 [P] Setup FastAPI application structure in backend/src/main.py with proper error handling
- [ ] T011 Create base data models for ChatSession, Message, UserQuery, RetrievedContext in backend/src/models/
- [ ] T012 Setup logging infrastructure in backend/src/utils/logging.py
- [ ] T013 Create API response models in backend/src/models/api_responses.py based on contracts

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Book Content (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content and receive accurate answers based on the entire book

**Independent Test**: Can be fully tested by asking questions about book content and verifying that the responses are accurate and relevant to the book's content.

### Implementation for User Story 1

- [x] T014 [P] Create BookContentChunk model in backend/src/models/chunk.py with embedding_vector field
- [x] T015 Implement document ingestion service in backend/src/services/ingestion_service.py
- [x] T016 Implement embedding generation using Qwen via OpenRouter in backend/src/services/openrouter_client.py
- [x] T017 Implement vector storage service for Qdrant in backend/src/database/qdrant_client.py
- [x] T018 Create ingestion endpoint /ingest in backend/src/api/ingestion_router.py
- [x] T019 Implement RAG retrieval service in backend/src/services/rag_service.py for full book queries
- [x] T020 Implement chat history service in backend/src/services/chat_history_service.py using Neon Postgres
- [x] T021 Create chat endpoint /chat in backend/src/api/chat_router.py for full book queries
- [x] T022 [P] Create OpenRouter API client in backend/src/services/openrouter_client.py
- [x] T023 Integrate LLM call to OpenRouter in backend/src/services/llm_service.py
- [x] T024 [US1] Implement end-to-end flow from ingestion to response in backend/src/services/main_rag_service.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Enable users to select specific text on the book page and ask questions about only that text

**Independent Test**: Can be fully tested by selecting text on the page, asking a question about that text, and verifying that the answer is based only on the selected content.

### Implementation for User Story 2

- [x] T025 [P] Update ChatRequest model to include selected_text field in backend/src/api/chat_router.py
- [x] T026 Modify RAG retrieval service to support selected text queries in backend/src/services/rag_service.py
- [x] T027 Update chat endpoint to handle selected_text query type in backend/src/api/chat_router.py
- [x] T028 Create frontend component for text selection in frontend/src/components/TextSelection.js
- [x] T029 Implement text selection functionality in frontend/src/hooks/useTextSelection.js
- [x] T030 Update frontend chat service to send selected_text with requests in frontend/src/services/chatService.js
- [x] T031 [US2] Integrate selected text mode with existing chat functionality

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Maintain Conversation History (Priority: P3)

**Goal**: Enable multi-turn conversations with the chatbot to support contextual follow-up questions

**Independent Test**: Can be fully tested by having a multi-turn conversation with the chatbot and verifying that context is maintained across turns.

### Implementation for User Story 3

- [x] T032 [P] Enhance ChatSession model to track conversation context in backend/src/models/chat_models.py
- [x] T033 Update chat history service to maintain conversation context in backend/src/services/chat_history_service.py
- [x] T034 Implement context window management for conversation history in backend/src/services/context_service.py
- [x] T035 Update RAG retrieval to consider conversation context in backend/src/services/rag_service.py
- [x] T036 Create frontend hooks for conversation history management in frontend/src/hooks/useConversationHistory.js
- [x] T037 Update frontend UI to display conversation history in frontend/src/components/ChatHistory.js
- [x] T038 [US3] Integrate conversation history with existing chat functionality

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Frontend Integration

**Goal**: Embed ChatKit UI into the book's web output and configure it to communicate with the backend

- [x] T039 Setup ChatKit UI components in frontend/src/components/ChatInterface.js
- [x] T040 Create API service layer for backend communication in frontend/src/services/chatService.js
- [x] T041 Implement real-time chat interface with message streaming in frontend/src/components/ChatInterface.js
- [x] T042 Create embeddable script in frontend/src/embed/ChatbotEmbed.js for Speckit integration
- [x] T043 Style and theme the chat interface to match book aesthetics in frontend/src/styles/ChatInterface.css
- [x] T044 [P] Add TypeScript type definitions in frontend/src/types/ for better type safety
- [x] T045 Create React context for chat state management in frontend/src/contexts/ChatContext.js

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T046 [P] Add comprehensive error handling and user feedback in frontend/src/components/ErrorBoundary.js
- [x] T047 Add rate limiting to backend API endpoints in backend/src/middleware/rate_limit.py
- [x] T048 [P] Add input validation and sanitization to all endpoints in backend/src/api/ingestion_router.py and backend/src/api/chat_router.py
- [ ] T049 Add caching layer for frequently accessed embeddings in backend/src/services/cache_service.py
- [ ] T050 [P] Add monitoring and metrics collection in backend/src/utils/metrics.py
- [x] T051 Add security headers and CORS configuration in backend/src/main.py
- [x] T052 [P] Add comprehensive logging for debugging and monitoring in backend/src/utils/logging.py
- [x] T053 Run integration tests across all user stories in backend/tests/integration/
- [x] T054 [P] Update documentation in backend/README.md
- [ ] T055 Deploy and validate with quickstart.md instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Frontend Integration (Phase 6)**: Can start after Phase 3 (US1) is complete
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 functionality
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Builds on US1/US2 functionality

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members
- Frontend and backend tasks can be developed in parallel after foundational setup

---

## Parallel Example: User Story 1

```bash
# Launch all parallel tasks for User Story 1 together:
Task: "Create BookContentChunk model in backend/src/models/chunk.py with embedding_vector field"
Task: "Create OpenRouter API client in backend/src/services/openrouter_client.py"
Task: "Create ingestion endpoint /ingest in backend/src/api/ingestion_router.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Frontend Integration → Test → Deploy/Demo
6. Add Polish → Test → Final Deployment
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Frontend developer: Frontend Integration
4. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence