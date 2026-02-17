# Tasks: RAG Chatbot Conversational Interface

**Input**: Design documents from `/specs/005-rag-chatbot-ui/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested. Test tasks omitted.

**Organization**: Tasks grouped by user story. Each story is independently testable.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4)

---

## Phase 1: Setup

**Purpose**: Install dependencies and create project scaffolding

- [x] T001 Install ChatKit Python SDK and Agents SDK in rag-backend/requirements.txt (`openai-chatkit`, `openai-agents`)
- [x] T002 Install ChatKit React package in humanoid-textbook/package.json (`@openai/chatkit-react`)
- [x] T003 Add `openai_api_key` setting to rag-backend/app/config.py and update rag-backend/.env.example
- [x] T004 Create backend module directory structure: rag-backend/app/chatkit/__init__.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create ChatKit database models (threads and items tables) in rag-backend/app/chatkit/models.py per data-model.md
- [x] T006 Implement PostgresStore class (ChatKit Store interface) in rag-backend/app/chatkit/store.py using existing engine from app.database
- [x] T007 Create OpenAI Agent with `search_textbook` function tool in rag-backend/app/chatkit/agent.py — tool wraps existing QueryExpander, CohereEmbedService, QdrantService, CohereRerankService
- [x] T008 Implement ChatKitServer subclass with `respond()` method in rag-backend/app/chatkit/server.py — loads thread history, runs agent, streams response events
- [x] T009 Add `/chatkit` POST endpoint to rag-backend/app/main.py — initialize ChatKitServer in lifespan, route to server.process(), return StreamingResponse or JSON Response
- [x] T010 Add CORS and auth header extraction in `/chatkit` endpoint — extract user_id from JWT Authorization header (optional, fallback to "anonymous")

**Checkpoint**: Backend ChatKit server functional. Can test with curl or ChatKit JS client.

---

## Phase 3: User Story 1 — Ask the Textbook a Question via Chat Widget (Priority: P1) MVP

**Goal**: Student opens chat widget on a docs page, types a question, receives a streaming response with textbook citations.

**Independent Test**: Open any `/docs/` page, click chat widget, type "What is Physical AI?", verify streamed response with citations appears. Verify widget does NOT appear on landing page.

### Implementation for User Story 1

- [x] T011 [US1] Create ChatWidget React component in humanoid-textbook/src/components/ChatWidget/index.tsx — use `useChatKit` hook pointing to backend `/chatkit` endpoint, wrap in `BrowserOnly` for SSR safety
- [x] T012 [US1] Create ChatWidget CSS module in humanoid-textbook/src/components/ChatWidget/ChatWidget.module.css — apply constitution color palette (Pacific Blue accent, Jet Black/Azure Mist surfaces, dark/light mode)
- [x] T013 [US1] Swizzle DocRoot/Layout to inject ChatWidget — run `npx docusaurus swizzle @docusaurus/theme-classic DocRoot/Layout --wrap`, add ChatWidget as floating overlay in the wrapper at humanoid-textbook/src/theme/DocRoot/Layout/index.tsx
- [x] T014 [US1] Pass JWT access token from AuthProvider to ChatWidget API headers — read `accessToken` from `useAuth()` hook, pass as `Authorization: Bearer` header in `useChatKit` config
- [x] T015 [US1] Configure agent system prompt for textbook persona in rag-backend/app/chatkit/agent.py — use constitution's RAG Chatbot Behavior persona ("You are a helpful teaching assistant...") with citation format instructions. Verify end-to-end that citations in `[Module X, Chapter Y, Lesson Z]` format render visibly in the ChatKit widget; if not auto-formatted, add minimal citation styling in ChatWidget component
- [x] T016 [US1] Add error handling in ChatKitServer.respond() — catch service exceptions, yield ErrorEvent with user-friendly messages per spec edge cases. Include: (a) input length validation — truncate messages exceeding 4000 chars to embedding model limit, log warning; (b) rate limit handling — catch 429/rate-limit errors from Cohere/OpenAI, yield ErrorEvent with "Too many requests, please wait a moment" message

**Checkpoint**: US1 complete. Chat widget visible on docs pages only, streaming answers with citations, not visible on landing/blog pages.

---

## Phase 4: User Story 2 — Continue a Multi-Turn Conversation (Priority: P1)

**Goal**: Student asks follow-up questions and the chatbot maintains conversational context across multiple messages in a thread.

**Independent Test**: Ask "What is ROS 2?", then ask "How do its nodes communicate?". Second response should reference ROS 2 without the user repeating it.

### Implementation for User Story 2

- [x] T017 [US2] Configure agent to receive full thread history as input in rag-backend/app/chatkit/server.py — load last 20 items via store.load_thread_items(), convert via simple_to_agent_input(), pass to Runner.run_streamed()
- [x] T018 [US2] Add context-aware search tool — update search_textbook tool in rag-backend/app/chatkit/agent.py to let the agent formulate its own search query from conversation context (not just the latest user message)
- [x] T019 [US2] Verify response cancellation support — ensure ChatKit streaming respects client disconnect, partial responses are preserved in store

**Checkpoint**: US2 complete. Multi-turn conversations work with context. Follow-up questions resolved correctly.

---

## Phase 5: User Story 3 — Browse Conversation History (Priority: P2)

**Goal**: Returning students see their previous conversation threads and can resume them.

**Independent Test**: Have a conversation, close browser, reopen docs page, verify previous thread is listed and loadable with full message history.

### Implementation for User Story 3

- [x] T020 [US3] Ensure PostgresStore.load_threads() filters by user_id in rag-backend/app/chatkit/store.py — authenticated users see only their threads, anonymous users see nothing on return
- [x] T021 [US3] Enable thread browsing in ChatKit frontend config — set `threads: { enabled: true }` in `useChatKit` options in humanoid-textbook/src/components/ChatWidget/index.tsx
- [x] T022 [US3] Auto-generate thread titles from first user message — in ChatKitServer.respond(), after first user message, update thread metadata with a title derived from the message content

**Checkpoint**: US3 complete. Authenticated users see thread list, can resume past conversations.

---

## Phase 6: User Story 4 — Retrieval Progress Indicators (Priority: P3)

**Goal**: Student sees visual progress feedback during knowledge base search and response generation.

**Independent Test**: Ask a question and observe "Searching knowledge base..." and "Generating response..." indicators before the answer streams.

### Implementation for User Story 4

- [x] T023 [US4] Add ProgressUpdateEvent streaming in search_textbook tool — yield `ProgressUpdateEvent(icon="search", text="Searching knowledge base...")` before search and `ProgressUpdateEvent(icon="check", text="Found N relevant sources")` after rerank in rag-backend/app/chatkit/agent.py
- [x] T024 [US4] Verify progress indicators render in ChatKit widget — confirm ChatKit JS automatically displays ProgressUpdateEvent as UI indicators (no frontend code needed, built into ChatKit)

**Checkpoint**: US4 complete. Users see real-time progress during retrieval.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements across all stories

- [x] T025 [P] Verify rag-backend/.env.example with OPENAI_API_KEY documentation
- [x] T026 [P] Update constitution environment variables section in .specify/memory/constitution.md — add OPENAI_API_KEY; also document `/chatkit` endpoint as a ChatKit-protocol exception to the standard JSON envelope format (per ADR-001)
- [ ] T027 Run quickstart.md end-to-end validation — start backend, start frontend, test chat on docs page, verify no widget on landing page, test multi-turn, test thread history, verify multiple browser tabs operate with independent conversation threads, verify docs-to-non-docs navigation preserves thread state on return
- [ ] T028 Verify mobile responsiveness of ChatWidget — test floating widget on narrow viewport, ensure no layout overflow

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 — BLOCKS all user stories
- **Phase 3 (US1)**: Depends on Phase 2
- **Phase 4 (US2)**: Depends on Phase 2 (can run in parallel with US1, but US1 is recommended first as it establishes the widget)
- **Phase 5 (US3)**: Depends on Phase 2
- **Phase 6 (US4)**: Depends on Phase 2
- **Phase 7 (Polish)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (P1)**: Independent — establishes core chat flow
- **US2 (P1)**: Builds on US1's server (T017-T018 modify server.py and agent.py created in US1)
- **US3 (P2)**: Independent of US1/US2 (store filtering and thread config)
- **US4 (P3)**: Independent (adds progress events to agent tool created in Phase 2)

### Within Each User Story

- Backend tasks before frontend tasks (server must exist before widget connects)
- Models → Store → Agent → Server → Endpoint → Widget

### Parallel Opportunities

- T001 and T002 can run in parallel (backend vs frontend installs)
- T005 and T007 can run in parallel (DB models vs agent definition)
- T011 and T012 can run in parallel (component vs CSS)
- T025 and T026 can run in parallel (env docs vs constitution update)
- US3 and US4 can run in parallel after US1 is complete

---

## Parallel Example: Phase 2 (Foundational)

```
# These can start in parallel:
T005: Create DB models in chatkit/models.py
T007: Create Agent with search tool in chatkit/agent.py

# Then sequentially:
T006: Implement Store (depends on T005 models)
T008: Implement ChatKitServer (depends on T006 store + T007 agent)
T009: Add /chatkit endpoint (depends on T008 server)
T010: Add auth extraction (depends on T009 endpoint)
```

---

## Implementation Strategy

### MVP First (US1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T010)
3. Complete Phase 3: US1 (T011-T016)
4. **STOP and VALIDATE**: Chat widget works on docs pages, streams answers with citations
5. Deploy/demo if ready

### Incremental Delivery

1. Setup + Foundational → Backend server functional
2. Add US1 → Chat widget works → **MVP deployed**
3. Add US2 → Multi-turn context works
4. Add US3 → Thread history persists
5. Add US4 → Progress indicators visible
6. Polish → Environment docs, mobile check, constitution sync

---

## Notes

- Leverage `@chatkit-integration` skill during implementation for ChatKit patterns
- Existing services (QueryExpander, CohereEmbedService, QdrantService, CohereRerankService) accessed via `app.state` — no modifications needed
- The `/chatkit` endpoint uses ChatKit protocol (not the project's REST JSON envelope) — this is expected per ADR-001
- Anonymous users get functional chat but no thread persistence — by design
