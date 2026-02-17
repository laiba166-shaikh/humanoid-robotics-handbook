# Feature Specification: RAG Chatbot Conversational Interface

**Feature Branch**: `005-rag-chatbot-ui`
**Created**: 2026-02-16
**Status**: Draft
**Input**: User description: "Add simple rag chatbot conversational interface for my book to work with my working backend. Use OpenAI Agent SDK and OpenAI ChatKit for the agents, server implementation and UI widgets. Leverage @chatkit-integration.skill for the implementation."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask the Textbook a Question via Chat Widget (Priority: P1)

A student reading the Physical AI & Humanoid Robotics Handbook wants to ask a question about the content. They open a chat widget embedded in the textbook site, type a natural language question (e.g., "What is Physical AI?"), and receive a streaming response with accurate, cited answers drawn from the textbook content. The response appears token-by-token as it is generated, giving immediate feedback.

**Why this priority**: This is the core value — connecting the existing RAG backend to a real user-facing chat interface. Without this, the backend has no consumer.

**Independent Test**: Can be fully tested by opening the textbook site, clicking the chat widget, typing "What is Physical AI?", and verifying that a streamed response appears with content sourced from Module 1 and formatted citations.

**Acceptance Scenarios**:

1. **Given** the chat widget is loaded on the textbook site, **When** a student types "What is Physical AI?" and sends it, **Then** the system streams a response token-by-token that includes content from the textbook with source citations.
2. **Given** the chat widget is open, **When** a student asks a question not covered in the textbook (e.g., "What is quantum computing?"), **Then** the system responds with a message indicating the topic is not in the textbook and suggests relevant lessons.
3. **Given** the chat widget is open, **When** a student sends an empty message, **Then** the system does not submit the message and shows an inline validation prompt.
4. **Given** the student is on a docs page (e.g., `/docs/module-1-ros2/...`), **When** the page loads, **Then** the chat widget is visible and accessible.
5. **Given** the student is on a non-docs page (e.g., landing page, blog), **When** the page loads, **Then** the chat widget is not rendered or visible.

---

### User Story 2 - Continue a Multi-Turn Conversation (Priority: P1)

A student asks a follow-up question after receiving an initial answer. The chatbot understands the conversational context and provides a relevant follow-up response without requiring the student to repeat the full question. For example, after asking "What is ROS 2?" and receiving an answer, the student asks "How do its nodes communicate?" and the chatbot understands "its" refers to ROS 2.

**Why this priority**: Multi-turn conversation is essential for a natural chat experience and is a core expectation of any conversational interface.

**Independent Test**: Can be tested by asking "What is ROS 2?", then asking "How do its nodes communicate?", and verifying the second response references ROS 2 nodes and communication patterns.

**Acceptance Scenarios**:

1. **Given** the student has asked "What is ROS 2?" and received an answer, **When** they ask "How do its nodes communicate?", **Then** the system understands the context and responds about ROS 2 node communication.
2. **Given** the student has an active conversation thread, **When** they ask "Tell me more about that", **Then** the system references the most recent topic and expands on it.
3. **Given** a conversation has 10+ messages, **When** the student asks a new question, **Then** the system still responds accurately within the expected latency.

---

### User Story 3 - Browse Conversation History (Priority: P2)

A student returns to the textbook after a previous session and wants to review their earlier conversations with the chatbot. They can see a list of past conversation threads in the chat widget, select one, and continue where they left off.

**Why this priority**: Conversation persistence adds significant value for returning learners but requires additional storage infrastructure. The core chat experience works without it.

**Independent Test**: Can be tested by having a conversation, closing the browser, returning to the site, and verifying the previous thread is listed and can be opened with full message history.

**Acceptance Scenarios**:

1. **Given** a student has had previous conversations, **When** they open the chat widget, **Then** they see a list of their past conversation threads with titles or preview text.
2. **Given** a student selects a previous conversation thread, **When** the thread loads, **Then** all messages from that thread are displayed in order.
3. **Given** a student has no previous conversations, **When** they open the chat widget, **Then** they see an empty state with a prompt to start a new conversation.

---

### User Story 4 - See Retrieval Progress Indicators (Priority: P3)

While the chatbot processes a question, the student sees visual progress indicators showing what the system is doing — searching the knowledge base, generating a response. This provides transparency and reduces perceived wait time.

**Why this priority**: Progress indicators improve user experience but are an enhancement over the core chat functionality.

**Independent Test**: Can be tested by asking a question and observing that progress indicators appear (e.g., "Searching knowledge base...", "Generating response...") before the answer streams in.

**Acceptance Scenarios**:

1. **Given** the student sends a question, **When** the system begins processing, **Then** a progress indicator shows "Searching knowledge base..." during retrieval.
2. **Given** the retrieval step completes, **When** the system begins generating a response, **Then** the progress indicator updates to show the response is being generated.

---

### Edge Cases

- What happens when the backend is unavailable? The chat widget shows a user-friendly error message (e.g., "The chatbot is temporarily unavailable. Please try again later.") and does not crash.
- What happens when the student's network connection drops mid-stream? The partial response already displayed is preserved, and the widget shows a "Connection lost" message with a retry option.
- What happens when the streaming response is interrupted by the student canceling? The partial response is displayed and the input is re-enabled for a new question.
- What happens when the student types a very long message (over 1000 characters)? The system accepts and processes the message, truncating if necessary at the embedding model's input limit, without showing an error.
- What happens when multiple tabs are open with the chat widget? Each tab operates independently with its own conversation thread.
- What happens when the chatbot receives a request during backend rate limiting? The system shows a friendly "Too many requests, please wait a moment" message.
- What happens when a student navigates from a docs page to a non-docs page while mid-conversation? The conversation state is preserved in the thread; when the student returns to any docs page, the widget reappears with the conversation intact.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a chat widget that is visible only on docs pages (routes under `/docs/`) of the Docusaurus textbook site, allowing students to type and send natural language questions. The widget MUST NOT appear on non-docs pages such as the landing page, blog, or custom pages.
- **FR-002**: System MUST stream responses token-by-token to the chat widget as they are generated, providing immediate visual feedback.
- **FR-003**: System MUST use the existing RAG backend services (vector search, embedding, reranking, chat generation) to produce answers from textbook content.
- **FR-004**: System MUST maintain conversation context across multiple messages within a single thread, enabling follow-up questions that reference prior messages.
- **FR-005**: System MUST display source citations in responses, linking back to the relevant module, chapter, and lesson.
- **FR-006**: System MUST persist conversation threads and messages for authenticated users so students can return to previous conversations.
- **FR-007**: System MUST provide a ChatKit-compatible server endpoint that handles thread management, message processing, and streaming responses.
- **FR-008**: System MUST display progress indicators to the student during retrieval and generation phases.
- **FR-009**: System MUST allow students to cancel an in-progress response stream.
- **FR-010**: System MUST show appropriate error messages when the backend is unavailable or an error occurs, without crashing the widget.
- **FR-011**: System MUST support theming to match the textbook site's visual design (colors, fonts, dark/light mode).
- **FR-012**: System MUST work with the existing authentication system, identifying the current user for conversation persistence.

### Key Entities

- **Conversation Thread**: A sequence of messages between a student and the chatbot. Has a unique identifier, creation timestamp, and belongs to a specific user. Persists across sessions.
- **Message**: A single exchange unit within a thread. Can be a student question or a chatbot response. Contains text content, timestamp, and role (user/assistant). Assistant messages may include source citations.
- **Source Citation**: A reference to a specific location in the textbook (module, chapter, lesson) that was used to generate a response. Attached to assistant messages.

### Non-Goals (Deferred to Future Phases)

- **Real-time collaborative chat** — Multiple users chatting in the same thread is not in scope.
- **Voice input/output** — Speech-to-text and text-to-speech are not part of this feature.
- **Custom widget configuration per page** — The chat widget has one configuration for the entire site.
- **Admin dashboard for conversation analytics** — Monitoring conversations is deferred.
- **File/image uploads in chat** — The chat interface handles text-only questions.

### Assumptions

- The existing RAG backend (FastAPI + Qdrant + Cohere) is deployed and operational with ingested textbook content.
- The existing authentication system (feature 004-user-auth) provides user identity that can be used for conversation ownership.
- The Docusaurus frontend can integrate React components (ChatKit React is compatible).
- OpenAI ChatKit JS and Python SDK are stable and available for use.
- The OpenAI Agents SDK can orchestrate tool calls to the existing Cohere-based RAG services (the agent wraps the existing services, it does not replace them).
- Conversation history is stored in the existing PostgreSQL database (same as auth).
- The ChatKit server endpoint will be added to the existing FastAPI application alongside the current `/api/chat` and `/api/search` endpoints.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can ask a question and see the first token of the streamed response within 2 seconds of submitting.
- **SC-002**: Multi-turn conversations maintain context correctly for at least 10 consecutive messages in a thread.
- **SC-003**: Returning students can access their previous conversation threads within 1 second of opening the chat widget.
- **SC-004**: 90% of test questions produce responses with at least one accurate source citation from the textbook.
- **SC-005**: The chat widget renders correctly on desktop and mobile viewports without layout overflow or broken UI elements.
- **SC-006**: Error states (backend down, network loss) display user-friendly messages within 3 seconds of the failure occurring.
