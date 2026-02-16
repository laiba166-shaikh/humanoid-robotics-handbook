---
id: 002
title: Plan ChatKit RAG Chatbot UI
stage: plan
date: 2026-02-16
surface: agent
model: claude-opus-4-6
feature: rag-chatbot-ui
branch: 005-rag-chatbot-ui
user: laiba166-shaikh
command: /sp.plan
labels: ["chatkit", "rag", "architecture", "planning", "agents-sdk"]
links:
  spec: specs/005-rag-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-rag-chatbot-ui/plan.md
  - specs/005-rag-chatbot-ui/research.md
  - specs/005-rag-chatbot-ui/data-model.md
  - specs/005-rag-chatbot-ui/contracts/chatkit-endpoint.md
  - specs/005-rag-chatbot-ui/quickstart.md
tests:
  - none
---

## Prompt

/sp.plan create the simple implementation of the chatbot UI with my skill and rag-backend

## Response snapshot

Created implementation plan for 005-rag-chatbot-ui. Architecture: ChatKit JS widget (docs-only via DocPage swizzle) → FastAPI /chatkit endpoint (ChatKitServer) → OpenAI Agents SDK agent with search_textbook tool → existing Cohere RAG pipeline (embed → Qdrant search → rerank). Conversation persistence in existing Neon PostgreSQL (2 new tables: chatkit_threads, chatkit_thread_items). Auth via JWT header passthrough. 5 research decisions documented. Data model, API contract, and quickstart generated. Constitution check passed all gates.

## Outcome

- Impact: Complete plan with all Phase 0-1 artifacts, ready for /sp.tasks
- Tests: none (planning phase)
- Files: plan.md, research.md, data-model.md, contracts/chatkit-endpoint.md, quickstart.md
- Next prompts: /sp.tasks to generate task breakdown
- Reflection: Clean architecture that reuses all existing services. The only new external dependency is OPENAI_API_KEY for the Agents SDK.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Constitution check PASS (all gates)
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
