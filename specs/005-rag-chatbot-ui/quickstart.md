# Quickstart: RAG Chatbot UI

## Prerequisites

- Existing RAG backend running (`rag-backend/`)
- Existing Docusaurus frontend running (`humanoid-textbook/`)
- Neon PostgreSQL database (same as auth)
- OpenAI API key (for Agents SDK)
- Existing Cohere API key (for RAG)
- Existing Qdrant Cloud (for vectors)

## Backend Setup

### 1. Install new dependencies

```bash
cd rag-backend
pip install openai-chatkit openai-agents
```

Add to `requirements.txt`:
```
openai-chatkit
openai-agents
```

### 2. Add environment variable

Add to `.env`:
```bash
OPENAI_API_KEY=sk-...
```

### 3. Verify backend starts

```bash
cd rag-backend
uvicorn app.main:app --reload --port 8000
```

The `/chatkit` endpoint should be available at `http://localhost:8000/chatkit`.

## Frontend Setup

### 1. Install ChatKit React

```bash
cd humanoid-textbook
npm install @openai/chatkit-react
```

### 2. Verify frontend builds

```bash
npm run build
npm run serve
```

Navigate to any `/docs/` page — the chat widget should appear.

## End-to-End Test

1. Start backend: `uvicorn app.main:app --reload --port 8000`
2. Start frontend: `cd humanoid-textbook && npm start`
3. Navigate to `http://localhost:3000/humanoid-robotics-handbook/docs/`
4. Chat widget should appear (floating button, bottom-right)
5. Type "What is Physical AI?" and send
6. Response should stream with textbook citations
7. Navigate to landing page — widget should NOT appear

## Key Files

| File | Purpose |
|------|---------|
| `rag-backend/app/chatkit/server.py` | ChatKitServer subclass |
| `rag-backend/app/chatkit/store.py` | PostgreSQL Store |
| `rag-backend/app/chatkit/agent.py` | OpenAI Agent with RAG tool |
| `rag-backend/app/chatkit/models.py` | DB table models |
| `humanoid-textbook/src/components/ChatWidget/index.tsx` | React widget |
| `humanoid-textbook/src/theme/DocPage/Layout.tsx` | Docs-only injection |
