# Quickstart: RAG Backend

**Feature**: 003-rag-backend | **Date**: 2026-02-11

## Prerequisites

1. **Python 3.10+** installed
2. **Cohere API key** — free at [dashboard.cohere.com](https://dashboard.cohere.com)
3. **Qdrant Cloud cluster** — free at [cloud.qdrant.io](https://cloud.qdrant.io) (1GB free tier)

## Setup

### 1. Create the backend directory and virtual environment

```bash
cd humanoid-robotics-handbook
mkdir -p rag-backend
cd rag-backend
python -m venv .venv
source .venv/bin/activate  # Linux/Mac
# .venv\Scripts\activate   # Windows
```

### 2. Install dependencies

```bash
pip install fastapi[standard] uvicorn cohere qdrant-client python-dotenv pyyaml tiktoken
```

### 3. Create `.env` file

```bash
# rag-backend/.env
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# Frontend CORS origin
ALLOWED_ORIGINS=http://localhost:3000,https://laiba166-shaikh.github.io
```

### 4. Verify connections

```bash
python -c "
import cohere, os
from dotenv import load_dotenv
load_dotenv()
co = cohere.ClientV2(api_key=os.getenv('COHERE_API_KEY'))
r = co.embed(texts=['test'], model='embed-english-v3.0', input_type='search_query', embedding_types=['float'])
print(f'Cohere OK: {len(r.embeddings.float_[0])} dimensions')
"

python -c "
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
cols = client.get_collections()
print(f'Qdrant OK: {len(cols.collections)} collections')
"
```

### 5. Run the server

```bash
uvicorn app.main:app --reload --port 8000
```

### 6. Ingest content

```bash
python -m app.ingest --docs-path ../humanoid-textbook/docs/module-1-ros2
```

### 7. Test endpoints

```bash
# Health check
curl http://localhost:8000/api/health

# Search
curl -X POST http://localhost:8000/api/search \
  -H "Content-Type: application/json" \
  -d '{"query": "What is Physical AI?", "top_k": 3}'

# Chat
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "How do ROS 2 nodes communicate?"}'
```

## Project Structure

```
rag-backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI app, CORS, lifespan
│   ├── config.py             # Settings via pydantic-settings
│   ├── models/
│   │   ├── __init__.py
│   │   ├── requests.py       # SearchRequest, ChatRequest
│   │   ├── responses.py      # ApiResponse, SearchResponse, ChatResponse, HealthResponse
│   │   └── domain.py         # Chunk, SectionType, LessonFrontmatter, IngestionReport
│   ├── api/
│   │   ├── __init__.py
│   │   ├── health.py         # GET /api/health
│   │   ├── search.py         # POST /api/search
│   │   └── chat.py           # POST /api/chat
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py     # Cohere embed wrapper (batch, input_type)
│   │   ├── vectorstore.py    # Qdrant CRUD (create, upsert, delete, search)
│   │   ├── chat.py           # Cohere Command-R with documents + citations
│   │   ├── reranker.py       # Cohere rerank wrapper
│   │   └── query.py          # Query expansion for structural sections
│   ├── ingestion/
│   │   ├── __init__.py
│   │   ├── parser.py         # Markdown + frontmatter parsing
│   │   ├── chunker.py        # Adaptive chunking (classify, enrich, split)
│   │   └── pipeline.py       # Orchestrator: parse → chunk → embed → upsert
│   └── ingest.py             # CLI entry point: python -m app.ingest
├── tests/
│   ├── __init__.py
│   ├── test_chunker.py       # Unit tests for adaptive chunking
│   ├── test_parser.py        # Unit tests for markdown parsing
│   ├── test_api.py           # Integration tests for endpoints
│   └── conftest.py           # Fixtures
├── .env                       # Environment variables (gitignored)
├── .env.example               # Template for .env
├── requirements.txt           # Pinned dependencies
└── Procfile                   # Railway deployment: web: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

## Key Dependencies

```
fastapi>=0.115.0
uvicorn[standard]>=0.32.0
cohere>=5.0.0
qdrant-client>=1.12.0
python-dotenv>=1.0.0
pyyaml>=6.0.0
tiktoken>=0.7.0
pydantic>=2.0.0
pydantic-settings>=2.0.0
```

## Environment Variables

| Variable | Required | Description |
|----------|----------|-------------|
| `COHERE_API_KEY` | Yes | Cohere API key (free tier) |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `ALLOWED_ORIGINS` | Yes | Comma-separated CORS origins |
| `COLLECTION_NAME` | No | Qdrant collection name (default: `handbook_lessons`) |
| `LOG_LEVEL` | No | Logging level (default: `INFO`) |
