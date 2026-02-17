# Contract: ChatKit Endpoint

## POST /chatkit

ChatKit protocol endpoint. Handles all ChatKit client requests (thread creation, message sending, thread listing, item loading).

**Note**: This is NOT a standard REST endpoint. ChatKit uses its own binary protocol. The endpoint receives raw request bytes from the ChatKit JS client and delegates to `ChatKitServer.process()`.

### Request

```
POST /chatkit
Content-Type: application/json
Authorization: Bearer <jwt_access_token>  (optional)
```

Body: ChatKit protocol payload (opaque, handled by SDK)

### Response

Two possible response types:

**Streaming (message responses)**:
```
HTTP/1.1 200 OK
Content-Type: text/event-stream

data: {"type": "thread_item_created", "item": {...}}
data: {"type": "text_delta", "item_id": "msg_xxx", "delta": "The "}
data: {"type": "text_delta", "item_id": "msg_xxx", "delta": "answer "}
data: {"type": "progress_update", "icon": "search", "text": "Searching..."}
data: {"type": "thread_item_done", "item": {...}}
```

**JSON (thread listing, item loading)**:
```
HTTP/1.1 200 OK
Content-Type: application/json

{...}  // ChatKit protocol response
```

### Auth Behavior

| Token State | Behavior |
|-------------|----------|
| Valid JWT | Extract `user_id`, threads persist per user |
| Missing/invalid | Anonymous mode, `user_id = "anonymous"`, no cross-session persistence |

### Error Responses

ChatKit errors are yielded as `ErrorEvent` within the stream:

```json
{"type": "error", "message": "The chatbot is temporarily unavailable.", "allow_retry": true}
```

## Existing Endpoints (unchanged)

These endpoints remain as-is:

| Method | Path | Purpose |
|--------|------|---------|
| GET | /api/health | Health check |
| POST | /api/search | Semantic search |
| POST | /api/chat | Direct RAG chat (non-streaming, non-ChatKit) |
| POST | /api/auth/* | Auth endpoints |
