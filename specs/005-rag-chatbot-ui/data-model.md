# Data Model: RAG Chatbot UI

## Entities

### ChatKit Thread (table: `chatkit_threads`)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string(32) | PK | Format: `thr_{uuid_hex[:8]}` |
| user_id | string(255) | NOT NULL, indexed | Owner's user ID from auth system |
| title | string(500) | nullable | Auto-generated from first message or null |
| metadata_json | text | NOT NULL, default `{}` | Serialized thread metadata (ChatKit SDK) |
| created_at | timestamp(tz) | server default now() | Thread creation time |
| updated_at | timestamp(tz) | auto-update on change | Last activity time |

**Indexes**:
- `ix_chatkit_threads_user_created` on (user_id, created_at) — for listing user's threads

### ChatKit Thread Item (table: `chatkit_thread_items`)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string(32) | PK | Format: `msg_{uuid_hex[:8]}`, `tc_{uuid_hex[:8]}`, etc. |
| thread_id | string(32) | FK → chatkit_threads.id, CASCADE | Parent thread |
| item_json | text | NOT NULL | Full ThreadItem serialized as JSON |
| sort_key | integer | NOT NULL | Auto-incrementing per thread for ordering |
| created_at | timestamp(tz) | server default now() | Item creation time |

**Indexes**:
- `ix_chatkit_items_thread_sort` on (thread_id, sort_key) — for loading items in order
- `ix_chatkit_items_thread_created` on (thread_id, created_at) — for cursor pagination

**Design decisions**:
- Items serialized as JSON (not individual columns) for forward compatibility with ChatKit SDK schema changes
- `sort_key` is a monotonically increasing integer per thread (not timestamp) to guarantee stable ordering
- CASCADE delete: removing a thread removes all its items

### Relationships

```
User (auth system)
  └── 1:N ChatKit Threads (via user_id)
           └── 1:N ChatKit Thread Items (via thread_id, CASCADE)
```

### ID Prefixes (ChatKit convention)

| Item Type | Prefix | Example |
|-----------|--------|---------|
| thread | `thr` | `thr_a1b2c3d4` |
| message | `msg` | `msg_e5f6g7h8` |
| tool_call | `tc` | `tc_i9j0k1l2` |
| workflow | `wf` | `wf_m3n4o5p6` |

## No Schema Changes to Existing Tables

The auth tables (`users`, `refresh_tokens`) remain unchanged. The ChatKit tables are additive only.
