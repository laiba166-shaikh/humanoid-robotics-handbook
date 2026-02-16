# ADR-002: Conversation Persistence in Existing PostgreSQL with JSON Serialization

> **Scope**: Covers the storage strategy for ChatKit conversation threads and messages — database choice, schema design pattern (JSON serialization vs relational columns), pagination strategy, and auth integration for thread ownership.

- **Status:** Accepted
- **Date:** 2026-02-16
- **Feature:** 005-rag-chatbot-ui
- **Context:** ChatKit requires a persistent `Store` implementation to save conversation threads and messages across sessions. The project already has a Neon PostgreSQL database (used by auth) with an async SQLAlchemy engine. We need to decide where and how to store conversation data, balancing simplicity, forward compatibility with ChatKit SDK updates, and reuse of existing infrastructure.

## Decision

- **Database**: Reuse existing Neon PostgreSQL instance and SQLAlchemy async engine from `app.database`
- **Schema**: Two new tables (`chatkit_threads`, `chatkit_thread_items`) using the existing `Base` declarative base
- **Serialization**: Thread items stored as full JSON blobs (`item_json` TEXT column) rather than mapped to individual relational columns
- **Ordering**: Integer `sort_key` column (auto-incrementing per thread) for stable item ordering
- **Pagination**: Cursor-based (using item ID as cursor), not OFFSET-based
- **Thread ownership**: `user_id` column on threads, populated from JWT claim; anonymous users get `user_id = "anonymous"`
- **Deletion**: CASCADE from threads to items — deleting a thread removes all messages
- **Table creation**: Via existing `Base.metadata.create_all()` in app lifespan (no migration tool needed)

## Consequences

### Positive

- **Zero new infrastructure**: No new database, no new connection pool, no new deployment configuration
- **Forward compatible**: JSON serialization means ChatKit SDK schema changes (new item fields, types) don't break storage — the JSON just gets larger
- **Simple schema**: 2 tables, 2 indexes per table — minimal maintenance burden
- **Auth reuse**: Same database as auth means user_id foreign key relationships are possible (though not enforced to keep modules decoupled)
- **Fast time-to-value**: No Alembic migrations needed for initial deployment — `create_all()` handles table creation

### Negative

- **No relational querying on item fields**: Can't SQL-query by message content, role, or tool call type without JSON functions — acceptable since ChatKit SDK handles all querying through the Store interface
- **Storage growth**: JSON blobs are larger than normalized columns — mitigated by conversation scale (hundreds, not millions of threads per user)
- **Anonymous thread sprawl**: Anonymous users create ephemeral threads with no cleanup — future enhancement: scheduled cleanup of anonymous threads older than 24h
- **No Alembic**: Initial table creation is fine, but schema changes later will need manual migration or Alembic setup

## Alternatives Considered

**Alternative A: Separate Database (e.g., dedicated SQLite or Redis)**
- Dedicated lightweight store for conversations, independent of auth database
- Why rejected: Adds operational complexity (new connection, new deployment config) for no benefit at this scale. Neon Postgres has ample capacity.

**Alternative B: Relational Column Mapping**
- Map each ThreadItem field to a database column (role, content, tool_name, etc.)
- Why rejected: ChatKit SDK evolves its item schema across versions. Relational mapping would require migration on every SDK update. JSON serialization is the pattern recommended in ChatKit documentation.

**Alternative C: MemoryStore (built-in, no persistence)**
- Use ChatKit's built-in in-memory store for development simplicity
- Why rejected: No persistence across server restarts. Spec requires conversation history across sessions (User Story 3).

## References

- Feature Spec: [specs/005-rag-chatbot-ui/spec.md](../../specs/005-rag-chatbot-ui/spec.md)
- Implementation Plan: [specs/005-rag-chatbot-ui/plan.md](../../specs/005-rag-chatbot-ui/plan.md)
- Data Model: [specs/005-rag-chatbot-ui/data-model.md](../../specs/005-rag-chatbot-ui/data-model.md)
- Research: [specs/005-rag-chatbot-ui/research.md](../../specs/005-rag-chatbot-ui/research.md) (R3)
- Related ADRs: ADR-001 (ChatKit SDK choice drives Store requirement)
- Evaluator Evidence: PHR 002 (plan phase)
