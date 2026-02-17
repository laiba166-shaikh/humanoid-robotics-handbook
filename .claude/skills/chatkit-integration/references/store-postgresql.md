# PostgreSQL Store Implementation

Complete `Store` implementation using SQLAlchemy async with PostgreSQL for ChatKit conversation persistence.

## Table of Contents

1. [Database Models](#database-models)
2. [Store Implementation](#store-implementation)
3. [Database Setup](#database-setup)
4. [Migration Notes](#migration-notes)

## Database Models

```python
# server/models.py
from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey, Index
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class ThreadModel(Base):
    __tablename__ = "chatkit_threads"

    id = Column(String(32), primary_key=True)           # thr_{uuid_hex[:8]}
    user_id = Column(String(255), nullable=False, index=True)
    title = Column(String(500), nullable=True)
    metadata_json = Column(Text, nullable=False, default="{}")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    __table_args__ = (
        Index("ix_threads_user_created", "user_id", "created_at"),
    )


class ThreadItemModel(Base):
    __tablename__ = "chatkit_thread_items"

    id = Column(String(32), primary_key=True)            # msg_{uuid_hex[:8]}
    thread_id = Column(String(32), ForeignKey("chatkit_threads.id", ondelete="CASCADE"), nullable=False)
    item_json = Column(Text, nullable=False)              # Full ThreadItem serialized as JSON
    sort_key = Column(Integer, nullable=False, default=0) # Auto-increment per thread
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    __table_args__ = (
        Index("ix_items_thread_sort", "thread_id", "sort_key"),
        Index("ix_items_thread_created", "thread_id", "created_at"),
    )


class AttachmentModel(Base):
    __tablename__ = "chatkit_attachments"

    id = Column(String(32), primary_key=True)
    thread_id = Column(String(32), ForeignKey("chatkit_threads.id", ondelete="CASCADE"), nullable=True)
    filename = Column(String(500), nullable=True)
    content_type = Column(String(255), nullable=True)
    size_bytes = Column(Integer, nullable=True)
    metadata_json = Column(Text, nullable=False, default="{}")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
```

## Store Implementation

```python
# server/store.py
import json
from uuid import uuid4
from typing import Literal
from sqlalchemy import select, delete, func as sa_func
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, async_sessionmaker

from chatkit.store import Store, Page
from chatkit.types import Thread, ThreadItem, Attachment
from chatkit.errors import NotFoundError

from .models import Base, ThreadModel, ThreadItemModel, AttachmentModel


class PostgresStore(Store[dict]):
    """PostgreSQL-backed ChatKit store using SQLAlchemy async."""

    def __init__(self, database_url: str):
        # database_url: "postgresql+asyncpg://user:pass@localhost:5432/dbname"
        self.engine = create_async_engine(database_url, echo=False)
        self.session_factory = async_sessionmaker(self.engine, class_=AsyncSession, expire_on_commit=False)

    async def initialize(self):
        """Create tables if they don't exist."""
        async with self.engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

    # --- Thread operations ---

    async def generate_thread_id(self, context: dict) -> str:
        return f"thr_{uuid4().hex[:8]}"

    async def load_thread(self, thread_id: str, context: dict) -> Thread:
        async with self.session_factory() as session:
            row = await session.get(ThreadModel, thread_id)
            if not row:
                raise NotFoundError(f"Thread {thread_id} not found")
            return Thread(
                id=row.id,
                metadata=json.loads(row.metadata_json),
            )

    async def save_thread(self, thread: Thread, context: dict) -> None:
        async with self.session_factory() as session:
            row = await session.get(ThreadModel, thread.id)
            metadata_json = json.dumps(thread.metadata if thread.metadata else {})
            if row:
                row.metadata_json = metadata_json
            else:
                user_id = context.get("user_id", "anonymous")
                row = ThreadModel(
                    id=thread.id,
                    user_id=user_id,
                    metadata_json=metadata_json,
                )
                session.add(row)
            await session.commit()

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        async with self.session_factory() as session:
            # CASCADE handles items and attachments
            await session.execute(
                delete(ThreadModel).where(ThreadModel.id == thread_id)
            )
            await session.commit()

    async def load_threads(
        self, limit: int, after: str | None, order: Literal["asc", "desc"], context: dict
    ) -> Page[Thread]:
        async with self.session_factory() as session:
            stmt = select(ThreadModel)
            user_id = context.get("user_id")
            if user_id:
                stmt = stmt.where(ThreadModel.user_id == user_id)

            if after:
                cursor_row = await session.get(ThreadModel, after)
                if cursor_row:
                    if order == "desc":
                        stmt = stmt.where(ThreadModel.created_at < cursor_row.created_at)
                    else:
                        stmt = stmt.where(ThreadModel.created_at > cursor_row.created_at)

            order_col = ThreadModel.created_at.desc() if order == "desc" else ThreadModel.created_at.asc()
            stmt = stmt.order_by(order_col).limit(limit + 1)
            result = await session.execute(stmt)
            rows = result.scalars().all()

            has_more = len(rows) > limit
            rows = rows[:limit]
            threads = [
                Thread(id=r.id, metadata=json.loads(r.metadata_json))
                for r in rows
            ]
            return Page(
                data=threads,
                has_more=has_more,
                after=rows[-1].id if rows and has_more else None,
            )

    # --- Item operations ---

    async def generate_item_id(
        self, item_type: Literal["thread", "message", "tool_call", "workflow", "task", "attachment"],
        thread: Thread, context: dict
    ) -> str:
        prefixes = {
            "thread": "thr", "message": "msg", "tool_call": "tc",
            "workflow": "wf", "task": "tsk", "attachment": "atc",
        }
        prefix = prefixes.get(item_type, "itm")
        return f"{prefix}_{uuid4().hex[:8]}"

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        async with self.session_factory() as session:
            # Get next sort_key
            stmt = select(sa_func.coalesce(sa_func.max(ThreadItemModel.sort_key), 0)).where(
                ThreadItemModel.thread_id == thread_id
            )
            result = await session.execute(stmt)
            next_key = result.scalar() + 1

            row = ThreadItemModel(
                id=item.id,
                thread_id=thread_id,
                item_json=item.model_dump_json(),
                sort_key=next_key,
            )
            session.add(row)
            await session.commit()

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        async with self.session_factory() as session:
            row = await session.get(ThreadItemModel, item.id)
            if row:
                row.item_json = item.model_dump_json()
            else:
                await self.add_thread_item(thread_id, item, context)
                return
            await session.commit()

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        async with self.session_factory() as session:
            row = await session.get(ThreadItemModel, item_id)
            if not row or row.thread_id != thread_id:
                raise NotFoundError(f"Item {item_id} not found in thread {thread_id}")
            return ThreadItem.model_validate_json(row.item_json)

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        async with self.session_factory() as session:
            await session.execute(
                delete(ThreadItemModel).where(
                    ThreadItemModel.id == item_id,
                    ThreadItemModel.thread_id == thread_id,
                )
            )
            await session.commit()

    async def load_thread_items(
        self, thread_id: str, limit: int, after: str | None,
        order: Literal["asc", "desc"], context: dict
    ) -> Page[ThreadItem]:
        async with self.session_factory() as session:
            stmt = select(ThreadItemModel).where(ThreadItemModel.thread_id == thread_id)

            if after:
                cursor_row = await session.get(ThreadItemModel, after)
                if cursor_row:
                    if order == "desc":
                        stmt = stmt.where(ThreadItemModel.sort_key < cursor_row.sort_key)
                    else:
                        stmt = stmt.where(ThreadItemModel.sort_key > cursor_row.sort_key)

            order_col = ThreadItemModel.sort_key.desc() if order == "desc" else ThreadItemModel.sort_key.asc()
            stmt = stmt.order_by(order_col).limit(limit + 1)
            result = await session.execute(stmt)
            rows = result.scalars().all()

            has_more = len(rows) > limit
            rows = rows[:limit]
            items = [ThreadItem.model_validate_json(r.item_json) for r in rows]
            return Page(
                data=items,
                has_more=has_more,
                after=rows[-1].id if rows and has_more else None,
            )

    # --- Attachment metadata ---

    async def load_attachment(self, attachment_id: str, context: dict) -> Attachment:
        async with self.session_factory() as session:
            row = await session.get(AttachmentModel, attachment_id)
            if not row:
                raise NotFoundError(f"Attachment {attachment_id} not found")
            return Attachment(
                id=row.id,
                filename=row.filename,
                content_type=row.content_type,
                metadata=json.loads(row.metadata_json),
            )

    async def save_attachment(self, attachment: Attachment, context: dict) -> None:
        async with self.session_factory() as session:
            row = await session.get(AttachmentModel, attachment.id)
            if row:
                row.metadata_json = json.dumps(attachment.metadata or {})
            else:
                row = AttachmentModel(
                    id=attachment.id,
                    filename=attachment.filename,
                    content_type=attachment.content_type,
                    metadata_json=json.dumps(attachment.metadata or {}),
                )
                session.add(row)
            await session.commit()

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        async with self.session_factory() as session:
            await session.execute(
                delete(AttachmentModel).where(AttachmentModel.id == attachment_id)
            )
            await session.commit()
```

## Database Setup

```python
# server/db.py
from .store import PostgresStore

DATABASE_URL = "postgresql+asyncpg://user:password@localhost:5432/chatkit_db"

store = PostgresStore(database_url=DATABASE_URL)

# Call during app startup
# await store.initialize()
```

In FastAPI lifespan:

```python
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    await store.initialize()
    yield
    await store.engine.dispose()

app = FastAPI(lifespan=lifespan)
```

## Migration Notes

- Serialize full ThreadItem objects as JSON (`item_json` column) for forward compatibility
- When ChatKit SDK updates item schemas, existing JSON still deserializes correctly
- For Alembic migrations, add new columns as nullable with defaults
- The `sort_key` integer ensures stable ordering within a thread
- Use CASCADE deletes so removing a thread cleans up all items and attachments
