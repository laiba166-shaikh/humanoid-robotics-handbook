import json
import logging
from uuid import uuid4

from pydantic import TypeAdapter
from sqlalchemy import select, delete, func as sa_func
from sqlalchemy.ext.asyncio import async_sessionmaker, AsyncSession

from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Attachment, Page
from app.chatkit.models import ThreadModel, ThreadItemModel

_thread_item_adapter = TypeAdapter(ThreadItem)

logger = logging.getLogger(__name__)


class NotFoundError(Exception):
    """Raised when a thread or item is not found in the store."""


class PostgresStore(Store[dict]):
    """PostgreSQL-backed ChatKit store using the existing SQLAlchemy engine."""

    def __init__(self, session_factory: async_sessionmaker[AsyncSession]):
        self.session_factory = session_factory

    # --- Thread operations ---

    def generate_thread_id(self, context: dict) -> str:
        return f"thr_{uuid4().hex[:8]}"

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        async with self.session_factory() as session:
            row = await session.get(ThreadModel, str(thread_id))
            if not row:
                raise NotFoundError(f"Thread {thread_id} not found")
            return ThreadMetadata(
                id=row.id,
                created_at=row.created_at,
                metadata=json.loads(row.metadata_json),
            )

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        async with self.session_factory() as session:
            row = await session.get(ThreadModel, str(thread.id))
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
            await session.execute(
                delete(ThreadModel).where(ThreadModel.id == str(thread_id))
            )
            await session.commit()

    async def load_threads(
        self, limit: int, after: str | None, order: str, context: dict
    ) -> Page[ThreadMetadata]:
        async with self.session_factory() as session:
            stmt = select(ThreadModel)

            user_id = context.get("user_id")
            if user_id and user_id != "anonymous":
                stmt = stmt.where(ThreadModel.user_id == user_id)
            else:
                # Anonymous users see no persisted threads
                return Page(data=[], has_more=False, after=None)

            if after:
                cursor_row = await session.get(ThreadModel, str(after))
                if cursor_row:
                    if order == "desc":
                        stmt = stmt.where(ThreadModel.created_at < cursor_row.created_at)
                    else:
                        stmt = stmt.where(ThreadModel.created_at > cursor_row.created_at)

            order_col = (
                ThreadModel.created_at.desc() if order == "desc" else ThreadModel.created_at.asc()
            )
            stmt = stmt.order_by(order_col).limit(limit + 1)
            result = await session.execute(stmt)
            rows = result.scalars().all()

            has_more = len(rows) > limit
            rows = rows[:limit]
            threads = [
                ThreadMetadata(id=r.id, created_at=r.created_at, metadata=json.loads(r.metadata_json))
                for r in rows
            ]
            return Page(
                data=threads,
                has_more=has_more,
                after=rows[-1].id if rows and has_more else None,
            )

    # --- Item operations ---

    def generate_item_id(
        self,
        item_type: str,
        thread: ThreadMetadata,
        context: dict,
    ) -> str:
        prefixes = {
            "thread": "thr",
            "message": "msg",
            "tool_call": "tc",
            "workflow": "wf",
            "task": "tsk",
            "attachment": "atc",
        }
        prefix = prefixes.get(item_type, "itm")
        return f"{prefix}_{uuid4().hex[:8]}"

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        tid = str(thread_id)
        iid = str(item.id)
        async with self.session_factory() as session:
            stmt = select(
                sa_func.coalesce(sa_func.max(ThreadItemModel.sort_key), 0)
            ).where(ThreadItemModel.thread_id == tid)
            result = await session.execute(stmt)
            next_key = (result.scalar() or 0) + 1

            row = ThreadItemModel(
                id=iid,
                thread_id=tid,
                item_json=item.model_dump_json(),
                sort_key=next_key,
            )
            session.add(row)
            await session.commit()

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        async with self.session_factory() as session:
            row = await session.get(ThreadItemModel, str(item.id))
            if row:
                row.item_json = item.model_dump_json()
                await session.commit()
            else:
                await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        async with self.session_factory() as session:
            row = await session.get(ThreadItemModel, str(item_id))
            if not row or row.thread_id != str(thread_id):
                raise NotFoundError(f"Item {item_id} not found in thread {thread_id}")
            return _thread_item_adapter.validate_json(row.item_json)

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        async with self.session_factory() as session:
            await session.execute(
                delete(ThreadItemModel).where(
                    ThreadItemModel.id == str(item_id),
                    ThreadItemModel.thread_id == str(thread_id),
                )
            )
            await session.commit()

    async def load_thread_items(
        self, thread_id: str, after: str | None, limit: int,
        order: str, context: dict,
    ) -> Page[ThreadItem]:
        tid = str(thread_id)
        async with self.session_factory() as session:
            stmt = select(ThreadItemModel).where(
                ThreadItemModel.thread_id == tid
            )

            if after:
                cursor_row = await session.get(ThreadItemModel, str(after))
                if cursor_row:
                    if order == "desc":
                        stmt = stmt.where(ThreadItemModel.sort_key < cursor_row.sort_key)
                    else:
                        stmt = stmt.where(ThreadItemModel.sort_key > cursor_row.sort_key)

            order_col = (
                ThreadItemModel.sort_key.desc()
                if order == "desc"
                else ThreadItemModel.sort_key.asc()
            )
            stmt = stmt.order_by(order_col).limit(limit + 1)
            result = await session.execute(stmt)
            rows = result.scalars().all()

            has_more = len(rows) > limit
            rows = rows[:limit]
            items = [_thread_item_adapter.validate_json(r.item_json) for r in rows]
            return Page(
                data=items,
                has_more=has_more,
                after=rows[-1].id if rows and has_more else None,
            )

    # --- Attachment metadata (minimal implementation) ---

    async def load_attachment(self, attachment_id: str, context: dict) -> Attachment:
        raise NotFoundError(f"Attachment {attachment_id} not found (attachments not supported)")

    async def save_attachment(self, attachment: Attachment, context: dict) -> None:
        pass

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        pass
