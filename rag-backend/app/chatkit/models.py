from sqlalchemy import Column, String, DateTime, Text, Integer, ForeignKey, Index
from sqlalchemy.sql import func

from app.database import Base


class ThreadModel(Base):
    __tablename__ = "chatkit_threads"

    id = Column(String(128), primary_key=True)
    user_id = Column(String(255), nullable=False, index=True)
    title = Column(String(500), nullable=True)
    metadata_json = Column(Text, nullable=False, default="{}")
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    __table_args__ = (
        Index("ix_chatkit_threads_user_created", "user_id", "created_at"),
    )


class ThreadItemModel(Base):
    __tablename__ = "chatkit_thread_items"

    id = Column(String(128), primary_key=True)
    thread_id = Column(
        String(128),
        ForeignKey("chatkit_threads.id", ondelete="CASCADE"),
        nullable=False,
    )
    item_json = Column(Text, nullable=False)
    sort_key = Column(Integer, nullable=False, default=0)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    __table_args__ = (
        Index("ix_chatkit_items_thread_sort", "thread_id", "sort_key"),
        Index("ix_chatkit_items_thread_created", "thread_id", "created_at"),
    )
