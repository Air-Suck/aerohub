"""Minimal shared lifecycle for fleet behaviors."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class BehaviorStatus(IntEnum):
    REJECTED = 0
    RUNNING = 1
    PAUSED = 2
    SUCCEEDED = 3
    FAILED = 4
    CANCELED = 5


TERMINAL_STATUSES = frozenset({
    BehaviorStatus.REJECTED,
    BehaviorStatus.SUCCEEDED,
    BehaviorStatus.FAILED,
    BehaviorStatus.CANCELED,
})


class TransitionError(ValueError):
    pass


@dataclass(frozen=True)
class BehaviorSnapshot:
    task_id: str
    status: BehaviorStatus
    failure_reason: str = "NONE"


class BehaviorLifecycle:
    def __init__(self) -> None:
        self._snapshot: BehaviorSnapshot | None = None

    @property
    def snapshot(self) -> BehaviorSnapshot | None:
        return self._snapshot

    def start(self, task_id: str) -> BehaviorSnapshot:
        if self._snapshot is not None and self._snapshot.status not in TERMINAL_STATUSES:
            raise TransitionError("an active behavior already exists")
        self._snapshot = BehaviorSnapshot(task_id, BehaviorStatus.RUNNING)
        return self._snapshot

    def reject(self, task_id: str, reason: str = "INVALID_TASK") -> BehaviorSnapshot:
        if self._snapshot is not None and self._snapshot.status not in TERMINAL_STATUSES:
            raise TransitionError("cannot reject while a behavior is active")
        self._snapshot = BehaviorSnapshot(task_id, BehaviorStatus.REJECTED, reason)
        return self._snapshot

    def pause(self) -> BehaviorSnapshot:
        return self._transition({BehaviorStatus.RUNNING}, BehaviorStatus.PAUSED)

    def resume(self) -> BehaviorSnapshot:
        return self._transition({BehaviorStatus.PAUSED}, BehaviorStatus.RUNNING)

    def cancel(self) -> BehaviorSnapshot:
        return self._transition(
            {BehaviorStatus.RUNNING, BehaviorStatus.PAUSED}, BehaviorStatus.CANCELED
        )

    def succeed(self) -> BehaviorSnapshot:
        return self._transition({BehaviorStatus.RUNNING}, BehaviorStatus.SUCCEEDED)

    def fail(self, reason: str) -> BehaviorSnapshot:
        if not reason or reason == "NONE":
            raise TransitionError("failed behavior requires a reason")
        return self._transition(
            {BehaviorStatus.RUNNING, BehaviorStatus.PAUSED}, BehaviorStatus.FAILED, reason
        )

    def _transition(
        self,
        allowed: set[BehaviorStatus],
        target: BehaviorStatus,
        reason: str = "NONE",
    ) -> BehaviorSnapshot:
        if self._snapshot is None or self._snapshot.status not in allowed:
            current = "IDLE" if self._snapshot is None else self._snapshot.status.name
            raise TransitionError(f"cannot transition from {current} to {target.name}")
        self._snapshot = BehaviorSnapshot(self._snapshot.task_id, target, reason)
        return self._snapshot
