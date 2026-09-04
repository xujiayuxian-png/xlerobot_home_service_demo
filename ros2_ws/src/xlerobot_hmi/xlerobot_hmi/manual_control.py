"""Thread-safe ownership state for bounded manual robot controls."""

from __future__ import annotations

import threading
from typing import Any


class ManualControlCoordinator:
    """Own manual actions, teleop and the task-runtime reservation as one domain."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._action_owner: object | None = None
        self._goal_handle: Any = None
        self._goal_future: Any = None
        self._result_future: Any = None
        self._cancel_requested = threading.Event()
        self._teleop_owner: object | None = None
        self._reserved = False
        self._drive_stop_latched: bool | None = None

    @property
    def drive_stop_latched(self) -> bool | None:
        with self._lock:
            return self._drive_stop_latched

    def set_drive_stop_latched(self, enabled: bool | None) -> None:
        with self._lock:
            self._drive_stop_latched = enabled

    def begin_action(self) -> object | None:
        with self._lock:
            if self._action_owner is not None or self._teleop_owner is not None:
                return None
            owner = object()
            self._action_owner = owner
            self._cancel_requested.clear()
            return owner

    def set_goal_future(self, owner: object, future: Any) -> bool:
        with self._lock:
            if self._action_owner is not owner:
                return False
            self._goal_future = future
            return True

    def set_goal_handle(self, owner: object, handle: Any) -> bool:
        with self._lock:
            if self._action_owner is not owner:
                return False
            self._goal_handle = handle
            self._goal_future = None
            return True

    def set_result_future(self, owner: object, future: Any) -> bool:
        with self._lock:
            if self._action_owner is not owner:
                return False
            self._result_future = future
            return True

    def finish_action(self, owner: object) -> bool:
        with self._lock:
            if self._action_owner is not owner:
                return False
            self._action_owner = None
            self._goal_handle = None
            self._goal_future = None
            self._result_future = None
            self._cancel_requested.clear()
            return True

    def request_cancel(self) -> None:
        with self._lock:
            if self._action_owner is not None:
                self._cancel_requested.set()

    def cancel_requested(self) -> bool:
        return self._cancel_requested.is_set()

    def action_active(self) -> bool:
        with self._lock:
            return self._action_owner is not None

    def claim_teleop(self, owner: object, *, task_active: bool) -> bool:
        with self._lock:
            if (
                task_active
                or self._teleop_owner is not None
                or self._action_owner is not None
                or self._reserved
            ):
                return False
            self._teleop_owner = owner
            return True

    def release_teleop(self, owner: object) -> bool:
        with self._lock:
            if self._teleop_owner is not owner:
                return False
            self._teleop_owner = None
            return True

    def owns_teleop(self, owner: object) -> bool:
        with self._lock:
            return self._teleop_owner is owner

    def teleop_active(self) -> bool:
        with self._lock:
            return self._teleop_owner is not None

    def set_reserved(self, enabled: bool) -> None:
        with self._lock:
            self._reserved = enabled

    def reserved(self) -> bool:
        with self._lock:
            return self._reserved

    def idle(self, *, task_active: bool) -> bool:
        with self._lock:
            return (
                not task_active
                and self._action_owner is None
                and self._teleop_owner is None
                and not self._reserved
            )
