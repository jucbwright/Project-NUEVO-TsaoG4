from __future__ import annotations

import math
import threading
import time


class TaskHandle:
    """
    Handle for a custom background task.

    Typical usage:
        handle = my_sequence_1(robot, blocking=False)
        handle.wait()
        handle.is_finished()
        handle.cancel()
    """

    def __init__(self) -> None:
        self._finished = threading.Event()
        self._cancel = threading.Event()

    def wait(self, timeout: float = None) -> bool:
        """Block until the task finishes. Returns True if it finished before timeout."""
        return self._finished.wait(timeout=timeout)

    def is_finished(self) -> bool:
        """Return True when the task thread has finished."""
        return self._finished.is_set()

    def cancel(self) -> None:
        """Ask the task to stop as soon as it can."""
        self._cancel.set()

    def cancelled(self) -> bool:
        """Return True if cancel() has been requested."""
        return self._cancel.is_set()

    def sleep(self, seconds: float) -> bool:
        """
        Sleep in small steps so cancel() can interrupt long waits.
        Returns False if the task was cancelled before the sleep finished.
        """
        deadline = time.monotonic() + seconds
        while not self.cancelled():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.02, remaining))
        return False

    def _mark_finished(self) -> None:
        self._finished.set()


def run_task(worker, blocking: bool = True, timeout: float = None):
    """
    Run a custom task in a background thread.

    worker(task_handle) should contain the actual sequence logic.

    blocking=True  → returns bool (True=finished, False=timeout)
    blocking=False → returns TaskHandle
    """
    handle = TaskHandle()

    def runner() -> None:
        try:
            worker(handle)
        finally:
            handle._mark_finished()

    thread = threading.Thread(target=runner, daemon=True)
    thread.start()

    if blocking:
        return handle.wait(timeout=timeout)
    return handle


def densify_polyline(
    control_points: list[tuple[float, float]],
    spacing: float,
) -> list[tuple[float, float]]:
    """Insert evenly spaced points between the control points."""
    dense_points = [control_points[0]]
    for start, end in zip(control_points[:-1], control_points[1:]):
        start_x, start_y = start
        end_x, end_y = end
        dx = end_x - start_x
        dy = end_y - start_y
        segment_length = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(segment_length / spacing)))
        for step in range(1, steps + 1):
            ratio = step / steps
            dense_points.append((
                start_x + dx * ratio,
                start_y + dy * ratio,
            ))
    return dense_points
