"""Immutable transition handoff and single-threaded ordered persistence."""

from collections import deque
from collections.abc import Mapping
from copy import deepcopy
import math
import threading
import time
from types import MappingProxyType


def _freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return deepcopy(value)


def _thaw(value):
    if isinstance(value, Mapping):
        return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw(item) for item in value]
    return deepcopy(value)


class ImmutableTransitionRecord(Mapping):
    """Deeply immutable snapshot of one fully formed action interval."""

    def __init__(self, record):
        if isinstance(record, ImmutableTransitionRecord):
            self._record = record._record
        elif isinstance(record, Mapping):
            self._record = _freeze(record)
        else:
            raise TypeError("transition record must be a mapping")

    def __getitem__(self, key):
        return self._record[key]

    def __iter__(self):
        return iter(self._record)

    def __len__(self):
        return len(self._record)

    def to_mutable(self):
        return _thaw(self._record)


class OrderedTransitionWriter:
    """Persist immutable formed transitions in submission order off-policy-clock.

    Submission validates the r02 identity/generation/terminal ordering contract.
    The single worker may be slow, but its persistence callback never owns Actor
    admission.  ``drain()`` is required before Episode boundary finalization.
    """

    def __init__(self, persist_transition, wall_clock=time.monotonic):
        if not callable(persist_transition):
            raise ValueError("persist_transition must be callable")
        self._persist_transition = persist_transition
        self._wall_clock = wall_clock
        self._condition = threading.Condition(threading.RLock())
        self._queue = deque()
        self._in_progress = False
        self._stopping = False
        self._error = None
        self._expected_by_episode = {}
        self._submitted_count = 0
        self._persisted_count = 0
        self._maximum_pending_count = 0
        self._worker = threading.Thread(
            target=self._run,
            name="learning_speed_ordered_replay_writer",
            daemon=True,
        )
        self._worker.start()

    @property
    def submitted_count(self):
        with self._condition:
            return self._submitted_count

    @property
    def persisted_count(self):
        with self._condition:
            return self._persisted_count

    @property
    def pending_count(self):
        with self._condition:
            return len(self._queue) + int(self._in_progress)

    @property
    def maximum_pending_count(self):
        with self._condition:
            return self._maximum_pending_count

    def _raise_if_failed_locked(self):
        if self._error is not None:
            raise RuntimeError("ordered Replay writer failed") from self._error

    def submit(self, record, payload=None):
        immutable = (
            record
            if isinstance(record, ImmutableTransitionRecord)
            else ImmutableTransitionRecord(record)
        )
        episode_id = str(immutable["episode_id"])
        step_index = int(immutable["step_index"])
        request_id = int(immutable["request_id"])
        generation = int(payload.get("reset_generation", -1)) if isinstance(
            payload, Mapping
        ) else -1
        if not episode_id or step_index < 0 or request_id != step_index + 1:
            raise ValueError("formed transition identity is invalid")
        if generation < 0:
            raise ValueError("formed transition generation is invalid")
        with self._condition:
            self._raise_if_failed_locked()
            if self._stopping:
                raise RuntimeError("ordered Replay writer is closed")
            expected = self._expected_by_episode.get(episode_id)
            if expected is None:
                if step_index != 0:
                    raise ValueError("ordered writer Episode must begin at step 0")
                expected = {
                    "next_step_index": 0,
                    "reset_generation": generation,
                    "closed": False,
                }
            if expected["closed"]:
                raise ValueError("formed transition arrived after terminal boundary")
            if generation != expected["reset_generation"]:
                raise ValueError("formed transition generation changed within Episode")
            if step_index != expected["next_step_index"]:
                raise ValueError("formed transition sequence is not contiguous")
            expected["next_step_index"] += 1
            expected["closed"] = bool(
                immutable.get("terminated", False)
                or immutable.get("truncated", False)
            )
            self._expected_by_episode[episode_id] = expected
            self._queue.append((immutable, deepcopy(payload)))
            self._submitted_count += 1
            self._maximum_pending_count = max(
                self._maximum_pending_count,
                len(self._queue) + int(self._in_progress),
            )
            self._condition.notify_all()

    def _run(self):
        while True:
            with self._condition:
                while not self._queue and not self._stopping:
                    self._condition.wait()
                if self._stopping and not self._queue:
                    return
                record, payload = self._queue.popleft()
                self._in_progress = True
            try:
                self._persist_transition(record.to_mutable(), payload)
            except BaseException as caught:  # surfaced synchronously by drain/submit
                with self._condition:
                    self._error = caught
                    self._queue.clear()
            finally:
                with self._condition:
                    if self._error is None:
                        self._persisted_count += 1
                    self._in_progress = False
                    self._condition.notify_all()
                    if self._error is not None:
                        return

    def drain(self, timeout_sec=None):
        deadline = None
        if timeout_sec is not None:
            timeout = float(timeout_sec)
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError("writer drain timeout must be positive and finite")
            deadline = self._wall_clock() + timeout
        with self._condition:
            while (self._queue or self._in_progress) and self._error is None:
                remaining = (
                    None if deadline is None else deadline - self._wall_clock()
                )
                if remaining is not None and remaining <= 0.0:
                    raise TimeoutError("ordered Replay writer drain timed out")
                self._condition.wait(remaining)
            self._raise_if_failed_locked()

    def close(self, timeout_sec=None):
        self.drain(timeout_sec)
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join(timeout=timeout_sec)
        if self._worker.is_alive():
            raise TimeoutError("ordered Replay writer did not stop")

