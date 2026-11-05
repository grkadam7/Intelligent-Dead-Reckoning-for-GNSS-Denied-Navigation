"""
High-Rate Ring Buffer for Inertial Sensor Feeds.
SIH Problem Statement ID: 26168 | ISRO

Provides a bounded, thread-safe, pre-allocated circular buffer designed for
high-frequency (up to 200 Hz) IMU stream ingestion with minimal synchronization overhead.
"""

import threading
from typing import List, Optional, Tuple
import numpy as np

from src.edge.types import IDRIMUSample


class HighRateRingBuffer:
    """
    Thread-safe circular ring buffer for IDRIMUSample streams.
    Pre-allocates memory to eliminate dynamic heap allocations during high-rate streaming.
    """

    def __init__(self, capacity: int = 2048, overwrite_on_full: bool = True):
        """
        Args:
            capacity: Maximum number of samples the buffer can hold.
            overwrite_on_full: If True, overwrites oldest sample when full.
                              If False, drops new incoming sample when full.
        """
        if capacity <= 0:
            raise ValueError(f"Buffer capacity must be positive, got {capacity}")

        self.capacity = capacity
        self.overwrite_on_full = overwrite_on_full
        self._lock = threading.Lock()

        # Pre-allocated object storage
        self._buffer: List[Optional[IDRIMUSample]] = [None] * capacity
        self._head = 0  # Write pointer
        self._tail = 0  # Read pointer
        self._size = 0  # Current occupancy

        # Telemetry counters
        self._total_pushed = 0
        self._total_popped = 0
        self._total_dropped = 0

    def push(self, sample: IDRIMUSample) -> bool:
        """
        Pushes a single sample into the ring buffer.
        
        Returns:
            True if accepted, False if dropped due to buffer full.
        """
        with self._lock:
            if self._size == self.capacity:
                if not self.overwrite_on_full:
                    self._total_dropped += 1
                    return False
                # Advance tail to drop oldest sample
                self._tail = (self._tail + 1) % self.capacity
                self._total_dropped += 1
                self._size -= 1

            self._buffer[self._head] = sample
            self._head = (self._head + 1) % self.capacity
            self._size += 1
            self._total_pushed += 1
            return True

    def pop(self) -> Optional[IDRIMUSample]:
        """
        Pops the oldest available sample from the buffer.
        
        Returns:
            IDRIMUSample or None if buffer is empty.
        """
        with self._lock:
            if self._size == 0:
                return None

            sample = self._buffer[self._tail]
            self._buffer[self._tail] = None
            self._tail = (self._tail + 1) % self.capacity
            self._size -= 1
            self._total_popped += 1
            return sample

    def pop_batch(self, max_samples: int = 100) -> List[IDRIMUSample]:
        """
        Pops up to `max_samples` oldest samples atomically.
        """
        with self._lock:
            count = min(max_samples, self._size)
            if count == 0:
                return []

            batch = []
            for _ in range(count):
                sample = self._buffer[self._tail]
                self._buffer[self._tail] = None
                self._tail = (self._tail + 1) % self.capacity
                batch.append(sample)

            self._size -= count
            self._total_popped += count
            return batch

    def peek(self) -> Optional[IDRIMUSample]:
        """Peeks at the oldest sample without removing it."""
        with self._lock:
            if self._size == 0:
                return None
            return self._buffer[self._tail]

    def peek_newest(self) -> Optional[IDRIMUSample]:
        """Peeks at the most recently pushed sample without removing it."""
        with self._lock:
            if self._size == 0:
                return None
            prev_idx = (self._head - 1) % self.capacity
            return self._buffer[prev_idx]

    def clear(self) -> None:
        """Empties the buffer and resets pointers."""
        with self._lock:
            self._buffer = [None] * self.capacity
            self._head = 0
            self._tail = 0
            self._size = 0

    @property
    def size(self) -> int:
        with self._lock:
            return self._size

    @property
    def is_empty(self) -> bool:
        with self._lock:
            return self._size == 0

    @property
    def is_full(self) -> bool:
        with self._lock:
            return self._size >= self.capacity

    @property
    def dropped_count(self) -> int:
        with self._lock:
            return self._total_dropped

    @property
    def pushed_count(self) -> int:
        with self._lock:
            return self._total_pushed

    @property
    def popped_count(self) -> int:
        with self._lock:
            return self._total_popped
