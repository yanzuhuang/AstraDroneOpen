"""Lock-agnostic single-slot mailbox for coalescing state snapshots."""


class LatestSampleMailbox:
    """Retain only the newest pending sample; caller owns synchronization."""

    def __init__(self):
        self._value = None
        self.input_count = 0
        self.replacement_count = 0

    @property
    def pending(self):
        return self._value is not None

    def push(self, value):
        replaced = self._value is not None
        self.input_count += 1
        self.replacement_count += int(replaced)
        self._value = value
        return replaced

    def pop(self):
        value = self._value
        self._value = None
        return value

    def clear(self):
        self._value = None
