import threading


class ThreadPauser:
    def __init__(self):
        # An Event rather than a Condition: if Unity answers before the waiting thread
        # reaches wait(), a Condition.notify() is lost and the waiter blocks forever.
        # An Event stays set, so the late waiter returns straight away.
        self.event = threading.Event()
        self.result = None

    def sleep_until_resumed(self, timeout=None):
        """Returns False if the timeout expired before resume_with_result() was called."""
        return self.event.wait(timeout)

    def resume_with_result(self, result):
        self.result = result
        self.event.set()
