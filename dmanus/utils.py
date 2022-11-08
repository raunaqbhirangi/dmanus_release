import time


class Rate:
    """
    Maintains constant control rate for POMDP loop
    """

    def __init__(self, frequency):
        self._period = 1.0 / frequency
        self._last = time.time()
        self.frequency = frequency

    def sleep(self):
        current_delta = time.time() - self._last
        sleep_time = max(0, self._period - current_delta)
        if sleep_time:
            time.sleep(sleep_time)
        self._last = time.time()
