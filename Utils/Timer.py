"""
Timer class for triggering things after a set duration
"""
class CountDownTimer:
    def __init__(self, duration_s: float):
        self.duration_s = duration_s
        self.elapsed_s = 0.0
        self.finished = False
        self._just_finished = True

    """
    Add delta_time to the timer, and mark as finished if duration is reached
    """
    def update(self, delta_time: float):
        if not self.finished:
            self.elapsed_s += delta_time
            if self.elapsed_s >= self.duration_s:
                self.finished = True
    """
    Reset the timer
    """
    def reset(self):
        self.elapsed_s = 0.0
        self.finished = False
        self._just_finished = True

    """
    Only returns true once after finishing
    """
    def just_finished(self) -> bool:
        finished = self.finished and self._just_finished
        if finished:
            self._just_finished = False
        return finished

    """
    Returns the fraction of the duration that has elapsed, clamped between 0 and 1
    """
    def fraction_complete(self) -> float:
        return min(self.elapsed_s / self.duration_s, 1.0) if self.duration_s > 0 else 1.0