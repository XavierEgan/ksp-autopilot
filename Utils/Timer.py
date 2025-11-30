"""
Timer class for triggering things after a set duration
"""
class Timer:
    def __init__(self, duration_s: float):
        self.duration_s = duration_s
        self.elapsed_s = 0.0
        self.finished = False

    def update(self, delta_time: float):
        if not self.finished:
            self.elapsed_s += delta_time
            if self.elapsed_s >= self.duration_s:
                self.finished = True
    
    def reset(self):
        self.elapsed_s = 0.0
        self.finished = False
    
    def fraction_complete(self) -> float:
        return min(self.elapsed_s / self.duration_s, 1.0) if self.duration_s > 0 else 1.0