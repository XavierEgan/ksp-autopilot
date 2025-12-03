class CentralDifference:
    def __init__(self):
        self.previous_value = 0.0
        self.curent_value = 0.0
        self.next_value = 0.0
    
    def get_curent(self):
        return self.curent_value
    
    def set_next(self, value: float):
        self.previous_value = self.curent_value
        self.curent_value = self.next_value
        self.next_value = value
    
    def get_current_derivative(self, dt: float):
        if dt <= 0.0:
            return 0.0
        derivative = (self.next_value - self.previous_value) / (2.0 * dt)
        return derivative