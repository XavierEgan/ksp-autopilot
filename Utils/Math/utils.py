def cyclic_error(desired_angle: float, current_angle: float, period: float = 360.0) -> float:
    half_period = period / 2.0
    return (desired_angle - current_angle + half_period) % period - half_period

def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))