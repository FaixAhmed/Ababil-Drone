# pid_controller.py
import time
from utils import clamp
import logging

logger = logging.getLogger(__name__)

class PID:
    def __init__(self, kp, ki, kd, output_limits=(-10, 10), anti_windup_gain=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limits = output_limits
        self.last_time = None
        # anti_windup_gain is not explicitly used in current integral clamping, but good to have if strategy changes

    def compute(self, target, current, current_time=None):
        if current_time is None: current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            error_on_first_call = target - current
            self.prev_error = error_on_first_call
            return clamp(self.kp * error_on_first_call, self.output_limits[0], self.output_limits[1])

        dt = current_time - self.last_time
        if dt <= 0.000001: # Avoid division by zero or very small dt
            error_no_dt = target - current
            # logger.debug(f"PID dt too small or zero ({dt_original:.6f}), using proportional only.") # dt_original not defined here
            return clamp(self.kp * error_no_dt, self.output_limits[0], self.output_limits[1])

        error = target - current
        self.integral += error * dt
        if self.ki != 0:
            # Allow integral to contribute significantly but not unboundedly
            integral_term_limit_abs = abs(self.output_limits[1] * 0.75 / self.ki) if self.ki != 0 else float('inf') 
            self.integral = clamp(self.integral, -integral_term_limit_abs, integral_term_limit_abs)
        else: self.integral = 0.0

        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        self.last_time = current_time
        return clamp(output, self.output_limits[0], self.output_limits[1])

    def reset(self):
        self.prev_error = 0.0; self.integral = 0.0; self.last_time = None
        logger.debug("PID controller has been reset.")