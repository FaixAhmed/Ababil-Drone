# obstacle_avoidance.py
from ultrasonic_sensor import UltrasonicSensor
from utils import clamp
import logging

logger = logging.getLogger(__name__)

class ObstacleAvoidance:
    def __init__(self, config):
        self.config = config; self.sensors = {}
        for direction, pins_dict in self.config.ULTRASONIC_PINS.items():
            self.sensors[direction] = UltrasonicSensor(pins_dict['trig'],pins_dict['echo'],name=direction,config=self.config)
        logger.info("Obstacle avoidance ultrasonic sensors initialized.")

    def get_distances(self):
        return {direction: sensor.read_distance_cm() for direction, sensor in self.sensors.items()}

    def calculate_avoidance_maneuver(self, distances_cm, current_pitch_input, current_roll_input, current_speed_proxy):
        avoid_p, avoid_r, detected = 0.0, 0.0, False
        norm_spd = clamp(abs(current_speed_proxy) / self.config.MAX_SPEED_PROXY if self.config.MAX_SPEED_PROXY > 0 else 0, 0, 1)
        dyn_safe_dist = clamp(self.config.MIN_SAFE_DISTANCE_CM + norm_spd * self.config.SPEED_TO_DISTANCE_FACTOR_CM_PER_UNIT,
                              self.config.MIN_SAFE_DISTANCE_CM, self.config.MAX_SAFE_DISTANCE_CM)
        for direction, dist_cm in distances_cm.items():
            if dist_cm < dyn_safe_dist:
                detected = True; logger.debug(f"OBSTACLE {direction.upper()}: {dist_cm:.1f}cm (Safe:{dyn_safe_dist:.1f}cm)")
                # Simple avoidance: if moving towards obstacle or stationary, try to move away slightly
                # More sophisticated logic would consider the magnitude of current_pitch/roll input
                if direction=='front' and (current_pitch_input >= 0): avoid_p = -0.3 # Gentle back
                elif direction=='back' and (current_pitch_input <= 0): avoid_p = 0.3  # Gentle forward
                elif direction=='left' and (current_roll_input <= 0): avoid_r = 0.3   # Gentle right
                elif direction=='right' and (current_roll_input >= 0): avoid_r = -0.3  # Gentle left
        return avoid_p, avoid_r, detected, dyn_safe_dist