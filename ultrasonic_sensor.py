# ultrasonic_sensor.py
import RPi.GPIO as GPIO
import time
import logging

logger = logging.getLogger(__name__)

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin, name="sensor", config=None):
        self.trig_pin, self.echo_pin, self.name = trig_pin, echo_pin, name
        self.timeout = config.ULTRASONIC_TIMEOUT if config and hasattr(config, 'ULTRASONIC_TIMEOUT') else 0.05
        try:
            GPIO.setup(self.trig_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.echo_pin, GPIO.IN)
        except RuntimeError as e: logger.warning(f"GPIO setup warning for ultrasonic sensor {self.name}: {e}")
        except Exception as e: logger.error(f"GPIO setup error for ultrasonic sensor {self.name}: {e}")
        time.sleep(0.02) # Sensor settle time

    def read_distance_cm(self):
        try:
            GPIO.output(self.trig_pin, True); time.sleep(0.00001); GPIO.output(self.trig_pin, False)
            pulse_start_time, pulse_end_time = time.time(), time.time() # Init to current time
            
            # Wait for echo to go high
            timeout_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                pulse_start_time = time.time()
                if pulse_start_time - timeout_start > self.timeout:
                    logger.debug(f"Timeout waiting for echo pulse start on {self.name}")
                    return float('inf')
            
            # Wait for echo to go low
            timeout_start = time.time() # Reset timeout for receiving pulse
            while GPIO.input(self.echo_pin) == 1:
                pulse_end_time = time.time()
                if pulse_end_time - timeout_start > self.timeout:
                    logger.debug(f"Timeout waiting for echo pulse end on {self.name}")
                    return float('inf')

            duration = pulse_end_time - pulse_start_time
            distance = (duration * 34300) / 2.0 # Speed of sound in cm/s
            
            # Basic range check for typical HC-SR04 sensors
            if distance < 2.0 or distance > 400.0: 
                # logger.debug(f"Ultrasonic {self.name} out of range: {distance:.1f} cm")
                return float('inf')
            return distance
        except RuntimeError: # Usually when GPIO has been cleaned up
             logger.debug(f"Runtime error reading ultrasonic sensor {self.name}. GPIOs likely cleaned up.")
             return float('inf')
        except Exception as e: # Catch any other unexpected errors
            logger.error(f"Unexpected error reading ultrasonic sensor {self.name}: {e}", exc_info=False)
            return float('inf')