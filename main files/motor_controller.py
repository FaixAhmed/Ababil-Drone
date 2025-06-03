# motor_controller.py
import RPi.GPIO as GPIO
import time
from utils import clamp
import logging

logger = logging.getLogger(__name__)

class MotorController:
    def __init__(self, config):
        self.config = config
        self.esc_pwms = []
        self._setup_gpio()

    def _setup_gpio(self):
        try:
            current_mode = GPIO.getmode()
            if current_mode is None: GPIO.setmode(GPIO.BCM)
            elif current_mode != GPIO.BCM:
                 logger.warning(f"GPIO mode was {current_mode}, re-setting to BCM.")
                 GPIO.setmode(GPIO.BCM)
        except Exception: # Fallback if getmode fails (e.g. on non-Pi or RPi.GPIO stub)
            try:
                GPIO.setmode(GPIO.BCM)
            except Exception as e_setmode: # Catch specific errors from setmode if necessary
                logger.error(f"Failed to set GPIO mode to BCM: {e_setmode}")


        GPIO.setwarnings(False)
        for pin in self.config.ESC_PINS:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, self.config.PWM_FREQ)
            pwm.start(self.config.MIN_PWM_DUTY)
            self.esc_pwms.append(pwm)
        logger.info("Motor GPIOs and PWMs initialized.")

    def set_motor_duty_cycle(self, motor_index, duty_cycle):
        safe_duty = clamp(duty_cycle, self.config.MIN_PWM_DUTY, self.config.MAX_PWM_DUTY)
        if 0 <= motor_index < len(self.esc_pwms):
            self.esc_pwms[motor_index].ChangeDutyCycle(safe_duty)

    def set_all_motors_duty_cycle(self, duty_cycles):
        for i, duty in enumerate(duty_cycles): self.set_motor_duty_cycle(i, duty)

    def arm_motors_sequence(self):
        logger.info("Motor arming sequence: setting to base throttle briefly.")
        for esc in self.esc_pwms: esc.ChangeDutyCycle(self.config.BASE_THROTTLE_DUTY)
        time.sleep(0.2) 

    def disarm_motors(self):
        logger.info("Motors disarmed: setting to min duty cycle.")
        for esc in self.esc_pwms: esc.ChangeDutyCycle(self.config.MIN_PWM_DUTY)

    def stop_motors_completely(self):
        logger.info("Stopping motors completely and releasing PWMs.")
        for esc in self.esc_pwms:
            try: esc.ChangeDutyCycle(0); esc.stop()
            except Exception as e: logger.error(f"Error stopping PWM for an ESC: {e}")
        self.esc_pwms.clear()