# config.py
import json
import logging
import os

logger = logging.getLogger(__name__)

class DroneConfig:
    def __init__(self, config_file_path="drone_config.json"):
        # --- Default Values ---
        self.ESC_PINS = [17, 18, 27, 22]; self.PWM_FREQ = 50; self.MIN_PWM_DUTY = 5.0; self.MAX_PWM_DUTY = 10.0
        self.BASE_THROTTLE_DUTY = 6.5; self.THROTTLE_RANGE_MODIFIER = 2.5
        self.ULTRASONIC_PINS = {'front': {'trig': 5, 'echo': 6}, 'left': {'trig': 13, 'echo': 19}, 'right': {'trig': 20, 'echo': 21}, 'back': {'trig': 23, 'echo': 24}}
        self.MAX_SPEED_PROXY = 1.0; self.MIN_SAFE_DISTANCE_CM = 30; self.MAX_SAFE_DISTANCE_CM = 200
        self.SPEED_TO_DISTANCE_FACTOR_CM_PER_UNIT = 70; self.ULTRASONIC_TIMEOUT = 0.05
        self.RETURN_HOME_MIN_DIST_METERS = 5; self.LOW_BATTERY_THRESHOLD_PERCENT = 15.0; self.RTH_ALTITUDE_METERS = 20
        self.VIDEO_STREAM_PORT = 8000; self.VIDEO_WIDTH = 640; self.VIDEO_HEIGHT = 480; self.VIDEO_FPS = 30; self.HUD_TEXT_COLOR = [0, 255, 0] # JSON uses lists for tuples
        self.GPS_PORT = "/dev/ttyAMA0"; self.GPS_BAUDRATE = 9600; self.GPS_TIMEOUT = 1; self.GPS_POLL_INTERVAL = 0.2; self.EARTH_RADIUS_KM = 6371
        self.PS5_BUTTON_X = 0; self.PS5_BUTTON_O = 1; self.BOOT_HOLD_TIME_SEC = 5; self.SHUTDOWN_HOLD_TIME_SEC = 3; self.CONTROLLER_REINIT_INTERVAL = 5.0
        self.PID_PITCH_KP, self.PID_PITCH_KI, self.PID_PITCH_KD = 1.5, 0.01, 0.6; self.PID_ROLL_KP, self.PID_ROLL_KI, self.PID_ROLL_KD = 1.5, 0.01, 0.6
        self.PID_YAW_KP, self.PID_YAW_KI, self.PID_YAW_KD = 2.0, 0.01, 0.8; self.PID_PITCH_LIMITS = [-3,3]; self.PID_ROLL_LIMITS = [-3,3]; self.PID_YAW_LIMITS = [-5,5]
        self.CONTROLLER_PITCH_ANGLE_SCALE = 20.0; self.CONTROLLER_ROLL_ANGLE_SCALE = 20.0; self.CONTROLLER_YAW_RATE_SCALE = 90.0
        self.MPU6050_ADDRESS = 0x68; self.ALPHA_ACCEL = 0.98; self.ALPHA_MAG = 0.02; self.IMU_READ_INTERVAL = 0.005
        self.IMU_ACCEL_OFFSETS = {'x': 0.0, 'y': 0.0, 'z': 0.0}; self.IMU_GYRO_OFFSETS = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.IMU_MAG_OFFSETS = {'x': 0.0, 'y': 0.0, 'z': 0.0}; self.IMU_MAG_SCALE_FACTORS = {'x': 1.0, 'y': 1.0, 'z': 1.0}
        self.BATTERY_CHECK_INTERVAL = 5.0
        self.MAIN_LOOP_HZ = 50; self.LOG_LEVEL = "INFO"; self.LOG_FILE = "drone_flight.log"
        self.ENABLE_RC_FAILSAFE = True; self.RC_FAILSAFE_TIMEOUT_SEC = 3.0; self.RC_FAILSAFE_ACTION = "RTH"
        self.ENABLE_GPS_FAILSAFE = True; self.GPS_FAILSAFE_MIN_FIX_QUALITY = 1; self.GPS_FAILSAFE_MIN_SATELLITES = 4
        self.GPS_FAILSAFE_TIMEOUT_SEC = 5.0; self.GPS_FAILSAFE_ACTION = "LAND"
        self.CRITICAL_BATTERY_THRESHOLD_PERCENT = 7.0
        self.GCS_DRONE_IP = "0.0.0.0"; self.GCS_COMMAND_PORT = 14550; self.GCS_TELEMETRY_TARGET_IP = "192.168.1.100"; self.GCS_TELEMETRY_TARGET_PORT = 14551
        self.GCS_TELEMETRY_SEND_INTERVAL_SEC = 0.2
        self.STALE_IMU_DATA_THRESHOLD_SEC = 0.2; self.STALE_GPS_DATA_THRESHOLD_SEC = 2.0

        self.config_file_path = config_file_path
        self._load_from_file()

    def _load_from_file(self):
        if not os.path.exists(self.config_file_path):
            logger.warning(f"Config file '{self.config_file_path}' not found. Using defaults & saving new file.")
            self.save_to_file()
            return
        try:
            with open(self.config_file_path, 'r') as f:
                config_data = json.load(f)
            for key, value in config_data.items():
                if hasattr(self, key):
                    if isinstance(getattr(self, key), tuple) and isinstance(value, list): setattr(self, key, tuple(value))
                    elif isinstance(getattr(self, key), dict) and isinstance(value, dict): setattr(self, key, value)
                    else: setattr(self, key, value)
            logger.info(f"Successfully loaded configuration from '{self.config_file_path}'.")
        except json.JSONDecodeError as e: logger.error(f"Error decoding JSON from '{self.config_file_path}': {e}. Using defaults.")
        except Exception as e: logger.error(f"Error loading config from '{self.config_file_path}': {e}. Using defaults.")

    def save_to_file(self, file_path=None):
        save_path = file_path if file_path else self.config_file_path
        config_data = {attr: getattr(self, attr) for attr in dir(self) 
                       if not callable(getattr(self, attr)) and not attr.startswith('_') 
                       and attr != 'config_file_path'}
        try:
            with open(save_path, 'w') as f:
                json.dump(config_data, f, indent=4, sort_keys=True)
            logger.info(f"Current configuration saved to '{save_path}'.")
        except Exception as e: logger.error(f"Error saving configuration to '{save_path}': {e}")