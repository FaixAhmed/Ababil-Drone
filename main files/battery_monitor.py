# battery_monitor.py
import time
from threading import Thread, Lock
from utils import clamp
import logging

logger = logging.getLogger(__name__)

class BatteryMonitor:
    def __init__(self, config, is_armed_or_arming_func_ref): # Pass a function to check if drone is armed
        self.config = config; self.lock = Lock(); self.current_percentage = 100.0
        self._running = False; self._thread = None; self.is_armed_or_arming_func = is_armed_or_arming_func_ref

    def _get_battery_percentage_simulated_drain(self):
        # Simulate battery drain for testing
        sim_drain_rate_armed = 0.1 # % per check interval when armed
        sim_drain_rate_idle = 0.01 # % per check interval when idle
        
        drain_rate = sim_drain_rate_armed if self.is_armed_or_arming_func() else sim_drain_rate_idle
        
        with self.lock:
            # Simulate a slight recharge if it goes too low and then disarmed, for testing RTH again
            if not self.is_armed_or_arming_func() and self.current_percentage < self.config.LOW_BATTERY_THRESHOLD_PERCENT - 2: # If significantly below low_bat and disarmed
                self.current_percentage += 0.05 # Very slow recovery
            else:
                self.current_percentage -= drain_rate
            self.current_percentage = clamp(self.current_percentage, 0, 100)
            return self.current_percentage

    def _monitor_thread_func(self):
        logger.info("Battery monitor thread started.")
        while self._running:
            level = self._get_battery_percentage_simulated_drain() # Using simulation
            # Actual ADC reading and voltage-to-percentage conversion would go here
            with self.lock: self.current_percentage = level
            logger.debug(f"Battery Level (Simulated): {self.current_percentage:.1f}%")
            time.sleep(self.config.BATTERY_CHECK_INTERVAL)
        logger.info("Battery monitor thread stopped.")

    def start(self): self._running=True; self._thread=Thread(target=self._monitor_thread_func,daemon=True,name="BatteryMonitorThread"); self._thread.start()
    def stop(self):
        self._running=False
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=1.0)
    def get_level_percentage(self):
        with self.lock: return self.current_percentage