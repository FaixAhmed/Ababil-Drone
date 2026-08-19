# sensor_fusion.py
import time
from mpu6050 import mpu6050 # Ensure this library is installed and matches your hardware
from threading import Thread, Lock
import math
import logging

logger = logging.getLogger(__name__)

class SensorFusion:
    def __init__(self, config):
        self.config = config; self.mpu = None; self.lock = Lock()
        self.pitch, self.roll, self.yaw = 0.0, 0.0, 0.0
        self.gyro_x, self.gyro_y, self.gyro_z = 0.0, 0.0, 0.0
        self.accel_x, self.accel_y, self.accel_z = 0.0, 0.0, 0.0
        self._running = False; self._thread = None
        self._last_imu_read_time = 0; self._last_successful_read_time = 0
        self._filtered_pitch, self._filtered_roll, self._filtered_yaw = 0.0, 0.0, 0.0
        
        # Ensure calibration data is dictionaries, copy from config
        self.accel_offsets = self.config.IMU_ACCEL_OFFSETS.copy() if isinstance(self.config.IMU_ACCEL_OFFSETS, dict) else {'x':0,'y':0,'z':0}
        self.gyro_offsets = self.config.IMU_GYRO_OFFSETS.copy() if isinstance(self.config.IMU_GYRO_OFFSETS, dict) else {'x':0,'y':0,'z':0}
        self.mag_offsets = self.config.IMU_MAG_OFFSETS.copy() if isinstance(self.config.IMU_MAG_OFFSETS, dict) else {'x':0,'y':0,'z':0}
        self.mag_scales = self.config.IMU_MAG_SCALE_FACTORS.copy() if isinstance(self.config.IMU_MAG_SCALE_FACTORS, dict) else {'x':1,'y':1,'z':1}

        self.calibration_samples_accel, self.calibration_samples_gyro, self.calibration_samples_mag = [], [], []
        self.is_calibrating_gyro, self.is_calibrating_accel, self.is_calibrating_mag = False, False, False

        try:
            self.mpu = mpu6050(self.config.MPU6050_ADDRESS) # mpu6050 lib often takes int address
            logger.info(f"MPU6050 IMU initialized at address 0x{self.config.MPU6050_ADDRESS:X}.")
        except Exception as e: logger.error(f"Error initializing MPU6050: {e}. Ensure it's connected.")

    def _apply_calibration(self, accel_raw, gyro_raw, mag_raw):
        ax_cal = accel_raw.get('x',0) - self.accel_offsets.get('x',0)
        ay_cal = accel_raw.get('y',0) - self.accel_offsets.get('y',0)
        az_cal = accel_raw.get('z',0) - self.accel_offsets.get('z',0)
        gx_cal = gyro_raw.get('x',0) - self.gyro_offsets.get('x',0)
        gy_cal = gyro_raw.get('y',0) - self.gyro_offsets.get('y',0)
        gz_cal = gyro_raw.get('z',0) - self.gyro_offsets.get('z',0)
        mx_cal = (mag_raw[0] - self.mag_offsets.get('x',0)) * self.mag_scales.get('x',1)
        my_cal = (mag_raw[1] - self.mag_offsets.get('y',0)) * self.mag_scales.get('y',1)
        mz_cal = (mag_raw[2] - self.mag_offsets.get('z',0)) * self.mag_scales.get('z',1)
        return {'x':ax_cal,'y':ay_cal,'z':az_cal}, {'x':gx_cal,'y':gy_cal,'z':gz_cal}, (mx_cal,my_cal,mz_cal)

    def _read_magnetometer_raw(self): return 100, 50, 0 # Placeholder, replace with actual sensor read

    def _calculate_mag_heading(self, mag_x_cal, mag_y_cal):
        heading_rad = math.atan2(mag_y_cal, mag_x_cal); heading_deg = math.degrees(heading_rad)
        return (heading_deg + 360) % 360

    def _sensor_thread_func(self):
        logger.info("Sensor fusion thread started.")
        try:
            mag_raw_init = self._read_magnetometer_raw()
            _, _, mag_cal_init = self._apply_calibration({'x':0,'y':0,'z':0}, {'x':0,'y':0,'z':0}, mag_raw_init)
            self._filtered_yaw = self._calculate_mag_heading(mag_cal_init[0], mag_cal_init[1])
            logger.info(f"Initial yaw (calibrated mag): {self._filtered_yaw:.2f} deg")
        except Exception as e: logger.warning(f"Could not get initial mag heading: {e}"); self._filtered_yaw = 0.0
        self._last_imu_read_time = time.time()

        while self._running:
            current_read_time = time.time(); dt = current_read_time - self._last_imu_read_time
            if dt <= 0.0001: time.sleep(self.config.IMU_READ_INTERVAL / 2); continue
            self._last_imu_read_time = current_read_time
            if self.mpu:
                try:
                    accel_r, gyro_r, mag_r = self.mpu.get_accel_data(), self.mpu.get_gyro_data(), self._read_magnetometer_raw()
                    if self.is_calibrating_gyro: self.calibration_samples_gyro.append(gyro_r.copy()); time.sleep(0.01); continue 
                    if self.is_calibrating_accel: self.calibration_samples_accel.append(accel_r.copy()); time.sleep(0.01); continue
                    if self.is_calibrating_mag: self.calibration_samples_mag.append(mag_r); time.sleep(0.01); continue

                    accel_c, gyro_c, mag_c = self._apply_calibration(accel_r, gyro_r, mag_r)
                    acc_tot = math.sqrt(accel_c['x']**2 + accel_c['y']**2 + accel_c['z']**2)
                    if acc_tot == 0: logger.warning("Accel vector zero length, skipping fusion step."); continue
                    nax,nay,naz = accel_c['x']/acc_tot, accel_c['y']/acc_tot, accel_c['z']/acc_tot
                    acc_p = math.degrees(math.atan2(nay, math.sqrt(nax**2 + naz**2))) # Corrected pitch
                    acc_r = math.degrees(math.atan2(-nax, math.sqrt(nay**2 + naz**2)))# Corrected roll (sign depends on axis convention)
                    if naz < 0 : # Adjust pitch for full 360 range if needed (atan2(y,z) gives -90 to 90)
                        acc_p = 180 - acc_p if nay > 0 else -180 - acc_p
                    
                    mag_h = self._calculate_mag_heading(mag_c[0],mag_c[1])
                    with self.lock:
                        self.gyro_x,self.gyro_y,self.gyro_z = gyro_c['x'],gyro_c['y'],gyro_c['z']
                        self.accel_x,self.accel_y,self.accel_z = accel_c['x'],accel_c['y'],accel_c['z']
                        self._filtered_pitch = self.config.ALPHA_ACCEL*(self._filtered_pitch+gyro_c['y']*dt) + (1-self.config.ALPHA_ACCEL)*acc_p
                        self._filtered_roll = self.config.ALPHA_ACCEL*(self._filtered_roll+gyro_c['x']*dt) + (1-self.config.ALPHA_ACCEL)*acc_r
                        gyro_dyaw = gyro_c['z']*dt; err_yaw = mag_h - self._filtered_yaw
                        err_yaw = (err_yaw + 180) % 360 - 180 # Normalize error to +/- 180
                        self._filtered_yaw = (self._filtered_yaw + gyro_dyaw + self.config.ALPHA_MAG*err_yaw) % 360 # Simpler blend for mag correction
                        self.pitch,self.roll,self.yaw = self._filtered_pitch,self._filtered_roll,self._filtered_yaw
                        self._last_successful_read_time = current_read_time
                except Exception as e: logger.error(f"IMU/Mag processing error: {e}", exc_info=False)
            time.sleep(self.config.IMU_READ_INTERVAL)
        logger.info("Sensor fusion thread stopped.")

    def start_gyro_calibration(self, duration_sec=5): logger.info(f"Gyro cal ({duration_sec}s). Keep STILL."); self.calibration_samples_gyro = []; self.is_calibrating_gyro = True
    def finish_gyro_calibration(self):
        self.is_calibrating_gyro = False
        if not self.calibration_samples_gyro: logger.warning("No gyro samples for cal."); return False
        gx,gy,gz = 0,0,0; n=len(self.calibration_samples_gyro)
        for s in self.calibration_samples_gyro: gx+=s.get('x',0); gy+=s.get('y',0); gz+=s.get('z',0)
        self.gyro_offsets = {'x':gx/n,'y':gy/n,'z':gz/n}
        logger.info(f"Gyro cal complete. Offsets: x={gx/n:.3f}, y={gy/n:.3f}, z={gz/n:.3f}")
        self.calibration_samples_gyro = []; return True
    def start_accel_calibration_step(self, step_name): logger.info(f"Accel cal step '{step_name}' started (stub)."); self.calibration_samples_accel = []; self.is_calibrating_accel = True
    def finish_accel_calibration(self): self.is_calibrating_accel = False; logger.info("Accel cal finished (stub - apply multi-point logic)."); return True
    def start_mag_calibration(self): logger.info("Mag cal started (stub). Rotate all axes."); self.calibration_samples_mag = []; self.is_calibrating_mag = True
    def finish_mag_calibration(self): self.is_calibrating_mag = False; logger.info("Mag cal finished (stub - apply ellipsoid fit)."); return True

    def start(self):
        if self.mpu: self._running=True; self._thread=Thread(target=self._sensor_thread_func,daemon=True,name="SensorFusionThread"); self._thread.start()
        else: logger.error("Cannot start SensorFusion: MPU6050 not initialized.")
    def stop(self):
        self._running=False
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=0.5)
    def get_orientation_with_status(self):
        with self.lock:
            age = time.time()-self._last_successful_read_time if self._last_successful_read_time > 0 else float('inf')
            stale = age > self.config.STALE_IMU_DATA_THRESHOLD_SEC
            if stale and self._last_successful_read_time > 0: logger.warning(f"IMU data stale (age:{age:.2f}s)")
            return self.pitch,self.roll,self.yaw,stale,self._last_successful_read_time
    def get_gyro_rates_with_status(self):
        with self.lock:
            age = time.time()-self._last_successful_read_time if self._last_successful_read_time > 0 else float('inf')
            stale = age > self.config.STALE_IMU_DATA_THRESHOLD_SEC
            return self.gyro_x,self.gyro_y,self.gyro_z,stale