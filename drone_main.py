# drone_main.py
import RPi.GPIO as GPIO
import time, signal, sys, os, traceback, logging, logging.handlers, math
from enum import Enum, auto

from config import DroneConfig
from utils import clamp, haversine_distance
from pid_controller import PID
from motor_controller import MotorController
from sensor_fusion import SensorFusion
from gps_manager import GPSManager
from obstacle_avoidance import ObstacleAvoidance
from ps5_controller import PS5ControllerManager
from battery_monitor import BatteryMonitor
from telemetry_hud import TelemetryHUDStreamer
from gcs_communicator import GCSCommunicator

logger = logging.getLogger(__name__)

class DroneState(Enum):
    INIT, DISARMED_IDLE, AWAITING_CALIBRATION_COMMAND, CALIBRATING_GYRO, CALIBRATING_ACCEL_STEP_LEVEL, CALIBRATING_MAG, \
    ARMING_SEQUENCE, ARMED_MANUAL, RTH_INITIATED, RTH_CLIMBING, RTH_NAVIGATING, RTH_DESCENDING_AT_HOME, \
    LANDING_SEQUENCE, EMERGENCY_LANDING, FAILSAFE_RC_TRIGGERED, FAILSAFE_GPS_TRIGGERED, \
    ERROR_STATE, SHUTTING_DOWN = range(18)

def setup_logging(log_level_str="INFO",log_file="drone_flight.log",console_log=True,file_log=True):
    lvl_map={"DEBUG":logging.DEBUG,"INFO":logging.INFO,"WARNING":logging.WARNING,"ERROR":logging.ERROR,"CRITICAL":logging.CRITICAL}
    lvl=lvl_map.get(log_level_str.upper(),logging.INFO); root_log=logging.getLogger(); root_log.setLevel(lvl)
    if root_log.hasHandlers():root_log.handlers.clear()
    fmt=logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(threadName)s - %(message)s')
    if console_log:ch=logging.StreamHandler(sys.stdout);ch.setLevel(lvl);ch.setFormatter(fmt);root_log.addHandler(ch)
    if file_log and log_file:fh=logging.handlers.RotatingFileHandler(log_file,maxBytes=5*1024*1024,backupCount=3);fh.setLevel(lvl);fh.setFormatter(fmt);root_log.addHandler(fh)
    root_log.info(f"Logging configured. Lvl:{log_level_str.upper()}. File:{log_file if file_log else 'N/A'}")

class Drone:
    def __init__(self, config_filepath="drone_config.json"):
        self.config = DroneConfig(config_file_path=config_filepath)
        setup_logging(self.config.LOG_LEVEL, self.config.LOG_FILE) 
        logger.info("Drone Systems Initializing with loaded configuration...")
        self.current_state=DroneState.INIT; self.running=True; self.last_loop_time=time.time()
        self.target_yaw_heading=0.0; self.rth_target_alt_reached=False; self.state_enter_time=time.time()
        self.rc_signal_lost_start_time, self.gps_signal_lost_start_time = None, None
        self.current_calibration_type, self.calibration_step_start_time = None, None

        self.motor_ctrl = MotorController(self.config); self.sensor_fusion = SensorFusion(self.config)
        self.gps_mgr = GPSManager(self.config); self.obstacle_avoider = ObstacleAvoidance(self.config) 
        self.ctrl_mgr = PS5ControllerManager(self.config)
        self.battery_mon = BatteryMonitor(self.config,self.is_armed_or_arming)
        self.hud_streamer = TelemetryHUDStreamer(self.config,self.get_hud_telemetry_data)
        self.gcs_comm = GCSCommunicator(self.config,self.handle_gcs_command)
        self.pid_p=PID(self.config.PID_PITCH_KP,self.config.PID_PITCH_KI,self.config.PID_PITCH_KD,self.config.PID_PITCH_LIMITS)
        self.pid_r=PID(self.config.PID_ROLL_KP,self.config.PID_ROLL_KI,self.config.PID_ROLL_KD,self.config.PID_ROLL_LIMITS)
        self.pid_y_h=PID(self.config.PID_YAW_KP,self.config.PID_YAW_KI,self.config.PID_YAW_KD,self.config.PID_YAW_LIMITS)
        signal.signal(signal.SIGINT,self._sig_exit); signal.signal(signal.SIGTERM,self._sig_exit)
        self.transition_to_state(DroneState.DISARMED_IDLE); logger.info(f"Drone class init complete. State: {self.current_state.name}")

    def transition_to_state(self,new_state):
        if self.current_state!=new_state: 
            logger.info(f"State Transition: {self.current_state.name} -> {new_state.name}")
            self.current_state=new_state; self.state_enter_time=time.time()
            if new_state==DroneState.ARMING_SEQUENCE: 
                self.motor_ctrl.arm_motors_sequence()
                _,_,_,yaw_on_arm,_,_=self.sensor_fusion.get_orientation_with_status()
                self.target_yaw_heading=yaw_on_arm if yaw_on_arm is not None else 0.0
                self.pid_p.reset();self.pid_r.reset();self.pid_y_h.reset()
                logger.info(f"ARMED! Target Yaw:{self.target_yaw_heading:.1f}")
                self.transition_to_state(DroneState.ARMED_MANUAL)
            elif new_state==DroneState.DISARMED_IDLE: self.motor_ctrl.disarm_motors()
            elif new_state==DroneState.RTH_INITIATED: self.rth_target_alt_reached=False; logger.warning("RTH initiated.")
            elif new_state in [DroneState.EMERGENCY_LANDING,DroneState.ERROR_STATE]: logger.critical(f"{new_state.name} INITIATED!"); self.motor_ctrl.disarm_motors()
            elif new_state in [DroneState.CALIBRATING_GYRO,DroneState.CALIBRATING_ACCEL_STEP_LEVEL,DroneState.CALIBRATING_MAG]: 
                self.calibration_step_start_time=time.time();logger.info(f"Entering {new_state.name}")
                if new_state==DroneState.CALIBRATING_GYRO: self.sensor_fusion.start_gyro_calibration()
                elif new_state==DroneState.CALIBRATING_ACCEL_STEP_LEVEL: self.sensor_fusion.start_accel_calibration_step("LEVEL") # Method in SF needs update
                elif new_state==DroneState.CALIBRATING_MAG: self.sensor_fusion.start_mag_calibration()

    def is_armed_or_arming(self): return self.current_state in [DroneState.ARMING_SEQUENCE,DroneState.ARMED_MANUAL,DroneState.RTH_INITIATED,DroneState.RTH_CLIMBING,DroneState.RTH_NAVIGATING,DroneState.RTH_DESCENDING_AT_HOME,DroneState.LANDING_SEQUENCE,DroneState.FAILSAFE_RC_TRIGGERED,DroneState.FAILSAFE_GPS_TRIGGERED]
    def _sig_exit(self,sig,frame): logger.info(f"Signal {signal.Signals(sig).name} received. Shutdown state."); self.running=False; self.transition_to_state(DroneState.SHUTTING_DOWN)
    def get_hud_telemetry_data(self):
        p,r,y,imu_s,_=self.sensor_fusion.get_orientation_with_status(); gps=self.gps_mgr.get_data_with_status()
        return {'pitch':p,'roll':r,'yaw':y,'imu_stale':imu_s, 'gps_lat':gps.get('lat'),'gps_lon':gps.get('lon'),'gps_alt':gps.get('alt',0.0),
                'gps_speed_kmh':gps.get('speed_kmh',0.0),'gps_fix_quality':gps.get('fix_quality',0),'gps_num_satellites':gps.get('num_satellites',0),
                'gps_stale':gps.get('is_stale',True),'battery_level':self.battery_mon.get_level_percentage(),'system_armed':self.is_armed_or_arming(),'current_state':self.current_state.name}
    def get_full_telemetry_data_for_gcs(self):
        data = self.get_hud_telemetry_data(); gx,gy,gz,g_stale = self.sensor_fusion.get_gyro_rates_with_status()
        data.update({'target_yaw':self.target_yaw_heading, 'loop_dt_ms':(time.time()-self.last_loop_time)*1000 if self.last_loop_time else 0,
                     'state_time_s': time.time() - self.state_enter_time, 'gyro_x':gx, 'gyro_y':gy, 'gyro_z':gz, 'gyro_stale':g_stale})
        return data
    def handle_gcs_command(self,cmd_data):
        cmd = cmd_data.get("command","").upper(); logger.info(f"GCS Cmd: {cmd}, Data: {cmd_data}")
        if self.current_state in [DroneState.SHUTTING_DOWN,DroneState.ERROR_STATE]: logger.warning(f"Ignoring GCS cmd '{cmd}' in state: {self.current_state.name}"); return
        if cmd=="ARM" and self.current_state==DroneState.DISARMED_IDLE: self.transition_to_state(DroneState.ARMING_SEQUENCE)
        elif cmd=="DISARM" and self.is_armed_or_arming(): self.transition_to_state(DroneState.DISARMED_IDLE)
        elif cmd=="START_RTH" and self.is_armed_or_arming(): self.transition_to_state(DroneState.RTH_INITIATED)
        elif cmd=="START_LANDING" and self.is_armed_or_arming(): self.transition_to_state(DroneState.LANDING_SEQUENCE)
        elif cmd=="START_CALIBRATION" and self.current_state==DroneState.DISARMED_IDLE:
            cal_type=cmd_data.get("type","").upper(); self.current_calibration_type=cal_type
            if cal_type=="GYRO": self.transition_to_state(DroneState.CALIBRATING_GYRO)
            elif cal_type=="ACCEL": self.transition_to_state(DroneState.CALIBRATING_ACCEL_STEP_LEVEL)
            elif cal_type=="MAG": self.transition_to_state(DroneState.CALIBRATING_MAG)
            else: logger.warning(f"GCS: Unknown cal type '{cal_type}'"); self.current_calibration_type=None;
        elif cmd=="SAVE_CONFIG": self.config.save_to_file(); logger.info("Config saved via GCS command.")
        else: logger.warning(f"GCS: Unknown cmd '{cmd}' or invalid state.")

    def _check_rc_failsafe(self,current_time):
        if not self.config.ENABLE_RC_FAILSAFE or not self.is_armed_or_arming(): self.rc_signal_lost_start_time=None; return False
        if self.ctrl_mgr.is_signal_lost(self.config.RC_FAILSAFE_TIMEOUT_SEC):
            if self.rc_signal_lost_start_time is None: self.rc_signal_lost_start_time=current_time; logger.warning("RC signal lost, failsafe timer started.")
            if current_time-self.rc_signal_lost_start_time >= self.config.RC_FAILSAFE_TIMEOUT_SEC: logger.critical("RC FAILSAFE TRIGGERED!"); self.rc_signal_lost_start_time=None; return True
        else:
            if self.rc_signal_lost_start_time is not None: logger.info("RC signal re-acquired.")
            self.rc_signal_lost_start_time=None
        return False
    def _check_gps_failsafe(self,current_time,gps_data):
        if not self.config.ENABLE_GPS_FAILSAFE or self.current_state not in [DroneState.RTH_INITIATED,DroneState.RTH_CLIMBING,DroneState.RTH_NAVIGATING,DroneState.RTH_DESCENDING_AT_HOME]: self.gps_signal_lost_start_time=None; return False
        if not gps_data.get('is_reliable_for_nav',False):
            if self.gps_signal_lost_start_time is None: self.gps_signal_lost_start_time=current_time; logger.warning(f"GPS unreliable. GPS failsafe timer started.")
            if current_time-self.gps_signal_lost_start_time >= self.config.GPS_FAILSAFE_TIMEOUT_SEC: logger.critical("GPS FAILSAFE TRIGGERED!"); self.gps_signal_lost_start_time=None; return True
        else:
            if self.gps_signal_lost_start_time is not None: logger.info("GPS signal reliable again.")
            self.gps_signal_lost_start_time=None
        return False
    def _check_critical_battery(self,bat_lvl):
        if bat_lvl < self.config.CRITICAL_BATTERY_THRESHOLD_PERCENT and self.current_state not in [DroneState.EMERGENCY_LANDING,DroneState.SHUTTING_DOWN]: logger.critical(f"CRITICAL BATTERY ({bat_lvl:.1f}%)! Emergency Land."); return True
        return False

    def handle_disarmed_idle_state(self,arm_req,shut_req): # Renamed from handle_..._controller to generic
        if shut_req: self.transition_to_state(DroneState.SHUTTING_DOWN)
        elif arm_req: self.transition_to_state(DroneState.ARMING_SEQUENCE)
    def handle_calibrating_gyro_state(self,current_time):
        cal_duration = 7.0 # Longer for more samples
        if current_time-self.calibration_step_start_time > cal_duration:
            if self.sensor_fusion.finish_gyro_calibration(): self.config.IMU_GYRO_OFFSETS=self.sensor_fusion.gyro_offsets.copy(); self.config.save_to_file(); logger.info("Gyro cal success & saved.")
            else: logger.error("Gyro cal failed.")
            self.current_calibration_type=None; self.transition_to_state(DroneState.DISARMED_IDLE)
    def handle_calibrating_accel_step_level_state(self,current_time): # Simplified stub
        logger.info("CAL_ACCEL_LEVEL: Place drone flat. Collecting data...")
        if current_time-self.calibration_step_start_time > 5.0: # Collect for 5s
            # self.sensor_fusion.process_accel_samples_for_step("LEVEL") # This method needs implementation in SF
            logger.info("Accel LEVEL step data collected (stub). Save/proceed in SF.finish_accel_calibration().")
            if self.sensor_fusion.finish_accel_calibration(): self.config.IMU_ACCEL_OFFSETS=self.sensor_fusion.accel_offsets.copy(); self.config.save_to_file(); logger.info("Accel cal (stub) success & saved.") # Needs full impl in SF
            self.current_calibration_type=None; self.transition_to_state(DroneState.DISARMED_IDLE)
    def handle_calibrating_mag_state(self,current_time): # Simplified stub
        logger.info("CAL_MAG: Rotate drone slowly through all axes...")
        if current_time-self.calibration_step_start_time > 30.0: # Collect for 30s
            logger.info("Mag sample collection period ended (stub). Process in SF.finish_mag_calibration().")
            if self.sensor_fusion.finish_mag_calibration(): self.config.IMU_MAG_OFFSETS=self.sensor_fusion.mag_offsets.copy(); self.config.IMU_MAG_SCALE_FACTORS=self.sensor_fusion.mag_scales.copy(); self.config.save_to_file(); logger.info("Mag cal (stub) success & saved.") # Needs full impl in SF
            self.current_calibration_type=None; self.transition_to_state(DroneState.DISARMED_IDLE)
    def handle_awaiting_calibration_command_state(self): # Entered via GCS command
        logger.info("Awaiting specific GCS calibration command (GYRO, ACCEL, MAG) or CANCEL_CALIBRATION.")
        if time.time()-self.state_enter_time > 60.0: logger.warning("Timeout awaiting cal cmd. To IDLE."); self.transition_to_state(DroneState.DISARMED_IDLE)

    def handle_armed_manual_state(self,ctrl_in,gps_dat,bat_lvl,imu_p,imu_r,imu_y,gz,curr_t,dt):
        us_dist=self.obstacle_avoider.get_distances(); sp_proxy=max(abs(ctrl_in['pitch']),abs(ctrl_in['roll']))
        oa_p,oa_r,obs_hit,_=self.obstacle_avoider.calculate_avoidance_maneuver(us_dist,ctrl_in['pitch'],ctrl_in['roll'],sp_proxy)
        if obs_hit:
            if abs(oa_p)>0.01: ctrl_in['pitch']=oa_p
            if abs(oa_r)>0.01: ctrl_in['roll']=oa_r
        base_thr=clamp(self.config.BASE_THROTTLE_DUTY+ctrl_in['throttle']*self.config.THROTTLE_RANGE_MODIFIER,self.config.MIN_PWM_DUTY,self.config.MAX_PWM_DUTY)
        tgt_p,tgt_r=ctrl_in['pitch']*self.config.CONTROLLER_PITCH_ANGLE_SCALE,ctrl_in['roll']*self.config.CONTROLLER_ROLL_ANGLE_SCALE
        corr_p,corr_r=self.pid_p.compute(tgt_p,imu_p,curr_t),self.pid_r.compute(tgt_r,imu_r,curr_t)
        des_y_rate=ctrl_in['yaw']*self.config.CONTROLLER_YAW_RATE_SCALE
        if abs(des_y_rate)>0.5: self.target_yaw_heading=(self.target_yaw_heading+des_y_rate*dt)%360 # Smoother update for target heading
        else: # Actively hold current heading if yaw stick is centered
            # This ensures target_yaw_heading doesn't drift if yaw stick is noisy around center
            # And allows PID to correct disturbances to maintain the last commanded heading.
            # No change to self.target_yaw_heading, PID works towards existing target.
            pass
            
        y_err=self.target_yaw_heading-imu_y; y_err=(y_err+180)%360-180
        corr_y=self.pid_y_h.compute(0,-y_err,curr_t)
        m1,m2,m3,m4 = base_thr-corr_p-corr_r-corr_y, base_thr-corr_p+corr_r+corr_y, base_thr+corr_p+corr_r-corr_y, base_thr+corr_p-corr_r+corr_y
        self.motor_ctrl.set_all_motors_duty_cycle([clamp(m,self.config.MIN_PWM_DUTY,self.config.MAX_PWM_DUTY) for m in [m1,m2,m3,m4]])
    
    def _rth_motor_commands(self, base_throttle, target_pitch_angle, target_roll_angle, target_yaw_heading_for_pid, current_pitch, current_roll, current_yaw, current_time):
        # Helper for RTH motor commands using current PID instances
        corr_p = self.pid_p.compute(target_pitch_angle, current_pitch, current_time)
        corr_r = self.pid_r.compute(target_roll_angle, current_roll, current_time)
        yaw_err = target_yaw_heading_for_pid - current_yaw; yaw_err = (yaw_err + 180) % 360 - 180
        corr_y = self.pid_y_h.compute(0, -yaw_err, current_time) # Target 0 error for heading
        m1,m2,m3,m4 = base_throttle-corr_p-corr_r-corr_y, base_throttle-corr_p+corr_r+corr_y, base_throttle+corr_p+corr_r-corr_y, base_throttle+corr_p-corr_r+corr_y
        self.motor_ctrl.set_all_motors_duty_cycle([clamp(m,self.config.MIN_PWM_DUTY,self.config.MAX_PWM_DUTY) for m in [m1,m2,m3,m4]])

    def handle_rth_logic_states(self, gps_dat, imu_p, imu_r, imu_y, curr_t):
        current_altitude = gps_dat.get('alt', 0.0)
        if self.current_state == DroneState.RTH_INITIATED:
            home_ok = gps_dat.get('home_lat') and gps_dat.get('home_lon') and gps_dat.get('is_reliable_for_nav', False)
            if not home_ok: logger.error("RTH: No reliable GPS/Home. To FAILSAFE_GPS."); self.transition_to_state(DroneState.FAILSAFE_GPS_TRIGGERED); return
            self.rth_target_alt_reached = False; self.transition_to_state(DroneState.RTH_CLIMBING)
        elif self.current_state == DroneState.RTH_CLIMBING:
            if current_altitude >= self.config.RTH_ALTITUDE_METERS-0.5: self.rth_target_alt_reached=True; logger.info("RTH: Climb alt reached."); self.transition_to_state(DroneState.RTH_NAVIGATING); return
            base_thr_climb = clamp(self.config.BASE_THROTTLE_DUTY + 2.0, self.config.MIN_PWM_DUTY, self.config.MAX_PWM_DUTY) # Stronger climb
            self._rth_motor_commands(base_thr_climb, 0, 0, self.target_yaw_heading, imu_p, imu_r, imu_y, curr_t) # Maintain current heading or align to home
        elif self.current_state == DroneState.RTH_NAVIGATING:
            home_lat,home_lon,curr_lat,curr_lon = gps_dat.get('home_lat'),gps_dat.get('home_lon'),gps_dat.get('lat'),gps_dat.get('lon')
            dist_m = haversine_distance(curr_lat,curr_lon,home_lat,home_lon,self.config.EARTH_RADIUS_KM)
            logger.debug(f"RTH Nav: Dist:{dist_m:.1f}m, Alt:{current_altitude:.1f}m")
            if dist_m <= self.config.RETURN_HOME_MIN_DIST_METERS: logger.info("RTH: Home vicinity. To RTH_DESCENDING."); self.transition_to_state(DroneState.RTH_DESCENDING_AT_HOME); return
            dlon=math.radians(home_lon-curr_lon); lat1r,lat2r=math.radians(curr_lat),math.radians(home_lat)
            y=math.sin(dlon)*math.cos(lat2r); x=math.cos(lat1r)*math.sin(lat2r)-math.sin(lat1r)*math.cos(lat2r)*math.cos(dlon)
            target_bearing = (math.degrees(math.atan2(y,x))+360)%360
            rth_fwd_pitch_angle = 0.10 * self.config.CONTROLLER_PITCH_ANGLE_SCALE # Slower, more controlled RTH fwd speed
            base_thr_rth = self.config.BASE_THROTTLE_DUTY # Crude altitude hold for RTH nav
            self._rth_motor_commands(base_thr_rth, rth_fwd_pitch_angle, 0, target_bearing, imu_p, imu_r, imu_y, curr_t)
        elif self.current_state == DroneState.RTH_DESCENDING_AT_HOME:
            logger.info(f"RTH Descending at Home: Alt:{current_altitude:.1f}m")
            if current_altitude < 0.8: logger.info("RTH: Assumed landed by altitude. Disarming."); self.transition_to_state(DroneState.DISARMED_IDLE); return
            desc_thr = self.config.MIN_PWM_DUTY + 0.3 # Very gentle descent
            self._rth_motor_commands(desc_thr, 0, 0, self.target_yaw_heading, imu_p, imu_r, imu_y, curr_t) # Hold position and descend

    def handle_landing_sequence_state(self, imu_p, imu_r, imu_y, curr_t, current_altitude):
        logger.info(f"Landing Sequence: Alt:{current_altitude:.1f}m")
        if current_altitude < 0.8: logger.info("Landing: Assumed landed. Disarming."); self.transition_to_state(DroneState.DISARMED_IDLE); return
        desc_thr = self.config.MIN_PWM_DUTY + 0.3
        self._rth_motor_commands(desc_thr, 0, 0, self.target_yaw_heading, imu_p, imu_r, imu_y, curr_t) # Using RTH motor command helper for descent

    def handle_failsafe_rc_triggered_state(self, gps_dat):
        logger.warning(f"RC Failsafe Active. Action: {self.config.RC_FAILSAFE_ACTION}")
        # Check if RC signal has returned
        if not self.ctrl_mgr.is_signal_lost(0.1): # Check with very short timeout, effectively "is signal present now?"
            logger.info("RC signal restored during RC Failsafe state. Returning to ARMED_MANUAL.")
            self.transition_to_state(DroneState.ARMED_MANUAL) # Or a "FAILSAFE_RECOVERED_AWAIT_INPUT" state
            return
        if self.config.RC_FAILSAFE_ACTION == "RTH":
            if gps_dat.get('is_reliable_for_nav',False) and gps_dat.get('home_lat') is not None: self.transition_to_state(DroneState.RTH_INITIATED)
            else: logger.warning("RC Failsafe: RTH failed (GPS/Home unreliable). To LANDING_SEQUENCE."); self.transition_to_state(DroneState.LANDING_SEQUENCE)
        else: self.transition_to_state(DroneState.LANDING_SEQUENCE) # Default to LAND if not RTH
    def handle_failsafe_gps_triggered_state(self, gps_dat):
        logger.warning(f"GPS Failsafe Active. Action: {self.config.GPS_FAILSAFE_ACTION}")
        # Check if GPS signal has recovered
        if gps_dat.get('is_reliable_for_nav', False):
            logger.info("GPS signal restored during GPS Failsafe state. Attempting to return to previous state or RTH.")
            # This logic needs to know the previous state to return to, or default to a safe mode.
            # For now, let's try to re-initiate RTH if home is set.
            if gps_dat.get('home_lat') is not None: self.transition_to_state(DroneState.RTH_INITIATED)
            else: self.transition_to_state(DroneState.LANDING_SEQUENCE) # Fallback if no home
            return
        if self.config.GPS_FAILSAFE_ACTION == "LAND": self.transition_to_state(DroneState.LANDING_SEQUENCE)
        else: logger.warning(f"GPS Failsafe: Unknown action. To LAND."); self.transition_to_state(DroneState.LANDING_SEQUENCE)
    def handle_emergency_landing_state(self):
        logger.critical("EMERGENCY LANDING: Motors min. Descending.")
        self.motor_ctrl.disarm_motors()
        if time.time()-self.state_enter_time > 10.0: logger.info("Emergency land period over. To DISARMED."); self.transition_to_state(DroneState.DISARMED_IDLE)
    def handle_error_state(self): logger.critical("ERROR_STATE. Motors disarmed. Manual check."); self.motor_ctrl.disarm_motors()

    def run_flight_loop(self):
        logger.info("Starting components..."); self.sensor_fusion.start(); self.gps_mgr.start(); self.battery_mon.start(); self.hud_streamer.start(); self.gcs_comm.start()
        logger.info(f"Main flight loop active. Initial State: {self.current_state.name}")
        try:
            while self.running:
                curr_t=time.time(); dt=curr_t-self.last_loop_time
                if dt<=0.0005: time.sleep(0.0005); continue # Ensure dt is positive and meaningful, give tiny sleep if too fast
                self.last_loop_time=curr_t
                raw_ax,arm_req,shut_req,py_q_req = self.ctrl_mgr.process_events()
                imu_p,imu_r,imu_y,imu_stale,_ = self.sensor_fusion.get_orientation_with_status()
                gx,gy,gz,gyro_stale = self.sensor_fusion.get_gyro_rates_with_status()
                gps_d = self.gps_mgr.get_data_with_status(); bat_l = self.battery_mon.get_level_percentage()
                if self.gcs_comm: self.gcs_comm.update_and_send_telemetry(self.get_full_telemetry_data_for_gcs())
                
                if (imu_stale or gyro_stale) and self.is_armed_or_arming() and self.current_state not in [DroneState.EMERGENCY_LANDING, DroneState.ERROR_STATE, DroneState.SHUTTING_DOWN]:
                    logger.error("CRITICAL: IMU data stale! To EMERGENCY_LANDING.")
                    self.transition_to_state(DroneState.EMERGENCY_LANDING)
                
                current_loop_state = self.current_state
                if current_loop_state==DroneState.SHUTTING_DOWN: self.running=False; break
                if py_q_req: self.transition_to_state(DroneState.SHUTTING_DOWN)

                # Global Failsafe Checks
                can_be_overridden = current_loop_state not in [DroneState.EMERGENCY_LANDING, DroneState.ERROR_STATE, DroneState.SHUTTING_DOWN, DroneState.FAILSAFE_RC_TRIGGERED, DroneState.FAILSAFE_GPS_TRIGGERED, DroneState.CALIBRATING_GYRO, DroneState.CALIBRATING_ACCEL_STEP_LEVEL, DroneState.CALIBRATING_MAG, DroneState.DISARMED_IDLE, DroneState.AWAITING_CALIBRATION_COMMAND, DroneState.INIT]
                if can_be_overridden:
                    if self._check_critical_battery(bat_l): self.transition_to_state(DroneState.EMERGENCY_LANDING); continue
                    if self._check_rc_failsafe(curr_t): self.transition_to_state(DroneState.FAILSAFE_RC_TRIGGERED); continue
                    if self._check_gps_failsafe(curr_t, gps_d) and current_loop_state in [DroneState.RTH_INITIATED, DroneState.RTH_CLIMBING, DroneState.RTH_NAVIGATING, DroneState.RTH_DESCENDING_AT_HOME]: self.transition_to_state(DroneState.FAILSAFE_GPS_TRIGGERED); continue
                
                # State-specific logic
                if current_loop_state==DroneState.INIT: self.transition_to_state(DroneState.DISARMED_IDLE)
                elif current_loop_state==DroneState.DISARMED_IDLE: self.handle_disarmed_idle_state(arm_req,shut_req)
                elif current_loop_state==DroneState.AWAITING_CALIBRATION_COMMAND: self.handle_awaiting_calibration_command_state()
                elif current_loop_state==DroneState.CALIBRATING_GYRO: self.handle_calibrating_gyro_state(curr_t)
                elif current_loop_state==DroneState.CALIBRATING_ACCEL_STEP_LEVEL: self.handle_calibrating_accel_step_level_state(curr_t)
                elif current_loop_state==DroneState.CALIBRATING_MAG: self.handle_calibrating_mag_state(curr_t)
                elif current_loop_state==DroneState.ARMED_MANUAL:
                    if shut_req: self.transition_to_state(DroneState.DISARMED_IDLE)
                    else: self.handle_armed_manual_state(raw_ax.copy(),gps_d,bat_l,imu_p,imu_r,imu_y,gz,curr_t,dt)
                elif current_loop_state in [DroneState.RTH_INITIATED, DroneState.RTH_CLIMBING, DroneState.RTH_NAVIGATING, DroneState.RTH_DESCENDING_AT_HOME]: self.handle_rth_logic_states(gps_d, imu_p, imu_r, imu_y, curr_t)
                elif current_loop_state==DroneState.LANDING_SEQUENCE: self.handle_landing_sequence_state(imu_p,imu_r,imu_y,curr_t,gps_d.get('alt',0.0))
                elif current_loop_state==DroneState.EMERGENCY_LANDING: self.handle_emergency_landing_state()
                elif current_loop_state==DroneState.FAILSAFE_RC_TRIGGERED: self.handle_failsafe_rc_triggered_state(gps_d)
                elif current_loop_state==DroneState.FAILSAFE_GPS_TRIGGERED: self.handle_failsafe_gps_triggered_state(gps_d) # Pass gps_d
                elif current_loop_state==DroneState.ERROR_STATE: self.handle_error_state()
                
                loop_ex = time.time()-curr_t; sleep_d=(1.0/self.config.MAIN_LOOP_HZ)-loop_ex
                if sleep_d > 0: time.sleep(sleep_d)
                # else: logger.debug(f"Loop overrun: {abs(sleep_d)*1000:.1f}ms")
        except KeyboardInterrupt: logger.info("KI in main_loop."); self.transition_to_state(DroneState.SHUTTING_DOWN)
        except Exception as e: logger.critical(f"MAIN LOOP CRASH: {e}",exc_info=True); self.transition_to_state(DroneState.ERROR_STATE)
        finally:
            if self.current_state!=DroneState.SHUTTING_DOWN: self.transition_to_state(DroneState.SHUTTING_DOWN) # Ensure it's set for perform_shutdown
            self.perform_shutdown()

    def perform_shutdown(self): # Ensure all components are stopped
        logger.info("Drone shutdown sequence starting...")
        self.running = False # Explicitly ensure running is false for all threads
        _ = [comp.stop() for comp in [self.gcs_comm, self.hud_streamer, self.battery_mon, self.gps_mgr, self.sensor_fusion] if comp]
        if self.ctrl_mgr: self.ctrl_mgr.quit()
        if self.motor_ctrl: self.motor_ctrl.stop_motors_completely()
        logger.info("GPIO cleanup...");
        try: GPIO.cleanup(); logger.info("GPIO cleaned up.")
        except Exception as e: logger.error(f"GPIO cleanup error: {e}")
        logger.info("Drone shutdown complete.")

if __name__ == "__main__":
    drone_instance = None # Ensure it's defined for finally block
    try:
        initial_conf = DroneConfig(config_file_path="drone_config.json") 
        setup_logging(log_level_str=initial_conf.LOG_LEVEL, log_file=initial_conf.LOG_FILE)
        drone_instance = Drone(config_filepath="drone_config.json") 
        drone_instance.run_flight_loop()
    except Exception as e: logger.critical(f"FATAL MAIN ERROR (pre-loop or unhandled): {e}", exc_info=True)
        if drone_instance and drone_instance.current_state != DroneState.SHUTTING_DOWN :
            drone_instance.transition_to_state(DroneState.ERROR_STATE) # Attempt to go to error state
            drone_instance.perform_shutdown() # Then try to shut down components
        elif drone_instance is None: # Failed very early
             try: GPIO.cleanup(); logger.info("Attempted basic GPIO cleanup after pre-instance failure.")
             except: pass # Best effort
    finally:
        logger.info("Application exiting.")
        logging.shutdown() # Flushes and closes all logging handlers