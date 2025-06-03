# gps_manager.py
import serial
import time
from threading import Thread, Lock
from utils import haversine_distance
import logging

logger = logging.getLogger(__name__)

class GPSManager:
    def __init__(self, config):
        self.config = config; self.serial_port = None; self.lock = Lock()
        self.home_lat,self.home_lon,self.current_lat,self.current_lon = None,None,None,None
        self.current_altitude,self.current_ground_speed_kmh = 0.0,0.0
        self.fix_quality, self.num_satellites = 0,0
        self._running = False; self._thread = None; self._last_successful_read_time = 0
        try:
            self.serial_port = serial.Serial(config.GPS_PORT, baudrate=config.GPS_BAUDRATE, timeout=config.GPS_TIMEOUT)
            logger.info(f"GPS module initialized on {config.GPS_PORT}.")
        except serial.SerialException as e: logger.error(f"Error initializing GPS on {config.GPS_PORT}: {e}.")

    def _parse_nmea_sentence(self, line): # Slightly more robust parsing
        lat,lon,alt,spd,fix,sats = None,None,None,None,None,None; parts=line.split(',')
        try:
            if line.startswith("$GPGGA") and len(parts) >= 10:
                if parts[2] and parts[3] and parts[4] and parts[5] and parts[6]: # Ensure essential fields are not empty
                    lat_raw, lat_dir = float(parts[2]), parts[3]; lon_raw, lon_dir = float(parts[4]), parts[5]
                    fix=int(parts[6]) if parts[6] else 0; sats=int(parts[7]) if parts[7] else 0
                    alt=float(parts[9]) if parts[9] else None
                    lat_deg, lat_min = int(lat_raw/100), lat_raw % 100; lat = lat_deg + (lat_min/60)
                    if lat_dir == 'S': lat *= -1
                    lon_deg, lon_min = int(lon_raw/100), lon_raw % 100; lon = lon_deg + (lon_min/60)
                    if lon_dir == 'W': lon *= -1
            elif line.startswith("$GPRMC") and len(parts) >= 8: # Check parts[2] for validity 'A'
                if parts[2] == 'A' and parts[7]: # Valid RMC sentence with speed
                    spd=float(parts[7])*1.852 # Knots to km/h
        except (ValueError,IndexError,TypeError) as e: logger.debug(f"NMEA parse error: {line[:40]}... : {e}")
        return lat,lon,alt,spd,fix,sats

    def _gps_reader_thread_func(self):
        logger.info("GPS reader thread started.")
        while self._running:
            if not self.serial_port or not self.serial_port.is_open:
                try:
                    if self.serial_port: self.serial_port.close()
                    logger.info("Attempting to reopen GPS port..."); time.sleep(2) # Shorter sleep before retry
                    self.serial_port = serial.Serial(self.config.GPS_PORT, baudrate=self.config.GPS_BAUDRATE, timeout=self.config.GPS_TIMEOUT)
                    logger.info(f"GPS module re-initialized on {self.config.GPS_PORT}.")
                except serial.SerialException: logger.warning(f"Failed to reopen GPS {self.config.GPS_PORT}, retrying in 3s."); time.sleep(1); continue # Shortened sleep
            try:
                line = self.serial_port.readline().decode('utf-8',errors='ignore').strip()
                if line:
                    lat,lon,alt,spd,fix,sats = self._parse_nmea_sentence(line); updated_this_cycle=False
                    with self.lock:
                        if lat is not None and lon is not None: self.current_lat,self.current_lon=lat,lon; updated_this_cycle=True
                        if self.home_lat is None and self.home_lon is None and fix and fix>0 and lat is not None:
                            self.home_lat,self.home_lon=lat,lon
                            logger.info(f"GPS Home set: LAT={lat:.6f},LON={lon:.6f} (Fix:{fix},Sats:{sats or 'N/A'})")
                        if alt is not None: self.current_altitude=alt; updated_this_cycle=True
                        if spd is not None: self.current_ground_speed_kmh=spd; updated_this_cycle=True
                        if fix is not None: self.fix_quality=fix; updated_this_cycle=True
                        if sats is not None: self.num_satellites=sats; updated_this_cycle=True
                        if updated_this_cycle: self._last_successful_read_time = time.time()
            except serial.SerialException as e: logger.error(f"GPS Serial error: {e}. Will attempt reopen."); self.serial_port=None; time.sleep(0.5)
            except Exception as e: logger.error(f"Unexpected GPS thread error: {e}", exc_info=False); time.sleep(0.5) # exc_info False for less noise
            time.sleep(self.config.GPS_POLL_INTERVAL)
        logger.info("GPS reader thread stopped.")

    def start(self):
        if self.serial_port: self._running=True; self._thread=Thread(target=self._gps_reader_thread_func,daemon=True,name="GPSReaderThread"); self._thread.start()
        else: logger.warning("GPSManager: Cannot start reader, serial port not initialized.")
    def stop(self):
        self._running=False
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open: self.serial_port.close(); logger.info("GPS serial port closed.")

    def get_data_with_status(self):
        with self.lock:
            age = time.time()-self._last_successful_read_time if self._last_successful_read_time > 0 else float('inf')
            stale = age > self.config.STALE_GPS_DATA_THRESHOLD_SEC
            if stale and self._last_successful_read_time > 0: logger.warning(f"GPS data stale (age:{age:.2f}s)")
            sane_fix = self.fix_quality >= self.config.GPS_FAILSAFE_MIN_FIX_QUALITY
            sane_sats = self.num_satellites >= self.config.GPS_FAILSAFE_MIN_SATELLITES
            reliable = sane_fix and sane_sats and not stale
            return {'lat':self.current_lat,'lon':self.current_lon,'alt':self.current_altitude,
                    'speed_kmh':self.current_ground_speed_kmh,'home_lat':self.home_lat,'home_lon':self.home_lon,
                    'fix_quality':self.fix_quality,'num_satellites':self.num_satellites,'is_stale':stale,
                    'last_update_time':self._last_successful_read_time, 'is_reliable_for_nav':reliable}
    def get_distance_to_home(self):
        d = self.get_data_with_status()
        return haversine_distance(d.get('lat'),d.get('lon'),d.get('home_lat'),d.get('home_lon'),self.config.EARTH_RADIUS_KM)