# telemetry_hud.py
import cv2
import socket
import struct
import pickle
import datetime
import time
from threading import Thread, Lock
import logging

logger = logging.getLogger(__name__)

class TelemetryHUDStreamer:
    def __init__(self, config, drone_data_provider_func):
        self.config = config; self.drone_data_provider = drone_data_provider_func
        self.server_socket, self.client_connection, self.video_capture = None,None,None
        self._running = False; self._thread = None; self.lock = Lock() # Lock for socket resources

    def _video_stream_thread_func(self):
        logger.info("Video stream thread starting...")
        try:
            with self.lock: # Protect socket creation during start/stop
                self.server_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                self.server_socket.bind(('0.0.0.0',self.config.VIDEO_STREAM_PORT))
                self.server_socket.listen(1)
            logger.info(f"Video server listening on port {self.config.VIDEO_STREAM_PORT}...")
            
            self.video_capture=cv2.VideoCapture(0) # TODO: Make camera index configurable
            if not self.video_capture.isOpened(): logger.error("Cannot open video device."); self._cleanup_resources(); return
            self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.config.VIDEO_WIDTH)
            self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.config.VIDEO_HEIGHT)
            self.video_capture.set(cv2.CAP_PROP_FPS,self.config.VIDEO_FPS)
            logger.info(f"Video capture configured: {self.config.VIDEO_WIDTH}x{self.config.VIDEO_HEIGHT} @ {self.config.VIDEO_FPS}FPS")
            
            while self._running:
                logger.info("Waiting for video client connection...")
                client_conn_obj = None # Use a local var for the accepted connection
                try:
                    with self.lock:
                        if not self._running: break
                        self.server_socket.settimeout(1.0) # Allow periodic check of self._running
                        try: client_conn_obj, addr = self.server_socket.accept()
                        except socket.timeout: continue # Go back to check self._running
                        self.server_socket.settimeout(None) # Back to blocking for this client
                    
                    self.client_connection = client_conn_obj.makefile('wb') # Assign to instance variable
                    logger.info(f"Video client {addr} connected.")
                    
                    while self._running: # Streaming loop for the connected client
                        ret,frame = self.video_capture.read()
                        if not ret: logger.warning("Failed to grab frame from camera."); time.sleep(0.1); continue
                        
                        telemetry_data = self.drone_data_provider() 
                        self._overlay_hud_on_frame(frame, telemetry_data)
                        
                        try:
                            # Using pickle for simplicity, but consider MJPEG or other methods for efficiency
                            frame_data = pickle.dumps(frame, protocol=pickle.HIGHEST_PROTOCOL)
                            self.client_connection.write(struct.pack('<L',len(frame_data)) + frame_data)
                            self.client_connection.flush()
                        except (BrokenPipeError,ConnectionResetError,EOFError, AttributeError): # AttributeError if client_connection is None
                            logger.info("Video client disconnected or connection error.")
                            break # Break inner loop, go back to accept new connection
                        except Exception as e_send: logger.error(f"Error sending video frame: {e_send}"); break
                
                except socket.error as e_sock: # Errors on server_socket itself (e.g. if stop is called)
                    if self._running: logger.error(f"Video server socket error: {e_sock}. Retrying if running...")
                    time.sleep(1) # Wait before retrying to bind/listen if still running
                finally: # Ensure client connection is closed if loop breaks
                    if self.client_connection:
                        try:self.client_connection.close()
                        except:pass
                    if client_conn_obj: # Also close the raw socket object if it exists
                        try: client_conn_obj.close()
                        except: pass
                    self.client_connection = None # Clear instance variable
                
                if not self._running: break # Exit outer loop if signalled to stop
        
        except Exception as e_thread_critical: logger.critical(f"Video stream thread encountered a critical error: {e_thread_critical}", exc_info=True)
        finally: self._cleanup_resources(); logger.info("Video stream thread has fully terminated.")

    def _overlay_hud_on_frame(self, frame, telemetry): # Condensed version
        font,scale,thick,color,spacing = cv2.FONT_HERSHEY_SIMPLEX,0.45,1,tuple(self.config.HUD_TEXT_COLOR),18
        def put_text(txt,x,y_line): cv2.putText(frame,txt,(x,y_line*spacing+15),font,scale,color,thick)
        ts=datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        put_text(f"T:{ts}",10,0); put_text(f"BAT:{telemetry.get('battery_level',0.0):.1f}%",10,1)
        put_text(f"P:{telemetry.get('pitch',0.0):.1f} R:{telemetry.get('roll',0.0):.1f} Y:{telemetry.get('yaw',0.0):.1f}",10,2)
        g_lat,g_lon,g_fix,g_sats=telemetry.get('gps_lat'),telemetry.get('gps_lon'),telemetry.get('gps_fix_quality',0),telemetry.get('gps_num_satellites',0)
        g_str=f"GPS:{g_lat:.5f},{g_lon:.5f}" if g_lat and g_lon and g_fix>0 else "GPS:NoFix"
        put_text(g_str,10,3); put_text(f"FIX:{g_fix} SAT:{g_sats} Stale:{telemetry.get('gps_stale',True)}",10,4)
        fw=frame.shape[1];
        alt_s,spd_s=f"ALT:{telemetry.get('gps_alt',0.0):.1f}m",f"SPD:{telemetry.get('gps_speed_kmh',0.0):.1f}km/h"
        (w_alt,_),_ = cv2.getTextSize(alt_s,font,scale,thick); put_text(alt_s,fw-w_alt-10,0)
        (w_spd,_),_ = cv2.getTextSize(spd_s,font,scale,thick); put_text(spd_s,fw-w_spd-10,1)
        state_s=f"STATE:{telemetry.get('current_state','N/A')}"[:25] # Truncate if too long
        (w_state,_),_ = cv2.getTextSize(state_s,font,scale,thick); put_text(state_s,fw-w_state-10,2)
        imu_stale_s = f"IMU_STALE:{telemetry.get('imu_stale',True)}"
        (w_imu_s,_),_ = cv2.getTextSize(imu_stale_s,font,scale,thick); put_text(imu_stale_s,fw-w_imu_s-10,3)
        status_s="ARMED" if telemetry.get('system_armed',False) else "DISARMED"
        status_c=(0,255,0) if telemetry.get('system_armed',False) else (0,0,255)
        (w_stat,h_stat),_ = cv2.getTextSize(status_s,font,scale+0.1,thick)
        cv2.putText(frame,status_s,(fw-w_stat-10,frame.shape[0]-h_stat-5),font,scale+0.1,status_c,thick) # Adjusted y for bottom

    def _cleanup_resources(self):
        logger.info("Cleaning up video stream resources...")
        if self.video_capture and self.video_capture.isOpened(): self.video_capture.release(); self.video_capture=None
        if self.client_connection: try:self.client_connection.close() except:pass; self.client_connection=None
        with self.lock: # Protect server_socket during cleanup
            if self.server_socket: try:self.server_socket.close() except:pass; self.server_socket=None

    def start(self):
        if self.config.VIDEO_STREAM_PORT<=0: logger.info("Video streaming disabled (port set to 0 or less)."); return
        self._running=True; self._thread=Thread(target=self._video_stream_thread_func,daemon=True,name="VideoStreamThread"); self._thread.start()
    def stop(self):
        logger.info("Attempting to stop video stream thread...")
        self._running=False # Signal thread to stop
        with self.lock: # Ensure server_socket is closed to unblock accept()
            if self.server_socket:
                try: # Create a dummy connection to unblock accept()
                    dummy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM); dummy_socket.settimeout(0.1)
                    dummy_socket.connect(('127.0.0.1', self.config.VIDEO_STREAM_PORT)); dummy_socket.close()
                except: pass # Ignore errors, server might be already down or unblocking
        if self._thread and self._thread.is_alive(): self._thread.join(timeout=2.0) # Give thread time to close
        self._cleanup_resources(); logger.info("Video stream stop process complete.")