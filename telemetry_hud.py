# telemetry_hud.py
import cv2
import datetime
import time
from threading import Thread, Lock
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer
import io # For sending image bytes

logger = logging.getLogger(__name__)

# Global variable to hold the latest frame for the HTTP server
# This is a common pattern for sharing data between the capture thread and HTTP handler threads
# A Lock is used to ensure thread-safe access to this shared frame.
latest_frame_for_http = None
frame_lock = Lock()

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global latest_frame_for_http, frame_lock
        
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--FRAME')
            self.end_headers()
            
            try:
                while True: # Keep streaming frames
                    with frame_lock:
                        frame_to_send = latest_frame_for_http
                    
                    if frame_to_send is None:
                        time.sleep(0.05) # Wait if no frame is ready yet
                        continue

                    # Encode the frame as JPEG
                    # Quality can be adjusted via self.server.config.MJPEG_QUALITY
                    ret, jpeg = cv2.imencode('.jpg', frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), self.server.config.MJPEG_QUALITY])
                    if not ret:
                        logger.warning("MJPEG: Could not encode frame to JPEG.")
                        time.sleep(0.1)
                        continue
                    
                    frame_bytes = jpeg.tobytes()
                    
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    
                    # Control frame rate - sleep for the inverse of the desired FPS
                    # This helps prevent overwhelming the client or the Pi's CPU
                    # The actual capture rate is handled by the TelemetryHUDStreamer's main loop
                    time.sleep(1.0 / self.server.config.VIDEO_FPS if self.server.config.VIDEO_FPS > 0 else 0.1)

            except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
                logger.info("MJPEG stream: Client disconnected.")
            except Exception as e:
                logger.error(f"MJPEG stream error: {e}", exc_info=True)
            finally:
                logger.info("MJPEG stream: Closing client connection.")
        else:
            self.send_error(404)
            self.end_headers()

class ThreadedHTTPServer(HTTPServer):
    """Handle requests in a separate thread."""
    def __init__(self, server_address, RequestHandlerClass, config_obj):
        super().__init__(server_address, RequestHandlerClass)
        self.config = config_obj # Make config accessible to handler

class TelemetryHUDStreamer:
    def __init__(self, config, drone_data_provider_func):
        self.config = config
        self.drone_data_provider = drone_data_provider_func
        self.video_capture = None
        self._running = False
        self._capture_thread = None # Thread for capturing and processing frames
        self._http_server_thread = None # Thread for HTTP server
        self.http_server = None

    def _capture_and_overlay_loop(self):
        global latest_frame_for_http, frame_lock
        logger.info("Video frame capture and HUD overlay thread started.")
        
        self.video_capture = cv2.VideoCapture(0) # TODO: Make camera index configurable
        if not self.video_capture.isOpened():
            logger.error("Cannot open video device for HUD streamer.")
            self._running = False # Signal other threads to stop if camera fails
            return
            
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.VIDEO_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.VIDEO_HEIGHT)
        self.video_capture.set(cv2.CAP_PROP_FPS, self.config.VIDEO_FPS) # Camera capture FPS
        logger.info(f"Video capture configured: {self.config.VIDEO_WIDTH}x{self.config.VIDEO_HEIGHT} @ {self.config.VIDEO_FPS}FPS")

        frame_time = 1.0 / self.config.VIDEO_FPS if self.config.VIDEO_FPS > 0 else 0.05 # Target time per frame

        while self._running:
            loop_start_time = time.time()
            ret, frame = self.video_capture.read()
            if not ret:
                logger.warning("Failed to grab frame from camera for HUD.")
                time.sleep(0.1) # Wait a bit before retrying
                continue
            
            telemetry_data = self.drone_data_provider() 
            self._overlay_hud_on_frame(frame, telemetry_data)
            
            with frame_lock:
                latest_frame_for_http = frame.copy() # Update the shared frame

            # Maintain capture frame rate
            elapsed_time = time.time() - loop_start_time
            sleep_time = frame_time - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if self.video_capture and self.video_capture.isOpened():
            self.video_capture.release()
        logger.info("Video frame capture and HUD overlay thread stopped.")

    def _start_http_server(self):
        try:
            self.http_server = ThreadedHTTPServer(
                ('0.0.0.0', self.config.MJPEG_HTTP_PORT), 
                MJPEGStreamHandler,
                self.config # Pass config to server, so handler can access it
            )
            logger.info(f"MJPEG HTTP server started on port {self.config.MJPEG_HTTP_PORT}. Stream at /stream.mjpg")
            self.http_server.serve_forever() # This will block until http_server.shutdown() is called
        except OSError as e: # Handle "address already in use"
            logger.error(f"Could not start MJPEG HTTP server on port {self.config.MJPEG_HTTP_PORT}: {e}")
            self._running = False # Signal other threads to stop if server fails
        except Exception as e:
            if self._running: # Only log as critical if we weren't intentionally stopping
                logger.critical(f"MJPEG HTTP server failed: {e}", exc_info=True)
        finally:
            logger.info("MJPEG HTTP server thread stopped.")


    def _overlay_hud_on_frame(self, frame, telemetry):
        font,scale,thick,color_list,spacing = cv2.FONT_HERSHEY_SIMPLEX,0.45,1,self.config.HUD_TEXT_COLOR,18
        color_tuple = tuple(color_list) if isinstance(color_list, list) else (0,255,0) # Ensure it's a tuple

        def put_text(txt,x,y_line): cv2.putText(frame,txt,(x,y_line*spacing+15),font,scale,color_tuple,thick)
        
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
        state_s=f"STATE:{telemetry.get('current_state','N/A')}"[:25]
        (w_state,_),_ = cv2.getTextSize(state_s,font,scale,thick); put_text(state_s,fw-w_state-10,2)
        imu_stale_s = f"IMU_STALE:{telemetry.get('imu_stale',True)}"
        (w_imu_s,_),_ = cv2.getTextSize(imu_stale_s,font,scale,thick); put_text(imu_stale_s,fw-w_imu_s-10,3)
        
        status_s="ARMED" if telemetry.get('system_armed',False) else "DISARMED"
        status_c_list = [0,255,0] if telemetry.get('system_armed',False) else [0,0,255]
        status_c_tuple = tuple(status_c_list)
        (w_stat,h_stat),_ = cv2.getTextSize(status_s,font,scale+0.1,thick)
        cv2.putText(frame,status_s,(fw-w_stat-10,frame.shape[0]-h_stat-5),font,scale+0.1,status_c_tuple,thick)

    def start(self):
        if self.config.MJPEG_HTTP_PORT <= 0:
            logger.info("MJPEG Video streaming disabled (port set to 0 or less).")
            return
            
        self._running = True
        # Start the frame capture and HUD overlay thread
        self._capture_thread = Thread(target=self._capture_and_overlay_loop, daemon=True, name="HUDCaptureThread")
        self._capture_thread.start()

        # Start the HTTP server in its own thread
        self._http_server_thread = Thread(target=self._start_http_server, daemon=True, name="MJPEGServerThread")
        self._http_server_thread.start()
        logger.info("TelemetryHUDStreamer (MJPEG) started.")

    def stop(self):
        logger.info("Attempting to stop TelemetryHUDStreamer (MJPEG)...")
        self._running = False # Signal threads to stop

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        
        if self.http_server:
            logger.info("Shutting down MJPEG HTTP server...")
            self.http_server.shutdown() # Shuts down serve_forever()
            self.http_server.server_close() # Closes the server socket
            self.http_server = None # Clear reference
        
        if self._http_server_thread and self._http_server_thread.is_alive():
            self._http_server_thread.join(timeout=1.0)
        
        if self.video_capture and self.video_capture.isOpened():
            self.video_capture.release()
        
        logger.info("TelemetryHUDStreamer (MJPEG) stop process complete.")

