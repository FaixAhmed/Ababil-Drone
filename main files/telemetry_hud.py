# telemetry_hud.py
import cv2
import datetime
import time
from threading import Thread, Lock
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer
import io 
import numpy as np # For graphical elements
import math

logger = logging.getLogger(__name__)

latest_frame_for_http = None
frame_lock = Lock()

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    # This class remains unchanged from the version that introduced MJPEG streaming.
    # It simply serves the 'latest_frame_for_http'.
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
                    
                    time.sleep(1.0 / (self.server.config.VIDEO_FPS + 5) if self.server.config.VIDEO_FPS > 0 else 0.1)
            except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
                logger.info("MJPEG stream: Client disconnected.")
            except Exception as e:
                logger.error(f"MJPEG stream error: {e}", exc_info=False)
            finally:
                logger.info("MJPEG stream: Closing client connection.")
        else:
            self.send_error(404)
            self.end_headers()

class ThreadedHTTPServer(HTTPServer):
    # This class remains unchanged.
    def __init__(self, server_address, RequestHandlerClass, config_obj):
        super().__init__(server_address, RequestHandlerClass); self.config = config_obj

class TelemetryHUDStreamer:
    def __init__(self, config, drone_data_provider_func):
        self.config = config; self.drone_data_provider = drone_data_provider_func
        self.video_capture = None; self._running = False
        self._capture_thread = None; self._http_server_thread = None; self.http_server = None

    def _capture_and_overlay_loop(self):
        # This loop remains largely the same, its job is to get frames and call _overlay_hud_on_frame.
        global latest_frame_for_http, frame_lock
        logger.info("Video capture & HUD overlay thread started.")
        # Try different camera indices if 0 doesn't work, or make it configurable
        camera_indices_to_try = [0, 1, -1, 2] 
        for index in camera_indices_to_try:
            self.video_capture = cv2.VideoCapture(index)
            if self.video_capture.isOpened():
                logger.info(f"Successfully opened camera with index {index}.")
                break
            else:
                logger.warning(f"Failed to open camera with index {index}.")
        
        if not self.video_capture or not self.video_capture.isOpened():
            logger.error("Cannot open any video device for HUD streamer. Exiting capture thread.")
            self._running = False 
            return
            
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,self.config.VIDEO_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,self.config.VIDEO_HEIGHT)
        self.video_capture.set(cv2.CAP_PROP_FPS,self.config.VIDEO_FPS)
        logger.info(f"Video capture configured: {self.config.VIDEO_WIDTH}x{self.config.VIDEO_HEIGHT} @ {self.config.VIDEO_FPS}FPS")
        target_frame_time = 1.0 / self.config.VIDEO_FPS if self.config.VIDEO_FPS > 0 else 0.05 

        while self._running:
            loop_start_time = time.time()
            ret, frame = self.video_capture.read()
            if not ret: logger.warning("Failed to grab frame for HUD."); time.sleep(0.1); continue
            
            telemetry_data = self.drone_data_provider() 
            self._overlay_hud_on_frame(frame, telemetry_data) # Call the updated overlay method
            
            with frame_lock: latest_frame_for_http = frame.copy()
            
            elapsed_time = time.time() - loop_start_time
            sleep_time = target_frame_time - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if self.video_capture and self.video_capture.isOpened(): self.video_capture.release()
        logger.info("Video capture & HUD overlay thread stopped.")

    def _start_http_server(self):
        # This method remains unchanged.
        try:
            self.http_server = ThreadedHTTPServer(('0.0.0.0',self.config.MJPEG_HTTP_PORT),MJPEGStreamHandler,self.config)
            logger.info(f"MJPEG HTTP server on port {self.config.MJPEG_HTTP_PORT}. Stream: /stream.mjpg")
            self.http_server.serve_forever()
        except OSError as e: logger.error(f"MJPEG HTTP server port {self.config.MJPEG_HTTP_PORT} error: {e}"); self._running = False
        except Exception as e:
            if self._running: logger.critical(f"MJPEG HTTP server failed: {e}", exc_info=True)
        finally: logger.info("MJPEG HTTP server thread stopped.")

    # --- NEW AND UPDATED DRAWING METHODS ---
    def _draw_full_screen_horizon(self, frame, pitch_deg, roll_deg, width, height):
        """Draws a full-screen horizon line that tilts with roll and shifts with pitch."""
        horizon_color = (255, 255, 255, 200) # White, slightly transparent for the line itself
        line_thickness = 2

        # Center of the screen
        center_x, center_y = width // 2, height // 2

        # Convert angles to radians
        roll_rad = math.radians(roll_deg)
        
        # Pitch effect: vertical shift of the horizon line.
        # Scale factor determines how many pixels per degree of pitch.
        pitch_pixel_shift = pitch_deg * (height / 90.0) # e.g., 90 deg pitch moves line by half screen height

        # Calculate endpoints of a very long line centered at (center_x, center_y - pitch_pixel_shift)
        # This line will be rotated.
        line_length_extended = width * 2 # Ensure it covers screen edges when rotated

        x1_orig = center_x - line_length_extended // 2
        y1_orig = center_y - pitch_pixel_shift
        x2_orig = center_x + line_length_extended // 2
        y2_orig = center_y - pitch_pixel_shift

        # Rotate these points around (center_x, center_y - pitch_pixel_shift) by roll_rad
        # Point 1
        x1_rot = center_x + (x1_orig - center_x) * math.cos(-roll_rad) - (y1_orig - (center_y - pitch_pixel_shift)) * math.sin(-roll_rad)
        y1_rot = (center_y - pitch_pixel_shift) + (x1_orig - center_x) * math.sin(-roll_rad) + (y1_orig - (center_y - pitch_pixel_shift)) * math.cos(-roll_rad)
        # Point 2
        x2_rot = center_x + (x2_orig - center_x) * math.cos(-roll_rad) - (y2_orig - (center_y - pitch_pixel_shift)) * math.sin(-roll_rad)
        y2_rot = (center_y - pitch_pixel_shift) + (x2_orig - center_x) * math.sin(-roll_rad) + (y2_orig - (center_y - pitch_pixel_shift)) * math.cos(-roll_rad)

        # Draw the line (clipped by frame boundaries automatically by OpenCV)
        cv2.line(frame, (int(x1_rot), int(y1_rot)), (int(x2_rot), int(y2_rot)), 
                 (horizon_color[0], horizon_color[1], horizon_color[2]), line_thickness, cv2.LINE_AA)

        # Optional: Add pitch ladder lines relative to this horizon line
        # This part can be complex to make it look good and rotate correctly with roll.
        # For simplicity, we'll keep it to just the main horizon line for now.

    def _draw_fixed_aircraft_symbol(self, frame, width, height):
        """Draws the fixed aircraft symbol (chevron + crosshair) in the center."""
        center_x, center_y = width // 2, height // 2
        symbol_color = (255, 255, 0, 230) # Yellow, slightly transparent
        line_thickness = 2
        
        # Chevron "^"
        chevron_size = 10 # pixels
        cv2.line(frame, (center_x - chevron_size, center_y - chevron_size // 2 - 5), (center_x, center_y - chevron_size - 5), symbol_color, line_thickness)
        cv2.line(frame, (center_x, center_y - chevron_size - 5), (center_x + chevron_size, center_y - chevron_size // 2 - 5), symbol_color, line_thickness)

        # Horizontal line (wings)
        wing_length = 35
        cv2.line(frame, (center_x - wing_length, center_y), (center_x + wing_length, center_y), symbol_color, line_thickness)
        # Vertical line (fuselage/tail)
        tail_length_up = 5
        tail_length_down = 15
        cv2.line(frame, (center_x, center_y - tail_length_up), (center_x, center_y + tail_length_down), symbol_color, line_thickness)
        # Center dot
        cv2.circle(frame, (center_x, center_y), 3, symbol_color, -1)


    def _draw_cross_compass_bottom_right(self, frame, yaw_deg, target_yaw_deg, width, height):
        """Draws a cross-style compass in the bottom-right corner."""
        compass_size = 80 # Diameter of the compass
        margin = 15
        center_x = width - (compass_size // 2) - margin
        center_y = height - (compass_size // 2) - margin - 20 # Move up a bit for text below

        rose_color = (255, 255, 255, 180)
        fixed_pointer_color = (0, 255, 0, 255) # Green
        target_bug_color = (255, 0, 255, 255) # Magenta
        text_color = (0,255,0,255)
        line_thickness = 1
        font_scale = 0.35

        # Draw compass background (optional circle)
        cv2.circle(frame, (center_x, center_y), compass_size // 2, (0, 0, 0, 100), -1) # Semi-transparent black bg
        cv2.circle(frame, (center_x, center_y), compass_size // 2, rose_color, 1)


        # Draw rotating cross (N, E, S, W arms)
        yaw_rad = math.radians(-yaw_deg) # Negative for clockwise rotation in OpenCV
        arm_len = compass_size // 2 - 5

        cardinals = {'N': 0, 'E': 90, 'S': 180, 'W': 270}
        for label, angle_deg in cardinals.items():
            angle_rad = math.radians(angle_deg) + yaw_rad # Add current yaw to rotate the label
            
            # Line for arm (from center to edge)
            # For a cross, we draw two lines (N-S and E-W)
            if label == 'N' or label == 'S': # Vertical part of the cross
                x_start = center_x + (arm_len if label == 'S' else -arm_len) * math.sin(angle_rad - yaw_rad) # Correct for fixed cross
                y_start = center_y - (arm_len if label == 'S' else -arm_len) * math.cos(angle_rad - yaw_rad)
                # For N-S line, it's always vertical in rose's frame, then rotated
                # For E-W line, it's always horizontal in rose's frame, then rotated
                # This simplified version just draws labels at rotated positions

            # Label position
            label_dist = arm_len + 10
            lx = int(center_x + label_dist * math.sin(angle_rad))
            ly = int(center_y - label_dist * math.cos(angle_rad))
            (w,h),_ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale-0.05, line_thickness)
            cv2.putText(frame, label, (lx - w//2, ly + h//2), cv2.FONT_HERSHEY_SIMPLEX, font_scale-0.05, rose_color, line_thickness)

        # Draw the actual cross lines (N-S and E-W) that rotate
        # N-S line
        ns_x1 = int(center_x + arm_len * math.sin(yaw_rad))
        ns_y1 = int(center_y - arm_len * math.cos(yaw_rad)) # North point
        ns_x2 = int(center_x - arm_len * math.sin(yaw_rad))
        ns_y2 = int(center_y + arm_len * math.cos(yaw_rad)) # South point
        cv2.line(frame, (ns_x1, ns_y1), (ns_x2, ns_y2), rose_color, line_thickness)
        # E-W line
        ew_x1 = int(center_x + arm_len * math.cos(yaw_rad)) # East point (sin(yaw+90))
        ew_y1 = int(center_y + arm_len * math.sin(yaw_rad)) # (cos(yaw+90))
        ew_x2 = int(center_x - arm_len * math.cos(yaw_rad)) # West point
        ew_y2 = int(center_y - arm_len * math.sin(yaw_rad))
        cv2.line(frame, (ew_x1, ew_y1), (ew_x2, ew_y2), rose_color, line_thickness)


        # Fixed pointer (drone's nose, points upwards relative to compass circle)
        cv2.line(frame, (center_x, center_y - arm_len - 2), (center_x, center_y - arm_len + 5), fixed_pointer_color, 2) # Small vertical line
        
        # Target heading bug
        relative_target_rad = math.radians((target_yaw_deg - yaw_deg + 360) % 360) # Angle of target relative to current North on rose
        bug_dist = arm_len - 3
        bug_x = int(center_x + bug_dist * math.sin(math.radians(target_yaw_deg))) # Bug position based on absolute target_yaw
        bug_y = int(center_y - bug_dist * math.cos(math.radians(target_yaw_deg)))
        # Rotate this bug point with the yaw of the rose itself
        bug_rot_x = center_x + (bug_x - center_x) * math.cos(yaw_rad) - (bug_y - center_y) * math.sin(yaw_rad)
        bug_rot_y = center_y + (bug_x - center_x) * math.sin(yaw_rad) + (bug_y - center_y) * math.cos(yaw_rad)
        cv2.circle(frame, (int(bug_rot_x), int(bug_rot_y)), 3, target_bug_color, -1)


        # Current heading text below compass
        heading_text = f"{int(yaw_deg % 360)}°"
        (w,h),_ = cv2.getTextSize(heading_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, line_thickness)
        cv2.putText(frame, heading_text, (center_x - w//2, center_y + arm_len + 15), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, line_thickness)


    def _draw_home_arrow_bottom_right(self, frame, current_lat, current_lon, home_lat, home_lon, current_yaw_deg, width, height):
        # Position near the compass
        margin = 10
        arrow_size = 15 # Length of arrow lines
        # Place it to the left of the arm/disarm status
        # Assume arm_status text width is around 80px, compass is 120px wide
        arrow_origin_x = width - 80 - arrow_size - margin # Adjust as needed
        arrow_origin_y = height - margin - arrow_size // 2 - 5 # Align with arm status text vertically

        if None in [current_lat, current_lon, home_lat, home_lon]:
            cv2.putText(frame, "H?", (arrow_origin_x - 5, arrow_origin_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255),1)
            return

        delta_lon = math.radians(home_lon - current_lon)
        lat1_rad, lat2_rad = math.radians(current_lat), math.radians(home_lat)
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        bearing_to_home_rad = math.atan2(y, x)
        
        relative_angle_rad = bearing_to_home_rad - math.radians(current_yaw_deg)
        
        arrow_tip_x = int(arrow_origin_x + arrow_size * math.sin(relative_angle_rad))
        arrow_tip_y = int(arrow_origin_y - arrow_size * math.cos(relative_angle_rad))
        cv2.arrowedLine(frame, (arrow_origin_x, arrow_origin_y), (arrow_tip_x, arrow_tip_y), (0, 255, 255), 2, tipLength=0.4)


    def _overlay_hud_on_frame(self, frame, telemetry):
        font,scale,thick,color_list,spacing = cv2.FONT_HERSHEY_SIMPLEX,0.42,1,self.config.HUD_TEXT_COLOR,17 
        color_tuple = tuple(color_list) if isinstance(color_list, list) else (0,255,0)
        fh, fw = frame.shape[:2]

        # --- 1. Draw Full-Screen Artificial Horizon Line ---
        pitch = telemetry.get('pitch', 0.0)
        roll = telemetry.get('roll', 0.0)
        self._draw_full_screen_horizon(frame, pitch, roll, fw, fh)

        # --- 2. Draw Fixed Aircraft Symbol ---
        self._draw_fixed_aircraft_symbol(frame, fw, fh)

        # --- 3. Text Telemetry (positioned around graphics) ---
        def put_text(txt,x,y_line_num, align_right=False, color=color_tuple, custom_scale=scale):
            # Ensure x is int for OpenCV
            x_int = int(x)
            if align_right:
                (w, _), _ = cv2.getTextSize(txt, font, custom_scale, thick)
                x_int = int(fw - w - 10)
            cv2.putText(frame,txt,(x_int, int(y_line_num*spacing+15)),font,custom_scale,color,thick, cv2.LINE_AA)
        
        # Top-Left Block
        y_offset_top = 1 # Start text a bit lower
        ts=datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        put_text(f"T:{ts}",10,y_offset_top+0)
        put_text(f"BAT:{telemetry.get('battery_level',0.0):.1f}%",10,y_offset_top+1)
        g_lat,g_lon,g_fix,g_sats=telemetry.get('gps_lat'),telemetry.get('gps_lon'),telemetry.get('gps_fix_quality',0),telemetry.get('gps_num_satellites',0)
        g_str=f"GPS:{g_lat:.5f},{g_lon:.5f}" if g_lat and g_lon and g_fix>0 else "GPS:NoFix"
        put_text(g_str,10,y_offset_top+2); put_text(f"FIX:{g_fix} SAT:{g_sats} Stale:{telemetry.get('gps_stale',True)}",10,y_offset_top+3)
        dist_home = telemetry.get('dist_to_home_m', float('nan'))
        dist_home_str = f"HomeD:{dist_home:.1f}m" if math.isfinite(dist_home) else "HomeD:N/A"
        put_text(dist_home_str, 10, y_offset_top+4)
        put_text(f"P:{pitch:.1f} R:{roll:.1f}", 10, y_offset_top+5) # Pitch/Roll values near AH

        # Top-Right Block
        put_text(f"ALT:{telemetry.get('gps_alt',0.0):.1f}m", 0, y_offset_top+0, align_right=True)
        put_text(f"SPD:{telemetry.get('gps_speed_kmh',0.0):.1f}km/h", 0, y_offset_top+1, align_right=True)
        put_text(f"STATE:{telemetry.get('current_state','N/A')}"[:18], 0, y_offset_top+2, align_right=True)
        put_text(f"IMU_STALE:{telemetry.get('imu_stale',True)}", 0, y_offset_top+3, align_right=True)
        put_text(f"YAW:{telemetry.get('yaw',0.0):.1f} TGT:{telemetry.get('target_yaw',0.0):.1f}",0, y_offset_top+4, align_right=True)


        # Bottom-Left (Controller Inputs)
        ctrl_thr = telemetry.get('ctrl_throttle', 0.0)
        ctrl_pch = telemetry.get('ctrl_pitch', 0.0)
        ctrl_rll = telemetry.get('ctrl_roll', 0.0)
        ctrl_yaw_stick = telemetry.get('ctrl_yaw_stick', 0.0)
        put_text(f"CTRL T:{ctrl_thr:.2f} P:{ctrl_pch:.2f}", 10, (fh // spacing) - 3)
        put_text(f"     R:{ctrl_rll:.2f} YS:{ctrl_yaw_stick:.2f}", 10, (fh // spacing) - 2)

        # Bottom-Center (Status Message)
        status_msg = telemetry.get('status_message', "")
        if status_msg:
            msg_color_tuple = (0,165,255) # Orange
            if "CRITICAL" in status_msg.upper() or "FAIL" in status_msg.upper(): msg_color_tuple = (0,0,255) # Red
            elif "LOW BATTERY" in status_msg.upper() : msg_color_tuple = (0,255,255) # Yellow
            (w_msg,_),_ = cv2.getTextSize(status_msg,font,scale+0.1,thick+1)
            cv2.putText(frame,status_msg,( (fw-w_msg)//2 , fh-25),font,scale+0.1,msg_color_tuple,thick+1, cv2.LINE_AA)


        # --- 4. Draw Cross Compass and Home Arrow (Bottom Right) ---
        current_yaw = telemetry.get('yaw', 0.0)
        target_yaw = telemetry.get('target_yaw', current_yaw)
        self._draw_cross_compass_bottom_right(frame, current_yaw, target_yaw, fw, fh)
        
        if telemetry.get('gps_fix_quality',0) > 0 and telemetry.get('home_lat') is not None:
             self._draw_home_arrow_bottom_right(frame, telemetry.get('gps_lat'), telemetry.get('gps_lon'),
                                   telemetry.get('home_lat'), telemetry.get('home_lon'),
                                   current_yaw, fw, fh)


        # Bottom-Right Arm Status (Positioned carefully with compass and arrow)
        status_s="ARMED" if telemetry.get('system_armed',False) else "DISARMED"
        status_c_list = [0,255,0] if telemetry.get('system_armed',False) else [0,0,255]
        status_c_tuple = tuple(status_c_list)
        (w_stat,h_stat),_ = cv2.getTextSize(status_s,font,scale+0.2,thick+1)
        # Position it above the compass or adjust compass position
        cv2.putText(frame,status_s,(fw-w_stat-15, fh - (80 + 20 + 15) - h_stat - 5 ),font,scale+0.2,status_c_tuple,thick+1, cv2.LINE_AA) # Adjusted Y


    def start(self): # Unchanged
        if self.config.MJPEG_HTTP_PORT <= 0: logger.info("MJPEG Video streaming disabled."); return
        self._running=True
        self._capture_thread = Thread(target=self._capture_and_overlay_loop, daemon=True, name="HUDCaptureThread")
        self._capture_thread.start()
        self._http_server_thread = Thread(target=self._start_http_server, daemon=True, name="MJPEGServerThread")
        self._http_server_thread.start()
        logger.info("TelemetryHUDStreamer (MJPEG with Graphical HUD) started.")

    def stop(self): # Unchanged
        logger.info("Stopping TelemetryHUDStreamer (MJPEG)...")
        self._running = False
        if self._capture_thread and self._capture_thread.is_alive(): self._capture_thread.join(timeout=1.0)
        if self.http_server:
            logger.info("Shutting down MJPEG HTTP server..."); self.http_server.shutdown(); self.http_server.server_close(); self.http_server = None
        if self._http_server_thread and self._http_server_thread.is_alive(): self._http_server_thread.join(timeout=1.0)
        if self.video_capture and self.video_capture.isOpened(): self.video_capture.release()
        logger.info("TelemetryHUDStreamer (MJPEG) stop complete.")

