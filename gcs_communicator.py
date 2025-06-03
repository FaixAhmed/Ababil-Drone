# gcs_communicator.py
import socket
import json
import time
from threading import Thread, Lock
import logging

logger = logging.getLogger(__name__)

class GCSCommunicator:
    def __init__(self, config, command_callback_func):
        self.config = config; self.command_callback = command_callback_func
        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gcs_telemetry_address = (self.config.GCS_TELEMETRY_TARGET_IP, self.config.GCS_TELEMETRY_TARGET_PORT)
        self._running = False; self._telemetry_thread_trigger = None; self._command_thread = None # Renamed telemetry thread
        self.received_commands_lock = Lock(); self.command_queue = [] # Not used if callback is direct
        self._last_telemetry_send_time = 0 # For controlling send interval

    def start(self):
        self._running = True # Set running true, threads check this
        if not self.config.GCS_TELEMETRY_TARGET_IP or self.config.GCS_TELEMETRY_TARGET_PORT <= 0:
            logger.warning("GCS Telemetry sending disabled (target IP/Port not configured).")
        else:
            # Telemetry is now sent by an explicit call from Drone's main loop,
            # so a dedicated sending thread here isn't strictly necessary for the send action itself,
            # but a placeholder for future periodic tasks could be kept if needed.
            logger.info(f"GCS Telemetry configured to send to {self.gcs_telemetry_address}")

        if self.config.GCS_COMMAND_PORT <= 0:
            logger.warning("GCS Command listening disabled (port not configured).")
        else:
            try:
                self.command_socket.setblocking(False) # Use non-blocking for command socket
                self.command_socket.bind((self.config.GCS_DRONE_IP, self.config.GCS_COMMAND_PORT))
                logger.info(f"GCS cmd listener bound to {self.config.GCS_DRONE_IP}:{self.config.GCS_COMMAND_PORT}")
                self._command_thread = Thread(target=self._receive_commands_loop, daemon=True, name="GCSCommandThread")
                self._command_thread.start()
            except socket.error as e: logger.error(f"Failed to bind GCS cmd socket: {e}")

    def stop(self):
        logger.info("Stopping GCS communicator...")
        self._running = False
        # Unblock command_socket.recvfrom if it was blocking (not strictly needed with setblocking(False) + timeout)
        if self.command_socket and self.config.GCS_COMMAND_PORT > 0:
            try: # Send a dummy packet to own listening port if it helps unblock a blocking recvfrom (less relevant now)
                 # This part is tricky with non-blocking, a simple flag check in loop is better.
                pass
            except Exception as e: logger.debug(f"Dummy packet error for cmd listener: {e}")
        
        if self._command_thread and self._command_thread.is_alive(): self._command_thread.join(timeout=1.0)
        
        self.telemetry_socket.close()
        if hasattr(self.command_socket, '_closed') and not self.command_socket._closed : # Check if socket isn't already closed
             self.command_socket.close()
        logger.info("GCS communicator stopped.")

    def update_and_send_telemetry(self, telemetry_data): # Called by Drone's main loop
        if not self._running or not self.config.GCS_TELEMETRY_TARGET_IP or self.config.GCS_TELEMETRY_TARGET_PORT <= 0: return

        current_time = time.time()
        if current_time - self._last_telemetry_send_time < self.config.GCS_TELEMETRY_SEND_INTERVAL_SEC:
            return # Send at configured interval
        self._last_telemetry_send_time = current_time
        
        try:
            data_with_ts = telemetry_data.copy(); data_with_ts['timestamp_gcs_sent_epoch'] = current_time
            message = json.dumps(data_with_ts, separators=(',', ':')).encode('utf-8') # Compact JSON
            self.telemetry_socket.sendto(message, self.gcs_telemetry_address)
            logger.debug(f"Sent telemetry to {self.gcs_telemetry_address}, size: {len(message)}")
        except socket.error as e: logger.warning(f"Socket error sending GCS telemetry: {e}")
        except Exception as e: logger.error(f"Error sending GCS telemetry: {e}", exc_info=False)

    def _receive_commands_loop(self):
        logger.info("GCS command receiving thread started.")
        while self._running:
            try:
                data, addr = self.command_socket.recvfrom(1024) # Non-blocking, will raise BlockingIOError
                message = data.decode('utf-8'); logger.info(f"GCS cmd from {addr}: {message}")
                cmd_data = json.loads(message)
                if self.command_callback: self.command_callback(cmd_data)
                # else: # Fallback queue if needed, but direct callback is cleaner
                #     with self.received_commands_lock: self.command_queue.append(cmd_data)
            except BlockingIOError: # Expected with non-blocking socket
                time.sleep(0.01) # Short sleep when no data
                continue
            except socket.error as e:
                if self._running: logger.error(f"Socket error GCS cmd recv: {e}"); # Don't break, try to recover or rely on re-bind
                time.sleep(0.5) # Wait a bit before retrying recv
            except json.JSONDecodeError: logger.warning(f"Invalid JSON GCS cmd from {addr if 'addr' in locals() else 'unknown'}: {message[:100] if 'message' in locals() else 'N/A'}")
            except Exception as e:
                if self._running: logger.error(f"Error processing GCS cmd: {e}", exc_info=True)
            # time.sleep(0.005) # Brief sleep to prevent busy-looping if continuously failing
        logger.info("GCS command receiving thread stopped.")

    def get_command_from_queue(self): # If not using direct callback
        with self.received_commands_lock:
            if self.command_queue: return self.command_queue.pop(0)
        return None