# ps5_controller.py
import pygame
import time
import logging

logger = logging.getLogger(__name__)

class PS5ControllerManager:
    def __init__(self, config):
        self.config = config; self.controller = None
        self.button_pressed_time = {config.PS5_BUTTON_X: None, config.PS5_BUTTON_O: None}
        self.axis_values = {'throttle':0.0,'pitch':0.0,'roll':0.0,'yaw':0.0}
        self.last_event_time = self.last_joystick_event_time = time.time()
        self.is_pygame_initialized = False; self.signal_lost_reported = False
        self._init_pygame_and_joystick()

    def _init_pygame_and_joystick(self):
        try:
            if not self.is_pygame_initialized: pygame.init(); pygame.joystick.init(); self.is_pygame_initialized=True
            if pygame.joystick.get_count() > 0:
                if self.controller is None: # Initialize only if not already holding a controller object
                    self.controller=pygame.joystick.Joystick(0); self.controller.init()
                    logger.info(f"Controller '{self.controller.get_name()}' initialized.")
                    self.last_joystick_event_time=time.time(); self.signal_lost_reported=False
                return True
            else: # No joysticks found
                if self.controller: logger.info("Controller disconnected (joystick count is 0)."); self.controller=None
                return False
        except pygame.error as e: logger.error(f"Pygame joystick init error: {e}"); self.controller=None; return False
        except Exception as e: logger.error(f"Unexpected joystick init error: {e}"); self.controller=None; return False

    def process_events(self):
        arm_sig, shut_sig, quit_py_sig = False,False,False; curr_axes = self.axis_values.copy()
        
        # Try to re-initialize if controller is None (e.g., was disconnected)
        if not self.controller and (time.time()-self.last_event_time > self.config.CONTROLLER_REINIT_INTERVAL):
            logger.debug("Attempting reinit joystick (controller object is None)...")
            self._init_pygame_and_joystick(); self.last_event_time=time.time()

        for event in pygame.event.get():
            self.last_event_time=time.time() # Record any pygame event
            if event.type == pygame.QUIT: quit_py_sig=True
            
            if self.controller: # Only process joystick events if controller object exists
                self.last_joystick_event_time=time.time(); # Record joystick-specific event
                if self.signal_lost_reported: logger.info("Controller signal re-acquired."); self.signal_lost_reported=False

                try:
                    if event.type==pygame.JOYAXISMOTION:
                        # Standard PS5 mapping on Linux often:
                        # Axis 0: Left Stick X (-1 left, 1 right) -> Yaw
                        # Axis 1: Left Stick Y (-1 up, 1 down) -> Throttle (inverted)
                        # Axis 2: Right Stick X (-1 left, 1 right) -> Roll
                        # Axis 3: L2 (-1 unpressed, 1 fully pressed)
                        # Axis 4: R2 (-1 unpressed, 1 fully pressed)
                        # Axis 5: Right Stick Y (-1 up, 1 down) -> Pitch
                        # This might vary, adjust self.config for AXIS_MAPPINGS if needed
                        if event.axis==0: curr_axes['yaw']=self.controller.get_axis(0) 
                        elif event.axis==1: curr_axes['throttle']=-self.controller.get_axis(1) 
                        elif event.axis==2: curr_axes['roll']=self.controller.get_axis(2)  
                        # Assuming Right Stick Y is axis 5 for pitch on your system, adjust if different.
                        # Common alternatives are axis 3 or 4 if L2/R2 are not those.
                        elif event.axis==5: curr_axes['pitch']=self.controller.get_axis(5) 
                        # Log unmapped axes if necessary for debugging:
                        # else: logger.debug(f"Unmapped axis: {event.axis}, value: {self.controller.get_axis(event.axis)}")
                    elif event.type==pygame.JOYBUTTONDOWN:
                        if event.button==self.config.PS5_BUTTON_X: self.button_pressed_time[self.config.PS5_BUTTON_X]=time.time(); logger.info("Controller X pressed.")
                        elif event.button==self.config.PS5_BUTTON_O: self.button_pressed_time[self.config.PS5_BUTTON_O]=time.time(); logger.info("Controller O pressed.")
                    elif event.type==pygame.JOYBUTTONUP:
                        btn_x_time = self.button_pressed_time[self.config.PS5_BUTTON_X]
                        btn_o_time = self.button_pressed_time[self.config.PS5_BUTTON_O]
                        if event.button==self.config.PS5_BUTTON_X:
                            if btn_x_time: logger.info(f"X released after {time.time()-btn_x_time:.2f}s.")
                            self.button_pressed_time[self.config.PS5_BUTTON_X]=None
                        elif event.button==self.config.PS5_BUTTON_O:
                            if btn_o_time: logger.info(f"O released after {time.time()-btn_o_time:.2f}s.")
                            self.button_pressed_time[self.config.PS5_BUTTON_O]=None
                    elif event.type==pygame.JOYDEVICEADDED:
                        logger.info(f"Joystick device added: {event.instance_id}. Re-initializing controller detection."); self.controller=None; self._init_pygame_and_joystick()
                    elif event.type==pygame.JOYDEVICEREMOVED:
                        logger.warning(f"Joystick device removed: {event.instance_id}. Controller lost."); self.controller=None; curr_axes={k:0.0 for k in self.axis_values}
                except pygame.error as e: # Catch if controller is suddenly yanked or other pygame issue
                    logger.error(f"Pygame controller error during event processing: {e}. Marking as disconnected.")
                    self.controller=None; curr_axes={k:0.0 for k in self.axis_values} # Reset axes
            elif event.type in [pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED]: # Handle add/remove even if self.controller is None (e.g. initial plug-in)
                 logger.info(f"Joystick event (no controller obj): {'Added' if event.type==pygame.JOYDEVICEADDED else 'Removed'}. Re-init."); self.controller=None; self._init_pygame_and_joystick()
        
        self.axis_values=curr_axes # Update stored axis values

        # Check Button Hold Times for Arm/Shutdown signals
        if self.button_pressed_time[self.config.PS5_BUTTON_X] and (time.time()-self.button_pressed_time[self.config.PS5_BUTTON_X] >= self.config.BOOT_HOLD_TIME_SEC): arm_sig=True; self.button_pressed_time[self.config.PS5_BUTTON_X]=None
        if self.button_pressed_time[self.config.PS5_BUTTON_O] and (time.time()-self.button_pressed_time[self.config.PS5_BUTTON_O] >= self.config.SHUTDOWN_HOLD_TIME_SEC): shut_sig=True; self.button_pressed_time[self.config.PS5_BUTTON_O]=None
        
        return self.axis_values, arm_sig, shut_sig, quit_py_sig

    def is_signal_lost(self, timeout_sec):
        if not self.controller: # No controller object means signal is effectively lost
            if not self.signal_lost_reported: logger.warning("Controller signal lost: No controller instance detected."); self.signal_lost_reported=True
            return True 
        
        # If controller object exists, check time since last joystick-specific event
        if time.time() - self.last_joystick_event_time > timeout_sec:
            if not self.signal_lost_reported: logger.warning(f"Controller signal lost: No joystick event for >{timeout_sec}s."); self.signal_lost_reported=True
            return True
        
        # If signal was reported lost but events are now coming through
        # This is handled by self.signal_lost_reported = False inside the event loop when controller is active
        return False

    def quit(self):
        if self.is_pygame_initialized: pygame.joystick.quit(); pygame.quit(); self.is_pygame_initialized=False; logger.info("Pygame quit.")