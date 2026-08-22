# Ababil — Raspberry Pi Drone Flight Controller

## Credits

* Faiz Patel

## Warning and Disclaimer of Liability

* **Safety First:** Operating drones, especially custom-built ones, carries significant risks of injury and property damage.
* **Thorough Testing:** Always test in a safe, open area, away from people, animals, and property. Start all tests with **propellers removed** to verify system responses.
* **Local Regulations:** You are solely responsible for understanding and complying with all local and national drone regulations (e.g., CAA rules in the UK) before flying.
* **No Guarantees:** This software is provided "as-is" without any warranties, express or implied. The creators and contributors are not liable for any damages or injuries arising from the use or misuse of this software or the drone built using it.
* **Unauthorized Use:** This software is intended for lawful and responsible use. Any use of this software for illegal activities, unauthorized surveillance, or any purpose that infringes on the rights or safety of others is strictly prohibited. The user assumes all liability for such unauthorized use.

## Intellectual Property and Contact

* **Copyright (c) 2025 Faiz Patel**
* All rights reserved.
* For inquiries, please contact: [patelfaiz333@gmail.com]

This software is provided for educational, research, and personal hobbyist purposes. If you intend to use this for commercial purposes, ensure you fully comply with all applicable commercial drone operation regulations, insurance requirements, and safety standards.

This project implements a comprehensive flight controller software for a Raspberry Pi-based drone, featuring autonomous capabilities, sensor integration, failsafes, and GCS communication.


## Features

* **Modular Design:** Code is structured into classes for different components (motors, sensors, GPS, controller, etc.).
* **State Machine:** Manages drone operational states (INIT, DISARMED_IDLE, ARMED_MANUAL, RTH, LANDING, CALIBRATION, FAILSAFE, etc.) for robust control flow.
* **PID Control:** For pitch, roll, and yaw stabilization.
* **Sensor Fusion:** Integrates MPU6050 IMU (accelerometer, gyroscope) and (simulated/placeholder) magnetometer data using a complementary filter.
* **GPS Integration:** For position hold (future), Return-To-Home (RTH), and telemetry.
* **Obstacle Avoidance:** Basic collision detection and avoidance using ultrasonic sensors.
* **PS5 Controller Input:** Uses a PS5 DualSense controller for manual flight control and system commands.
* **Video Streaming:** Streams video with an overlaid telemetry HUD via **MJPEG over HTTP**, compatible with VLC and web browsers.
* **GCS Communication (UDP):**
    * Sends detailed telemetry to a Ground Control Station.
    * Receives basic commands from a GCS (e.g., ARM, DISARM, START_RTH, START_CALIBRATION).
* **Failsafe Mechanisms:**
    * RC (Remote Control) signal loss detection and configurable action (RTH, Land).
    * GPS signal loss detection and configurable action during GPS-dependent modes.
    * Critical battery level detection triggers emergency landing.
* **Sensor Calibration Routines (Stubs):** Framework for IMU (gyro, accel, mag) calibration triggered via GCS.
* **External JSON Configuration:** Flight parameters, pin configurations, and settings are loaded from `drone_config.json`.
* **Comprehensive Logging:** Detailed logging of events, errors, and debug information to console and file.

## Hardware Requirements

* **Raspberry Pi:** Raspberry Pi 4 Model B (2GB, 4GB, or 8GB recommended) or Raspberry Pi 5.
* **MicroSD Card:** 32GB+ Class 10/A1/U3 high-speed card.
* **IMU:** MPU6050 Gyroscope/Accelerometer module.
* **GPS Module:** U-blox NEO-M8N or NEO-7N (or similar) with antenna.
* **Ultrasonic Sensors:** 4x HC-SR04 (or similar) for obstacle avoidance.
* **Camera:** Raspberry Pi Camera Module (v2 or v3) or a compatible USB webcam.
* **Quadcopter Frame:** Size appropriate for your motors/props (e.g., 210mm - 450mm).
* **Brushless Motors:** 4x (e.g., 2205/2207/2306 size, kV rating matched to battery and props).
* **ESCs (Electronic Speed Controllers):** 4x (e.g., 20-35A, BLHeli_S or BLHeli_32 recommended, PWM input).
* **Propellers:** Multiple sets (CW and CCW) matched to motors and frame.
* **LiPo Battery:** 3S or 4S, 1500mAh-5000mAh (depending on drone size), with XT60 connector.
* **Power Distribution Board (PDB):** With 5V BEC output for Raspberry Pi.
* **LiPo Battery Charger:** Dedicated balance charger.
* **PS5 DualSense Controller:** For manual control.
* **Miscellaneous:** Wires, XT60/JST connectors, heat shrink, nylon standoffs, soldering equipment, Wi-Fi access for GCS.

## Wiring Diagram

**Note:** This is a textual description. It is highly recommended to create a visual diagram using tools like Fritzing, draw.io, or KiCad before assembly. **Always double-check pinouts for your specific components.**

**Power System:**
* **LiPo Battery (+, -)** ->  **PDB (Battery IN +, -)**
* **PDB (ESC Output +, - for each motor)** ->  **Each ESC (Power IN +, -)**
* **PDB (5V Regulated Output +, GND)** ->  **Raspberry Pi (GPIO Pin 2 or 4 for 5V, Pin 6 or 9 for GND)**. *Ensure clean and stable 5V.*

**Raspberry Pi Connections (BCM Pin Numbering):**

1.  **ESCs (PWM Signal to Raspberry Pi GPIOs):**
    * ESC 1 Signal  ->  RPi GPIO 17
    * ESC 2 Signal  ->  RPi GPIO 18
    * ESC 3 Signal  ->  RPi GPIO 27
    * ESC 4 Signal  ->  RPi GPIO 22
    * *All ESC signal grounds should be connected to a common Raspberry Pi GND pin.*

2.  **MPU6050 (IMU - I2C):**
    * MPU6050 VCC  ->  RPi 3.3V (Pin 1 or 17)
    * MPU6050 GND  ->  RPi GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39)
    * MPU6050 SDA  ->  RPi GPIO 2 (SDA)
    * MPU6050 SCL  ->  RPi GPIO 3 (SCL)

3.  **GPS Module (UART/Serial - e.g., U-blox NEO-M8N):**
    * GPS VCC  ->  RPi 3.3V or 5V (Check your GPS module's specification!)
    * GPS GND  ->  RPi GND
    * GPS TXD  ->  RPi GPIO 15 (RXD0)
    * GPS RXD  ->  RPi GPIO 14 (TXD0)

4.  **Ultrasonic Sensors (HC-SR04 x4 - GPIO):**
    * Each HC-SR04 VCC  ->  RPi 5V (Pin 2 or 4)
    * Each HC-SR04 GND  ->  RPi GND
    * **Front Sensor:** Trig  ->  RPi GPIO 5, Echo  ->  RPi GPIO 6
    * **Left Sensor:** Trig  ->  RPi GPIO 13, Echo  ->  RPi GPIO 19
    * **Right Sensor:** Trig  ->  RPi GPIO 20, Echo  ->  RPi GPIO 21
    * **Back Sensor:** Trig  ->  RPi GPIO 23, Echo  ->  RPi GPIO 24
    * *Note on HC-SR04 Echo Pin: Consider using a voltage divider or logic level shifter for long-term reliability if connecting the 5V Echo output to the 3.3V RPi GPIO input.*

5.  **Camera:**
    * **Raspberry Pi Camera Module:** Connect via the CSI ribbon cable to the Pi's camera port.
    * **USB Webcam:** Connect to any available USB port on the Raspberry Pi.

6.  **Motors to ESCs:**
    * Connect the 3 wires from each brushless motor to the 3 output wires/pads on its corresponding ESC. If a motor spins in the wrong direction, swap any two of these three wires.

**Important Wiring Considerations:**
* **Common Ground:** Ensure all components share a common ground.
* **Voltage Levels:** Double-check voltage requirements for all peripherals.
* **Soldering:** Use good soldering practices.
* **Wire Management:** Keep wiring neat and secure.
* **Vibration Dampening:** Mount the MPU6050 IMU on vibration-dampening material.

## Software Setup and Installation (on Raspberry Pi)

1.  **Prepare Raspberry Pi OS:**
    * Install Raspberry Pi OS (Lite recommended).
    * Update: `sudo apt update && sudo apt full-upgrade -y`

2.  **Enable Interfaces (via `sudo raspi-config`):**
    * `SSH`
    * `I2C`
    * `Serial Port` (Login shell: No, Serial hardware: Yes)
    * `Camera`

3.  **Install Python and Pip:**
    * `sudo apt install python3-pip -y`

4.  **Install Python Dependencies:**
    ```bash
    sudo pip3 install RPi.GPIO pygame opencv-python pyserial mpu6050-raspberrypi
    ```
    * *(Adjust `mpu6050-raspberrypi` if you use a different MPU6050 library).*
    * *(For OpenCV, additional system libraries might be needed if installation fails).*

5.  **Clone the Repository (or Copy Files):**
    * `git clone https://github.com/FaixAhmed/Ababil-Drone.git` and `cd Ababil-Drone` OR copy files to `/home/pi/Ababil`.

## Configuration

1.  **`drone_config.json`:**
    * Located in the project directory. **Review and edit this file before the first flight.**
    * Key parameters: Pin configurations, PID gains, calibration offsets (updated by routines), failsafe settings, GCS IP/Port, **`MJPEG_HTTP_PORT`**, **`MJPEG_QUALITY`**.
    * A default file is created if not found. Ensure `MJPEG_HTTP_PORT` is set (e.g., 8080).

2.  **PS5 Controller Pairing:** Pair via Bluetooth or connect via USB.

## Running the Drone Software

1.  Navigate to project directory: `cd /path/to/your/Ababil`
2.  Run (requires sudo): `sudo python3 drone_main.py`
3.  **To run on boot (recommended):** Use `systemd`. Create `/etc/systemd/system/drone.service`:
    ```ini
    [Unit]
    Description=Drone Flight Controller Service
    After=network.target multi-user.target

    [Service]
    ExecStart=/usr/bin/python3 /home/pi/Ababil/drone_main.py
    WorkingDirectory=/home/pi/Ababil/
    StandardOutput=journal
    StandardError=journal
    Restart=on-failure
    User=root # Or 'pi' if user has necessary hardware access permissions
    # Environment="PYTHONUNBUFFERED=1"

    [Install]
    WantedBy=multi-user.target
    ```
    * Enable: `sudo systemctl daemon-reload && sudo systemctl enable drone.service && sudo systemctl start drone.service`
    * Check: `sudo systemctl status drone.service` and `journalctl -u drone.service -f`

## Viewing the Video Stream

The drone streams video with an overlaid HUD using **MJPEG over HTTP**. This stream can be viewed using VLC media player or most modern web browsers.

**Steps to View:**

1.  **Ensure the drone software is running** on the Raspberry Pi and the `TelemetryHUDStreamer` has started its MJPEG HTTP server.
2.  **Find your Raspberry Pi's IP Address:** On the Pi, use the command `hostname -I`.
3.  **Open VLC Media Player or a Web Browser** on a computer or mobile device on the same network as the Raspberry Pi.
4.  **Enter the Stream URL:**
    * The URL will be: `http://<RASPBERRY_PI_IP_ADDRESS>:<MJPEG_HTTP_PORT>/stream.mjpg`
    * Replace `<RASPBERRY_PI_IP_ADDRESS>` with the actual IP address of your Raspberry Pi.
    * Replace `<MJPEG_HTTP_PORT>` with the port number configured in your `drone_config.json` (default is `8080`).
    * **Example:** `http://192.168.1.101:8080/stream.mjpg`
5.  **In VLC:**
    * Go to "Media" -> "Open Network Stream..."
    * Paste the URL into the network URL field and click "Play".
6.  **In a Web Browser (e.g., Chrome, Firefox):**
    * Simply type the URL into the address bar and press Enter. The browser should display the video stream.

The video stream will show the live camera feed with telemetry data overlaid.

## GCS Communication

* Drone listens for commands on UDP port `GCS_COMMAND_PORT` (default: 14550).
* Sends telemetry via UDP to `GCS_TELEMETRY_TARGET_IP` and `GCS_TELEMETRY_TARGET_PORT` (default: 14551).
* A separate GCS application is needed.
* **Example GCS Commands (JSON via UDP):** `{"command": "ARM"}`, `{"command": "START_CALIBRATION", "type": "GYRO"}`, etc.

## Calibration

* Trigger calibration routines (IMU Gyro, Accel, Mag) via GCS commands.
* **Gyro:** Requires drone to be perfectly still.
* **Accelerometer/Magnetometer:** (Stubs) Follow on-screen/GCS prompts for positioning/movement.
* Successful calibration updates values that can be saved to `drone_config.json` via a `SAVE_CONFIG` GCS command.
* **Sensor calibration is CRITICAL for stable and safe flight.**
